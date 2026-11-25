#!/usr/bin/env python3
"""ultrasonic_node — sensor jarak JSN-SR04T depan & belakang.

Dua sensor ultrasonik waterproof JSN-SR04T-V3.3 (satu menghadap
depan, satu menghadap belakang) untuk mendeteksi ujung kapal:
selama masih ada lambung di depan, jarak terbaca kecil dan stabil;
begitu robot sampai di ujung kapal, pantulan hilang (tidak ada echo)
dan jarak dibaca sebagai -1.0.

PENTING — kecepatan suara di air:
  Modul ini dihitung untuk dipakai TERENDAM. Kecepatan suara di air
  (~1480 m/s) JAUH berbeda dari di udara (~343 m/s). Driver yang
  memakai konstanta udara akan melaporkan jarak ~4,3x lebih pendek
  dari sebenarnya. Nilai bawaan parameter `sound_speed_m_s` adalah
  1480.0 (air tawar); air laut sekitar 1500-1530 m/s — kalibrasi di
  lapangan bila presisi penting (atau terima galat beberapa persen).

PENTING — arti nilai range:
  range = -1.0  -> TIDAK ADA PANTULAN (ujung kapal / di luar jangkau).
                   Ini kondisi VALID, bukan sensor rusak. Konsumen
                   (state machine semi-auto) harus memperlakukannya
                   sebagai "ujung ketemu", bukan fault.
  range >= 0    -> jarak terukur dalam meter.

PENTING — pemasangan:
  Board PCB biru TIDAK waterproof — hanya transducer-nya yang
  waterproof. Board tetap di dalam housing kering; hanya transducer
  yang keluar lewat kabel putihnya.
  V3.3 = logika 3,3 V, jadi pin Echo boleh langsung ke GPIO Raspi
  tanpa voltage divider (berbeda dengan versi 5 V).

PEMBACAAN: mode Trig/Echo GPIO (bukan UART). Kedua sensor dibaca
berurutan (bukan bersamaan) agar tidak saling mengganggu (crosstalk).
"""

import time

import RPi.GPIO as GPIO

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Range


# Lebar pulsa Trig yang diminta datasheet (mikrodetik).
TRIG_PULSE_US = 10
# Batas tunggu Echo naik/turun (detik). 6 meter bolak-balik di air
# butuh ~8 ms; 50 ms memberi ruang longgar sebelum dianggap no-echo.
ECHO_TIMEOUT_S = 0.05
# Jeda antar pembacaan dua sensor (detik), anti-crosstalk.
JEDA_ANTAR_SENSOR_S = 0.02
# Sudut beam JSN-SR04T (perkiraan, radian). Total beam ~15 derajat.
FIELD_OF_VIEW_RAD = 0.26


class UltrasonicNode(Node):

    def __init__(self):
        super().__init__('ultrasonic_node')

        # --- Parameter (pin BCM; hindari pin relay, I2C, dan UART) ---
        self.declare_parameter('front_trig_pin', 17)
        self.declare_parameter('front_echo_pin', 27)
        self.declare_parameter('back_trig_pin', 20)
        self.declare_parameter('back_echo_pin', 21)
        self.declare_parameter('sound_speed_m_s', 1480.0)
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('min_range_m', 0.2)
        self.declare_parameter('max_range_m', 6.0)

        self.front_trig = self.get_parameter('front_trig_pin').value
        self.front_echo = self.get_parameter('front_echo_pin').value
        self.back_trig = self.get_parameter('back_trig_pin').value
        self.back_echo = self.get_parameter('back_echo_pin').value
        self.sound_speed = float(
            self.get_parameter('sound_speed_m_s').value)
        self.min_range = float(self.get_parameter('min_range_m').value)
        self.max_range = float(self.get_parameter('max_range_m').value)
        rate = float(self.get_parameter('publish_rate_hz').value)

        # --- GPIO ---
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for pin in (self.front_trig, self.back_trig):
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        for pin in (self.front_echo, self.back_echo):
            GPIO.setup(pin, GPIO.IN)

        # --- Publisher ---
        self.front_pub = self.create_publisher(
            Range, '/range_front', 10)
        self.back_pub = self.create_publisher(
            Range, '/range_back', 10)

        self.timer = self.create_timer(1.0 / rate, self.publish_ranges)

        self.get_logger().info(
            'Ultrasonic Node Started '
            f'(depan Trig={self.front_trig}/Echo={self.front_echo}, '
            f'belakang Trig={self.back_trig}/Echo={self.back_echo}, '
            f'kecepatan suara {self.sound_speed:.0f} m/s)')

    # ------------------------------------------------------------------
    # Pengukuran satu sensor
    # ------------------------------------------------------------------

    def _measure(self, trig_pin, echo_pin):
        """Ukur jarak satu sensor; kembalikan meter, atau None bila
        tidak ada pantulan (no-echo)."""
        # Pulsa Trig 10 mikrodetik.
        GPIO.output(trig_pin, GPIO.HIGH)
        time.sleep(TRIG_PULSE_US / 1_000_000.0)
        GPIO.output(trig_pin, GPIO.LOW)

        # Tunggu Echo naik (batas waktu = no-echo).
        deadline = time.monotonic() + ECHO_TIMEOUT_S
        while GPIO.input(echo_pin) == GPIO.LOW:
            if time.monotonic() > deadline:
                return None
        t_start = time.monotonic_ns()

        # Tunggu Echo turun (batas waktu = pantulan rusak).
        deadline = time.monotonic() + ECHO_TIMEOUT_S
        while GPIO.input(echo_pin) == GPIO.HIGH:
            if time.monotonic() > deadline:
                return None
        t_end = time.monotonic_ns()

        # Jarak = waktu tempuh bolak-balik x kecepatan suara / 2.
        jarak = (t_end - t_start) / 1e9 * self.sound_speed / 2.0
        return max(self.min_range, min(self.max_range, jarak))

    # ------------------------------------------------------------------
    # Publikasi
    # ------------------------------------------------------------------

    def _publish_range(self, pub, frame_id, jarak):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = FIELD_OF_VIEW_RAD
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        # -1.0 = tidak ada pantulan (lihat docstring di atas).
        msg.range = -1.0 if jarak is None else float(jarak)
        pub.publish(msg)

    def publish_ranges(self):
        depan = self._measure(self.front_trig, self.front_echo)
        time.sleep(JEDA_ANTAR_SENSOR_S)
        belakang = self._measure(self.back_trig, self.back_echo)

        self._publish_range(
            self.front_pub, 'ultrasonic_front', depan)
        self._publish_range(
            self.back_pub, 'ultrasonic_back', belakang)

    # ------------------------------------------------------------------

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
