#!/usr/bin/env python3
"""relay_operator_node — penggerak relay/solenoid di underwater robot.

Satu-satunya node yang menyentuh GPIO. Membaca /relay_operator
(brushkart_msgs/RelayOperator) dari command_gateway_node dan
menerjemahkannya menjadi keluaran relay:

  * 6 relay hidrolik (brush, drive, steering, lampu)
  * 4 relay thruster DC brushed (satu arah, digital)

Pengaman yang hidup di node ini (lapis terakhir sebelum hardware):
  1. WATCHDOG — /relay_operator diterbitkan command_gateway_node
     secara periodik (heartbeat). Bila pesan berhenti datang lebih
     lama dari `watchdog_timeout_s`, semua relay dimatikan sendiri.
     Ini yang membuat robot berhenti saat tether putus / GS mati.
  2. INTERLOCK — kombinasi relay yang saling bertabrakan secara
     mekanis (forward+retreat, left+right) tidak akan pernah
     diteruskan ke GPIO; pasangan yang konflik dimatikan.
  3. E-STOP — field estop pada pesan masuk mematikan semua output
     seketika, mengalahkan semua field lain.
  4. Semua relay mati saat node start dan saat node dimatikan.

PETA RELAY -> GPIO (BCM) — satu-satunya tempat peta ini ditulis:
  relay1_brush     -> GPIO 5
  relay2_forward   -> GPIO 6
  relay3_retreat   -> GPIO 13
  relay4_left      -> GPIO 19
  relay5_right     -> GPIO 26
  relay6_light     -> GPIO 16
  relay7_thruster1 -> GPIO 12
  relay8_thruster2 -> GPIO 23
  relay9_thruster3 -> GPIO 24
  relay10_thruster4-> GPIO 25
Semua nomor pin bisa diubah lewat parameter `pin_*` (mis. untuk robot
simulasi) tanpa mengubah kode.
"""

import time

import rclpy
from rclpy.node import Node

import RPi.GPIO as GPIO

from brushkart_msgs.msg import RelayOperator


# Nama field pada RelayOperator.msg, urut sesuai nomor relay.
FIELD_RELAY = (
    'relay1_brush',
    'relay2_forward',
    'relay3_retreat',
    'relay4_left',
    'relay5_right',
    'relay6_light',
    'relay7_thruster1',
    'relay8_thruster2',
    'relay9_thruster3',
    'relay10_thruster4',
)

# Pin BCM bawaan untuk tiap field di atas (urutan yang sama).
PIN_BAWAAN = (5, 6, 13, 19, 26, 16, 12, 23, 24, 25)

# Pasangan field yang tidak boleh menyala bersamaan (nama pendek
# hanya untuk pesan log).
PASANGAN_KONFLIK = (
    ('relay2_forward', 'relay3_retreat', 'forward+retreat'),
    ('relay4_left', 'relay5_right', 'left+right'),
)


class RelayOperatorNode(Node):

    def __init__(self):
        super().__init__('relay_operator_node')

        # --- Parameter ---
        self.declare_parameter('watchdog_timeout_s', 1.0)
        # Ubah ke False bila modul relay-nya active-low.
        self.declare_parameter('active_high', True)
        for field, pin in zip(FIELD_RELAY, PIN_BAWAAN):
            self.declare_parameter(f'pin_{field}', pin)

        self.watchdog_timeout = self.get_parameter(
            'watchdog_timeout_s').value
        self.active_high = self.get_parameter('active_high').value
        # {nama field: nomor pin BCM}
        self.pin_relay = {
            field: self.get_parameter(f'pin_{field}').value
            for field in FIELD_RELAY
        }

        # --- GPIO ---
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for pin in self.pin_relay.values():
            GPIO.setup(pin, GPIO.OUT)
        self.all_off()

        # Waktu pesan terakhir diterima (None = belum pernah).
        self.last_rx_time = None
        self._watchdog_aktif = False

        self.subscription = self.create_subscription(
            RelayOperator, '/relay_operator', self.relay_callback, 10)
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_check)

        self.get_logger().info('Relay Operator Node Started')

    # ------------------------------------------------------------------
    # Penulisan GPIO
    # ------------------------------------------------------------------

    def relay_write(self, field, state):
        pin = self.pin_relay[field]
        if self.active_high:
            GPIO.output(pin, GPIO.HIGH if state else GPIO.LOW)
        else:
            GPIO.output(pin, GPIO.LOW if state else GPIO.HIGH)

    def all_off(self):
        for field in FIELD_RELAY:
            self.relay_write(field, False)

    # ------------------------------------------------------------------
    # Penerimaan perintah relay
    # ------------------------------------------------------------------

    def relay_callback(self, msg):
        self.last_rx_time = time.monotonic()
        self._watchdog_aktif = False

        # E-STOP — prioritas tertinggi, abaikan field lain.
        if msg.estop:
            self.all_off()
            self.get_logger().warning('EMERGENCY STOP — semua relay mati')
            return

        # Salin keadaan yang diminta, lalu terapkan interlock.
        keadaan = {field: getattr(msg, field) for field in FIELD_RELAY}
        for field_a, field_b, label in PASANGAN_KONFLIK:
            if keadaan[field_a] and keadaan[field_b]:
                keadaan[field_a] = False
                keadaan[field_b] = False
                self.get_logger().warning(
                    f'Interlock: perintah {label} bersamaan ditolak, '
                    'keduanya dimatikan')

        for field, state in keadaan.items():
            self.relay_write(field, state)

    # ------------------------------------------------------------------
    # Watchdog heartbeat
    # ------------------------------------------------------------------

    def watchdog_check(self):
        if self.last_rx_time is None or self._watchdog_aktif:
            return
        diam = time.monotonic() - self.last_rx_time
        if diam > self.watchdog_timeout:
            self._watchdog_aktif = True
            self.all_off()
            self.get_logger().error(
                f'Watchdog: tidak ada pesan /relay_operator selama '
                f'{diam:.1f} detik — semua relay dimatikan')

    # ------------------------------------------------------------------

    def destroy_node(self):
        self.all_off()
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RelayOperatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
