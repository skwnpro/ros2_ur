#!/usr/bin/env python3
"""sensor_node — pembaca sensor underwater robot.

Membaca dua sensor I2C pada bus yang sama:
  * BNO055  (IMU: roll/pitch/yaw, accel, gyro)   — brushkart_sensor.bno055_driver
  * MS5803  (tekanan & suhu -> kedalaman)        — brushkart_sensor.ms5803_driver

dan menerbitkan hasilnya sebagai /sensor_data (brushkart_msgs/SensorData)
pada 10 Hz.

Sifat yang dijaga node ini:
  * ISOLASI ERROR — kegagalan satu sensor tidak membunuh node dan tidak
    menghilangkan data sensor lainnya; flag `imu_ok`/`pressure_ok` pada
    pesan menjadi false dan node terus berjalan.
  * Driver yang gagal diinisialisasi saat start dicoba lagi secara
    berkala, jadi sensor yang telat siap tidak mengharuskan restart.
  * Header pesan selalu diisi (stamp + frame_id) agar konsumen bisa
    menilai umur data.

CATATAN KEDALAMAN: depth_m dihitung dari selisih tekanan terukur
terhadap tekanan permukaan. Tekanan permukaan dikalibrasi otomatis
dari rata-rata 20 sampel pertama setelah start (robot diasumsikan
sedang di permukaan saat itu), atau diisi manual lewat parameter
`surface_pressure_mbar`. Sensor yang terpasang saat ini adalah
MS5803-01BA (0..1 bar absolut) yang JENUH di sekitar permukaan air —
pembacaan kedalaman di bawah air tidak valid sampai sensor diganti
varian yang sesuai (mis. MS5837-30BA).
"""

import time

import rclpy
from rclpy.node import Node

from brushkart_msgs.msg import SensorData

from brushkart_sensor.bno055_driver import BNO055Driver
from brushkart_sensor.ms5803_driver import MS5803Driver


JUMLAH_SAMPEL_KALIBRASI = 20
JEDA_COBA_LAGI_S = 5.0  # jeda mencoba ulang inisialisasi driver


class SensorNode(Node):

    def __init__(self):
        super().__init__('sensor_node')

        # 0.0 = kalibrasi otomatis saat start.
        self.declare_parameter('surface_pressure_mbar', 0.0)
        self.surface_param = self.get_parameter(
            'surface_pressure_mbar').value

        self.publisher_ = self.create_publisher(
            SensorData, '/sensor_data', 10)

        self.imu = None
        self.pressure = None
        self._terakhir_coba_imu = 0.0
        self._terakhir_coba_pressure = 0.0
        self._coba_driver()

        # Akumulator kalibrasi tekanan permukaan.
        self._sampel_kalibrasi = []
        self._kalibrasi_selesai = self.surface_param > 0.0
        if self._kalibrasi_selesai and self.pressure is not None:
            self.pressure.surface_pressure_mbar = self.surface_param

        self._hitung_publikasi = 0
        self.timer = self.create_timer(0.1, self.publish_data)

        self.get_logger().info('Sensor Node Started')

    # ------------------------------------------------------------------
    # Inisialisasi driver (dengan percobaan ulang)
    # ------------------------------------------------------------------

    def _coba_driver(self):
        sekarang = time.monotonic()
        if self.imu is None \
                and sekarang - self._terakhir_coba_imu >= JEDA_COBA_LAGI_S:
            self._terakhir_coba_imu = sekarang
            try:
                self.imu = BNO055Driver()
                self.get_logger().info('BNO055 siap')
            except Exception as e:
                self.get_logger().warning(
                    f'BNO055 belum siap: {e}',
                    throttle_duration_sec=10.0)
        if self.pressure is None \
                and sekarang - self._terakhir_coba_pressure >= JEDA_COBA_LAGI_S:
            self._terakhir_coba_pressure = sekarang
            try:
                self.pressure = MS5803Driver()
                if self.surface_param > 0.0:
                    self.pressure.surface_pressure_mbar = self.surface_param
                    self._kalibrasi_selesai = True
                self.get_logger().info('MS5803 siap')
            except Exception as e:
                self.get_logger().warning(
                    f'MS5803 belum siap: {e}',
                    throttle_duration_sec=10.0)

    # ------------------------------------------------------------------
    # Kalibrasi tekanan permukaan
    # ------------------------------------------------------------------

    def _kalibrasi_dari_sampel(self, pressure_mbar):
        if self._kalibrasi_selesai or self.pressure is None:
            return
        self._sampel_kalibrasi.append(pressure_mbar)
        if len(self._sampel_kalibrasi) >= JUMLAH_SAMPEL_KALIBRASI:
            rata = sum(self._sampel_kalibrasi) / len(self._sampel_kalibrasi)
            self.pressure.surface_pressure_mbar = rata
            self._kalibrasi_selesai = True
            self.get_logger().info(
                f'Tekanan permukaan terkalibrasi: {rata:.1f} mbar')

    # ------------------------------------------------------------------
    # Siklus publikasi
    # ------------------------------------------------------------------

    def publish_data(self):
        if self.imu is None or self.pressure is None:
            self._coba_driver()

        msg = SensorData()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'sensor_link'

        # --- IMU ---
        if self.imu is not None:
            try:
                imu = self.imu.get_data()
                msg.roll_deg = imu['roll_deg']
                msg.pitch_deg = imu['pitch_deg']
                msg.yaw_deg = imu['yaw_deg']
                msg.accel_x = imu['accel_x']
                msg.accel_y = imu['accel_y']
                msg.accel_z = imu['accel_z']
                msg.gyro_x = imu['gyro_x']
                msg.gyro_y = imu['gyro_y']
                msg.gyro_z = imu['gyro_z']
                msg.imu_ok = True
            except Exception as e:
                msg.imu_ok = False
                self.get_logger().warning(
                    f'Gagal membaca BNO055: {e}',
                    throttle_duration_sec=5.0)

        # --- Tekanan / kedalaman ---
        if self.pressure is not None:
            try:
                tek = self.pressure.get_data()
                msg.pressure_mbar = tek['pressure_mbar']
                msg.temperature_c = tek['temperature_c']
                msg.depth_m = tek['depth_m']
                msg.pressure_ok = True
                self._kalibrasi_dari_sampel(tek['pressure_mbar'])
            except Exception as e:
                msg.pressure_ok = False
                self.get_logger().warning(
                    f'Gagal membaca MS5803: {e}',
                    throttle_duration_sec=5.0)

        self.publisher_.publish(msg)

        # Status kalibrasi IMU dicatat tiap ~10 detik untuk pemantauan.
        self._hitung_publikasi += 1
        if self.imu is not None and self._hitung_publikasi % 100 == 0:
            try:
                kal = self.imu.get_calibration_status()
                self.get_logger().info(
                    f"Kalibrasi BNO055 — sys={kal['sys']} "
                    f"gyro={kal['gyro']} acc={kal['acc']} mag={kal['mag']}")
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
