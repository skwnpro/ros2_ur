from smbus2 import SMBus

import time


class BNO055Driver:
    """Driver BNO055 (I2C 0x29) dalam mode NDOF (fusi 9-sumbu).

    Catatan: yaw pada mode NDOF berasal dari fusi magnetometer —
    di dekat lambung baja / motor, bacaan yaw absolut bisa menyimpang.
    Untuk manuver belok pendek, integrasi gyro relatif lebih bisa
    dipercaya daripada yaw absolut ini.
    """

    ADDRESS = 0x29

    # Register status kalibrasi (SYS_CALIB_STAT).
    REG_CALIB_STAT = 0x35

    def __init__(self):

        self.bus = SMBus(1)

        # CONFIG MODE

        self.bus.write_byte_data(
            self.ADDRESS,
            0x3D,
            0x00
        )

        time.sleep(0.1)

        # NDOF MODE

        self.bus.write_byte_data(
            self.ADDRESS,
            0x3D,
            0x0C
        )

        time.sleep(1)

    def read_signed_16(self, reg):

        lsb = self.bus.read_byte_data(
            self.ADDRESS,
            reg
        )

        msb = self.bus.read_byte_data(
            self.ADDRESS,
            reg + 1
        )

        value = (msb << 8) | lsb

        if value > 32767:

            value -= 65536

        return value

    def get_calibration_status(self):
        """Status kalibrasi per sub-sistem, masing-masing 0..3.

        3 = terkalibrasi penuh. Nilai dibaca dari register 0x35:
        bit 7-6 SYS, bit 5-4 GYRO, bit 3-2 ACC, bit 1-0 MAG.
        Kalibrasi hilang setiap power cycle — pantau lewat method ini.
        """
        raw = self.bus.read_byte_data(self.ADDRESS, self.REG_CALIB_STAT)
        return {
            "sys": (raw >> 6) & 0x03,
            "gyro": (raw >> 4) & 0x03,
            "acc": (raw >> 2) & 0x03,
            "mag": raw & 0x03,
        }

    def get_data(self):

        yaw = self.read_signed_16(0x1A) / 16.0

        roll = self.read_signed_16(0x1C) / 16.0

        pitch = self.read_signed_16(0x1E) / 16.0

        accel_x = self.read_signed_16(0x08) / 100.0

        accel_y = self.read_signed_16(0x0A) / 100.0

        accel_z = self.read_signed_16(0x0C) / 100.0

        gyro_x = self.read_signed_16(0x14) / 16.0

        gyro_y = self.read_signed_16(0x16) / 16.0

        gyro_z = self.read_signed_16(0x18) / 16.0

        return {
            "roll_deg": roll,
            "pitch_deg": pitch,
            "yaw_deg": yaw,
            "accel_x": accel_x,
            "accel_y": accel_y,
            "accel_z": accel_z,
            "gyro_x": gyro_x,
            "gyro_y": gyro_y,
            "gyro_z": gyro_z,
            "imu_ok": True
        }
