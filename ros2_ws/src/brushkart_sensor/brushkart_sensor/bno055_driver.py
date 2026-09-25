from smbus2 import SMBus

import time

class BNO055Driver:

    ADDRESS = 0x29

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