from smbus2 import SMBus

import time

ADDRESS = 0x29

bus = SMBus(1)

def read_signed_16(reg):

    lsb = bus.read_byte_data(ADDRESS, reg)

    msb = bus.read_byte_data(ADDRESS, reg + 1)

    value = (msb << 8) | lsb

    if value > 32767:

        value -= 65536

    return value

while True:

    yaw   = read_signed_16(0x1A) / 16.0

    roll  = read_signed_16(0x1C) / 16.0

    pitch = read_signed_16(0x1E) / 16.0

    accel_x = read_signed_16(0x08) / 100.0

    accel_y = read_signed_16(0x0A) / 100.0

    accel_z = read_signed_16(0x0C) / 100.0

    gyro_x = read_signed_16(0x14) / 16.0

    gyro_y = read_signed_16(0x16) / 16.0

    gyro_z = read_signed_16(0x18) / 16.0

    print("=" * 50)

    print(f"Yaw   : {yaw:.2f}")

    print(f"Roll  : {roll:.2f}")

    print(f"Pitch : {pitch:.2f}")

    print()

    print(f"Accel X : {accel_x:.2f}")

    print(f"Accel Y : {accel_y:.2f}")

    print(f"Accel Z : {accel_z:.2f}")

    print()

    print(f"Gyro X : {gyro_x:.2f}")

    print(f"Gyro Y : {gyro_y:.2f}")

    print(f"Gyro Z : {gyro_z:.2f}")

    time.sleep(1)