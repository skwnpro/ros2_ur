#!/usr/bin/env python3

from smbus2 import SMBus

import time

ADDR = 0x76

bus = SMBus(1)

#========================

# Reset Sensor

#========================

bus.write_byte(ADDR, 0x1E)

time.sleep(0.1)

#========================

# Read PROM Coefficients

#========================

C = [0] * 7

for i in range(1, 7):

    reg = 0xA0 + (i * 2)

    data = bus.read_i2c_block_data(

        ADDR,

        reg,

        2

    )

    C[i] = (data[0] << 8) | data[1]

C1 = C[1]

C2 = C[2]

C3 = C[3]

C4 = C[4]

C5 = C[5]

C6 = C[6]

print("Calibration Data")

print(f"C1 = {C1}")

print(f"C2 = {C2}")

print(f"C3 = {C3}")

print(f"C4 = {C4}")

print(f"C5 = {C5}")

print(f"C6 = {C6}")

#========================

# Read D1

#========================

def read_pressure_raw():

    # OSR 4096

    bus.write_byte(ADDR, 0x48)

    time.sleep(0.05)

    data = bus.read_i2c_block_data(

        ADDR,

        0x00,

        3

    )

    D1 = (

        (data[0] << 16)

        |

        (data[1] << 8)

        |

        data[2]

    )

    return D1

#========================

# Read D2

#========================

def read_temperature_raw():

    # OSR 4096

    bus.write_byte(ADDR, 0x58)

    time.sleep(0.05)

    data = bus.read_i2c_block_data(

        ADDR,

        0x00,

        3

    )

    D2 = (

        (data[0] << 16)

        |

        (data[1] << 8)

        |

        data[2]

    )

    return D2

#========================

# Main Loop

#========================

while True:

    D1 = read_pressure_raw()

    D2 = read_temperature_raw()

    dT = D2 - (C5 * 256)

    TEMP = 2000 + (dT * C6) / 8388608

    OFF = (

        C2 * 65536

        +

        (C4 * dT) / 128

    )

    SENS = (

        C1 * 32768

        +

        (C3 * dT) / 256

    )

    P = (

        (

            D1 * SENS / 2097152

        )

        - OFF

    ) / 32768

    temperature_c = TEMP / 100.0

    pressure_mbar = P / 100.0

    depth_m = (

        pressure_mbar - 1013.25

    ) / 100.0

    print("=" * 50)

    print(f"D1 Raw        : {D1}")

    print(f"D2 Raw        : {D2}")

    print(f"Temperature   : {temperature_c:.2f} C")

    print(f"Pressure      : {pressure_mbar:.2f} mbar")

    print(f"Depth         : {depth_m:.2f} m")

    time.sleep(1)