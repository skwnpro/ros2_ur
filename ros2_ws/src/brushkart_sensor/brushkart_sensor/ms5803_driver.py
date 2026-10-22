from smbus2 import SMBus

import time


class MS5803Driver:
    """Driver MS5803 (I2C 0x76) — tekanan & suhu, plus estimasi kedalaman.

    PERHATIAN VARIAN: keluarga MS5803 punya banyak varian range
    (01BA = 0..1 bar, 05BA, 14BA, 30BA). Varian 01BA jenuh di sekitar
    tekanan atmosfer sehingga tidak bisa mengukur kedalaman di bawah
    air. Kedalaman dihitung dari selisih terhadap
    `surface_pressure_mbar`, yang bisa dikalibrasi dari sensor_node.
    """

    ADDRESS = 0x76

    def __init__(self):

        self.bus = SMBus(1)

        # Tekanan referensi permukaan (mbar). Nilai bawaan adalah
        # tekanan atmosfer standar; sensor_node menimpanya dengan
        # hasil kalibrasi di permukaan air yang sebenarnya.
        self.surface_pressure_mbar = 1013.25

        self.bus.write_byte(
            self.ADDRESS,
            0x1E
        )

        time.sleep(0.1)

        self.read_prom()

    def read_prom(self):

        coeff = []

        for i in range(1, 7):

            reg = 0xA0 + (i * 2)

            data = self.bus.read_i2c_block_data(
                self.ADDRESS,
                reg,
                2
            )

            value = (
                (data[0] << 8)
                |
                data[1]
            )

            coeff.append(value)

        self.C1 = coeff[0]

        self.C2 = coeff[1]

        self.C3 = coeff[2]

        self.C4 = coeff[3]

        self.C5 = coeff[4]

        self.C6 = coeff[5]

    def read_d1(self):

        self.bus.write_byte(
            self.ADDRESS,
            0x48
        )

        time.sleep(0.05)

        data = self.bus.read_i2c_block_data(
            self.ADDRESS,
            0x00,
            3
        )

        return (
            (data[0] << 16)
            |
            (data[1] << 8)
            |
            data[2]
        )

    def read_d2(self):

        self.bus.write_byte(
            self.ADDRESS,
            0x58
        )

        time.sleep(0.05)

        data = self.bus.read_i2c_block_data(
            self.ADDRESS,
            0x00,
            3
        )

        return (
            (data[0] << 16)
            |
            (data[1] << 8)
            |
            data[2]
        )

    def get_data(self):

        D1 = self.read_d1()

        D2 = self.read_d2()

        dT = D2 - (self.C5 * 256)

        TEMP = (
            2000 +
            dT * self.C6 / 8388608
        )

        OFF = (
            self.C2 * 65536 +
            (self.C4 * dT) / 128
        )

        SENS = (
            self.C1 * 32768 +
            (self.C3 * dT) / 256
        )

        pressure = (
            (
                D1 * SENS / 2097152
            ) - OFF
        ) / 32768

        temperature_c = TEMP / 100.0

        pressure_mbar = pressure / 100.0

        # Kedalaman: selisih tekanan terhadap permukaan.
        # ~100 mbar per meter (air tawar); air laut ~101 mbar/m,
        # jadi ada galat sekitar 1-3% yang diterima untuk V1.
        depth_m = (
            pressure_mbar - self.surface_pressure_mbar
        ) / 100.0

        return {
            "pressure_mbar": pressure_mbar,
            "temperature_c": temperature_c,
            "depth_m": depth_m,
            "pressure_ok": True
        }
