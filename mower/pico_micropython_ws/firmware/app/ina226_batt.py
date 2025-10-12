# INA226 battery monitor helper for MicroPython (RP2040)
from machine import I2C

class INA226:
    def __init__(self, i2c: I2C, addr: int = 0x40, shunt_ohms: float = 0.002, max_current_a: float = 20.0):
        self.i2c = i2c
        self.addr = addr
        self.shunt_ohms = float(shunt_ohms)
        self.max_current_a = float(max_current_a)
        self.current_lsb = None
        self._init_ok = self._init()

    def ok(self) -> bool:
        return bool(self._init_ok and (self.current_lsb is not None))

    def _wr16(self, reg: int, val: int):
        b = bytes(((val >> 8) & 0xFF, val & 0xFF))
        self.i2c.writeto_mem(self.addr, reg, b)

    def _rd16(self, reg: int) -> int:
        b = self.i2c.readfrom_mem(self.addr, reg, 2)
        return ((b[0] << 8) | b[1]) & 0xFFFF

    def _init(self) -> bool:
        try:
            # Compute calibration
            i_lsb = self.max_current_a / 32768.0
            if i_lsb <= 0:
                i_lsb = 1e-6
            cal = int(0.00512 / (i_lsb * self.shunt_ohms))
            if cal < 1:
                cal = 1
            if cal > 0xFFFF:
                cal = 0xFFFF
            self._wr16(0x05, cal)  # CALIBRATION
            self.current_lsb = i_lsb
            return True
        except Exception:
            return False

    def read(self):
        """Returns (voltage_V, current_A) or None on error."""
        if not self.ok():
            return None
        try:
            raw_v = self._rd16(0x02)  # BUS VOLTAGE
            voltage_V = float(raw_v) * 1.25e-3
            raw_i = self._rd16(0x04)  # CURRENT (signed)
            if raw_i & 0x8000:
                raw_i = -(((~raw_i) & 0xFFFF) + 1)
            current_A = float(raw_i) * float(self.current_lsb)
            return voltage_V, current_A
        except Exception:
            return None
