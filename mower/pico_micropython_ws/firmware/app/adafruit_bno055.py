from machine import I2C

# Constants from Adafruit library
BNO055_ADDRESS_A            = 0x28
BNO055_ID                   = 0xA0

BNO055_PAGE_ID_ADDR         = 0x07
BNO055_CHIP_ID_ADDR         = 0x00
BNO055_ACCEL_DATA_X_LSB     = 0x08
BNO055_MAG_DATA_X_LSB       = 0x0E
BNO055_GYRO_DATA_X_LSB      = 0x14
BNO055_EULER_H_LSB          = 0x1A
BNO055_QUATERNION_DATA_W_LSB= 0x20
BNO055_LINEAR_ACCEL_X_LSB   = 0x28
BNO055_GRAVITY_X_LSB        = 0x2E
BNO055_TEMP_ADDR            = 0x34
BNO055_CALIB_STAT_ADDR      = 0x35
BNO055_SYS_STAT_ADDR        = 0x39
BNO055_SYS_ERR_ADDR         = 0x3A
BNO055_UNIT_SEL_ADDR        = 0x3B
BNO055_OPR_MODE_ADDR        = 0x3D
BNO055_PWR_MODE_ADDR        = 0x3E
BNO055_SYS_TRIGGER_ADDR     = 0x3F

# Power modes
POWER_MODE_NORMAL           = 0x00

# Operation modes
OPERATION_MODE_CONFIG       = 0x00
OPERATION_MODE_NDOF         = 0x0C


class BNO055_I2C:
    def __init__(self, i2c: I2C, address: int = BNO055_ADDRESS_A):
        self._i2c = i2c
        self._addr = address
        self._ok = False
        self._ok = self._begin()

    # ---- Low level helpers ----
    def _wr8(self, reg: int, val: int):
        self._i2c.writeto_mem(self._addr, reg, bytes((val & 0xFF,)))

    def _rdn(self, reg: int, n: int) -> bytes:
        return self._i2c.readfrom_mem(self._addr, reg, n)

    def _set_mode(self, mode: int):
        self._wr8(BNO055_OPR_MODE_ADDR, mode & 0xFF)
        import utime
        utime.sleep_ms(30)

    # ---- Init sequence (Adafruit style) ----
    def _begin(self) -> bool:
        import utime
        try:
            # Ensure PAGE 0, enter CONFIG
            try:
                self._wr8(BNO055_PAGE_ID_ADDR, 0)
            except Exception:
                pass
            self._set_mode(OPERATION_MODE_CONFIG)
            self._wr8(BNO055_PAGE_ID_ADDR, 0)
            # Check chip ID
            cid = self._rdn(BNO055_CHIP_ID_ADDR, 1)
            if not cid or cid[0] != BNO055_ID:
                return False
            # Reset via SYS_TRIGGER
            self._wr8(BNO055_SYS_TRIGGER_ADDR, 0x20)
            utime.sleep_ms(650)
            # Normal power
            self._wr8(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL)
            # Internal oscillator
            self._wr8(BNO055_SYS_TRIGGER_ADDR, 0x00)
            # Units: m/s^2, dps, degrees, Celsius (0x00)
            self._wr8(BNO055_UNIT_SEL_ADDR, 0x00)
            # Enter NDOF
            self._set_mode(OPERATION_MODE_NDOF)
            # Small warm-up wait
            utime.sleep_ms(150)
            return True
        except Exception:
            return False

    # ---- Public helpers compatible with existing main.py diagnostics ----
    def ok(self) -> bool:
        return bool(self._ok)

    def chip_id(self) -> int:
        try:
            b = self._rdn(BNO055_CHIP_ID_ADDR, 1)
            return b[0] if b and len(b) == 1 else -1
        except Exception:
            return -1

    def sys_status(self) -> int:
        try:
            b = self._rdn(BNO055_SYS_STAT_ADDR, 1)
            return b[0] if b and len(b) == 1 else -1
        except Exception:
            return -1

    def sys_error(self) -> int:
        try:
            b = self._rdn(BNO055_SYS_ERR_ADDR, 1)
            return b[0] if b and len(b) == 1 else -1
        except Exception:
            return -1

    # ---- Adafruit-like properties ----
    @property
    def calibration_status(self):
        try:
            b = self._rdn(BNO055_CALIB_STAT_ADDR, 1)
            if not b or len(b) != 1:
                return None
            val = b[0]
            sys = (val >> 6) & 0x03
            gyro = (val >> 4) & 0x03
            accel = (val >> 2) & 0x03
            mag = val & 0x03
            return (sys, gyro, accel, mag)
        except Exception:
            return None

    @property
    def temperature(self):
        try:
            b = self._rdn(BNO055_TEMP_ADDR, 1)
            if not b or len(b) != 1:
                return None
            t = b[0]
            if t > 127:
                t -= 256
            return float(t)
        except Exception:
            return None

    @property
    def euler(self):
        try:
            data = self._rdn(BNO055_EULER_H_LSB, 6)
            if not data or len(data) != 6:
                return None
            # signed 16-bit values
            def s16(lo, hi):
                v = ((hi << 8) | lo) & 0xFFFF
                return v - 65536 if v > 32767 else v
            h = s16(data[0], data[1]) / 16.0
            r = s16(data[2], data[3]) / 16.0
            p = s16(data[4], data[5]) / 16.0
            return (h, r, p)
        except Exception:
            return None

    def read_euler(self):
        return self.euler

    @property
    def quaternion(self):
        try:
            data = self._rdn(BNO055_QUATERNION_DATA_W_LSB, 8)
            if not data or len(data) != 8:
                return None
            def s16(lo, hi):
                v = ((hi << 8) | lo) & 0xFFFF
                return v - 65536 if v > 32767 else v
            w = s16(data[0], data[1])
            x = s16(data[2], data[3])
            y = s16(data[4], data[5])
            z = s16(data[6], data[7])
            scale = 1.0 / (1 << 14)
            return (x * scale, y * scale, z * scale, w * scale)
        except Exception:
            return None

    @property
    def acceleration(self):
        try:
            data = self._rdn(BNO055_ACCEL_DATA_X_LSB, 6)
            if not data or len(data) != 6:
                return None
            def s16(lo, hi):
                v = ((hi << 8) | lo) & 0xFFFF
                return v - 65536 if v > 32767 else v
            x = s16(data[0], data[1]) / 100.0
            y = s16(data[2], data[3]) / 100.0
            z = s16(data[4], data[5]) / 100.0
            return (x, y, z)
        except Exception:
            return None

    @property
    def gyro(self):
        try:
            data = self._rdn(BNO055_GYRO_DATA_X_LSB, 6)
            if not data or len(data) != 6:
                return None
            def s16(lo, hi):
                v = ((hi << 8) | lo) & 0xFFFF
                return v - 65536 if v > 32767 else v
            # According to Adafruit lib: values are in degrees/second, scale 900 LSB per deg/s
            x = s16(data[0], data[1]) / 900.0
            y = s16(data[2], data[3]) / 900.0
            z = s16(data[4], data[5]) / 900.0
            return (x, y, z)
        except Exception:
            return None
