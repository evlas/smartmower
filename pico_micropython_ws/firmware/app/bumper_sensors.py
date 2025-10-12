# Bumper sensors via GPIO list or PCF8574 mapped bits
from machine import Pin, I2C
import time

try:
    import pins_config as cfg
except Exception:
    cfg = object()

try:
    from pcf8574 import PCF8574  # type: ignore
except Exception:
    PCF8574 = None


class BumperSensors:
    """Read bumper left/right as booleans.

    If PCF8574 is available, reads bits mapped by cfg:
      - PCF_BIT_BUMPER_LEFT
      - PCF_BIT_BUMPER_RIGHT
    Else, reads provided GPIO pins list.
    Returns [left, right].
    """

    def __init__(
        self,
        pins=None,
        i2c: I2C | None = None,
        pcf_address: int | None = None,
        active_low=True,
        debounce_ms: int = 10,
    ):
        self._active_low = active_low
        self._debounce_ms = debounce_ms
        self._last_time = time.ticks_ms()

        # PCF8574
        self._pcf = None
        if i2c is None:
            i2c_id = getattr(cfg, 'I2C_ID', 0)
            scl = getattr(cfg, 'I2C0_SCL_PIN', None)
            sda = getattr(cfg, 'I2C0_SDA_PIN', None)
            freq = getattr(cfg, 'I2C0_FREQ_HZ', 400_000)
            if scl is not None and sda is not None:
                try:
                    i2c = I2C(i2c_id, scl=Pin(scl), sda=Pin(sda), freq=freq)
                except Exception:
                    i2c = None
        if pcf_address is None:
            pcf_address = getattr(cfg, 'PCF8574_ADDR', 0x20)
        if PCF8574 and i2c is not None:
            try:
                self._pcf = PCF8574(i2c, pcf_address)
            except Exception:
                self._pcf = None

        # GPIO fallback
        if pins is None:
            pins = getattr(cfg, 'BUMPER_PINS', None)
            if pins is None:
                candidates = [
                    getattr(cfg, 'BUMPER_FL_PIN', None),
                    getattr(cfg, 'BUMPER_FR_PIN', None),
                    getattr(cfg, 'BUMPER_RL_PIN', None),
                    getattr(cfg, 'BUMPER_RR_PIN', None),
                ]
                pins = [p for p in candidates if p is not None]
        self._gpio_pins = [Pin(p, Pin.IN, Pin.PULL_UP) for p in (pins or [])]
        self._last_state = self._read_now()

    def _read_now(self):
        if self._pcf is not None:
            try:
                raw = self._pcf.port
                bl = getattr(cfg, 'PCF_BIT_BUMPER_LEFT', 2)
                br = getattr(cfg, 'PCF_BIT_BUMPER_RIGHT', 3)
                l = (raw >> int(bl)) & 1
                r = (raw >> int(br)) & 1
                if self._active_low:
                    l = int(not l)
                    r = int(not r)
                return [bool(l), bool(r)]
            except Exception:
                pass
        vals = [bool(p.value()) for p in self._gpio_pins]
        if self._active_low:
            vals = [not v for v in vals]
        return vals

    def read(self):
        now = time.ticks_ms()
        if time.ticks_diff(now, self._last_time) < self._debounce_ms:
            return self._last_state
        self._last_time = now
        self._last_state = self._read_now()
        return self._last_state

    def any_pressed(self) -> bool:
        return any(self.read())
