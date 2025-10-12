# Lift sensors via GPIO or PCF8574
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


class LiftSensors:
    """Read lift switch (single boolean) via PCF8574 bit or GPIO list.
    Returns a list with one element when using PCF bit mapping; otherwise one
    value per configured GPIO pin.
    """

    def __init__(
        self,
        pins=None,
        i2c: I2C | None = None,
        pcf_address: int | None = None,
        pull=Pin.PULL_UP,
        active_low=True,
        debounce_ms: int = 20,
    ):
        self._active_low = active_low
        self._debounce_ms = debounce_ms
        self._last_time = time.ticks_ms()

        # Optional PCF8574
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
            pins = getattr(cfg, 'LIFT_PINS', None)
            if pins is None:
                left = getattr(cfg, 'LIFT_LEFT_PIN', None)
                right = getattr(cfg, 'LIFT_RIGHT_PIN', None)
                pins = [p for p in (left, right) if p is not None]
        self._pins = [Pin(p, Pin.IN, pull) for p in (pins or [])]
        self._last_state = self._read_now()

    def _read_now(self):
        if self._pcf is not None:
            try:
                raw = self._pcf.port
                bit_idx = getattr(cfg, 'PCF_BIT_LIFT', None)
                if bit_idx is not None:
                    b = (raw >> int(bit_idx)) & 1
                    if self._active_low:
                        b = int(not b)
                    return [bool(b)]
            except Exception:
                pass
        vals = [bool(pin.value()) for pin in self._pins]
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

    def any_lifted(self) -> bool:
        return any(self.read())
