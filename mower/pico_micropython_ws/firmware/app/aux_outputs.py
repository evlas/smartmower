# AUX outputs via GPIO or PCF8574
from machine import Pin, I2C

try:
    import pins_config as cfg
except Exception:
    cfg = object()

try:
    from pcf8574 import PCF8574  # type: ignore
except Exception:
    PCF8574 = None


class AuxOutputs:
    """Auxiliary outputs via GPIO pins or PCF8574 (bits P4..P7 for AUX1..AUX4).

    When using PCF8574, uses read-modify-write on the 8-bit port to preserve
    non-AUX bits (rain/lift/bumper inputs).
    """

    def __init__(self, pins=None, i2c: I2C | None = None, pcf_address: int | None = None):
        self._pcf = None
        if pins is None:
            pins = getattr(cfg, 'AUX_PINS', None)
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

        # Prepare GPIO outputs fallback
        self._gpio_pins = [Pin(p, Pin.OUT) for p in (pins or [])]

    def write_gpio(self, index: int, value: bool):
        if 0 <= index < len(self._gpio_pins):
            self._gpio_pins[index].value(1 if value else 0)

    def write_pcf(self, mask: int):
        if self._pcf is not None:
            try:
                self._pcf.port = mask & 0xFF
            except Exception:
                pass

    def write_aux(self, index: int, value: bool):
        """Set AUX1..AUX4 (mapped to PCF bits 4..7) without disturbing other bits.

        index: 0..3 corresponds to AUX1..AUX4.
        """
        if self._pcf is None:
            # Fall back to GPIO outputs if available
            self.write_gpio(index, value)
            return
        try:
            bits = getattr(cfg, 'PCF_BITS_AUX', (4, 5, 6, 7))
            bit = int(bits[index])
            current = int(self._pcf.port) & 0xFF
            if value:
                new_val = current | (1 << bit)
            else:
                new_val = current & ~(1 << bit)
            self._pcf.port = new_val & 0xFF
        except Exception:
            pass

    def write_aux_mask(self, aux_mask_4bits: int):
        """Set AUX1..AUX4 from a 4-bit mask (bit0=AUX1 ... bit3=AUX4).
        Preserves non-AUX bits on the PCF.
        """
        if self._pcf is None:
            for i in range(4):
                self.write_gpio(i, bool((aux_mask_4bits >> i) & 1))
            return
        try:
            bits = getattr(cfg, 'PCF_BITS_AUX', (4, 5, 6, 7))
            current = int(self._pcf.port) & 0xFF
            # Clear AUX bits
            for b in bits:
                current &= ~(1 << int(b))
            # Set according to mask
            for i, b in enumerate(bits):
                if (aux_mask_4bits >> i) & 1:
                    current |= (1 << int(b))
            self._pcf.port = current & 0xFF
        except Exception:
            pass

    def all_off(self):
        for p in self._gpio_pins:
            p.value(0)
        if self._pcf is not None:
            try:
                self._pcf.port = 0x00
            except Exception:
                pass
