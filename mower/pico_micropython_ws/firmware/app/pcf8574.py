# Minimal PCF8574 driver for MicroPython (RP2040 / Pico)
# Exposes a .port property (0..255) for read/write of the 8-bit I/O expander.
#
# Notes:
# - PCF8574 pins are quasi-bidirectional. Writing a '1' makes the pin an input
#   with weak pull-up. Writing a '0' drives the pin low (output).
# - For pure input use (rain/lift/bumper), keep the corresponding bit set to 1.
# - For output (AUX), clear the bit to drive low; set to 1 to release/high.

from machine import I2C


class PCF8574:
    def __init__(self, i2c: I2C, address: int = 0x20):
        self.i2c = i2c
        self.address = address
        # Initialize shadow with all 1s (inputs/high)
        self._shadow = 0xFF
        try:
            # Try reading current port to sync shadow if device responds
            data = self.i2c.readfrom(self.address, 1)
            if data and len(data) == 1:
                self._shadow = data[0]
        except Exception:
            # If read fails (e.g., not yet wired), keep default 0xFF
            pass

    @property
    def port(self) -> int:
        """Return current 8-bit port value (reads from device)."""
        try:
            data = self.i2c.readfrom(self.address, 1)
            if data and len(data) == 1:
                self._shadow = data[0]
        except Exception:
            # On error, return last shadow
            pass
        return int(self._shadow) & 0xFF

    @port.setter
    def port(self, value: int):
        """Write 8-bit value to the port (0..255)."""
        v = int(value) & 0xFF
        try:
            self.i2c.writeto(self.address, bytes([v]))
            self._shadow = v
        except Exception:
            # Keep shadow unchanged if write fails
            pass
