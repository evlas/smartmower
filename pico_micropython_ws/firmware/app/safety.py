# Safety relay controller (RP2040 / MicroPython)
from machine import Pin
import pins_config as cfg


class SafetyRelay:
    """Shared safety relay controller. Active-high assumed (1=enabled)."""

    def __init__(self, safety_relay_pin=cfg.SAFETY_RELAY_PIN):
        self._relay = Pin(safety_relay_pin, Pin.OUT)
        self._enabled = False
        self.disable()

    def enable(self):
        self._relay.value(0)  # Invertito: 0 per attivare
        self._enabled = True

    def disable(self):
        self._relay.value(1)  # Invertito: 1 per disattivare
        self._enabled = False

    def is_enabled(self) -> bool:
        return self._enabled
