# Single-channel encoder support for RP2040 (MicroPython)
# Counts rising edges on a GPIO with IRQ and optional debounce.

from machine import Pin
import time

class SingleChannelEncoder:
    def __init__(self, pin_num: int, debounce_us: int = 200):
        # debounce_us: minimum time between valid edges to filter bounce
        self._pin = Pin(pin_num, Pin.IN)
        self._pin.init(pull=Pin.PULL_DOWN)
        self._debounce = debounce_us
        self._count = 0
        self._last_us = time.ticks_us()
        # IRQ on rising edges
        self._pin.irq(trigger=Pin.IRQ_RISING, handler=self._irq)

    def _irq(self, _pin):
        now = time.ticks_us()
        if self._debounce > 0:
            if time.ticks_diff(now, self._last_us) < self._debounce:
                return
        self._last_us = now
        # increment count
        self._count += 1

    def read_and_reset(self) -> int:
        c = self._count
        self._count = 0
        return c

    def read(self) -> int:
        return self._count
