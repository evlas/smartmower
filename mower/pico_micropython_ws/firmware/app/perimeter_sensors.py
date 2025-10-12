# Dual perimeter sensors via ADC (left/right)
from machine import ADC

try:
    import pins_config as cfg
except Exception:
    cfg = object()

import time


class PerimeterSensors:
    """Read left and right perimeter sensors via ADC with thresholding and averaging."""

    def __init__(self, left_adc_pin=None, right_adc_pin=None, threshold=None, samples=8):
        if left_adc_pin is None:
            left_adc_pin = getattr(cfg, 'PERIMETER_LEFT_ADC_PIN', None)
        if right_adc_pin is None:
            right_adc_pin = getattr(cfg, 'PERIMETER_RIGHT_ADC_PIN', None)
        if threshold is None:
            threshold = getattr(cfg, 'PERIMETER_THRESHOLD', 1800)  # 0..4095
        self._threshold = int(threshold)
        self._samples = max(1, int(samples))
        self._left_adc = ADC(left_adc_pin) if left_adc_pin is not None else None
        self._right_adc = ADC(right_adc_pin) if right_adc_pin is not None else None

    def _read_adc(self, adc) -> int:
        if adc is None:
            return 0
        total = 0
        for _ in range(self._samples):
            total += adc.read_u16() >> 4  # scale to ~12-bit
        return total // self._samples

    def raw_left(self) -> int:
        return self._read_adc(self._left_adc)

    def raw_right(self) -> int:
        return self._read_adc(self._right_adc)

    def left_active(self) -> bool:
        return self.raw_left() >= self._threshold

    def right_active(self) -> bool:
        return self.raw_right() >= self._threshold

    def set_threshold(self, value: int):
        self._threshold = int(value)
