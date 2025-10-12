# Ultrasonic sonar helper for 3 sensors (L,C,R)
from machine import Pin, time_pulse_us
import time

class Sonar3:
    def __init__(self, trig_L: int, echo_L: int,
                       trig_C: int, echo_C: int,
                       trig_R: int, echo_R: int,
                       timeout_us: int = 30000):
        self.tL = Pin(trig_L, Pin.OUT)
        self.eL = Pin(echo_L, Pin.IN)
        self.tC = Pin(trig_C, Pin.OUT)
        self.eC = Pin(echo_C, Pin.IN)
        self.tR = Pin(trig_R, Pin.OUT)
        self.eR = Pin(echo_R, Pin.IN)
        self.timeout_us = int(timeout_us)
        # ensure low
        self.tL.value(0)
        self.tC.value(0)
        self.tR.value(0)

    @staticmethod
    def _measure(trig: Pin, echo: Pin, timeout_us: int) -> float:
        try:
            trig.value(0)
            time.sleep_us(2)
            trig.value(1)
            time.sleep_us(10)
            trig.value(0)
            dur = time_pulse_us(echo, 1, timeout_us)
            if dur < 0:
                return -1.0
            return (dur / 1_000_000.0 * 343.0) * 0.5
        except Exception:
            return -1.0

    def read_all(self) -> tuple:
        dL = self._measure(self.tL, self.eL, self.timeout_us)
        time.sleep_us(200)
        dC = self._measure(self.tC, self.eC, self.timeout_us)
        time.sleep_us(200)
        dR = self._measure(self.tR, self.eR, self.timeout_us)
        return (dL, dC, dR)
