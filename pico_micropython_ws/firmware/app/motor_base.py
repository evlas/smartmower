# Low-level Motor class: PWM + DIR for RP2040 (MicroPython)
from machine import Pin, PWM
import pins_config as cfg


class Motor:
    def __init__(self, pwm_pin: int, dir_pin: int, pwm_freq_hz: int = cfg.PWM_FREQ_HZ,
                 deadzone: float = cfg.DEFAULT_DEADZONE):
        self._dir = Pin(dir_pin, Pin.OUT)
        self._pwm = PWM(Pin(pwm_pin))
        self._pwm.freq(pwm_freq_hz)

        # normalized command in [-1.0, 1.0]
        self._cmd = 0.0
        self._deadzone = max(0.0, min(deadzone, 0.2))

        # initialize stopped
        self._apply_output(0.0)

    def set_command(self, cmd: float):
        """Set normalized command in [-1.0, 1.0]."""
        if cmd is None:
            cmd = 0.0
        if cmd > 1.0:
            cmd = 1.0
        elif cmd < -1.0:
            cmd = -1.0
        self._cmd = cmd

    def get_command(self) -> float:
        return self._cmd

    def stop(self):
        self._cmd = 0.0
        self._apply_output(0.0)

    def _apply_output(self, cmd: float):
        # Direction: True = forward (cmd >= 0), False = reverse
        forward = cmd >= 0.0
        self._dir.value(1 if forward else 0)

        # Deadzone handling
        mag = abs(cmd)
        if mag < self._deadzone:
            duty = 0
        else:
            duty = int(mag * cfg.PWM_DUTY_MAX)
            if duty > cfg.PWM_DUTY_MAX:
                duty = cfg.PWM_DUTY_MAX
        self._pwm.duty_u16(duty)

    def update(self, desired: float, accel_limit: float, dt: float):
        """Ramps current command toward desired with accel_limit (units/s)."""
        # clamp desired
        if desired > 1.0:
            desired = 1.0
        elif desired < -1.0:
            desired = -1.0

        if accel_limit is None or accel_limit <= 0.0:
            self._cmd = desired
        else:
            # ramp
            delta = desired - self._cmd
            max_step = accel_limit * dt
            if delta > max_step:
                delta = max_step
            elif delta < -max_step:
                delta = -max_step
            self._cmd += delta

        self._apply_output(self._cmd)

    def dir_forward(self) -> bool:
        """Returns True if current output direction is forward (non-negative)."""
        try:
            return bool(self._dir.value())
        except Exception:
            # Fallback to command sign
            return self._cmd >= 0.0
