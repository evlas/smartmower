# Differential drive (left/right) using two Motor instances
from machine import time_pulse_us  # not used, placeholder to keep import structure minimal
import time
import pins_config as cfg
from motor_base import Motor
from safety import SafetyRelay


class Drive:
    """Two-wheel differential drive (left/right)."""

    def __init__(self,
                 left_pwm_pin=cfg.MOTOR_LEFT_PWM_PIN,
                 left_dir_pin=cfg.MOTOR_LEFT_DIR_PIN,
                 right_pwm_pin=cfg.MOTOR_RIGHT_PWM_PIN,
                 right_dir_pin=cfg.MOTOR_RIGHT_DIR_PIN,
                 pwm_freq_hz: int = cfg.PWM_FREQ_HZ,
                 safety=None):
        self.left = Motor(left_pwm_pin, left_dir_pin, pwm_freq_hz)
        self.right = Motor(right_pwm_pin, right_dir_pin, pwm_freq_hz)

        # Safety relay is handled by SafetyRelay class (shared)
        self._safety = safety if safety is not None else SafetyRelay()

        # Limits
        self._max_abs_speed = cfg.DEFAULT_MAX_ABS_SPEED
        self._accel_limit = cfg.DEFAULT_ACCEL_LIMIT

        # Timing
        self._last_update_ms = time.ticks_ms()

    def enable(self):
        """Enable via shared safety relay."""
        self._safety.enable()

    def disable(self):
        """Disable via shared safety relay and stop drive motors."""
        self._safety.disable()
        self.left.stop()
        self.right.stop()

    def is_enabled(self) -> bool:
        return self._safety.is_enabled()

    def set_limits(self, max_abs_speed: float = None, accel_limit: float = None):
        if max_abs_speed is not None:
            if max_abs_speed < 0.0:
                max_abs_speed = 0.0
            if max_abs_speed > 1.0:
                max_abs_speed = 1.0
            self._max_abs_speed = max_abs_speed
        if accel_limit is not None:
            if accel_limit < 0.0:
                accel_limit = 0.0
            self._accel_limit = accel_limit

    def command(self, left: float, right: float):
        """Set desired normalized speeds [-1..1] for each wheel (subject to limit)."""
        if not self._safety.is_enabled():
            # Ignore commands when disabled
            return
        # apply max limit
        left = max(-self._max_abs_speed, min(self._max_abs_speed, left))
        right = max(-self._max_abs_speed, min(self._max_abs_speed, right))
        self.left.set_command(left)
        self.right.set_command(right)

    def update(self):
        now = time.ticks_ms()
        dt = time.ticks_diff(now, self._last_update_ms) / 1000.0
        self._last_update_ms = now

#        if not self._safety.is_enabled():
#            # keep outputs off
#            self.left._apply_output(0.0)
#            self.right._apply_output(0.0)
#            return

        self.left.update(self.left.get_command(), self._accel_limit, dt)
        self.right.update(self.right.get_command(), self._accel_limit, dt)
