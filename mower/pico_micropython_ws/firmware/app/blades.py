# Two independent blade motors using Motor and SafetyRelay
import time
import pins_config as cfg
from motor_base import Motor
from safety import SafetyRelay


class Blades:
    """Two independent blade motors managed separately from drive."""

    def __init__(self,
                 blade1_pwm_pin=cfg.BLADE1_PWM_PIN,
                 blade1_dir_pin=cfg.BLADE1_DIR_PIN,
                 blade2_pwm_pin=cfg.BLADE2_PWM_PIN,
                 blade2_dir_pin=cfg.BLADE2_DIR_PIN,
                 pwm_freq_hz: int = cfg.PWM_FREQ_HZ,
                 safety=None):
        self.blade1 = Motor(blade1_pwm_pin, blade1_dir_pin, pwm_freq_hz)
        self.blade2 = Motor(blade2_pwm_pin, blade2_dir_pin, pwm_freq_hz)

        self._max_abs_speed = getattr(cfg, 'BLADE_DEFAULT_MAX_ABS_SPEED', 1.0)
        self._accel_limit = getattr(cfg, 'BLADE_DEFAULT_ACCEL_LIMIT', 2.0)

        # Safety relay shared
        self._safety = safety if safety is not None else SafetyRelay()

        # Timing
        self._last_update_ms = time.ticks_ms()

    def enable(self):
        self._safety.enable()

    def disable(self):
        self._safety.disable()
        self.blade1.stop()
        self.blade2.stop()

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

    def command(self, blade1: float, blade2: float):
        if not self._safety.is_enabled():
            return
        if blade1 is None:
            blade1 = 0.0
        if blade2 is None:
            blade2 = 0.0
        blade1 = max(-self._max_abs_speed, min(self._max_abs_speed, blade1))
        blade2 = max(-self._max_abs_speed, min(self._max_abs_speed, blade2))
        self.blade1.set_command(blade1)
        self.blade2.set_command(blade2)

    def update(self):
        now = time.ticks_ms()
        dt = time.ticks_diff(now, self._last_update_ms) / 1000.0
        self._last_update_ms = now

        if not self._safety.is_enabled():
            self.blade1._apply_output(0.0)
            self.blade2._apply_output(0.0)
            return

        self.blade1.update(self.blade1.get_command(), self._accel_limit, dt)
        self.blade2.update(self.blade2.get_command(), self._accel_limit, dt)
