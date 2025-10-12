# Odometry computation for differential drive using single-channel encoders
# Computes pose (x,y,theta) and velocities from signed tick counts.

import math
import pins_config as cfg


class DiffOdometry:
    def __init__(self,
                 ticks_per_rev_motor: int = cfg.TICKS_PER_REV_MOTOR,
                 gear_ratio: float = cfg.GEAR_RATIO,
                 wheel_radius_m: float = cfg.WHEEL_RADIUS_M,
                 wheel_separation_m: float = cfg.WHEEL_SEPARATION_M):
        # geometry
        self.R = float(wheel_radius_m)
        self.B = float(wheel_separation_m)
        # tick scale
        self.ticks_per_rev_wheel = float(ticks_per_rev_motor) * float(gear_ratio)
        if self.ticks_per_rev_wheel <= 0:
            self.ticks_per_rev_wheel = 1.0
        self.m_per_tick = (2.0 * math.pi * self.R) / self.ticks_per_rev_wheel

        # state
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # last velocities
        self.v = 0.0
        self.vth = 0.0

    def update(self, ticks_left: int, ticks_right: int, dt: float):
        # distances per wheel
        dL = float(ticks_left) * self.m_per_tick
        dR = float(ticks_right) * self.m_per_tick
        # twist
        d = 0.5 * (dL + dR)
        dth = (dR - dL) / self.B if self.B != 0.0 else 0.0

        # integrate pose (exact integration for diff drive small step)
        if abs(dth) < 1e-6:
            # straight approx
            dx = d * math.cos(self.th)
            dy = d * math.sin(self.th)
        else:
            R_icc = d / dth
            dx = R_icc * (math.sin(self.th + dth) - math.sin(self.th))
            dy = -R_icc * (math.cos(self.th + dth) - math.cos(self.th))

        self.x += dx
        self.y += dy
        self.th += dth
        # normalize theta to [-pi, pi]
        if self.th > math.pi or self.th < -math.pi:
            self.th = (self.th + math.pi) % (2.0 * math.pi) - math.pi

        # velocities
        if dt > 0.0:
            self.v = d / dt
            self.vth = dth / dt
        else:
            self.v = 0.0
            self.vth = 0.0

    def state(self):
        # return x, y, theta, vx, vy(=0), vth
        return self.x, self.y, self.th, self.v, 0.0, self.vth
