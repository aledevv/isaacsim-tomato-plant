"""One-time break-force arming after sustained support rest; no force controller."""
import math


class SettlingGate:
    def __init__(self, minimum_s=2., quiet_s=1., linear_limit=.005, angular_limit=.05):
        self.minimum_s = minimum_s
        self.quiet_s = quiet_s
        self.linear_limit = linear_limit
        self.angular_limit = angular_limit
        self.quiet_elapsed = 0.
        self.armed_at = None

    def update(self, time_s, dt, linear_speed, angular_speed, dragging=False):
        if self.armed_at is not None:
            return False
        quiet = (time_s >= self.minimum_s and not dragging and
                 math.isfinite(linear_speed) and math.isfinite(angular_speed) and
                 linear_speed < self.linear_limit and angular_speed < self.angular_limit)
        self.quiet_elapsed = self.quiet_elapsed + dt if quiet else 0.
        if self.quiet_elapsed + 1e-9 >= self.quiet_s:
            self.armed_at = time_s
            return True
        return False

    def summary(self):
        return dict(armed_at_s=self.armed_at, quiet_elapsed_s=self.quiet_elapsed,
                    minimum_s=self.minimum_s, required_quiet_s=self.quiet_s,
                    linear_limit_mps=self.linear_limit, angular_limit_radps=self.angular_limit,
                    one_time=True)
