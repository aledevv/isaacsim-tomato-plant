"""Mass-scaled mouse following for an already detached rigid body."""
from __future__ import annotations

import numpy as np


class FreeFruitGrip:
    """Velocity-damped tracking without changing the body's pose or velocity.

    The grip counters gravity while held. Releasing it means applying no more
    forces, so the fruit keeps its physical momentum and resumes free fall.
    """
    position_gain = 5.0  # desired velocity per metre of cursor error
    response_seconds = .05  # critically damped unsaturated linear response
    maximum_speed = 2.0  # m/s desired speed, not a velocity overwrite
    maximum_acceleration = 20.0  # m/s^2 commanded net acceleration

    @staticmethod
    def _limit(vector, magnitude):
        length = np.linalg.norm(vector)
        return vector * min(1., magnitude / max(float(length), 1e-12))

    def force(self, position, velocity, target, mass, gravity):
        position, velocity, target, gravity = [np.asarray(v, dtype=float) for v in
                                                (position, velocity, target, gravity)]
        if not all(np.isfinite(v).all() for v in (position, velocity, target, gravity)) or not np.isfinite(mass) or mass <= 0:
            raise ValueError("invalid free-fruit grip state")
        desired_velocity = self._limit(self.position_gain * (target - position), self.maximum_speed)
        acceleration = self._limit((desired_velocity - velocity) / self.response_seconds,
                                   self.maximum_acceleration)
        return self._limit(float(mass) * (acceleration - gravity), 12.)

    def settings(self):
        return {"position_gain_per_s": self.position_gain, "velocity_response_s": self.response_seconds,
                "desired_speed_cap_mps": self.maximum_speed,
                "net_acceleration_cap_mps2": self.maximum_acceleration,
                "force_cap_n": 12., "gravity_compensation": True}
