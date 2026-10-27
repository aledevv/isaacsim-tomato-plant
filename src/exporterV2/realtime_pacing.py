"""Wall-clock pacing without changing the simulation timestep."""
import time


class RealtimePacer:
    def __init__(self, dt, clock=time.perf_counter, sleep=time.sleep):
        self.dt, self.clock, self.sleep = dt, clock, sleep
        self.anchor = None

    def reset(self):
        self.anchor = None

    def wait(self, simulation_s):
        now = self.clock()
        if self.anchor is None:
            self.anchor = now - simulation_s
        delay = self.anchor + simulation_s - now
        if delay > 0:
            self.sleep(delay)
            return delay
        if delay < -self.dt:
            # A slow frame or pause must not trigger a burst of catch-up steps.
            self.anchor = now - simulation_s
        return 0.0
