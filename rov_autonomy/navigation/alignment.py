"""
navigation/alignment.py
PID controller + Image-Based Visual Servoing (IBVS) alignment.

The ESP32 already does roll/pitch/depth stabilisation.
This PID only needs to produce yaw and surge commands for centering on targets.
"""

import time


class PID:
    """Simple PID with anti-windup and output clamping."""

    def __init__(self, kp: float, ki: float, kd: float,
                 out_min: float = -1.0, out_max: float = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None

    def update(self, error: float) -> float:
        now = time.monotonic()
        if self._prev_time is None:
            dt = 0.02
        else:
            dt = now - self._prev_time
            dt = max(0.001, min(dt, 0.1))
        self._prev_time = now

        p = self.kp * error
        self._integral += error * dt
        i = self.ki * self._integral
        d = self.kd * (error - self._prev_error) / dt
        self._prev_error = error

        out = p + i + d
        # Anti-windup: clamp integral if output is saturated
        if out > self.out_max:
            out = self.out_max
            self._integral -= error * dt
        elif out < self.out_min:
            out = self.out_min
            self._integral -= error * dt

        return out

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None


class IBVSAligner:
    """
    Image-Based Visual Servoing.
    Produces yaw_cmd (−1…+1) to center a target horizontally in frame.

    error_x = target_cx − frame_cx
    yaw_cmd = PID(error_x)  → sent to CH4 on ESP32
    """

    def __init__(self, cfg: dict):
        a = cfg["alignment"]
        self._pid = PID(
            kp=a["yaw_kp"],
            ki=a["yaw_ki"],
            kd=a["yaw_kd"],
            out_min=-a["yaw_max_cmd"],
            out_max=a["yaw_max_cmd"]
        )
        self._tolerance_px = a["center_tolerance_px"]

    def compute_yaw(self, error_x: int) -> float:
        """
        error_x: target_cx - frame_cx  (pixels)
        Returns yaw_cmd −1…+1 to feed into ibus.set_motion(yaw=...)
        """
        return self._pid.update(error_x)

    def is_aligned(self, error_x: int) -> bool:
        return abs(error_x) <= self._tolerance_px

    def reset(self):
        self._pid.reset()
