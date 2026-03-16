"""
navigation/alignment.py
─────────────────────────────────────────────────────────────────────────────
IBVS yaw alignment PID + dead-reckoning position tracker.
"""

import time
import math
from dataclasses import dataclass

from rov_logger import get_logger

log = get_logger("navigation")


# ── PID ───────────────────────────────────────────────────────────────────

class PID:
    """Anti-windup PID with derivative filtering."""

    def __init__(self, kp: float, ki: float, kd: float,
                 out_min: float = -1.0, out_max: float = 1.0,
                 d_filter_alpha: float = 0.7):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self._alpha   = d_filter_alpha
        self._integral = 0.0
        self._prev_err = 0.0
        self._d_filt   = 0.0
        self._prev_t   = None

    def update(self, error: float) -> float:
        now = time.monotonic()
        dt  = (now - self._prev_t) if self._prev_t else 0.02
        dt  = max(0.001, min(0.1, dt))
        self._prev_t = now

        p = self.kp * error
        self._integral += error * dt
        d_raw     = (error - self._prev_err) / dt
        self._d_filt = self._alpha * self._d_filt + (1 - self._alpha) * d_raw
        self._prev_err = error

        out = p + self.ki * self._integral + self.kd * self._d_filt
        # Anti-windup clamp
        if out > self.out_max:
            out = self.out_max
            self._integral -= error * dt
        elif out < self.out_min:
            out = self.out_min
            self._integral -= error * dt
        return out

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_err = 0.0
        self._d_filt   = 0.0
        self._prev_t   = None


# ── IBVS Aligner ──────────────────────────────────────────────────────────

class IBVSAligner:
    """
    Image-Based Visual Servoing.
    error_x = target_cx − 320  → yaw_cmd via PID
    """

    def __init__(self, cfg: dict):
        a = cfg["alignment"]
        self._pid = PID(
            kp=a["yaw_kp"], ki=a["yaw_ki"], kd=a["yaw_kd"],
            out_min=-a["yaw_max_cmd"], out_max=a["yaw_max_cmd"]
        )
        self._tol = a["center_tolerance_px"]

    def compute_yaw(self, error_x: int) -> float:
        return self._pid.update(float(error_x))

    def is_aligned(self, error_x: int) -> bool:
        return abs(error_x) <= self._tol

    def reset(self) -> None:
        self._pid.reset()


# ── Dead Reckoning ────────────────────────────────────────────────────────

@dataclass
class Pose:
    x:       float = 0.0   # metres (forward)
    y:       float = 0.0   # metres (lateral)
    heading: float = 0.0   # degrees (0 = start direction)


class DeadReckoning:
    """
    Tracks estimated X/Y/heading from motion commands.
    No external sensor feedback — pure integration.
    Sufficient for Robofest where the pool is known and bounded.
    """

    def __init__(self, surge_mps: float = 0.5, yaw_dps: float = 60.0):
        """
        surge_mps: full-throttle surge speed in m/s (calibrate in pool)
        yaw_dps:   full-throttle yaw rate in deg/s (calibrate in pool)
        """
        self._surge_mps = surge_mps
        self._yaw_dps   = yaw_dps
        self._pose      = Pose()
        self._last_t    = time.monotonic()

    def update(self, surge: float, yaw: float) -> Pose:
        """
        surge, yaw: normalised −1…+1 commands
        Returns updated Pose.
        """
        now = time.monotonic()
        dt  = min(now - self._last_t, 0.2)
        self._last_t = now

        # Update heading
        dh = yaw * self._yaw_dps * dt
        self._pose.heading = (self._pose.heading + dh) % 360.0

        # Update X/Y
        speed = surge * self._surge_mps
        rad   = math.radians(self._pose.heading)
        self._pose.x += speed * math.cos(rad) * dt
        self._pose.y += speed * math.sin(rad) * dt

        return self.pose

    def reset(self) -> None:
        self._pose = Pose()
        self._last_t = time.monotonic()
        log.debug("Dead reckoning reset")

    @property
    def pose(self) -> Pose:
        return Pose(self._pose.x, self._pose.y, self._pose.heading)
