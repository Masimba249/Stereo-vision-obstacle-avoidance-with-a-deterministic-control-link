"""Free-space profile -> (v, omega) setpoint.

Kept deliberately simple and stateless-except-for-slew.  The Pi is the advisory
layer here: it can be late, it can be wrong, and the ESP32 still has to be safe.
Anything genuinely safety-critical (proximity stop, link loss) lives in firmware.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from .pdo import SetpointFlags


@dataclass
class Setpoint:
    v_mm_s: float
    w_mrad_s: float
    obstacle_cm: int
    flags: SetpointFlags
    reason: str = ""


class AvoidancePlanner:
    def __init__(self, cfg):
        self.cfg = cfg
        self._w_prev = 0.0
        self._t_prev: float | None = None
        self._last_valid_t: float | None = None

    def reset(self) -> None:
        self._w_prev = 0.0
        self._t_prev = None
        self._last_valid_t = None

    def plan(self, report, now: float | None = None, estop: bool = False) -> Setpoint:
        cfg = self.cfg
        now = time.monotonic() if now is None else now
        dt = 0.1 if self._t_prev is None else max(now - self._t_prev, 1e-3)
        self._t_prev = now

        flags = SetpointFlags.NONE
        if estop:
            self._w_prev = 0.0
            return Setpoint(0.0, 0.0, 0, SetpointFlags.ESTOP_REQUEST, "estop")

        vision_ok = report is not None and report.valid
        if vision_ok:
            self._last_valid_t = now
            flags |= SetpointFlags.VISION_VALID
        elif self._last_valid_t is None or (now - self._last_valid_t) > cfg.vision_timeout_s:
            # No trustworthy depth for longer than the timeout.  We do not
            # coast on a stale plan; we command zero and let the firmware's
            # own watchdog be the backstop if even this stops arriving.
            self._w_prev = 0.0
            return Setpoint(0.0, 0.0, 0, flags, "vision_stale")

        rng = float(report.min_range_m) if report is not None else float("inf")
        obstacle_cm = int(np.clip(rng * 100.0, 0, 65535)) if np.isfinite(rng) else 65535

        # Speed: full stop inside stop_distance, linear ramp up to
        # slow_distance, cruise beyond.  Linear in distance (not in time) means
        # the de-rate is a function of geometry alone and is trivially auditable.
        if not np.isfinite(rng) or rng >= cfg.slow_distance_m:
            v = cfg.v_cruise_mm_s
            reason = "cruise"
        elif rng <= cfg.stop_distance_m:
            v = 0.0
            reason = "stop"
            flags |= SetpointFlags.OBSTACLE_NEAR
        else:
            span = max(cfg.slow_distance_m - cfg.stop_distance_m, 1e-3)
            frac = (rng - cfg.stop_distance_m) / span
            v = cfg.v_min_mm_s + frac * (cfg.v_cruise_mm_s - cfg.v_min_mm_s)
            reason = "slow"
            flags |= SetpointFlags.OBSTACLE_NEAR

        # Steering toward the chosen corridor.  When stopped we still command
        # yaw, so the robot turns in place to find a way out instead of sitting
        # in front of the obstacle forever.
        bearing = float(report.best_bearing) if report is not None else 0.0
        w_cmd = -cfg.steer_gain * bearing
        if report is not None and report.blocked and v == 0.0:
            w_cmd = cfg.w_max_mrad_s * (1.0 if bearing >= 0 else -1.0)
            reason = "turn_in_place"

        if not vision_ok:
            # Inside the grace window: creep, do not steer on stale geometry.
            v = min(v, cfg.v_min_mm_s)
            w_cmd = 0.0
            reason = "vision_grace"

        w_cmd = float(np.clip(w_cmd, -cfg.w_max_mrad_s, cfg.w_max_mrad_s))
        # Slew-limit yaw so a one-frame disparity glitch cannot snap the
        # wheels over; the firmware would honour it faithfully, which is
        # exactly why the limit has to be applied before it is sent.
        max_step = cfg.steer_slew_mrad_s2 * dt
        w = float(np.clip(w_cmd, self._w_prev - max_step, self._w_prev + max_step))
        self._w_prev = w

        return Setpoint(float(v), w, obstacle_cm, flags, reason)
