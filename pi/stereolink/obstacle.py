"""Depth map -> obstacle geometry.

The pipeline deliberately does not try to segment or classify anything.  For
obstacle avoidance the useful reduction is a 1-D free-space profile: for each
of N angular columns, how far can we travel before something blocks us.  That
is cheap, robust to disparity noise, and it is what the planner actually needs.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class ObstacleReport:
    # Per-column nearest range in metres; inf where the column is clear,
    # NaN where there was not enough valid disparity to decide.
    column_range_m: np.ndarray
    # Bearing of each column centre, normalised to [-1, +1] (left..right).
    column_bearing: np.ndarray
    min_range_m: float
    min_range_bearing: float
    valid_fraction: float
    # Best heading through free space, normalised like column_bearing.
    best_bearing: float
    blocked: bool
    coverage: np.ndarray = field(repr=False, default=None)

    @property
    def valid(self) -> bool:
        return self.valid_fraction > 0.15


class ObstacleAnalyzer:
    def __init__(self, cfg, rectifier):
        self.cfg = cfg
        self.rect = rectifier
        self._ema: np.ndarray | None = None

    def _roi_rows(self, height: int) -> tuple[int, int]:
        top = int(np.clip(self.cfg.roi_top, 0.0, 1.0) * height)
        bottom = int(np.clip(self.cfg.roi_bottom, 0.0, 1.0) * height)
        return top, max(bottom, top + 1)

    def analyze(self, depth_m: np.ndarray) -> ObstacleReport:
        cfg = self.cfg
        h, w = depth_m.shape
        top, bottom = self._roi_rows(h)
        roi = depth_m[top:bottom, :]

        # Three distinct things, which must not be conflated:
        #   solved    - stereo produced a usable depth here at all
        #   in_range  - that depth is close enough to be an obstacle candidate
        #   far       - solved, but beyond max_range: positive evidence of
        #               FREE SPACE, not a failure to measure
        # Treating "far" as invalid was a real bug: a clear corridor at 9 m
        # would read as unknown, and the planner would refuse to drive into it.
        solved = np.isfinite(roi) & (roi >= cfg.min_range_m)
        in_range = solved & (roi <= cfg.max_range_m)
        far = solved & (roi > cfg.max_range_m)
        valid = in_range.copy()

        # Ground rejection: for a level camera at height h_c, a point on the
        # floor at range Z projects to row v = cy + f*h_c/Z.  Anything whose
        # measured depth agrees with that within a tolerance is floor, not
        # obstacle, and must not stop the robot.
        rows = np.arange(top, bottom, dtype=np.float32)[:, None]
        dv = rows - self.rect.cy
        with np.errstate(divide="ignore", invalid="ignore"):
            ground_z = self.rect.focal_px * cfg.camera_height_m / dv
        ground_z = np.where(dv > 1.0, ground_z, np.inf)
        with np.errstate(invalid="ignore"):
            # roi and ground_z are both inf on cleared sky; inf-inf is NaN,
            # which compares false and is exactly the answer we want.
            is_ground = np.isfinite(ground_z) & np.isfinite(roi) & (
                np.abs(roi - ground_z) < cfg.ground_reject_m * np.maximum(roi, 1.0))
        valid &= ~is_ground

        # Split into columns and take a low percentile as the column range.
        # A percentile rather than the minimum, because a handful of bad
        # disparities would otherwise trigger an emergency stop every cycle.
        n = cfg.num_columns
        edges = np.linspace(0, w, n + 1).astype(int)
        col_range = np.full(n, np.nan, np.float32)
        counts = np.zeros(n, np.int32)
        for i in range(n):
            lo, hi = edges[i], edges[i + 1]
            vals = roi[:, lo:hi][valid[:, lo:hi]]
            counts[i] = vals.size
            n_far = int(np.count_nonzero(far[:, lo:hi]))
            if vals.size >= cfg.min_valid_px:
                col_range[i] = np.percentile(vals, cfg.percentile)
            elif n_far >= cfg.min_valid_px:
                # Measured, and everything measured is beyond our horizon:
                # this column is genuinely clear.
                col_range[i] = np.inf
            else:
                # Too little evidence either way.  Left as NaN = UNKNOWN, which
                # is deliberately not the same as clear: the planner de-rates
                # speed on a low valid_fraction rather than driving blind.
                col_range[i] = np.nan

        # Temporal EMA over columns that have a measurement, which removes the
        # frame-to-frame flicker SGBM produces at range without adding the lag
        # of a full filter.  inf/NaN columns pass through untouched.
        finite = np.isfinite(col_range)
        if self._ema is None or self._ema.shape != col_range.shape:
            self._ema = col_range.copy()
        else:
            a = cfg.temporal_alpha
            prev_ok = finite & np.isfinite(self._ema)
            self._ema = np.where(prev_ok, a * col_range + (1 - a) * self._ema, col_range)
        smoothed = self._ema.astype(np.float32)

        centers = (edges[:-1] + edges[1:]) / 2.0
        bearing = ((centers - self.rect.cx) / (w / 2.0)).astype(np.float32)

        measured = np.isfinite(smoothed)
        if measured.any():
            idx = int(np.nanargmin(np.where(measured, smoothed, np.inf)))
            min_range = float(smoothed[idx])
            min_bearing = float(bearing[idx])
        else:
            min_range, min_bearing = float("inf"), 0.0

        total = max(int(np.prod(roi.shape)), 1)
        valid_fraction = float(np.count_nonzero(solved)) / total

        best_bearing = self._choose_heading(smoothed, bearing)
        blocked = min_range < self.cfg.min_range_m * 1.5 or (
            np.all(np.isfinite(smoothed)) and float(np.nanmax(smoothed)) < 0.6)

        return ObstacleReport(smoothed, bearing, min_range, min_bearing,
                              valid_fraction, best_bearing, bool(blocked), counts)

    def _choose_heading(self, col_range: np.ndarray, bearing: np.ndarray) -> float:
        """Pick the centre of the widest sufficiently-deep corridor.

        Scoring depth alone would make the robot chase a single far-away column
        between two obstacles that it cannot physically fit through.  Weighting
        by contiguous width biases it toward gaps it can actually use, and the
        small centre bias keeps it from oscillating between two equal gaps.
        """
        # Unknown columns are scored as mid-range: traversable, but less
        # attractive than a column we have actually measured as clear.  Scoring
        # them 0 would make the robot treat every unmeasurable surface as a
        # wall; scoring them max would make it prefer driving where it is
        # blind.  The global valid_fraction check is what keeps this safe.
        unknown = np.isnan(col_range)
        depth = np.where(unknown, self.cfg.max_range_m * 0.5, col_range)
        depth = np.minimum(depth, self.cfg.max_range_m)
        passable = depth >= self.cfg.min_range_m * 4.0

        best_score, best = -1.0, 0.0
        i = 0
        n = len(depth)
        while i < n:
            if not passable[i]:
                i += 1
                continue
            j = i
            while j + 1 < n and passable[j + 1]:
                j += 1
            width = (j - i + 1) / n
            gap_depth = float(np.min(depth[i:j + 1])) / self.cfg.max_range_m
            centre = float(np.mean(bearing[i:j + 1]))
            score = 0.55 * width + 0.45 * gap_depth - 0.15 * abs(centre)
            if score > best_score:
                best_score, best = score, centre
            i = j + 1

        if best_score < 0:
            # Fully blocked: steer toward whichever side is least bad.
            best = float(bearing[int(np.argmax(depth))]) if n else 0.0
        return float(np.clip(best, -1.0, 1.0))
