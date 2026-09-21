"""Stereo calibration storage and rectification map construction.

Rectification maps are built once at startup and reused for every frame - this
is why the rectify stage costs ~2 ms instead of ~40 ms in the latency budget.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


@dataclass
class StereoCalibration:
    image_size: tuple[int, int]          # (width, height) of the calibrated eye
    K1: np.ndarray
    D1: np.ndarray
    K2: np.ndarray
    D2: np.ndarray
    R: np.ndarray                        # right w.r.t. left
    T: np.ndarray                        # metres
    rms: float = 0.0

    @property
    def baseline_m(self) -> float:
        return float(np.linalg.norm(self.T))

    def save(self, path: str | Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
        fs.write("image_width", int(self.image_size[0]))
        fs.write("image_height", int(self.image_size[1]))
        for name, mat in (("K1", self.K1), ("D1", self.D1), ("K2", self.K2),
                          ("D2", self.D2), ("R", self.R), ("T", self.T)):
            fs.write(name, np.asarray(mat, dtype=np.float64))
        fs.write("rms", float(self.rms))
        fs.release()

    @classmethod
    def load(cls, path: str | Path) -> "StereoCalibration":
        # Check first: cv2.FileStorage writes its own error to stderr for a
        # missing file, which is noise on the synthetic path where the absence
        # is expected and handled.
        if not Path(path).is_file():
            raise FileNotFoundError(f"no calibration file at {path}")
        fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            raise FileNotFoundError(f"cannot open calibration file: {path}")
        try:
            size = (int(fs.getNode("image_width").real()),
                    int(fs.getNode("image_height").real()))
            mats = {n: fs.getNode(n).mat() for n in ("K1", "D1", "K2", "D2", "R", "T")}
            rms_node = fs.getNode("rms")
            rms = float(rms_node.real()) if not rms_node.empty() else 0.0
        finally:
            fs.release()
        missing = [n for n, m in mats.items() if m is None]
        if missing:
            raise ValueError(f"calibration file {path} missing: {', '.join(missing)}")
        return cls(size, mats["K1"], mats["D1"], mats["K2"], mats["D2"],
                   mats["R"], mats["T"], rms)

    @classmethod
    def synthetic(cls, width: int = 1280, height: int = 720,
                  fov_deg: float = 70.0, baseline_m: float = 0.06) -> "StereoCalibration":
        """A plausible, perfectly-rectified rig.

        Used by the synthetic camera and the unit tests so the whole pipeline
        runs end to end on a laptop with no hardware and no calibration file.
        """
        f = (width / 2.0) / np.tan(np.deg2rad(fov_deg) / 2.0)
        K = np.array([[f, 0, width / 2.0], [0, f, height / 2.0], [0, 0, 1]], np.float64)
        zero = np.zeros((1, 5), np.float64)
        return cls((width, height), K.copy(), zero.copy(), K.copy(), zero.copy(),
                   np.eye(3, dtype=np.float64),
                   np.array([[-baseline_m], [0.0], [0.0]], np.float64), 0.0)


class Rectifier:
    """Pre-computed remap tables plus the Q matrix for reprojection."""

    def __init__(self, calib: StereoCalibration,
                 work_size: tuple[int, int] | None = None, alpha: float = 0.0):
        self.calib = calib
        self.work_size = work_size or calib.image_size
        sx = self.work_size[0] / calib.image_size[0]
        sy = self.work_size[1] / calib.image_size[1]

        # Scale the intrinsics into the working resolution rather than
        # rectifying at full res and then resizing: one interpolation instead
        # of two, and remap runs on the smaller image.
        S = np.array([[sx, 0, 0], [0, sy, 0], [0, 0, 1]], np.float64)
        K1 = S @ calib.K1
        K2 = S @ calib.K2

        R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
            K1, calib.D1, K2, calib.D2, self.work_size, calib.R, calib.T,
            flags=cv2.CALIB_ZERO_DISPARITY, alpha=alpha, newImageSize=self.work_size,
        )
        self.R1, self.R2, self.P1, self.P2, self.Q = R1, R2, P1, P2, Q
        self.roi1, self.roi2 = roi1, roi2

        # CV_16SC2 fixed-point maps are ~2x faster to remap than float maps.
        self.map1x, self.map1y = cv2.initUndistortRectifyMap(
            K1, calib.D1, R1, P1, self.work_size, cv2.CV_16SC2)
        self.map2x, self.map2y = cv2.initUndistortRectifyMap(
            K2, calib.D2, R2, P2, self.work_size, cv2.CV_16SC2)

    @property
    def focal_px(self) -> float:
        return float(self.P1[0, 0])

    @property
    def baseline_m(self) -> float:
        # P2[0,3] = -f * Tx  (Tx negative for a left-referenced rig)
        f = self.focal_px
        return abs(float(self.P2[0, 3]) / f) if f else self.calib.baseline_m

    @property
    def cx(self) -> float:
        return float(self.P1[0, 2])

    @property
    def cy(self) -> float:
        return float(self.P1[1, 2])

    def rectify(self, left: np.ndarray, right: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        interp = cv2.INTER_LINEAR
        lr = cv2.remap(left, self.map1x, self.map1y, interp)
        rr = cv2.remap(right, self.map2x, self.map2y, interp)
        return lr, rr

    def depth_from_disparity(self, disparity_px: np.ndarray) -> np.ndarray:
        """Z = f * B / d, with invalid/zero disparity mapped to +inf."""
        f_b = self.focal_px * self.baseline_m
        with np.errstate(divide="ignore", invalid="ignore"):
            depth = f_b / disparity_px
        depth[~np.isfinite(depth)] = np.inf
        depth[disparity_px <= 0] = np.inf
        return depth
