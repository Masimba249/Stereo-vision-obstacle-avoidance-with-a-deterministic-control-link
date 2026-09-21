"""Stereo capture sources.

Two implementations behind one interface:

* :class:`V4L2StereoCamera` - the Waveshare binocular module, which presents as
  a single UVC device streaming both eyes side by side in one frame.  That is
  the useful property of this hardware: the two eyes share a frame buffer, so
  they are captured by the same exposure window and need no software sync.
* :class:`SyntheticStereoCamera` - renders a geometrically correct stereo pair
  from a known depth scene, so the entire pipeline (including SGBM, the planner
  and the CAN link) can be exercised and regression-tested on a desktop.
"""

from __future__ import annotations

import time
from abc import ABC, abstractmethod
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class StereoFrame:
    left: np.ndarray
    right: np.ndarray
    # Monotonic timestamp taken as close to the driver handoff as we can get.
    # Stage 0 of the latency budget; every later stage is measured against it.
    t_capture: float
    index: int


class StereoSource(ABC):
    @abstractmethod
    def read(self) -> StereoFrame | None: ...

    @property
    @abstractmethod
    def eye_size(self) -> tuple[int, int]: ...

    def close(self) -> None:
        pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


class V4L2StereoCamera(StereoSource):
    def __init__(self, device: int | str = 0, width: int = 2560, height: int = 720,
                 fps: int = 30, fourcc: str = "MJPG", swap_eyes: bool = False):
        self._cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            raise RuntimeError(f"cannot open stereo camera device {device!r}")
        # Order matters: FOURCC before size, or the driver may refuse the
        # high-bandwidth mode and silently fall back to YUYV at 5 fps.
        self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._cap.set(cv2.CAP_PROP_FPS, fps)
        # A 1-frame buffer is what keeps capture latency bounded: we would
        # rather drop frames than hand the planner a stale one.
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if actual_w != width or actual_h != height:
            # Not fatal - just make sure it is visible in the log rather than
            # showing up later as a mysterious calibration mismatch.
            print(f"[camera] requested {width}x{height}, driver gave "
                  f"{actual_w}x{actual_h}")
        if actual_w % 2:
            raise RuntimeError(f"odd capture width {actual_w}, cannot split eyes")
        self._w, self._h = actual_w, actual_h
        self._swap = swap_eyes
        self._index = 0

    @property
    def eye_size(self) -> tuple[int, int]:
        return (self._w // 2, self._h)

    def read(self) -> StereoFrame | None:
        # grab() then retrieve() so the timestamp brackets the actual DMA
        # handoff and excludes JPEG decode, which we bill to its own stage.
        ok = self._cap.grab()
        t_capture = time.monotonic()
        if not ok:
            return None
        ok, frame = self._cap.retrieve()
        if not ok or frame is None:
            return None
        half = frame.shape[1] // 2
        a, b = frame[:, :half], frame[:, half:]
        left, right = (b, a) if self._swap else (a, b)
        self._index += 1
        return StereoFrame(np.ascontiguousarray(left), np.ascontiguousarray(right),
                           t_capture, self._index)

    def close(self) -> None:
        self._cap.release()


@dataclass
class SceneBox:
    """An axis-aligned obstacle placed in camera coordinates (metres)."""

    x_m: float        # lateral, +right
    z_m: float        # range
    width_m: float
    height_m: float
    y_m: float = 0.0  # vertical centre, +down from optical axis
    vx_m_s: float = 0.0
    vz_m_s: float = 0.0


class SyntheticStereoCamera(StereoSource):
    """Renders a stereo pair from a depth buffer by forward-warping the left eye.

    The right image is produced by shifting each left pixel by d = f*B/Z with
    z-buffered occlusion, so the disparity SGBM recovers is the disparity we
    put in.  That makes it a real test of the pipeline, not a mock.
    """

    def __init__(self, calib, fps: float = 30.0, seed: int = 7,
                 boxes: list[SceneBox] | None = None, background_z: float = 9.0):
        self.calib = calib
        self._w, self._h = calib.image_size
        self._f = float(calib.K1[0, 0])
        self._b = calib.baseline_m
        self._cx = float(calib.K1[0, 2])
        self._cy = float(calib.K1[1, 2])
        self._fps = fps
        self._dt = 1.0 / fps
        self._bg_z = background_z
        self._index = 0
        self._t0 = time.monotonic()
        self._next_due = self._t0
        rng = np.random.default_rng(seed)

        # Strong high-frequency texture, otherwise SGBM has nothing to match
        # and we would be testing the block matcher's failure mode instead.
        noise = rng.integers(0, 255, size=(self._h, self._w), dtype=np.uint8)
        self._texture = cv2.GaussianBlur(noise, (3, 3), 0)

        self.boxes = boxes if boxes is not None else [
            SceneBox(x_m=-0.35, z_m=2.6, width_m=0.45, height_m=0.70, vx_m_s=0.06),
            SceneBox(x_m=0.70, z_m=1.5, width_m=0.35, height_m=0.55, vz_m_s=-0.05),
        ]

    @property
    def eye_size(self) -> tuple[int, int]:
        return (self._w, self._h)

    def depth_scene(self, t: float) -> np.ndarray:
        """Ground-truth depth in metres - the oracle the tests compare against."""
        z = np.full((self._h, self._w), self._bg_z, np.float32)
        vv, uu = np.mgrid[0:self._h, 0:self._w].astype(np.float32)

        # Ground plane: camera at h above it, looking level.  For a pixel below
        # the horizon, Z = f * h / (v - cy).
        h_cam = 0.16
        below = vv - self._cy
        with np.errstate(divide="ignore", invalid="ignore"):
            ground = self._f * h_cam / below
        ground[below <= 1.0] = np.inf
        z = np.minimum(z, np.where(np.isfinite(ground), ground, self._bg_z)).astype(np.float32)

        for box in self.boxes:
            bz = max(0.3, box.z_m + box.vz_m_s * t)
            bx = box.x_m + box.vx_m_s * t
            # Project the box extents through the pinhole model.
            u0 = self._cx + self._f * (bx - box.width_m / 2) / bz
            u1 = self._cx + self._f * (bx + box.width_m / 2) / bz
            v0 = self._cy + self._f * (box.y_m - box.height_m / 2) / bz
            v1 = self._cy + self._f * (box.y_m + box.height_m / 2) / bz
            m = ((uu >= u0) & (uu < u1) & (vv >= v0) & (vv < v1) & (z > bz))
            z[m] = bz
        return z

    def _render_pair(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        z = self.depth_scene(t)
        left = self._texture
        disp = (self._f * self._b / np.maximum(z, 1e-3)).astype(np.float32)

        # Forward-warp left -> right, nearest surface wins.  Sorting by depth
        # descending and scattering in that order makes the nearest write last.
        h, w = z.shape
        uu = np.tile(np.arange(w, dtype=np.float32), (h, 1))
        u_r = np.rint(uu - disp).astype(np.int32)
        vv = np.repeat(np.arange(h, dtype=np.int32)[:, None], w, axis=1)

        valid = (u_r >= 0) & (u_r < w)
        order = np.argsort(-z, axis=1, kind="stable")
        rows = np.repeat(np.arange(h)[:, None], w, axis=1)
        u_r_s, v_s, val_s = u_r[rows, order], vv[rows, order], valid[rows, order]
        src_s = left[rows, order]

        right = np.zeros_like(left)
        filled = np.zeros_like(left, dtype=bool)
        right[v_s[val_s], u_r_s[val_s]] = src_s[val_s]
        filled[v_s[val_s], u_r_s[val_s]] = True
        # Disocclusions: fill with background texture so SGBM sees a plausible
        # (and unmatchable) region rather than black bars.
        right[~filled] = self._texture[~filled]
        return left, right

    def read(self) -> StereoFrame:
        # Pace like a real sensor so the pipeline's frame-drop logic is exercised.
        now = time.monotonic()
        if now < self._next_due:
            time.sleep(self._next_due - now)
        self._next_due += self._dt
        if self._next_due < time.monotonic() - self._dt:
            self._next_due = time.monotonic()

        t = self._index * self._dt
        left, right = self._render_pair(t)
        self._index += 1
        return StereoFrame(left, right, time.monotonic(), self._index)


def open_source(cfg, calib) -> StereoSource:
    if cfg.source == "synthetic":
        return SyntheticStereoCamera(calib, fps=cfg.fps)
    return V4L2StereoCamera(cfg.device, cfg.capture_width, cfg.capture_height,
                            cfg.fps, cfg.fourcc, cfg.swap_eyes)
