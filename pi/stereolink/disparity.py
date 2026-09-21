"""StereoSGBM disparity with the parameters that matter for a 10 Hz budget."""

from __future__ import annotations

import cv2
import numpy as np

_MODES = {
    "SGBM": cv2.STEREO_SGBM_MODE_SGBM,
    "HH": cv2.STEREO_SGBM_MODE_HH,
    "SGBM_3WAY": cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    "HH4": getattr(cv2, "STEREO_SGBM_MODE_HH4", cv2.STEREO_SGBM_MODE_HH),
}


class DisparityEngine:
    """Wraps StereoSGBM and (optionally) the contrib WLS filter.

    SGBM emits fixed-point disparity scaled by 16; everything downstream wants
    float pixels, so the conversion happens here exactly once.
    """

    SCALE = 16.0

    def __init__(self, cfg, channels: int = 1):
        if cfg.num_disparities % 16:
            raise ValueError("num_disparities must be a multiple of 16")
        block = cfg.block_size | 1  # SGBM requires odd
        # P1/P2 are the smoothness penalties.  The OpenCV-recommended scaling
        # with block size is what keeps thin obstacles from being smoothed away.
        p1 = cfg.p1_factor * channels * block * block
        p2 = cfg.p2_factor * channels * block * block

        self.matcher = cv2.StereoSGBM_create(
            minDisparity=cfg.min_disparity,
            numDisparities=cfg.num_disparities,
            blockSize=block,
            P1=p1, P2=p2,
            disp12MaxDiff=cfg.disp12_max_diff,
            uniquenessRatio=cfg.uniqueness_ratio,
            speckleWindowSize=cfg.speckle_window_size,
            speckleRange=cfg.speckle_range,
            preFilterCap=cfg.pre_filter_cap,
            mode=_MODES.get(cfg.mode, cv2.STEREO_SGBM_MODE_SGBM_3WAY),
        )
        self.cfg = cfg
        self._wls = None
        self._right_matcher = None
        if cfg.wls_filter:
            self._init_wls()

    def _init_wls(self) -> None:
        ximgproc = getattr(cv2, "ximgproc", None)
        if ximgproc is None:
            print("[disparity] wls_filter requested but opencv-contrib is not "
                  "installed; continuing unfiltered")
            return
        self._right_matcher = ximgproc.createRightMatcher(self.matcher)
        self._wls = ximgproc.createDisparityWLSFilter(self.matcher)
        self._wls.setLambda(8000.0)
        self._wls.setSigmaColor(1.5)

    @staticmethod
    def to_gray(img: np.ndarray) -> np.ndarray:
        if img.ndim == 3:
            return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return img

    def compute(self, left_rect: np.ndarray, right_rect: np.ndarray) -> np.ndarray:
        """Return float32 disparity in pixels; non-matches are NaN."""
        lg = self.to_gray(left_rect)
        rg = self.to_gray(right_rect)
        raw = self.matcher.compute(lg, rg)

        if self._wls is not None and self._right_matcher is not None:
            raw_r = self._right_matcher.compute(rg, lg)
            raw = self._wls.filter(raw, lg, disparity_map_right=raw_r)

        disp = raw.astype(np.float32) / self.SCALE
        # SGBM marks "no match" with minDisparity-1 (scaled); anything at or
        # below the search floor is not a measurement.
        disp[raw <= (self.cfg.min_disparity - 1) * self.SCALE] = np.nan
        disp[disp <= 0.0] = np.nan
        return disp

    @staticmethod
    def colorize(disp: np.ndarray, num_disparities: int) -> np.ndarray:
        """8-bit colour map for the preview window / north-bound thumbnails."""
        vis = np.nan_to_num(disp, nan=0.0)
        vis = np.clip(vis / max(num_disparities, 1) * 255.0, 0, 255).astype(np.uint8)
        return cv2.applyColorMap(vis, cv2.COLORMAP_TURBO)
