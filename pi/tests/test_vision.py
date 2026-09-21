"""Vision tests against a synthetic scene with known ground-truth depth.

The synthetic camera forward-warps a textured image by d = f*B/Z, so the
disparity SGBM recovers is the disparity that was put in.  That makes these
real tests of the geometry, not mocks.
"""

import numpy as np
import pytest

from stereolink.calibration import Rectifier, StereoCalibration
from stereolink.camera import SceneBox, SyntheticStereoCamera
from stereolink.config import ObstacleConfig, StereoConfig
from stereolink.disparity import DisparityEngine
from stereolink.obstacle import ObstacleAnalyzer


@pytest.fixture(scope="module")
def rig():
    calib = StereoCalibration.synthetic(1280, 720)
    rect = Rectifier(calib, (640, 360))
    return calib, rect


def run_scene(calib, rect, boxes, frames=3):
    cam = SyntheticStereoCamera(calib, fps=1000, boxes=boxes)
    engine = DisparityEngine(StereoConfig())
    analyzer = ObstacleAnalyzer(ObstacleConfig(), rect)
    report = None
    for _ in range(frames):
        frame = cam.read()
        import cv2
        left = cv2.resize(frame.left, (640, 360), interpolation=cv2.INTER_AREA)
        right = cv2.resize(frame.right, (640, 360), interpolation=cv2.INTER_AREA)
        lr, rr = rect.rectify(left, right)
        disp = engine.compute(lr, rr)
        depth = rect.depth_from_disparity(np.nan_to_num(disp, nan=0.0))
        report = analyzer.analyze(depth)
    return report


def test_depth_from_disparity_matches_the_pinhole_model(rig):
    _, rect = rig
    disp = np.array([[rect.focal_px * rect.baseline_m / 2.0]], np.float32)
    depth = rect.depth_from_disparity(disp)
    assert depth[0, 0] == pytest.approx(2.0, rel=1e-4)


def test_zero_disparity_is_infinite_not_a_division_error(rig):
    _, rect = rig
    depth = rect.depth_from_disparity(np.array([[0.0, -1.0]], np.float32))
    assert np.isinf(depth).all()


def test_recovers_a_single_obstacle_distance(rig):
    calib, rect = rig
    truth_z = 2.0
    report = run_scene(calib, rect,
                       [SceneBox(x_m=0.0, z_m=truth_z, width_m=0.6, height_m=0.8)])
    assert report.valid
    # 12 cm tolerance at 2 m. The estimate runs slightly short because the
    # analyzer takes a low percentile of each column, which is the safe
    # direction to be wrong in.
    assert report.min_range_m == pytest.approx(truth_z, abs=0.12)
    assert abs(report.min_range_bearing) < 0.25   # roughly straight ahead


def test_obstacle_bearing_tracks_lateral_position(rig):
    calib, rect = rig
    left_report = run_scene(calib, rect,
                            [SceneBox(x_m=-0.8, z_m=2.0, width_m=0.5, height_m=0.8)])
    right_report = run_scene(calib, rect,
                             [SceneBox(x_m=+0.8, z_m=2.0, width_m=0.5, height_m=0.8)])
    assert left_report.min_range_bearing < -0.1
    assert right_report.min_range_bearing > 0.1


def test_nearer_obstacle_wins(rig):
    calib, rect = rig
    report = run_scene(calib, rect, [
        SceneBox(x_m=-0.7, z_m=3.5, width_m=0.5, height_m=0.8),
        SceneBox(x_m=+0.7, z_m=1.5, width_m=0.5, height_m=0.8),
    ])
    assert report.min_range_m == pytest.approx(1.5, abs=0.15)
    assert report.min_range_bearing > 0    # the near one is on the right


def test_planner_steers_into_the_gap(rig):
    calib, rect = rig
    # Obstacles left and right, gap in the middle.
    report = run_scene(calib, rect, [
        SceneBox(x_m=-1.1, z_m=2.0, width_m=0.8, height_m=0.9),
        SceneBox(x_m=+1.1, z_m=2.0, width_m=0.8, height_m=0.9),
    ])
    assert abs(report.best_bearing) < 0.4, \
        f"should aim down the middle, got {report.best_bearing:+.2f}"


def test_empty_scene_is_clear(rig):
    calib, rect = rig
    report = run_scene(calib, rect, [])
    # Only the ground plane is present, and ground rejection must discard it -
    # otherwise the robot would brake for the floor.
    assert not report.blocked
    assert report.min_range_m > 1.5, \
        f"floor was mistaken for an obstacle at {report.min_range_m:.2f} m"


def test_ground_plane_is_rejected_not_treated_as_an_obstacle(rig):
    _, rect = rig
    cfg = ObstacleConfig()
    analyzer = ObstacleAnalyzer(cfg, rect)

    # Build a depth map that is pure ground plane: Z = f*h/(v - cy).
    h, w = 360, 640
    rows = np.arange(h, dtype=np.float32)[:, None]
    dv = rows - rect.cy
    with np.errstate(divide="ignore", invalid="ignore"):
        ground = rect.focal_px * cfg.camera_height_m / dv
    ground = np.where(dv > 1.0, ground, np.inf)
    depth = np.repeat(ground, w, axis=1).astype(np.float32)

    report = analyzer.analyze(depth)
    assert not report.blocked
    assert not np.isfinite(report.min_range_m) or report.min_range_m > 2.0
