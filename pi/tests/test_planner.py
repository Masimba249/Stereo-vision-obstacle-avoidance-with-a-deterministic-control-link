"""Planner tests: the speed/steering policy, in isolation from the vision."""

import numpy as np
import pytest

from stereolink.config import PlannerConfig
from stereolink.obstacle import ObstacleReport
from stereolink.pdo import SetpointFlags
from stereolink.planner import AvoidancePlanner


def make_report(min_range_m, best_bearing=0.0, valid_fraction=0.8,
                blocked=False, min_range_bearing=0.0, n=16):
    return ObstacleReport(
        column_range_m=np.full(n, min_range_m, np.float32),
        column_bearing=np.linspace(-1, 1, n).astype(np.float32),
        min_range_m=min_range_m, min_range_bearing=min_range_bearing,
        valid_fraction=valid_fraction, best_bearing=best_bearing,
        blocked=blocked)


@pytest.fixture
def planner():
    return AvoidancePlanner(PlannerConfig())


def test_cruises_when_the_path_is_clear(planner):
    cfg = PlannerConfig()
    sp = planner.plan(make_report(float("inf")), now=0.0)
    assert sp.v_mm_s == pytest.approx(cfg.v_cruise_mm_s)
    assert sp.reason == "cruise"
    assert SetpointFlags.VISION_VALID in sp.flags


def test_stops_inside_the_stop_distance(planner):
    cfg = PlannerConfig()
    sp = planner.plan(make_report(cfg.stop_distance_m - 0.1), now=0.0)
    assert sp.v_mm_s == 0.0
    assert sp.reason in ("stop", "turn_in_place")


def test_speed_decreases_monotonically_as_the_obstacle_nears():
    cfg = PlannerConfig()
    speeds = []
    for i, rng in enumerate(np.linspace(cfg.slow_distance_m, cfg.stop_distance_m, 12)):
        p = AvoidancePlanner(cfg)   # fresh, to isolate from slew state
        speeds.append(p.plan(make_report(float(rng)), now=0.0).v_mm_s)
    assert all(b <= a + 1e-6 for a, b in zip(speeds, speeds[1:])), speeds
    assert speeds[0] > speeds[-1]


def test_steers_away_from_the_obstacle_side(planner):
    # best_bearing positive (gap on the right) should produce a right turn.
    # Sign convention: positive omega is a left turn, so a gap on the right
    # must give negative omega.
    sp = planner.plan(make_report(3.0, best_bearing=+0.8), now=0.0)
    assert sp.w_mrad_s < 0
    planner.reset()
    sp = planner.plan(make_report(3.0, best_bearing=-0.8), now=0.0)
    assert sp.w_mrad_s > 0


def test_yaw_is_slew_limited_across_cycles():
    cfg = PlannerConfig()
    p = AvoidancePlanner(cfg)
    # A hard swing from one extreme to the other in one 100 ms cycle must not
    # be passed through: a single bad disparity frame would snap the wheels.
    p.plan(make_report(3.0, best_bearing=-1.0), now=0.0)
    first = p._w_prev
    sp = p.plan(make_report(3.0, best_bearing=+1.0), now=0.1)
    max_change = cfg.steer_slew_mrad_s2 * 0.1
    assert abs(sp.w_mrad_s - first) <= max_change + 1e-6


def test_estop_overrides_everything(planner):
    sp = planner.plan(make_report(float("inf")), now=0.0, estop=True)
    assert sp.v_mm_s == 0.0
    assert sp.w_mrad_s == 0.0
    assert SetpointFlags.ESTOP_REQUEST in sp.flags


def test_invalid_vision_creeps_then_stops():
    cfg = PlannerConfig()
    p = AvoidancePlanner(cfg)
    p.plan(make_report(3.0), now=0.0)                       # establish validity

    # Inside the grace window: creep, and do not steer on stale geometry.
    sp = p.plan(make_report(3.0, valid_fraction=0.01), now=0.1)
    assert sp.v_mm_s <= cfg.v_min_mm_s
    assert sp.w_mrad_s == 0.0
    assert sp.reason == "vision_grace"

    # Past the timeout: full stop.
    sp = p.plan(make_report(3.0, valid_fraction=0.01),
                now=0.1 + cfg.vision_timeout_s + 0.05)
    assert sp.v_mm_s == 0.0
    assert sp.reason == "vision_stale"


def test_vision_valid_flag_reflects_reality(planner):
    assert SetpointFlags.VISION_VALID in planner.plan(make_report(3.0), now=0.0).flags
    planner.reset()
    sp = planner.plan(make_report(3.0, valid_fraction=0.0), now=0.0)
    assert SetpointFlags.VISION_VALID not in sp.flags


def test_obstacle_distance_is_reported_in_centimetres(planner):
    sp = planner.plan(make_report(1.23), now=0.0)
    assert sp.obstacle_cm == 123
    planner.reset()
    # Infinity has no integer encoding; it saturates rather than overflowing.
    sp = planner.plan(make_report(float("inf")), now=0.0)
    assert sp.obstacle_cm == 65535


def test_blocked_scene_turns_in_place(planner):
    sp = planner.plan(make_report(0.3, best_bearing=0.5, blocked=True), now=0.0)
    assert sp.v_mm_s == 0.0
    assert abs(sp.w_mrad_s) > 0, "should rotate to look for a way out"
