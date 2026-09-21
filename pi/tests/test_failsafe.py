"""Failsafe tests: what the node does when the link misbehaves.

These run the behavioural node model (tools/esp32_sim.py) over the in-process
CAN bus.  They verify the *logic* of the failsafe; the firmware's real timing
is measured on hardware and reported in docs/jitter-report.md.
"""

import time

import pytest

from esp32_sim import Esp32NodeSim
from stereolink.canlink import CanOpenMaster, LoopbackTransport
from stereolink.config import CanConfig
from stereolink.pdo import (
    DriveState, FaultFlags, LimitsPdo, SetpointFlags, SetpointPdo,
)

WATCHDOG_MS = 100
DECEL = 1200.0


def wait_until(predicate, timeout=4.0, interval=0.005):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


@pytest.fixture
def link():
    bus = LoopbackTransport()
    cfg = CanConfig(interface="loopback", send_sync=False)
    master = CanOpenMaster(cfg, bus.endpoint("master"))
    node = Esp32NodeSim(bus.endpoint("node"), node_id=cfg.node_id,
                        watchdog_ms=WATCHDOG_MS, decel_mm_s2=DECEL)
    master.start()
    node.start()
    master.start_node()
    try:
        yield bus, master, node
    finally:
        node.stop()
        master.stop()


def drive_at(master, node, v_mm_s, cycles=12, period=0.02):
    """Hold a setpoint until the node is actually moving at it."""
    for _ in range(cycles):
        master.send_setpoint(SetpointPdo(v_mm_s, 0, 200, master.next_seq(),
                                         SetpointFlags.VISION_VALID))
        time.sleep(period)


def test_node_runs_while_setpoints_arrive(link):
    _, master, node = link
    drive_at(master, node, 600)
    assert node.state == DriveState.RUN
    assert node.v_meas > 0
    assert not (node.faults & FaultFlags.LINK_LOSS)


def test_watchdog_fires_after_the_configured_timeout(link):
    bus, master, node = link
    drive_at(master, node, 600)

    bus.cut("master")
    # Measure from the last setpoint the node actually RECEIVED, not from the
    # cut: the cut lands somewhere inside a send period, so those two instants
    # differ by up to one period and only the former is what the watchdog times.
    t_last_rpdo = node._t_last_rpdo

    assert wait_until(lambda: node.state == DriveState.DEGRADED), \
        "node never noticed the link was gone"
    age_at_trip_ms = (time.monotonic() - t_last_rpdo) * 1000.0

    # It must fire after the timeout (not prematurely on a single late frame)
    # and within a reasonable margin of it.
    assert age_at_trip_ms >= WATCHDOG_MS, \
        f"fired early, at {age_at_trip_ms:.0f} ms of setpoint age"
    assert age_at_trip_ms < WATCHDOG_MS + 120, \
        f"fired late, at {age_at_trip_ms:.0f} ms of setpoint age"
    assert node.faults & FaultFlags.LINK_LOSS


def test_motors_ramp_to_zero_and_latch_safe_stop(link):
    bus, master, node = link
    drive_at(master, node, 600)
    v_before = node.v_meas
    assert v_before > 100

    bus.cut("master")
    assert wait_until(lambda: node.state == DriveState.SAFE_STOP, timeout=5.0), \
        "node did not reach SAFE_STOP"
    assert node.v_meas == 0
    assert not node.ir_mask


def test_the_stop_is_a_ramp_not_an_instant_zero(link):
    """A step to zero would be a torque step through the drivetrain."""
    bus, master, node = link
    drive_at(master, node, 900)
    v_before = node.v_meas

    bus.cut("master")
    wait_until(lambda: node.state == DriveState.DEGRADED)

    # Sample during the ramp: speed must pass through intermediate values.
    seen = []
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and node.v_meas > 0:
        seen.append(node.v_meas)
        time.sleep(0.02)

    intermediate = [v for v in seen if 0 < v < v_before * 0.95]
    assert len(intermediate) >= 3, \
        f"speed jumped to zero instead of ramping: {seen[:10]}"
    # And it must be monotonically decreasing.
    assert all(b <= a + 1 for a, b in zip(seen, seen[1:])), \
        f"ramp was not monotonic: {seen[:10]}"


def test_ramp_duration_matches_the_configured_deceleration(link):
    bus, master, node = link
    drive_at(master, node, 600)
    v_before = node.v_meas

    bus.cut("master")
    t_cut = time.monotonic()
    assert wait_until(lambda: node.v_meas == 0, timeout=5.0)
    total_ms = (time.monotonic() - t_cut) * 1000.0

    expected_ms = WATCHDOG_MS + (v_before / DECEL) * 1000.0
    # Generous bound: this is a Python model on a non-realtime OS, so the
    # assertion is that the physics is right, not that the timing is tight.
    assert total_ms == pytest.approx(expected_ms, rel=0.6), \
        f"stopped in {total_ms:.0f} ms, expected ~{expected_ms:.0f} ms"


def test_safe_stop_is_latched_and_does_not_self_recover(link):
    """The robot must not restart by itself when a cable is reseated."""
    bus, master, node = link
    drive_at(master, node, 600)
    bus.cut("master")
    assert wait_until(lambda: node.state == DriveState.SAFE_STOP, timeout=5.0)

    bus.restore()
    drive_at(master, node, 600, cycles=15)

    assert node.state == DriveState.SAFE_STOP, \
        "node resumed driving on its own after a link loss"
    assert node.v_meas == 0


def test_explicit_fault_clear_releases_the_latch(link):
    bus, master, node = link
    drive_at(master, node, 600)
    bus.cut("master")
    assert wait_until(lambda: node.state == DriveState.SAFE_STOP, timeout=5.0)
    bus.restore()

    master.send_setpoint(SetpointPdo(0, 0, 0, master.next_seq(),
                                     SetpointFlags.CLEAR_FAULT))
    assert wait_until(lambda: node.state != DriveState.SAFE_STOP, timeout=1.0)
    drive_at(master, node, 500)
    assert node.state == DriveState.RUN
    assert node.v_meas > 0


def test_emergency_object_is_emitted_once_per_transition(link):
    bus, master, node = link
    drive_at(master, node, 600)
    bus.cut("master")
    assert wait_until(lambda: master.node.emcy_count >= 1, timeout=5.0)
    count = master.node.emcy_count
    time.sleep(0.4)
    assert master.node.emcy_count == count, "EMCY is being spammed"
    assert master.node.last_emcy.state == DriveState.SAFE_STOP


def test_ir_layer_stops_the_drive_without_any_link_involvement(link):
    """The fast safety layer must not depend on the Pi or the bus."""
    _, master, node = link
    drive_at(master, node, 700)
    assert node.v_meas > 0

    node.trip_ir(0x02)   # centre sensor
    # Setpoints keep arriving and keep asking for speed; IR must still win.
    drive_at(master, node, 700, cycles=20)

    assert node.v_meas == 0, "IR obstacle did not stop the drive"
    assert node.faults & FaultFlags.IR_BLOCKED


def test_node_ignores_setpoints_before_nmt_start():
    """A PDO outside Operational is not a valid command."""
    bus = LoopbackTransport()
    cfg = CanConfig(interface="loopback", send_sync=False)
    master = CanOpenMaster(cfg, bus.endpoint("master"))
    node = Esp32NodeSim(bus.endpoint("node"), node_id=cfg.node_id,
                        watchdog_ms=WATCHDOG_MS)
    master.start()
    node.start()
    try:
        # Deliberately no start_node().
        for _ in range(10):
            master.send_setpoint(SetpointPdo(800, 0, 300, master.next_seq(),
                                             SetpointFlags.VISION_VALID))
            time.sleep(0.02)
        assert node.v_meas == 0
        assert node.state != DriveState.RUN
    finally:
        node.stop()
        master.stop()


def test_watchdog_cannot_be_widened_over_the_link(link):
    """A safety timeout that the bus can relax is not a safety function."""
    _, master, node = link
    master.send_limits(LimitsPdo(v_max_mm_s=1000, decel_mm_s2=1200,
                                 watchdog_ms=900, ir_stop_cm=40))
    time.sleep(0.1)
    assert node.watchdog_ms <= WATCHDOG_MS, \
        f"node accepted a wider watchdog: {node.watchdog_ms} ms"


def test_watchdog_can_be_tightened_over_the_link(link):
    _, master, node = link
    master.send_limits(LimitsPdo(v_max_mm_s=1000, decel_mm_s2=1200,
                                 watchdog_ms=50, ir_stop_cm=40))
    time.sleep(0.1)
    assert node.watchdog_ms == 50
