"""Wire-format tests.

These pin the byte layout.  ``esp32/host_test/test_firmware_logic.c`` decodes
the same literal bytes on the C side, so if either end drifts, one of the two
suites fails.
"""

import pytest

from stereolink.pdo import (
    DiagnosticsPdo, DriveState, EmcyPdo, FaultFlags, LatencyEchoPdo, LimitsPdo,
    NmtState, SetpointFlags, SetpointPdo, StatusPdo, cob, COB_RPDO1_BASE,
    COB_TPDO1_BASE, V_MAX_MM_S, W_MAX_MRAD_S,
)

ALL_PDOS = [
    SetpointPdo(500, -250, 142, 7, SetpointFlags.VISION_VALID),
    LimitsPdo(1200, 900, 150, 25),
    StatusPdo(-300, 40, DriveState.DEGRADED, FaultFlags.LINK_LOSS, 7, 0x02),
    DiagnosticsPdo(12, 1010, 31, 44, 9),
    LatencyEchoPdo(250, 7, 1, 10),
    EmcyPdo(0x8130, 0x81, DriveState.SAFE_STOP, FaultFlags.LINK_LOSS),
]


@pytest.mark.parametrize("pdo", ALL_PDOS, ids=lambda p: type(p).__name__)
def test_every_pdo_is_exactly_eight_bytes(pdo):
    # CANopen PDOs are capped at 8 bytes; anything longer would be silently
    # truncated by the transport.
    assert len(pdo.encode()) == 8


@pytest.mark.parametrize("pdo", ALL_PDOS, ids=lambda p: type(p).__name__)
def test_encode_decode_roundtrip(pdo):
    assert type(pdo).decode(pdo.encode()) == pdo


def test_setpoint_wire_bytes_are_pinned():
    # This exact literal is decoded by the C test too. Do not "fix" it without
    # changing esp32/main/pdo_defs.h and the C test in the same commit.
    assert SetpointPdo(500, -250, 142, 7, SetpointFlags.VISION_VALID).encode() \
        == bytes.fromhex("f40106ff8e000701")


def test_negative_velocity_survives_the_wire():
    decoded = SetpointPdo.decode(SetpointPdo(-1200, -3000, 0, 3).encode())
    assert decoded.v_mm_s == -1200
    assert decoded.w_mrad_s == -3000


def test_setpoint_saturates_instead_of_overflowing():
    # An int16 would wrap silently; wrapping a velocity command means full
    # speed in the wrong direction, so clamping is the only safe behaviour.
    decoded = SetpointPdo.decode(SetpointPdo(99999, -99999, 0, 1).encode())
    assert decoded.v_mm_s == V_MAX_MM_S
    assert decoded.w_mrad_s == -W_MAX_MRAD_S


def test_short_frame_is_padded_not_rejected():
    # A truncated frame mid-flight must not raise inside the RX thread.
    sp = SetpointPdo.decode(b"\xf4\x01")
    assert sp.v_mm_s == 500
    assert sp.w_mrad_s == 0


def test_cob_ids_follow_cia301():
    assert cob(COB_RPDO1_BASE, 0x22) == 0x222
    assert cob(COB_TPDO1_BASE, 0x22) == 0x1A2


def test_limits_watchdog_is_clamped_to_a_sane_range():
    # A zero watchdog would disable the failsafe; the encoder refuses it.
    assert LimitsPdo.decode(LimitsPdo(500, 900, 0, 25).encode()).watchdog_ms == 20
    assert LimitsPdo.decode(LimitsPdo(500, 900, 99999, 25).encode()).watchdog_ms == 1000


def test_fault_flags_compose_and_survive():
    faults = FaultFlags.LINK_LOSS | FaultFlags.IR_BLOCKED
    st = StatusPdo.decode(StatusPdo(0, 0, DriveState.SAFE_STOP, faults, 0, 0).encode())
    assert st.faults is faults
    assert FaultFlags.LINK_LOSS in st.faults


def test_nmt_states_match_cia301_values():
    assert NmtState.OPERATIONAL == 0x05
    assert NmtState.PRE_OPERATIONAL == 0x7F
    assert NmtState.STOPPED == 0x04
