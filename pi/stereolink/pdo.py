"""CANopen process-data objects for the vision -> control link.

This module is the single normative definition of the wire format on the Pi
side.  ``esp32/main/pdo_defs.h`` mirrors it byte for byte; ``tests/test_pdo.py``
pins the layout so the two cannot drift apart silently.

Every PDO is exactly 8 bytes and little-endian, which is what CANopen mandates
and what lets the ESP32 memcpy straight into a packed struct.

COB-ID map (node id N, default 0x22)
------------------------------------
    0x000           NMT node control          master -> node
    0x080           SYNC                      master -> node
    0x080 + N       EMCY (emergency)          node -> master
    0x180 + N       TPDO1  Status             node -> master   50 Hz
    0x200 + N       RPDO1  Setpoint           master -> node   10 Hz
    0x280 + N       TPDO2  Diagnostics        node -> master    5 Hz
    0x300 + N       RPDO2  Limits/config      master -> node   on change
    0x380 + N       TPDO3  Latency echo       node -> master   per RPDO1
    0x580 + N       SDO tx (server -> client) node -> master
    0x600 + N       SDO rx (client -> server) master -> node
    0x700 + N       Heartbeat                 node -> master    5 Hz
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum, IntFlag

DEFAULT_NODE_ID = 0x22

# --- COB-ID bases ----------------------------------------------------------
COB_NMT = 0x000
COB_SYNC = 0x080
COB_EMCY_BASE = 0x080
COB_TPDO1_BASE = 0x180
COB_RPDO1_BASE = 0x200
COB_TPDO2_BASE = 0x280
COB_RPDO2_BASE = 0x300
COB_TPDO3_BASE = 0x380
COB_SDO_TX_BASE = 0x580
COB_SDO_RX_BASE = 0x600
COB_HEARTBEAT_BASE = 0x700


def cob(base: int, node_id: int) -> int:
    return base + node_id


class NmtCommand(IntEnum):
    START = 0x01          # -> Operational
    STOP = 0x02           # -> Stopped
    ENTER_PREOP = 0x80
    RESET_NODE = 0x81
    RESET_COMM = 0x82


class NmtState(IntEnum):
    BOOTUP = 0x00
    STOPPED = 0x04
    OPERATIONAL = 0x05
    PRE_OPERATIONAL = 0x7F


class DriveState(IntEnum):
    """Firmware state machine, reported in TPDO1 byte 4."""

    INIT = 0
    IDLE = 1
    RUN = 2
    DEGRADED = 3     # link lost, actively ramping to zero
    SAFE_STOP = 4    # ramp finished, outputs disabled, latched
    FAULT = 5


class FaultFlags(IntFlag):
    NONE = 0x00
    LINK_LOSS = 0x01        # RPDO1 watchdog expired
    IR_BLOCKED = 0x02       # fast safety layer tripped
    OVERCURRENT = 0x04
    ENCODER_STALL = 0x08
    ESTOP = 0x10
    CAN_BUS_OFF = 0x20
    SETPOINT_RANGE = 0x40   # setpoint clamped (rejected as out of range)
    CONTROL_OVERRUN = 0x80  # control loop missed its deadline


class SetpointFlags(IntFlag):
    NONE = 0x00
    VISION_VALID = 0x01     # disparity solution trusted this cycle
    ESTOP_REQUEST = 0x02    # operator / north-bound stop
    OBSTACLE_NEAR = 0x04    # planner already de-rated for an obstacle
    CLEAR_FAULT = 0x08      # rising edge clears a latched SAFE_STOP


# --- limits used for clamping / saturation detection -----------------------
V_MAX_MM_S = 1500
W_MAX_MRAD_S = 4000
OBSTACLE_MAX_CM = 65535


def _clamp(value: float, lo: int, hi: int) -> tuple[int, bool]:
    iv = int(round(value))
    if iv < lo:
        return lo, True
    if iv > hi:
        return hi, True
    return iv, False


@dataclass(frozen=True)
class SetpointPdo:
    """RPDO1 - the command the ESP32 closes its loop around.

    ``seq`` is the latency token: the Pi records the frame-capture timestamp
    against it and the ESP32 echoes it back in TPDO1/TPDO3, which closes the
    end-to-end measurement without a synchronised clock.
    """

    v_mm_s: int
    w_mrad_s: int
    obstacle_cm: int
    seq: int
    flags: SetpointFlags = SetpointFlags.NONE

    STRUCT = struct.Struct("<hhHBB")
    SIZE = 8

    def encode(self) -> bytes:
        v, v_sat = _clamp(self.v_mm_s, -V_MAX_MM_S, V_MAX_MM_S)
        w, w_sat = _clamp(self.w_mrad_s, -W_MAX_MRAD_S, W_MAX_MRAD_S)
        obstacle, _ = _clamp(self.obstacle_cm, 0, OBSTACLE_MAX_CM)
        flags = self.flags
        if v_sat or w_sat:
            # The node also range-checks, but flagging here makes the
            # saturation visible in the north-bound telemetry too.
            flags = flags | SetpointFlags.OBSTACLE_NEAR
        return self.STRUCT.pack(v, w, obstacle, self.seq & 0xFF, int(flags) & 0xFF)

    @classmethod
    def decode(cls, data: bytes) -> "SetpointPdo":
        v, w, obstacle, seq, flags = cls.STRUCT.unpack(_exact(data, cls.SIZE))
        return cls(v, w, obstacle, seq, SetpointFlags(flags))


@dataclass(frozen=True)
class LimitsPdo:
    """RPDO2 - runtime limits.  Sent on change and after every node bootup.

    Note that ``watchdog_ms`` only *tightens* the firmware default; the node
    refuses a value that would disable the watchdog.  A safety timeout must not
    be removable over the very link it protects.
    """

    v_max_mm_s: int
    decel_mm_s2: int
    watchdog_ms: int
    ir_stop_cm: int
    reserved: int = 0

    STRUCT = struct.Struct("<HHHBB")
    SIZE = 8

    def encode(self) -> bytes:
        return self.STRUCT.pack(
            _clamp(self.v_max_mm_s, 0, V_MAX_MM_S)[0],
            _clamp(self.decel_mm_s2, 100, 65535)[0],
            _clamp(self.watchdog_ms, 20, 1000)[0],
            _clamp(self.ir_stop_cm, 1, 255)[0],
            self.reserved & 0xFF,
        )

    @classmethod
    def decode(cls, data: bytes) -> "LimitsPdo":
        return cls(*cls.STRUCT.unpack(_exact(data, cls.SIZE)))


@dataclass(frozen=True)
class StatusPdo:
    """TPDO1 - what the drive is actually doing."""

    v_meas_mm_s: int
    w_meas_mrad_s: int
    state: DriveState
    faults: FaultFlags
    seq_echo: int
    ir_mask: int

    STRUCT = struct.Struct("<hhBBBB")
    SIZE = 8

    def encode(self) -> bytes:
        return self.STRUCT.pack(
            self.v_meas_mm_s, self.w_meas_mrad_s,
            int(self.state) & 0xFF, int(self.faults) & 0xFF,
            self.seq_echo & 0xFF, self.ir_mask & 0xFF,
        )

    @classmethod
    def decode(cls, data: bytes) -> "StatusPdo":
        v, w, state, faults, seq, ir = cls.STRUCT.unpack(_exact(data, cls.SIZE))
        return cls(v, w, DriveState(state), FaultFlags(faults), seq, ir)


@dataclass(frozen=True)
class DiagnosticsPdo:
    """TPDO2 - the evidence for the priority-separation claim.

    ``jitter_p99_us`` and ``period_max_us`` are measured inside the 1 kHz
    control task itself, so they hold up while the comms tasks are being
    hammered by the load generator.
    """

    jitter_p99_us: int
    period_max_us: int
    rpdo_age_ms: int
    ctrl_cpu_pct: int
    comms_cpu_pct: int

    STRUCT = struct.Struct("<HHHBB")
    SIZE = 8

    def encode(self) -> bytes:
        return self.STRUCT.pack(
            min(self.jitter_p99_us, 0xFFFF), min(self.period_max_us, 0xFFFF),
            min(self.rpdo_age_ms, 0xFFFF),
            min(self.ctrl_cpu_pct, 0xFF), min(self.comms_cpu_pct, 0xFF),
        )

    @classmethod
    def decode(cls, data: bytes) -> "DiagnosticsPdo":
        return cls(*cls.STRUCT.unpack(_exact(data, cls.SIZE)))


@dataclass(frozen=True)
class LatencyEchoPdo:
    """TPDO3 - emitted the moment a setpoint is consumed by the control loop.

    ``rx_to_apply_us`` is the node-local span from TWAI receive interrupt to the
    PWM register write, which is the one stage the Pi cannot time for itself.
    """

    rx_to_apply_us: int
    seq_echo: int
    watchdog_misses: int
    rpdo_rate_hz: int
    reserved: int = 0

    STRUCT = struct.Struct("<IBBBB")
    SIZE = 8

    def encode(self) -> bytes:
        return self.STRUCT.pack(
            min(self.rx_to_apply_us, 0xFFFFFFFF), self.seq_echo & 0xFF,
            min(self.watchdog_misses, 0xFF), min(self.rpdo_rate_hz, 0xFF),
            self.reserved & 0xFF,
        )

    @classmethod
    def decode(cls, data: bytes) -> "LatencyEchoPdo":
        return cls(*cls.STRUCT.unpack(_exact(data, cls.SIZE)))


@dataclass(frozen=True)
class EmcyPdo:
    """Emergency object - sent once on every transition into SAFE_STOP/FAULT."""

    error_code: int
    error_register: int
    state: DriveState
    faults: FaultFlags

    STRUCT = struct.Struct("<HBBBBBB")
    SIZE = 8

    # CiA 301 / DS402-flavoured codes for the conditions we can actually hit.
    CODE_LINK_LOSS = 0x8130      # life guard / heartbeat error
    CODE_IR_BLOCKED = 0xFF01     # manufacturer specific: proximity stop
    CODE_CONTROL_OVERRUN = 0xFF02
    CODE_BUS_OFF = 0x8140

    def encode(self) -> bytes:
        return self.STRUCT.pack(
            self.error_code & 0xFFFF, self.error_register & 0xFF,
            int(self.state) & 0xFF, int(self.faults) & 0xFF, 0, 0, 0,
        )

    @classmethod
    def decode(cls, data: bytes) -> "EmcyPdo":
        code, reg, state, faults, *_ = cls.STRUCT.unpack(_exact(data, cls.SIZE))
        return cls(code, reg, DriveState(state), FaultFlags(faults))


def _exact(data: bytes, size: int) -> bytes:
    """CANopen allows a short frame; pad it rather than raising mid-control."""
    if len(data) == size:
        return data
    if len(data) < size:
        return bytes(data) + bytes(size - len(data))
    return bytes(data[:size])
