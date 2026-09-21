#!/usr/bin/env python3
"""Verify that the C and Python definitions of the wire format still agree.

The Pi and the ESP32 keep two independent descriptions of the same bytes.  That
is unavoidable (different languages) but it is exactly the kind of duplication
that rots silently: a field widened on one side and not the other produces a
robot that drives at a plausible-looking wrong speed rather than an error.

This parses esp32/main/pdo_defs.h and compares it against the Python structs.
Run it in CI alongside the tests.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "pi"))

from stereolink import pdo as P  # noqa: E402

HEADER = ROOT / "esp32" / "main" / "pdo_defs.h"

# C type -> (struct format char, size)
C_TYPES = {
    "int8_t": ("b", 1), "uint8_t": ("B", 1),
    "int16_t": ("h", 2), "uint16_t": ("H", 2),
    "int32_t": ("i", 4), "uint32_t": ("I", 4),
}

# C struct name -> Python class
STRUCT_MAP = {
    "pdo_setpoint_t": P.SetpointPdo,
    "pdo_limits_t": P.LimitsPdo,
    "pdo_status_t": P.StatusPdo,
    "pdo_diag_t": P.DiagnosticsPdo,
    "pdo_latency_echo_t": P.LatencyEchoPdo,
    "pdo_emcy_t": P.EmcyPdo,
}

# Constants that must hold the same value on both sides.
CONST_MAP = {
    "COB_TPDO1_BASE": P.COB_TPDO1_BASE,
    "COB_RPDO1_BASE": P.COB_RPDO1_BASE,
    "COB_TPDO2_BASE": P.COB_TPDO2_BASE,
    "COB_RPDO2_BASE": P.COB_RPDO2_BASE,
    "COB_TPDO3_BASE": P.COB_TPDO3_BASE,
    "COB_HEARTBEAT_BASE": P.COB_HEARTBEAT_BASE,
    "COB_NMT": P.COB_NMT,
    "COB_SYNC": P.COB_SYNC,
    "NMT_CMD_START": P.NmtCommand.START,
    "NMT_CMD_STOP": P.NmtCommand.STOP,
    "NMT_CMD_RESET_NODE": P.NmtCommand.RESET_NODE,
    "NMT_STATE_OPERATIONAL": P.NmtState.OPERATIONAL,
    "NMT_STATE_PREOP": P.NmtState.PRE_OPERATIONAL,
    "NMT_STATE_STOPPED": P.NmtState.STOPPED,
    "DRIVE_RUN": P.DriveState.RUN,
    "DRIVE_DEGRADED": P.DriveState.DEGRADED,
    "DRIVE_SAFE_STOP": P.DriveState.SAFE_STOP,
    "FAULT_LINK_LOSS": P.FaultFlags.LINK_LOSS,
    "FAULT_IR_BLOCKED": P.FaultFlags.IR_BLOCKED,
    "SP_FLAG_VISION_VALID": P.SetpointFlags.VISION_VALID,
    "SP_FLAG_ESTOP_REQUEST": P.SetpointFlags.ESTOP_REQUEST,
    "SP_FLAG_CLEAR_FAULT": P.SetpointFlags.CLEAR_FAULT,
    "EMCY_LINK_LOSS": P.EmcyPdo.CODE_LINK_LOSS,
    "EMCY_IR_BLOCKED": P.EmcyPdo.CODE_IR_BLOCKED,
}


def parse_structs(text: str) -> dict[str, str]:
    """Return {c_struct_name: struct-format-string}."""
    out: dict[str, str] = {}
    pattern = re.compile(r"typedef\s+struct\s*\{(.*?)\}\s*(\w+)\s*;", re.S)
    for body, name in pattern.findall(text):
        fmt = "<"
        for line in body.splitlines():
            line = re.sub(r"/\*.*?\*/", "", line).strip()
            if not line or line.startswith("//") or line.startswith("*"):
                continue
            m = re.match(r"(\w+)\s+(\w+)\s*(?:\[(\d+)\])?\s*;", line)
            if not m:
                continue
            ctype, _field, count = m.groups()
            if ctype not in C_TYPES:
                continue
            ch, _size = C_TYPES[ctype]
            fmt += ch * (int(count) if count else 1)
        out[name] = fmt
    return out


def parse_defines(text: str) -> dict[str, int]:
    out: dict[str, int] = {}
    for name, value in re.findall(r"#define\s+(\w+)\s+(0[xX][0-9a-fA-F]+|\d+)u?", text):
        out[name] = int(value, 0)
    # enum values, e.g. DRIVE_RUN = 2,
    for name, value in re.findall(r"(DRIVE_\w+)\s*=\s*(\d+)", text):
        out[name] = int(value)
    return out


def main() -> int:
    if not HEADER.exists():
        print(f"ERROR: {HEADER} not found")
        return 2
    text = HEADER.read_text()
    structs = parse_structs(text)
    defines = parse_defines(text)
    failures: list[str] = []

    print(f"checking {HEADER.relative_to(ROOT)} against stereolink.pdo\n")

    for c_name, py_cls in STRUCT_MAP.items():
        c_fmt = structs.get(c_name)
        if c_fmt is None:
            failures.append(f"{c_name}: not found in the C header")
            continue
        py_fmt = py_cls.STRUCT.format
        py_fmt = py_fmt if isinstance(py_fmt, str) else py_fmt.decode()
        # Compare the normalised field sequence, not the literal string.
        if c_fmt != py_fmt:
            failures.append(
                f"{c_name}: C says {c_fmt!r}, {py_cls.__name__} says {py_fmt!r}")
            status = "MISMATCH"
        else:
            status = "ok"
        size = py_cls.STRUCT.size
        if size != 8:
            failures.append(f"{c_name}: {size} bytes, CANopen PDOs must be 8")
        print(f"  {status:8} {c_name:22} {c_fmt:10} {size} bytes")

    print()
    for c_name, py_value in CONST_MAP.items():
        c_value = defines.get(c_name)
        if c_value is None:
            failures.append(f"{c_name}: not defined in the C header")
            print(f"  MISSING  {c_name}")
            continue
        if c_value != int(py_value):
            failures.append(
                f"{c_name}: C=0x{c_value:X}, Python=0x{int(py_value):X}")
            print(f"  MISMATCH {c_name:24} C=0x{c_value:02X} "
                  f"py=0x{int(py_value):02X}")
        else:
            print(f"  ok       {c_name:24} 0x{c_value:02X}")

    print()
    if failures:
        print(f"PROTOCOL OUT OF SYNC - {len(failures)} problem(s):")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("protocol definitions are in sync")
    return 0


if __name__ == "__main__":
    sys.exit(main())
