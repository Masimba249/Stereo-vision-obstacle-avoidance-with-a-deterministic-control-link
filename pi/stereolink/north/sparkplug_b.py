"""Sparkplug B payload encoder.

Written against the Sparkplug B specification's protobuf schema rather than
pulling in ``tahu``/``protobuf``: the payload is a handful of fields, and a
self-contained encoder keeps the edge node's dependency list short - which
matters on a Pi image that has to be reproducible.

Encodes ``org.eclipse.tahu.protobuf.Payload``:

    Payload { uint64 timestamp=1; repeated Metric metrics=2;
              uint64 seq=3; string uuid=4; bytes body=5; }
    Metric  { string name=1; uint64 alias=2; uint64 timestamp=3;
              uint32 datatype=4; bool is_historical=5; bool is_transient=6;
              bool is_null=7; ... oneof value {
                  uint32 int_value=10; uint64 long_value=11;
                  float float_value=12; double double_value=13;
                  bool boolean_value=14; string string_value=15;
                  bytes bytes_value=16; } }

A decoder is included so the unit tests can round-trip without a broker, and
so ``tools/sparkplug_listen.py`` can print what an SCADA client would see.
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Any

# --- wire primitives -------------------------------------------------------
WIRE_VARINT = 0
WIRE_FIXED64 = 1
WIRE_LEN = 2
WIRE_FIXED32 = 5


def _varint(value: int) -> bytes:
    if value < 0:
        # Protobuf encodes negative varints as 64-bit two's complement.
        value += 1 << 64
    out = bytearray()
    while True:
        b = value & 0x7F
        value >>= 7
        if value:
            out.append(b | 0x80)
        else:
            out.append(b)
            return bytes(out)


def _tag(field: int, wire: int) -> bytes:
    return _varint((field << 3) | wire)


def _len_field(field: int, payload: bytes) -> bytes:
    return _tag(field, WIRE_LEN) + _varint(len(payload)) + payload


def _read_varint(buf: bytes, pos: int) -> tuple[int, int]:
    result = shift = 0
    while True:
        b = buf[pos]
        pos += 1
        result |= (b & 0x7F) << shift
        if not b & 0x80:
            return result, pos
        shift += 7


class DataType(IntEnum):
    Unknown = 0
    Int8 = 1
    Int16 = 2
    Int32 = 3
    Int64 = 4
    UInt8 = 5
    UInt16 = 6
    UInt32 = 7
    UInt64 = 8
    Float = 9
    Double = 10
    Boolean = 11
    String = 12
    DateTime = 13
    Text = 14
    UUID = 15
    DataSet = 16
    Bytes = 17
    File = 18
    Template = 19


_INT_TYPES = {DataType.Int8, DataType.Int16, DataType.Int32,
              DataType.UInt8, DataType.UInt16, DataType.UInt32}
_LONG_TYPES = {DataType.Int64, DataType.UInt64, DataType.DateTime}
_STRING_TYPES = {DataType.String, DataType.Text, DataType.UUID}
_SIGNED = {DataType.Int8: 8, DataType.Int16: 16, DataType.Int32: 32, DataType.Int64: 64}


@dataclass
class Metric:
    name: str | None
    value: Any
    datatype: DataType
    alias: int | None = None
    timestamp_ms: int | None = None
    is_null: bool = False

    def encode(self, include_name: bool = True) -> bytes:
        out = bytearray()
        # After a BIRTH has bound name->alias, DATA messages may send the alias
        # alone.  That is the whole point of aliases: it roughly halves the
        # payload at 5 Hz over a cellular backhaul.
        if include_name and self.name is not None:
            out += _len_field(1, self.name.encode("utf-8"))
        if self.alias is not None:
            out += _tag(2, WIRE_VARINT) + _varint(self.alias)
        ts = self.timestamp_ms if self.timestamp_ms is not None else int(time.time() * 1000)
        out += _tag(3, WIRE_VARINT) + _varint(ts)
        out += _tag(4, WIRE_VARINT) + _varint(int(self.datatype))
        if self.is_null or self.value is None:
            out += _tag(7, WIRE_VARINT) + _varint(1)
            return bytes(out)
        out += self._encode_value()
        return bytes(out)

    def _encode_value(self) -> bytes:
        dt, v = self.datatype, self.value
        if dt in _INT_TYPES:
            iv = int(v)
            if dt in _SIGNED and iv < 0:
                iv += 1 << 32        # two's complement in the uint32 slot
            return _tag(10, WIRE_VARINT) + _varint(iv & 0xFFFFFFFF)
        if dt in _LONG_TYPES:
            iv = int(v)
            if dt in _SIGNED and iv < 0:
                iv += 1 << 64
            return _tag(11, WIRE_VARINT) + _varint(iv & 0xFFFFFFFFFFFFFFFF)
        if dt == DataType.Float:
            return _tag(12, WIRE_FIXED32) + struct.pack("<f", float(v))
        if dt == DataType.Double:
            return _tag(13, WIRE_FIXED64) + struct.pack("<d", float(v))
        if dt == DataType.Boolean:
            return _tag(14, WIRE_VARINT) + _varint(1 if v else 0)
        if dt in _STRING_TYPES:
            return _len_field(15, str(v).encode("utf-8"))
        if dt in (DataType.Bytes, DataType.File):
            return _len_field(16, bytes(v))
        raise ValueError(f"unsupported Sparkplug datatype for encode: {dt!r}")


@dataclass
class Payload:
    metrics: list[Metric]
    seq: int | None = None
    timestamp_ms: int | None = None
    uuid: str | None = None
    body: bytes | None = None

    def encode(self, include_names: bool = True) -> bytes:
        out = bytearray()
        ts = self.timestamp_ms if self.timestamp_ms is not None else int(time.time() * 1000)
        out += _tag(1, WIRE_VARINT) + _varint(ts)
        for m in self.metrics:
            out += _len_field(2, m.encode(include_name=include_names))
        if self.seq is not None:
            out += _tag(3, WIRE_VARINT) + _varint(self.seq & 0xFF)
        if self.uuid is not None:
            out += _len_field(4, self.uuid.encode("utf-8"))
        if self.body is not None:
            out += _len_field(5, self.body)
        return bytes(out)


# --- decoding (test / tooling side) ---------------------------------------
def _skip(buf: bytes, pos: int, wire: int) -> tuple[bytes | int, int]:
    if wire == WIRE_VARINT:
        return _read_varint(buf, pos)
    if wire == WIRE_FIXED64:
        return buf[pos:pos + 8], pos + 8
    if wire == WIRE_FIXED32:
        return buf[pos:pos + 4], pos + 4
    if wire == WIRE_LEN:
        n, pos = _read_varint(buf, pos)
        return buf[pos:pos + n], pos + n
    raise ValueError(f"unsupported wire type {wire}")


def decode_metric(buf: bytes) -> dict:
    out: dict[str, Any] = {"name": None, "alias": None, "value": None,
                           "datatype": None, "timestamp_ms": None, "is_null": False}
    pos = 0
    while pos < len(buf):
        key, pos = _read_varint(buf, pos)
        field, wire = key >> 3, key & 0x07
        val, pos = _skip(buf, pos, wire)
        if field == 1:
            out["name"] = val.decode("utf-8")
        elif field == 2:
            out["alias"] = val
        elif field == 3:
            out["timestamp_ms"] = val
        elif field == 4:
            out["datatype"] = DataType(val)
        elif field == 7:
            out["is_null"] = bool(val)
        elif field == 10:
            out["value"] = val
        elif field == 11:
            out["value"] = val
        elif field == 12:
            out["value"] = struct.unpack("<f", val)[0]
        elif field == 13:
            out["value"] = struct.unpack("<d", val)[0]
        elif field == 14:
            out["value"] = bool(val)
        elif field == 15:
            out["value"] = val.decode("utf-8")
        elif field == 16:
            out["value"] = bytes(val)

    dt = out["datatype"]
    if dt in _SIGNED and isinstance(out["value"], int):
        # Re-sign: a negative Int8/16/32 travels as two's complement widened
        # into the 32-bit int_value slot, so mask back to the DECLARED width
        # (not the storage width) before testing the sign bit.
        bits = _SIGNED[dt]
        v = out["value"] & ((1 << bits) - 1)
        if v >= 1 << (bits - 1):
            v -= 1 << bits
        out["value"] = v
    return out


def decode_payload(buf: bytes) -> dict:
    out: dict[str, Any] = {"timestamp_ms": None, "metrics": [], "seq": None,
                           "uuid": None, "body": None}
    pos = 0
    while pos < len(buf):
        key, pos = _read_varint(buf, pos)
        field, wire = key >> 3, key & 0x07
        val, pos = _skip(buf, pos, wire)
        if field == 1:
            out["timestamp_ms"] = val
        elif field == 2:
            out["metrics"].append(decode_metric(val))
        elif field == 3:
            out["seq"] = val
        elif field == 4:
            out["uuid"] = val.decode("utf-8")
        elif field == 5:
            out["body"] = bytes(val)
    return out


# --- topic helpers ---------------------------------------------------------
NAMESPACE = "spBv1.0"


def topic(group_id: str, message_type: str, edge_node_id: str,
          device_id: str | None = None) -> str:
    parts = [NAMESPACE, group_id, message_type, edge_node_id]
    if device_id:
        parts.append(device_id)
    return "/".join(parts)
