"""Sparkplug B encoder tests, including the cases a hand-rolled protobuf
implementation is most likely to get wrong."""

import pytest

from stereolink.north.sparkplug_b import (
    DataType, Metric, Payload, decode_payload, topic,
)


@pytest.mark.parametrize("value,datatype", [
    (0, DataType.Int8), (127, DataType.Int8), (-128, DataType.Int8),
    (-450, DataType.Int16), (32767, DataType.Int16), (-32768, DataType.Int16),
    (-70000, DataType.Int32), (2147483647, DataType.Int32),
    (-5, DataType.Int64), (2**63 - 1, DataType.UInt64),
    (255, DataType.UInt8), (65535, DataType.UInt16), (4294967295, DataType.UInt32),
    (True, DataType.Boolean), (False, DataType.Boolean),
    ("DEGRADED", DataType.String),
])
def test_scalar_roundtrip(value, datatype):
    decoded = decode_payload(Payload([Metric("m", value, datatype)]).encode())
    assert decoded["metrics"][0]["value"] == value


def test_float_and_double_precision():
    out = decode_payload(Payload([
        Metric("f", 1.452, DataType.Float),
        Metric("d", 1.452, DataType.Double),
    ]).encode())["metrics"]
    # Float is 32-bit, so it is lossy; Double must be exact.
    assert out[0]["value"] == pytest.approx(1.452, abs=1e-6)
    assert out[1]["value"] == 1.452


def test_negative_int16_is_two_complement_not_garbage():
    # The bug this pins: widening a negative Int16 into the uint32 value slot
    # and then re-signing against the wrong width.
    decoded = decode_payload(Payload([Metric("v", -450, DataType.Int16)]).encode())
    assert decoded["metrics"][0]["value"] == -450


def test_null_metric_sets_is_null_and_omits_value():
    decoded = decode_payload(
        Payload([Metric("x", None, DataType.Float, is_null=True)]).encode())
    metric = decoded["metrics"][0]
    assert metric["is_null"] is True
    assert metric["value"] is None


def test_seq_and_timestamp_survive():
    decoded = decode_payload(
        Payload([Metric("a", 1, DataType.Int8)], seq=200,
                timestamp_ms=1700000000000).encode())
    assert decoded["seq"] == 200
    assert decoded["timestamp_ms"] == 1700000000000


def test_alias_only_payload_is_smaller_and_keeps_aliases():
    metrics = [Metric(f"Some/Long/Metric/Name/{i}", i, DataType.Int16, alias=i)
               for i in range(20)]
    with_names = Payload(metrics, seq=1).encode(include_names=True)
    alias_only = Payload(metrics, seq=1).encode(include_names=False)
    # This is the whole reason BIRTH declares aliases.
    assert len(alias_only) < len(with_names) / 2
    decoded = decode_payload(alias_only)["metrics"]
    assert [m["alias"] for m in decoded] == list(range(20))
    assert all(m["name"] is None for m in decoded)


def test_topic_namespace_matches_spec():
    assert topic("G", "NBIRTH", "edge") == "spBv1.0/G/NBIRTH/edge"
    assert topic("G", "DDATA", "edge", "dev") == "spBv1.0/G/DDATA/edge/dev"


def test_varint_boundaries():
    # 7-bit group boundaries are where a hand-rolled varint encoder breaks.
    for value in (0, 1, 127, 128, 16383, 16384, 2097151, 2097152, 2**32 - 1):
        decoded = decode_payload(
            Payload([Metric("v", value, DataType.UInt64)]).encode())
        assert decoded["metrics"][0]["value"] == value
