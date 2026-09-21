"""Latency accounting tests."""

import json

import pytest

from stereolink.latency import LatencyBudget, percentile


def test_percentile_returns_an_observed_value():
    values = [1.0, 2.0, 3.0, 4.0, 100.0]
    assert percentile(values, 50) in values
    assert percentile(values, 100) == 100.0
    assert percentile(values, 0) == 1.0
    assert percentile([], 50) == 0.0


def test_stage_timer_records_elapsed_time():
    b = LatencyBudget(50)
    with b.stage("disparity"):
        sum(range(200000))
    row = next(r for r in b.snapshot()["stages"] if r["stage"] == "disparity")
    assert row["count"] == 1
    assert row["p50_ms"] > 0


def test_roundtrip_closes_on_matching_seq():
    b = LatencyBudget(50)
    b.mark_sent(7, t_capture=100.0, t_tx=100.02)
    assert b.mark_echo(7, t_echo=100.05) is True
    totals = b.snapshot()["totals"]
    assert totals["end_to_end"]["p50_ms"] == pytest.approx(50.0, abs=0.5)
    assert totals["round_trip"]["p50_ms"] == pytest.approx(30.0, abs=0.5)


def test_unknown_echo_is_reported_not_crashed():
    b = LatencyBudget(50)
    assert b.mark_echo(99, t_echo=1.0) is False


def test_echo_is_matched_only_once():
    b = LatencyBudget(50)
    b.mark_sent(3, 10.0, 10.01)
    assert b.mark_echo(3, 10.03) is True
    # A duplicate TPDO would otherwise double-count the round trip.
    assert b.mark_echo(3, 10.04) is False


def test_seq_wraps_at_one_byte_without_collision():
    b = LatencyBudget(50)
    b.mark_sent(255, 1.0, 1.01)
    b.mark_sent(256, 2.0, 2.01)   # wraps to 0
    assert b.mark_echo(0, 2.02) is True
    assert b.mark_echo(255, 1.02) is True


def test_pending_table_is_bounded_when_echoes_stop():
    b = LatencyBudget(50)
    for i in range(2000):
        b.mark_sent(i, float(i), float(i) + 0.01)
    # Without a bound this would grow without limit on a one-way link failure.
    assert len(b._pending) <= 512


def test_node_apply_derives_a_link_estimate():
    b = LatencyBudget(50)
    b.mark_sent(1, 0.0, 0.0)
    b.mark_echo(1, 0.010)          # 10 ms round trip
    b.note_node_apply(2000)        # 2 ms spent inside the node
    stages = {r["stage"]: r for r in b.snapshot()["stages"]}
    assert stages["node_apply"]["p50_ms"] == pytest.approx(2.0, abs=0.01)
    # (10 - 2) / 2 = 4 ms each way
    assert stages["link_to_node"]["p50_ms"] == pytest.approx(4.0, abs=0.01)


def test_markdown_and_json_reports_render(tmp_path):
    b = LatencyBudget(50)
    with b.stage("capture"):
        pass
    b.mark_sent(1, 0.0, 0.01)
    b.mark_echo(1, 0.03)

    md = b.to_markdown()
    assert "| Stage |" in md and "capture" in md and "end to end" in md

    jf, mf = tmp_path / "l.json", tmp_path / "l.md"
    b.write(jf, mf)
    data = json.loads(jf.read_text())
    assert data["echoes_matched"] == 1
    assert mf.read_text().startswith("# Latency budget")
