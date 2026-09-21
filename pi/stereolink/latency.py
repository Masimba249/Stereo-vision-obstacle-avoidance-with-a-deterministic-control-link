"""End-to-end latency accounting, broken down per stage.

The measurement problem: the Pi and the ESP32 have unrelated clocks, so we
cannot just subtract timestamps across the link.  Instead:

* the Pi stamps ``t_capture`` per frame and carries a one-byte ``seq`` token
  through the pipeline and into the RPDO;
* the ESP32 echoes ``seq`` back in TPDO1/TPDO3 and reports its *own local*
  rx->apply span, which needs no shared clock;
* the Pi measures the full loop as ``t_echo_rx - t_capture``.

That gives a per-stage breakdown on the Pi, a measured actuation span on the
node, and a round-trip figure that bounds everything in between.
"""

from __future__ import annotations

import json
import statistics
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

# Canonical stage order.  Reported in this order so the table reads as the
# path a photon takes to become a PWM duty cycle.
STAGES = (
    "capture",       # sensor exposure handoff -> frame in userspace
    "decode",        # MJPEG decode + eye split + downscale
    "rectify",       # remap both eyes
    "disparity",     # StereoSGBM
    "depth",         # disparity -> metric depth
    "obstacle",      # depth -> free-space columns
    "plan",          # columns -> (v, omega)
    "encode_tx",     # PDO encode + socketcan write
    "link_to_node",  # wire + node rx queue (derived)
    "node_apply",    # node rx ISR -> PWM register write (reported by node)
)


@dataclass
class StageStats:
    name: str
    count: int
    mean_ms: float
    p50_ms: float
    p95_ms: float
    p99_ms: float
    max_ms: float

    def as_dict(self) -> dict:
        return {
            "stage": self.name, "count": self.count,
            "mean_ms": round(self.mean_ms, 3), "p50_ms": round(self.p50_ms, 3),
            "p95_ms": round(self.p95_ms, 3), "p99_ms": round(self.p99_ms, 3),
            "max_ms": round(self.max_ms, 3),
        }


def percentile(values: list[float], pct: float) -> float:
    if not values:
        return 0.0
    s = sorted(values)
    if len(s) == 1:
        return s[0]
    # Nearest-rank on the sorted sample: no interpolation games, and it always
    # returns a value that was actually observed.
    k = max(0, min(len(s) - 1, int(round(pct / 100.0 * (len(s) - 1)))))
    return s[k]


class LatencyBudget:
    """Rolling per-stage timing plus seq-keyed round-trip closure."""

    def __init__(self, window: int = 600):
        self.window = window
        self._stages: dict[str, deque[float]] = {s: deque(maxlen=window) for s in STAGES}
        self._roundtrip: deque[float] = deque(maxlen=window)
        self._e2e: deque[float] = deque(maxlen=window)
        # seq -> (t_capture, t_tx).  One byte of seq wraps every 256 frames,
        # which at 10 Hz is 25 s - far longer than any plausible round trip.
        self._pending: dict[int, tuple[float, float]] = {}
        self._dropped_echoes = 0
        self._matched_echoes = 0

    # -- per-frame stage timing --------------------------------------------
    def record(self, stage: str, seconds: float) -> None:
        if stage in self._stages:
            self._stages[stage].append(seconds * 1000.0)

    def stage(self, name: str) -> "_StageTimer":
        return _StageTimer(self, name)

    # -- cross-link closure -------------------------------------------------
    def mark_sent(self, seq: int, t_capture: float, t_tx: float) -> None:
        """Remember when the frame behind ``seq`` was captured and transmitted."""
        if len(self._pending) > 512:
            # Bound the table if echoes stop coming back entirely.
            oldest = sorted(self._pending, key=lambda k: self._pending[k][0])[:256]
            for k in oldest:
                self._pending.pop(k, None)
                self._dropped_echoes += 1
        self._pending[seq & 0xFF] = (t_capture, t_tx)

    def mark_echo(self, seq: int, t_echo: float) -> bool:
        """Close the loop for ``seq``.  Returns False if we had no record of it."""
        entry = self._pending.pop(seq & 0xFF, None)
        if entry is None:
            return False
        t_capture, t_tx = entry
        self._matched_echoes += 1
        self._e2e.append((t_echo - t_capture) * 1000.0)
        self._roundtrip.append((t_echo - t_tx) * 1000.0)
        return True

    def note_node_apply(self, apply_us: int) -> None:
        """Record the node-local rx->apply span, and derive the wire time.

        TPDO3 carries this figure but arrives just after TPDO1 has already
        closed the round trip for the same seq, so it is paired with the most
        recent round-trip sample rather than re-keyed on seq - at 10 Hz the two
        frames are microseconds apart on the same bus.

        Whatever the round trip has left over after the node's own work is wire
        time plus queueing in both directions, so halving it gives a one-way
        estimate.  It is an estimate: it assumes the two directions are
        symmetric, which is true for equal-length PDOs at one bitrate.
        """
        apply_ms = apply_us / 1000.0
        self.record("node_apply", apply_ms / 1000.0)
        if self._roundtrip:
            link_ms = max(self._roundtrip[-1] - apply_ms, 0.0) / 2.0
            self.record("link_to_node", link_ms / 1000.0)

    # -- reporting ----------------------------------------------------------
    def stats(self, name: str, samples: Iterable[float]) -> StageStats | None:
        vals = list(samples)
        if not vals:
            return None
        return StageStats(
            name, len(vals), statistics.fmean(vals), percentile(vals, 50),
            percentile(vals, 95), percentile(vals, 99), max(vals),
        )

    def snapshot(self) -> dict:
        rows = []
        for stage in STAGES:
            st = self.stats(stage, self._stages[stage])
            if st is not None:
                rows.append(st.as_dict())
        totals = {}
        for key, data in (("end_to_end", self._e2e), ("round_trip", self._roundtrip)):
            st = self.stats(key, data)
            if st is not None:
                totals[key] = st.as_dict()
        # The sum of stage medians is the "typical" path cost; it is not the
        # same as the median end-to-end (stages do not peak together), and
        # both numbers are reported so nobody has to guess which is which.
        sum_p50 = sum(r["p50_ms"] for r in rows if r["stage"] != "link_to_node")
        return {
            "generated_at": time.time(),
            "window": self.window,
            "stages": rows,
            "totals": totals,
            "sum_of_stage_p50_ms": round(sum_p50, 3),
            "echoes_matched": self._matched_echoes,
            "echoes_lost": self._dropped_echoes,
        }

    def to_markdown(self) -> str:
        snap = self.snapshot()
        lines = [
            "# Latency budget (measured)", "",
            f"Window: last {snap['window']} samples per stage. "
            f"Echoes matched: {snap['echoes_matched']}, lost: {snap['echoes_lost']}.", "",
            "| Stage | n | mean ms | p50 ms | p95 ms | p99 ms | max ms |",
            "|---|---:|---:|---:|---:|---:|---:|",
        ]
        for r in snap["stages"]:
            lines.append(
                f"| {r['stage']} | {r['count']} | {r['mean_ms']:.2f} | {r['p50_ms']:.2f} "
                f"| {r['p95_ms']:.2f} | {r['p99_ms']:.2f} | {r['max_ms']:.2f} |")
        for key, label in (("end_to_end", "**capture -> echo (end to end)**"),
                           ("round_trip", "**tx -> echo (link round trip)**")):
            r = snap["totals"].get(key)
            if r:
                lines.append(
                    f"| {label} | {r['count']} | {r['mean_ms']:.2f} | {r['p50_ms']:.2f} "
                    f"| {r['p95_ms']:.2f} | {r['p99_ms']:.2f} | {r['max_ms']:.2f} |")
        lines += ["", f"Sum of stage medians: {snap['sum_of_stage_p50_ms']:.2f} ms", ""]
        return "\n".join(lines)

    def write(self, json_path: str | Path | None, md_path: str | Path | None) -> None:
        if json_path:
            p = Path(json_path)
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text(json.dumps(self.snapshot(), indent=2), encoding="utf-8")
        if md_path:
            p = Path(md_path)
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text(self.to_markdown(), encoding="utf-8")


class _StageTimer:
    __slots__ = ("_budget", "_name", "_t0")

    def __init__(self, budget: LatencyBudget, name: str):
        self._budget = budget
        self._name = name

    def __enter__(self):
        self._t0 = time.perf_counter()
        return self

    def __exit__(self, *exc):
        self._budget.record(self._name, time.perf_counter() - self._t0)
        return False
