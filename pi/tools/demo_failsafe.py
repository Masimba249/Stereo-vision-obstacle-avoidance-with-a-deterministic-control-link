#!/usr/bin/env python3
"""End-to-end demonstration: vision -> link -> drive, then cut the cable.

Runs the real pipeline (synthetic camera, real SGBM, real planner, real PDO
codec) against the behavioural node model over an in-process CAN bus, then
severs the Pi->node direction and shows the node bringing itself to a stop on
its own clock.  No hardware, no broker, no kernel CAN required:

    python3 pi/tools/demo_failsafe.py
"""

from __future__ import annotations

import argparse
import logging
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from esp32_sim import Esp32NodeSim  # noqa: E402
from stereolink.canlink import LoopbackTransport  # noqa: E402
from stereolink.config import AppConfig  # noqa: E402
from stereolink.pdo import DriveState  # noqa: E402
from stereolink.pipeline import Pipeline  # noqa: E402

log = logging.getLogger("demo")


def build_config(frames_hz: float = 10.0) -> AppConfig:
    cfg = AppConfig()
    cfg.camera.source = "synthetic"
    cfg.camera.capture_width = 1280      # per-eye 640 after the split
    cfg.camera.capture_height = 360
    cfg.camera.process_hz = frames_hz
    cfg.can.interface = "loopback"
    cfg.can.send_sync = False
    cfg.can.setpoint_watchdog_ms = 150
    cfg.mqtt.enabled = False
    cfg.opcua.enabled = False
    cfg.latency.report_path = "reports/latency.json"
    cfg.latency.markdown_path = "reports/latency-budget.md"
    cfg.latency.report_every_s = 5.0
    return cfg


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-s", type=float, default=4.0,
                    help="seconds of healthy operation before the cut")
    ap.add_argument("--cut-s", type=float, default=3.0,
                    help="seconds to observe after the link is severed")
    ap.add_argument("--watchdog-ms", type=int, default=150)
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    logging.basicConfig(level=logging.WARNING if args.quiet else logging.INFO,
                        format="%(asctime)s %(levelname)-7s %(name)-12s %(message)s")

    bus = LoopbackTransport()
    cfg = build_config()
    cfg.can.setpoint_watchdog_ms = args.watchdog_ms

    node = Esp32NodeSim(bus.endpoint("node"), node_id=cfg.can.node_id,
                        watchdog_ms=args.watchdog_ms)
    pipeline = Pipeline(cfg, transport=bus.endpoint("master"))

    timeline: list[tuple[float, str, float, str]] = []
    t0 = time.monotonic()

    def sample(tag: str) -> None:
        st = pipeline.link.node.status
        timeline.append((time.monotonic() - t0, tag,
                         float(st.v_meas_mm_s) if st else 0.0,
                         st.state.name if st else "-"))

    node.start()
    runner = threading.Thread(target=pipeline.run, daemon=True)
    runner.start()

    print("\n--- phase 1: healthy link ---")
    deadline = time.monotonic() + args.run_s
    while time.monotonic() < deadline:
        time.sleep(0.25)
        sample("healthy")
        st = pipeline.link.node.status
        print(f"  t={timeline[-1][0]:5.2f}s  state={timeline[-1][3]:<9} "
              f"v_meas={timeline[-1][2]:6.0f} mm/s  "
              f"rpdo_age={node.rpdo_age_ms:5.0f} ms  reason={pipeline.stats.last_reason}")

    print(f"\n--- phase 2: CUTTING the Pi->node link (watchdog = {args.watchdog_ms} ms) ---")
    t_cut = time.monotonic()
    bus.cut("master")
    stopped_at = None
    degraded_at = None
    deadline = time.monotonic() + args.cut_s
    while time.monotonic() < deadline:
        time.sleep(0.05)
        st = pipeline.link.node.status
        if st is not None:
            if degraded_at is None and st.state == DriveState.DEGRADED:
                degraded_at = time.monotonic() - t_cut
            if stopped_at is None and st.v_meas_mm_s == 0 and \
                    st.state in (DriveState.DEGRADED, DriveState.SAFE_STOP):
                stopped_at = time.monotonic() - t_cut
        if len(timeline) == 0 or (time.monotonic() - t0) - timeline[-1][0] > 0.2:
            sample("cut")
            print(f"  t={timeline[-1][0]:5.2f}s  state={timeline[-1][3]:<9} "
                  f"v_meas={timeline[-1][2]:6.0f} mm/s  "
                  f"rpdo_age={node.rpdo_age_ms:5.0f} ms")

    pipeline._running = False
    runner.join(timeout=5.0)
    node.stop()

    print("\n--- result ---")
    print(f"  watchdog configured:        {args.watchdog_ms} ms")
    print(f"  node entered DEGRADED at:   "
          f"{degraded_at*1000:.0f} ms after the cut" if degraded_at
          else "  node never entered DEGRADED  <-- FAILSAFE DID NOT FIRE")
    print(f"  wheels reached zero at:     "
          f"{stopped_at*1000:.0f} ms after the cut" if stopped_at
          else "  wheels never reached zero    <-- FAILSAFE DID NOT FIRE")
    print(f"  final node state:           "
          f"{pipeline.link.node.status.state.name if pipeline.link.node.status else '-'}")
    print(f"  EMCY frames received:       {pipeline.link.node.emcy_count}")
    print(f"  setpoints delivered:        {node.rpdo_count}")
    print(f"  frames processed:           {pipeline.stats.frames}")

    if pipeline.latency is not None:
        print()
        print(pipeline.latency.to_markdown())

    ok = degraded_at is not None and stopped_at is not None
    print("PASS: link loss was contained by the node" if ok
          else "FAIL: failsafe did not engage")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
