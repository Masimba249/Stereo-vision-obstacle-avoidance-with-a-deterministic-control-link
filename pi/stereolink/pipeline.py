"""The 10 Hz vision -> setpoint loop, and the glue to the link and north-bound.

Threading model (this is the whole design):

    main thread      capture -> rectify -> SGBM -> obstacles -> plan -> TX
    can-rx thread    drains the CAN socket, updates NodeView, closes latency
    can-sync thread  emits SYNC
    north thread(s)  MQTT client loop / OPC UA event loop

Only the main thread produces setpoints, and it never blocks on the network.
North-bound publishing is rate-limited and non-blocking, because an IT-layer
stall must not be able to slow down the OT-layer loop.  That separation is the
same principle the firmware applies between its control and comms tasks.
"""

from __future__ import annotations

import logging
import signal
import time
from dataclasses import dataclass

import cv2
import numpy as np

from .calibration import Rectifier, StereoCalibration
from .camera import open_source
from .canlink import CanOpenMaster
from .disparity import DisparityEngine
from .latency import LatencyBudget
from .north.mqtt_north import SparkplugPublisher
from .north.opcua_north import OpcUaPublisher
from .obstacle import ObstacleAnalyzer
from .pdo import DriveState, LimitsPdo, SetpointFlags, SetpointPdo
from .planner import AvoidancePlanner

log = logging.getLogger(__name__)


@dataclass
class PipelineStats:
    frames: int = 0
    dropped: int = 0
    fps: float = 0.0
    last_reason: str = ""


class Pipeline:
    def __init__(self, cfg, transport=None):
        self.cfg = cfg
        self.stats = PipelineStats()
        self.latency = LatencyBudget(cfg.latency.window) if cfg.latency.enabled else None
        self._estop = False
        self._running = False
        self._t_last_frame = 0.0
        self._t_last_report = 0.0
        self._t_last_north = 0.0
        self._fps_ema = 0.0

        self.calib = self._load_calibration()
        work = (cfg.stereo.work_width, cfg.stereo.work_height)
        self.rectifier = Rectifier(self.calib, work)
        self.source = open_source(cfg.camera, self.calib)
        self.disparity = DisparityEngine(cfg.stereo)
        self.analyzer = ObstacleAnalyzer(cfg.obstacle, self.rectifier)
        self.planner = AvoidancePlanner(cfg.planner)

        self.link = CanOpenMaster(cfg.can, transport, latency=self.latency)
        self.mqtt = SparkplugPublisher(cfg.mqtt, on_command=self._on_north_command)
        self.opcua = OpcUaPublisher(cfg.opcua, on_command=self._on_north_command)

        log.info("stereo rig: f=%.1f px  baseline=%.3f m  work=%dx%d",
                 self.rectifier.focal_px, self.rectifier.baseline_m, *work)

    def _load_calibration(self) -> StereoCalibration:
        try:
            calib = StereoCalibration.load(self.cfg.calibration_path)
            log.info("loaded calibration from %s (rms=%.3f px, baseline=%.1f mm)",
                     self.cfg.calibration_path, calib.rms, calib.baseline_m * 1000)
            return calib
        except (FileNotFoundError, ValueError) as exc:
            if self.cfg.camera.source != "synthetic":
                # Running on real hardware with guessed intrinsics would give
                # plausible-looking but wrong distances, which is worse than
                # refusing to start.
                raise RuntimeError(
                    f"no usable stereo calibration at "
                    f"{self.cfg.calibration_path!r} ({exc}). Run "
                    f"tools/calibrate_stereo.py first, or set "
                    f"camera.source: synthetic to run the simulator.") from exc
            log.warning("no calibration file; using synthetic rig geometry")
            return StereoCalibration.synthetic(
                self.cfg.camera.capture_width // 2, self.cfg.camera.capture_height)

    # -- north-bound commands ----------------------------------------------
    def _on_north_command(self, name: str | None, value) -> None:
        if name is None:
            return
        if name.endswith("EStopRequest") or name.endswith("Node Control/EStop"):
            self._estop = bool(value)
            log.warning("north-bound e-stop request: %s", self._estop)
        elif name.endswith("Node Control/Rebirth") and value:
            self.mqtt._publish_births()

    # -- one cycle ----------------------------------------------------------
    def process_frame(self, frame) -> tuple[object, object]:
        lat = self.latency
        cfg = self.cfg

        def stage(name):
            return lat.stage(name) if lat is not None else _NullCtx()

        with stage("decode"):
            left = self.disparity.to_gray(frame.left)
            right = self.disparity.to_gray(frame.right)
            work = (cfg.stereo.work_width, cfg.stereo.work_height)
            if (left.shape[1], left.shape[0]) != work:
                # INTER_AREA is the right kernel for downscaling: it averages
                # the source block instead of point-sampling, which preserves
                # the texture SGBM needs to match.
                left = cv2.resize(left, work, interpolation=cv2.INTER_AREA)
                right = cv2.resize(right, work, interpolation=cv2.INTER_AREA)

        with stage("rectify"):
            lr, rr = self.rectifier.rectify(left, right)

        with stage("disparity"):
            disp = self.disparity.compute(lr, rr)

        with stage("depth"):
            depth = self.rectifier.depth_from_disparity(np.nan_to_num(disp, nan=0.0))

        with stage("obstacle"):
            report = self.analyzer.analyze(depth)

        with stage("plan"):
            setpoint = self.planner.plan(report, estop=self._estop)

        return report, setpoint

    def _send(self, setpoint, t_capture: float) -> int:
        lat = self.latency
        seq = self.link.next_seq()
        pdo = SetpointPdo(
            v_mm_s=int(round(setpoint.v_mm_s)),
            w_mrad_s=int(round(setpoint.w_mrad_s)),
            obstacle_cm=int(setpoint.obstacle_cm),
            seq=seq,
            flags=setpoint.flags,
        )
        if lat is not None:
            with lat.stage("encode_tx"):
                self.link.send_setpoint(pdo, t_capture=t_capture)
        else:
            self.link.send_setpoint(pdo)
        return seq

    # -- telemetry ----------------------------------------------------------
    def _telemetry(self, report, setpoint) -> dict:
        node = self.link.node
        values: dict[str, object] = {
            "Setpoint/v_mm_s": int(round(setpoint.v_mm_s)),
            "Setpoint/w_mrad_s": int(round(setpoint.w_mrad_s)),
            "Setpoint/reason": setpoint.reason,
            "Vision/fps": float(self._fps_ema),
            "Link/online": node.online(self.cfg.can.heartbeat_timeout_s),
            "Link/heartbeats": int(node.heartbeat_count),
            "Link/emcy_count": int(node.emcy_count),
        }
        if report is not None:
            rng = report.min_range_m
            values.update({
                # Sparkplug has no +inf; report the configured max instead so
                # the historian does not store a NaN nobody can chart.
                "Vision/min_range_m": float(rng) if np.isfinite(rng)
                else float(self.cfg.obstacle.max_range_m),
                "Vision/min_range_bearing": float(report.min_range_bearing),
                "Vision/best_bearing": float(report.best_bearing),
                "Vision/valid_fraction": float(report.valid_fraction),
                "Vision/blocked": bool(report.blocked),
            })
        if node.status is not None:
            values.update({
                "Drive/v_meas_mm_s": int(node.status.v_meas_mm_s),
                "Drive/w_meas_mrad_s": int(node.status.w_meas_mrad_s),
                "Drive/state": node.status.state.name,
                "Drive/faults": int(node.status.faults),
                "Drive/ir_mask": int(node.status.ir_mask),
            })
        if node.diagnostics is not None:
            d = node.diagnostics
            values.update({
                "Diag/jitter_p99_us": int(d.jitter_p99_us),
                "Diag/period_max_us": int(d.period_max_us),
                "Diag/ctrl_cpu_pct": int(d.ctrl_cpu_pct),
                "Diag/comms_cpu_pct": int(d.comms_cpu_pct),
                "Link/rpdo_age_ms": int(d.rpdo_age_ms),
            })
        if self.latency is not None:
            snap = self.latency.snapshot()
            e2e = snap["totals"].get("end_to_end")
            rt = snap["totals"].get("round_trip")
            if e2e:
                values["Latency/e2e_p50_ms"] = float(e2e["p50_ms"])
                values["Latency/e2e_p95_ms"] = float(e2e["p95_ms"])
            if rt:
                values["Latency/roundtrip_p95_ms"] = float(rt["p95_ms"])
            for row in snap["stages"]:
                if row["stage"] == "node_apply":
                    values["Latency/node_apply_p95_ms"] = float(row["p95_ms"])
        return values

    def _publish_north(self, report, setpoint, now: float) -> None:
        period = 1.0 / max(self.cfg.mqtt.publish_hz, 0.1)
        if now - self._t_last_north < period:
            return
        self._t_last_north = now
        values = self._telemetry(report, setpoint)
        self.mqtt.publish(values)
        self.opcua.publish(values)

    def _maybe_write_report(self, now: float) -> None:
        if self.latency is None or self.cfg.latency.report_every_s <= 0:
            return
        if now - self._t_last_report < self.cfg.latency.report_every_s:
            return
        self._t_last_report = now
        self.latency.write(self.cfg.latency.report_path, self.cfg.latency.markdown_path)
        log.info("latency report written to %s", self.cfg.latency.report_path)

    # -- run ----------------------------------------------------------------
    def start(self) -> None:
        self.link.start()
        self.link.reset_node()
        time.sleep(0.05)
        self.link.send_limits(LimitsPdo(
            v_max_mm_s=int(self.cfg.planner.v_cruise_mm_s * 1.2),
            decel_mm_s2=1200,
            watchdog_ms=self.cfg.can.setpoint_watchdog_ms,
            ir_stop_cm=int(self.cfg.planner.stop_distance_m * 100),
        ))
        self.link.start_node()
        self.mqtt.start()
        self.opcua.start()
        self._running = True

    def stop(self) -> None:
        self._running = False
        try:
            # Command zero before dropping the link, so the node stops because
            # we told it to rather than because its watchdog fired.  Both work;
            # only one of them is graceful.
            self.link.send_setpoint(SetpointPdo(0, 0, 0, self.link.next_seq(),
                                                SetpointFlags.ESTOP_REQUEST))
            time.sleep(0.05)
            self.link.stop_node()
        except Exception:
            log.exception("failed to command a clean stop")
        if self.latency is not None:
            self.latency.write(self.cfg.latency.report_path,
                               self.cfg.latency.markdown_path)
        self.mqtt.stop()
        self.opcua.stop()
        self.link.stop()
        self.source.close()
        if self.cfg.preview:
            cv2.destroyAllWindows()

    def run(self, max_frames: int | None = None) -> None:
        self.start()
        period = 1.0 / max(self.cfg.camera.process_hz, 0.1)
        next_due = time.monotonic()
        stop_requested = {"v": False}

        def _handle(signum, _frame):
            log.info("signal %d received, shutting down", signum)
            stop_requested["v"] = True

        try:
            signal.signal(signal.SIGINT, _handle)
            signal.signal(signal.SIGTERM, _handle)
        except ValueError:
            pass  # not on the main thread (tests)

        try:
            while self._running and not stop_requested["v"]:
                if max_frames is not None and self.stats.frames >= max_frames:
                    break
                t_grab = time.monotonic()
                frame = self.source.read()
                if frame is None:
                    self.stats.dropped += 1
                    if self.stats.dropped % 50 == 1:
                        log.warning("camera returned no frame (%d so far)",
                                    self.stats.dropped)
                    time.sleep(0.01)
                    continue
                if self.latency is not None:
                    self.latency.record("capture", frame.t_capture - t_grab
                                        if frame.t_capture > t_grab else 0.0)

                report, setpoint = self.process_frame(frame)
                self._send(setpoint, frame.t_capture)

                now = time.monotonic()
                self.stats.frames += 1
                self.stats.last_reason = setpoint.reason
                if self._t_last_frame:
                    inst = 1.0 / max(now - self._t_last_frame, 1e-6)
                    self._fps_ema = inst if self._fps_ema == 0 else (
                        0.2 * inst + 0.8 * self._fps_ema)
                self._t_last_frame = now

                self._publish_north(report, setpoint, now)
                self._maybe_write_report(now)
                if self.cfg.preview:
                    self._draw(frame, report, setpoint)

                # Fixed-rate pacing that never tries to "catch up" after a
                # slow frame - bunching setpoints would only add jitter.
                next_due += period
                sleep_for = next_due - time.monotonic()
                if sleep_for < -period:
                    next_due = time.monotonic()
                elif sleep_for > 0:
                    time.sleep(sleep_for)
        finally:
            self.stop()

    def _draw(self, frame, report, setpoint) -> None:
        vis = cv2.cvtColor(self.disparity.to_gray(frame.left), cv2.COLOR_GRAY2BGR)
        vis = cv2.resize(vis, (self.cfg.stereo.work_width, self.cfg.stereo.work_height))
        h, w = vis.shape[:2]
        if report is not None:
            n = len(report.column_range_m)
            for i, rng in enumerate(report.column_range_m):
                x0, x1 = int(i * w / n), int((i + 1) * w / n)
                if not np.isfinite(rng):
                    colour, bar = (90, 90, 90), h
                else:
                    frac = float(np.clip(rng / self.cfg.obstacle.max_range_m, 0, 1))
                    colour = (0, int(255 * frac), int(255 * (1 - frac)))
                    bar = int(h * frac)
                cv2.rectangle(vis, (x0, h - bar), (x1 - 2, h), colour, -1)
        txt = (f"{setpoint.reason} v={setpoint.v_mm_s:.0f} w={setpoint.w_mrad_s:.0f} "
               f"d={setpoint.obstacle_cm}cm {self._fps_ema:.1f}fps")
        cv2.putText(vis, txt, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        cv2.imshow("stereolink", vis)
        cv2.waitKey(1)


class _NullCtx:
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False
