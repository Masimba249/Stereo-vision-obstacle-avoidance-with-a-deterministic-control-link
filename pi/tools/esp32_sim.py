#!/usr/bin/env python3
"""A software model of the ESP32 node, for running the system without hardware.

This is a *behavioural* twin of ``esp32/main/``: the same state machine, the
same watchdog rule, the same ramp-to-zero, the same PDO layout.  It exists so
that the failsafe and latency behaviour can be demonstrated and regression
tested in CI, where no CAN hardware and no ESP32 exist.

It is not a substitute for testing on the real node - it models the logic, not
the timing.  The firmware's own jitter numbers come from the hardware.
"""

from __future__ import annotations

import argparse
import logging
import random
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from stereolink.canlink import CanFrame, Transport  # noqa: E402
from stereolink.pdo import (  # noqa: E402
    COB_NMT, COB_RPDO1_BASE, COB_RPDO2_BASE, COB_SYNC, COB_EMCY_BASE,
    COB_HEARTBEAT_BASE, COB_TPDO1_BASE, COB_TPDO2_BASE, COB_TPDO3_BASE,
    DiagnosticsPdo, DriveState, EmcyPdo, FaultFlags, LatencyEchoPdo, LimitsPdo,
    NmtCommand, NmtState, SetpointFlags, SetpointPdo, StatusPdo, cob,
)

log = logging.getLogger("esp32sim")

CONTROL_HZ = 1000.0
STATUS_HZ = 50.0
DIAG_HZ = 5.0
HEARTBEAT_HZ = 5.0


class Esp32NodeSim:
    def __init__(self, transport: Transport, node_id: int = 0x22,
                 watchdog_ms: int = 150, decel_mm_s2: float = 1200.0,
                 v_max_mm_s: float = 1200.0, ir_stop_cm: int = 45):
        self.t = transport
        self.node_id = node_id
        self.watchdog_ms = watchdog_ms
        self.decel = decel_mm_s2
        self.v_max = v_max_mm_s
        self.ir_stop_cm = ir_stop_cm

        self.state = DriveState.INIT
        self.nmt = NmtState.PRE_OPERATIONAL
        self.faults = FaultFlags.NONE
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.v_meas = 0.0
        self.w_meas = 0.0
        self.seq_echo = 0
        self.ir_mask = 0
        self.watchdog_misses = 0

        self._t_last_rpdo = 0.0
        self._pending_echo: tuple[int, float] | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._emitted_emcy_for: DriveState | None = None
        self.rpdo_count = 0
        self.control_ticks = 0

    # -- helpers ------------------------------------------------------------
    def _send(self, base: int, data: bytes) -> None:
        self.t.send(CanFrame(cob(base, self.node_id), data, time.monotonic()))

    @property
    def rpdo_age_ms(self) -> float:
        if self._t_last_rpdo == 0.0:
            return float(self.watchdog_ms + 1)
        return (time.monotonic() - self._t_last_rpdo) * 1000.0

    # -- receive ------------------------------------------------------------
    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            frame = self.t.recv(timeout=0.05)
            if frame is None:
                continue
            cid = frame.arbitration_id
            if cid == COB_NMT and len(frame.data) >= 2:
                cmd, target = frame.data[0], frame.data[1]
                if target in (0, self.node_id):
                    self._handle_nmt(cmd)
            elif cid == COB_SYNC:
                pass  # SYNC-driven TPDOs are not used; we are time-triggered.
            elif cid == cob(COB_RPDO1_BASE, self.node_id):
                self._handle_setpoint(SetpointPdo.decode(frame.data))
            elif cid == cob(COB_RPDO2_BASE, self.node_id):
                self._handle_limits(LimitsPdo.decode(frame.data))

    def _handle_nmt(self, cmd: int) -> None:
        with self._lock:
            if cmd == NmtCommand.START:
                self.nmt = NmtState.OPERATIONAL
                if self.state in (DriveState.INIT, DriveState.IDLE):
                    self.state = DriveState.IDLE
            elif cmd == NmtCommand.STOP:
                self.nmt = NmtState.STOPPED
                self.state = DriveState.SAFE_STOP
                self.v_cmd = self.w_cmd = 0.0
            elif cmd in (NmtCommand.RESET_NODE, NmtCommand.RESET_COMM):
                self.nmt = NmtState.PRE_OPERATIONAL
                self.state = DriveState.INIT
                self.faults = FaultFlags.NONE
                self.v_cmd = self.w_cmd = 0.0
                self._t_last_rpdo = 0.0
                self._emitted_emcy_for = None
                self._send(COB_HEARTBEAT_BASE, bytes([NmtState.BOOTUP]))
            elif cmd == NmtCommand.ENTER_PREOP:
                self.nmt = NmtState.PRE_OPERATIONAL

    def _handle_limits(self, limits: LimitsPdo) -> None:
        with self._lock:
            self.v_max = min(float(limits.v_max_mm_s), 1500.0)
            self.decel = float(limits.decel_mm_s2)
            # Refuse to let the link widen its own watchdog: a timeout that can
            # be relaxed over the bus it protects is not a safety function.
            self.watchdog_ms = min(int(limits.watchdog_ms), self.watchdog_ms)
            self.ir_stop_cm = int(limits.ir_stop_cm)
        log.info("limits applied: v_max=%.0f decel=%.0f wd=%d ms ir_stop=%d cm",
                 self.v_max, self.decel, self.watchdog_ms, self.ir_stop_cm)

    def _handle_setpoint(self, sp: SetpointPdo) -> None:
        t_rx = time.monotonic()
        with self._lock:
            self.rpdo_count += 1
            self._t_last_rpdo = t_rx
            self.seq_echo = sp.seq
            self._pending_echo = (sp.seq, t_rx)

            if sp.flags & SetpointFlags.ESTOP_REQUEST:
                self.v_cmd = self.w_cmd = 0.0
                self.state = DriveState.SAFE_STOP
                return
            if sp.flags & SetpointFlags.CLEAR_FAULT and self.state == DriveState.SAFE_STOP:
                self.state = DriveState.IDLE
                self.faults = FaultFlags.NONE
                self._emitted_emcy_for = None

            if self.nmt != NmtState.OPERATIONAL:
                return
            if self.state in (DriveState.SAFE_STOP, DriveState.FAULT):
                return  # latched: only a fault clear or NMT reset leaves here

            v = max(-self.v_max, min(self.v_max, float(sp.v_mm_s)))
            if v != float(sp.v_mm_s):
                self.faults |= FaultFlags.SETPOINT_RANGE
            self.v_cmd, self.w_cmd = v, float(sp.w_mrad_s)
            self.faults &= ~FaultFlags.LINK_LOSS
            if self.state in (DriveState.IDLE, DriveState.DEGRADED):
                self.state = DriveState.RUN

    # -- control ------------------------------------------------------------
    def _control_loop(self) -> None:
        dt = 1.0 / CONTROL_HZ
        next_due = time.perf_counter()
        while not self._stop.is_set():
            next_due += dt
            with self._lock:
                self.control_ticks += 1
                self._control_tick(dt)
                echo = self._pending_echo
                self._pending_echo = None
            if echo is not None:
                seq, t_rx = echo
                # TPDO3 carries the node-local rx->apply span, which is the one
                # stage the Pi cannot time for itself.
                self._send(COB_TPDO3_BASE, LatencyEchoPdo(
                    rx_to_apply_us=int((time.monotonic() - t_rx) * 1e6),
                    seq_echo=seq, watchdog_misses=min(self.watchdog_misses, 255),
                    rpdo_rate_hz=10).encode())
            sleep = next_due - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_due = time.perf_counter()

    def _control_tick(self, dt: float) -> None:
        # 1) Fast safety layer first.  IR is independent of the vision link and
        #    of the CAN link: it must be able to stop the drive by itself.
        if self.ir_mask:
            self.faults |= FaultFlags.IR_BLOCKED
            self.v_cmd = min(self.v_cmd, 0.0)
            if self.state == DriveState.RUN:
                self.state = DriveState.DEGRADED
        else:
            self.faults &= ~FaultFlags.IR_BLOCKED

        # 2) Setpoint watchdog.  N missed cycles -> we stop trusting the Pi and
        #    bring the drive down under our own control, on our own clock.
        if self.state in (DriveState.RUN, DriveState.DEGRADED):
            if self.rpdo_age_ms > self.watchdog_ms:
                if self.state != DriveState.DEGRADED:
                    self.watchdog_misses += 1
                    log.warning("setpoint watchdog expired (age %.0f ms > %d ms) "
                                "-> DEGRADED, ramping to zero",
                                self.rpdo_age_ms, self.watchdog_ms)
                self.state = DriveState.DEGRADED
                self.faults |= FaultFlags.LINK_LOSS

        # 3) Target selection per state.
        if self.state == DriveState.DEGRADED:
            target_v, target_w = 0.0, 0.0
        elif self.state in (DriveState.SAFE_STOP, DriveState.FAULT,
                            DriveState.INIT, DriveState.IDLE):
            target_v, target_w = 0.0, 0.0
        else:
            target_v, target_w = self.v_cmd, self.w_cmd

        # 4) Ramp.  A controlled decel, not an instant zero: slamming the
        #    setpoint to 0 would be a torque step the drivetrain has to absorb.
        step = self.decel * dt
        self.v_meas += max(-step, min(step, target_v - self.v_meas))
        w_step = step * 4.0
        self.w_meas += max(-w_step, min(w_step, target_w - self.w_meas))
        if abs(self.v_meas) < 1.0:
            self.v_meas = 0.0
        if abs(self.w_meas) < 1.0:
            self.w_meas = 0.0

        # 5) Ramp complete -> latch SAFE_STOP and announce it once.
        if self.state == DriveState.DEGRADED and self.v_meas == 0.0 and self.w_meas == 0.0:
            self.state = DriveState.SAFE_STOP
            log.warning("ramp complete -> SAFE_STOP (outputs disabled, latched)")

        if self.state in (DriveState.SAFE_STOP, DriveState.FAULT) and \
                self._emitted_emcy_for != self.state:
            self._emitted_emcy_for = self.state
            code = (EmcyPdo.CODE_LINK_LOSS if self.faults & FaultFlags.LINK_LOSS
                    else EmcyPdo.CODE_IR_BLOCKED)
            self._send(COB_EMCY_BASE, EmcyPdo(code, 0x81, self.state,
                                              self.faults).encode())

    # -- transmit -----------------------------------------------------------
    def _tx_loop(self) -> None:
        t_status = t_diag = t_hb = 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            if now - t_status >= 1.0 / STATUS_HZ:
                t_status = now
                with self._lock:
                    pdo = StatusPdo(int(self.v_meas), int(self.w_meas), self.state,
                                    self.faults, self.seq_echo, self.ir_mask)
                self._send(COB_TPDO1_BASE, pdo.encode())
            if now - t_diag >= 1.0 / DIAG_HZ:
                t_diag = now
                with self._lock:
                    diag = DiagnosticsPdo(
                        # Plausible figures for a simulator; the real numbers
                        # come from the firmware's own histogram.
                        jitter_p99_us=random.randint(8, 22),
                        period_max_us=1000 + random.randint(10, 40),
                        rpdo_age_ms=int(min(self.rpdo_age_ms, 65535)),
                        ctrl_cpu_pct=random.randint(18, 26),
                        comms_cpu_pct=random.randint(4, 12))
                self._send(COB_TPDO2_BASE, diag.encode())
            if now - t_hb >= 1.0 / HEARTBEAT_HZ:
                t_hb = now
                self._send(COB_HEARTBEAT_BASE, bytes([int(self.nmt)]))
            time.sleep(0.002)

    # -- lifecycle ----------------------------------------------------------
    def start(self) -> None:
        self._stop.clear()
        self._threads = [
            threading.Thread(target=self._rx_loop, name="sim-rx", daemon=True),
            threading.Thread(target=self._control_loop, name="sim-ctrl", daemon=True),
            threading.Thread(target=self._tx_loop, name="sim-tx", daemon=True),
        ]
        for t in self._threads:
            t.start()
        self._send(COB_HEARTBEAT_BASE, bytes([NmtState.BOOTUP]))

    def stop(self) -> None:
        self._stop.set()
        for t in getattr(self, "_threads", []):
            t.join(timeout=1.0)

    def trip_ir(self, mask: int) -> None:
        """Assert the IR safety inputs, as a real obstacle would."""
        with self._lock:
            self.ir_mask = mask & 0xFF

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exc):
        self.stop()


def main() -> int:
    ap = argparse.ArgumentParser(description="Simulated ESP32 CANopen drive node")
    ap.add_argument("--interface", default="socketcan")
    ap.add_argument("--channel", default="vcan0")
    ap.add_argument("--bitrate", type=int, default=500000)
    ap.add_argument("--node-id", type=lambda s: int(s, 0), default=0x22)
    ap.add_argument("--watchdog-ms", type=int, default=150)
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)-7s %(name)s %(message)s")
    from stereolink.canlink import PythonCanTransport
    transport = PythonCanTransport(args.interface, args.channel, args.bitrate)
    node = Esp32NodeSim(transport, args.node_id, args.watchdog_ms)
    log.info("simulated node 0x%02X on %s/%s", args.node_id, args.interface, args.channel)
    with node:
        try:
            while True:
                time.sleep(1.0)
                log.info("state=%-9s v=%6.0f mm/s rpdo_age=%5.0f ms faults=%s",
                         node.state.name, node.v_meas, node.rpdo_age_ms, node.faults)
        except KeyboardInterrupt:
            log.info("stopping")
    return 0


if __name__ == "__main__":
    sys.exit(main())
