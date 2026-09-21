"""CANopen master for the Pi side of the link.

Deliberately not built on a full CANopen stack.  The link carries four PDOs and
an NMT/heartbeat pair; a full object-dictionary stack would add SDO negotiation
and a threading model we would then have to characterise for latency.  What is
implemented here is the part of CiA 301 the link actually uses, with the frame
IDs and semantics kept standard so a CANopen analyser understands the bus.

The transport is pluggable: ``socketcan`` on the Pi, ``virtual`` for a desktop
with python-can installed, and a dependency-free in-process ``loopback`` used by
the tests and by ``tools/esp32_sim.py``.
"""

from __future__ import annotations

import logging
import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Callable

from .pdo import (
    COB_EMCY_BASE, COB_HEARTBEAT_BASE, COB_NMT, COB_RPDO1_BASE, COB_RPDO2_BASE,
    COB_SYNC, COB_TPDO1_BASE, COB_TPDO2_BASE, COB_TPDO3_BASE, DiagnosticsPdo,
    DriveState, EmcyPdo, FaultFlags, LatencyEchoPdo, LimitsPdo, NmtCommand,
    NmtState, SetpointPdo, StatusPdo, cob,
)

log = logging.getLogger(__name__)


@dataclass
class CanFrame:
    arbitration_id: int
    data: bytes
    timestamp: float = 0.0
    is_extended_id: bool = False


class Transport:
    """Minimal send/recv interface shared by all backends."""

    def send(self, frame: CanFrame) -> None:
        raise NotImplementedError

    def recv(self, timeout: float) -> CanFrame | None:
        raise NotImplementedError

    def close(self) -> None:
        pass


class LoopbackTransport(Transport):
    """In-process bus. Frames sent by one endpoint appear at the other.

    Used to run the full Pi<->node protocol, including the watchdog behaviour,
    with no kernel CAN and no python-can.  ``drop_tx`` is the hook the failsafe
    demo uses to simulate a severed cable.
    """

    def __init__(self):
        self._a: queue.Queue[CanFrame] = queue.Queue()
        self._b: queue.Queue[CanFrame] = queue.Queue()
        # Directional cuts, so a test can sever master->node (the case the
        # firmware watchdog exists for) without also blinding the master to
        # the node's heartbeat - which is what makes the failure observable.
        self.drop: dict[str, bool] = {"master": False, "node": False}
        self.tx_count = 0
        self.dropped_count = 0

    def cut(self, side: str = "master", dropped: bool = True) -> None:
        self.drop[side] = dropped

    def restore(self) -> None:
        self.drop = {"master": False, "node": False}

    def endpoint(self, side: str) -> "Transport":
        tx, rx = (self._a, self._b) if side == "master" else (self._b, self._a)
        return _LoopbackEndpoint(self, side, tx, rx)

    def close(self) -> None:
        pass


class _LoopbackEndpoint(Transport):
    def __init__(self, bus: LoopbackTransport, side: str,
                 tx: queue.Queue, rx: queue.Queue):
        self._bus, self._side, self._tx, self._rx = bus, side, tx, rx

    def send(self, frame: CanFrame) -> None:
        if self._bus.drop.get(self._side, False):
            self._bus.dropped_count += 1
            return
        frame.timestamp = time.monotonic()
        self._bus.tx_count += 1
        self._tx.put(frame)

    def recv(self, timeout: float) -> CanFrame | None:
        try:
            return self._rx.get(timeout=timeout)
        except queue.Empty:
            return None


class PythonCanTransport(Transport):
    """socketcan / virtual / any python-can backend."""

    def __init__(self, interface: str, channel: str, bitrate: int):
        try:
            import can  # noqa: F401
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError(
                "python-can is required for the '%s' interface; "
                "install it (pip install python-can) or use interface: loopback"
                % interface) from exc
        import can
        kwargs = {"interface": interface, "channel": channel}
        if interface not in ("virtual",):
            kwargs["bitrate"] = bitrate
        self._bus = can.Bus(**kwargs)

    def send(self, frame: CanFrame) -> None:
        import can
        self._bus.send(can.Message(arbitration_id=frame.arbitration_id,
                                   data=frame.data, is_extended_id=False))

    def recv(self, timeout: float) -> CanFrame | None:
        msg = self._bus.recv(timeout=timeout)
        if msg is None:
            return None
        return CanFrame(msg.arbitration_id, bytes(msg.data),
                        msg.timestamp or time.monotonic(), msg.is_extended_id)

    def close(self) -> None:
        try:
            self._bus.shutdown()
        except Exception:  # pragma: no cover
            pass


def make_transport(cfg) -> Transport:
    if cfg.interface == "loopback":
        bus = LoopbackTransport()
        return bus.endpoint("master")
    return PythonCanTransport(cfg.interface, cfg.channel, cfg.bitrate)


@dataclass
class NodeView:
    """Everything the master believes about the node right now."""

    status: StatusPdo | None = None
    diagnostics: DiagnosticsPdo | None = None
    last_echo: LatencyEchoPdo | None = None
    last_emcy: EmcyPdo | None = None
    nmt_state: NmtState | None = None
    t_status: float = 0.0
    t_heartbeat: float = 0.0
    heartbeat_count: int = 0
    emcy_count: int = 0
    rx_frames: int = 0

    def online(self, timeout_s: float, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else now
        return self.t_heartbeat > 0.0 and (now - self.t_heartbeat) < timeout_s


class CanOpenMaster:
    """Sends setpoints, consumes node telemetry, tracks node liveness.

    Receive runs on its own thread so that a slow north-bound publish or a long
    SGBM frame can never delay draining the RX queue - a full queue would drop
    the very heartbeats we use to detect that the node is gone.
    """

    def __init__(self, cfg, transport: Transport | None = None,
                 latency: "object | None" = None):
        self.cfg = cfg
        self.node_id = cfg.node_id
        self.transport = transport if transport is not None else make_transport(cfg)
        self.latency = latency
        self.node = NodeView()
        self._seq = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._rx_thread: threading.Thread | None = None
        self._sync_thread: threading.Thread | None = None
        self._on_status: list[Callable[[StatusPdo], None]] = []
        self._on_emcy: list[Callable[[EmcyPdo], None]] = []
        self.tx_count = 0
        self.rx_unknown = 0

        self._id_status = cob(COB_TPDO1_BASE, self.node_id)
        self._id_diag = cob(COB_TPDO2_BASE, self.node_id)
        self._id_echo = cob(COB_TPDO3_BASE, self.node_id)
        self._id_hb = cob(COB_HEARTBEAT_BASE, self.node_id)
        self._id_emcy = cob(COB_EMCY_BASE, self.node_id)
        self._id_setpoint = cob(COB_RPDO1_BASE, self.node_id)
        self._id_limits = cob(COB_RPDO2_BASE, self.node_id)

    # -- lifecycle ----------------------------------------------------------
    def start(self) -> None:
        self._stop.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, name="can-rx", daemon=True)
        self._rx_thread.start()
        if self.cfg.send_sync and self.cfg.sync_hz > 0:
            self._sync_thread = threading.Thread(target=self._sync_loop, name="can-sync",
                                                 daemon=True)
            self._sync_thread.start()

    def stop(self) -> None:
        self._stop.set()
        for t in (self._rx_thread, self._sync_thread):
            if t is not None:
                t.join(timeout=1.0)
        self.transport.close()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exc):
        self.stop()

    # -- transmit -----------------------------------------------------------
    def _send(self, can_id: int, data: bytes) -> None:
        self.transport.send(CanFrame(can_id, data, time.monotonic()))
        self.tx_count += 1

    def nmt(self, command: NmtCommand) -> None:
        """NMT frames are broadcast on 0x000 with (command, target node)."""
        self._send(COB_NMT, bytes((int(command), self.node_id)))

    def start_node(self) -> None:
        self.nmt(NmtCommand.START)

    def stop_node(self) -> None:
        self.nmt(NmtCommand.STOP)

    def reset_node(self) -> None:
        self.nmt(NmtCommand.RESET_NODE)

    def sync(self) -> None:
        self._send(COB_SYNC, b"")

    def send_limits(self, limits: LimitsPdo) -> None:
        self._send(self._id_limits, limits.encode())

    def next_seq(self) -> int:
        with self._lock:
            self._seq = (self._seq + 1) & 0xFF
            return self._seq

    def send_setpoint(self, sp: SetpointPdo, t_capture: float | None = None) -> int:
        """Transmit RPDO1 and register the seq for latency closure."""
        self._send(self._id_setpoint, sp.encode())
        t_tx = time.monotonic()
        if self.latency is not None and t_capture is not None:
            self.latency.mark_sent(sp.seq, t_capture, t_tx)
        return sp.seq

    # -- receive ------------------------------------------------------------
    def on_status(self, fn: Callable[[StatusPdo], None]) -> None:
        self._on_status.append(fn)

    def on_emcy(self, fn: Callable[[EmcyPdo], None]) -> None:
        self._on_emcy.append(fn)

    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            try:
                frame = self.transport.recv(timeout=0.1)
            except Exception:  # pragma: no cover - driver/bus teardown
                log.exception("CAN receive failed")
                time.sleep(0.05)
                continue
            if frame is None:
                continue
            try:
                self._dispatch(frame)
            except Exception:  # pragma: no cover
                log.exception("bad frame 0x%03X %s", frame.arbitration_id,
                              frame.data.hex())

    def _dispatch(self, frame: CanFrame) -> None:
        now = time.monotonic()
        cid = frame.arbitration_id
        self.node.rx_frames += 1

        if cid == self._id_status:
            st = StatusPdo.decode(frame.data)
            self.node.status = st
            self.node.t_status = now
            # TPDO1 also closes the latency loop; TPDO3 carries the node-local
            # span but TPDO1 arrives first and is the timestamp that counts.
            if self.latency is not None:
                self.latency.mark_echo(st.seq_echo, now)
            for fn in self._on_status:
                fn(st)

        elif cid == self._id_diag:
            self.node.diagnostics = DiagnosticsPdo.decode(frame.data)

        elif cid == self._id_echo:
            echo = LatencyEchoPdo.decode(frame.data)
            self.node.last_echo = echo
            if self.latency is not None:
                # Feeds the node-local apply span in and derives the wire time
                # without re-closing the round trip (TPDO1 already popped seq).
                self.latency.note_node_apply(echo.rx_to_apply_us)

        elif cid == self._id_hb:
            self.node.t_heartbeat = now
            self.node.heartbeat_count += 1
            if frame.data:
                try:
                    self.node.nmt_state = NmtState(frame.data[0] & 0x7F)
                except ValueError:
                    self.node.nmt_state = None

        elif cid == self._id_emcy:
            emcy = EmcyPdo.decode(frame.data)
            self.node.last_emcy = emcy
            self.node.emcy_count += 1
            log.warning("EMCY from node 0x%02X: code=0x%04X state=%s faults=%s",
                        self.node_id, emcy.error_code, emcy.state.name, emcy.faults)
            for fn in self._on_emcy:
                fn(emcy)
        else:
            self.rx_unknown += 1

    def _sync_loop(self) -> None:
        period = 1.0 / self.cfg.sync_hz
        next_due = time.monotonic()
        while not self._stop.is_set():
            self.sync()
            next_due += period
            delay = next_due - time.monotonic()
            if delay < -period:
                next_due = time.monotonic()
                delay = 0.0
            self._stop.wait(max(delay, 0.0))
