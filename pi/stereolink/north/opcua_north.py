"""OPC UA server exposing the same telemetry as the Sparkplug publisher.

MQTT/Sparkplug suits a broker-based, report-by-exception architecture; OPC UA
suits a SCADA/HMI client that wants to browse an address space and subscribe.
Both are offered because which one you get is usually decided by the customer's
existing infrastructure, not by the edge device.

Uses ``asyncua`` (the maintained successor to ``python-opcua``); the server runs
its event loop on a dedicated thread so the 10 Hz vision pipeline stays on the
main thread and is never blocked by a client's browse or subscription work.
"""

from __future__ import annotations

import asyncio
import logging
import threading

log = logging.getLogger(__name__)

# Variable name -> (python type, initial value).  Mirrors METRIC_SPEC so the
# two north-bound protocols cannot drift apart in what they expose.
VARIABLES: dict[str, tuple[type, object]] = {
    "min_range_m": (float, 0.0),
    "min_range_bearing": (float, 0.0),
    "best_bearing": (float, 0.0),
    "valid_fraction": (float, 0.0),
    "vision_fps": (float, 0.0),
    "blocked": (bool, False),
    "setpoint_v_mm_s": (int, 0),
    "setpoint_w_mrad_s": (int, 0),
    "setpoint_reason": (str, "init"),
    "drive_v_meas_mm_s": (int, 0),
    "drive_w_meas_mrad_s": (int, 0),
    "drive_state": (str, "INIT"),
    "drive_faults": (int, 0),
    "drive_ir_mask": (int, 0),
    "link_online": (bool, False),
    "link_heartbeats": (int, 0),
    "link_emcy_count": (int, 0),
    "link_rpdo_age_ms": (int, 0),
    "jitter_p99_us": (int, 0),
    "period_max_us": (int, 0),
    "ctrl_cpu_pct": (int, 0),
    "comms_cpu_pct": (int, 0),
    "e2e_p50_ms": (float, 0.0),
    "e2e_p95_ms": (float, 0.0),
    "roundtrip_p95_ms": (float, 0.0),
    "node_apply_p95_ms": (float, 0.0),
}

# Sparkplug metric name -> OPC UA browse name, so callers pass one dict.
SPARKPLUG_TO_OPCUA = {
    "Vision/min_range_m": "min_range_m",
    "Vision/min_range_bearing": "min_range_bearing",
    "Vision/best_bearing": "best_bearing",
    "Vision/valid_fraction": "valid_fraction",
    "Vision/fps": "vision_fps",
    "Vision/blocked": "blocked",
    "Setpoint/v_mm_s": "setpoint_v_mm_s",
    "Setpoint/w_mrad_s": "setpoint_w_mrad_s",
    "Setpoint/reason": "setpoint_reason",
    "Drive/v_meas_mm_s": "drive_v_meas_mm_s",
    "Drive/w_meas_mrad_s": "drive_w_meas_mrad_s",
    "Drive/state": "drive_state",
    "Drive/faults": "drive_faults",
    "Drive/ir_mask": "drive_ir_mask",
    "Link/online": "link_online",
    "Link/heartbeats": "link_heartbeats",
    "Link/emcy_count": "link_emcy_count",
    "Link/rpdo_age_ms": "link_rpdo_age_ms",
    "Diag/jitter_p99_us": "jitter_p99_us",
    "Diag/period_max_us": "period_max_us",
    "Diag/ctrl_cpu_pct": "ctrl_cpu_pct",
    "Diag/comms_cpu_pct": "comms_cpu_pct",
    "Latency/e2e_p50_ms": "e2e_p50_ms",
    "Latency/e2e_p95_ms": "e2e_p95_ms",
    "Latency/roundtrip_p95_ms": "roundtrip_p95_ms",
    "Latency/node_apply_p95_ms": "node_apply_p95_ms",
}


class OpcUaPublisher:
    def __init__(self, cfg, on_command=None):
        self.cfg = cfg
        self.on_command = on_command
        self.enabled = bool(cfg.enabled)
        self._thread: threading.Thread | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._nodes: dict[str, object] = {}
        self._ready = threading.Event()
        self._stop = threading.Event()
        self._pending: dict[str, object] = {}
        self._lock = threading.Lock()

    def start(self) -> bool:
        if not self.enabled:
            return False
        try:
            import asyncua  # noqa: F401
        except ImportError:
            log.warning("asyncua not installed; OPC UA server disabled "
                        "(pip install asyncua)")
            self.enabled = False
            return False
        self._thread = threading.Thread(target=self._run, name="opcua", daemon=True)
        self._thread.start()
        # Bounded wait: a server that cannot bind must not stall the robot.
        if not self._ready.wait(timeout=10.0):
            log.warning("OPC UA server did not become ready in 10 s")
        return self._ready.is_set()

    def stop(self) -> None:
        self._stop.set()
        if self._loop is not None:
            self._loop.call_soon_threadsafe(lambda: None)
        if self._thread is not None:
            self._thread.join(timeout=5.0)

    def publish(self, values: dict) -> None:
        """Queue a batch of updates; the server thread applies them."""
        if not self.enabled or not self._ready.is_set():
            return
        with self._lock:
            for name, value in values.items():
                key = SPARKPLUG_TO_OPCUA.get(name, name)
                if key in VARIABLES and value is not None:
                    self._pending[key] = value

    # -- server thread ------------------------------------------------------
    def _run(self) -> None:
        try:
            asyncio.run(self._serve())
        except Exception:
            log.exception("OPC UA server thread died")
            self.enabled = False
            self._ready.set()  # unblock start()

    async def _serve(self) -> None:
        from asyncua import Server, ua

        server = Server()
        await server.init()
        server.set_endpoint(self.cfg.endpoint)
        server.set_server_name(self.cfg.name)
        # Anonymous + no encryption: correct for a lab demo, and explicitly
        # the thing to change before this faces a real plant network.
        server.set_security_policy([ua.SecurityPolicyType.NoSecurity])

        idx = await server.register_namespace(self.cfg.uri)
        objects = server.get_objects_node()
        device = await objects.add_object(idx, "StereoDrive")

        folders = {
            "Vision": await device.add_object(idx, "Vision"),
            "Setpoint": await device.add_object(idx, "Setpoint"),
            "Drive": await device.add_object(idx, "Drive"),
            "Link": await device.add_object(idx, "Link"),
            "Diag": await device.add_object(idx, "Diag"),
            "Latency": await device.add_object(idx, "Latency"),
        }

        def folder_for(key: str):
            for sp_name, ua_name in SPARKPLUG_TO_OPCUA.items():
                if ua_name == key:
                    return folders.get(sp_name.split("/")[0], device)
            return device

        for key, (typ, initial) in VARIABLES.items():
            var = await folder_for(key).add_variable(idx, key, initial)
            self._nodes[key] = var

        # One writable control point.  It is a *request*: the pipeline reads it
        # and commands zero, and the firmware failsafe is still what guarantees
        # the stop actually happens if this path is compromised or slow.
        estop = await device.add_variable(idx, "EStopRequest", False)
        await estop.set_writable()
        self._nodes["EStopRequest"] = estop

        async with server:
            log.info("OPC UA server listening on %s", self.cfg.endpoint)
            self._ready.set()
            period = 1.0 / max(self.cfg.publish_hz, 0.5)
            while not self._stop.is_set():
                await asyncio.sleep(period)
                with self._lock:
                    batch, self._pending = self._pending, {}
                for key, value in batch.items():
                    node = self._nodes.get(key)
                    if node is None:
                        continue
                    try:
                        await node.write_value(_coerce(key, value))
                    except Exception:
                        log.exception("OPC UA write failed for %s", key)
                if self.on_command is not None:
                    try:
                        if bool(await estop.read_value()):
                            self.on_command("EStopRequest", True)
                    except Exception:
                        log.exception("OPC UA e-stop read failed")


def _coerce(key: str, value):
    typ = VARIABLES.get(key, (None, None))[0]
    if typ is None:
        return value
    try:
        # OPC UA is strictly typed: writing an int into a Double variable
        # raises a BadTypeMismatch, so coerce to the declared type.
        return typ(value)
    except (TypeError, ValueError):
        return value
