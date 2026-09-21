"""Sparkplug B edge node over MQTT.

Implements the parts of the Sparkplug B session model that make a node behave
correctly in a real SCADA deployment:

* NDEATH is registered as the MQTT Will *before* CONNECT, so an ungraceful
  drop (which is the normal way an edge node dies) still marks the node
  offline.  Publishing a death message on the way out only covers the case
  where nothing went wrong.
* NBIRTH is seq 0 and declares every metric with its alias; DDATA afterwards
  sends aliases only.
* ``bdSeq`` increments per connection so the host can tell a reconnect from a
  fresh boot.
* NCMD/DCMD are subscribed, which is how the north-bound layer is allowed to
  request a stop.  Note the command path is advisory: it sets an e-stop
  request flag that the planner honours, and the firmware's own safety layer
  remains the thing that guarantees the motors actually stop.
"""

from __future__ import annotations

import logging
import threading
import time

from .sparkplug_b import DataType, Metric, Payload, topic

log = logging.getLogger(__name__)

# name -> (alias, datatype).  Aliases are stable for the life of the build;
# changing one without a new NBIRTH would silently mislabel data at the host.
METRIC_SPEC: dict[str, tuple[int, DataType]] = {
    "Vision/min_range_m":      (1, DataType.Float),
    "Vision/min_range_bearing": (2, DataType.Float),
    "Vision/best_bearing":     (3, DataType.Float),
    "Vision/valid_fraction":   (4, DataType.Float),
    "Vision/fps":              (5, DataType.Float),
    "Vision/blocked":          (6, DataType.Boolean),
    "Setpoint/v_mm_s":         (10, DataType.Int16),
    "Setpoint/w_mrad_s":       (11, DataType.Int16),
    "Setpoint/reason":         (12, DataType.String),
    "Drive/v_meas_mm_s":       (20, DataType.Int16),
    "Drive/w_meas_mrad_s":     (21, DataType.Int16),
    "Drive/state":             (22, DataType.String),
    "Drive/faults":            (23, DataType.UInt8),
    "Drive/ir_mask":           (24, DataType.UInt8),
    "Link/online":             (30, DataType.Boolean),
    "Link/heartbeats":         (31, DataType.UInt32),
    "Link/emcy_count":         (32, DataType.UInt32),
    "Link/rpdo_age_ms":        (33, DataType.UInt16),
    "Diag/jitter_p99_us":      (40, DataType.UInt16),
    "Diag/period_max_us":      (41, DataType.UInt16),
    "Diag/ctrl_cpu_pct":       (42, DataType.UInt8),
    "Diag/comms_cpu_pct":      (43, DataType.UInt8),
    "Latency/e2e_p50_ms":      (50, DataType.Float),
    "Latency/e2e_p95_ms":      (51, DataType.Float),
    "Latency/roundtrip_p95_ms": (52, DataType.Float),
    "Latency/node_apply_p95_ms": (53, DataType.Float),
}

_BD_SEQ_ALIAS = 0


class SparkplugPublisher:
    def __init__(self, cfg, on_command=None):
        self.cfg = cfg
        self.on_command = on_command
        self._client = None
        self._seq = 0
        self._bd_seq = 0
        self._connected = threading.Event()
        self._lock = threading.Lock()
        self._birth_sent = False
        self.publish_count = 0
        self.enabled = bool(cfg.enabled)

        self._t_node = topic(cfg.group_id, "NBIRTH", cfg.edge_node_id)
        self._t_ndeath = topic(cfg.group_id, "NDEATH", cfg.edge_node_id)
        self._t_dbirth = topic(cfg.group_id, "DBIRTH", cfg.edge_node_id, cfg.device_id)
        self._t_ddata = topic(cfg.group_id, "DDATA", cfg.edge_node_id, cfg.device_id)
        self._t_ncmd = topic(cfg.group_id, "NCMD", cfg.edge_node_id)
        self._t_dcmd = topic(cfg.group_id, "DCMD", cfg.edge_node_id, cfg.device_id)

    # -- session ------------------------------------------------------------
    def start(self) -> bool:
        if not self.enabled:
            return False
        try:
            import paho.mqtt.client as mqtt
        except ImportError:
            log.warning("paho-mqtt not installed; north-bound MQTT disabled")
            self.enabled = False
            return False

        try:
            self._client = mqtt.Client(
                mqtt.CallbackAPIVersion.VERSION2,
                client_id=f"{self.cfg.edge_node_id}-{int(time.time())}")
        except (AttributeError, TypeError):  # paho 1.x
            self._client = mqtt.Client(client_id=f"{self.cfg.edge_node_id}")

        if self.cfg.username:
            self._client.username_pw_set(self.cfg.username, self.cfg.password)
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._client.on_message = self._on_message

        # Will must be set before connect() or it is not part of the session.
        self._bd_seq = (self._bd_seq + 1) & 0xFF
        will = Payload([Metric("bdSeq", self._bd_seq, DataType.UInt64,
                               alias=_BD_SEQ_ALIAS)], seq=None).encode()
        self._client.will_set(self._t_ndeath, will, qos=1, retain=False)

        try:
            self._client.connect(self.cfg.host, self.cfg.port, self.cfg.keepalive)
        except OSError as exc:
            log.warning("MQTT connect to %s:%d failed (%s); continuing without "
                        "north-bound telemetry", self.cfg.host, self.cfg.port, exc)
            self.enabled = False
            return False
        self._client.loop_start()
        return True

    def stop(self) -> None:
        if self._client is None:
            return
        try:
            if self._connected.is_set():
                # Graceful death: say so explicitly, then disconnect cleanly
                # so the broker does not also fire the Will.
                self._client.publish(
                    self._t_ndeath,
                    Payload([Metric("bdSeq", self._bd_seq, DataType.UInt64,
                                    alias=_BD_SEQ_ALIAS)]).encode(), qos=1)
                time.sleep(0.05)
            self._client.loop_stop()
            self._client.disconnect()
        except Exception:  # pragma: no cover
            log.exception("MQTT shutdown failed")

    # -- callbacks ----------------------------------------------------------
    def _on_connect(self, client, userdata, flags, reason_code, properties=None):
        rc = getattr(reason_code, "value", reason_code)
        if rc != 0:
            log.warning("MQTT connect refused: %s", reason_code)
            return
        log.info("MQTT connected to %s:%d", self.cfg.host, self.cfg.port)
        self._connected.set()
        client.subscribe([(self._t_ncmd, 1), (self._t_dcmd, 1)])
        self._publish_births()

    def _on_disconnect(self, client, userdata, *args):
        self._connected.clear()
        self._birth_sent = False
        log.warning("MQTT disconnected; will re-BIRTH on reconnect")

    def _on_message(self, client, userdata, msg):
        if self.on_command is None:
            return
        try:
            from .sparkplug_b import decode_payload
            payload = decode_payload(msg.payload)
        except Exception:
            log.exception("undecodable command payload on %s", msg.topic)
            return
        for metric in payload["metrics"]:
            # Command metrics from the host are untrusted input: we pass the
            # name/value up and let the application decide what it honours.
            try:
                self.on_command(metric.get("name"), metric.get("value"))
            except Exception:
                log.exception("command handler failed for %s", metric.get("name"))

    # -- publishing ---------------------------------------------------------
    def _next_seq(self) -> int:
        with self._lock:
            s = self._seq
            self._seq = (self._seq + 1) & 0xFF
            return s

    def _publish_births(self) -> None:
        # Sparkplug requires NBIRTH to carry seq 0; reset before building it.
        with self._lock:
            self._seq = 0
        now_ms = int(time.time() * 1000)
        node_metrics = [
            Metric("bdSeq", self._bd_seq, DataType.UInt64, alias=_BD_SEQ_ALIAS,
                   timestamp_ms=now_ms),
            Metric("Node Control/Rebirth", False, DataType.Boolean, alias=900,
                   timestamp_ms=now_ms),
            Metric("Properties/edge_node", self.cfg.edge_node_id, DataType.String,
                   alias=901, timestamp_ms=now_ms),
        ]
        self._raw_publish(self._t_node,
                          Payload(node_metrics, seq=self._next_seq(),
                                  timestamp_ms=now_ms).encode())

        # DBIRTH declares the full metric set with aliases and null-safe
        # defaults, which is what lets DDATA be alias-only afterwards.
        device_metrics = [
            Metric(name, None, dt, alias=alias, timestamp_ms=now_ms, is_null=True)
            for name, (alias, dt) in METRIC_SPEC.items()
        ]
        self._raw_publish(self._t_dbirth,
                          Payload(device_metrics, seq=self._next_seq(),
                                  timestamp_ms=now_ms).encode())
        self._birth_sent = True
        log.info("Sparkplug BIRTH published (%d metrics, bdSeq=%d)",
                 len(device_metrics), self._bd_seq)

    def publish(self, values: dict) -> None:
        """Publish a DDATA with whichever known metrics are present."""
        if not self.enabled or not self._connected.is_set() or not self._birth_sent:
            return
        now_ms = int(time.time() * 1000)
        metrics = []
        for name, value in values.items():
            spec = METRIC_SPEC.get(name)
            if spec is None or value is None:
                continue
            alias, dt = spec
            metrics.append(Metric(None, value, dt, alias=alias, timestamp_ms=now_ms))
        if not metrics:
            return
        payload = Payload(metrics, seq=self._next_seq(), timestamp_ms=now_ms)
        self._raw_publish(self._t_ddata, payload.encode(include_names=False))
        self.publish_count += 1

    def _raw_publish(self, topic_str: str, payload: bytes) -> None:
        # QoS 0: telemetry at 5 Hz is superseded by the next sample, and we
        # would rather drop a stale one than let the broker queue build up.
        try:
            self._client.publish(topic_str, payload, qos=0, retain=False)
        except Exception:  # pragma: no cover
            log.exception("MQTT publish to %s failed", topic_str)
