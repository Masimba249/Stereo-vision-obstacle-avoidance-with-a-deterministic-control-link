#!/usr/bin/env python3
"""Subscribe to the edge node's Sparkplug B topics and print decoded metrics.

Stands in for a SCADA client, so the north-bound payloads can be verified
without deploying Ignition or a full MQTT historian.

    python3 pi/tools/sparkplug_listen.py --host localhost
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from stereolink.north.sparkplug_b import decode_payload  # noqa: E402
from stereolink.north.mqtt_north import METRIC_SPEC  # noqa: E402

ALIAS_TO_NAME = {alias: name for name, (alias, _) in METRIC_SPEC.items()}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--group", default="EdgeRobotics")
    args = ap.parse_args()

    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        print("paho-mqtt is required: pip install paho-mqtt")
        return 2

    topic = f"spBv1.0/{args.group}/#"

    def on_connect(client, userdata, flags, rc, properties=None):
        print(f"connected, subscribing to {topic}")
        client.subscribe(topic, qos=0)

    def on_message(client, userdata, msg):
        kind = msg.topic.split("/")[2]
        try:
            payload = decode_payload(msg.payload)
        except Exception as exc:
            print(f"{msg.topic}: undecodable ({exc})")
            return
        print(f"\n[{kind}] {msg.topic}  seq={payload['seq']} "
              f"({len(payload['metrics'])} metrics, {len(msg.payload)} bytes)")
        for m in payload["metrics"]:
            # DDATA is alias-only by design, so resolve names from the BIRTH
            # metric map - which is exactly what a real host does.
            name = m["name"] or ALIAS_TO_NAME.get(m["alias"], f"alias:{m['alias']}")
            value = "null" if m["is_null"] else m["value"]
            if isinstance(value, float):
                value = f"{value:.3f}"
            print(f"    {name:32} = {value}")

    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except (AttributeError, TypeError):
        client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(args.host, args.port, 30)
    try:
        client.loop_forever()
    except KeyboardInterrupt:
        print("\nstopping")
    return 0


if __name__ == "__main__":
    sys.exit(main())
