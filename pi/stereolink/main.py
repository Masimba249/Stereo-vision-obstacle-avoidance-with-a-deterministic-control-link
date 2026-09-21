"""CLI entry point: ``python -m stereolink.main [--config config/pi.yml]``."""

from __future__ import annotations

import argparse
import logging
import sys

from .config import AppConfig
from .pipeline import Pipeline


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="stereolink",
        description="Stereo obstacle avoidance with a CANopen control link")
    p.add_argument("-c", "--config", default=None, help="YAML config file")
    p.add_argument("--source", choices=("v4l2", "synthetic"),
                   help="override camera source")
    p.add_argument("--can-interface", help="socketcan | virtual | loopback")
    p.add_argument("--can-channel", help="e.g. can0 or vcan0")
    p.add_argument("--no-mqtt", action="store_true", help="disable Sparkplug B")
    p.add_argument("--opcua", action="store_true", help="enable the OPC UA server")
    p.add_argument("--preview", action="store_true", help="show a debug window")
    p.add_argument("--frames", type=int, default=None,
                   help="process N frames then exit (benchmarks/CI)")
    p.add_argument("--log-level", default=None)
    return p


def apply_overrides(cfg: AppConfig, args) -> AppConfig:
    if args.source:
        cfg.camera.source = args.source
    if args.can_interface:
        cfg.can.interface = args.can_interface
    if args.can_channel:
        cfg.can.channel = args.can_channel
    if args.no_mqtt:
        cfg.mqtt.enabled = False
    if args.opcua:
        cfg.opcua.enabled = True
    if args.preview:
        cfg.preview = True
    if args.log_level:
        cfg.log_level = args.log_level
    return cfg


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    cfg = apply_overrides(AppConfig.load(args.config), args)

    logging.basicConfig(
        level=getattr(logging, cfg.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)-7s %(name)-22s %(message)s")
    log = logging.getLogger("stereolink")

    try:
        pipeline = Pipeline(cfg)
    except RuntimeError as exc:
        log.error("%s", exc)
        return 2

    log.info("starting: camera=%s can=%s/%s mqtt=%s opcua=%s",
             cfg.camera.source, cfg.can.interface, cfg.can.channel,
             cfg.mqtt.enabled, cfg.opcua.enabled)
    try:
        pipeline.run(max_frames=args.frames)
    except KeyboardInterrupt:
        log.info("interrupted")
    except Exception:
        log.exception("pipeline failed")
        return 1

    if pipeline.latency is not None:
        print()
        print(pipeline.latency.to_markdown())
    log.info("processed %d frames (%d dropped)",
             pipeline.stats.frames, pipeline.stats.dropped)
    return 0


if __name__ == "__main__":
    sys.exit(main())
