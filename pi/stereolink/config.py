"""Configuration for the whole Pi-side pipeline, loaded from one YAML file."""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

try:
    import yaml
except ImportError:  # pragma: no cover - yaml is optional for the sim path
    yaml = None


@dataclass
class CameraConfig:
    source: str = "v4l2"          # "v4l2" | "synthetic"
    device: int | str = 0
    # The Waveshare binocular module enumerates as ONE UVC device that streams
    # the two sensors side by side, so the capture width is 2x the eye width.
    capture_width: int = 2560
    capture_height: int = 720
    fps: int = 30
    fourcc: str = "MJPG"
    swap_eyes: bool = False
    # Vision runs slower than capture on purpose: we always grab the newest
    # frame and drop the backlog rather than processing a stale queue.
    process_hz: float = 10.0


@dataclass
class StereoConfig:
    # Downscale before SGBM.  This is the single biggest latency lever;
    # see docs/latency-budget.md for the measured trade.
    work_width: int = 640
    work_height: int = 360
    min_disparity: int = 0
    num_disparities: int = 96     # must be divisible by 16
    block_size: int = 5
    uniqueness_ratio: int = 10
    speckle_window_size: int = 100
    speckle_range: int = 2
    disp12_max_diff: int = 1
    pre_filter_cap: int = 31
    p1_factor: int = 8            # P1 = factor * channels * block^2
    p2_factor: int = 32
    mode: str = "SGBM_3WAY"       # SGBM | HH | SGBM_3WAY
    wls_filter: bool = False      # needs opencv-contrib; costs ~6 ms at 640x360


@dataclass
class ObstacleConfig:
    num_columns: int = 16         # steering columns across the image
    roi_top: float = 0.35         # ignore sky/ceiling above this fraction
    roi_bottom: float = 0.92      # ignore the chassis / immediate ground
    min_valid_px: int = 60        # per column, below this the column is unknown
    percentile: float = 12.0      # robust "nearest" depth per column
    min_range_m: float = 0.25
    max_range_m: float = 6.0
    ground_reject_m: float = 0.08  # drop points within this of the ground plane
    camera_height_m: float = 0.16
    temporal_alpha: float = 0.6   # EMA on per-column distance


@dataclass
class PlannerConfig:
    v_cruise_mm_s: float = 700.0
    v_min_mm_s: float = 120.0
    stop_distance_m: float = 0.45
    slow_distance_m: float = 1.60
    clear_distance_m: float = 3.0
    w_max_mrad_s: float = 2200.0
    steer_gain: float = 2600.0    # mrad/s per unit of normalised bearing error
    steer_slew_mrad_s2: float = 9000.0
    reverse_on_blocked: bool = False
    vision_timeout_s: float = 0.5  # stale vision -> creep/stop, flags cleared


@dataclass
class CanConfig:
    interface: str = "socketcan"  # socketcan | virtual | loopback
    channel: str = "can0"
    bitrate: int = 500000
    node_id: int = 0x22
    send_sync: bool = True
    sync_hz: float = 10.0
    heartbeat_timeout_s: float = 0.6
    # Mirrors the firmware watchdog so both ends agree on what "lost" means.
    setpoint_watchdog_ms: int = 150


@dataclass
class MqttConfig:
    enabled: bool = True
    host: str = "localhost"
    port: int = 1883
    username: str | None = None
    password: str | None = None
    group_id: str = "EdgeRobotics"
    edge_node_id: str = "pi-stereo-01"
    device_id: str = "esp32-drive"
    keepalive: int = 30
    publish_hz: float = 5.0


@dataclass
class OpcUaConfig:
    enabled: bool = False
    endpoint: str = "opc.tcp://0.0.0.0:4840/stereolink/server/"
    uri: str = "http://example.org/stereolink/"
    name: str = "StereoLink Edge Server"
    publish_hz: float = 5.0


@dataclass
class LatencyConfig:
    enabled: bool = True
    window: int = 600             # rolling samples kept per stage
    report_path: str = "reports/latency.json"
    markdown_path: str = "reports/latency-budget.md"
    report_every_s: float = 30.0


@dataclass
class AppConfig:
    camera: CameraConfig = field(default_factory=CameraConfig)
    stereo: StereoConfig = field(default_factory=StereoConfig)
    obstacle: ObstacleConfig = field(default_factory=ObstacleConfig)
    planner: PlannerConfig = field(default_factory=PlannerConfig)
    can: CanConfig = field(default_factory=CanConfig)
    mqtt: MqttConfig = field(default_factory=MqttConfig)
    opcua: OpcUaConfig = field(default_factory=OpcUaConfig)
    latency: LatencyConfig = field(default_factory=LatencyConfig)
    calibration_path: str = "config/stereo_calibration.yml"
    log_level: str = "INFO"
    preview: bool = False

    @classmethod
    def load(cls, path: str | Path | None) -> "AppConfig":
        if path is None:
            return cls()
        if yaml is None:
            raise RuntimeError("PyYAML is required to load a config file")
        with open(path, "r", encoding="utf-8") as fh:
            raw = yaml.safe_load(fh) or {}
        return cls.from_dict(raw)

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> "AppConfig":
        kwargs: dict[str, Any] = {}
        for f in dataclasses.fields(cls):
            if f.name not in raw:
                continue
            value = raw[f.name]
            if dataclasses.is_dataclass(f.type) or isinstance(value, dict):
                sub = _sub_type(cls, f.name)
                kwargs[f.name] = sub(**value) if sub is not None else value
            else:
                kwargs[f.name] = value
        return cls(**kwargs)

    def to_dict(self) -> dict[str, Any]:
        return dataclasses.asdict(self)


_SUBTYPES = {
    "camera": CameraConfig, "stereo": StereoConfig, "obstacle": ObstacleConfig,
    "planner": PlannerConfig, "can": CanConfig, "mqtt": MqttConfig,
    "opcua": OpcUaConfig, "latency": LatencyConfig,
}


def _sub_type(_cls, name: str):
    return _SUBTYPES.get(name)
