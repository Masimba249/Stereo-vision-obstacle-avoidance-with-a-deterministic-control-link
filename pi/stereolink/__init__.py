"""Stereo-vision obstacle avoidance over a deterministic CANopen control link.

Layering, which is the point of the project:

    stereolink.camera / calibration / disparity / obstacle / planner
        the perception + advisory-planning stack (soft real time, ~10 Hz)
    stereolink.canlink / pdo
        the OT boundary: a deterministic, watchdogged fieldbus link
    stereolink.north
        the IT boundary: Sparkplug B over MQTT, and OPC UA
    stereolink.latency
        the measurement that proves the pipeline meets its budget
"""

__version__ = "1.0.0"
