# Architecture

## The shape of the system

```
                    IT layer  (best effort, may stall, may be absent)
        ┌───────────────────────────────────────────────────────┐
        │   MQTT broker / Sparkplug B host   •   OPC UA client   │
        └───────────────▲───────────────────────────▲───────────┘
                        │ spBv1.0/…/DDATA            │ opc.tcp://…
                        │ 5 Hz, QoS 0                │ 5 Hz
   ═════════════════════╪════════════════════════════╪══════════════  OT/IT
                        │                            │
        ┌───────────────┴────────────────────────────┴───────────┐
        │  Raspberry Pi — Linux, soft real time, ~10 Hz          │
        │                                                        │
        │  Waveshare binocular (one UVC device, both eyes)       │
        │      → split → downscale → rectify (pre-built maps)    │
        │      → StereoSGBM → depth → free-space columns         │
        │      → planner → (v, ω) setpoint                       │
        └────────────────────────┬───────────────────────────────┘
                                 │ CANopen, 500 kbit/s
                                 │ RPDO1 setpoint @ 10 Hz
                                 │ TPDO1/2/3 + heartbeat + EMCY
        ┌────────────────────────▼───────────────────────────────┐
        │  ESP32 — FreeRTOS, hard real time                      │
        │                                                        │
        │  core 1, prio 23 : control @ 1 kHz, watchdog, failsafe │
        │  core 0, prio 12 : CAN receive                         │
        │  core 0, prio 11 : CAN transmit                        │
        │  core 0, prio  6 : load generator (test fixture)       │
        │                                                        │
        │  IR sensors ──► fast safety layer (1 ms, independent)  │
        │  encoders  ──► PCNT ──► PI velocity loops ──► H-bridge │
        └────────────────────────────────────────────────────────┘
```

## The organising principle

Each layer must be safe against the failure of the layer above it.

| Layer | May fail how | Contained by |
|---|---|---|
| IT (MQTT/OPC UA) | broker down, network partition, slow consumer | Publishers are non-blocking and self-disable; the pipeline never waits on them |
| Vision (Pi) | bad disparity, crash, stall, slow frame | Planner de-rates on low `valid_fraction`; setpoints go stale and the node's watchdog fires |
| Link (CAN) | cable cut, bus-off, connector vibration | Node's setpoint watchdog → ramp to zero → latched `SAFE_STOP` |
| Control (ESP32) | — | IR safety layer, output disable, task watchdog |

Reading upward: nothing above the ESP32 can cause an unsafe state by being
absent. That is the property the whole design exists to deliver, and it is why
the watchdog lives in firmware rather than in the Pi's code.

## Why the split is where it is

**Vision on Linux.** SGBM needs the memory bandwidth and the SIMD. It is also
the part most likely to need changing, and changing it must not mean touching
anything safety-relevant.

**Control on a microcontroller.** A 1 kHz loop with microsecond jitter is not
something a general-purpose kernel gives you without real effort. An ESP32
timer interrupt gives it for free.

**CANopen between them.** Not because the data rate demands it — the setpoint
is 8 bytes at 10 Hz — but because of what the protocol brings with it:
node states, heartbeats, emergency objects, and a bus that behaves predictably
under fault. The link is the safety boundary, so it should be a link with
defined failure semantics.

**Sparkplug B or OPC UA north.** Both are implemented because which one you
get is decided by the customer's existing infrastructure, not by the device.
Sparkplug suits report-by-exception through a broker; OPC UA suits a SCADA
client that wants to browse an address space.

## Data flow for one frame

1. `camera.py` grabs a side-by-side frame, timestamps it (`t_capture`), splits the eyes.
2. `pipeline.py` downscales to the working resolution (`INTER_AREA`).
3. `calibration.Rectifier` remaps both eyes using maps built once at startup.
4. `disparity.py` runs StereoSGBM; fixed-point output becomes float pixels.
5. `Z = fB/d` gives metric depth.
6. `obstacle.py` reduces the depth map to N per-column ranges, rejecting the
   ground plane geometrically, and picks the widest usable corridor.
7. `planner.py` turns that into `(v, ω)`, de-rating for distance and
   slew-limiting yaw.
8. `canlink.py` encodes RPDO1 with a one-byte `seq` and sends it.
9. The node consumes it within one control tick and echoes `seq` back.
10. `latency.py` closes the loop and attributes time to each stage.

## Threading

| Where | Thread/task | Priority | Job |
|---|---|---|---|
| Pi | main | normal | The whole vision→setpoint path |
| Pi | `can-rx` | normal | Drains the CAN socket, updates `NodeView` |
| Pi | `can-sync` | normal | Emits SYNC |
| Pi | paho / asyncua | normal | North-bound publishing |
| ESP32 | `control` | 23, core 1 | 1 kHz loop, watchdog, failsafe |
| ESP32 | `can_rx` | 12, core 0 | TWAI receive → mailbox |
| ESP32 | `can_tx` | 11, core 0 | TPDOs, heartbeat, EMCY |
| ESP32 | `loadgen` | 6, core 0 | Test fixture only |
| ESP32 | `stats` | 5, core 0 | Logging |

On the Pi, CAN receive is on its own thread specifically so that a long SGBM
frame or a slow north-bound publish cannot delay draining the socket — a full
queue would drop the heartbeats used to detect that the node is gone.

On the ESP32, the setpoint crosses from `can_rx` to `control` through a
seqlock ([`setpoint_mailbox.h`](../esp32/main/setpoint_mailbox.h)), never a
mutex. A mutex would let a priority-12 task block a priority-23 task — the
exact priority inversion the split exists to prevent.

## Source map

| Path | What lives there |
|---|---|
| [`pi/stereolink/pdo.py`](../pi/stereolink/pdo.py) | Normative wire format |
| [`pi/stereolink/camera.py`](../pi/stereolink/camera.py) | V4L2 + synthetic stereo sources |
| [`pi/stereolink/calibration.py`](../pi/stereolink/calibration.py) | Calibration I/O, rectification maps |
| [`pi/stereolink/disparity.py`](../pi/stereolink/disparity.py) | StereoSGBM wrapper |
| [`pi/stereolink/obstacle.py`](../pi/stereolink/obstacle.py) | Depth → free-space profile |
| [`pi/stereolink/planner.py`](../pi/stereolink/planner.py) | Free space → `(v, ω)` |
| [`pi/stereolink/canlink.py`](../pi/stereolink/canlink.py) | CANopen master, transports |
| [`pi/stereolink/latency.py`](../pi/stereolink/latency.py) | Per-stage budget, seq closure |
| [`pi/stereolink/north/`](../pi/stereolink/north/) | Sparkplug B, MQTT, OPC UA |
| [`esp32/main/control.c`](../esp32/main/control.c) | Control loop, state machine, failsafe |
| [`esp32/main/canopen.c`](../esp32/main/canopen.c) | TWAI, PDO handling |
| [`esp32/main/pdo_defs.h`](../esp32/main/pdo_defs.h) | C mirror of the wire format |
