# Stereo-vision obstacle avoidance with a deterministic control link

Heavy compute on Linux, deterministic control on a microcontroller, a fieldbus
between them, and an IT-layer protocol going north — the architecture most
industrial edge systems converge on.

A Raspberry Pi turns a binocular camera into a steering setpoint at 10 Hz. An
ESP32 closes a 1 kHz motor loop around that setpoint, and stops the machine by
itself when the Pi stops talking. Telemetry goes north over MQTT/Sparkplug B
or OPC UA.

```
 Waveshare binocular ─► Pi: rectify → StereoSGBM → depth → free space → (v,ω)
                             │
                             │  CANopen @ 500 kbit/s, RPDO1 @ 10 Hz
                             ▼
                        ESP32: 1 kHz PI loops, IR safety layer,
                               setpoint watchdog → ramp to zero
                             │
                             ▼  MQTT/Sparkplug B  ·  OPC UA
                        dashboard / SCADA
```

## What this demonstrates

| | Where |
|---|---|
| Sensor-to-actuator pipeline | [`pi/stereolink/pipeline.py`](pi/stereolink/pipeline.py) |
| Real-time task design | [`esp32/main/control.c`](esp32/main/control.c), [`docs/jitter-report.md`](docs/jitter-report.md) |
| Failsafe thinking | [`docs/failsafe.md`](docs/failsafe.md) |
| OT/IT boundary | [`pi/stereolink/north/`](pi/stereolink/north/), [`docs/canopen-profile.md`](docs/canopen-profile.md) |
| Latency measurement | [`docs/latency-budget.md`](docs/latency-budget.md) |

## Run it without hardware

Everything below works on a laptop. No camera, no ESP32, no CAN interface, no
MQTT broker.

```bash
pip install numpy opencv-python PyYAML pytest

# The headline demo: full pipeline, then cut the cable and watch the node
# bring itself to a stop on its own clock.
python3 pi/tools/demo_failsafe.py
```

```
--- phase 2: CUTTING the Pi->node link (watchdog = 150 ms) ---
  t= 3.21s  state=DEGRADED  v_meas=   546 mm/s  rpdo_age=  223 ms
  t= 3.41s  state=DEGRADED  v_meas=   298 mm/s  rpdo_age=  423 ms
  t= 3.61s  state=DEGRADED  v_meas=    72 mm/s  rpdo_age=  624 ms
  t= 3.81s  state=SAFE_STOP v_meas=     0 mm/s  rpdo_age=  824 ms

  node entered DEGRADED at:   150 ms after the cut
  wheels reached zero at:     652 ms after the cut
  final node state:           SAFE_STOP
  PASS: link loss was contained by the node
```

More:

```bash
make test          # 82 Python tests + the C firmware-logic tests
make bench         # regenerate docs/latency-budget.md
make check-protocol  # verify the C and Python wire formats still agree
```

The synthetic camera is not a mock. It renders a stereo pair by forward-warping
a textured image with `d = f·B/Z` and z-buffered occlusion, so SGBM recovers
the disparity that was put in — the vision tests assert recovered distance
against known ground truth to ±12 cm at 2 m.

## Run it on hardware

### Pi

```bash
pip install -r pi/requirements.txt

# 1. Calibrate. The pipeline REFUSES to start on real hardware without this:
#    a plausible-looking wrong distance is worse than no distance.
python3 pi/tools/calibrate_stereo.py capture --out captures/
python3 pi/tools/calibrate_stereo.py solve --in captures/ \
    --rows 6 --cols 9 --square-mm 25 --out pi/config/stereo_calibration.yml

# 2. Bring up CAN (MCP2515 HAT or similar)
sudo ip link set can0 up type can bitrate 500000 restart-ms 100

# 3. Go
cd pi && python3 -m stereolink.main --config config/pi.yml
```

### ESP32

```bash
cd esp32
idf.py set-target esp32
idf.py build flash monitor
```

Wiring is in [`esp32/main/app_config.h`](esp32/main/app_config.h) — reference
build is an ESP32-WROOM-32 with an SN65HVD230 transceiver, a TB6612FNG dual
H-bridge, two quadrature encoders and three digital IR sensors.

### Without CAN hardware, but with the real socketcan path

```bash
sudo ./pi/tools/vcan_up.sh vcan0
python3 pi/tools/esp32_sim.py --interface socketcan --channel vcan0 &
cd pi && python3 -m stereolink.main --source synthetic \
    --can-interface socketcan --can-channel vcan0 --no-mqtt
candump -tz vcan0
```

## The four things this project is actually about

### 1. The watchdog

If `RPDO1` stops arriving for 150 ms, the ESP32 stops trusting the Pi and
ramps the motors to zero at a controlled deceleration, then latches outputs
off. It does not resume by itself when the link returns — a robot that
restarts when someone reseats a connector is a hazard.

The watchdog can be *tightened* over the bus but never widened. A safety
timeout that can be relaxed over the link it protects is not a safety
function. → [`docs/failsafe.md`](docs/failsafe.md)

### 2. The latency budget

Frame capture to motor command, broken down per stage, measured rather than
estimated. The Pi and ESP32 have unrelated clocks, so each setpoint carries a
one-byte `seq`; the node echoes it and separately reports its own local
receive→apply span.

At 640×360 on an i5-1235U: SGBM ~13.6 ms p50 dominates, and the whole
capture→echo path lands at ~26 ms p50 / ~35 ms p99.
→ [`docs/latency-budget.md`](docs/latency-budget.md)

### 3. Priority separation

Control at priority 23 pinned to core 1; all comms at 12/11/6 pinned to core
0; the setpoint crosses between them through a seqlock rather than a mutex,
because a mutex would let a priority-12 task block a priority-23 one.

A load generator is included specifically to attack this claim — it floods the
TWAI queue and burns CPU on the comms core simultaneously.
→ [`docs/jitter-report.md`](docs/jitter-report.md)

### 4. The OT/IT boundary

Sparkplug B is implemented directly against the spec's protobuf schema (no
`tahu` dependency): NDEATH registered as the MQTT Will *before* connect,
NBIRTH at seq 0 declaring aliases, alias-only DDATA afterwards, `bdSeq` per
session. OPC UA via `asyncua` exposes the same metrics for a SCADA client.

Both are strictly non-blocking and self-disable if their dependency or broker
is missing. The IT layer is never allowed to stall the OT layer.

## Layout

```
pi/
  stereolink/          camera, calibration, disparity, obstacle, planner
    pdo.py             normative wire format
    canlink.py         CANopen master + pluggable transports
    latency.py         per-stage budget, seq-keyed round-trip closure
    north/             sparkplug_b.py, mqtt_north.py, opcua_north.py
  tools/
    demo_failsafe.py       the headline demo
    esp32_sim.py           behavioural model of the node
    benchmark_latency.py   regenerates docs/latency-budget.md
    parse_jitter_log.py    regenerates docs/jitter-report.md
    check_protocol_sync.py verifies C and Python agree
    calibrate_stereo.py    stereo calibration
  tests/               82 tests
esp32/
  main/                firmware: control.c, canopen.c, drivers
  host_test/           firmware logic tested on the host, no IDF needed
docs/                  architecture, failsafe, latency, jitter, CANopen profile
```

## Honest limitations

- **Not a certified safety function.** No SIL/PL rating, no dual-channel
  redundancy. The H-bridge standby pin is a software-commanded output, not a
  hardware-interlocked STO. A machine that can injure someone needs a rated
  E-stop circuit as well as this, not instead of it.
- **The jitter numbers are not in the repo yet.** The procedure and
  instrumentation are; the measurements have to come from real silicon and
  would be worthless invented. `docs/jitter-report.md` says so.
- **`esp32_sim.py` models logic, not timing.** It is a behavioural twin used
  to regression-test the failsafe in CI. Real timing comes from the board.
- **The planner is intentionally simple** — a free-space profile and a
  de-rate, not a path planner. The interesting engineering here is the link
  and the failsafe, and the planner is kept small enough to audit.
- **OPC UA runs anonymous and unencrypted**, which is right for a lab demo and
  the first thing to change before it faces a plant network.

## Licence

See [LICENSE](LICENSE).
