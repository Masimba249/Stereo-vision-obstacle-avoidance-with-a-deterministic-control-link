# CANopen profile

The device profile for the drive node. Both sides implement this from one
normative source: [`pi/stereolink/pdo.py`](../pi/stereolink/pdo.py), mirrored
byte-for-byte in [`esp32/main/pdo_defs.h`](../esp32/main/pdo_defs.h) and
checked by `pi/tools/check_protocol_sync.py`.

- Bitrate: **500 kbit/s**
- Node ID: **0x22** (34)
- Frame format: 11-bit identifiers, data frames only
- Byte order: **little-endian**, all PDOs exactly **8 bytes**

## Why not a full CANopen stack

The link carries four PDOs plus NMT and heartbeat. A full CiA 301 stack would
add SDO negotiation, a full object dictionary, and a threading model that would
then have to be characterised for latency. What is implemented is the subset
the link actually uses, with standard COB-IDs and semantics so a commercial
CANopen analyser still understands the bus.

## COB-ID map

| COB-ID | Object | Direction | Rate | Payload |
|---|---|---|---|---|
| `0x000` | NMT node control | master → node | on change | `[command, node_id]` |
| `0x080` | SYNC | master → node | 10 Hz | empty |
| `0x0A2` | EMCY | node → master | on fault | `pdo_emcy_t` |
| `0x1A2` | TPDO1 Status | node → master | 50 Hz | `pdo_status_t` |
| `0x222` | RPDO1 Setpoint | master → node | 10 Hz | `pdo_setpoint_t` |
| `0x2A2` | TPDO2 Diagnostics | node → master | 5 Hz | `pdo_diag_t` |
| `0x322` | RPDO2 Limits | master → node | on change | `pdo_limits_t` |
| `0x3A2` | TPDO3 Latency echo | node → master | per RPDO1 | `pdo_latency_echo_t` |
| `0x722` | Heartbeat | node → master | 5 Hz | `[nmt_state]` |

Bus load at steady state is roughly 10 + 50 + 5 + 10 + 5 ≈ 80 frames/s. At
500 kbit/s an 8-byte frame is ~230 µs, so this is under 2 % utilisation —
deliberately, so that a burst of diagnostics can never delay a setpoint.

## RPDO1 — Setpoint (master → node, `0x222`)

The command the node closes its loop around.

| Byte | Field | Type | Units | Notes |
|---|---|---|---|---|
| 0–1 | `v_mm_s` | int16 | mm/s | Body forward velocity, ±1500 |
| 2–3 | `w_mrad_s` | int16 | mrad/s | Yaw rate, +ve = left, ±4000 |
| 4–5 | `obstacle_cm` | uint16 | cm | Nearest obstacle; 65535 = none seen |
| 6 | `seq` | uint8 | — | Latency token, wraps at 256 |
| 7 | `flags` | uint8 | — | See below |

`flags`: `0x01` VISION_VALID · `0x02` ESTOP_REQUEST · `0x04` OBSTACLE_NEAR ·
`0x08` CLEAR_FAULT

`seq` is how end-to-end latency is measured without a shared clock: the Pi
records `t_capture` against it, the node echoes it back.

Out-of-range velocities are **clamped, not wrapped**, on both sides. A wrapped
int16 velocity means full speed in the wrong direction.

## RPDO2 — Limits (master → node, `0x322`)

| Byte | Field | Type | Units | Notes |
|---|---|---|---|---|
| 0–1 | `v_max_mm_s` | uint16 | mm/s | Only ever reduces the firmware limit |
| 2–3 | `decel_mm_s2` | uint16 | mm/s² | Failsafe ramp rate, min 100 |
| 4–5 | `watchdog_ms` | uint16 | ms | **Tighten only** — see below |
| 6 | `ir_stop_cm` | uint8 | cm | IR stop threshold |
| 7 | `reserved` | uint8 | — | Zero |

The node refuses a `watchdog_ms` wider than its current value. A safety timeout
that can be relaxed over the bus it protects is not a safety function.

## TPDO1 — Status (node → master, `0x1A2`)

| Byte | Field | Type | Notes |
|---|---|---|---|
| 0–1 | `v_meas_mm_s` | int16 | Measured from encoders |
| 2–3 | `w_meas_mrad_s` | int16 | Measured from encoders |
| 4 | `state` | uint8 | 0 INIT · 1 IDLE · 2 RUN · 3 DEGRADED · 4 SAFE_STOP · 5 FAULT |
| 5 | `faults` | uint8 | Bitfield, below |
| 6 | `seq_echo` | uint8 | Echo of the last consumed RPDO1 `seq` |
| 7 | `ir_mask` | uint8 | `0x01` left · `0x02` centre · `0x04` right |

`faults`: `0x01` LINK_LOSS · `0x02` IR_BLOCKED · `0x04` OVERCURRENT ·
`0x08` ENCODER_STALL · `0x10` ESTOP · `0x20` CAN_BUS_OFF ·
`0x40` SETPOINT_RANGE · `0x80` CONTROL_OVERRUN

## TPDO2 — Diagnostics (node → master, `0x2A2`)

| Byte | Field | Type | Notes |
|---|---|---|---|
| 0–1 | `jitter_p99_us` | uint16 | From the control task's own histogram |
| 2–3 | `period_max_us` | uint16 | Worst observed loop period |
| 4–5 | `rpdo_age_ms` | uint16 | Time since the last setpoint |
| 6 | `ctrl_cpu_pct` | uint8 | Loop exec time ÷ period |
| 7 | `comms_cpu_pct` | uint8 | Bus utilisation estimate |

This is what makes the real-time behaviour observable from the SCADA layer
rather than only over a serial cable.

## TPDO3 — Latency echo (node → master, `0x3A2`)

| Byte | Field | Type | Notes |
|---|---|---|---|
| 0–3 | `rx_to_apply_us` | uint32 | TWAI receive → PWM write, node-local |
| 4 | `seq_echo` | uint8 | Which setpoint this refers to |
| 5 | `watchdog_misses` | uint8 | Cumulative watchdog trips |
| 6 | `rpdo_rate_hz` | uint8 | Observed setpoint rate |
| 7 | `reserved` | uint8 | Zero |

`rx_to_apply_us` is the one stage the Pi cannot time for itself, and it needs
no shared clock because it is a difference between two node-local timestamps.

## EMCY (node → master, `0x0A2`)

| Byte | Field | Notes |
|---|---|---|
| 0–1 | `error_code` | uint16, below |
| 2 | `error_register` | uint8, `0x81` = generic + manufacturer |
| 3 | `state` | Drive state at the transition |
| 4 | `faults` | Fault bitfield |
| 5–7 | reserved | Zero |

| Code | Meaning |
|---|---|
| `0x8130` | Life guard / heartbeat error — setpoint watchdog expired |
| `0x8140` | Recovered from bus-off |
| `0xFF01` | Manufacturer: IR proximity stop |
| `0xFF02` | Manufacturer: control loop deadline overrun |

Emitted **once per transition** into a latched stop, not repeatedly — pinned
by `test_emergency_object_is_emitted_once_per_transition`.

## NMT

Commands on `0x000` as `[command, target]`, where target 0 is broadcast.

| Command | Value | Effect |
|---|---|---|
| Start | `0x01` | → Operational; PDOs are honoured |
| Stop | `0x02` | → Stopped; drive commanded to stop |
| Enter pre-op | `0x80` | → Pre-operational |
| Reset node | `0x81` | Full reset, boot-up message |
| Reset comms | `0x82` | Comms reset, boot-up message |

Heartbeat on `0x722` carries the state byte: `0x00` boot-up, `0x04` stopped,
`0x05` operational, `0x7F` pre-operational.

**A setpoint received outside Operational is ignored.** Silently obeying it
would let the drive move before the master had actually started the node.

## Startup sequence

```
node → master   0x722  [0x00]              boot-up
master → node   0x000  [0x81, 0x22]        reset node
master → node   0x322  <limits>            configure
master → node   0x000  [0x01, 0x22]        start → Operational
node → master   0x722  [0x05]              heartbeat, operational
master → node   0x222  <setpoint>          …at 10 Hz from here
node → master   0x1A2  <status>            …at 50 Hz
```

## Watching the bus

```bash
sudo ./pi/tools/vcan_up.sh vcan0
candump -tz vcan0                  # everything
candump vcan0,222:7FF              # setpoints only
cansend vcan0 222#F401000064000701 # inject a setpoint by hand
```
