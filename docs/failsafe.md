# Failsafe design

## What is being defended against

The Pi will stop sending setpoints. Not "might" — will. Connector vibration,
a kernel oops, an OOM kill, a full SD card, a segfault in OpenCV, someone
pulling the wrong cable. The question a design has to answer is not whether it
happens but what the machine does in the seconds afterwards.

An open-loop drive that holds its last commanded speed keeps going until it
hits something. That is the failure mode being designed out.

## The rule

**If no `RPDO1` arrives for longer than the watchdog timeout, the node stops
trusting the master and brings the drive to rest under its own control.**

Implemented in [`esp32/main/control.c`](../esp32/main/control.c), inside the
1 kHz control task — not in an interrupt, not in the comms task, not on the Pi.
It runs every single control tick regardless of what the bus is doing.

## State machine

```
   INIT ──► IDLE ──────────────► RUN
              ▲                   │
              │            setpoint stale (> watchdog_ms)
              │            or IR centre blocked
              │            or CAN bus-off
              │                   ▼
              │              DEGRADED  ── ramp v,ω → 0 at decel_mm_s2
              │                   │
              │          ramp complete (|v| < 5 mm/s)
              │                   ▼
              └───────────── SAFE_STOP  (outputs DISABLED, latched)
                  explicit
                CLEAR_FAULT
                + cause gone
```

Two details that matter more than the diagram:

**`DEGRADED` recovers, `SAFE_STOP` does not.** While still ramping, a fresh
setpoint returns the node to `RUN` — a brief dropout should not require a
technician. Once the ramp has completed and outputs are off, only an explicit
`CLEAR_FAULT` with the cause actually gone will release it. A robot that
restarts by itself when someone reseats a connector is a hazard.

**The stop is a ramp, not a step.** Commanding zero in one tick is a torque
step the drivetrain and the payload both have to absorb. `FAILSAFE_DECEL_MM_S2`
(default 1200 mm/s²) is a controlled deceleration.

## Timing

Worst case from link loss to stationary:

```
  t_stop  =  watchdog_timeout  +  v / decel
          =  150 ms            +  700 / 1200 s
          =  150 ms            +  583 ms
          =  733 ms                            (≈ 0.26 m at 0.7 m/s)
```

Verified two ways:

- [`esp32/host_test/test_firmware_logic.c`](../esp32/host_test/test_firmware_logic.c)
  `test_failsafe_ramp_duration` reproduces the exact ramp arithmetic the
  control loop performs, and asserts the total.
- [`pi/tests/test_failsafe.py`](../pi/tests/test_failsafe.py) runs the
  behavioural node model over the in-process bus and measures it end to end.

`pi/tools/demo_failsafe.py` shows it live. A representative run:

```
--- phase 2: CUTTING the Pi->node link (watchdog = 150 ms) ---
  t= 3.21s  state=DEGRADED  v_meas=   546 mm/s  rpdo_age=  223 ms
  t= 3.41s  state=DEGRADED  v_meas=   298 mm/s  rpdo_age=  423 ms
  t= 3.61s  state=DEGRADED  v_meas=    72 mm/s  rpdo_age=  624 ms
  t= 3.81s  state=SAFE_STOP v_meas=     0 mm/s  rpdo_age=  824 ms

  node entered DEGRADED at:   150 ms after the cut
  wheels reached zero at:     652 ms after the cut
  final node state:           SAFE_STOP
  EMCY frames received:       1
```

## Choosing the timeout

The Pi sends at 10 Hz (100 ms). The default watchdog is 150 ms — 1.5 missed
cycles.

- Too tight (say 110 ms) and one late frame from a GC pause trips a stop.
  Nuisance stops train operators to ignore or disable the safety function,
  which is worse than not having it.
- Too loose (say 1 s) and the machine travels 0.7 m before reacting.

150 ms tolerates a single late frame but not two. If the Pi's rate changes,
this must change with it; they are coupled and the coupling is documented in
both `config.py` and `app_config.h`.

## The watchdog cannot be widened over the link

`RPDO2` carries runtime limits including `watchdog_ms`. The node accepts a
*tighter* value and refuses a wider one:

```c
/* control.c */
uint32_t requested = limits->watchdog_ms;
if (requested >= WATCHDOG_TIMEOUT_MS_MIN && requested < s_watchdog_ms) {
    s_watchdog_ms = requested;
}
```

A safety timeout that can be relaxed over the very bus it protects is not a
safety function — a confused or hostile master could disable it entirely.
Pinned by `test_watchdog_cannot_be_widened_over_the_link`.

## The independent layers

The watchdog is one of four, and they fail independently:

| Layer | Reacts in | Depends on |
|---|---|---|
| IR proximity | ≤ 1 ms (one control tick) | Nothing but the sensor and the MCU |
| Setpoint watchdog | `watchdog_ms` | The MCU's own clock |
| CAN bus-off detection | ~500 ms (health poll) | The TWAI peripheral |
| Vision stop distance | ~100 ms + pipeline latency | The Pi being alive and correct |

The IR layer is deliberately the fastest and the dumbest. It does not know
what an obstacle is, it cannot be wrong about depth, and it does not care
whether the Pi or the bus exists. It overrides forward motion directly in the
control loop:

```c
if (ir != 0) {
    s_faults |= FAULT_IR_BLOCKED;
    if (s_v_target_mm_s > 0.0f) s_v_target_mm_s = 0.0f;   /* forward only */
    if ((ir & IR_MASK_CENTER) && s_state == DRIVE_RUN)
        enter_state(DRIVE_DEGRADED, EMCY_IR_BLOCKED);
}
```

Reverse and yaw stay available, so the machine can back out of whatever it is
stuck against rather than becoming immovable.

## Boot order

From [`app_main.c`](../esp32/main/app_main.c):

1. Motors — **outputs disabled**. Nothing can move because the MCU booted.
2. Encoders and IR — the control loop needs real inputs.
3. Control task and its 1 kHz timer — **the watchdog is now live**.
4. CAN, last.

CAN comes up last on purpose. Bringing comms up first would open a window in
which setpoints could arrive with no watchdog running to time them.

## What is deliberately not claimed

This is a well-engineered failsafe, not a certified safety function. There is
no SIL/PL rating, no dual-channel redundancy, no certified safety relay. The
H-bridge standby pin is a software-commanded output, not a
hardware-interlocked STO. A machine that can injure someone needs a rated
E-stop circuit in addition to all of this, not instead of it.

`exec_max` in the jitter report is the worst *observed* loop-body time, not a
WCET bound — empirical, not a static timing analysis.
