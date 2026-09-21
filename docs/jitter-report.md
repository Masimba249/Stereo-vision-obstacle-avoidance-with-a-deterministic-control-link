# Control-loop jitter under comms load

> **Status: procedure documented, hardware numbers not yet collected.**
>
> This file describes how the measurement is taken and what it has to show.
> It contains **no measured values**, because they have to come from a real
> ESP32 — the timing behaviour being tested is a property of the silicon and
> the scheduler, and a number invented on a laptop would be worthless.
>
> Run the procedure below and `pi/tools/parse_jitter_log.py` will overwrite
> this file with the real table.

## The claim

> Control-loop timing is unaffected by comms load: the 1 kHz loop keeps its
> period whether the CAN bus is idle or saturated.

This is the claim the priority separation exists to support, so it is stated
as something falsifiable and then attacked.

## The mechanism

Three things, in order of how much work they do:

1. **Core pinning.** The control task owns core 1 (`CORE_CONTROL`); every
   comms task is pinned to core 0 (`CORE_COMMS`). Most contention cannot
   happen at all.
2. **Priority.** Control at 23; CAN rx at 12, CAN tx at 11, load generator at
   6, stats at 5. Even sharing a core, control preempts.
3. **No shared locks.** The setpoint crosses between them through a seqlock
   ([`setpoint_mailbox.h`](../esp32/main/setpoint_mailbox.h)), not a mutex.
   A mutex would let a priority-12 task block a priority-23 task — the precise
   priority inversion the split exists to prevent.

The tick is dispatched from the `esp_timer` ISR (`ESP_TIMER_ISR`), not via the
timer service task, removing a scheduling hop from the deterministic path.

## The instrument

[`esp32/main/jitter.c`](../esp32/main/jitter.c) keeps a histogram of
`|measured period − 1000 µs|` in 2 µs bins, **sampled inside the control task
itself**. That matters: a lower-priority observer would be preempted by
exactly the events being measured and would under-report.

It separately tracks:

- `period_max_us` — worst observed wakeup-to-wakeup interval
- `exec_max_us` — worst observed *loop body* execution time
- `overruns` — ticks where the body outlasted its own period

The last is the one that actually breaks determinism; a late wakeup that still
completes on time is a different (and lesser) problem.

## Procedure

```bash
cd esp32

# 1. Baseline: normal traffic, no load generator.
idf.py -B build-idle build flash
idf.py -B build-idle monitor | tee ../reports/jitter-idle.log
#    …let it run at least 2 minutes, with the Pi sending setpoints…

# 2. Attack: flood the TWAI queue and burn CPU on core 0.
idf.py -B build-loaded -DENABLE_LOAD_GENERATOR=1 build flash
idf.py -B build-loaded monitor | tee ../reports/jitter-loaded.log
#    …same duration, same setpoint traffic…

# 3. Build the comparison.
cd ..
python3 pi/tools/parse_jitter_log.py \
    --idle reports/jitter-idle.log \
    --loaded reports/jitter-loaded.log \
    --out docs/jitter-report.md
```

The parser exits non-zero if the loaded run shows more deadline overruns than
the baseline, or if p99 deviation degrades badly — so it can gate a CI job on
hardware rather than being a document someone has to remember to read.

## What the load generator does

[`loadgen.c`](../esp32/main/loadgen.c), at priority 6 on core 0, every 5 ms:

- pushes `LOADGEN_BURST_FRAMES` (24) frames at high COB-IDs into the TWAI
  queue — bus and driver pressure;
- runs a 20 000-iteration integer loop — CPU pressure on the comms core.

Both at once, because either alone would leave the obvious objection open. The
high COB-IDs mean these frames lose arbitration against real traffic, which is
what you want from a load fixture: it stresses the system without corrupting
the thing being measured.

It is a **test fixture**, compiled out by default (`ENABLE_LOAD_GENERATOR=0`).

## What "pass" looks like

- Deadline overruns: **0**, in both runs.
- p99 period deviation: within a few µs of baseline.
- `period_max_us`: no large outliers appearing only under load.
- `exec_max_us`: essentially unchanged — the loop body does not get slower,
  it just has more competition for the core it is not sharing.

A result where p99 stays flat but `period_max` grows is still informative: it
means the rare case got worse while the typical case did not, which is worth
knowing before someone builds a deadline assumption on the median.

## Caveat

`exec_max` is the worst *observed* execution time, not a WCET bound. This is
an empirical measurement under a specific load, not a static timing analysis,
and it does not by itself support a certification argument.
