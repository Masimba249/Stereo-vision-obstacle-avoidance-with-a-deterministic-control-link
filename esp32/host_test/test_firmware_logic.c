/* Host-side unit tests for the portable parts of the drive firmware.
 *
 * ESP-IDF is not required: pid.c, jitter.c, the setpoint mailbox and the PDO
 * struct layouts are all plain C and can be tested (and regression-tested in
 * CI) on a development machine.  What cannot be tested here is timing - that
 * comes from the hardware, via docs/jitter-report.md.
 *
 * Build and run:  make -C esp32/host_test test
 */
#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "jitter.h"
#include "pdo_defs.h"
#include "pid.h"
#include "setpoint_mailbox.h"

static int g_failures;

#define CHECK(cond, ...)                                            \
    do {                                                            \
        if (!(cond)) {                                              \
            printf("  FAIL: " __VA_ARGS__);                         \
            printf("        at %s:%d\n", __FILE__, __LINE__);       \
            g_failures++;                                           \
        }                                                           \
    } while (0)

/* ---------------------------------------------------------------- layout */
static void test_pdo_layout(void)
{
    printf("test_pdo_layout\n");
    /* The _Static_asserts in pdo_defs.h already enforce the sizes at compile
     * time; here we pin the field offsets, which is what actually has to match
     * the Python struct format strings. */
    CHECK(offsetof(pdo_setpoint_t, v_mm_s) == 0, "v_mm_s offset\n");
    CHECK(offsetof(pdo_setpoint_t, w_mrad_s) == 2, "w_mrad_s offset\n");
    CHECK(offsetof(pdo_setpoint_t, obstacle_cm) == 4, "obstacle_cm offset\n");
    CHECK(offsetof(pdo_setpoint_t, seq) == 6, "seq offset\n");
    CHECK(offsetof(pdo_setpoint_t, flags) == 7, "flags offset\n");

    CHECK(offsetof(pdo_status_t, state) == 4, "status.state offset\n");
    CHECK(offsetof(pdo_status_t, faults) == 5, "status.faults offset\n");
    CHECK(offsetof(pdo_status_t, seq_echo) == 6, "status.seq_echo offset\n");
    CHECK(offsetof(pdo_status_t, ir_mask) == 7, "status.ir_mask offset\n");

    CHECK(offsetof(pdo_latency_echo_t, rx_to_apply_us) == 0, "echo.rx offset\n");
    CHECK(offsetof(pdo_latency_echo_t, seq_echo) == 4, "echo.seq offset\n");

    /* Decode the exact bytes the Python encoder produced for
     * SetpointPdo(500, -250, 142, 7, VISION_VALID): f40106ff8e000701 */
    const uint8_t wire[8] = {0xf4, 0x01, 0x06, 0xff, 0x8e, 0x00, 0x07, 0x01};
    pdo_setpoint_t sp;
    memcpy(&sp, wire, sizeof(sp));
    CHECK(sp.v_mm_s == 500, "v_mm_s: got %d\n", sp.v_mm_s);
    CHECK(sp.w_mrad_s == -250, "w_mrad_s: got %d\n", sp.w_mrad_s);
    CHECK(sp.obstacle_cm == 142, "obstacle_cm: got %u\n", sp.obstacle_cm);
    CHECK(sp.seq == 7, "seq: got %u\n", sp.seq);
    CHECK(sp.flags == SP_FLAG_VISION_VALID, "flags: got 0x%02X\n", sp.flags);
}

/* ------------------------------------------------------------------- PID */
static void test_pid_converges(void)
{
    printf("test_pid_converges\n");
    pid_t p;
    const float dt = 0.001f;
    pid_init(&p, PID_KP_T, PID_KI_T, 0.0f, 600.0f, 60.0f, dt, -1500.0f, 1500.0f);

    /* First-order plant: v' = (u - v) / tau, a reasonable stand-in for a
     * geared DC motor's velocity response. */
    float v = 0.0f, tau = 0.08f;
    const float target = 500.0f;
    for (int i = 0; i < 3000; i++) {
        float u = pid_update(&p, target, v, dt);
        v += (u - v) * (dt / tau);
    }
    CHECK(fabsf(v - target) < 5.0f, "settled at %.1f, wanted %.1f\n",
          (double)v, (double)target);
}

static void test_pid_antiwindup(void)
{
    printf("test_pid_antiwindup\n");
    pid_t p;
    const float dt = 0.001f;
    pid_init(&p, PID_KP_T, PID_KI_T, 0.0f, 600.0f, 60.0f, dt, -100.0f, 100.0f);

    /* Drive hard into saturation with an unreachable target. */
    for (int i = 0; i < 2000; i++) {
        (void)pid_update(&p, 5000.0f, 0.0f, dt);
    }
    float wound = p.integral;

    /* Now ask for zero. Without anti-windup the output would stay pinned high
     * for a long time; with it, the sign flips almost immediately - this is
     * what stops the drive lurching when a failsafe ramp releases. */
    float out = pid_update(&p, 0.0f, 0.0f, dt);
    CHECK(wound <= 600.0f + 1e-3f, "integral exceeded its limit: %.1f\n",
          (double)wound);
    CHECK(out <= 0.0f, "output did not reverse after setpoint drop: %.1f\n",
          (double)out);
}

/* ---------------------------------------------------------------- jitter */
static void test_jitter_percentiles(void)
{
    printf("test_jitter_percentiles\n");
    jitter_stats_t j;
    jitter_init(&j);

    /* 990 perfectly on-period samples, 10 samples 50 us late. */
    for (int i = 0; i < 990; i++) {
        jitter_add(&j, CONTROL_PERIOD_US, 120);
    }
    for (int i = 0; i < 10; i++) {
        jitter_add(&j, CONTROL_PERIOD_US + 50, 120);
    }

    CHECK(j.samples == 1000, "samples: %u\n", (unsigned)j.samples);
    CHECK(j.period_max_us == (uint32_t)CONTROL_PERIOD_US + 50,
          "period_max: %u\n", (unsigned)j.period_max_us);
    CHECK(j.overruns == 0, "unexpected overruns: %u\n", (unsigned)j.overruns);

    uint32_t p50 = jitter_percentile_us(&j, 50.0f);
    uint32_t p99 = jitter_percentile_us(&j, 99.0f);
    CHECK(p50 <= JITTER_BIN_US, "p50 should be ~0, got %u\n", (unsigned)p50);
    CHECK(p99 <= JITTER_BIN_US, "p99 should be ~0 (99%% on time), got %u\n",
          (unsigned)p99);
    /* The 1% tail must be visible somewhere above the p99. */
    CHECK(jitter_percentile_us(&j, 99.9f) > p99,
          "the late tail is not represented\n");
}

static void test_jitter_overrun_detection(void)
{
    printf("test_jitter_overrun_detection\n");
    jitter_stats_t j;
    jitter_init(&j);
    jitter_add(&j, CONTROL_PERIOD_US, CONTROL_PERIOD_US + 200);
    CHECK(j.overruns == 1, "overrun not counted: %u\n", (unsigned)j.overruns);
    CHECK(j.exec_max_us == (uint32_t)CONTROL_PERIOD_US + 200,
          "exec_max wrong: %u\n", (unsigned)j.exec_max_us);
}

/* --------------------------------------------------------------- mailbox */
static void test_mailbox_roundtrip(void)
{
    printf("test_mailbox_roundtrip\n");
    setpoint_mailbox_t mb;
    mailbox_init(&mb);

    setpoint_msg_t in = {
        .sp = {.v_mm_s = 700, .w_mrad_s = -120, .obstacle_cm = 210,
               .seq = 42, .flags = SP_FLAG_VISION_VALID},
        .t_rx_us = 123456789,
        .fresh = true,
    };
    mailbox_publish(&mb, &in);

    setpoint_msg_t out;
    CHECK(mailbox_read(&mb, &out), "read failed on an idle mailbox\n");
    CHECK(out.sp.v_mm_s == 700, "v_mm_s: %d\n", out.sp.v_mm_s);
    CHECK(out.sp.seq == 42, "seq: %u\n", out.sp.seq);
    CHECK(out.t_rx_us == 123456789, "t_rx_us mismatch\n");
    CHECK(out.fresh, "freshness lost\n");

    mailbox_mark_consumed(&mb);
    CHECK(mailbox_read(&mb, &out), "read failed after consume\n");
    CHECK(!out.fresh, "consumed flag not observed\n");
}

static void test_mailbox_detects_torn_write(void)
{
    printf("test_mailbox_detects_torn_write\n");
    setpoint_mailbox_t mb;
    mailbox_init(&mb);
    setpoint_msg_t msg = {.sp = {.v_mm_s = 100}, .fresh = true};
    mailbox_publish(&mb, &msg);

    /* Simulate the writer being interrupted mid-update: the sequence counter
     * is left odd.  The reader must refuse the snapshot rather than return
     * half-written data. */
    atomic_store(&mb.seq, atomic_load(&mb.seq) + 1);
    setpoint_msg_t out;
    CHECK(!mailbox_read(&mb, &out),
          "reader accepted a snapshot during an in-flight write\n");

    /* Writer completes; the reader recovers on the next attempt. */
    atomic_store(&mb.seq, atomic_load(&mb.seq) + 1);
    CHECK(mailbox_read(&mb, &out), "reader did not recover after the write\n");
}

/* ------------------------------------------------------- failsafe timing */
static void test_failsafe_ramp_duration(void)
{
    printf("test_failsafe_ramp_duration\n");
    /* The ramp the control loop performs in DEGRADED, reproduced exactly:
     * a fixed deceleration applied once per control period. */
    const float dt = 1.0f / (float)CONTROL_HZ;
    const float decel = FAILSAFE_DECEL_MM_S2;
    float v = 700.0f;
    int ticks = 0;
    while (fabsf(v) >= ZERO_SPEED_EPS_MM_S && ticks < CONTROL_HZ * 10) {
        float step = decel * dt;
        if (v > step) {
            v -= step;
        } else if (v < -step) {
            v += step;
        } else {
            v = 0.0f;
        }
        ticks++;
    }
    float expected_ms = (700.0f / decel) * 1000.0f;
    float actual_ms = (float)ticks * dt * 1000.0f;
    CHECK(fabsf(actual_ms - expected_ms) < 10.0f,
          "ramp took %.1f ms, expected ~%.1f ms\n",
          (double)actual_ms, (double)expected_ms);

    /* And the number the failsafe actually promises: detection + ramp. */
    float worst_case_ms = (float)WATCHDOG_TIMEOUT_MS_DEFAULT + expected_ms;
    printf("  worst-case stop from 700 mm/s: %.0f ms detect + %.0f ms ramp "
           "= %.0f ms\n", (double)WATCHDOG_TIMEOUT_MS_DEFAULT,
           (double)expected_ms, (double)worst_case_ms);
    CHECK(worst_case_ms < 1000.0f, "stop distance budget exceeded\n");
}

int main(void)
{
    printf("=== firmware logic tests (host build) ===\n");
    test_pdo_layout();
    test_pid_converges();
    test_pid_antiwindup();
    test_jitter_percentiles();
    test_jitter_overrun_detection();
    test_mailbox_roundtrip();
    test_mailbox_detects_torn_write();
    test_failsafe_ramp_duration();

    if (g_failures == 0) {
        printf("=== ALL PASS ===\n");
        return 0;
    }
    printf("=== %d FAILURE(S) ===\n", g_failures);
    return 1;
}
