/* The high-priority control task and the shared drive state it owns. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "jitter.h"
#include "pdo_defs.h"
#include "setpoint_mailbox.h"

typedef struct {
    /* Written by the control task, read by the comms tasks for telemetry. */
    volatile int16_t  v_meas_mm_s;
    volatile int16_t  w_meas_mrad_s;
    volatile uint8_t  state;
    volatile uint8_t  faults;
    volatile uint8_t  seq_echo;
    volatile uint8_t  ir_mask;
    volatile uint32_t rpdo_age_ms;
    volatile uint32_t watchdog_misses;

    /* Latency echo, published by the control task when it consumes a setpoint.
     * pending_echo is cleared by the TX task once it has been sent. */
    volatile uint32_t echo_rx_to_apply_us;
    volatile uint8_t  echo_seq;
    volatile bool     pending_echo;

    /* Set by the control task on entry to a latched stop so the TX task can
     * emit exactly one EMCY per transition. */
    volatile bool     pending_emcy;
    volatile uint16_t pending_emcy_code;
} drive_status_t;

extern drive_status_t g_drive;
extern setpoint_mailbox_t g_setpoint_mb;
extern jitter_stats_t g_jitter;

void control_task_start(void);
/* Runtime limits from RPDO2. Applied by the comms task; only ever tightens
 * the watchdog. */
void control_apply_limits(const pdo_limits_t *limits);
void control_notify_bus_off(bool off);
void control_request_estop(bool on);
uint32_t control_loop_count(void);
