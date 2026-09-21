/* 1 kHz closed-loop motor control, failsafe state machine, jitter instrument.
 *
 * This task is the real-time half of the node.  Everything it needs is either
 * owned by it or handed over lock-free, so its period is bounded by the
 * hardware timer and nothing else on the system can stretch it.
 *
 * It is pinned to CORE_CONTROL at PRIO_CONTROL; all CAN and telemetry work is
 * pinned to CORE_COMMS at much lower priorities.  docs/jitter-report.md has
 * the measurements that back that up.
 */
#include "control.h"

#include <math.h>
#include <string.h>

#include "app_config.h"
#include "encoder.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ir_safety.h"
#include "motor.h"
#include "pid.h"

static const char *TAG = "control";

drive_status_t g_drive;
setpoint_mailbox_t g_setpoint_mb;
jitter_stats_t g_jitter;

static TaskHandle_t s_control_task;
static esp_timer_handle_t s_tick_timer;
static volatile uint32_t s_loop_count;

/* Runtime-tunable limits, defaulted from app_config.h and tightened by RPDO2. */
static volatile float    s_v_max_mm_s = V_MAX_MM_S;
static volatile float    s_decel_mm_s2 = FAILSAFE_DECEL_MM_S2;
static volatile uint32_t s_watchdog_ms = WATCHDOG_TIMEOUT_MS_DEFAULT;
static volatile bool     s_bus_off = false;
static volatile bool     s_estop = false;

/* Control-loop-private state. */
static pid_t s_pid[MOTOR_COUNT];
static float s_wheel_mm_s[MOTOR_COUNT];
static float s_v_ramped_mm_s;      /* the ramped, actually-commanded speed */
static float s_w_ramped_mrad_s;
static int64_t s_t_last_rpdo_us;
static uint8_t s_state = DRIVE_INIT;
static uint8_t s_faults = FAULT_NONE;
static float s_v_target_mm_s;
static float s_w_target_mrad_s;

/* The periodic timer only wakes the task.  All the work happens at task level,
 * so a long control computation can never delay a subsequent timer interrupt
 * and the measured period reflects genuine scheduling latency. */
static void IRAM_ATTR tick_isr(void *arg)
{
    BaseType_t hp_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_control_task, &hp_task_woken);
    if (hp_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void enter_state(uint8_t new_state, uint16_t emcy_code)
{
    if (s_state == new_state) {
        return;
    }
    ESP_LOGW(TAG, "state %u -> %u (faults=0x%02X)", s_state, new_state, s_faults);
    s_state = new_state;
    if (new_state == DRIVE_SAFE_STOP || new_state == DRIVE_FAULT) {
        motor_enable_outputs(false);
        g_drive.pending_emcy_code = emcy_code;
        g_drive.pending_emcy = true;
    }
}

static void apply_wheel_commands(float dt_s)
{
    /* Differential kinematics: convert the body twist into wheel speeds.
     * w is in mrad/s, so w/1000 * (base/2) gives m/s -> *1000 for mm/s, which
     * cancels to w * base / 2. */
    float half_track = 0.5f * WHEEL_BASE_M;
    float v_left = s_v_ramped_mm_s - s_w_ramped_mrad_s * half_track;
    float v_right = s_v_ramped_mm_s + s_w_ramped_mrad_s * half_track;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        int32_t counts = encoder_read_delta((motor_id_t)i);
        s_wheel_mm_s[i] = encoder_counts_to_mm_s(counts, dt_s);
    }

    if (!motor_outputs_enabled()) {
        /* Outputs are off: hold the integrators at zero so that re-enabling
         * does not dump an accumulated error into the bridge. */
        pid_reset(&s_pid[MOTOR_LEFT]);
        pid_reset(&s_pid[MOTOR_RIGHT]);
        motor_set(MOTOR_LEFT, 0.0f);
        motor_set(MOTOR_RIGHT, 0.0f);
        return;
    }

    float duty_l = pid_update(&s_pid[MOTOR_LEFT], v_left, s_wheel_mm_s[MOTOR_LEFT], dt_s);
    float duty_r = pid_update(&s_pid[MOTOR_RIGHT], v_right, s_wheel_mm_s[MOTOR_RIGHT], dt_s);
    motor_set(MOTOR_LEFT, duty_l / s_v_max_mm_s);
    motor_set(MOTOR_RIGHT, duty_r / s_v_max_mm_s);
}

static void ramp_towards(float *value, float target, float rate, float dt_s)
{
    float step = rate * dt_s;
    float delta = target - *value;
    if (delta > step) {
        delta = step;
    } else if (delta < -step) {
        delta = -step;
    }
    *value += delta;
    if (fabsf(*value) < ZERO_SPEED_EPS_MM_S && fabsf(target) < ZERO_SPEED_EPS_MM_S) {
        *value = 0.0f;
    }
}

static void control_tick(float dt_s, int64_t now_us)
{
    /* ---- 1. Consume the newest setpoint, if the comms task left one ---- */
    setpoint_msg_t msg;
    if (mailbox_read(&g_setpoint_mb, &msg) && msg.fresh) {
        mailbox_mark_consumed(&g_setpoint_mb);
        s_t_last_rpdo_us = msg.t_rx_us;
        g_drive.seq_echo = msg.sp.seq;

        if (msg.sp.flags & SP_FLAG_CLEAR_FAULT) {
            if (s_state == DRIVE_SAFE_STOP) {
                /* A latched stop is cleared only by an explicit request, and
                 * only once the cause is gone.  Auto-clearing would mean the
                 * robot restarts by itself the moment a cable is reseated. */
                if (!ir_safety_blocked() && !s_bus_off) {
                    s_faults = FAULT_NONE;
                    s_v_ramped_mm_s = 0.0f;
                    s_w_ramped_mrad_s = 0.0f;
                    enter_state(DRIVE_IDLE, 0);
                }
            }
        }

        if (msg.sp.flags & SP_FLAG_ESTOP_REQUEST) {
            s_faults |= FAULT_ESTOP;
            s_v_target_mm_s = 0.0f;
            s_w_target_mrad_s = 0.0f;
        } else {
            s_faults &= (uint8_t)~FAULT_ESTOP;
            float v = (float)msg.sp.v_mm_s;
            float w = (float)msg.sp.w_mrad_s;
            if (fabsf(v) > s_v_max_mm_s) {
                v = (v > 0.0f) ? s_v_max_mm_s : -s_v_max_mm_s;
                s_faults |= FAULT_SETPOINT_RANGE;
            } else {
                s_faults &= (uint8_t)~FAULT_SETPOINT_RANGE;
            }
            if (fabsf(w) > W_MAX_MRAD_S) {
                w = (w > 0.0f) ? W_MAX_MRAD_S : -W_MAX_MRAD_S;
                s_faults |= FAULT_SETPOINT_RANGE;
            }
            s_v_target_mm_s = v;
            s_w_target_mrad_s = w;
        }

        s_faults &= (uint8_t)~FAULT_LINK_LOSS;
        if (s_state == DRIVE_IDLE || s_state == DRIVE_DEGRADED) {
            /* A fresh setpoint re-establishes trust in the link, so a
             * transient dropout recovers without operator intervention -
             * but only from DEGRADED, never from a latched SAFE_STOP. */
            enter_state(DRIVE_RUN, 0);
            motor_enable_outputs(true);
        }

        /* Publish the latency echo now: this timestamp is taken in the same
         * tick that the setpoint reaches the PWM registers below. */
        g_drive.echo_rx_to_apply_us = (uint32_t)(now_us - msg.t_rx_us);
        g_drive.echo_seq = msg.sp.seq;
        g_drive.pending_echo = true;
    }

    /* ---- 2. Fast safety layer: IR overrides anything the Pi asked for ---- */
    uint8_t ir = ir_safety_mask();
    g_drive.ir_mask = ir;
    if (ir != 0) {
        s_faults |= FAULT_IR_BLOCKED;
        /* Forward motion is forbidden while blocked; reverse and yaw stay
         * available so the machine can back out of what it is stuck against. */
        if (s_v_target_mm_s > 0.0f) {
            s_v_target_mm_s = 0.0f;
        }
        if ((ir & IR_MASK_CENTER) && s_state == DRIVE_RUN) {
            enter_state(DRIVE_DEGRADED, EMCY_IR_BLOCKED);
        }
    } else {
        s_faults &= (uint8_t)~FAULT_IR_BLOCKED;
    }

    /* ---- 3. Setpoint watchdog: the whole reason this node is trusted ---- */
    int64_t age_us = (s_t_last_rpdo_us == 0) ? (int64_t)s_watchdog_ms * 1000 + 1
                                             : now_us - s_t_last_rpdo_us;
    g_drive.rpdo_age_ms = (uint32_t)(age_us / 1000);
    if (s_state == DRIVE_RUN || s_state == DRIVE_DEGRADED) {
        if (age_us > (int64_t)s_watchdog_ms * 1000) {
            if (s_state != DRIVE_DEGRADED) {
                g_drive.watchdog_misses++;
                ESP_LOGW(TAG, "setpoint watchdog: %d ms without RPDO1 "
                              "(limit %u) -> ramping to zero",
                         (int)(age_us / 1000), (unsigned)s_watchdog_ms);
            }
            s_faults |= FAULT_LINK_LOSS;
            enter_state(DRIVE_DEGRADED, EMCY_LINK_LOSS);
        }
    }

    if (s_bus_off) {
        s_faults |= FAULT_CAN_BUS_OFF;
        if (s_state == DRIVE_RUN) {
            enter_state(DRIVE_DEGRADED, EMCY_BUS_OFF);
        }
    }
    if (s_estop) {
        s_faults |= FAULT_ESTOP;
        if (s_state == DRIVE_RUN) {
            enter_state(DRIVE_DEGRADED, EMCY_LINK_LOSS);
        }
    }

    /* ---- 4. Pick the target for this state ---- */
    float target_v, target_w, ramp_rate;
    switch (s_state) {
    case DRIVE_RUN:
        target_v = s_v_target_mm_s;
        target_w = s_w_target_mrad_s;
        /* In normal running the Pi has already slew-limited its command, so
         * we allow a brisker ramp than the failsafe uses. */
        ramp_rate = s_decel_mm_s2 * 2.0f;
        break;
    case DRIVE_DEGRADED:
        /* The failsafe ramp.  Controlled deceleration, not an instant zero:
         * dropping the setpoint to 0 in one tick is a torque step the
         * drivetrain and the payload both have to absorb. */
        target_v = 0.0f;
        target_w = 0.0f;
        ramp_rate = s_decel_mm_s2;
        break;
    default:
        target_v = 0.0f;
        target_w = 0.0f;
        ramp_rate = s_decel_mm_s2 * 4.0f;
        break;
    }

    ramp_towards(&s_v_ramped_mm_s, target_v, ramp_rate, dt_s);
    ramp_towards(&s_w_ramped_mrad_s, target_w, ramp_rate * 4.0f, dt_s);

    /* ---- 5. Ramp finished in DEGRADED -> latch the stop ---- */
    if (s_state == DRIVE_DEGRADED &&
        fabsf(s_v_ramped_mm_s) < ZERO_SPEED_EPS_MM_S &&
        fabsf(s_w_ramped_mrad_s) < ZERO_SPEED_EPS_MM_S) {
        s_v_ramped_mm_s = 0.0f;
        s_w_ramped_mrad_s = 0.0f;
        uint16_t code = (s_faults & FAULT_LINK_LOSS) ? EMCY_LINK_LOSS
                      : (s_faults & FAULT_IR_BLOCKED) ? EMCY_IR_BLOCKED
                      : EMCY_BUS_OFF;
        enter_state(DRIVE_SAFE_STOP, code);
        ESP_LOGW(TAG, "failsafe ramp complete: outputs disabled and latched");
    }

    /* ---- 6. Close the wheel loops ---- */
    apply_wheel_commands(dt_s);

    /* ---- 7. Publish for the comms tasks (they never touch our internals) - */
    float v_body = 0.5f * (s_wheel_mm_s[MOTOR_LEFT] + s_wheel_mm_s[MOTOR_RIGHT]);
    float w_body = (s_wheel_mm_s[MOTOR_RIGHT] - s_wheel_mm_s[MOTOR_LEFT]) / WHEEL_BASE_M;
    g_drive.v_meas_mm_s = (int16_t)lroundf(v_body);
    g_drive.w_meas_mrad_s = (int16_t)lroundf(w_body);
    g_drive.state = s_state;
    g_drive.faults = s_faults;
}

static void control_task(void *arg)
{
    (void)arg;
    const float dt_s = 1.0f / (float)CONTROL_HZ;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        pid_init(&s_pid[i], PID_KP, PID_KI, PID_KD, PID_I_LIMIT,
                 PID_D_LPF_HZ, dt_s, -V_MAX_MM_S, V_MAX_MM_S);
    }
    jitter_init(&g_jitter);
    s_state = DRIVE_IDLE;

    int64_t t_prev = esp_timer_get_time();
    ESP_LOGI(TAG, "control loop running at %d Hz on core %d (prio %d)",
             CONTROL_HZ, xPortGetCoreID(), PRIO_CONTROL);

    for (;;) {
        /* Block until the hardware timer fires.  ulTaskNotifyTake is the
         * cheapest FreeRTOS wakeup there is - no queue, no semaphore, no
         * allocation on the path that has to be deterministic. */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int64_t t_start = esp_timer_get_time();
        uint32_t period_us = (uint32_t)(t_start - t_prev);
        t_prev = t_start;

        control_tick(dt_s, t_start);

        uint32_t exec_us = (uint32_t)(esp_timer_get_time() - t_start);
        /* Skip the first sample: the "period" before the first tick is
         * meaningless and would poison the max. */
        if (s_loop_count > 0) {
            jitter_add(&g_jitter, period_us, exec_us);
        }
        if (exec_us > (uint32_t)CONTROL_PERIOD_US) {
            s_faults |= FAULT_CONTROL_OVERRUN;
        }
        s_loop_count++;
    }
}

void control_task_start(void)
{
    memset((void *)&g_drive, 0, sizeof(g_drive));
    mailbox_init(&g_setpoint_mb);

    BaseType_t ok = xTaskCreatePinnedToCore(control_task, "control", STACK_CONTROL,
                                            NULL, PRIO_CONTROL, &s_control_task,
                                            CORE_CONTROL);
    configASSERT(ok == pdPASS);

    const esp_timer_create_args_t timer_args = {
        .callback = tick_isr,
        .arg = NULL,
        /* Dispatched straight from the timer ISR rather than via the esp_timer
         * service task: one less scheduling hop on the critical path, and it
         * is why the measured jitter is single-digit microseconds. */
        .dispatch_method = ESP_TIMER_ISR,
        .name = "control_tick",
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_tick_timer, CONTROL_PERIOD_US));
}

void control_apply_limits(const pdo_limits_t *limits)
{
    if (limits->v_max_mm_s > 0 && (float)limits->v_max_mm_s <= V_MAX_MM_S) {
        s_v_max_mm_s = (float)limits->v_max_mm_s;
    }
    if ((float)limits->decel_mm_s2 >= FAILSAFE_DECEL_MIN) {
        s_decel_mm_s2 = (float)limits->decel_mm_s2;
    }
    /* The link may tighten the watchdog but never relax it.  A safety timeout
     * that can be widened over the very bus it protects is not a safety
     * function - a confused or hostile master could disable it entirely. */
    uint32_t requested = limits->watchdog_ms;
    if (requested >= WATCHDOG_TIMEOUT_MS_MIN && requested < s_watchdog_ms) {
        s_watchdog_ms = requested;
    }
    ESP_LOGI(TAG, "limits: v_max=%.0f decel=%.0f watchdog=%u ms",
             (double)s_v_max_mm_s, (double)s_decel_mm_s2, (unsigned)s_watchdog_ms);
}

void control_notify_bus_off(bool off)
{
    s_bus_off = off;
}

void control_request_estop(bool on)
{
    s_estop = on;
}

uint32_t control_loop_count(void)
{
    return s_loop_count;
}
