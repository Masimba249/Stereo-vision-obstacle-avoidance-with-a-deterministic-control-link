/* CANopen comms, deliberately confined to the low-priority half of the system.
 *
 * Two tasks, both on CORE_COMMS:
 *   canopen_rx  blocks on twai_receive, decodes, hands setpoints to the
 *               control task through a lock-free mailbox
 *   canopen_tx  time-triggered TPDOs, heartbeat, EMCY, latency echo
 *
 * Neither ever takes a lock the control task could want, and neither runs on
 * CORE_CONTROL.  That is the entire mechanism behind the isolation claim.
 */
#include "canopen.h"

#include <string.h>

#include "app_config.h"
#include "control.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pdo_defs.h"

static const char *TAG = "canopen";

static volatile uint8_t s_nmt_state = NMT_STATE_PREOP;
static volatile uint32_t s_rx_count, s_tx_count, s_rx_dropped;

static void send_frame(uint32_t id, const void *data, uint8_t len)
{
    twai_message_t msg = {
        .identifier = id,
        .data_length_code = len,
        .flags = 0,
    };
    msg.extd = 0;
    msg.rtr = 0;
    if (data != NULL && len > 0) {
        memcpy(msg.data, data, len);
    }
    /* A short, bounded timeout.  A blocked bus must not wedge the TX task, and
     * dropping a telemetry frame is always preferable to queueing indefinitely:
     * stale telemetry has no value once the next sample exists. */
    if (twai_transmit(&msg, pdMS_TO_TICKS(5)) == ESP_OK) {
        s_tx_count++;
    } else {
        s_rx_dropped++;
    }
}

/* ------------------------------------------------------------ receive */
static void handle_nmt(const twai_message_t *msg)
{
    if (msg->data_length_code < 2) {
        return;
    }
    uint8_t cmd = msg->data[0];
    uint8_t target = msg->data[1];
    if (target != 0 && target != CANOPEN_NODE_ID) {
        return;   /* broadcast (0) or addressed to us only */
    }

    switch (cmd) {
    case NMT_CMD_START:
        s_nmt_state = NMT_STATE_OPERATIONAL;
        ESP_LOGI(TAG, "NMT: -> OPERATIONAL");
        break;
    case NMT_CMD_STOP:
        s_nmt_state = NMT_STATE_STOPPED;
        control_request_estop(true);
        ESP_LOGW(TAG, "NMT: -> STOPPED (drive commanded to stop)");
        break;
    case NMT_CMD_ENTER_PREOP:
        s_nmt_state = NMT_STATE_PREOP;
        control_request_estop(true);
        break;
    case NMT_CMD_RESET_NODE:
    case NMT_CMD_RESET_COMM: {
        s_nmt_state = NMT_STATE_PREOP;
        control_request_estop(false);
        uint8_t boot = NMT_STATE_BOOTUP;
        send_frame(COB(COB_HEARTBEAT_BASE, CANOPEN_NODE_ID), &boot, 1);
        ESP_LOGI(TAG, "NMT: reset, boot-up message sent");
        break;
    }
    default:
        break;
    }
}

static void handle_setpoint(const twai_message_t *msg, int64_t t_rx_us)
{
    if (msg->data_length_code < (int)sizeof(pdo_setpoint_t)) {
        ESP_LOGW(TAG, "short RPDO1 (%d bytes), ignored", msg->data_length_code);
        return;
    }
    if (s_nmt_state != NMT_STATE_OPERATIONAL) {
        /* Outside Operational a PDO is not valid CANopen traffic.  Silently
         * obeying it would let the drive move before the master had actually
         * started the node. */
        return;
    }

    setpoint_msg_t out;
    memcpy(&out.sp, msg->data, sizeof(out.sp));
    out.t_rx_us = t_rx_us;
    out.fresh = true;
    mailbox_publish(&g_setpoint_mb, &out);
}

static void handle_limits(const twai_message_t *msg)
{
    if (msg->data_length_code < (int)sizeof(pdo_limits_t)) {
        return;
    }
    pdo_limits_t limits;
    memcpy(&limits, msg->data, sizeof(limits));
    control_apply_limits(&limits);
}

static void canopen_rx_task(void *arg)
{
    (void)arg;
    const uint32_t id_rpdo1 = COB(COB_RPDO1_BASE, CANOPEN_NODE_ID);
    const uint32_t id_rpdo2 = COB(COB_RPDO2_BASE, CANOPEN_NODE_ID);
    ESP_LOGI(TAG, "rx task on core %d (prio %d)", xPortGetCoreID(), PRIO_CAN_RX);

    for (;;) {
        twai_message_t msg;
        if (twai_receive(&msg, pdMS_TO_TICKS(100)) != ESP_OK) {
            continue;
        }
        /* Timestamp immediately: everything after this is our own latency and
         * is what the node reports in TPDO3. */
        int64_t t_rx_us = esp_timer_get_time();
        s_rx_count++;

        if (msg.extd || msg.rtr) {
            continue;   /* this profile is 11-bit data frames only */
        }

        if (msg.identifier == COB_NMT) {
            handle_nmt(&msg);
        } else if (msg.identifier == id_rpdo1) {
            handle_setpoint(&msg, t_rx_us);
        } else if (msg.identifier == id_rpdo2) {
            handle_limits(&msg);
        }
        /* SYNC (0x080) and everything else is ignored: this node is
         * time-triggered, not SYNC-triggered. */
    }
}

/* ----------------------------------------------------------- transmit */
static void send_status(void)
{
    pdo_status_t pdo = {
        .v_meas_mm_s = g_drive.v_meas_mm_s,
        .w_meas_mrad_s = g_drive.w_meas_mrad_s,
        .state = g_drive.state,
        .faults = g_drive.faults,
        .seq_echo = g_drive.seq_echo,
        .ir_mask = g_drive.ir_mask,
    };
    send_frame(COB(COB_TPDO1_BASE, CANOPEN_NODE_ID), &pdo, sizeof(pdo));
}

static void send_diag(void)
{
    jitter_stats_t snap;
    jitter_snapshot(&g_jitter, &snap);

    /* Control CPU load is exec time over period; the control task's own
     * measurement, not an estimate from the idle hook. */
    uint32_t ctrl_pct = 0;
    if (snap.samples > 0) {
        ctrl_pct = (snap.exec_max_us * 100u) / (uint32_t)CONTROL_PERIOD_US;
    }

    pdo_diag_t pdo = {
        .jitter_p99_us = (uint16_t)jitter_percentile_us(&snap, 99.0f),
        .period_max_us = (uint16_t)(snap.period_max_us > 0xFFFF ? 0xFFFF
                                                                : snap.period_max_us),
        .rpdo_age_ms = (uint16_t)(g_drive.rpdo_age_ms > 0xFFFF ? 0xFFFF
                                                               : g_drive.rpdo_age_ms),
        .ctrl_cpu_pct = (uint8_t)(ctrl_pct > 255 ? 255 : ctrl_pct),
        .comms_cpu_pct = 0,   /* filled by the stats task via g_comms_cpu_pct */
    };
    extern volatile uint8_t g_comms_cpu_pct;
    pdo.comms_cpu_pct = g_comms_cpu_pct;
    send_frame(COB(COB_TPDO2_BASE, CANOPEN_NODE_ID), &pdo, sizeof(pdo));
}

static void send_latency_echo(void)
{
    pdo_latency_echo_t pdo = {
        .rx_to_apply_us = g_drive.echo_rx_to_apply_us,
        .seq_echo = g_drive.echo_seq,
        .watchdog_misses = (uint8_t)(g_drive.watchdog_misses > 255 ? 255
                                     : g_drive.watchdog_misses),
        .rpdo_rate_hz = 10,
        .reserved = 0,
    };
    g_drive.pending_echo = false;
    send_frame(COB(COB_TPDO3_BASE, CANOPEN_NODE_ID), &pdo, sizeof(pdo));
}

static void send_emcy(void)
{
    pdo_emcy_t pdo = {
        .error_code = g_drive.pending_emcy_code,
        .error_register = 0x81,   /* generic + manufacturer-specific */
        .state = g_drive.state,
        .faults = g_drive.faults,
        .reserved = {0, 0, 0},
    };
    g_drive.pending_emcy = false;
    send_frame(COB(COB_EMCY_BASE, CANOPEN_NODE_ID), &pdo, sizeof(pdo));
    ESP_LOGW(TAG, "EMCY sent: code=0x%04X state=%u faults=0x%02X",
             pdo.error_code, pdo.state, pdo.faults);
}

static void check_bus_health(void)
{
    twai_status_info_t status;
    if (twai_get_status_info(&status) != ESP_OK) {
        return;
    }
    if (status.state == TWAI_STATE_BUS_OFF) {
        ESP_LOGE(TAG, "TWAI bus-off; initiating recovery");
        control_notify_bus_off(true);
        twai_initiate_recovery();
    } else if (status.state == TWAI_STATE_STOPPED) {
        twai_start();
    } else if (status.state == TWAI_STATE_RUNNING) {
        control_notify_bus_off(false);
    }
    if (status.rx_missed_count > 0 || status.rx_overrun_count > 0) {
        s_rx_dropped += status.rx_missed_count + status.rx_overrun_count;
    }
}

static void canopen_tx_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(2);
    TickType_t last_wake = xTaskGetTickCount();
    int64_t t_status = 0, t_diag = 0, t_hb = 0, t_health = 0;
    ESP_LOGI(TAG, "tx task on core %d (prio %d)", xPortGetCoreID(), PRIO_CAN_TX);

    for (;;) {
        vTaskDelayUntil(&last_wake, period);
        int64_t now = esp_timer_get_time();

        /* Event-driven frames first: an EMCY or a latency echo is worth more
         * than the next periodic status frame. */
        if (g_drive.pending_emcy) {
            send_emcy();
        }
        if (g_drive.pending_echo) {
            send_latency_echo();
        }
        if (now - t_status >= 1000000 / STATUS_TPDO_HZ) {
            t_status = now;
            send_status();
        }
        if (now - t_diag >= 1000000 / DIAG_TPDO_HZ) {
            t_diag = now;
            send_diag();
        }
        if (now - t_hb >= 1000000 / HEARTBEAT_HZ) {
            t_hb = now;
            uint8_t state = s_nmt_state;
            send_frame(COB(COB_HEARTBEAT_BASE, CANOPEN_NODE_ID), &state, 1);
        }
        if (now - t_health >= 500000) {
            t_health = now;
            check_bus_health();
        }
    }
}

/* ------------------------------------------------------------- startup */
void canopen_start(void)
{
    twai_general_config_t g_cfg =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    /* Deep queues on the comms side absorb bus bursts without ever making the
     * control task wait; this is the memory we happily spend to keep the RT
     * path short. */
    g_cfg.rx_queue_len = 32;
    g_cfg.tx_queue_len = 32;
    g_cfg.intr_flags = ESP_INTR_FLAG_LEVEL1;

#if CAN_BITRATE_KBPS == 1000
    twai_timing_config_t t_cfg = TWAI_TIMING_CONFIG_1MBITS();
#elif CAN_BITRATE_KBPS == 500
    twai_timing_config_t t_cfg = TWAI_TIMING_CONFIG_500KBITS();
#elif CAN_BITRATE_KBPS == 250
    twai_timing_config_t t_cfg = TWAI_TIMING_CONFIG_250KBITS();
#else
#error "unsupported CAN_BITRATE_KBPS"
#endif

    twai_filter_config_t f_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_cfg, &t_cfg, &f_cfg));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "TWAI up at %d kbit/s, node id 0x%02X",
             CAN_BITRATE_KBPS, CANOPEN_NODE_ID);

    uint8_t boot = NMT_STATE_BOOTUP;
    send_frame(COB(COB_HEARTBEAT_BASE, CANOPEN_NODE_ID), &boot, 1);

    xTaskCreatePinnedToCore(canopen_rx_task, "can_rx", STACK_CAN_RX, NULL,
                            PRIO_CAN_RX, NULL, CORE_COMMS);
    xTaskCreatePinnedToCore(canopen_tx_task, "can_tx", STACK_CAN_TX, NULL,
                            PRIO_CAN_TX, NULL, CORE_COMMS);
}

uint8_t canopen_nmt_state(void) { return s_nmt_state; }
uint32_t canopen_rx_count(void) { return s_rx_count; }
uint32_t canopen_tx_count(void) { return s_tx_count; }
uint32_t canopen_rx_dropped(void) { return s_rx_dropped; }
