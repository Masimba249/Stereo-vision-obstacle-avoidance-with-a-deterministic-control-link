/* Stereo-vision obstacle avoidance: ESP32 drive node.
 *
 * Boot order matters and is deliberate:
 *   1. motors, with outputs DISABLED  - nothing can move yet
 *   2. encoders and the IR safety layer - so the control loop has real inputs
 *   3. the control task + its 1 kHz timer - the failsafe is now live
 *   4. CAN, last - the link may only start once something is already
 *      watching it.  Bringing comms up first would create a window where
 *      setpoints could arrive with no watchdog running.
 */
#include <stdio.h>

#include "app_config.h"
#include "canopen.h"
#include "control.h"
#include "encoder.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ir_safety.h"
#include "jitter.h"
#include "loadgen.h"
#include "motor.h"

static const char *TAG = "app";

/* Published to TPDO2 by the comms task. */
volatile uint8_t g_comms_cpu_pct = 0;

static void stats_task(void *arg)
{
    (void)arg;
    uint32_t last_loops = 0;
    int64_t last_us = esp_timer_get_time();

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));

        int64_t now = esp_timer_get_time();
        float elapsed_s = (float)(now - last_us) / 1e6f;
        last_us = now;

        uint32_t loops = control_loop_count();
        float rate = (float)(loops - last_loops) / elapsed_s;
        last_loops = loops;

        jitter_stats_t snap;
        jitter_snapshot(&g_jitter, &snap);
        jitter_log(&snap, loadgen_enabled() ? "LOADED " : "IDLE   ");

        ESP_LOGI(TAG,
                 "loop=%.1f Hz  state=%u faults=0x%02X  v=%d mm/s  "
                 "rpdo_age=%u ms  wd_misses=%u  can rx=%u tx=%u dropped=%u  "
                 "loadgen=%s(%u frames)  heap=%u",
                 (double)rate, g_drive.state, g_drive.faults,
                 (int)g_drive.v_meas_mm_s, (unsigned)g_drive.rpdo_age_ms,
                 (unsigned)g_drive.watchdog_misses, (unsigned)canopen_rx_count(),
                 (unsigned)canopen_tx_count(), (unsigned)canopen_rx_dropped(),
                 loadgen_enabled() ? "on" : "off",
                 (unsigned)loadgen_frames_sent(),
                 (unsigned)esp_get_free_heap_size());

        /* Crude but honest comms-load figure: CAN frames handled per second
         * against the theoretical frame rate at this bitrate. */
        uint32_t frames = canopen_rx_count() + canopen_tx_count();
        static uint32_t last_frames = 0;
        float fps = (float)(frames - last_frames) / elapsed_s;
        last_frames = frames;
        float max_fps = (float)(CAN_BITRATE_KBPS * 1000) / 128.0f;
        uint32_t pct = (uint32_t)(fps * 100.0f / max_fps);
        g_comms_cpu_pct = (uint8_t)(pct > 100 ? 100 : pct);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "stereo-link drive node starting");
    ESP_LOGI(TAG, "control: core %d prio %d @ %d Hz | comms: core %d prio %d/%d",
             CORE_CONTROL, PRIO_CONTROL, CONTROL_HZ, CORE_COMMS,
             PRIO_CAN_RX, PRIO_CAN_TX);

    motor_init();          /* outputs start disabled */
    encoder_init();
    ir_safety_init();

    control_task_start();  /* watchdog is live from here on */
    vTaskDelay(pdMS_TO_TICKS(50));

    canopen_start();       /* only now may setpoints arrive */
    loadgen_start();

    xTaskCreatePinnedToCore(stats_task, "stats", STACK_STATS, NULL,
                            PRIO_STATS, NULL, CORE_COMMS);

    ESP_LOGI(TAG, "boot complete");
}
