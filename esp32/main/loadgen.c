#include "loadgen.h"

#include "app_config.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "loadgen";

static volatile bool s_enabled = (ENABLE_LOAD_GENERATOR != 0);
static volatile uint32_t s_frames;

static void loadgen_task(void *arg)
{
    (void)arg;
    ESP_LOGW(TAG, "comms load generator on core %d (prio %d) - "
                  "this is a test fixture, not production behaviour",
             xPortGetCoreID(), PRIO_LOADGEN);
    TickType_t last_wake = xTaskGetTickCount();
    uint8_t counter = 0;

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LOADGEN_PERIOD_MS));
        if (!s_enabled) {
            continue;
        }

        /* Saturate the TX queue and the bus with low-priority (high COB-ID)
         * frames, and burn CPU on core 0 at the same time.  Both of the ways
         * comms could plausibly steal time from control, at once. */
        for (int i = 0; i < LOADGEN_BURST_FRAMES; i++) {
            twai_message_t msg = {
                .identifier = 0x7F0 + (i & 0x0F),
                .data_length_code = 8,
            };
            msg.extd = 0;
            msg.rtr = 0;
            for (int b = 0; b < 8; b++) {
                msg.data[b] = counter++;
            }
            if (twai_transmit(&msg, 0) == ESP_OK) {
                s_frames++;
            }
        }

        /* A busy stretch on the comms core, to make sure the isolation being
         * measured is scheduler isolation and not just an idle second core. */
        volatile uint32_t sink = 0;
        for (uint32_t k = 0; k < 20000; k++) {
            sink += k * 2654435761u;
        }
        (void)sink;
    }
}

void loadgen_start(void)
{
    xTaskCreatePinnedToCore(loadgen_task, "loadgen", STACK_LOADGEN, NULL,
                            PRIO_LOADGEN, NULL, CORE_COMMS);
}

void loadgen_set_enabled(bool enabled)
{
    s_enabled = enabled;
    ESP_LOGW(TAG, "load generator %s", enabled ? "ENABLED" : "disabled");
}

bool loadgen_enabled(void) { return s_enabled; }
uint32_t loadgen_frames_sent(void) { return s_frames; }
