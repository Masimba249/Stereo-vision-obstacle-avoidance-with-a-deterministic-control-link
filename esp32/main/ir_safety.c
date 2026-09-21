#include "ir_safety.h"

#include "app_config.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "ir";

static const struct {
    int gpio;
    uint8_t mask;
} SENSORS[] = {
    {IR_LEFT_GPIO,   IR_MASK_LEFT},
    {IR_CENTER_GPIO, IR_MASK_CENTER},
    {IR_RIGHT_GPIO,  IR_MASK_RIGHT},
};
#define IR_SENSOR_COUNT (sizeof(SENSORS) / sizeof(SENSORS[0]))

/* Written by the ISR, read by the control task.  Single byte, single writer:
 * a plain volatile read is coherent and costs the control loop nothing. */
static volatile uint8_t s_mask = 0;
static volatile int64_t s_last_edge_us[IR_SENSOR_COUNT];

static void IRAM_ATTR ir_isr(void *arg)
{
    int idx = (int)(intptr_t)arg;
    int64_t now = esp_timer_get_time();
    /* Debounce in the ISR by timestamp rather than by delay: an ISR must not
     * block, and a bouncing sensor must not be able to flood the CPU. */
    if (now - s_last_edge_us[idx] < IR_DEBOUNCE_US) {
        return;
    }
    s_last_edge_us[idx] = now;

    if (gpio_get_level(SENSORS[idx].gpio) == IR_ACTIVE_LEVEL) {
        s_mask |= SENSORS[idx].mask;
    } else {
        s_mask &= (uint8_t)~SENSORS[idx].mask;
    }
}

void ir_safety_init(void)
{
    uint64_t pins = 0;
    for (size_t i = 0; i < IR_SENSOR_COUNT; i++) {
        pins |= (1ULL << SENSORS[i].gpio);
    }
    gpio_config_t io = {
        .pin_bit_mask = pins,
        .mode = GPIO_MODE_INPUT,
        /* Pull-up with active-low sensors: an unplugged or broken-wire sensor
         * reads inactive-high... which is why the initial sample below also
         * seeds the mask, and why a stuck-high line is caught by the sensor
         * self-test in the field procedure rather than assumed away. */
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));

    int64_t now = esp_timer_get_time();
    for (size_t i = 0; i < IR_SENSOR_COUNT; i++) {
        s_last_edge_us[i] = now;
        ESP_ERROR_CHECK(gpio_isr_handler_add(SENSORS[i].gpio, ir_isr,
                                             (void *)(intptr_t)i));
        /* Seed from the current level: if we boot with an obstacle already in
         * front of the sensor there is no edge to wait for. */
        if (gpio_get_level(SENSORS[i].gpio) == IR_ACTIVE_LEVEL) {
            s_mask |= SENSORS[i].mask;
        }
    }
    ESP_LOGI(TAG, "IR safety layer armed, initial mask=0x%02X", s_mask);
}

uint8_t ir_safety_mask(void)
{
    return s_mask;
}

bool ir_safety_blocked(void)
{
    return s_mask != 0;
}
