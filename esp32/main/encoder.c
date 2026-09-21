#include "encoder.h"

#include <math.h>

#include "app_config.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"

static const char *TAG = "encoder";

#ifndef PI_F
#define PI_F 3.14159265358979323846f
#endif

/* PCNT is 16-bit; these watch points let the driver accumulate past it. */
#define PCNT_HIGH_LIMIT  30000
#define PCNT_LOW_LIMIT  -30000

static pcnt_unit_handle_t s_units[MOTOR_COUNT];
static int s_last[MOTOR_COUNT];

/* The right wheel faces the opposite way on a differential base, so forward
 * motion produces opposite count signs.  Correct it once, here. */
static const int SIGN[MOTOR_COUNT] = {[MOTOR_LEFT] = +1, [MOTOR_RIGHT] = -1};

static void setup_unit(motor_id_t id, int gpio_a, int gpio_b)
{
    pcnt_unit_config_t unit_cfg = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
        .flags.accum_count = true,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &s_units[id]));

    /* A short glitch filter kills encoder contact bounce without eating real
     * edges: 1 us at our top wheel speed is far below one count period. */
    pcnt_glitch_filter_config_t filter = {.max_glitch_ns = 1000};
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(s_units[id], &filter));

    pcnt_chan_config_t chan_a_cfg = {.edge_gpio_num = gpio_a, .level_gpio_num = gpio_b};
    pcnt_chan_config_t chan_b_cfg = {.edge_gpio_num = gpio_b, .level_gpio_num = gpio_a};
    pcnt_channel_handle_t chan_a = NULL, chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(s_units[id], &chan_a_cfg, &chan_a));
    ESP_ERROR_CHECK(pcnt_new_channel(s_units[id], &chan_b_cfg, &chan_b));

    /* Full 4x quadrature decode: count both edges of both channels, with the
     * other channel's level selecting the direction. */
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_a,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_a,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_b,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_units[id], PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_units[id], PCNT_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(s_units[id]));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(s_units[id]));
    ESP_ERROR_CHECK(pcnt_unit_start(s_units[id]));
    s_last[id] = 0;
}

void encoder_init(void)
{
    setup_unit(MOTOR_LEFT, ENC_L_A_GPIO, ENC_L_B_GPIO);
    setup_unit(MOTOR_RIGHT, ENC_R_A_GPIO, ENC_R_B_GPIO);
    ESP_LOGI(TAG, "quadrature encoders ready (CPR=%.0f)", (double)ENCODER_CPR);
}

int32_t encoder_read_delta(motor_id_t id)
{
    int now = 0;
    if (pcnt_unit_get_count(s_units[id], &now) != ESP_OK) {
        return 0;
    }
    int32_t delta = (int32_t)(now - s_last[id]);
    s_last[id] = now;
    return delta * SIGN[id];
}

float encoder_counts_to_mm_s(int32_t counts, float dt_s)
{
    if (dt_s <= 0.0f) {
        return 0.0f;
    }
    float revs = (float)counts / ENCODER_CPR;
    float distance_mm = revs * 2.0f * PI_F * WHEEL_RADIUS_M * 1000.0f;
    return distance_mm / dt_s;
}
