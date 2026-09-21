#include "motor.h"

#include <math.h>

#include "app_config.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "motor";

static const struct {
    int pwm_gpio, in1_gpio, in2_gpio;
    ledc_channel_t channel;
} MOTORS[MOTOR_COUNT] = {
    [MOTOR_LEFT]  = {MOTOR_L_PWM_GPIO, MOTOR_L_IN1_GPIO, MOTOR_L_IN2_GPIO, LEDC_CHANNEL_0},
    [MOTOR_RIGHT] = {MOTOR_R_PWM_GPIO, MOTOR_R_IN1_GPIO, MOTOR_R_IN2_GPIO, LEDC_CHANNEL_1},
};

static bool s_enabled = false;

void motor_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = MOTOR_PWM_RES_BITS,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = MOTOR_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    for (int i = 0; i < MOTOR_COUNT; i++) {
        ledc_channel_config_t ch = {
            .gpio_num = MOTORS[i].pwm_gpio,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = MOTORS[i].channel,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch));

        gpio_config_t io = {
            .pin_bit_mask = (1ULL << MOTORS[i].in1_gpio) | (1ULL << MOTORS[i].in2_gpio),
            .mode = GPIO_MODE_OUTPUT,
        };
        ESP_ERROR_CHECK(gpio_config(&io));
        gpio_set_level(MOTORS[i].in1_gpio, 0);
        gpio_set_level(MOTORS[i].in2_gpio, 0);
    }

    gpio_config_t stby = {
        .pin_bit_mask = (1ULL << MOTOR_STBY_GPIO),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&stby));
    /* Start disabled.  The drive must not be able to move because the MCU
     * booted; it moves only once the state machine says so. */
    gpio_set_level(MOTOR_STBY_GPIO, 0);
    s_enabled = false;
    ESP_LOGI(TAG, "initialised, outputs disabled");
}

void motor_set(motor_id_t id, float duty)
{
    if (id >= MOTOR_COUNT) {
        return;
    }
    if (!isfinite(duty)) {
        duty = 0.0f;    /* a NaN reaching the PWM register would latch a duty */
    }
    if (duty > 1.0f) {
        duty = 1.0f;
    } else if (duty < -1.0f) {
        duty = -1.0f;
    }

    int magnitude = (int)lroundf(fabsf(duty) * (float)MOTOR_PWM_MAX);
    int forward = (duty > 0.0f);
    int reverse = (duty < 0.0f);

    if (magnitude > 0 && magnitude < MOTOR_MIN_DUTY) {
        /* Below the stiction threshold: commanding it only heats the bridge. */
        magnitude = 0;
        forward = reverse = 0;
    }

    gpio_set_level(MOTORS[id].in1_gpio, forward);
    gpio_set_level(MOTORS[id].in2_gpio, reverse);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTORS[id].channel, magnitude);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTORS[id].channel);
}

void motor_enable_outputs(bool enable)
{
    if (!enable) {
        motor_coast_all();
    }
    gpio_set_level(MOTOR_STBY_GPIO, enable ? 1 : 0);
    s_enabled = enable;
}

bool motor_outputs_enabled(void)
{
    return s_enabled;
}

void motor_coast_all(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        gpio_set_level(MOTORS[i].in1_gpio, 0);
        gpio_set_level(MOTORS[i].in2_gpio, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTORS[i].channel, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTORS[i].channel);
    }
}
