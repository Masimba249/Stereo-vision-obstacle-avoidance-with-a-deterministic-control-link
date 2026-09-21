/* Fixed-period PI(D) velocity controller with integral anti-windup. */
#pragma once

#include <stdbool.h>

typedef struct {
    float kp, ki, kd;
    float i_limit;
    float d_alpha;       /* one-pole low-pass on the derivative term */
    float integral;
    float prev_error;
    float d_filtered;
    float out_min, out_max;
    bool  initialised;
} pid_t;

void pid_init(pid_t *p, float kp, float ki, float kd, float i_limit,
              float d_lpf_hz, float dt_s, float out_min, float out_max);
void pid_reset(pid_t *p);
float pid_update(pid_t *p, float setpoint, float measurement, float dt_s);
