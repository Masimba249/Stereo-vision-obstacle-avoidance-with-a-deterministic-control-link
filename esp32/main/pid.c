#include "pid.h"

#include <math.h>

/* M_PI is a POSIX extension, not standard C, and is not guaranteed to be
 * visible under -std=c11. Define it ourselves rather than depend on the
 * toolchain's headers. */
#ifndef PI_F
#define PI_F 3.14159265358979323846f
#endif

void pid_init(pid_t *p, float kp, float ki, float kd, float i_limit,
              float d_lpf_hz, float dt_s, float out_min, float out_max)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->i_limit = i_limit;
    /* Discrete one-pole coefficient for the D term.  Unfiltered derivative on
     * a quantised encoder signal is mostly amplified counting noise. */
    float rc = 1.0f / (2.0f * PI_F * d_lpf_hz);
    p->d_alpha = dt_s / (rc + dt_s);
    p->out_min = out_min;
    p->out_max = out_max;
    pid_reset(p);
}

void pid_reset(pid_t *p)
{
    p->integral = 0.0f;
    p->prev_error = 0.0f;
    p->d_filtered = 0.0f;
    p->initialised = false;
}

float pid_update(pid_t *p, float setpoint, float measurement, float dt_s)
{
    float error = setpoint - measurement;

    float d_raw = 0.0f;
    if (p->initialised && dt_s > 0.0f) {
        d_raw = (error - p->prev_error) / dt_s;
    }
    p->prev_error = error;
    p->initialised = true;
    p->d_filtered += p->d_alpha * (d_raw - p->d_filtered);

    /* Provisional output without the new integral contribution. */
    float unintegrated = p->kp * error + p->kd * p->d_filtered;

    float integral = p->integral + error * dt_s;
    if (integral > p->i_limit) {
        integral = p->i_limit;
    } else if (integral < -p->i_limit) {
        integral = -p->i_limit;
    }

    float out = unintegrated + p->ki * integral;

    /* Conditional integration: only commit the integral if the output is not
     * saturated, or if the error would drive it back out of saturation.  This
     * is what stops the drive from lurching when a failsafe ramp releases. */
    bool saturated_high = out > p->out_max;
    bool saturated_low = out < p->out_min;
    if ((saturated_high && error > 0.0f) || (saturated_low && error < 0.0f)) {
        out = unintegrated + p->ki * p->integral;   /* keep the old integral */
    } else {
        p->integral = integral;
    }

    if (out > p->out_max) {
        out = p->out_max;
    } else if (out < p->out_min) {
        out = p->out_min;
    }
    return out;
}
