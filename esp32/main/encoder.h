/* Quadrature wheel encoders via the ESP32 PCNT peripheral.
 *
 * PCNT counts in hardware, so the 1 kHz control loop only reads a register -
 * no edge interrupts competing with the control task for CPU time.
 */
#pragma once

#include "motor.h"

void encoder_init(void);
/* Counts accumulated since the previous call, sign-corrected per wheel. */
int32_t encoder_read_delta(motor_id_t id);
/* Wheel speed in mm/s from a count delta over dt seconds. */
float encoder_counts_to_mm_s(int32_t counts, float dt_s);
