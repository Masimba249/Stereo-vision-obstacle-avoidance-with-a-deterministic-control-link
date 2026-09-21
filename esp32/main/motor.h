/* TB6612FNG dual H-bridge driver: signed duty in [-1, +1] per wheel. */
#pragma once

#include <stdbool.h>

typedef enum { MOTOR_LEFT = 0, MOTOR_RIGHT = 1, MOTOR_COUNT } motor_id_t;

void motor_init(void);
/* Signed normalised command; clamped internally. */
void motor_set(motor_id_t id, float duty);
/* Cuts the H-bridge standby pin: outputs go high-impedance regardless of the
 * PWM state.  This is the hardware-level stop the failsafe ends with. */
void motor_enable_outputs(bool enable);
bool motor_outputs_enabled(void);
void motor_coast_all(void);
