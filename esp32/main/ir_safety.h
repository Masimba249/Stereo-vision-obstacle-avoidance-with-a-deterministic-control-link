/* IR proximity sensors: the fast safety layer.
 *
 * This layer is deliberately independent of both the vision pipeline and the
 * CAN link.  Vision can be wrong, and the link can be gone; the IR path still
 * stops the drive, and it reacts within one control tick (1 ms) rather than
 * one vision frame (100 ms).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

void ir_safety_init(void);
/* Bitmask of currently-asserted sensors (IR_MASK_LEFT/CENTER/RIGHT). */
uint8_t ir_safety_mask(void);
/* True when anything in the forward path is blocked. */
bool ir_safety_blocked(void);
