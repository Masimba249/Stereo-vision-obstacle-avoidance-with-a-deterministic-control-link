/* Comms-load generator: the experiment that tests the isolation claim.
 *
 * It does not make the product better - it exists to try to break the control
 * loop's timing from the comms side, so that the jitter numbers mean
 * something.  Enable with -DENABLE_LOAD_GENERATOR=1 (see docs/jitter-report.md).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

void loadgen_start(void);
void loadgen_set_enabled(bool enabled);
bool loadgen_enabled(void);
uint32_t loadgen_frames_sent(void);
