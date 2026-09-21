/* Control-loop period jitter histogram.
 *
 * This is the instrument that backs the priority-separation claim: it is
 * sampled inside the control task itself, so it measures what the control loop
 * actually experienced rather than what a lower-priority observer could see.
 */
#pragma once

#include <stdint.h>

#include "app_config.h"

typedef struct {
    uint32_t bins[JITTER_BINS];   /* |period - nominal| in JITTER_BIN_US steps */
    uint32_t overflow;            /* deviations beyond the last bin */
    uint32_t samples;
    uint32_t period_min_us;
    uint32_t period_max_us;
    uint32_t overruns;            /* loop body exceeded its own period */
    uint64_t period_sum_us;
    uint32_t exec_max_us;         /* worst observed loop body execution time */
} jitter_stats_t;

void jitter_init(jitter_stats_t *j);
void jitter_add(jitter_stats_t *j, uint32_t period_us, uint32_t exec_us);
uint32_t jitter_percentile_us(const jitter_stats_t *j, float pct);
void jitter_snapshot(const jitter_stats_t *src, jitter_stats_t *dst);
void jitter_log(const jitter_stats_t *j, const char *tag);
