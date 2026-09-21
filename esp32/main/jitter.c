#include "jitter.h"

#include <string.h>

#include "esp_log.h"

void jitter_init(jitter_stats_t *j)
{
    memset(j, 0, sizeof(*j));
    j->period_min_us = UINT32_MAX;
}

void jitter_add(jitter_stats_t *j, uint32_t period_us, uint32_t exec_us)
{
    int32_t dev = (int32_t)period_us - (int32_t)CONTROL_PERIOD_US;
    if (dev < 0) {
        dev = -dev;
    }
    uint32_t bin = (uint32_t)dev / JITTER_BIN_US;
    if (bin < JITTER_BINS) {
        j->bins[bin]++;
    } else {
        j->overflow++;
    }

    j->samples++;
    j->period_sum_us += period_us;
    if (period_us < j->period_min_us) {
        j->period_min_us = period_us;
    }
    if (period_us > j->period_max_us) {
        j->period_max_us = period_us;
    }
    if (exec_us > j->exec_max_us) {
        j->exec_max_us = exec_us;
    }
    /* An overrun is the loop body outlasting its own period - that is the
     * failure that actually breaks determinism, distinct from a late wakeup. */
    if (exec_us > (uint32_t)CONTROL_PERIOD_US) {
        j->overruns++;
    }
}

uint32_t jitter_percentile_us(const jitter_stats_t *j, float pct)
{
    if (j->samples == 0) {
        return 0;
    }
    uint32_t target = (uint32_t)((pct / 100.0f) * (float)j->samples);
    uint32_t cum = 0;
    for (int i = 0; i < JITTER_BINS; i++) {
        cum += j->bins[i];
        if (cum >= target) {
            /* Report the upper edge of the bin: it is the honest bound. */
            return (uint32_t)(i + 1) * JITTER_BIN_US;
        }
    }
    return (uint32_t)JITTER_BINS * JITTER_BIN_US;
}

void jitter_snapshot(const jitter_stats_t *src, jitter_stats_t *dst)
{
    /* A plain copy.  The reader is a low-priority task and the writer is the
     * control loop, so a torn read costs at most one slightly wrong stat -
     * which is a far better trade than making the control loop take a lock. */
    memcpy(dst, src, sizeof(*dst));
}

void jitter_log(const jitter_stats_t *j, const char *tag)
{
    static const char *TAG = "jitter";
    if (j->samples == 0) {
        ESP_LOGI(TAG, "%s: no samples", tag);
        return;
    }
    uint32_t mean = (uint32_t)(j->period_sum_us / j->samples);
    ESP_LOGI(TAG,
             "%s: n=%u period min/mean/max=%u/%u/%u us  "
             "|dev| p50=%u p95=%u p99=%u p99.9=%u us  exec_max=%u us  "
             "overruns=%u overflow=%u",
             tag, (unsigned)j->samples, (unsigned)j->period_min_us,
             (unsigned)mean, (unsigned)j->period_max_us,
             (unsigned)jitter_percentile_us(j, 50.0f),
             (unsigned)jitter_percentile_us(j, 95.0f),
             (unsigned)jitter_percentile_us(j, 99.0f),
             (unsigned)jitter_percentile_us(j, 99.9f),
             (unsigned)j->exec_max_us, (unsigned)j->overruns,
             (unsigned)j->overflow);
}
