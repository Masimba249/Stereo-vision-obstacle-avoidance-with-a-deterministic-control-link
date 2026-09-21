/* Lock-free single-writer / single-reader handoff of the latest setpoint.
 *
 * The control task must never block on the comms task.  A mutex here would
 * introduce exactly the priority inversion the task split exists to avoid: the
 * 1 kHz control loop at priority 23 would wait on a priority-12 CAN task.
 *
 * This is a seqlock.  The writer bumps an odd counter, writes, then bumps it
 * even.  The reader snapshots the counter, copies, and re-checks; an odd or
 * changed counter means it raced and it retries.
 *
 * Safety of the retry: the writer runs on CORE_COMMS and the reader on
 * CORE_CONTROL, so the writer always makes progress even while the reader
 * spins.  Were they ever pinned to the same core, a higher-priority reader
 * could spin forever against a preempted writer - so the retry count is also
 * bounded, and on exhaustion the reader keeps its previous value.  A slightly
 * stale setpoint is safe; a stalled control loop is not.
 */
#pragma once

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>

#include "pdo_defs.h"

#define MAILBOX_MAX_RETRIES 4

typedef struct {
    pdo_setpoint_t sp;
    int64_t        t_rx_us;     /* esp_timer_get_time() at TWAI receive */
    bool           fresh;       /* not yet consumed by the control loop */
} setpoint_msg_t;

typedef struct {
    _Atomic uint32_t seq;
    setpoint_msg_t   data;
} setpoint_mailbox_t;

static inline void mailbox_init(setpoint_mailbox_t *mb)
{
    atomic_store(&mb->seq, 0);
    mb->data = (setpoint_msg_t){0};
}

/* Writer side: called from the CAN RX task only. */
static inline void mailbox_publish(setpoint_mailbox_t *mb, const setpoint_msg_t *msg)
{
    uint32_t s = atomic_load_explicit(&mb->seq, memory_order_relaxed);
    atomic_store_explicit(&mb->seq, s + 1, memory_order_relaxed);  /* -> odd */
    atomic_thread_fence(memory_order_release);
    mb->data = *msg;
    atomic_thread_fence(memory_order_release);
    atomic_store_explicit(&mb->seq, s + 2, memory_order_relaxed);  /* -> even */
}

/* Reader side: called from the control task only.
 * Returns true if *out holds a coherent snapshot. */
static inline bool mailbox_read(setpoint_mailbox_t *mb, setpoint_msg_t *out)
{
    for (int i = 0; i < MAILBOX_MAX_RETRIES; i++) {
        uint32_t s1 = atomic_load_explicit(&mb->seq, memory_order_relaxed);
        if (s1 & 1u) {
            continue;                   /* writer mid-update */
        }
        atomic_thread_fence(memory_order_acquire);
        *out = mb->data;
        atomic_thread_fence(memory_order_acquire);
        uint32_t s2 = atomic_load_explicit(&mb->seq, memory_order_relaxed);
        if (s1 == s2) {
            return true;                /* clean snapshot */
        }
    }
    return false;
}

/* Clear the freshness flag once the control loop has acted on a setpoint, so
 * the latency echo is emitted exactly once per received frame. */
static inline void mailbox_mark_consumed(setpoint_mailbox_t *mb)
{
    mb->data.fresh = false;
}
