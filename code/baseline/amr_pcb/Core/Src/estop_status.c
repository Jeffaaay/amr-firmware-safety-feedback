#include "estop_status.h"
#include "app_config.h"
#include "jetson_tx.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* All state belongs to the main loop. Queues own copies of emitted frames. */
static struct {
    bool initialized;
    bool active;
    bool release_pending;
    bool event_pending;
    bool event_attempted;
    uint32_t release_since_ms;
    uint32_t change_ms;
    uint32_t presses;
    uint32_t seq;
    uint32_t last_event_attempt_ms;
    uint32_t last_snapshot_ms;
} s_estop;

/* Four uint32 decimals (10 digits each), three binary fields, fixed text and
 * CRLF: at most 105 wire bytes. The TX manager adds its own NUL storage. */
_Static_assert(JETSON_TX_FRAME_CAPACITY >= 105U,
               "ESTOP v1 must fit a complete TX frame");

static bool Emit(JetsonTxPriority_t priority, uint32_t now, bool raw)
{
    /* Sequence numbers describe enqueue attempts, not UART delivery ACKs.
     * Rejected attempts leave gaps; unsigned counters wrap modulo 2^32. */
    s_estop.seq++;
    return JetsonTx_Printf(priority,
        "ESTOP v=1 seq=%lu t_ms=%lu raw=%u active=%u valid=1 "
        "presses=%lu change_ms=%lu\r\n",
        (unsigned long)s_estop.seq, (unsigned long)now,
        raw ? 1U : 0U, s_estop.active ? 1U : 0U,
        (unsigned long)s_estop.presses, (unsigned long)s_estop.change_ms);
}

void EstopStatus_Init(void)
{
    memset(&s_estop, 0, sizeof(s_estop));
    const bool raw = HAL_GPIO_ReadPin(ESTOP_INPUT_GPIO_Port,
                                      ESTOP_INPUT_Pin) == GPIO_PIN_SET;
    const uint32_t now = HAL_GetTick();
    /* Fixed hardware polarity: do not learn a baseline from the boot level. */
    s_estop.active = raw;
    s_estop.presses = raw ? 1U : 0U;
    s_estop.change_ms = now;
    s_estop.last_snapshot_ms = now;
    s_estop.event_pending = true;
    s_estop.initialized = true;
}

void EstopStatus_Update(void)
{
    if (!s_estop.initialized) {
        return;
    }
    const bool raw = HAL_GPIO_ReadPin(ESTOP_INPUT_GPIO_Port,
                                      ESTOP_INPUT_Pin) == GPIO_PIN_SET;
    const uint32_t now = HAL_GetTick();
    if (raw) {
        s_estop.release_pending = false;
        if (!s_estop.active) {
            s_estop.active = true;
            s_estop.presses++;
            s_estop.change_ms = now;
            s_estop.event_pending = true;
        }
    } else if (s_estop.active) {
        if (!s_estop.release_pending) {
            s_estop.release_pending = true;
            s_estop.release_since_ms = now;
        }
        if ((uint32_t)(now - s_estop.release_since_ms) >=
            ESTOP_RELEASE_DEBOUNCE_MS) {
            s_estop.active = false;
            s_estop.release_pending = false;
            s_estop.change_ms = now;
            s_estop.event_pending = true;
        }
    }

    if (s_estop.event_pending &&
        (!s_estop.event_attempted ||
         (uint32_t)(now - s_estop.last_event_attempt_ms) >=
             ESTOP_EVENT_MIN_INTERVAL_MS)) {
        s_estop.event_attempted = true;
        s_estop.last_event_attempt_ms = now;
        if (Emit(JETSON_TX_PRIORITY_CRITICAL, now, raw)) {
            s_estop.event_pending = false;
            s_estop.last_snapshot_ms = now;
            return;
        }
        /* Failed critical enqueue retains the latest state/count; retry after
         * 50 ms. Periodic STATUS remains independent when CRITICAL is full. */
    }
    if ((uint32_t)(now - s_estop.last_snapshot_ms) >= ESTOP_PUBLISH_PERIOD_MS) {
        s_estop.last_snapshot_ms = now;
        (void)Emit(JETSON_TX_PRIORITY_STATUS, now, raw);
    }
}
