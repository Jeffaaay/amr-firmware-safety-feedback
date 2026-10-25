#include "estop_status.h"
#include "jetson_tx.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

GPIO_TypeDef host_gpio_d;
static uint32_t tick;
static GPIO_PinState pin;
static bool reject_critical;
static bool reject_status;
static unsigned calls;
static unsigned critical_calls;
static unsigned status_calls;
static unsigned raw, active, valid;
static unsigned long seq, t_ms, presses, change_ms;
static JetsonTxPriority_t priority;
static char line[JETSON_TX_FORMAT_BUFFER_SIZE];
static char last_format[160];

uint32_t HAL_GetTick(void) { return tick; }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t mask)
{
    assert(port == GPIOD && mask == GPIO_PIN_15);
    return pin;
}
bool JetsonTx_Printf(JetsonTxPriority_t p, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    assert(len > 0 && (unsigned)len <= JETSON_TX_FRAME_CAPACITY);
    assert(line[len - 2] == '\r' && line[len - 1] == '\n');
    assert(strlen(fmt) < sizeof(last_format));
    strcpy(last_format, fmt);
    assert(sscanf(line, "ESTOP v=1 seq=%lu t_ms=%lu raw=%u active=%u valid=%u presses=%lu change_ms=%lu",
                  &seq, &t_ms, &raw, &active, &valid, &presses, &change_ms) == 7);
    assert(raw <= 1U && active <= 1U && valid == 1U);
    priority = p;
    calls++;
    if (p == JETSON_TX_PRIORITY_CRITICAL) {
        critical_calls++;
        return !reject_critical;
    }
    assert(p == JETSON_TX_PRIORITY_STATUS);
    status_calls++;
    return !reject_status;
}
static void update(uint32_t at, GPIO_PinState level)
{
    tick = at;
    pin = level;
    EstopStatus_Update();
}
static void start(uint32_t at, GPIO_PinState level)
{
    tick = at;
    pin = level;
    calls = critical_calls = status_calls = 0;
    reject_critical = reject_status = false;
    EstopStatus_Init();
    EstopStatus_Update();
    assert(calls == 1U && seq == 1U && priority == JETSON_TX_PRIORITY_CRITICAL);
}
int main(void)
{
    /* No fabricated valid data before initialization. */
    update(0U, GPIO_PIN_RESET);
    assert(calls == 0U);
    start(1000U, GPIO_PIN_RESET);
    assert(raw == 0U && active == 0U && presses == 0U && change_ms == 1000U);
    update(1099U, GPIO_PIN_RESET);
    assert(calls == 1U);
    update(1100U, GPIO_PIN_RESET);
    assert(calls == 2U && seq == 2U && priority == JETSON_TX_PRIORITY_STATUS);

    /* Assertion is immediate on the first observed high; release is filtered. */
    update(1101U, GPIO_PIN_SET);
    assert(active == 1U && presses == 1U && change_ms == 1101U && t_ms == 1101U);
    update(1152U, GPIO_PIN_RESET);
    update(1181U, GPIO_PIN_RESET);
    assert(calls == 3U);
    update(1182U, GPIO_PIN_RESET);
    assert(calls == 4U && active == 0U && presses == 1U && change_ms == 1182U);

    /* Boot while pressed is active, counted once; a bounce restarts release. */
    start(0U, GPIO_PIN_SET);
    assert(active == 1U && presses == 1U);
    update(10U, GPIO_PIN_RESET);
    update(39U, GPIO_PIN_RESET);
    update(40U, GPIO_PIN_SET);
    update(60U, GPIO_PIN_RESET);
    update(89U, GPIO_PIN_RESET);
    assert(calls == 1U);
    update(90U, GPIO_PIN_RESET);
    assert(calls == 2U && active == 0U && change_ms == 90U && presses == 1U);

    /* Both edges within the event interval coalesce, but the press is counted. */
    start(0U, GPIO_PIN_RESET);
    update(1U, GPIO_PIN_SET);
    update(2U, GPIO_PIN_RESET);
    update(32U, GPIO_PIN_RESET);
    assert(calls == 1U);
    update(50U, GPIO_PIN_RESET);
    assert(active == 0U && presses == 1U && change_ms == 32U && seq == 2U);

    /* Critical queue rejection retries latest state at 50 ms, without a flood.
     * Periodic STATUS is still attempted at 100 ms despite critical overload. */
    start(0U, GPIO_PIN_RESET);
    reject_critical = true;
    update(50U, GPIO_PIN_SET);
    assert(seq == 2U && active == 1U);
    for (uint32_t i = 51U; i < 100U; ++i) update(i, GPIO_PIN_SET);
    assert(calls == 2U);
    update(100U, GPIO_PIN_SET);
    assert(critical_calls == 3U && status_calls == 1U && seq == 4U);
    update(110U, GPIO_PIN_RESET);
    update(140U, GPIO_PIN_RESET);
    reject_critical = false;
    update(150U, GPIO_PIN_RESET);
    assert(priority == JETSON_TX_PRIORITY_CRITICAL && active == 0U && presses == 1U);
    update(249U, GPIO_PIN_RESET);
    assert(calls == 5U);
    reject_status = true;
    update(250U, GPIO_PIN_RESET);
    assert(calls == 6U);
    update(251U, GPIO_PIN_RESET);
    assert(calls == 6U);
    reject_status = false;
    update(350U, GPIO_PIN_RESET);
    assert(calls == 7U && active == 0U);

    /* During release filtering raw=0 but active=1 is intentional. */
    start(0U, GPIO_PIN_SET);
    update(90U, GPIO_PIN_RESET);
    update(100U, GPIO_PIN_RESET);
    assert(raw == 0U && active == 1U && priority == JETSON_TX_PRIORITY_STATUS);

    /* Tick wrap: both periodic cadence and release timer use elapsed uint32. */
    start(UINT32_MAX - 20U, GPIO_PIN_SET);
    update(UINT32_MAX - 10U, GPIO_PIN_RESET);
    update(18U, GPIO_PIN_RESET);
    assert(calls == 1U);
    update(19U, GPIO_PIN_RESET);
    update(29U, GPIO_PIN_RESET);
    assert(calls == 2U && active == 0U && change_ms == 19U);
    update(128U, GPIO_PIN_RESET);
    assert(calls == 2U);
    update(129U, GPIO_PIN_RESET);
    assert(calls == 3U && priority == JETSON_TX_PRIORITY_STATUS);

    /* After a long main-loop stall send one current sample, no catch-up burst. */
    update(100000U, GPIO_PIN_SET);
    assert(calls == 4U && active == 1U && presses == 2U && t_ms == 100000U);
    update(100000U, GPIO_PIN_SET);
    assert(calls == 4U);

    /* Use the production format, not a duplicate, for the longest wire line. */
    int max_len = snprintf(line, sizeof(line), last_format,
                           (unsigned long)UINT32_MAX, (unsigned long)UINT32_MAX,
                           1U, 1U, (unsigned long)UINT32_MAX, (unsigned long)UINT32_MAX);
    assert(max_len == 105 && (unsigned)max_len <= JETSON_TX_FRAME_CAPACITY);
    puts("PASS: PD15 production-C boot/polarity, debounce/bounce, coalescing, queue retry, periodic fallback, tick wrap, stall, 105-byte limit");
    return 0;
}
