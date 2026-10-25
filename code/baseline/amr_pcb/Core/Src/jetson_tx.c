/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : jetson_tx.c
  * @brief          : Central nonblocking USART3 Jetson frame transmitter.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "jetson_tx.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>

typedef struct
{
    uint16_t length;
    uint8_t data[JETSON_TX_FRAME_CAPACITY];
} JetsonTxFrame_t;

typedef struct
{
    JetsonTxFrame_t *storage;
    uint16_t capacity;
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} JetsonTxQueue_t;

static JetsonTxFrame_t s_critical_storage[JETSON_TX_CRITICAL_QUEUE_DEPTH];
static JetsonTxFrame_t s_realtime_storage[JETSON_TX_REALTIME_QUEUE_DEPTH];
static JetsonTxFrame_t s_status_storage[JETSON_TX_STATUS_QUEUE_DEPTH];
static JetsonTxFrame_t s_debug_storage[JETSON_TX_DEBUG_QUEUE_DEPTH];
static JetsonTxQueue_t s_queues[JETSON_TX_PRIORITY_COUNT];
static JetsonTxFrame_t s_in_flight_frame;
static UART_HandleTypeDef *s_uart;
static JetsonTxStats_t s_stats;
static bool s_initialized;
static volatile bool s_in_flight;
static JetsonTxPriority_t s_in_flight_priority;
static bool s_in_flight_is_resync;
static uint16_t s_in_flight_length;
static uint32_t s_in_flight_start_tick;
static uint32_t s_in_flight_wire_time_ms;
static uint32_t s_in_flight_timeout_ms;
static volatile uint32_t s_in_flight_generation;
static uint32_t s_generation_counter;
static volatile bool s_completion_armed;
static volatile bool s_completion_pending;
static volatile uint32_t s_completion_generation;
static volatile uint32_t s_uart_error_isr_count;
static volatile uint32_t s_uart_error_code;
static uint32_t s_uart_error_processed_count;
static bool s_resync_delimiter_pending;
/* Rolling per-query snapshot id echoed on every UART_TX_DEBUG line. */
static uint16_t s_debug_snapshot_sequence;

static void JetsonTx_IncrementU32(uint32_t *value)
{
    if (*value != UINT32_MAX) {
        (*value)++;
    }
}

static bool JetsonTx_PriorityValid(JetsonTxPriority_t priority)
{
    return (priority >= JETSON_TX_PRIORITY_CRITICAL) &&
           (priority < JETSON_TX_PRIORITY_COUNT);
}

static uint32_t JetsonTx_CalculateWireTimeMs(uint16_t length)
{
    uint32_t numerator = (uint32_t)length * 10U * 1000U;

    return (numerator + JETSON_TX_BAUD_RATE - 1U) /
           JETSON_TX_BAUD_RATE;
}

static uint32_t JetsonTx_CalculateTimeoutMs(uint16_t length)
{
    uint32_t timeout_ms = JetsonTx_CalculateWireTimeMs(length) +
                          JETSON_TX_TIMEOUT_MARGIN_MS;

    if (timeout_ms > JETSON_TX_TIMEOUT_CEILING_MS) {
        timeout_ms = JETSON_TX_TIMEOUT_CEILING_MS;
    }
    return timeout_ms;
}

static void JetsonTx_ConfigureQueue(JetsonTxPriority_t priority,
                                    JetsonTxFrame_t *storage,
                                    uint16_t capacity)
{
    s_queues[priority].storage = storage;
    s_queues[priority].capacity = capacity;
    s_queues[priority].head = 0U;
    s_queues[priority].tail = 0U;
    s_queues[priority].count = 0U;
}

static void JetsonTx_DropOldest(JetsonTxPriority_t priority)
{
    JetsonTxQueue_t *queue = &s_queues[priority];

    if (queue->count == 0U) {
        return;
    }
    queue->head = (uint16_t)((queue->head + 1U) % queue->capacity);
    queue->count--;
    JetsonTx_IncrementU32(&s_stats.dropped[priority]);
}

static bool JetsonTx_Enqueue(JetsonTxPriority_t priority,
                             const char *data,
                             uint16_t length)
{
    JetsonTxQueue_t *queue = &s_queues[priority];
    JetsonTxFrame_t *frame;

    if (queue->count == queue->capacity) {
        if ((priority == JETSON_TX_PRIORITY_REALTIME) ||
            (priority == JETSON_TX_PRIORITY_STATUS)) {
            /* For sampled/state traffic, the newest complete frame is more
             * useful than an older queued frame. In-flight storage is separate
             * and is therefore never overwritten by this policy. */
            JetsonTx_DropOldest(priority);
        } else {
            JetsonTx_IncrementU32(&s_stats.dropped[priority]);
            return false;
        }
    }

    frame = &queue->storage[queue->tail];
    frame->length = length;
    memcpy(frame->data, data, length);
    queue->tail = (uint16_t)((queue->tail + 1U) % queue->capacity);
    queue->count++;
    JetsonTx_IncrementU32(&s_stats.enqueued[priority]);
    if ((uint32_t)queue->count > s_stats.queue_high_water[priority]) {
        s_stats.queue_high_water[priority] = queue->count;
    }
    return true;
}

static bool JetsonTx_QueueOverflowLine(JetsonTxPriority_t priority)
{
    static const char overflow_line[] = "UART_FORMAT_OVERFLOW\r\n";

    return JetsonTx_Enqueue(priority,
                            overflow_line,
                            (uint16_t)(sizeof(overflow_line) - 1U));
}

void JetsonTx_Init(UART_HandleTypeDef *uart)
{
    memset(&s_stats, 0, sizeof(s_stats));
    memset(&s_in_flight_frame, 0, sizeof(s_in_flight_frame));
    JetsonTx_ConfigureQueue(JETSON_TX_PRIORITY_CRITICAL,
                            s_critical_storage,
                            JETSON_TX_CRITICAL_QUEUE_DEPTH);
    JetsonTx_ConfigureQueue(JETSON_TX_PRIORITY_REALTIME,
                            s_realtime_storage,
                            JETSON_TX_REALTIME_QUEUE_DEPTH);
    JetsonTx_ConfigureQueue(JETSON_TX_PRIORITY_STATUS,
                            s_status_storage,
                            JETSON_TX_STATUS_QUEUE_DEPTH);
    JetsonTx_ConfigureQueue(JETSON_TX_PRIORITY_DEBUG,
                            s_debug_storage,
                            JETSON_TX_DEBUG_QUEUE_DEPTH);
    s_uart = uart;
    s_in_flight = false;
    s_in_flight_priority = JETSON_TX_PRIORITY_COUNT;
    s_in_flight_is_resync = false;
    s_in_flight_length = 0U;
    s_in_flight_start_tick = 0U;
    s_in_flight_wire_time_ms = 0U;
    s_in_flight_timeout_ms = 0U;
    s_in_flight_generation = 0U;
    s_generation_counter = 0U;
    s_completion_armed = false;
    s_completion_pending = false;
    s_completion_generation = 0U;
    s_uart_error_isr_count = 0U;
    s_uart_error_code = HAL_UART_ERROR_NONE;
    s_uart_error_processed_count = 0U;
    s_resync_delimiter_pending = false;
    s_debug_snapshot_sequence = 0U;
    s_initialized = (uart != NULL);
}

bool JetsonTx_Send(JetsonTxPriority_t priority,
                   const char *data,
                   uint16_t length)
{
    if (!s_initialized || !JetsonTx_PriorityValid(priority) ||
        (data == NULL) || (length == 0U)) {
        if (JetsonTx_PriorityValid(priority)) {
            JetsonTx_IncrementU32(&s_stats.dropped[priority]);
        }
        return false;
    }
    if (length > JETSON_TX_FRAME_CAPACITY) {
        JetsonTx_IncrementU32(&s_stats.format_overflow_count);
        return JetsonTx_QueueOverflowLine(priority);
    }
    return JetsonTx_Enqueue(priority, data, length);
}

bool JetsonTx_VPrintf(JetsonTxPriority_t priority,
                      const char *format,
                      va_list args)
{
    char buffer[JETSON_TX_FORMAT_BUFFER_SIZE];
    int length;

    if (!s_initialized || !JetsonTx_PriorityValid(priority) ||
        (format == NULL)) {
        if (JetsonTx_PriorityValid(priority)) {
            JetsonTx_IncrementU32(&s_stats.dropped[priority]);
        }
        return false;
    }
    length = vsnprintf(buffer, sizeof(buffer), format, args);
    if ((length <= 0) || (length >= (int)sizeof(buffer))) {
        JetsonTx_IncrementU32(&s_stats.format_overflow_count);
        return JetsonTx_QueueOverflowLine(priority);
    }
    return JetsonTx_Send(priority, buffer, (uint16_t)length);
}

bool JetsonTx_Printf(JetsonTxPriority_t priority,
                     const char *format,
                     ...)
{
    bool result;
    va_list args;

    va_start(args, format);
    result = JetsonTx_VPrintf(priority, format, args);
    va_end(args);
    return result;
}

void JetsonTx_OnTransmitComplete(UART_HandleTypeDef *uart)
{
    if (s_initialized && s_in_flight && s_completion_armed &&
        (uart == s_uart)) {
        s_completion_generation = s_in_flight_generation;
        s_completion_pending = true;
        /* Disarm in the ISR so duplicate callbacks cannot create a second
         * completion for the same manager-owned generation. */
        s_completion_armed = false;
    }
}

void JetsonTx_OnUartErrorFromISR(UART_HandleTypeDef *uart,
                                 uint32_t error_code)
{
    if (!s_initialized || (uart != s_uart)) {
        return;
    }
    s_uart_error_code = error_code;
    if (s_uart_error_isr_count != UINT32_MAX) {
        s_uart_error_isr_count++;
    }
    if (s_in_flight) {
        /* Error recovery in main context owns this generation from now on.
         * A completion arriving after this point is intentionally ignored. */
        s_completion_armed = false;
    }
}

static void JetsonTx_ClearInFlight(void)
{
    s_completion_armed = false;
    s_completion_pending = false;
    s_completion_generation = 0U;
    s_in_flight = false;
    s_in_flight_priority = JETSON_TX_PRIORITY_COUNT;
    s_in_flight_is_resync = false;
    s_in_flight_length = 0U;
    s_in_flight_start_tick = 0U;
    s_in_flight_wire_time_ms = 0U;
    s_in_flight_timeout_ms = 0U;
    s_in_flight_generation = 0U;
}

static void JetsonTx_RecoverInFlight(bool timed_out)
{
    bool failed_resync = s_in_flight_is_resync;
    JetsonTxPriority_t failed_priority = s_in_flight_priority;
    HAL_StatusTypeDef abort_status;

    /* USART3 has no TX DMA in this project. HAL_UART_AbortTransmit therefore
     * only disables TX interrupts, flushes the optional FIFO, and restores the
     * HAL TX state; it contains no polling wait on this configuration. */
    s_completion_armed = false;
    abort_status = HAL_UART_AbortTransmit(s_uart);
    if (abort_status != HAL_OK) {
        JetsonTx_IncrementU32(&s_stats.hal_error_count);
    }
    if (timed_out) {
        JetsonTx_IncrementU32(&s_stats.tx_timeout_count);
    }
    JetsonTx_IncrementU32(&s_stats.tx_recovery_count);
    if (!failed_resync && JetsonTx_PriorityValid(failed_priority)) {
        /* The frame was removed from its queue only after HAL accepted the
         * start. A partial text frame is dropped, never blindly retransmitted. */
        JetsonTx_IncrementU32(&s_stats.dropped[failed_priority]);
    }
    JetsonTx_ClearInFlight();
    /* A standalone CRLF is sent before queued traffic to restore the host line
     * parser boundary after a possibly partial frame. */
    s_resync_delimiter_pending = true;
}

static HAL_StatusTypeDef JetsonTx_StartFrame(JetsonTxPriority_t priority,
                                             uint16_t length,
                                             bool is_resync)
{
    HAL_StatusTypeDef status;
    uint32_t now_ms = HAL_GetTick();

    if (s_generation_counter == UINT32_MAX) {
        s_generation_counter = 1U;
    } else {
        s_generation_counter++;
        if (s_generation_counter == 0U) {
            s_generation_counter = 1U;
        }
    }
    s_completion_pending = false;
    s_completion_generation = 0U;
    s_in_flight = true;
    s_in_flight_priority = priority;
    s_in_flight_is_resync = is_resync;
    s_in_flight_length = length;
    s_in_flight_start_tick = now_ms;
    s_in_flight_wire_time_ms = JetsonTx_CalculateWireTimeMs(length);
    s_in_flight_timeout_ms = JetsonTx_CalculateTimeoutMs(length);
    s_in_flight_generation = s_generation_counter;
    s_completion_armed = true;
    status = HAL_UART_Transmit_IT(s_uart,
                                  s_in_flight_frame.data,
                                  length);
    if (status != HAL_OK) {
        s_completion_armed = false;
        s_in_flight = false;
        s_in_flight_priority = JETSON_TX_PRIORITY_COUNT;
        s_in_flight_is_resync = false;
        s_in_flight_length = 0U;
        s_in_flight_start_tick = 0U;
        s_in_flight_wire_time_ms = 0U;
        s_in_flight_timeout_ms = 0U;
        s_in_flight_generation = 0U;
    }
    return status;
}

void JetsonTx_Process(void)
{
    JetsonTxPriority_t priority;
    JetsonTxQueue_t *queue;
    JetsonTxFrame_t *queued_frame;
    HAL_StatusTypeDef status;
    uint32_t now_ms;
    uint32_t uart_error_count;

    if (!s_initialized) {
        return;
    }
    uart_error_count = s_uart_error_isr_count;
    if (uart_error_count != s_uart_error_processed_count) {
        s_uart_error_processed_count = uart_error_count;
        s_stats.uart_error_count = uart_error_count;
        s_stats.last_uart_error_code = s_uart_error_code;
        if (s_in_flight) {
            JetsonTx_RecoverInFlight(false);
            return;
        }
    }
    if (s_completion_pending) {
        uint32_t completion_generation = s_completion_generation;

        s_completion_pending = false;
        if (s_in_flight &&
            (completion_generation == s_in_flight_generation)) {
            if (s_in_flight_is_resync) {
                JetsonTx_IncrementU32(&s_stats.resync_delimiter_count);
            } else {
                JetsonTx_IncrementU32(
                    &s_stats.transmitted[s_in_flight_priority]);
            }
            JetsonTx_IncrementU32(&s_stats.completion_count);
            JetsonTx_ClearInFlight();
        }
    }
    if (s_in_flight) {
        now_ms = HAL_GetTick();
        if ((uint32_t)(now_ms - s_in_flight_start_tick) >=
            s_in_flight_timeout_ms) {
            JetsonTx_RecoverInFlight(true);
        }
        return;
    }

    if (s_resync_delimiter_pending) {
        static const char delimiter[] = "\r\n";

        memcpy(s_in_flight_frame.data, delimiter,
               sizeof(delimiter) - 1U);
        s_in_flight_frame.length = (uint16_t)(sizeof(delimiter) - 1U);
        status = JetsonTx_StartFrame(JETSON_TX_PRIORITY_CRITICAL,
                                     s_in_flight_frame.length, true);
        if (status == HAL_BUSY) {
            JetsonTx_IncrementU32(&s_stats.hal_busy_count);
            return;
        }
        if (status != HAL_OK) {
            JetsonTx_IncrementU32(&s_stats.hal_error_count);
            return;
        }
        s_resync_delimiter_pending = false;
        return;
    }

    for (priority = JETSON_TX_PRIORITY_CRITICAL;
         priority < JETSON_TX_PRIORITY_COUNT;
         priority = (JetsonTxPriority_t)(priority + 1)) {
        if (s_queues[priority].count != 0U) {
            break;
        }
    }
    if (priority == JETSON_TX_PRIORITY_COUNT) {
        return;
    }

    queue = &s_queues[priority];
    queued_frame = &queue->storage[queue->head];
    s_in_flight_frame = *queued_frame;
    status = JetsonTx_StartFrame(priority,
                                 s_in_flight_frame.length, false);
    if (status == HAL_BUSY) {
        JetsonTx_IncrementU32(&s_stats.hal_busy_count);
        return;
    }
    if (status != HAL_OK) {
        JetsonTx_IncrementU32(&s_stats.hal_error_count);
        return;
    }

    queue->head = (uint16_t)((queue->head + 1U) % queue->capacity);
    queue->count--;
}

JetsonTxStats_t JetsonTx_GetStats(void)
{
    JetsonTxStats_t snapshot = s_stats;
    JetsonTxPriority_t priority;

    for (priority = JETSON_TX_PRIORITY_CRITICAL;
         priority < JETSON_TX_PRIORITY_COUNT;
         priority = (JetsonTxPriority_t)(priority + 1)) {
        snapshot.queue_depth[priority] = s_queues[priority].count;
    }
    snapshot.in_flight = s_in_flight;
    snapshot.in_flight_priority = s_in_flight_priority;
    snapshot.in_flight_length = s_in_flight_length;
    snapshot.in_flight_start_tick = s_in_flight_start_tick;
    snapshot.in_flight_age_ms = s_in_flight ?
        (uint32_t)(HAL_GetTick() - s_in_flight_start_tick) : 0U;
    snapshot.in_flight_wire_time_ms = s_in_flight_wire_time_ms;
    snapshot.in_flight_generation = s_in_flight_generation;
    return snapshot;
}

void JetsonTx_PrintDebug(uint32_t request_nonce)
{
    JetsonTxStats_t stats = JetsonTx_GetStats();
    unsigned snap;

    /* Per-query snapshot id: all six lines of one response carry the same
     * snap= and echo the host's req= so a parser can never stitch lines
     * from different snapshots (or a stale queued response) together. */
    s_debug_snapshot_sequence++;
    snap = (unsigned)s_debug_snapshot_sequence;
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_DEBUG,
        "UART_TX_DEBUG 1/6 active=%u age=%lu len=%u wire=%lu gen=%lu prio=%u snap=%u req=%08lX\r\n",
        stats.in_flight ? 1U : 0U,
        (unsigned long)stats.in_flight_age_ms,
        stats.in_flight_length,
        (unsigned long)stats.in_flight_wire_time_ms,
        (unsigned long)stats.in_flight_generation,
        (unsigned)stats.in_flight_priority,
        snap, (unsigned long)request_nonce);
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_DEBUG,
        "UART_TX_DEBUG 2/6 depth=%u/%u/%u/%u high=%lu/%lu/%lu/%lu snap=%u req=%08lX\r\n",
        stats.queue_depth[JETSON_TX_PRIORITY_CRITICAL],
        stats.queue_depth[JETSON_TX_PRIORITY_REALTIME],
        stats.queue_depth[JETSON_TX_PRIORITY_STATUS],
        stats.queue_depth[JETSON_TX_PRIORITY_DEBUG],
        (unsigned long)stats.queue_high_water[JETSON_TX_PRIORITY_CRITICAL],
        (unsigned long)stats.queue_high_water[JETSON_TX_PRIORITY_REALTIME],
        (unsigned long)stats.queue_high_water[JETSON_TX_PRIORITY_STATUS],
        (unsigned long)stats.queue_high_water[JETSON_TX_PRIORITY_DEBUG],
        snap, (unsigned long)request_nonce);
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_DEBUG,
        "UART_TX_DEBUG 3/6 enqueued=%lu/%lu/%lu/%lu snap=%u req=%08lX\r\n",
        (unsigned long)stats.enqueued[JETSON_TX_PRIORITY_CRITICAL],
        (unsigned long)stats.enqueued[JETSON_TX_PRIORITY_REALTIME],
        (unsigned long)stats.enqueued[JETSON_TX_PRIORITY_STATUS],
        (unsigned long)stats.enqueued[JETSON_TX_PRIORITY_DEBUG],
        snap, (unsigned long)request_nonce);
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_DEBUG,
        "UART_TX_DEBUG 4/6 sent=%lu/%lu/%lu/%lu dropped=%lu/%lu/%lu/%lu snap=%u req=%08lX\r\n",
        (unsigned long)stats.transmitted[JETSON_TX_PRIORITY_CRITICAL],
        (unsigned long)stats.transmitted[JETSON_TX_PRIORITY_REALTIME],
        (unsigned long)stats.transmitted[JETSON_TX_PRIORITY_STATUS],
        (unsigned long)stats.transmitted[JETSON_TX_PRIORITY_DEBUG],
        (unsigned long)stats.dropped[JETSON_TX_PRIORITY_CRITICAL],
        (unsigned long)stats.dropped[JETSON_TX_PRIORITY_REALTIME],
        (unsigned long)stats.dropped[JETSON_TX_PRIORITY_STATUS],
        (unsigned long)stats.dropped[JETSON_TX_PRIORITY_DEBUG],
        snap, (unsigned long)request_nonce);
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_DEBUG,
        "UART_TX_DEBUG 5/6 hal_busy=%lu hal_error=%lu uart_error=%lu last_error=0x%08lX timeout=%lu recovery=%lu snap=%u req=%08lX\r\n",
        (unsigned long)stats.hal_busy_count,
        (unsigned long)stats.hal_error_count,
        (unsigned long)stats.uart_error_count,
        (unsigned long)stats.last_uart_error_code,
        (unsigned long)stats.tx_timeout_count,
        (unsigned long)stats.tx_recovery_count,
        snap, (unsigned long)request_nonce);
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_DEBUG,
        "UART_TX_DEBUG 6/6 format_overflow=%lu completion=%lu resync=%lu snap=%u req=%08lX\r\n",
        (unsigned long)stats.format_overflow_count,
        (unsigned long)stats.completion_count,
        (unsigned long)stats.resync_delimiter_count,
        snap, (unsigned long)request_nonce);
}
