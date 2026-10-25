/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : jetson_tx.h
  * @brief          : Nonblocking, frame-owned USART3 Jetson TX queues.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef JETSON_TX_H
#define JETSON_TX_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdbool.h>
#include <stdarg.h>
#include <stdint.h>

/* Capacity contract: a frame may carry up to JETSON_TX_FRAME_CAPACITY
 * payload bytes, and the printf staging buffer holds exactly one payload
 * plus its terminating '\0'. Keeping the two tied together means a format
 * that fits the frame can never be falsely reported as an overflow. Every
 * structured emitter must keep its worst-case line within this capacity. */
#define JETSON_TX_FRAME_CAPACITY       192U
#define JETSON_TX_FORMAT_BUFFER_SIZE \
    (JETSON_TX_FRAME_CAPACITY + 1U)
#define JETSON_TX_CRITICAL_QUEUE_DEPTH   8U
#define JETSON_TX_REALTIME_QUEUE_DEPTH  16U
#define JETSON_TX_STATUS_QUEUE_DEPTH    16U
#define JETSON_TX_DEBUG_QUEUE_DEPTH     16U
#define JETSON_TX_BAUD_RATE         460800U
#define JETSON_TX_TIMEOUT_MARGIN_MS     45U
#define JETSON_TX_TIMEOUT_CEILING_MS    50U

typedef enum
{
    JETSON_TX_PRIORITY_CRITICAL = 0,
    JETSON_TX_PRIORITY_REALTIME,
    JETSON_TX_PRIORITY_STATUS,
    JETSON_TX_PRIORITY_DEBUG,
    JETSON_TX_PRIORITY_COUNT
} JetsonTxPriority_t;

typedef struct
{
    uint32_t enqueued[JETSON_TX_PRIORITY_COUNT];
    uint32_t transmitted[JETSON_TX_PRIORITY_COUNT];
    uint32_t dropped[JETSON_TX_PRIORITY_COUNT];
    uint32_t queue_high_water[JETSON_TX_PRIORITY_COUNT];
    uint16_t queue_depth[JETSON_TX_PRIORITY_COUNT];
    uint32_t hal_busy_count;
    uint32_t hal_error_count;
    uint32_t uart_error_count;
    uint32_t last_uart_error_code;
    uint32_t tx_timeout_count;
    uint32_t tx_recovery_count;
    uint32_t resync_delimiter_count;
    uint32_t format_overflow_count;
    uint32_t completion_count;
    bool in_flight;
    JetsonTxPriority_t in_flight_priority;
    uint16_t in_flight_length;
    uint32_t in_flight_start_tick;
    uint32_t in_flight_age_ms;
    uint32_t in_flight_wire_time_ms;
    uint32_t in_flight_generation;
} JetsonTxStats_t;

void JetsonTx_Init(UART_HandleTypeDef *uart);
void JetsonTx_Process(void);
bool JetsonTx_Send(JetsonTxPriority_t priority,
                   const char *data,
                   uint16_t length);
bool JetsonTx_VPrintf(JetsonTxPriority_t priority,
                      const char *format,
                      va_list args);
bool JetsonTx_Printf(JetsonTxPriority_t priority,
                     const char *format,
                     ...);
void JetsonTx_OnTransmitComplete(UART_HandleTypeDef *uart);
void JetsonTx_OnUartErrorFromISR(UART_HandleTypeDef *uart,
                                 uint32_t error_code);
JetsonTxStats_t JetsonTx_GetStats(void);
void JetsonTx_PrintDebug(uint32_t request_nonce);

#ifdef __cplusplus
}
#endif

#endif /* JETSON_TX_H */
