/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : jetson_protocol.h
  * @brief          : Jetson UART command protocol dispatch interface.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef JETSON_PROTOCOL_H
#define JETSON_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void JetsonProtocol_SetRawCommandCallback(void (*fn)(int32_t left_rpm,
                                                     int32_t right_rpm,
                                                     uint32_t duration_ms));
/* Starts the persistent USART3 circular DMA + IDLE/HT/TC transport. The
 * historical function name is retained as an internal call-site compatibility
 * seam; normal RxEvent callbacks never restart the DMA. */
void JetsonProtocol_StartUartRxInterrupts(void);
/* Main-context only: atomically consumes a pending RX-damage safety request
 * and calls the established Drive + Lift stop paths outside the RX critical
 * section. */
void JetsonProtocol_DispatchPendingRxSafetyStop(void);
void JetsonProtocol_ServiceRx(void);
void JetsonProtocol_PrintPendingWarnings(void);
void JetsonProtocol_PollLine(void);
void JetsonProtocol_Execute(const char *line);

/* Optional secondary-UART (USART2) RX/error hooks. The single global HAL UART
 * callbacks live here; a module owning USART2 (e.g. battery) registers these to
 * receive its byte/error events. USART3/Jetson handling is unaffected. The
 * registered RX hook is responsible for re-arming its own USART2 reception. */
void JetsonProtocol_SetUart2RxByteCallback(void (*fn)(void));
void JetsonProtocol_SetUart2ErrorCallback(void (*fn)(void));

#ifdef AMR_HOST_TEST
/* Host-test hooks so the production RX ring/poller/dispatcher can be
 * exercised end to end (never a reference reimplementation). Not compiled
 * into firmware. */
void JetsonProtocol_TestInjectRxByte(uint8_t byte);
void JetsonProtocol_TestInjectRxError(void);
void JetsonProtocol_TestScheduleRxErrorAfterPops(uint32_t pop_count);
void JetsonProtocol_TestSetRxCompletionSequence(uint32_t sequence);
uint8_t *JetsonProtocol_TestGetRxDmaBuffer(void);
uint32_t JetsonProtocol_TestGetRxDmaBufferSize(void);
uint32_t JetsonProtocol_TestGetRxDmaEpoch(void);
void JetsonProtocol_TestWriteRxDma(uint32_t absolute_index,
                                  const uint8_t *data,
                                  uint32_t length);
void JetsonProtocol_TestInjectRxDmaEvent(uint16_t size,
                                        uint32_t event_type,
                                        uint32_t callback_epoch);
void JetsonProtocol_TestSetRxDmaAccounting(uint32_t ht_epoch,
                                           uint32_t tc_epoch,
                                           uint32_t produced_total,
                                           uint32_t consumed_total);
void JetsonProtocol_TestScheduleRxDmaHardwareTotalAfterCopy(
    uint32_t hardware_total);
void JetsonProtocol_TestResetRxState(void);
void JetsonProtocol_TestGetRxStats(uint32_t *ring_overflow,
                                   uint32_t *line_overflow,
                                   uint32_t *uart_error,
                                   uint32_t *invalid_char,
                                   uint32_t *assembly_timeout,
                                   uint32_t *desync,
                                   uint32_t *synced);

typedef struct
{
    uint32_t ring_overflow;
    uint32_t line_overflow;
    uint32_t uart_error;
    uint32_t invalid_char;
    uint32_t assembly_timeout;
    uint32_t stale_line;
    uint32_t damage_generation;
    uint32_t desync;
    uint32_t wire_discard;
    uint32_t rx_armed;
    uint32_t rearm_pending;
    uint32_t arm_attempt_sequence;
    uint32_t completion_sequence;
    uint32_t link_state;
    uint32_t outage_age;
    uint32_t rearm_failures;
    uint32_t rearm_recoveries;
    uint32_t rearm_fault_events;
    uint32_t immediate_stop_pending;
    uint32_t immediate_stop_events;
    uint32_t immediate_stop_dispatches;
    uint32_t rearm_consecutive_failures;
    uint32_t last_rearm_hal_status;
    uint32_t byte_budget_hits;
    uint32_t command_budget_hits;
    uint32_t queued_entries;
    uint32_t dma_session_epoch;
    uint32_t dma_produced_total;
    uint32_t dma_consumed_total;
    uint32_t dma_backlog;
    uint32_t dma_backlog_high_water;
    uint32_t dma_idle_events;
    uint32_t dma_ht_events;
    uint32_t dma_tc_events;
    uint32_t dma_duplicate_events;
    uint32_t dma_overwrites;
    uint32_t dma_stale_callbacks;
    uint32_t dma_last_event_position;
    uint32_t last_uart_error;
} JetsonProtocolTestRxStats_t;

void JetsonProtocol_TestGetRxDetailedStats(
    JetsonProtocolTestRxStats_t *stats);
#endif

#ifdef __cplusplus
}
#endif

#endif /* JETSON_PROTOCOL_H */
