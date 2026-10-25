/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : diagnostics.h
  * @brief          : Logging and diagnostic helper interfaces.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "jetson_tx.h"
#include <stdbool.h>
#include <stdint.h>

#define UART_PRINTF_BUFFER_SIZE 160U
#define UART_PRINTF_TX_TIMEOUT_MS 10U

typedef struct
{
    uint32_t lec;
    uint32_t bus_off;
    uint32_t tec;
    uint32_t rec;
} CAN1_DiagStatus_t;

void uart_printf(const char *fmt, ...);
void uart_printf_priority(JetsonTxPriority_t priority, const char *fmt, ...);
const char *FDCAN_HandleName(FDCAN_HandleTypeDef *hfdcan);
const char *hal_status_name(HAL_StatusTypeDef status);
const char *fdcan_lec_name(uint32_t lec);
void CAN_Print_Rx_Frame(FDCAN_HandleTypeDef *hfdcan,
                        const FDCAN_RxHeaderTypeDef *rx,
                        const uint8_t *data,
                        uint8_t len);
bool CAN1_Print_Status(const char *tag, CAN1_DiagStatus_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* DIAGNOSTICS_H */
