/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : diagnostics.c
  * @brief          : Logging and diagnostic helper implementations.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "app_config.h"
#include "diagnostics.h"
#include "jetson_tx.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;

void uart_printf(const char *fmt, ...)
{
#if DEBUG_TO_JETSON
    va_list args;

    va_start(args, fmt);
    (void)JetsonTx_VPrintf(JETSON_TX_PRIORITY_DEBUG, fmt, args);
    va_end(args);
#else
    (void)fmt;
#endif
}

void uart_printf_priority(JetsonTxPriority_t priority, const char *fmt, ...)
{
#if DEBUG_TO_JETSON
    va_list args;

    va_start(args, fmt);
    (void)JetsonTx_VPrintf(priority, fmt, args);
    va_end(args);
#else
    (void)priority;
    (void)fmt;
#endif
}

const char *FDCAN_HandleName(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == &hfdcan1) {
        return "FDCAN1";
    }

    return "FDCAN?";
}

const char *hal_status_name(HAL_StatusTypeDef status)
{
    switch (status) {
        case HAL_OK: return "HAL_OK";
        case HAL_ERROR: return "HAL_ERROR";
        case HAL_BUSY: return "HAL_BUSY";
        case HAL_TIMEOUT: return "HAL_TIMEOUT";
        default: return "HAL_UNKNOWN";
    }
}

const char *fdcan_lec_name(uint32_t lec)
{
    switch (lec) {
        case FDCAN_PROTOCOL_ERROR_NONE: return "NONE";
        case FDCAN_PROTOCOL_ERROR_STUFF: return "STUFF";
        case FDCAN_PROTOCOL_ERROR_FORM: return "FORM";
        case FDCAN_PROTOCOL_ERROR_ACK: return "ACK";
        case FDCAN_PROTOCOL_ERROR_BIT1: return "BIT1";
        case FDCAN_PROTOCOL_ERROR_BIT0: return "BIT0";
        case FDCAN_PROTOCOL_ERROR_CRC: return "CRC";
        case FDCAN_PROTOCOL_ERROR_NO_CHANGE: return "NO_CHANGE";
        default: return "?";
    }
}

void CAN_Print_Rx_Frame(FDCAN_HandleTypeDef *hfdcan,
                        const FDCAN_RxHeaderTypeDef *rx,
                        const uint8_t *data,
                        uint8_t len)
{
    char line[JETSON_TX_FRAME_CAPACITY];
    int written = snprintf(line, sizeof(line),
                           "%s RX ID=0x%03lX DLC=%u DATA=",
                           FDCAN_HandleName(hfdcan),
                           (unsigned long)rx->Identifier,
                           len);
    size_t used;

    if ((written <= 0) || (written >= (int)sizeof(line))) {
        (void)JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                              "UART_FORMAT_OVERFLOW\r\n");
        return;
    }
    used = (size_t)written;
    for (uint8_t i = 0U; i < len; i++) {
        written = snprintf(&line[used], sizeof(line) - used,
                           "%02X ", data[i]);
        if ((written <= 0) || ((size_t)written >= (sizeof(line) - used))) {
            (void)JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                                  "UART_FORMAT_OVERFLOW\r\n");
            return;
        }
        used += (size_t)written;
    }
    if ((used + 2U) > sizeof(line)) {
        (void)JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                              "UART_FORMAT_OVERFLOW\r\n");
        return;
    }
    memcpy(&line[used], "\r\n", 2U);
    used += 2U;
    (void)JetsonTx_Send(JETSON_TX_PRIORITY_DEBUG, line, (uint16_t)used);
}

bool CAN1_Print_Status(const char *tag, CAN1_DiagStatus_t *snapshot)
{
    FDCAN_ProtocolStatusTypeDef ps = {0};
    FDCAN_ErrorCountersTypeDef ec = {0};

    HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps);
    HAL_FDCAN_GetErrorCounters(&hfdcan1, &ec);

    if (snapshot != NULL) {
        snapshot->lec = ps.LastErrorCode;
        snapshot->bus_off = ps.BusOff;
        snapshot->tec = ec.TxErrorCnt;
        snapshot->rec = ec.RxErrorCnt;
    }

    /*
     * LEC: Last Error Code; ACK means no other node acknowledged the frame.
     * TEC/REC: transmit/receive error counters used by CAN fault confinement.
     * BUS_OFF: controller stopped after too many TX errors; reset/reinit needed.
     */
    uart_printf("CAN STATUS %s: LEC=%lu(%s) ACT=%lu BUS_OFF=%lu WARN=%lu PASSIVE=%lu\r\n",
                tag,
                (unsigned long)ps.LastErrorCode,
                fdcan_lec_name(ps.LastErrorCode),
                (unsigned long)ps.Activity,
                (unsigned long)ps.BusOff,
                (unsigned long)ps.Warning,
                (unsigned long)ps.ErrorPassive);

    uart_printf("CAN ERR %s: TEC=%lu REC=%lu ELOG=%lu TxFree=%lu\r\n",
                tag,
                (unsigned long)ec.TxErrorCnt,
                (unsigned long)ec.RxErrorCnt,
                (unsigned long)ec.ErrorLogging,
                (unsigned long)HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1));

    if (ps.BusOff != 0U) {
        uart_printf("BUS_OFF detected, reset required\r\n");
        return true;
    }

    return false;
}
