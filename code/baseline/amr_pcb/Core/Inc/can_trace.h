/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : can_trace.h
  * @brief          : Bounded FDCAN1/SDO/runtime flight recorder interfaces.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef CAN_TRACE_H
#define CAN_TRACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

#define CAN_TRACE_BUFFER_CAPACITY       192U
#define CAN_TRACE_PRE_TRIGGER_CAPACITY  128U
#define CAN_TRACE_POST_CRITICAL_RESERVE  16U
#define CAN_TRACE_POST_TRIGGER_MS       500U
#define CAN_TRACE_DUMP_LINES_PER_SERVICE  1U
#define CAN_TRACE_TPDO_AGGREGATE_MS      100U

typedef enum
{
    CAN_TRACE_STATE_ROLLING = 0,
    CAN_TRACE_STATE_POST_TRIGGER,
    CAN_TRACE_STATE_FROZEN
} CanTraceState_t;

typedef enum
{
    CAN_TRACE_TX_WHEEL_SDO = 0,
    CAN_TRACE_TX_LIFT_SDO,
    CAN_TRACE_TX_MASTER_HEARTBEAT,
    CAN_TRACE_TX_NMT,
    CAN_TRACE_TX_EMERGENCY_ZERO,
    CAN_TRACE_TX_NODE_GUARD,
    CAN_TRACE_TX_OTHER
} CanTraceTxOwner_t;

typedef enum
{
    CAN_TRACE_EVENT_TX = 0,
    CAN_TRACE_EVENT_RX,
    CAN_TRACE_EVENT_RX_ERROR,
    CAN_TRACE_EVENT_SDO_START,
    CAN_TRACE_EVENT_SDO_ENQUEUE_OK,
    CAN_TRACE_EVENT_SDO_ENQUEUE_FAILED,
    CAN_TRACE_EVENT_SDO_RESPONSE,
    CAN_TRACE_EVENT_SDO_REMOTE_ABORT,
    CAN_TRACE_EVENT_SDO_MALFORMED_RESPONSE,
    CAN_TRACE_EVENT_SDO_WRONG_RESPONSE,
    CAN_TRACE_EVENT_SDO_TIMEOUT,
    CAN_TRACE_EVENT_SDO_COMPLETE,
    CAN_TRACE_EVENT_HEARTBEAT_STALE,
    CAN_TRACE_EVENT_MASTER_TX_FAILURE,
    CAN_TRACE_EVENT_FDCAN_ERROR_PASSIVE,
    CAN_TRACE_EVENT_FDCAN_BUS_OFF,
    CAN_TRACE_EVENT_COMM_TRANSITION_FAILURE,
    CAN_TRACE_EVENT_RUNTIME_OVERRUN
} CanTraceEventType_t;

typedef enum
{
    CAN_TRACE_TRIGGER_NONE = 0,
    CAN_TRACE_TRIGGER_SDO_TIMEOUT,
    CAN_TRACE_TRIGGER_MASTER_TX,
    CAN_TRACE_TRIGGER_HEARTBEAT_STALE,
    CAN_TRACE_TRIGGER_TX_BUSY,
    CAN_TRACE_TRIGGER_TX_ERROR,
    CAN_TRACE_TRIGGER_ERROR_PASSIVE,
    CAN_TRACE_TRIGGER_BUS_OFF,
    CAN_TRACE_TRIGGER_SDO_PROTOCOL,
    CAN_TRACE_TRIGGER_COMM_TRANSITION
} CanTraceTriggerReason_t;

typedef enum
{
    CAN_TRACE_SDO_INTERNAL_NONE = 0,
    CAN_TRACE_SDO_INTERNAL_TX_ENQUEUE_FAILED,
    CAN_TRACE_SDO_INTERNAL_NO_RESPONSE_TIMEOUT,
    CAN_TRACE_SDO_INTERNAL_REMOTE_ABORT,
    CAN_TRACE_SDO_INTERNAL_MALFORMED_RESPONSE,
    CAN_TRACE_SDO_INTERNAL_WRONG_NODE_OR_OBJECT,
    CAN_TRACE_SDO_INTERNAL_STATE_ERROR
} CanTraceSdoReason_t;

typedef enum
{
    CAN_TRACE_RX_DETAIL_NONE = 0,
    CAN_TRACE_RX_DETAIL_TPDO_AGGREGATE,
    CAN_TRACE_RX_DETAIL_INVALID_DLC,
    CAN_TRACE_RX_DETAIL_TPDO_GAP,
    CAN_TRACE_RX_DETAIL_TPDO_STALE,
    CAN_TRACE_RX_DETAIL_TPDO_RECOVERY
} CanTraceRxDetail_t;

typedef enum
{
    CAN_TRACE_RUNTIME_MAIN_LOOP = 0,
    CAN_TRACE_RUNTIME_CANOPEN_RX,
    CAN_TRACE_RUNTIME_HEARTBEAT,
    CAN_TRACE_RUNTIME_DRIVE_UPDATE,
    CAN_TRACE_RUNTIME_LIFT_UPDATE,
    CAN_TRACE_RUNTIME_BLOCKING_SDO,
    CAN_TRACE_RUNTIME_COUNT
} CanTraceRuntimeChannel_t;

typedef struct
{
    uint32_t timestamp_ms;
    uint32_t can_id;
    uint32_t hal_error;
    uint32_t elapsed_ms;
    uint32_t code;
    uint16_t index;
    uint8_t subindex;
    uint8_t event;
    uint8_t owner;
    uint8_t node_id;
    uint8_t dlc;
    uint8_t hal_status;
    uint8_t tx_free_before;
    uint8_t tx_free_after;
    uint8_t lec;
    uint8_t tec;
    uint8_t rec;
    uint8_t fdcan_flags;
    uint8_t drive_state;
    uint8_t lift_state;
    uint8_t sdo_write;
    uint8_t detail;
    uint8_t fdcan_valid;
} CanTraceEvent_t;

typedef struct
{
    uint32_t current_ms;
    uint32_t maximum_ms;
    uint32_t current_gap_ms;
    uint32_t maximum_gap_ms;
    uint32_t over_5ms_count;
    uint32_t over_20ms_count;
    uint32_t over_100ms_count;
    uint32_t over_350ms_count;
} CanTraceRuntimeStats_t;

typedef struct
{
    CanTraceState_t state;
    CanTraceTriggerReason_t trigger_reason;
    uint32_t snapshot_id;
    uint32_t trigger_ms;
    uint32_t freeze_ms;
    uint32_t overwritten_or_dropped;
    uint32_t dropped_normal;
    uint32_t dropped_critical;
    uint16_t entry_count;
    bool dump_active;
    bool automatic_dump_completed;
} CanTraceStatus_t;

void CanTrace_Init(FDCAN_HandleTypeDef *hfdcan, uint32_t now_ms);
void CanTrace_SetSubsystemStates(uint32_t drive_state, uint32_t lift_state);
HAL_StatusTypeDef CanTrace_SubmitTx(
    FDCAN_HandleTypeDef *hfdcan,
    const FDCAN_TxHeaderTypeDef *header,
    const uint8_t *data,
    CanTraceTxOwner_t owner,
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    bool sdo_write);
void CanTrace_RecordRx(uint32_t now_ms,
                       uint32_t can_id,
                       uint8_t dlc,
                       uint8_t node_id);
void CanTrace_RecordRxError(uint32_t now_ms,
                            HAL_StatusTypeDef hal_status);
void CanTrace_RecordTpdoTransition(uint32_t now_ms,
                                   uint32_t can_id,
                                   uint8_t node_id,
                                   CanTraceRxDetail_t detail,
                                   uint32_t gap_ms);
void CanTrace_RecordSdo(CanTraceEventType_t event,
                        CanTraceTxOwner_t owner,
                        uint8_t node_id,
                        uint16_t index,
                        uint8_t subindex,
                        bool write,
                        uint32_t start_ms,
                        uint32_t now_ms,
                        HAL_StatusTypeDef enqueue_status,
                        CanTraceSdoReason_t reason,
                        uint32_t code);
void CanTrace_RecordHeartbeatFault(uint8_t node_id,
                                   bool master_tx_failure,
                                   uint32_t now_ms,
                                   uint32_t detail);
void CanTrace_RecordCommunicationFailure(uint8_t node_id,
                                         uint32_t now_ms,
                                         uint32_t detail);
void CanTrace_RuntimeBegin(CanTraceRuntimeChannel_t channel,
                           uint32_t now_ms);
void CanTrace_RuntimeEnd(CanTraceRuntimeChannel_t channel,
                         uint32_t now_ms);
void CanTrace_RuntimeObserve(CanTraceRuntimeChannel_t channel,
                             uint32_t start_ms,
                             uint32_t end_ms);
void CanTrace_Service(uint32_t now_ms);
bool CanTrace_RequestStatus(uint32_t request_nonce);
bool CanTrace_RequestDump(uint32_t request_nonce);
CanTraceStatus_t CanTrace_GetStatus(void);
CanTraceRuntimeStats_t CanTrace_GetRuntimeStats(
    CanTraceRuntimeChannel_t channel);
bool CanTrace_GetEvent(uint16_t chronological_index,
                       CanTraceEvent_t *event);
const char *CanTrace_EventName(CanTraceEventType_t event);
const char *CanTrace_OwnerName(CanTraceTxOwner_t owner);
const char *CanTrace_TriggerName(CanTraceTriggerReason_t reason);

#ifdef AMR_HOST_TEST
void CanTrace_TestReset(uint32_t now_ms);
#endif

#ifdef __cplusplus
}
#endif

#endif /* CAN_TRACE_H */
