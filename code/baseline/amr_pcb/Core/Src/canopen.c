/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : canopen.c
  * @brief          : CANopen dispatcher, runtime transport, and SDO helpers.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "app_config.h"
#include "can_trace.h"
#include "canopen.h"
#include "wheel_pdo_config.h"
#include "wheel_rpdo_tx.h"
#include "diagnostics.h"

#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;

typedef enum
{
    WHEEL_READ_NONE = 0,
    WHEEL_READ_VELOCITY,
    WHEEL_READ_POSITION
} WheelReadObject_t;

typedef struct
{
    bool pending;
    bool velocity_requested;
    bool position_requested;
    bool fault_monitor_requested;
    WheelReadObject_t object_kind;
    uint32_t sent_ms;
    uint32_t last_velocity_request_ms;
    uint32_t last_position_request_ms;
} WheelFeedbackRequest_t;

typedef struct
{
    bool pending;
    uint32_t sent_ms;
} WheelTargetRequest_t;

typedef struct
{
    bool configured;
    bool qualification_active;
    bool qualified;
    bool fault_latched;
    uint32_t baseline_sequence;
    uint8_t fresh_valid_count;
    uint32_t last_heartbeat_ms;
    uint8_t last_nmt_state;
} CanopenHeartbeatState_t;

typedef struct
{
    bool started;
    bool fault_latched;
    uint32_t next_tx_ms;
    uint32_t consecutive_failures;
} CanopenMasterHeartbeatState_t;

typedef struct
{
    CanopenSdoTransactionState_t state;
    CanopenSdoOperation_t operation;
    uint8_t node_id;
    uint16_t index;
    uint8_t subindex;
    uint8_t expected_upload_size;
    uint32_t start_ms;
    uint32_t timeout_ms;
    uint32_t quarantine_until_ms;
    bool completion_success;
    CanopenSdoOwner_t owner;
    CanopenAsyncResult_t async_result;
    uint32_t completion_value;
    uint32_t completion_rx_ms;
    uint32_t abort_code;
    bool result_reported;
} CanopenSdoTransaction_t;

SDO_Response_t s_last_sdo_response;

static bool s_sdo_quiet = false;
static bool s_rpdo_zero_wait;
static WheelFeedbackRequest_t s_feedback_request[2];
static WheelTargetRequest_t s_target_request[2];
static CanopenValueSample_t s_wheel_velocity[2];
static CanopenValueSample_t s_wheel_velocity_sdo[2];
static CanopenValueSample_t s_wheel_position[2];
static CanopenWheelTpdoSample_t s_wheel_tpdo_shadow[2];
static CanopenWheelTpdoPair_t s_wheel_tpdo_pair;
static uint32_t s_wheel_tpdo_committed_node_sequence[2];
static uint32_t s_wheel_tpdo_arm_baseline_sequence[2];
static uint32_t s_wheel_tpdo_arm_start_ms;
static bool s_wheel_velocity_from_tpdo[2];
static bool s_wheel_tpdo_configured[2];
static CanopenWheelTpdoConfigDiagnostic_t s_wheel_tpdo_config_diagnostic;
static CanopenRuntimeDiagnostics_t s_runtime_diag;
static bool s_runtime_fault_pending;
static bool s_feedback_continuity_fault_pending;
static CanopenHeartbeatState_t s_heartbeat[4];
static CanopenMasterHeartbeatState_t s_master_heartbeat;
static bool s_heartbeat_fault_pending;
static uint8_t s_heartbeat_fault_node;
static CanopenHeartbeatFaultReason_t s_heartbeat_fault_reason;
static bool s_lift_fault_emcy_pending;
static uint16_t s_lift_fault_emcy_code;
static CanopenSdoTransaction_t s_sdo_transaction[4];
static CanopenEmergencyZeroStatus_t s_emergency_zero_status =
    CANOPEN_EMERGENCY_ZERO_FAILED;
static uint8_t s_emergency_zero_ack_mask;
static uint8_t s_emergency_zero_confirmation_needed_mask;
static CanopenWheelFeedbackTransport_t s_wheel_feedback_transport =
    CANOPEN_WHEEL_FEEDBACK_TPDO_SHADOW;
static CanopenWheelFeedbackAuthority_t s_wheel_feedback_authority =
    CANOPEN_WHEEL_AUTHORITY_SDO;
static const char *s_last_wheel_tpdo_active_fault = "NONE";
static uint32_t s_last_fault_monitor_defer_ms;
static bool s_fault_monitor_defer_recorded;
static bool s_boot_sdo_observing;
static bool s_boot_sdo_fenced;
static CanopenBootSdoEvidence_t s_boot_sdo_evidence;

static HAL_StatusTypeDef SendRuntimeTarget(uint8_t node_id, int32_t value);
static HAL_StatusTypeDef Canopen_SendMasterHeartbeat(void);
static void ServiceWheelTpdoFreshness(uint32_t now_ms);

static void IncrementSaturated(uint32_t *counter)
{
    if (*counter != UINT32_MAX) {
        (*counter)++;
    }
}

static int8_t WheelSlotFromNode(uint8_t node_id)
{
    if (node_id == LEFT_NODE_ID) {
        return 0;
    }
    if (node_id == RIGHT_NODE_ID) {
        return 1;
    }
    return -1;
}

static bool BootSdoNodeIsFenced(uint8_t node_id)
{
    return s_boot_sdo_fenced && (WheelSlotFromNode(node_id) >= 0);
}

static void BootSdoLatchFault(CanopenBootSdoFaultReason_t reason,
                              uint8_t node_id,
                              uint16_t statusword,
                              uint32_t abort_code)
{
    if (s_boot_sdo_evidence.hard_fault_reason != CANOPEN_BOOT_SDO_FAULT_NONE) {
        return;
    }
    s_boot_sdo_evidence.hard_fault_reason = reason;
    s_boot_sdo_evidence.node_id = node_id;
    s_boot_sdo_evidence.statusword = statusword;
    s_boot_sdo_evidence.abort_code = abort_code;
}

static void BootSdoObserveResponse(uint8_t node_id,
                                    const uint8_t *data,
                                    uint8_t len)
{
    uint16_t index;

    if (!s_boot_sdo_observing || (WheelSlotFromNode(node_id) < 0)) {
        return;
    }
    /* Hard evidence must survive cancellation, quarantine and an old object
     * response. A success outside its request is still never accepted. */
    if (len != 8U) {
        BootSdoLatchFault(CANOPEN_BOOT_SDO_FAULT_PROTOCOL, node_id, 0U, 0U);
        return;
    }
    index = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
    if (data[0] == 0x80U) {
        uint32_t abort_code = 0U;

        if (len == 8U) {
            abort_code = (uint32_t)data[4] | ((uint32_t)data[5] << 8) |
                ((uint32_t)data[6] << 16) | ((uint32_t)data[7] << 24);
        }
        BootSdoLatchFault((len == 8U) ? CANOPEN_BOOT_SDO_FAULT_ABORT :
                          CANOPEN_BOOT_SDO_FAULT_PROTOCOL,
                          node_id, 0U, abort_code);
    } else if ((index == 0x6040U) && (data[3] == 0U)) {
        if ((data[0] != 0x60U) || (len != 8U)) {
            BootSdoLatchFault(CANOPEN_BOOT_SDO_FAULT_PROTOCOL,
                              node_id, 0U, 0U);
        }
    } else if ((index == 0x6041U) && (data[3] == 0U)) {
        /* 0x6041 is UNSIGNED16: accepting an 8-bit upload can fabricate a
         * plausible statusword. Boot confirmation requires an exact upload. */
        if ((data[0] != 0x4BU) || (len != 8U)) {
            BootSdoLatchFault(CANOPEN_BOOT_SDO_FAULT_PROTOCOL,
                              node_id, 0U, 0U);
        } else {
            uint16_t statusword = (uint16_t)data[4] |
                ((uint16_t)data[5] << 8);

            if ((statusword & (1U << 3)) != 0U) {
                BootSdoLatchFault(CANOPEN_BOOT_SDO_FAULT_STATUS,
                                  node_id, statusword, 0U);
            }
        }
    }
}

static CanopenSdoTransaction_t *SdoTransactionFromNode(uint8_t node_id)
{
    if ((node_id == 0U) || (node_id >= 4U)) {
        return NULL;
    }
    return &s_sdo_transaction[node_id];
}

static bool CanopenTraceSdoIsWrite(CanopenSdoOperation_t operation)
{
    return (operation == CANOPEN_SDO_OP_BLOCKING_WRITE) ||
           (operation == CANOPEN_SDO_OP_ASYNC_WRITE) ||
           (operation == CANOPEN_SDO_OP_RUNTIME_TARGET) ||
           (operation == CANOPEN_SDO_OP_EMERGENCY_ZERO);
}

static CanTraceTxOwner_t CanopenTraceSdoOwner(
    const CanopenSdoTransaction_t *transaction,
    uint8_t node_id)
{
    if ((transaction != NULL) &&
        ((transaction->operation == CANOPEN_SDO_OP_EMERGENCY_ZERO) ||
         (transaction->owner == CANOPEN_SDO_OWNER_EMERGENCY_ZERO))) {
        return CAN_TRACE_TX_EMERGENCY_ZERO;
    }
    return (node_id == LIFT_NODE_ID) ?
        CAN_TRACE_TX_LIFT_SDO : CAN_TRACE_TX_WHEEL_SDO;
}

static void CanopenTraceSdoComplete(
    const CanopenSdoTransaction_t *transaction,
    uint32_t now_ms,
    CanTraceSdoReason_t reason,
    uint32_t code)
{
    if (transaction == NULL) {
        return;
    }
    CanTrace_RecordSdo(
        CAN_TRACE_EVENT_SDO_COMPLETE,
        CanopenTraceSdoOwner(transaction, transaction->node_id),
        transaction->node_id, transaction->index, transaction->subindex,
        CanopenTraceSdoIsWrite(transaction->operation),
        transaction->start_ms, now_ms, HAL_OK, reason, code);
}

static void SdoTransactionSetIdle(CanopenSdoTransaction_t *transaction)
{
    if (transaction == NULL) {
        return;
    }
    transaction->state = CANOPEN_SDO_TX_IDLE;
    transaction->operation = CANOPEN_SDO_OP_NONE;
    transaction->expected_upload_size = 0U;
    transaction->completion_success = false;
    transaction->owner = CANOPEN_SDO_OWNER_NONE;
    transaction->async_result = CANOPEN_ASYNC_RESULT_IDLE;
    transaction->completion_value = 0U;
    transaction->completion_rx_ms = 0U;
    transaction->abort_code = 0U;
    transaction->result_reported = false;
}

static void SdoTransactionRefreshQuarantine(
    CanopenSdoTransaction_t *transaction,
    uint32_t now_ms)
{
    if ((transaction != NULL) &&
        (transaction->state == CANOPEN_SDO_TX_QUARANTINE) &&
        ((int32_t)(now_ms - transaction->quarantine_until_ms) >= 0)) {
        if (((transaction->operation == CANOPEN_SDO_OP_ASYNC_READ) ||
             (transaction->operation == CANOPEN_SDO_OP_ASYNC_WRITE)) &&
            (transaction->async_result == CANOPEN_ASYNC_RESULT_TIMEOUT) &&
            !transaction->result_reported) {
            return;
        }
        SdoTransactionSetIdle(transaction);
    }
}

static bool SdoTransactionCanStart(uint8_t node_id, uint32_t now_ms)
{
    CanopenSdoTransaction_t *transaction = SdoTransactionFromNode(node_id);

    if (BootSdoNodeIsFenced(node_id)) {
        return false;
    }
    SdoTransactionRefreshQuarantine(transaction, now_ms);
    return (transaction != NULL) &&
           (transaction->state == CANOPEN_SDO_TX_IDLE);
}

static bool SdoTransactionStart(uint8_t node_id,
                                uint16_t index,
                                uint8_t subindex,
                                CanopenSdoOperation_t operation,
                                CanopenSdoOwner_t owner,
                                uint32_t now_ms,
                                uint32_t timeout_ms,
                                uint8_t expected_upload_size)
{
    CanopenSdoTransaction_t *transaction = SdoTransactionFromNode(node_id);

    if (!SdoTransactionCanStart(node_id, now_ms)) {
        return false;
    }
    transaction->state = CANOPEN_SDO_TX_ACTIVE;
    transaction->operation = operation;
    transaction->owner = owner;
    transaction->node_id = node_id;
    transaction->index = index;
    transaction->subindex = subindex;
    transaction->expected_upload_size = expected_upload_size;
    transaction->start_ms = now_ms;
    transaction->timeout_ms = timeout_ms;
    transaction->completion_success = false;
    transaction->async_result = CANOPEN_ASYNC_RESULT_PENDING;
    transaction->completion_value = 0U;
    transaction->completion_rx_ms = 0U;
    transaction->abort_code = 0U;
    transaction->result_reported = false;
    CanTrace_RecordSdo(
        CAN_TRACE_EVENT_SDO_START,
        CanopenTraceSdoOwner(transaction, node_id),
        node_id, index, subindex,
        CanopenTraceSdoIsWrite(operation),
        now_ms, now_ms, HAL_OK,
        CAN_TRACE_SDO_INTERNAL_NONE, 0U);
    return true;
}

static void SdoTransactionEnterQuarantine(
    CanopenSdoTransaction_t *transaction,
    uint32_t now_ms)
{
    if (transaction == NULL) {
        return;
    }
    transaction->state = CANOPEN_SDO_TX_QUARANTINE;
    transaction->quarantine_until_ms = now_ms + DRIVE_SDO_QUARANTINE_MS;
    transaction->completion_success = false;
    IncrementSaturated(&s_runtime_diag.sdo_quarantine_count);
}

static uint8_t fdcan_dlc_to_len(uint32_t dlc)
{
    switch (dlc) {
        case FDCAN_DLC_BYTES_0: return 0U;
        case FDCAN_DLC_BYTES_1: return 1U;
        case FDCAN_DLC_BYTES_2: return 2U;
        case FDCAN_DLC_BYTES_3: return 3U;
        case FDCAN_DLC_BYTES_4: return 4U;
        case FDCAN_DLC_BYTES_5: return 5U;
        case FDCAN_DLC_BYTES_6: return 6U;
        case FDCAN_DLC_BYTES_7: return 7U;
        case FDCAN_DLC_BYTES_8: return 8U;
        default: return 0U; /* Unsupported DLC must never resemble Classic SDO. */
    }
}

static int32_t DecodeI32LittleEndian(const uint8_t *data)
{
    uint32_t raw = (uint32_t)data[0] |
                   ((uint32_t)data[1] << 8) |
                   ((uint32_t)data[2] << 16) |
                   ((uint32_t)data[3] << 24);
    return (int32_t)raw;
}

static HAL_StatusTypeDef Canopen_SendSdoFrame(const CanopenNode_t *node,
                                              const uint8_t data[8],
                                              const char *label,
                                              bool reset_mailbox,
                                              bool quiet)
{
    FDCAN_TxHeaderTypeDef tx = {0};
    HAL_StatusTypeDef status;
    CanopenSdoTransaction_t *transaction;
    CanTraceTxOwner_t trace_owner;
    uint16_t trace_index;
    uint8_t trace_subindex;
    bool trace_write;
    uint32_t trace_start_ms;

    if ((node == NULL) || (node->hfdcan == NULL)) {
        return HAL_ERROR;
    }
    if (BootSdoNodeIsFenced(node->node_id)) {
        return HAL_BUSY;
    }

    if (reset_mailbox) {
        s_last_sdo_response.valid = false;
    }

    tx.Identifier = 0x600U + node->node_id;
    tx.IdType = FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME;
    tx.DataLength = FDCAN_DLC_BYTES_8;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch = FDCAN_BRS_OFF;
    tx.FDFormat = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker = 0U;

    transaction = SdoTransactionFromNode(node->node_id);
    trace_owner = CanopenTraceSdoOwner(transaction, node->node_id);
    trace_index = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
    trace_subindex = data[3];
    trace_write = data[0] != 0x40U;
    trace_start_ms = (transaction != NULL) ?
        transaction->start_ms : HAL_GetTick();
    status = CanTrace_SubmitTx(node->hfdcan, &tx, data,
                               trace_owner, node->node_id,
                               trace_index, trace_subindex,
                               trace_write);
    CanTrace_RecordSdo(
        (status == HAL_OK) ? CAN_TRACE_EVENT_SDO_ENQUEUE_OK :
                             CAN_TRACE_EVENT_SDO_ENQUEUE_FAILED,
        trace_owner, node->node_id, trace_index, trace_subindex,
        trace_write, trace_start_ms, HAL_GetTick(), status,
        (status == HAL_OK) ? CAN_TRACE_SDO_INTERNAL_NONE :
                             CAN_TRACE_SDO_INTERNAL_TX_ENQUEUE_FAILED,
        0U);

    if (DEBUG_SDO && !s_sdo_quiet && !quiet) {
        uart_printf("%s TX SDO %s node=%u ID=0x%03lX DATA=%02X %02X %02X %02X %02X %02X %02X %02X: %s(%d)\r\n",
                    FDCAN_HandleName(node->hfdcan),
                    label,
                    node->node_id,
                    (unsigned long)tx.Identifier,
                    data[0], data[1], data[2], data[3],
                    data[4], data[5], data[6], data[7],
                    hal_status_name(status),
                    status);
    }

    if (status != HAL_OK) {
        FDCAN_ProtocolStatusTypeDef protocol = {0};
        FDCAN_ErrorCountersTypeDef counters = {0};

        HAL_FDCAN_GetProtocolStatus(node->hfdcan, &protocol);
        HAL_FDCAN_GetErrorCounters(node->hfdcan, &counters);
        uart_printf("%s TX FAILED HAL=0x%08lX free=%lu LEC=%lu BUS_OFF=%lu TEC=%lu REC=%lu\r\n",
                    FDCAN_HandleName(node->hfdcan),
                    (unsigned long)HAL_FDCAN_GetError(node->hfdcan),
                    (unsigned long)HAL_FDCAN_GetTxFifoFreeLevel(node->hfdcan),
                    (unsigned long)protocol.LastErrorCode,
                    (unsigned long)protocol.BusOff,
                    (unsigned long)counters.TxErrorCnt,
                    (unsigned long)counters.RxErrorCnt);
    }

    return status;
}

bool Canopen_StartRemoteNode(uint8_t node_id)
{
    FDCAN_TxHeaderTypeDef tx = {0};
    uint8_t payload[4] = {0x01U, node_id, 0U, 0U};

    /* This public helper is deliberately wheel-only. In particular, reject
     * CANopen's node_id=0 broadcast so configuring wheel feedback can never
     * start the Lift node as an unintended side effect. */
    if ((node_id != LEFT_NODE_ID) && (node_id != RIGHT_NODE_ID)) {
        return false;
    }

    tx.Identifier = 0x000U;
    tx.IdType = FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME;
    tx.DataLength = FDCAN_DLC_BYTES_2;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch = FDCAN_BRS_OFF;
    tx.FDFormat = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker = 0U;

    return CanTrace_SubmitTx(
        &hfdcan1, &tx, payload,
        CAN_TRACE_TX_NMT, node_id, 0U, 0U, false) == HAL_OK;
}

static HAL_StatusTypeDef Canopen_SendMasterHeartbeat(void)
{
    FDCAN_TxHeaderTypeDef tx = {0};
    uint8_t payload[4] = {
        DRIVE_MASTER_HEARTBEAT_STATE,
        0U,
        0U,
        0U
    };

    tx.Identifier = DRIVE_MASTER_HEARTBEAT_COB_ID;
    tx.IdType = FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME;
    tx.DataLength = FDCAN_DLC_BYTES_1;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch = FDCAN_BRS_OFF;
    tx.FDFormat = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker = 0U;
    return CanTrace_SubmitTx(
        &hfdcan1, &tx, payload,
        CAN_TRACE_TX_MASTER_HEARTBEAT,
        DRIVE_MASTER_HEARTBEAT_NODE_ID, 0U, 0U, false);
}

static void StoreSdoMailbox(uint8_t node_id,
                            uint16_t index,
                            uint8_t subindex,
                            const uint8_t *data,
                            uint8_t len)
{
    s_last_sdo_response.valid = (data[0] != 0x80U);
    s_last_sdo_response.node_id = node_id;
    s_last_sdo_response.command = data[0];
    s_last_sdo_response.len = len;
    s_last_sdo_response.index = index;
    s_last_sdo_response.subindex = subindex;
    s_last_sdo_response.sequence++;
    memset(s_last_sdo_response.data, 0, sizeof(s_last_sdo_response.data));
    memcpy(s_last_sdo_response.data, data, len);
}

static void MarkRuntimeTargetFailure(
    uint8_t node_id,
    CanopenTargetFailureReason_t reason)
{
    IncrementSaturated(&s_runtime_diag.wheel_target_failure_count);
    s_runtime_diag.last_wheel_target_failure_node = node_id;
    s_runtime_diag.last_wheel_target_failure_reason = (uint8_t)reason;
    s_runtime_fault_pending = true;
    CanTrace_RecordCommunicationFailure(
        node_id, HAL_GetTick(), (uint32_t)reason);
}

static void ServiceSdoTransactionTimeouts(uint32_t now_ms)
{
    uint8_t node_id;

    for (node_id = 1U; node_id < 4U; node_id++) {
        CanopenSdoTransaction_t *transaction =
            SdoTransactionFromNode(node_id);
        int8_t wheel_slot = WheelSlotFromNode(node_id);

        SdoTransactionRefreshQuarantine(transaction, now_ms);
        if ((transaction == NULL) ||
            (transaction->state != CANOPEN_SDO_TX_ACTIVE) ||
            ((uint32_t)(now_ms - transaction->start_ms) <=
             transaction->timeout_ms)) {
            continue;
        }

        IncrementSaturated(&s_runtime_diag.sdo_timeout_count);
        CanTrace_RecordSdo(
            CAN_TRACE_EVENT_SDO_TIMEOUT,
            CanopenTraceSdoOwner(transaction, node_id),
            node_id, transaction->index, transaction->subindex,
            CanopenTraceSdoIsWrite(transaction->operation),
            transaction->start_ms, now_ms, HAL_OK,
            CAN_TRACE_SDO_INTERNAL_NO_RESPONSE_TIMEOUT, 0U);
        if ((transaction->operation == CANOPEN_SDO_OP_ASYNC_READ) ||
            (transaction->operation == CANOPEN_SDO_OP_ASYNC_WRITE)) {
            transaction->async_result = CANOPEN_ASYNC_RESULT_TIMEOUT;
            transaction->result_reported = false;
        }
        if (wheel_slot >= 0) {
            uint8_t slot = (uint8_t)wheel_slot;

            if ((transaction->operation == CANOPEN_SDO_OP_RUNTIME_VELOCITY) ||
                (transaction->operation == CANOPEN_SDO_OP_RUNTIME_POSITION)) {
                s_feedback_request[slot].pending = false;
                if (transaction->operation ==
                    CANOPEN_SDO_OP_RUNTIME_VELOCITY) {
                    if (s_feedback_request[slot].fault_monitor_requested) {
                        IncrementSaturated(
                            &s_runtime_diag.fault_velocity_timeout_count);
                    }
                    s_feedback_request[slot].fault_monitor_requested = false;
                    if (s_wheel_feedback_authority ==
                        CANOPEN_WHEEL_AUTHORITY_SDO) {
                        s_wheel_velocity[slot].valid = false;
                    }
                    s_wheel_velocity_sdo[slot].valid = false;
                } else {
                    s_wheel_position[slot].valid = false;
                }
                IncrementSaturated(
                    &s_runtime_diag.wheel_feedback_failure_count);
                s_feedback_continuity_fault_pending = true;
            } else if (transaction->operation ==
                       CANOPEN_SDO_OP_RUNTIME_TARGET) {
                s_target_request[slot].pending = false;
                MarkRuntimeTargetFailure(
                    node_id, CANOPEN_TARGET_FAILURE_TIMEOUT);
            } else if (transaction->operation ==
                       CANOPEN_SDO_OP_EMERGENCY_ZERO) {
                s_emergency_zero_status = CANOPEN_EMERGENCY_ZERO_FAILED;
                MarkRuntimeTargetFailure(
                    node_id, CANOPEN_TARGET_FAILURE_TIMEOUT);
            } else if ((transaction->operation !=
                        CANOPEN_SDO_OP_ASYNC_READ) &&
                       (transaction->operation !=
                        CANOPEN_SDO_OP_ASYNC_WRITE)) {
                s_feedback_continuity_fault_pending = true;
            }
        }
        SdoTransactionEnterQuarantine(transaction, now_ms);
    }
}

static void HandleSdoResponse(uint8_t node_id,
                              const uint8_t *data,
                              uint8_t len,
                              uint32_t now_ms)
{
    uint16_t index;
    uint8_t subindex;
    int8_t wheel_slot;
    CanopenSdoTransaction_t *transaction;
    bool object_matches;
    bool upload_response;
    CanTraceTxOwner_t trace_owner;
    /* Classic expedited SDO responses occupy exactly eight data bytes,
     * including padding. Apply this to every object and client, not only
     * boot controlwords. Object/node matching remains mandatory below. */
    bool complete_frame = len == 8U;
    bool write_ack = complete_frame && (data[0] == 0x60U);
    bool abort_response = complete_frame && (data[0] == 0x80U);

    BootSdoObserveResponse(node_id, data, len);
    if (BootSdoNodeIsFenced(node_id)) {
        IncrementSaturated(&s_runtime_diag.sdo_late_response_count);
        return;
    }
    if (len < 4U) {
        IncrementSaturated(&s_runtime_diag.unhandled_frame_count);
        transaction = SdoTransactionFromNode(node_id);
        CanTrace_RecordSdo(
            CAN_TRACE_EVENT_SDO_MALFORMED_RESPONSE,
            CanopenTraceSdoOwner(transaction, node_id),
            node_id, (transaction != NULL) ? transaction->index : 0U,
            (transaction != NULL) ? transaction->subindex : 0U,
            (transaction != NULL) ?
                CanopenTraceSdoIsWrite(transaction->operation) : false,
            (transaction != NULL) ? transaction->start_ms : now_ms,
            now_ms, HAL_OK,
            CAN_TRACE_SDO_INTERNAL_MALFORMED_RESPONSE, len);
        return;
    }

    index = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
    subindex = data[3];
    wheel_slot = WheelSlotFromNode(node_id);
    transaction = SdoTransactionFromNode(node_id);
    trace_owner = CanopenTraceSdoOwner(transaction, node_id);
    IncrementSaturated(&s_runtime_diag.sdo_response_count);

    if (transaction == NULL) {
        IncrementSaturated(&s_runtime_diag.sdo_unexpected_response_count);
        CanTrace_RecordSdo(
            CAN_TRACE_EVENT_SDO_WRONG_RESPONSE, trace_owner,
            node_id, index, subindex, false,
            now_ms, now_ms, HAL_OK,
            CAN_TRACE_SDO_INTERNAL_WRONG_NODE_OR_OBJECT, data[0]);
        return;
    }
    object_matches = (transaction->index == index) &&
                     (transaction->subindex == subindex);
    if (transaction->state == CANOPEN_SDO_TX_QUARANTINE) {
        if (object_matches) {
            IncrementSaturated(&s_runtime_diag.sdo_late_response_count);
        } else {
            IncrementSaturated(&s_runtime_diag.sdo_unexpected_response_count);
        }
        CanTrace_RecordSdo(
            CAN_TRACE_EVENT_SDO_WRONG_RESPONSE, trace_owner,
            node_id, index, subindex,
            CanopenTraceSdoIsWrite(transaction->operation),
            transaction->start_ms, now_ms, HAL_OK,
            CAN_TRACE_SDO_INTERNAL_WRONG_NODE_OR_OBJECT, data[0]);
        return;
    }
    if ((transaction->state != CANOPEN_SDO_TX_ACTIVE) || !object_matches) {
        IncrementSaturated(&s_runtime_diag.sdo_unexpected_response_count);
        CanTrace_RecordSdo(
            CAN_TRACE_EVENT_SDO_WRONG_RESPONSE, trace_owner,
            node_id, index, subindex,
            CanopenTraceSdoIsWrite(transaction->operation),
            transaction->start_ms, now_ms, HAL_OK,
            CAN_TRACE_SDO_INTERNAL_WRONG_NODE_OR_OBJECT, data[0]);
        return;
    }
    /* Expedited upload padding is not object data. The typed caller fixes
     * the only acceptable response width before the request is sent. */
    upload_response = complete_frame &&
        (((transaction->expected_upload_size == 1U) && (data[0] == 0x4FU)) ||
         ((transaction->expected_upload_size == 2U) && (data[0] == 0x4BU)) ||
         ((transaction->expected_upload_size == 4U) && (data[0] == 0x43U)));
    if (s_boot_sdo_observing && (wheel_slot >= 0) &&
        (index == 0x6041U) && (subindex == 0U) && (data[0] != 0x4BU)) {
        /* Retain the boot statusword's object-specific UNSIGNED16 check. */
        upload_response = false;
        write_ack = false;
    }
    /* Keep the dispatch time with the completion even if its owner cannot
     * poll until a later main-loop iteration or blocking diagnostic returns. */
    transaction->completion_rx_ms = now_ms;
    CanTrace_RecordSdo(
        CAN_TRACE_EVENT_SDO_RESPONSE, trace_owner,
        node_id, index, subindex,
        CanopenTraceSdoIsWrite(transaction->operation),
        transaction->start_ms, now_ms, HAL_OK,
        CAN_TRACE_SDO_INTERNAL_NONE, data[0]);
    if (abort_response) {
        s_runtime_diag.last_sdo_abort_code = (uint32_t)data[4] |
            ((uint32_t)data[5] << 8) |
            ((uint32_t)data[6] << 16) |
            ((uint32_t)data[7] << 24);
        s_runtime_diag.last_sdo_abort_node = node_id;
        IncrementSaturated(&s_runtime_diag.sdo_abort_count);
        CanTrace_RecordSdo(
            CAN_TRACE_EVENT_SDO_REMOTE_ABORT, trace_owner,
            node_id, index, subindex,
            CanopenTraceSdoIsWrite(transaction->operation),
            transaction->start_ms, now_ms, HAL_OK,
            CAN_TRACE_SDO_INTERNAL_REMOTE_ABORT,
            s_runtime_diag.last_sdo_abort_code);
    } else if (data[0] == 0x80U) {
        CanTrace_RecordSdo(
            CAN_TRACE_EVENT_SDO_MALFORMED_RESPONSE, trace_owner,
            node_id, index, subindex,
            CanopenTraceSdoIsWrite(transaction->operation),
            transaction->start_ms, now_ms, HAL_OK,
            CAN_TRACE_SDO_INTERNAL_MALFORMED_RESPONSE, len);
    }

    if ((transaction->operation == CANOPEN_SDO_OP_BLOCKING_READ) ||
        (transaction->operation == CANOPEN_SDO_OP_BLOCKING_WRITE)) {
        StoreSdoMailbox(node_id, index, subindex, data, len);
        transaction->completion_success =
            complete_frame && (data[0] != 0x80U) &&
            (((transaction->operation == CANOPEN_SDO_OP_BLOCKING_WRITE) &&
              write_ack) ||
             ((transaction->operation == CANOPEN_SDO_OP_BLOCKING_READ) &&
              upload_response));
        /* Legacy diagnostic callers inspect the mailbox even when a blocking
         * read failed. Do not publish malformed replies as valid evidence. */
        s_last_sdo_response.valid = transaction->completion_success;
        transaction->state = CANOPEN_SDO_TX_COMPLETED;
        return;
    }

    if ((transaction->operation == CANOPEN_SDO_OP_ASYNC_READ) ||
        (transaction->operation == CANOPEN_SDO_OP_ASYNC_WRITE)) {
        transaction->completion_value = 0U;
        transaction->abort_code = 0U;
        transaction->result_reported = false;
        if (abort_response) {
            transaction->abort_code = (uint32_t)data[4] |
                ((uint32_t)data[5] << 8) |
                ((uint32_t)data[6] << 16) |
                ((uint32_t)data[7] << 24);
            transaction->async_result = CANOPEN_ASYNC_RESULT_ABORT;
        } else if ((transaction->operation ==
                    CANOPEN_SDO_OP_ASYNC_WRITE) &&
                   write_ack) {
            transaction->async_result = CANOPEN_ASYNC_RESULT_SUCCESS;
        } else if ((transaction->operation ==
                    CANOPEN_SDO_OP_ASYNC_READ) &&
                   upload_response) {
            transaction->completion_value = (uint32_t)data[4];
            if (data[0] != 0x4FU) {
                transaction->completion_value |=
                    ((uint32_t)data[5] << 8);
            }
            if (data[0] == 0x43U) {
                transaction->completion_value |=
                    ((uint32_t)data[6] << 16) |
                    ((uint32_t)data[7] << 24);
            }
            transaction->async_result = CANOPEN_ASYNC_RESULT_SUCCESS;
        } else {
            transaction->async_result =
                CANOPEN_ASYNC_RESULT_PROTOCOL_ERROR;
            CanTrace_RecordSdo(
                CAN_TRACE_EVENT_SDO_MALFORMED_RESPONSE, trace_owner,
                node_id, index, subindex,
                CanopenTraceSdoIsWrite(transaction->operation),
                transaction->start_ms, now_ms, HAL_OK,
                CAN_TRACE_SDO_INTERNAL_MALFORMED_RESPONSE, data[0]);
        }
        transaction->state = CANOPEN_SDO_TX_COMPLETED;
        CanopenTraceSdoComplete(
            transaction, now_ms,
            (transaction->async_result == CANOPEN_ASYNC_RESULT_SUCCESS) ?
                CAN_TRACE_SDO_INTERNAL_NONE :
                ((transaction->async_result == CANOPEN_ASYNC_RESULT_ABORT) ?
                    CAN_TRACE_SDO_INTERNAL_REMOTE_ABORT :
                    CAN_TRACE_SDO_INTERNAL_MALFORMED_RESPONSE),
            transaction->abort_code);
        return;
    }

    if ((wheel_slot >= 0) &&
        ((transaction->operation == CANOPEN_SDO_OP_RUNTIME_VELOCITY) ||
         (transaction->operation == CANOPEN_SDO_OP_RUNTIME_POSITION))) {
        uint8_t slot = (uint8_t)wheel_slot;
        bool velocity_sample = transaction->operation ==
            CANOPEN_SDO_OP_RUNTIME_VELOCITY;
        CanopenValueSample_t *sample = velocity_sample ?
            &s_wheel_velocity_sdo[slot] : &s_wheel_position[slot];

        s_feedback_request[slot].pending = false;
        s_feedback_request[slot].fault_monitor_requested = false;
        if ((data[0] == 0x43U) && complete_frame) {
            sample->value = DecodeI32LittleEndian(&data[4]);
            sample->timestamp_ms = now_ms;
            sample->sequence++;
            sample->valid = true;
            if (velocity_sample &&
                (s_wheel_feedback_authority ==
                 CANOPEN_WHEEL_AUTHORITY_SDO)) {
                s_wheel_velocity[slot] = *sample;
                s_wheel_velocity_from_tpdo[slot] = false;
            }
            IncrementSaturated(&s_runtime_diag.wheel_feedback_success_count);
        } else {
            sample->valid = false;
            if (velocity_sample &&
                (s_wheel_feedback_authority ==
                 CANOPEN_WHEEL_AUTHORITY_SDO)) {
                s_wheel_velocity[slot].valid = false;
                s_wheel_velocity_from_tpdo[slot] = false;
            }
            IncrementSaturated(&s_runtime_diag.wheel_feedback_failure_count);
            s_feedback_continuity_fault_pending = true;
            CanTrace_RecordSdo(
                CAN_TRACE_EVENT_SDO_MALFORMED_RESPONSE, trace_owner,
                node_id, index, subindex, false,
                transaction->start_ms, now_ms, HAL_OK,
                CAN_TRACE_SDO_INTERNAL_MALFORMED_RESPONSE, data[0]);
        }
        CanopenTraceSdoComplete(
            transaction, now_ms,
            ((data[0] == 0x43U) && complete_frame) ?
                CAN_TRACE_SDO_INTERNAL_NONE :
                (abort_response ?
                    CAN_TRACE_SDO_INTERNAL_REMOTE_ABORT :
                    CAN_TRACE_SDO_INTERNAL_MALFORMED_RESPONSE),
            abort_response ?
                s_runtime_diag.last_sdo_abort_code : 0U);
        SdoTransactionSetIdle(transaction);
        return;
    }

    if ((wheel_slot >= 0) &&
        (transaction->operation == CANOPEN_SDO_OP_RUNTIME_TARGET)) {
        uint8_t slot = (uint8_t)wheel_slot;

        s_target_request[slot].pending = false;
        if (!write_ack) {
            MarkRuntimeTargetFailure(
                node_id,
                abort_response ?
                    CANOPEN_TARGET_FAILURE_SDO_ABORT :
                    CANOPEN_TARGET_FAILURE_PROTOCOL);
            if (!abort_response) {
                CanTrace_RecordSdo(
                    CAN_TRACE_EVENT_SDO_MALFORMED_RESPONSE, trace_owner,
                    node_id, index, subindex, true,
                    transaction->start_ms, now_ms, HAL_OK,
                    CAN_TRACE_SDO_INTERNAL_MALFORMED_RESPONSE, data[0]);
            }
        } else if (!s_target_request[0].pending &&
                   !s_target_request[1].pending &&
                   !s_runtime_fault_pending) {
            IncrementSaturated(&s_runtime_diag.wheel_target_complete_count);
        }
        CanopenTraceSdoComplete(
            transaction, now_ms,
            write_ack ? CAN_TRACE_SDO_INTERNAL_NONE :
                (abort_response ?
                    CAN_TRACE_SDO_INTERNAL_REMOTE_ABORT :
                    CAN_TRACE_SDO_INTERNAL_MALFORMED_RESPONSE),
            abort_response ?
                s_runtime_diag.last_sdo_abort_code : 0U);
        SdoTransactionSetIdle(transaction);
        return;
    }

    if ((wheel_slot >= 0) &&
        (transaction->operation == CANOPEN_SDO_OP_EMERGENCY_ZERO)) {
        uint8_t slot = (uint8_t)wheel_slot;

        if (write_ack) {
            s_emergency_zero_ack_mask |= (uint8_t)(1U << slot);
            if (s_emergency_zero_ack_mask == 0x03U) {
                s_emergency_zero_status =
                    CANOPEN_EMERGENCY_ZERO_ACKNOWLEDGED;
                IncrementSaturated(&s_runtime_diag.emergency_zero_ack_count);
            }
        } else {
            s_emergency_zero_status = CANOPEN_EMERGENCY_ZERO_FAILED;
            MarkRuntimeTargetFailure(
                node_id,
                abort_response ?
                    CANOPEN_TARGET_FAILURE_SDO_ABORT :
                    CANOPEN_TARGET_FAILURE_PROTOCOL);
            if (!abort_response) {
                CanTrace_RecordSdo(
                    CAN_TRACE_EVENT_SDO_MALFORMED_RESPONSE, trace_owner,
                    node_id, index, subindex, true,
                    transaction->start_ms, now_ms, HAL_OK,
                    CAN_TRACE_SDO_INTERNAL_MALFORMED_RESPONSE, data[0]);
            }
        }
        CanopenTraceSdoComplete(
            transaction, now_ms,
            write_ack ? CAN_TRACE_SDO_INTERNAL_NONE :
                (abort_response ?
                    CAN_TRACE_SDO_INTERNAL_REMOTE_ABORT :
                    CAN_TRACE_SDO_INTERNAL_MALFORMED_RESPONSE),
            abort_response ?
                s_runtime_diag.last_sdo_abort_code : 0U);
        SdoTransactionSetIdle(transaction);
        return;
    }

    IncrementSaturated(&s_runtime_diag.sdo_unexpected_response_count);
}

static bool IsTpdoCobId(uint32_t identifier)
{
    return ((identifier >= 0x180U) && (identifier <= 0x1FFU)) ||
           ((identifier >= 0x280U) && (identifier <= 0x2FFU)) ||
           ((identifier >= 0x380U) && (identifier <= 0x3FFU)) ||
           ((identifier >= 0x480U) && (identifier <= 0x4FFU));
}

static void HeartbeatLatchFault(uint8_t node_id,
                                CanopenHeartbeatFaultReason_t reason)
{
    CanopenHeartbeatState_t *heartbeat = NULL;

    if ((node_id == LEFT_NODE_ID) || (node_id == RIGHT_NODE_ID)) {
        heartbeat = &s_heartbeat[node_id];
        heartbeat->qualification_active = false;
        heartbeat->qualified = false;
        s_runtime_diag.heartbeat_qualification_active[node_id] = 0U;
        s_runtime_diag.heartbeat_qualified[node_id] = 0U;
        s_runtime_diag.heartbeat_fault_reason[node_id] = (uint8_t)reason;
        if (heartbeat->fault_latched) {
            return;
        }
        heartbeat->fault_latched = true;
    } else if ((node_id == 0U) &&
               (reason == CANOPEN_HEARTBEAT_FAULT_MASTER_TX)) {
        if (s_master_heartbeat.fault_latched) {
            return;
        }
        s_master_heartbeat.fault_latched = true;
        s_runtime_diag.master_heartbeat_fault_latched = 1U;
    } else {
        return;
    }

    if (!s_heartbeat_fault_pending) {
        s_heartbeat_fault_pending = true;
        s_heartbeat_fault_node = node_id;
        s_heartbeat_fault_reason = reason;
    }
    if ((reason == CANOPEN_HEARTBEAT_FAULT_STALE) ||
        (reason == CANOPEN_HEARTBEAT_FAULT_MASTER_TX)) {
        CanTrace_RecordHeartbeatFault(
            node_id,
            reason == CANOPEN_HEARTBEAT_FAULT_MASTER_TX,
            HAL_GetTick(), (uint32_t)reason);
    }
}

static bool HeartbeatNmtStateIsAlive(uint8_t state)
{
    return (state == 0x04U) || (state == 0x05U) || (state == 0x7FU);
}

static void HandleHeartbeatFrame(uint8_t node_id,
                                 const uint8_t *data,
                                 uint8_t len,
                                 uint32_t now_ms)
{
    CanopenHeartbeatState_t *heartbeat = &s_heartbeat[node_id];
    uint8_t state;

    IncrementSaturated(&s_runtime_diag.heartbeat_frame_count);
    IncrementSaturated(&s_runtime_diag.heartbeat_receive_count[node_id]);
    IncrementSaturated(&s_runtime_diag.heartbeat_sequence[node_id]);
    if (len != 1U) {
        IncrementSaturated(&s_runtime_diag.heartbeat_invalid_count[node_id]);
        if (heartbeat->qualified) {
            HeartbeatLatchFault(node_id, CANOPEN_HEARTBEAT_FAULT_LENGTH);
        }
        return;
    }

    state = data[0];
    s_runtime_diag.last_heartbeat_state[node_id] = state;
    if (state == 0x00U) {
        IncrementSaturated(&s_runtime_diag.heartbeat_bootup_count[node_id]);
        if (heartbeat->qualified) {
            HeartbeatLatchFault(node_id, CANOPEN_HEARTBEAT_FAULT_BOOTUP);
        } else if (heartbeat->qualification_active) {
            heartbeat->baseline_sequence =
                s_runtime_diag.heartbeat_sequence[node_id];
            heartbeat->fresh_valid_count = 0U;
            s_runtime_diag.heartbeat_baseline_sequence[node_id] =
                heartbeat->baseline_sequence;
            s_runtime_diag.heartbeat_qualification_count[node_id] = 0U;
        }
        return;
    }

    if (!HeartbeatNmtStateIsAlive(state)) {
        IncrementSaturated(&s_runtime_diag.heartbeat_invalid_count[node_id]);
        if (heartbeat->qualified) {
            HeartbeatLatchFault(node_id, CANOPEN_HEARTBEAT_FAULT_STATE);
        }
        return;
    }

    heartbeat->last_heartbeat_ms = now_ms;
    heartbeat->last_nmt_state = state;
    s_runtime_diag.last_heartbeat_ms[node_id] = now_ms;
    if (!heartbeat->qualification_active || heartbeat->fault_latched ||
        (s_runtime_diag.heartbeat_sequence[node_id] <=
         heartbeat->baseline_sequence)) {
        return;
    }
    if (heartbeat->fresh_valid_count != UINT8_MAX) {
        heartbeat->fresh_valid_count++;
    }
    s_runtime_diag.heartbeat_qualification_count[node_id] =
        heartbeat->fresh_valid_count;
    if (heartbeat->fresh_valid_count >= DRIVE_HEARTBEAT_QUALIFY_FRAMES) {
        heartbeat->qualification_active = false;
        heartbeat->qualified = true;
        s_runtime_diag.heartbeat_qualification_active[node_id] = 0U;
        s_runtime_diag.heartbeat_qualified[node_id] = 1U;
    }
}

static void WheelTpdoLogTransition(
    uint8_t slot,
    CanopenWheelTpdoSampleState_t from,
    CanopenWheelTpdoSampleState_t to)
{
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_STATUS,
        "DRIVE_TPDO node=%u state=%s->%s\r\n",
        (unsigned)((slot == 0U) ? LEFT_NODE_ID : RIGHT_NODE_ID),
        Canopen_WheelTpdoSampleStateName(from),
        Canopen_WheelTpdoSampleStateName(to));
}

static uint32_t WheelTpdoTimestampSkew(uint32_t left_ms, uint32_t right_ms)
{
    int32_t delta = (int32_t)(left_ms - right_ms);

    return (delta >= 0) ? (uint32_t)delta : (uint32_t)(-(int64_t)delta);
}

static void ResetWheelTpdoPairEpoch(void)
{
    memset(&s_wheel_tpdo_pair, 0, sizeof(s_wheel_tpdo_pair));
    s_wheel_tpdo_pair.state = CANOPEN_TPDO_PAIR_NO_DATA;
    s_wheel_tpdo_committed_node_sequence[0] = 0U;
    s_wheel_tpdo_committed_node_sequence[1] = 0U;
}

static void PublishAuthoritativeTpdoPair(void)
{
    uint8_t slot;

    for (slot = 0U; slot < 2U; slot++) {
        const CanopenWheelTpdoSample_t *node = &s_wheel_tpdo_shadow[slot];

        s_wheel_velocity[slot].value = node->raw;
        s_wheel_velocity[slot].timestamp_ms = node->timestamp_ms;
        s_wheel_velocity[slot].sequence = node->sequence;
        s_wheel_velocity[slot].valid = true;
        s_wheel_velocity_from_tpdo[slot] = true;
        IncrementSaturated(&s_runtime_diag.wheel_feedback_success_count);
    }
}

static void TryCommitWheelTpdoPair(void)
{
    const CanopenWheelTpdoSample_t *left = &s_wheel_tpdo_shadow[0];
    const CanopenWheelTpdoSample_t *right = &s_wheel_tpdo_shadow[1];
    uint32_t skew_ms;
    bool arming = s_wheel_feedback_transport ==
        CANOPEN_WHEEL_FEEDBACK_TPDO_ACTIVE_ARMING;

    if ((s_wheel_feedback_transport != CANOPEN_WHEEL_FEEDBACK_TPDO_SHADOW) &&
        !arming &&
        (s_wheel_feedback_transport != CANOPEN_WHEEL_FEEDBACK_TPDO_ACTIVE)) {
        return;
    }
    if (!left->valid || !right->valid ||
        (left->state != CANOPEN_TPDO_SAMPLE_FRESH) ||
        (right->state != CANOPEN_TPDO_SAMPLE_FRESH) ||
        (left->sequence == s_wheel_tpdo_committed_node_sequence[0]) ||
        (right->sequence == s_wheel_tpdo_committed_node_sequence[1])) {
        return;
    }
    if (arming &&
        ((left->sequence == s_wheel_tpdo_arm_baseline_sequence[0]) ||
         (right->sequence == s_wheel_tpdo_arm_baseline_sequence[1]) ||
         ((int32_t)(left->timestamp_ms - s_wheel_tpdo_arm_start_ms) < 0) ||
         ((int32_t)(right->timestamp_ms - s_wheel_tpdo_arm_start_ms) < 0))) {
        return;
    }

    skew_ms = WheelTpdoTimestampSkew(left->timestamp_ms,
                                     right->timestamp_ms);
    if (skew_ms > DRIVE_WHEEL_FEEDBACK_MAX_SKEW_MS) {
        return;
    }

    s_wheel_tpdo_pair.left_raw = left->raw;
    s_wheel_tpdo_pair.right_raw = right->raw;
    s_wheel_tpdo_pair.left_timestamp_ms = left->timestamp_ms;
    s_wheel_tpdo_pair.right_timestamp_ms = right->timestamp_ms;
    s_wheel_tpdo_pair.left_sequence = left->sequence;
    s_wheel_tpdo_pair.right_sequence = right->sequence;
    s_wheel_tpdo_pair.timestamp_ms =
        ((int32_t)(left->timestamp_ms - right->timestamp_ms) >= 0) ?
        left->timestamp_ms : right->timestamp_ms;
    s_wheel_tpdo_pair.sequence++;
    if (s_wheel_tpdo_pair.sequence == 0U) {
        s_wheel_tpdo_pair.sequence++;
    }
    s_wheel_tpdo_pair.pair_skew_ms = skew_ms;
    s_wheel_tpdo_pair.last_coherent_pair_timestamp_ms =
        s_wheel_tpdo_pair.timestamp_ms;
    s_wheel_tpdo_pair.valid = true;
    s_wheel_tpdo_pair.state = CANOPEN_TPDO_PAIR_FRESH;
    s_wheel_tpdo_committed_node_sequence[0] = left->sequence;
    s_wheel_tpdo_committed_node_sequence[1] = right->sequence;

    if (arming) {
        s_wheel_feedback_authority = CANOPEN_WHEEL_AUTHORITY_TPDO;
        s_wheel_feedback_transport = CANOPEN_WHEEL_FEEDBACK_TPDO_ACTIVE;
        s_last_wheel_tpdo_active_fault = "NONE";
    }
    if (s_wheel_feedback_authority == CANOPEN_WHEEL_AUTHORITY_TPDO) {
        PublishAuthoritativeTpdoPair();
    }
}

static void FaultAuthoritativeTpdoPair(void)
{
    if ((s_wheel_feedback_authority != CANOPEN_WHEEL_AUTHORITY_TPDO) ||
        (s_wheel_feedback_transport == CANOPEN_WHEEL_FEEDBACK_TPDO_FAULT)) {
        return;
    }

    s_wheel_tpdo_pair.valid = false;
    s_wheel_tpdo_pair.state = CANOPEN_TPDO_PAIR_FAULT;
    s_wheel_velocity[0].valid = false;
    s_wheel_velocity[1].valid = false;
    s_wheel_feedback_transport = CANOPEN_WHEEL_FEEDBACK_TPDO_FAULT;
    s_last_wheel_tpdo_active_fault = "TPDO_ACTIVE_FEEDBACK_STALE";
    s_feedback_continuity_fault_pending = true;
}

static void ServiceWheelTpdoFreshness(uint32_t now_ms)
{
    uint8_t slot;

    for (slot = 0U; slot < 2U; slot++) {
        CanopenWheelTpdoSample_t *sample = &s_wheel_tpdo_shadow[slot];

        if (!sample->seen || (sample->state != CANOPEN_TPDO_SAMPLE_FRESH) ||
            ((uint32_t)(now_ms - sample->timestamp_ms) <=
             DRIVE_WHEEL_FEEDBACK_STALE_MS)) {
            continue;
        }
        sample->state = CANOPEN_TPDO_SAMPLE_STALE;
        sample->valid = false;
        IncrementSaturated(&sample->stale_transition_count);
        if ((s_wheel_feedback_transport ==
             CANOPEN_WHEEL_FEEDBACK_TPDO_ACTIVE) &&
            s_wheel_velocity_from_tpdo[slot]) {
            s_wheel_velocity[slot].valid = false;
        }
        WheelTpdoLogTransition(slot, CANOPEN_TPDO_SAMPLE_FRESH,
                               CANOPEN_TPDO_SAMPLE_STALE);
        CanTrace_RecordTpdoTransition(
            now_ms,
            (slot == 0U) ? DRIVE_WHEEL_TPDO_LEFT_COB_ID :
                           DRIVE_WHEEL_TPDO_RIGHT_COB_ID,
            (slot == 0U) ? LEFT_NODE_ID : RIGHT_NODE_ID,
            CAN_TRACE_RX_DETAIL_TPDO_STALE,
            (uint32_t)(now_ms - sample->timestamp_ms));
    }

    if ((s_wheel_feedback_authority == CANOPEN_WHEEL_AUTHORITY_TPDO) &&
        s_wheel_tpdo_pair.valid &&
        (((uint32_t)(now_ms -
                     s_wheel_tpdo_pair.last_coherent_pair_timestamp_ms) >
          DRIVE_WHEEL_FEEDBACK_STALE_MS) ||
         !s_wheel_tpdo_shadow[0].valid || !s_wheel_tpdo_shadow[1].valid)) {
        FaultAuthoritativeTpdoPair();
    } else if ((s_wheel_feedback_authority ==
                CANOPEN_WHEEL_AUTHORITY_SDO) &&
               s_wheel_tpdo_pair.valid &&
               (((uint32_t)(now_ms -
                            s_wheel_tpdo_pair.last_coherent_pair_timestamp_ms) >
                 DRIVE_WHEEL_FEEDBACK_STALE_MS) ||
                !s_wheel_tpdo_shadow[0].valid ||
                !s_wheel_tpdo_shadow[1].valid)) {
        s_wheel_tpdo_pair.valid = false;
        s_wheel_tpdo_pair.state = CANOPEN_TPDO_PAIR_STALE;
    }
}

static void HandleWheelTpdoFrame(uint8_t slot,
                                 const uint8_t *data,
                                 uint8_t len,
                                 uint32_t now_ms)
{
    CanopenWheelTpdoSample_t *sample = &s_wheel_tpdo_shadow[slot];
    CanopenWheelTpdoSampleState_t previous_state;
    uint32_t gap_ms = 0U;

    /* These two counts are lifetime receive diagnostics for every exact
     * wheel TPDO COB-ID, including frames rejected below. */
    IncrementSaturated(&sample->frame_count);
    IncrementSaturated(&s_runtime_diag.tpdo_frame_count);
    if (!s_wheel_tpdo_configured[slot]) {
        IncrementSaturated(&sample->ignored_unconfigured_frame_count);
        return;
    }
    if ((s_wheel_feedback_transport !=
         CANOPEN_WHEEL_FEEDBACK_TPDO_SHADOW) &&
        (s_wheel_feedback_transport !=
         CANOPEN_WHEEL_FEEDBACK_TPDO_ACTIVE_ARMING) &&
        (s_wheel_feedback_transport !=
         CANOPEN_WHEEL_FEEDBACK_TPDO_ACTIVE) &&
        (s_wheel_feedback_transport !=
         CANOPEN_WHEEL_FEEDBACK_TPDO_FAULT)) {
        IncrementSaturated(&sample->ignored_transport_mode_frame_count);
        return;
    }
    if (len != DRIVE_WHEEL_TPDO_EXPECTED_DLC) {
        IncrementSaturated(&sample->invalid_dlc_count);
        return;
    }
    previous_state = sample->state;
    if (sample->seen) {
        gap_ms = (uint32_t)(now_ms - sample->timestamp_ms);

        if (gap_ms > sample->max_interarrival_gap_ms) {
            sample->max_interarrival_gap_ms = gap_ms;
        }
    }

    sample->raw = DecodeI32LittleEndian(data);
    sample->timestamp_ms = now_ms;
    sample->sequence++;
    sample->seen = true;
    sample->valid = true;
    sample->state = CANOPEN_TPDO_SAMPLE_FRESH;
    if (previous_state == CANOPEN_TPDO_SAMPLE_STALE) {
        IncrementSaturated(&sample->recovery_transition_count);
        CanTrace_RecordTpdoTransition(
            now_ms,
            (slot == 0U) ? DRIVE_WHEEL_TPDO_LEFT_COB_ID :
                           DRIVE_WHEEL_TPDO_RIGHT_COB_ID,
            (slot == 0U) ? LEFT_NODE_ID : RIGHT_NODE_ID,
            CAN_TRACE_RX_DETAIL_TPDO_RECOVERY, gap_ms);
    } else if (sample->seen &&
               (gap_ms > CAN_TRACE_TPDO_AGGREGATE_MS)) {
        CanTrace_RecordTpdoTransition(
            now_ms,
            (slot == 0U) ? DRIVE_WHEEL_TPDO_LEFT_COB_ID :
                           DRIVE_WHEEL_TPDO_RIGHT_COB_ID,
            (slot == 0U) ? LEFT_NODE_ID : RIGHT_NODE_ID,
            CAN_TRACE_RX_DETAIL_TPDO_GAP, gap_ms);
    }
    if (previous_state != CANOPEN_TPDO_SAMPLE_FRESH) {
        WheelTpdoLogTransition(slot, previous_state,
                               CANOPEN_TPDO_SAMPLE_FRESH);
    }

    TryCommitWheelTpdoPair();
}

static void DispatchStandardFrame(uint32_t identifier,
                                  const uint8_t *data,
                                  uint8_t len,
                                  uint32_t now_ms)
{
    if ((identifier == DRIVE_WHEEL_TPDO_LEFT_COB_ID) ||
        (identifier == DRIVE_WHEEL_TPDO_RIGHT_COB_ID)) {
        uint8_t slot = (identifier == DRIVE_WHEEL_TPDO_LEFT_COB_ID) ? 0U : 1U;

        HandleWheelTpdoFrame(slot, data, len, now_ms);
        return;
    }

    if ((identifier >= 0x581U) && (identifier <= 0x5FFU)) {
        HandleSdoResponse((uint8_t)(identifier - 0x580U), data, len, now_ms);
        return;
    }

    if ((identifier == (0x700U + LEFT_NODE_ID)) ||
        (identifier == (0x700U + RIGHT_NODE_ID))) {
        uint8_t node_id = (uint8_t)(identifier - 0x700U);

        if (len == 1U) { WheelPdoConfig_OnHeartbeat(node_id, data[0], now_ms); }
        HandleHeartbeatFrame(node_id, data, len, now_ms);
        return;
    }

    if ((identifier >= 0x703U) && (identifier <= 0x77FU) && (len >= 1U)) {
        uint8_t node_id = (uint8_t)(identifier - 0x700U);
        IncrementSaturated(&s_runtime_diag.heartbeat_frame_count);
        if (node_id < 4U) {
            s_runtime_diag.last_heartbeat_ms[node_id] = now_ms;
            s_runtime_diag.last_heartbeat_state[node_id] = data[0];
            IncrementSaturated(&s_runtime_diag.heartbeat_sequence[node_id]);
        }
        return;
    }

    if ((identifier >= 0x081U) && (identifier <= 0x0FFU) && (len >= 2U)) {
        uint8_t node_id = (uint8_t)(identifier - 0x080U);
        IncrementSaturated(&s_runtime_diag.emcy_frame_count);
        if (node_id < 4U) {
            s_runtime_diag.last_emcy_code[node_id] =
                (uint16_t)data[0] | ((uint16_t)data[1] << 8);
            s_runtime_diag.last_emcy_ms[node_id] = now_ms;
            IncrementSaturated(&s_runtime_diag.emcy_sequence[node_id]);
            if ((node_id == LIFT_NODE_ID) &&
                (s_runtime_diag.last_emcy_code[node_id] != 0U)) {
                s_lift_fault_emcy_code = s_runtime_diag.last_emcy_code[node_id];
                s_lift_fault_emcy_pending = true;
            }
            if (((node_id == LEFT_NODE_ID) || (node_id == RIGHT_NODE_ID)) &&
                (s_runtime_diag.last_emcy_code[node_id] != 0U)) {
                if (s_boot_sdo_observing &&
                    (s_boot_sdo_evidence.hard_fault_reason == CANOPEN_BOOT_SDO_FAULT_NONE)) {
                    BootSdoLatchFault(CANOPEN_BOOT_SDO_FAULT_EMCY,
                                      node_id, 0U, 0U);
                    s_boot_sdo_evidence.emcy_code = s_runtime_diag.last_emcy_code[node_id];
                }
                WheelRpdoTx_Cancel(now_ms);
                MarkRuntimeTargetFailure(node_id, CANOPEN_TARGET_FAILURE_EMCY);
            }
        }
        return;
    }

    if (IsTpdoCobId(identifier)) {
        /* Count and retain the frame class, but do not guess its mapping. */
        IncrementSaturated(&s_runtime_diag.tpdo_frame_count);
        return;
    }

    IncrementSaturated(&s_runtime_diag.unhandled_frame_count);
}

void Canopen_ProcessRx(uint32_t now_ms, uint32_t frame_budget)
{
    uint32_t processed = 0U;

    CanTrace_RuntimeBegin(CAN_TRACE_RUNTIME_CANOPEN_RX, HAL_GetTick());
    ServiceWheelTpdoFreshness(now_ms);
    while ((processed < frame_budget) &&
           (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) != 0U)) {
        FDCAN_RxHeaderTypeDef rx = {0};
        /* HAL copies the decoded DLC before callers can reject the frame.
         * Reserve its maximum payload, then accept only supported lengths. */
        uint8_t data[64] = {0};

        HAL_StatusTypeDef rx_status = HAL_FDCAN_GetRxMessage(
            &hfdcan1, FDCAN_RX_FIFO0, &rx, data);

        if (rx_status != HAL_OK) {
            IncrementSaturated(&s_runtime_diag.rx_error_count);
            CanTrace_RecordRxError(now_ms, rx_status);
            break;
        }

        processed++;
        IncrementSaturated(&s_runtime_diag.rx_frame_count);
        CanTrace_RecordRx(
            now_ms, rx.Identifier, fdcan_dlc_to_len(rx.DataLength),
            (uint8_t)(rx.Identifier & 0x7FU));
        if ((rx.IdType == FDCAN_STANDARD_ID) &&
            (rx.RxFrameType == FDCAN_DATA_FRAME)) {
            DispatchStandardFrame(rx.Identifier,
                                  data,
                                  fdcan_dlc_to_len(rx.DataLength),
                                  now_ms);
        } else {
            IncrementSaturated(&s_runtime_diag.unhandled_frame_count);
        }
    }
    CanTrace_RuntimeEnd(CAN_TRACE_RUNTIME_CANOPEN_RX, HAL_GetTick());
}

static bool Wait_SDO_Response(const CanopenNode_t *node,
                              uint16_t expected_index,
                              uint8_t expected_subindex,
                              uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    CanopenSdoTransaction_t *transaction =
        SdoTransactionFromNode(node->node_id);

    while ((uint32_t)(HAL_GetTick() - start) <= timeout_ms) {
        uint32_t now_ms = HAL_GetTick();

        Canopen_ProcessRx(now_ms, 1U);
        Canopen_HeartbeatService(now_ms);
        JetsonTx_Process();
        ServiceSdoTransactionTimeouts(now_ms);
        if ((transaction != NULL) &&
            (transaction->state == CANOPEN_SDO_TX_COMPLETED) &&
            (transaction->index == expected_index) &&
            (transaction->subindex == expected_subindex)) {
            bool success = transaction->completion_success;

            bool remote_abort = (s_last_sdo_response.command == 0x80U) &&
                                (s_last_sdo_response.len == 8U);
            if (!success && remote_abort) {
                uint32_t abort_code = 0U;
                if (s_last_sdo_response.len >= 8U) {
                    abort_code = (uint32_t)s_last_sdo_response.data[4] |
                                 ((uint32_t)s_last_sdo_response.data[5] << 8) |
                                 ((uint32_t)s_last_sdo_response.data[6] << 16) |
                                 ((uint32_t)s_last_sdo_response.data[7] << 24);
                }
                if (!s_sdo_quiet) {
                    uart_printf("%s SDO abort node=%u index=0x%04X sub=0x%02X code=0x%08lX\r\n",
                                FDCAN_HandleName(node->hfdcan),
                                node->node_id,
                                expected_index,
                                expected_subindex,
                                (unsigned long)abort_code);
                }
            }
            CanTrace_RecordSdo(
                CAN_TRACE_EVENT_SDO_COMPLETE,
                CanopenTraceSdoOwner(transaction, node->node_id),
                node->node_id, expected_index, expected_subindex,
                CanopenTraceSdoIsWrite(transaction->operation),
                transaction->start_ms, now_ms, HAL_OK,
                success ? CAN_TRACE_SDO_INTERNAL_NONE :
                    (remote_abort ?
                     CAN_TRACE_SDO_INTERNAL_REMOTE_ABORT :
                     CAN_TRACE_SDO_INTERNAL_MALFORMED_RESPONSE),
                remote_abort ?
                    ((uint32_t)s_last_sdo_response.data[4] |
                     ((uint32_t)s_last_sdo_response.data[5] << 8) |
                     ((uint32_t)s_last_sdo_response.data[6] << 16) |
                     ((uint32_t)s_last_sdo_response.data[7] << 24)) : 0U);
            SdoTransactionSetIdle(transaction);
            CanTrace_RuntimeObserve(
                CAN_TRACE_RUNTIME_BLOCKING_SDO, start, now_ms);
            return success;
        }
        if ((transaction != NULL) &&
            (transaction->state == CANOPEN_SDO_TX_QUARANTINE)) {
            break;
        }

        HAL_Delay(1U);
    }

    ServiceSdoTransactionTimeouts(HAL_GetTick());
    if ((transaction != NULL) &&
        (transaction->state == CANOPEN_SDO_TX_ACTIVE)) {
        IncrementSaturated(&s_runtime_diag.sdo_timeout_count);
        {
            uint32_t timeout_now = HAL_GetTick();

            CanTrace_RecordSdo(
                CAN_TRACE_EVENT_SDO_TIMEOUT,
                CanopenTraceSdoOwner(transaction, node->node_id),
                node->node_id, expected_index, expected_subindex,
                CanopenTraceSdoIsWrite(transaction->operation),
                transaction->start_ms, timeout_now, HAL_OK,
                CAN_TRACE_SDO_INTERNAL_NO_RESPONSE_TIMEOUT, 0U);
            SdoTransactionEnterQuarantine(transaction, timeout_now);
        }
    }
    s_last_sdo_response.valid = false;
    if (!s_sdo_quiet) {
        uart_printf("%s SDO timeout node=%u index=0x%04X sub=0x%02X after %lu ms\r\n",
                    FDCAN_HandleName(node->hfdcan),
                    node->node_id,
                    expected_index,
                    expected_subindex,
                    (unsigned long)timeout_ms);
    }
    CanTrace_RuntimeObserve(
        CAN_TRACE_RUNTIME_BLOCKING_SDO, start, HAL_GetTick());
    return false;
}

static bool WaitSdoNodeAvailable(uint8_t node_id, uint32_t maximum_wait_ms)
{
    uint32_t start_ms = HAL_GetTick();

    if (BootSdoNodeIsFenced(node_id)) {
        return false;
    }
    do {
        uint32_t now_ms = HAL_GetTick();
        CanopenSdoTransaction_t *transaction =
            SdoTransactionFromNode(node_id);

        Canopen_ProcessRx(now_ms, 1U);
        Canopen_HeartbeatService(now_ms);
        JetsonTx_Process();
        ServiceSdoTransactionTimeouts(now_ms);
        SdoTransactionRefreshQuarantine(transaction, now_ms);
        if ((transaction != NULL) &&
            (transaction->state == CANOPEN_SDO_TX_IDLE)) {
            return true;
        }
        HAL_Delay(1U);
    } while ((uint32_t)(HAL_GetTick() - start_ms) < maximum_wait_ms);

    return false;
}

static bool StartBlockingSdo(const CanopenNode_t *node,
                             uint16_t index,
                             uint8_t subindex,
                             CanopenSdoOperation_t operation,
                             uint8_t expected_upload_size,
                             uint32_t timeout_ms,
                             const uint8_t data[8],
                             const char *label)
{
    CanopenSdoTransaction_t *transaction;
    uint32_t now_ms;

    if ((node == NULL) ||
        !WaitSdoNodeAvailable(
            node->node_id,
            DRIVE_RUNTIME_TARGET_TIMEOUT_MS + DRIVE_SDO_QUARANTINE_MS + 5U)) {
        if (node != NULL) {
            CanTrace_RecordSdo(
                CAN_TRACE_EVENT_SDO_ENQUEUE_FAILED,
                (node->node_id == LIFT_NODE_ID) ?
                    CAN_TRACE_TX_LIFT_SDO : CAN_TRACE_TX_WHEEL_SDO,
                node->node_id, index, subindex,
                CanopenTraceSdoIsWrite(operation),
                HAL_GetTick(), HAL_GetTick(), HAL_BUSY,
                CAN_TRACE_SDO_INTERNAL_STATE_ERROR, 0U);
        }
        return false;
    }
    now_ms = HAL_GetTick();
    if (!SdoTransactionStart(node->node_id, index, subindex, operation,
                             CANOPEN_SDO_OWNER_BLOCKING,
                             now_ms, timeout_ms, expected_upload_size)) {
        CanTrace_RecordSdo(
            CAN_TRACE_EVENT_SDO_ENQUEUE_FAILED,
            (node->node_id == LIFT_NODE_ID) ?
                CAN_TRACE_TX_LIFT_SDO : CAN_TRACE_TX_WHEEL_SDO,
            node->node_id, index, subindex,
            CanopenTraceSdoIsWrite(operation),
            now_ms, now_ms, HAL_BUSY,
            CAN_TRACE_SDO_INTERNAL_STATE_ERROR, 0U);
        return false;
    }
    s_last_sdo_response.valid = false;
    if (Canopen_SendSdoFrame(node, data, label, false, false) != HAL_OK) {
        transaction = SdoTransactionFromNode(node->node_id);
        SdoTransactionSetIdle(transaction);
        return false;
    }
    return true;
}

static CanopenNode_t CanopenNode_OnFDCAN1(uint8_t node_id)
{
    CanopenNode_t node = { &hfdcan1, node_id };
    return node;
}

static void WheelTpdoConfigBeginStep(uint8_t node_id,
                                     uint8_t step,
                                     uint16_t index,
                                     uint8_t subindex,
                                     uint32_t expected_value)
{
    memset(&s_wheel_tpdo_config_diagnostic, 0,
           sizeof(s_wheel_tpdo_config_diagnostic));
    s_wheel_tpdo_config_diagnostic.node_id = node_id;
    s_wheel_tpdo_config_diagnostic.step = step;
    s_wheel_tpdo_config_diagnostic.index = index;
    s_wheel_tpdo_config_diagnostic.subindex = subindex;
    s_wheel_tpdo_config_diagnostic.expected_value = expected_value;
    s_wheel_tpdo_config_diagnostic.sdo_result =
        CANOPEN_TPDO_CONFIG_SDO_NOT_RUN;
    s_wheel_tpdo_config_diagnostic.result =
        CANOPEN_TPDO_CONFIG_RESULT_NONE;
}

static bool WheelTpdoConfigFinishSdo(bool ok,
                                     uint32_t timeout_count_before,
                                     uint32_t abort_count_before)
{
    if (ok) {
        s_wheel_tpdo_config_diagnostic.sdo_result =
            CANOPEN_TPDO_CONFIG_SDO_SUCCESS;
        return true;
    }
    s_wheel_tpdo_config_diagnostic.result =
        CANOPEN_TPDO_CONFIG_RESULT_SDO_FAILURE;
    if (s_runtime_diag.sdo_abort_count != abort_count_before) {
        s_wheel_tpdo_config_diagnostic.sdo_result =
            CANOPEN_TPDO_CONFIG_SDO_ABORT;
        s_wheel_tpdo_config_diagnostic.abort_code =
            s_runtime_diag.last_sdo_abort_code;
    } else if (s_runtime_diag.sdo_timeout_count != timeout_count_before) {
        s_wheel_tpdo_config_diagnostic.sdo_result =
            CANOPEN_TPDO_CONFIG_SDO_TIMEOUT;
    } else {
        s_wheel_tpdo_config_diagnostic.sdo_result =
            CANOPEN_TPDO_CONFIG_SDO_ERROR;
    }
    return false;
}

static bool WheelTpdoConfigWriteU8(const CanopenNode_t *node,
                                   uint8_t step,
                                   uint16_t index,
                                   uint8_t subindex,
                                   uint8_t value)
{
    uint32_t timeout_before = s_runtime_diag.sdo_timeout_count;
    uint32_t abort_before = s_runtime_diag.sdo_abort_count;
    bool ok;

    WheelTpdoConfigBeginStep(node->node_id, step, index, subindex, value);
    ok = SDO_Write_U8_Timeout_Node(
        node, index, subindex, value,
        DRIVE_WHEEL_TPDO_CONFIG_SDO_TIMEOUT_MS);
    if (WheelTpdoConfigFinishSdo(ok, timeout_before, abort_before)) {
        s_wheel_tpdo_config_diagnostic.actual_value = value;
        return true;
    }
    return false;
}

static bool WheelTpdoConfigWriteU16(const CanopenNode_t *node,
                                    uint8_t step,
                                    uint16_t index,
                                    uint8_t subindex,
                                    uint16_t value)
{
    uint32_t timeout_before = s_runtime_diag.sdo_timeout_count;
    uint32_t abort_before = s_runtime_diag.sdo_abort_count;
    bool ok;

    WheelTpdoConfigBeginStep(node->node_id, step, index, subindex, value);
    ok = SDO_Write_U16_Timeout_Node(
        node, index, subindex, value,
        DRIVE_WHEEL_TPDO_CONFIG_SDO_TIMEOUT_MS);
    if (WheelTpdoConfigFinishSdo(ok, timeout_before, abort_before)) {
        s_wheel_tpdo_config_diagnostic.actual_value = value;
        return true;
    }
    return false;
}

static bool WheelTpdoConfigWriteU32(const CanopenNode_t *node,
                                    uint8_t step,
                                    uint16_t index,
                                    uint8_t subindex,
                                    uint32_t value)
{
    uint32_t timeout_before = s_runtime_diag.sdo_timeout_count;
    uint32_t abort_before = s_runtime_diag.sdo_abort_count;
    bool ok;

    WheelTpdoConfigBeginStep(node->node_id, step, index, subindex, value);
    ok = SDO_Write_I32_Timeout_Node(
        node, index, subindex, (int32_t)value,
        DRIVE_WHEEL_TPDO_CONFIG_SDO_TIMEOUT_MS);
    if (WheelTpdoConfigFinishSdo(ok, timeout_before, abort_before)) {
        s_wheel_tpdo_config_diagnostic.actual_value = value;
        return true;
    }
    return false;
}

static bool WheelTpdoConfigCheckReadback(bool sdo_ok,
                                         uint32_t actual_value,
                                         uint32_t timeout_count_before,
                                         uint32_t abort_count_before)
{
    s_wheel_tpdo_config_diagnostic.actual_value = actual_value;
    if (!WheelTpdoConfigFinishSdo(sdo_ok, timeout_count_before,
                                  abort_count_before)) {
        return false;
    }
    if (actual_value != s_wheel_tpdo_config_diagnostic.expected_value) {
        s_wheel_tpdo_config_diagnostic.result =
            CANOPEN_TPDO_CONFIG_RESULT_READBACK_MISMATCH;
        return false;
    }
    return true;
}

static bool WheelTpdoConfigReadU8(const CanopenNode_t *node,
                                  uint8_t step,
                                  uint16_t index,
                                  uint8_t subindex,
                                  uint8_t expected_value)
{
    uint32_t timeout_before = s_runtime_diag.sdo_timeout_count;
    uint32_t abort_before = s_runtime_diag.sdo_abort_count;
    int8_t actual = 0;
    bool ok;

    WheelTpdoConfigBeginStep(node->node_id, step, index, subindex,
                             expected_value);
    ok = SDO_Read_I8_Timeout_Node(
        node, index, subindex, &actual,
        DRIVE_WHEEL_TPDO_CONFIG_SDO_TIMEOUT_MS);
    return WheelTpdoConfigCheckReadback(ok, (uint8_t)actual,
                                        timeout_before, abort_before);
}

static bool WheelTpdoConfigReadU16(const CanopenNode_t *node,
                                   uint8_t step,
                                   uint16_t index,
                                   uint8_t subindex,
                                   uint16_t expected_value)
{
    uint32_t timeout_before = s_runtime_diag.sdo_timeout_count;
    uint32_t abort_before = s_runtime_diag.sdo_abort_count;
    uint16_t actual = 0U;
    bool ok;

    WheelTpdoConfigBeginStep(node->node_id, step, index, subindex,
                             expected_value);
    ok = SDO_Read_U16_Timeout_Node(
        node, index, subindex, &actual,
        DRIVE_WHEEL_TPDO_CONFIG_SDO_TIMEOUT_MS);
    return WheelTpdoConfigCheckReadback(ok, actual,
                                        timeout_before, abort_before);
}

static bool WheelTpdoConfigReadU32(const CanopenNode_t *node,
                                   uint8_t step,
                                   uint16_t index,
                                   uint8_t subindex,
                                   uint32_t expected_value)
{
    uint32_t timeout_before = s_runtime_diag.sdo_timeout_count;
    uint32_t abort_before = s_runtime_diag.sdo_abort_count;
    int32_t actual = 0;
    bool ok;

    WheelTpdoConfigBeginStep(node->node_id, step, index, subindex,
                             expected_value);
    ok = SDO_Read_I32_Timeout_Node(
        node, index, subindex, &actual,
        DRIVE_WHEEL_TPDO_CONFIG_SDO_TIMEOUT_MS);
    return WheelTpdoConfigCheckReadback(ok, (uint32_t)actual,
                                        timeout_before, abort_before);
}

static void ResetWheelTpdoMeasurementEpoch(uint8_t slot)
{
    CanopenWheelTpdoSample_t *sample = &s_wheel_tpdo_shadow[slot];

    /* Configuration starts a new measurement epoch. Lifetime receive,
     * rejection, invalid-DLC, and freshness-transition counters survive. */
    sample->raw = 0;
    sample->timestamp_ms = 0U;
    sample->sequence = 0U;
    sample->seen = false;
    sample->valid = false;
    sample->max_interarrival_gap_ms = 0U;
    sample->state = CANOPEN_TPDO_SAMPLE_NO_DATA;
}

bool Canopen_ConfigureWheelTpdoFeedbackNode(uint8_t node_id)
{
    int8_t slot = WheelSlotFromNode(node_id);
    CanopenNode_t node;
    uint32_t cob_id;
    bool ok;

    if (slot < 0) {
        WheelTpdoConfigBeginStep(node_id, 0U, 0U, 0U, 0U);
        s_wheel_tpdo_config_diagnostic.result =
            CANOPEN_TPDO_CONFIG_RESULT_INVALID_NODE;
        return false;
    }
    node = CanopenNode_OnFDCAN1(node_id);
    cob_id = (node_id == LEFT_NODE_ID) ?
        DRIVE_WHEEL_TPDO_LEFT_COB_ID : DRIVE_WHEEL_TPDO_RIGHT_COB_ID;
    s_wheel_tpdo_configured[(uint8_t)slot] = false;
    ResetWheelTpdoMeasurementEpoch((uint8_t)slot);
    ResetWheelTpdoPairEpoch();
    s_wheel_feedback_authority = CANOPEN_WHEEL_AUTHORITY_SDO;
    s_wheel_feedback_transport =
        CANOPEN_WHEEL_FEEDBACK_TPDO_SHADOW_CONFIGURING;
    IncrementSaturated(&s_runtime_diag.pdo_config_attempt_count);

    ok = WheelTpdoConfigWriteU32(
             &node, 1U, DRIVE_WHEEL_TPDO_COMM_INDEX, 0x01U,
             cob_id | 0x80000000UL) &&
         WheelTpdoConfigWriteU8(
             &node, 2U, DRIVE_WHEEL_TPDO_MAP_INDEX, 0x00U, 0U) &&
         WheelTpdoConfigWriteU32(
             &node, 3U, DRIVE_WHEEL_TPDO_MAP_INDEX, 0x01U,
             DRIVE_WHEEL_TPDO_MAP_ENTRY) &&
         WheelTpdoConfigWriteU8(
             &node, 4U, DRIVE_WHEEL_TPDO_MAP_INDEX, 0x00U,
             DRIVE_WHEEL_TPDO_MAP_COUNT) &&
         WheelTpdoConfigWriteU8(
             &node, 5U, DRIVE_WHEEL_TPDO_COMM_INDEX, 0x02U,
             DRIVE_WHEEL_TPDO_TRANSMISSION_TYPE) &&
         WheelTpdoConfigWriteU16(
             &node, 6U, DRIVE_WHEEL_TPDO_COMM_INDEX, 0x03U,
             DRIVE_WHEEL_TPDO_INHIBIT_TIME) &&
         WheelTpdoConfigWriteU16(
             &node, 7U, DRIVE_WHEEL_TPDO_COMM_INDEX, 0x05U,
             DRIVE_WHEEL_TPDO_EVENT_TIMER_MS) &&
         WheelTpdoConfigWriteU32(
             &node, 8U, DRIVE_WHEEL_TPDO_COMM_INDEX, 0x01U, cob_id) &&
         WheelTpdoConfigReadU32(
             &node, 9U, DRIVE_WHEEL_TPDO_COMM_INDEX, 0x01U, cob_id) &&
         WheelTpdoConfigReadU8(
             &node, 10U, DRIVE_WHEEL_TPDO_COMM_INDEX, 0x02U,
             DRIVE_WHEEL_TPDO_TRANSMISSION_TYPE) &&
         WheelTpdoConfigReadU16(
             &node, 11U, DRIVE_WHEEL_TPDO_COMM_INDEX, 0x03U,
             DRIVE_WHEEL_TPDO_INHIBIT_TIME) &&
         WheelTpdoConfigReadU16(
             &node, 12U, DRIVE_WHEEL_TPDO_COMM_INDEX, 0x05U,
             DRIVE_WHEEL_TPDO_EVENT_TIMER_MS) &&
         WheelTpdoConfigReadU8(
             &node, 13U, DRIVE_WHEEL_TPDO_MAP_INDEX, 0x00U,
             DRIVE_WHEEL_TPDO_MAP_COUNT) &&
         WheelTpdoConfigReadU32(
             &node, 14U, DRIVE_WHEEL_TPDO_MAP_INDEX, 0x01U,
             DRIVE_WHEEL_TPDO_MAP_ENTRY);
    if (!ok) {
        IncrementSaturated(&s_runtime_diag.pdo_config_failure_count);
        s_wheel_feedback_transport = CANOPEN_WHEEL_FEEDBACK_TPDO_FAULT;
        return false;
    }

    WheelTpdoConfigBeginStep(node_id, 15U, 0U, 0U, node_id);
    if (!Canopen_StartRemoteNode(node_id)) {
        s_wheel_tpdo_config_diagnostic.result =
            CANOPEN_TPDO_CONFIG_RESULT_NMT_START_FAILED;
        IncrementSaturated(&s_runtime_diag.pdo_config_failure_count);
        s_wheel_feedback_transport = CANOPEN_WHEEL_FEEDBACK_TPDO_FAULT;
        return false;
    }

    s_wheel_tpdo_configured[(uint8_t)slot] = true;
    s_wheel_tpdo_config_diagnostic.result =
        CANOPEN_TPDO_CONFIG_RESULT_SUCCESS;
    IncrementSaturated(&s_runtime_diag.pdo_config_success_count);
    s_wheel_feedback_transport = CANOPEN_WHEEL_FEEDBACK_TPDO_SHADOW;
    return true;
}

bool Canopen_WheelTpdoConfigured(uint8_t node_id)
{
    int8_t slot = WheelSlotFromNode(node_id);

    return (slot >= 0) && s_wheel_tpdo_configured[(uint8_t)slot];
}

CanopenWheelTpdoConfigDiagnostic_t Canopen_GetWheelTpdoConfigDiagnostic(void)
{
    return s_wheel_tpdo_config_diagnostic;
}

CanopenWheelTargetTransport_t Canopen_GetWheelTargetTransport(void)
{
    return CANOPEN_WHEEL_TARGET_RPDO;
}

CanopenWheelFeedbackTransport_t Canopen_GetWheelFeedbackTransport(void)
{
    return s_wheel_feedback_transport;
}

CanopenWheelFeedbackAuthority_t Canopen_GetWheelFeedbackAuthority(void)
{
    return s_wheel_feedback_authority;
}

const char *Canopen_WheelFeedbackTransportName(
    CanopenWheelFeedbackTransport_t transport)
{
    switch (transport) {
        case CANOPEN_WHEEL_FEEDBACK_SDO_POLL: return "SDO_POLL";
        case CANOPEN_WHEEL_FEEDBACK_TPDO_SHADOW_CONFIGURING:
            return "TPDO_SHADOW_CONFIGURING";
        case CANOPEN_WHEEL_FEEDBACK_TPDO_SHADOW: return "TPDO_SHADOW";
        case CANOPEN_WHEEL_FEEDBACK_TPDO_ACTIVE_ARMING:
            return "TPDO_ACTIVE_ARMING";
        case CANOPEN_WHEEL_FEEDBACK_TPDO_ACTIVE: return "TPDO_ACTIVE";
        case CANOPEN_WHEEL_FEEDBACK_TPDO_FAULT: return "TPDO_FAULT";
        default: return "UNKNOWN";
    }
}

const char *Canopen_WheelFeedbackAuthorityName(
    CanopenWheelFeedbackAuthority_t authority)
{
    switch (authority) {
        case CANOPEN_WHEEL_AUTHORITY_SDO: return "SDO";
        case CANOPEN_WHEEL_AUTHORITY_TPDO: return "TPDO";
        default: return "UNKNOWN";
    }
}

const char *Canopen_WheelTpdoSampleStateName(
    CanopenWheelTpdoSampleState_t state)
{
    switch (state) {
        case CANOPEN_TPDO_SAMPLE_NO_DATA: return "NO_DATA";
        case CANOPEN_TPDO_SAMPLE_FRESH: return "FRESH";
        case CANOPEN_TPDO_SAMPLE_STALE: return "STALE";
        default: return "UNKNOWN";
    }
}

const char *Canopen_WheelTpdoPairStateName(CanopenWheelTpdoPairState_t state)
{
    switch (state) {
        case CANOPEN_TPDO_PAIR_NO_DATA: return "NO_DATA";
        case CANOPEN_TPDO_PAIR_FRESH: return "FRESH";
        case CANOPEN_TPDO_PAIR_STALE: return "STALE";
        case CANOPEN_TPDO_PAIR_FAULT: return "FAULT";
        default: return "UNKNOWN";
    }
}

bool Canopen_GetWheelTpdoPair(uint32_t now_ms,
                              CanopenWheelTpdoPair_t *pair)
{
    if (pair == NULL) {
        return false;
    }
    ServiceWheelTpdoFreshness(now_ms);
    *pair = s_wheel_tpdo_pair;
    return pair->valid;
}

bool Canopen_BeginWheelTpdoActiveArm(uint32_t now_ms)
{
    if ((s_wheel_feedback_transport != CANOPEN_WHEEL_FEEDBACK_TPDO_SHADOW) ||
        (s_wheel_feedback_authority != CANOPEN_WHEEL_AUTHORITY_SDO) ||
        !s_wheel_tpdo_configured[0] || !s_wheel_tpdo_configured[1]) {
        return false;
    }

    s_wheel_tpdo_arm_start_ms = now_ms;
    s_wheel_tpdo_arm_baseline_sequence[0] =
        s_wheel_tpdo_shadow[0].sequence;
    s_wheel_tpdo_arm_baseline_sequence[1] =
        s_wheel_tpdo_shadow[1].sequence;
    ResetWheelTpdoPairEpoch();
    s_wheel_feedback_transport =
        CANOPEN_WHEEL_FEEDBACK_TPDO_ACTIVE_ARMING;
    s_last_wheel_tpdo_active_fault = "NONE";
    return true;
}

void Canopen_AbortWheelTpdoActiveArm(void)
{
    uint8_t slot;

    if (s_wheel_feedback_transport !=
        CANOPEN_WHEEL_FEEDBACK_TPDO_ACTIVE_ARMING) {
        return;
    }
    s_wheel_feedback_transport = CANOPEN_WHEEL_FEEDBACK_TPDO_SHADOW;
    s_wheel_feedback_authority = CANOPEN_WHEEL_AUTHORITY_SDO;
    ResetWheelTpdoPairEpoch();
    for (slot = 0U; slot < 2U; slot++) {
        s_wheel_velocity[slot] = s_wheel_velocity_sdo[slot];
        s_wheel_velocity_from_tpdo[slot] = false;
    }
}

void Canopen_LatchWheelTpdoStartupFault(const char *reason)
{
    uint8_t slot;

    ResetWheelTpdoPairEpoch();
    s_wheel_tpdo_pair.state = CANOPEN_TPDO_PAIR_FAULT;
    s_wheel_feedback_transport = CANOPEN_WHEEL_FEEDBACK_TPDO_FAULT;
    /* Keep the requested production authority explicit even though no valid
     * pair exists. This disables every runtime SDO 0x606C schedule and makes
     * the failure visible instead of silently falling back to SDO. */
    s_wheel_feedback_authority = CANOPEN_WHEEL_AUTHORITY_TPDO;
    s_last_wheel_tpdo_active_fault =
        (reason != NULL) ? reason : "TPDO_STARTUP_FAILED";
    for (slot = 0U; slot < 2U; slot++) {
        s_wheel_tpdo_configured[slot] = false;
        ResetWheelTpdoMeasurementEpoch(slot);
        s_wheel_velocity[slot].valid = false;
        s_wheel_velocity_from_tpdo[slot] = false;
    }
}

bool Canopen_DisableWheelTpdoActive(void)
{
    uint8_t slot;

    if ((s_wheel_feedback_authority != CANOPEN_WHEEL_AUTHORITY_TPDO) ||
        ((s_wheel_feedback_transport != CANOPEN_WHEEL_FEEDBACK_TPDO_ACTIVE) &&
         (s_wheel_feedback_transport != CANOPEN_WHEEL_FEEDBACK_TPDO_FAULT))) {
        return false;
    }
    s_wheel_feedback_transport = CANOPEN_WHEEL_FEEDBACK_TPDO_SHADOW;
    s_wheel_feedback_authority = CANOPEN_WHEEL_AUTHORITY_SDO;
    ResetWheelTpdoPairEpoch();
    for (slot = 0U; slot < 2U; slot++) {
        s_wheel_velocity[slot] = s_wheel_velocity_sdo[slot];
        s_wheel_velocity_from_tpdo[slot] = false;
    }
    return true;
}

bool Canopen_RuntimeSdo606cPollEnabled(void)
{
    return (s_wheel_feedback_authority == CANOPEN_WHEEL_AUTHORITY_SDO) &&
           (s_wheel_feedback_transport !=
            CANOPEN_WHEEL_FEEDBACK_TPDO_ACTIVE_ARMING);
}

const char *Canopen_GetLastWheelTpdoActiveFault(void)
{
    return s_last_wheel_tpdo_active_fault;
}

static bool SendWheelFeedbackRequest(uint8_t slot,
                                     WheelReadObject_t object_kind,
                                     uint32_t now_ms,
                                     CanopenSdoOwner_t owner)
{
    uint8_t node_id = (slot == 0U) ? LEFT_NODE_ID : RIGHT_NODE_ID;
    CanopenNode_t node = CanopenNode_OnFDCAN1(node_id);
    uint8_t index_low = (object_kind == WHEEL_READ_VELOCITY) ? 0x6CU : 0x63U;
    uint8_t data[8] = {0x40U, index_low, 0x60U, 0x00U, 0U, 0U, 0U, 0U};
    uint16_t index = (object_kind == WHEEL_READ_VELOCITY) ? 0x606CU : 0x6063U;
    CanopenSdoOperation_t operation =
        (object_kind == WHEEL_READ_VELOCITY) ?
        CANOPEN_SDO_OP_RUNTIME_VELOCITY : CANOPEN_SDO_OP_RUNTIME_POSITION;

    if (!SdoTransactionStart(node_id, index, 0x00U, operation, owner, now_ms,
                             DRIVE_WHEEL_FEEDBACK_REQUEST_TIMEOUT_MS, 4U)) {
        return false;
    }
    if (Canopen_SendSdoFrame(&node, data, "runtime-read-wheel", false, true) != HAL_OK) {
        SdoTransactionSetIdle(SdoTransactionFromNode(node_id));
        if ((object_kind == WHEEL_READ_VELOCITY) &&
            (owner == CANOPEN_SDO_OWNER_FAULT_MONITOR)) {
            s_feedback_request[slot].velocity_requested = true;
            s_feedback_request[slot].last_velocity_request_ms = now_ms;
        }
        IncrementSaturated(&s_runtime_diag.wheel_feedback_failure_count);
        return false;
    }

    s_feedback_request[slot].pending = true;
    s_feedback_request[slot].fault_monitor_requested =
        (object_kind == WHEEL_READ_VELOCITY) &&
        (owner == CANOPEN_SDO_OWNER_FAULT_MONITOR);
    s_feedback_request[slot].object_kind = object_kind;
    s_feedback_request[slot].sent_ms = now_ms;
    if (object_kind == WHEEL_READ_VELOCITY) {
        s_feedback_request[slot].velocity_requested = true;
        s_feedback_request[slot].last_velocity_request_ms = now_ms;
        IncrementSaturated(&s_runtime_diag.wheel_velocity_request_count);
        IncrementSaturated(&s_runtime_diag.runtime_sdo_606c_poll_count);
    } else {
        s_feedback_request[slot].position_requested = true;
        s_feedback_request[slot].last_position_request_ms = now_ms;
        IncrementSaturated(&s_runtime_diag.wheel_position_request_count);
    }
    IncrementSaturated(&s_runtime_diag.wheel_feedback_request_count);
    return true;
}

static void ServiceEmergencyZeroConfirmation(uint32_t now_ms)
{
    uint8_t slot;

    if ((s_emergency_zero_status !=
         CANOPEN_EMERGENCY_ZERO_ENQUEUED_UNCONFIRMED) ||
        (s_emergency_zero_confirmation_needed_mask == 0U)) {
        return;
    }

    for (slot = 0U; slot < 2U; slot++) {
        uint8_t bit = (uint8_t)(1U << slot);
        uint8_t node_id = (slot == 0U) ? LEFT_NODE_ID : RIGHT_NODE_ID;

        if (((s_emergency_zero_confirmation_needed_mask & bit) == 0U) ||
            !SdoTransactionCanStart(node_id, now_ms)) {
            continue;
        }
        if (!SdoTransactionStart(node_id, 0x60FFU, 0x00U,
                                 CANOPEN_SDO_OP_EMERGENCY_ZERO,
                                 CANOPEN_SDO_OWNER_EMERGENCY_ZERO, now_ms,
                                 DRIVE_RUNTIME_TARGET_TIMEOUT_MS, 0U) ||
            (SendRuntimeTarget(node_id, 0) != HAL_OK)) {
            SdoTransactionSetIdle(SdoTransactionFromNode(node_id));
            s_emergency_zero_confirmation_needed_mask = 0U;
            s_emergency_zero_status = CANOPEN_EMERGENCY_ZERO_FAILED;
            MarkRuntimeTargetFailure(
                node_id, CANOPEN_TARGET_FAILURE_TX);
            return;
        }
        s_emergency_zero_confirmation_needed_mask &= (uint8_t)~bit;
    }

    if (s_emergency_zero_confirmation_needed_mask == 0U) {
        s_emergency_zero_status = CANOPEN_EMERGENCY_ZERO_PENDING_ACK;
    }
}

void Canopen_WheelRuntimeService(uint32_t now_ms,
                                 CanopenWheelFeedbackSchedule_t schedule)
{
    uint8_t slot;

    Canopen_ProcessRx(now_ms, DRIVE_CANOPEN_RX_BUDGET_FRAMES);
    ServiceSdoTransactionTimeouts(now_ms);
    WheelRpdoTx_Service(now_ms);
    if (s_rpdo_zero_wait && WheelRpdoTx_Quiescent()) {
        s_rpdo_zero_wait = false;
        (void)Canopen_EmergencyZeroWheelTargets();
    }
    ServiceEmergencyZeroConfirmation(now_ms);

    if (schedule == CANOPEN_FEEDBACK_DISPATCH_ONLY) {
        return;
    }

    for (slot = 0U; slot < 2U; slot++) {
        uint32_t velocity_poll_ms =
            (schedule == CANOPEN_FEEDBACK_STOP_CONFIRM_VELOCITY_ONLY) ?
            DRIVE_STOP_FEEDBACK_POLL_MS :
            ((schedule == CANOPEN_FEEDBACK_FAULT_MONITOR_VELOCITY_ONLY) ?
             DRIVE_FAULT_FEEDBACK_POLL_MS : DRIVE_WHEEL_FEEDBACK_POLL_MS);
        bool runtime_velocity_suppressed =
            !Canopen_RuntimeSdo606cPollEnabled();
        bool velocity_due =
            !runtime_velocity_suppressed &&
            (!s_feedback_request[slot].velocity_requested ||
             ((uint32_t)(now_ms -
                         s_feedback_request[slot].last_velocity_request_ms) >=
              velocity_poll_ms));
        bool position_due =
            (s_wheel_feedback_authority == CANOPEN_WHEEL_AUTHORITY_SDO) &&
            (schedule == CANOPEN_FEEDBACK_NORMAL_MOTION) &&
            (!s_feedback_request[slot].position_requested ||
             ((uint32_t)(now_ms -
                         s_feedback_request[slot].last_position_request_ms) >=
              DRIVE_WHEEL_POSITION_POLL_MS));

        uint8_t node_id = (slot == 0U) ? LEFT_NODE_ID : RIGHT_NODE_ID;

        if (SdoTransactionCanStart(node_id, now_ms)) {
            if (velocity_due) {
                CanopenSdoOwner_t owner =
                    (schedule ==
                     CANOPEN_FEEDBACK_STOP_CONFIRM_VELOCITY_ONLY) ?
                    CANOPEN_SDO_OWNER_STOP_FEEDBACK :
                    ((schedule ==
                      CANOPEN_FEEDBACK_FAULT_MONITOR_VELOCITY_ONLY) ?
                     CANOPEN_SDO_OWNER_FAULT_MONITOR :
                     CANOPEN_SDO_OWNER_NORMAL_FEEDBACK);

                if (SendWheelFeedbackRequest(
                        slot, WHEEL_READ_VELOCITY, now_ms, owner) &&
                    (schedule ==
                     CANOPEN_FEEDBACK_STOP_CONFIRM_VELOCITY_ONLY)) {
                    IncrementSaturated(
                        &s_runtime_diag.stop_velocity_request_count);
                } else if (s_feedback_request[slot].pending &&
                           (schedule ==
                            CANOPEN_FEEDBACK_FAULT_MONITOR_VELOCITY_ONLY)) {
                    IncrementSaturated(
                        &s_runtime_diag.fault_velocity_request_count);
                }
            } else if (position_due) {
                (void)SendWheelFeedbackRequest(
                    slot, WHEEL_READ_POSITION, now_ms,
                    CANOPEN_SDO_OWNER_NORMAL_FEEDBACK);
            }
        }
    }
}

bool Canopen_GetWheelVelocitySample(uint8_t node_id,
                                    CanopenValueSample_t *sample)
{
    int8_t slot = WheelSlotFromNode(node_id);

    if ((slot < 0) || (sample == NULL)) {
        return false;
    }

    *sample = s_wheel_velocity[(uint8_t)slot];
    return sample->valid;
}

bool Canopen_GetWheelSdoVelocitySample(uint8_t node_id,
                                       CanopenValueSample_t *sample)
{
    int8_t slot = WheelSlotFromNode(node_id);

    if ((slot < 0) || (sample == NULL)) {
        return false;
    }
    *sample = s_wheel_velocity_sdo[(uint8_t)slot];
    return sample->valid;
}

bool Canopen_GetWheelTpdoShadowSample(uint8_t node_id,
                                      uint32_t now_ms,
                                      CanopenWheelTpdoSample_t *sample)
{
    int8_t slot = WheelSlotFromNode(node_id);

    if ((slot < 0) || (sample == NULL)) {
        return false;
    }
    ServiceWheelTpdoFreshness(now_ms);
    *sample = s_wheel_tpdo_shadow[(uint8_t)slot];
    return sample->valid;
}

bool Canopen_GetWheelPositionSample(uint8_t node_id,
                                    CanopenValueSample_t *sample)
{
    int8_t slot = WheelSlotFromNode(node_id);

    if ((slot < 0) || (sample == NULL)) {
        return false;
    }

    *sample = s_wheel_position[(uint8_t)slot];
    return sample->valid;
}

static HAL_StatusTypeDef SendRuntimeTarget(uint8_t node_id, int32_t value)
{
    CanopenNode_t node = CanopenNode_OnFDCAN1(node_id);
    uint32_t raw = (uint32_t)value;
    uint8_t data[8] = {
        0x23U, 0xFFU, 0x60U, 0x00U,
        (uint8_t)(raw & 0xFFU),
        (uint8_t)((raw >> 8) & 0xFFU),
        (uint8_t)((raw >> 16) & 0xFFU),
        (uint8_t)((raw >> 24) & 0xFFU)
    };

    return Canopen_SendSdoFrame(&node, data, "runtime-write-60FF", false, true);
}

CanopenRuntimeSubmitResult_t Canopen_SubmitWheelTargetPair(
    int32_t left_velocity_dec,
    int32_t right_velocity_dec,
    uint32_t now_ms)
{
    if (!SdoTransactionCanStart(LEFT_NODE_ID, now_ms) ||
        !SdoTransactionCanStart(RIGHT_NODE_ID, now_ms)) {
        return CANOPEN_RUNTIME_SUBMIT_BUSY;
    }

    s_runtime_fault_pending = false;
    if (!SdoTransactionStart(LEFT_NODE_ID, 0x60FFU, 0x00U,
                             CANOPEN_SDO_OP_RUNTIME_TARGET,
                             CANOPEN_SDO_OWNER_RUNTIME_TARGET, now_ms,
                             DRIVE_RUNTIME_TARGET_TIMEOUT_MS, 0U)) {
        return CANOPEN_RUNTIME_SUBMIT_BUSY;
    }
    if (SendRuntimeTarget(LEFT_NODE_ID, left_velocity_dec) != HAL_OK) {
        SdoTransactionSetIdle(SdoTransactionFromNode(LEFT_NODE_ID));
        MarkRuntimeTargetFailure(
            LEFT_NODE_ID, CANOPEN_TARGET_FAILURE_TX);
        return CANOPEN_RUNTIME_SUBMIT_ERROR;
    }
    s_target_request[0].pending = true;
    s_target_request[0].sent_ms = now_ms;

    if (!SdoTransactionStart(RIGHT_NODE_ID, 0x60FFU, 0x00U,
                             CANOPEN_SDO_OP_RUNTIME_TARGET,
                             CANOPEN_SDO_OWNER_RUNTIME_TARGET, now_ms,
                             DRIVE_RUNTIME_TARGET_TIMEOUT_MS, 0U)) {
        MarkRuntimeTargetFailure(
            RIGHT_NODE_ID, CANOPEN_TARGET_FAILURE_PREEMPT);
        (void)Canopen_EmergencyZeroWheelTargets();
        return CANOPEN_RUNTIME_SUBMIT_ERROR;
    }
    if (SendRuntimeTarget(RIGHT_NODE_ID, right_velocity_dec) != HAL_OK) {
        SdoTransactionSetIdle(SdoTransactionFromNode(RIGHT_NODE_ID));
        MarkRuntimeTargetFailure(
            RIGHT_NODE_ID, CANOPEN_TARGET_FAILURE_TX);
        (void)Canopen_EmergencyZeroWheelTargets();
        return CANOPEN_RUNTIME_SUBMIT_ERROR;
    }
    s_target_request[1].pending = true;
    s_target_request[1].sent_ms = now_ms;
    IncrementSaturated(&s_runtime_diag.wheel_target_submit_count);
    return CANOPEN_RUNTIME_SUBMIT_OK;
}

static bool PrepareEmergencyZeroTransaction(uint8_t node_id,
                                            uint32_t now_ms,
                                            bool *confirmation_tracked)
{
    CanopenSdoTransaction_t *transaction =
        SdoTransactionFromNode(node_id);
    int8_t wheel_slot = WheelSlotFromNode(node_id);

    if ((transaction == NULL) || (wheel_slot < 0) ||
        (confirmation_tracked == NULL)) {
        return false;
    }
    SdoTransactionRefreshQuarantine(transaction, now_ms);
    *confirmation_tracked = false;
    if ((transaction->state == CANOPEN_SDO_TX_ACTIVE) ||
        (transaction->state == CANOPEN_SDO_TX_COMPLETED)) {
        uint8_t slot = (uint8_t)wheel_slot;

        if ((transaction->operation == CANOPEN_SDO_OP_RUNTIME_VELOCITY) ||
            (transaction->operation == CANOPEN_SDO_OP_RUNTIME_POSITION)) {
            s_feedback_request[slot].pending = false;
            s_feedback_request[slot].fault_monitor_requested = false;
            IncrementSaturated(
                &s_runtime_diag.safety_request_preemption_count);
        } else if (transaction->operation ==
                   CANOPEN_SDO_OP_RUNTIME_TARGET) {
            s_target_request[slot].pending = false;
            IncrementSaturated(&s_runtime_diag.wheel_target_failure_count);
            IncrementSaturated(
                &s_runtime_diag.safety_request_preemption_count);
        }
        SdoTransactionEnterQuarantine(transaction, now_ms);
        return true;
    }
    if (transaction->state == CANOPEN_SDO_TX_QUARANTINE) {
        return true;
    }
    if (!SdoTransactionStart(node_id, 0x60FFU, 0x00U,
                             CANOPEN_SDO_OP_EMERGENCY_ZERO,
                             CANOPEN_SDO_OWNER_EMERGENCY_ZERO, now_ms,
                             DRIVE_RUNTIME_TARGET_TIMEOUT_MS, 0U)) {
        return false;
    }
    *confirmation_tracked = true;
    return true;
}

CanopenEmergencyZeroStatus_t Canopen_EmergencyZeroWheelTargets(void)
{
    bool left_ok;
    bool right_ok;
    bool left_tracked = false;
    bool right_tracked = false;
    uint32_t now_ms = HAL_GetTick();

    WheelRpdoTx_Cancel(now_ms);
    WheelRpdoTx_Service(now_ms);
    if (!WheelRpdoTx_Quiescent()) {
        s_rpdo_zero_wait = true;
        s_emergency_zero_status = CANOPEN_EMERGENCY_ZERO_PENDING_ACK;
        return s_emergency_zero_status;
    }
    s_rpdo_zero_wait = false;

    s_feedback_request[0].pending = false;
    s_feedback_request[1].pending = false;
    s_target_request[0].pending = false;
    s_target_request[1].pending = false;
    s_emergency_zero_ack_mask = 0U;
    s_emergency_zero_confirmation_needed_mask = 0U;
    s_emergency_zero_status = CANOPEN_EMERGENCY_ZERO_FAILED;
    if (!PrepareEmergencyZeroTransaction(
            LEFT_NODE_ID, now_ms, &left_tracked)) {
        MarkRuntimeTargetFailure(
            LEFT_NODE_ID, CANOPEN_TARGET_FAILURE_PREEMPT);
        return s_emergency_zero_status;
    }
    if (!PrepareEmergencyZeroTransaction(
            RIGHT_NODE_ID, now_ms, &right_tracked)) {
        MarkRuntimeTargetFailure(
            RIGHT_NODE_ID, CANOPEN_TARGET_FAILURE_PREEMPT);
        return s_emergency_zero_status;
    }

    /* Queue zero after any already-transmitted request. CAN preserves order, so
     * the final requested value at each node is zero even if an older command
     * is in flight. No response wait is allowed in the STOP path. */
    left_ok = (SendRuntimeTarget(LEFT_NODE_ID, 0) == HAL_OK);
    right_ok = (SendRuntimeTarget(RIGHT_NODE_ID, 0) == HAL_OK);
    IncrementSaturated(&s_runtime_diag.emergency_zero_count);
    if (!left_ok || !right_ok) {
        if (!left_ok && left_tracked) {
            SdoTransactionSetIdle(SdoTransactionFromNode(LEFT_NODE_ID));
        }
        if (!right_ok && right_tracked) {
            SdoTransactionSetIdle(SdoTransactionFromNode(RIGHT_NODE_ID));
        }
        MarkRuntimeTargetFailure(
            !left_ok ? LEFT_NODE_ID : RIGHT_NODE_ID,
            CANOPEN_TARGET_FAILURE_TX);
        s_emergency_zero_status = CANOPEN_EMERGENCY_ZERO_FAILED;
    } else if (left_tracked && right_tracked) {
        s_emergency_zero_status = CANOPEN_EMERGENCY_ZERO_PENDING_ACK;
    } else {
        s_emergency_zero_confirmation_needed_mask =
            (uint8_t)((left_tracked ? 0U : 0x01U) |
                      (right_tracked ? 0U : 0x02U));
        s_emergency_zero_status =
            CANOPEN_EMERGENCY_ZERO_ENQUEUED_UNCONFIRMED;
    }
    return s_emergency_zero_status;
}

CanopenEmergencyZeroStatus_t Canopen_GetEmergencyZeroStatus(void)
{
    ServiceSdoTransactionTimeouts(HAL_GetTick());
    return s_emergency_zero_status;
}

static CanopenAsyncStartResult_t Canopen_AsyncBegin(
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    CanopenSdoOperation_t operation,
    uint8_t expected_upload_size,
    CanopenSdoOwner_t owner,
    uint32_t now_ms,
    uint32_t timeout_ms,
    const uint8_t data[8],
    const char *label)
{
    CanopenSdoTransaction_t *transaction =
        SdoTransactionFromNode(node_id);
    CanopenNode_t node = CanopenNode_OnFDCAN1(node_id);
    int8_t wheel_slot = WheelSlotFromNode(node_id);

    ServiceSdoTransactionTimeouts(now_ms);
    SdoTransactionRefreshQuarantine(transaction, now_ms);
    if (transaction == NULL) {
        return CANOPEN_ASYNC_START_ERROR;
    }
    if ((transaction->state == CANOPEN_SDO_TX_ACTIVE) &&
        (transaction->owner == CANOPEN_SDO_OWNER_FAULT_MONITOR) &&
        (owner >= CANOPEN_SDO_OWNER_SAFETY_ENABLE)) {
        if (wheel_slot >= 0) {
            uint8_t slot = (uint8_t)wheel_slot;

            s_feedback_request[slot].pending = false;
            s_feedback_request[slot].fault_monitor_requested = false;
        }
        IncrementSaturated(&s_runtime_diag.safety_request_preemption_count);
        SdoTransactionEnterQuarantine(transaction, now_ms);
        return CANOPEN_ASYNC_START_BUSY;
    }
    if (transaction->state != CANOPEN_SDO_TX_IDLE) {
        return CANOPEN_ASYNC_START_BUSY;
    }
    if (!SdoTransactionStart(node_id, index, subindex, operation, owner,
                             now_ms, timeout_ms, expected_upload_size)) {
        return CANOPEN_ASYNC_START_BUSY;
    }
    if (Canopen_SendSdoFrame(&node, data, label, false, true) != HAL_OK) {
        SdoTransactionSetIdle(transaction);
        return CANOPEN_ASYNC_START_ERROR;
    }
    return CANOPEN_ASYNC_START_OK;
}

CanopenAsyncStartResult_t Canopen_AsyncWriteU8(
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    uint8_t value,
    CanopenSdoOwner_t owner,
    uint32_t now_ms,
    uint32_t timeout_ms)
{
    uint8_t data[8] = {
        0x2FU, (uint8_t)(index & 0xFFU),
        (uint8_t)((index >> 8) & 0xFFU), subindex,
        value, 0U, 0U, 0U
    };

    return Canopen_AsyncBegin(node_id, index, subindex,
                              CANOPEN_SDO_OP_ASYNC_WRITE, 0U, owner,
                              now_ms, timeout_ms, data,
                              "async-write-u8");
}

CanopenAsyncStartResult_t Canopen_AsyncWriteU16(
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    uint16_t value,
    CanopenSdoOwner_t owner,
    uint32_t now_ms,
    uint32_t timeout_ms)
{
    uint8_t data[8] = {
        0x2BU, (uint8_t)(index & 0xFFU),
        (uint8_t)((index >> 8) & 0xFFU), subindex,
        (uint8_t)(value & 0xFFU), (uint8_t)((value >> 8) & 0xFFU),
        0U, 0U
    };

    return Canopen_AsyncBegin(node_id, index, subindex,
                              CANOPEN_SDO_OP_ASYNC_WRITE, 0U, owner,
                              now_ms, timeout_ms, data,
                              "async-write-u16");
}

CanopenAsyncStartResult_t Canopen_AsyncWriteI8(
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    int8_t value,
    CanopenSdoOwner_t owner,
    uint32_t now_ms,
    uint32_t timeout_ms)
{
    uint8_t data[8] = {
        0x2FU, (uint8_t)(index & 0xFFU),
        (uint8_t)((index >> 8) & 0xFFU), subindex,
        (uint8_t)value, 0U, 0U, 0U
    };

    return Canopen_AsyncBegin(node_id, index, subindex,
                              CANOPEN_SDO_OP_ASYNC_WRITE, 0U, owner,
                              now_ms, timeout_ms, data,
                              "async-write-i8");
}

CanopenAsyncStartResult_t Canopen_AsyncWriteI32(
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    int32_t value,
    CanopenSdoOwner_t owner,
    uint32_t now_ms,
    uint32_t timeout_ms)
{
    uint32_t raw = (uint32_t)value;
    uint8_t data[8] = {
        0x23U, (uint8_t)(index & 0xFFU),
        (uint8_t)((index >> 8) & 0xFFU), subindex,
        (uint8_t)(raw & 0xFFU), (uint8_t)((raw >> 8) & 0xFFU),
        (uint8_t)((raw >> 16) & 0xFFU),
        (uint8_t)((raw >> 24) & 0xFFU)
    };

    return Canopen_AsyncBegin(node_id, index, subindex,
                              CANOPEN_SDO_OP_ASYNC_WRITE, 0U, owner,
                              now_ms, timeout_ms, data,
                              "async-write-i32");
}

CanopenAsyncStartResult_t Canopen_AsyncWriteU32(
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    uint32_t value,
    CanopenSdoOwner_t owner,
    uint32_t now_ms,
    uint32_t timeout_ms)
{
    uint8_t data[8] = {
        0x23U, (uint8_t)(index & 0xFFU),
        (uint8_t)((index >> 8) & 0xFFU), subindex,
        (uint8_t)(value & 0xFFU), (uint8_t)((value >> 8) & 0xFFU),
        (uint8_t)((value >> 16) & 0xFFU),
        (uint8_t)((value >> 24) & 0xFFU)
    };

    return Canopen_AsyncBegin(node_id, index, subindex,
                              CANOPEN_SDO_OP_ASYNC_WRITE, 0U, owner,
                              now_ms, timeout_ms, data,
                              "async-write-u32");
}

static CanopenAsyncStartResult_t Canopen_AsyncRead(
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    uint8_t expected_upload_size,
    CanopenSdoOwner_t owner,
    uint32_t now_ms,
    uint32_t timeout_ms,
    const char *label)
{
    uint8_t data[8] = {
        0x40U, (uint8_t)(index & 0xFFU),
        (uint8_t)((index >> 8) & 0xFFU), subindex,
        0U, 0U, 0U, 0U
    };

    return Canopen_AsyncBegin(node_id, index, subindex,
                              CANOPEN_SDO_OP_ASYNC_READ, expected_upload_size,
                              owner,
                              now_ms, timeout_ms, data, label);
}

CanopenAsyncStartResult_t Canopen_AsyncReadU16(
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    CanopenSdoOwner_t owner,
    uint32_t now_ms,
    uint32_t timeout_ms)
{
    return Canopen_AsyncRead(node_id, index, subindex, 2U, owner,
                             now_ms, timeout_ms, "async-read-u16");
}

CanopenAsyncStartResult_t Canopen_AsyncReadU8(
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    CanopenSdoOwner_t owner,
    uint32_t now_ms,
    uint32_t timeout_ms)
{
    return Canopen_AsyncRead(node_id, index, subindex, 1U, owner,
                             now_ms, timeout_ms, "async-read-u8");
}

CanopenAsyncStartResult_t Canopen_AsyncReadU32(
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    CanopenSdoOwner_t owner,
    uint32_t now_ms,
    uint32_t timeout_ms)
{
    return Canopen_AsyncRead(node_id, index, subindex, 4U, owner,
                             now_ms, timeout_ms, "async-read-u32");
}

CanopenAsyncStartResult_t Canopen_AsyncReadI8(
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    CanopenSdoOwner_t owner,
    uint32_t now_ms,
    uint32_t timeout_ms)
{
    return Canopen_AsyncRead(node_id, index, subindex, 1U, owner,
                             now_ms, timeout_ms, "async-read-i8");
}

CanopenAsyncResult_t Canopen_AsyncPoll(
    uint8_t node_id,
    CanopenSdoOwner_t owner,
    uint32_t now_ms,
    uint32_t *value,
    uint32_t *abort_code)
{
    return Canopen_AsyncPollWithTimestamp(node_id, owner, now_ms, value,
                                         abort_code, NULL);
}

CanopenAsyncResult_t Canopen_AsyncPollWithTimestamp(
    uint8_t node_id,
    CanopenSdoOwner_t owner,
    uint32_t now_ms,
    uint32_t *value,
    uint32_t *abort_code,
    uint32_t *rx_ms_out)
{
    CanopenSdoTransaction_t *transaction =
        SdoTransactionFromNode(node_id);
    CanopenAsyncResult_t result;

    ServiceSdoTransactionTimeouts(now_ms);
    if (transaction == NULL) {
        return CANOPEN_ASYNC_RESULT_PROTOCOL_ERROR;
    }
    if (transaction->state == CANOPEN_SDO_TX_IDLE) {
        return CANOPEN_ASYNC_RESULT_IDLE;
    }
    if (transaction->owner != owner) {
        return CANOPEN_ASYNC_RESULT_OWNER_MISMATCH;
    }
    if (transaction->state == CANOPEN_SDO_TX_ACTIVE) {
        return CANOPEN_ASYNC_RESULT_PENDING;
    }
    if (transaction->state == CANOPEN_SDO_TX_QUARANTINE) {
        if ((transaction->async_result == CANOPEN_ASYNC_RESULT_TIMEOUT) &&
            !transaction->result_reported) {
            transaction->result_reported = true;
            return CANOPEN_ASYNC_RESULT_TIMEOUT;
        }
        return CANOPEN_ASYNC_RESULT_PENDING;
    }
    if (transaction->state != CANOPEN_SDO_TX_COMPLETED) {
        return CANOPEN_ASYNC_RESULT_PROTOCOL_ERROR;
    }

    result = transaction->async_result;
    if (value != NULL) {
        *value = transaction->completion_value;
    }
    if (abort_code != NULL) {
        *abort_code = transaction->abort_code;
    }
    if (rx_ms_out != NULL) {
        *rx_ms_out = transaction->completion_rx_ms;
    }
    SdoTransactionSetIdle(transaction);
    return result;
}

void Canopen_CancelAsyncOwner(CanopenSdoOwner_t owner, uint32_t now_ms)
{
    uint8_t node_id;

    for (node_id = 1U; node_id < 4U; node_id++) {
        CanopenSdoTransaction_t *transaction =
            SdoTransactionFromNode(node_id);

        if ((transaction == NULL) || (transaction->owner != owner) ||
            ((transaction->state != CANOPEN_SDO_TX_ACTIVE) &&
             (transaction->state != CANOPEN_SDO_TX_COMPLETED))) {
            continue;
        }
        IncrementSaturated(&s_runtime_diag.safety_request_preemption_count);
        SdoTransactionEnterQuarantine(transaction, now_ms);
    }
}

void Canopen_BootSdoBegin(void)
{
    if (s_boot_sdo_observing) {
        return;
    }
    memset(&s_boot_sdo_evidence, 0, sizeof(s_boot_sdo_evidence));
    s_boot_sdo_fenced = false;
    s_boot_sdo_observing = true;
}

static void BootSdoRetireWheelRequests(void)
{
    uint8_t node_id;

    for (node_id = LEFT_NODE_ID; node_id <= RIGHT_NODE_ID; node_id++) {
        uint8_t slot = (uint8_t)WheelSlotFromNode(node_id);

        /* Also retires timed-out requests already in QUARANTINE. Preserve
         * runtime hard-fault flags, heartbeat and PDO authority/history. */
        SdoTransactionSetIdle(SdoTransactionFromNode(node_id));
        s_feedback_request[slot].pending = false;
        s_feedback_request[slot].fault_monitor_requested = false;
        s_target_request[slot].pending = false;
        s_wheel_velocity[slot].valid = false;
        s_wheel_velocity_sdo[slot].valid = false;
        s_wheel_position[slot].valid = false;
    }
    if (WheelSlotFromNode(s_last_sdo_response.node_id) >= 0) {
        s_last_sdo_response.valid = false;
    }
}

void Canopen_BootSdoFence(uint32_t now_ms)
{
    (void)now_ms;
    if (!s_boot_sdo_observing) {
        return;
    }
    s_boot_sdo_fenced = true;
    BootSdoRetireWheelRequests();
}

bool Canopen_BootSdoResume(uint32_t now_ms)
{
    uint32_t queued;

    if (!s_boot_sdo_observing || !s_boot_sdo_fenced) {
        return true;
    }
    /* Drain the current FIFO snapshot through the normal dispatcher while
     * the wheel-SDO fence still applies. This is bounded by the FIFO capacity;
     * Lift/EMCY/heartbeat/PDO frames retain their normal handling. This drain
     * is not a proof of freshness: a same-object response arriving after the
     * new request cannot be distinguished from its reply by standard SDO. */
    queued = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0);
    Canopen_ProcessRx(now_ms, queued);
    BootSdoRetireWheelRequests();
    /* Frames can arrive while that snapshot is dispatched. Keep the fence
     * until a later bounded pass consumes them before admitting attempt 2. */
    if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) != 0U) {
        return false;
    }
    s_boot_sdo_fenced = false;
    return true;
}

void Canopen_BootSdoEnd(void)
{
    /* Hard-fault cleanup may need the ordinary wheel safety SDO channel. */
    s_boot_sdo_fenced = false;
    s_boot_sdo_observing = false;
}

CanopenBootSdoEvidence_t Canopen_GetBootSdoEvidence(void)
{
    return s_boot_sdo_evidence;
}

void Canopen_RecordFaultMonitorDeferred(uint32_t now_ms)
{
    if (!s_fault_monitor_defer_recorded ||
        ((uint32_t)(now_ms - s_last_fault_monitor_defer_ms) >=
         DRIVE_FAULT_FEEDBACK_POLL_MS)) {
        s_fault_monitor_defer_recorded = true;
        s_last_fault_monitor_defer_ms = now_ms;
        IncrementSaturated(&s_runtime_diag.fault_monitor_deferred_count);
    }
}

bool Canopen_TakeWheelRuntimeFault(void)
{
    bool pending = s_runtime_fault_pending;
    s_runtime_fault_pending = false;
    return pending;
}

bool Canopen_TakeWheelFeedbackContinuityFault(void)
{
    bool pending = s_feedback_continuity_fault_pending;
    s_feedback_continuity_fault_pending = false;
    return pending;
}

bool Canopen_WheelRuntimeFaultPending(void)
{
    return s_runtime_fault_pending;
}

bool Canopen_WheelFeedbackContinuityFaultPending(void)
{
    return s_feedback_continuity_fault_pending;
}

void Canopen_HeartbeatResetForExplicitEnable(uint32_t now_ms)
{
    uint8_t node_id;

    (void)now_ms;
    for (node_id = LEFT_NODE_ID; node_id <= RIGHT_NODE_ID; node_id++) {
        memset(&s_heartbeat[node_id], 0, sizeof(s_heartbeat[node_id]));
        s_runtime_diag.heartbeat_configured[node_id] = 0U;
        s_runtime_diag.heartbeat_qualification_active[node_id] = 0U;
        s_runtime_diag.heartbeat_qualified[node_id] = 0U;
        s_runtime_diag.heartbeat_baseline_sequence[node_id] =
            s_runtime_diag.heartbeat_sequence[node_id];
        s_runtime_diag.heartbeat_qualification_count[node_id] = 0U;
        s_runtime_diag.heartbeat_fault_reason[node_id] =
            CANOPEN_HEARTBEAT_FAULT_NONE;
    }
    s_heartbeat_fault_pending = false;
    s_heartbeat_fault_node = 0U;
    s_heartbeat_fault_reason = CANOPEN_HEARTBEAT_FAULT_NONE;
    s_master_heartbeat.fault_latched = false;
    s_runtime_diag.master_heartbeat_fault_latched = 0U;
}

bool Canopen_HeartbeatSetConfigured(uint8_t node_id, uint32_t now_ms)
{
    CanopenHeartbeatState_t *heartbeat;

    (void)now_ms;
    if ((node_id != LEFT_NODE_ID) && (node_id != RIGHT_NODE_ID)) {
        return false;
    }
    heartbeat = &s_heartbeat[node_id];
    if (heartbeat->fault_latched || s_master_heartbeat.fault_latched) {
        return false;
    }
    heartbeat->configured = true;
    heartbeat->qualification_active = true;
    heartbeat->qualified = false;
    heartbeat->baseline_sequence =
        s_runtime_diag.heartbeat_sequence[node_id];
    heartbeat->fresh_valid_count = 0U;
    s_runtime_diag.heartbeat_configured[node_id] = 1U;
    s_runtime_diag.heartbeat_qualification_active[node_id] = 1U;
    s_runtime_diag.heartbeat_qualified[node_id] = 0U;
    s_runtime_diag.heartbeat_baseline_sequence[node_id] =
        heartbeat->baseline_sequence;
    s_runtime_diag.heartbeat_qualification_count[node_id] = 0U;
    s_runtime_diag.heartbeat_fault_reason[node_id] =
        CANOPEN_HEARTBEAT_FAULT_NONE;
    return true;
}

void Canopen_HeartbeatService(uint32_t now_ms)
{
    uint8_t node_id;

    CanTrace_RuntimeBegin(CAN_TRACE_RUNTIME_HEARTBEAT, HAL_GetTick());
    if (!s_master_heartbeat.started) {
        s_master_heartbeat.started = true;
        s_master_heartbeat.next_tx_ms = now_ms;
        s_runtime_diag.master_heartbeat_started = 1U;
    }
    if ((int32_t)(now_ms - s_master_heartbeat.next_tx_ms) >= 0) {
        uint32_t late_ms = (uint32_t)(
            now_ms - s_master_heartbeat.next_tx_ms);
        uint32_t periods =
            (late_ms / DRIVE_MASTER_HEARTBEAT_PERIOD_MS) + 1U;

        /* Advance past every missed slot, but enqueue only this one frame. */
        s_master_heartbeat.next_tx_ms +=
            periods * DRIVE_MASTER_HEARTBEAT_PERIOD_MS;
        IncrementSaturated(
            &s_runtime_diag.master_heartbeat_tx_attempt_count);
        if (Canopen_SendMasterHeartbeat() == HAL_OK) {
            IncrementSaturated(
                &s_runtime_diag.master_heartbeat_tx_success_count);
            s_master_heartbeat.consecutive_failures = 0U;
            s_runtime_diag.master_heartbeat_consecutive_failures = 0U;
            s_runtime_diag.master_heartbeat_last_success_ms = now_ms;
        } else {
            IncrementSaturated(
                &s_runtime_diag.master_heartbeat_tx_failure_count);
            if (s_master_heartbeat.consecutive_failures != UINT32_MAX) {
                s_master_heartbeat.consecutive_failures++;
            }
            s_runtime_diag.master_heartbeat_consecutive_failures =
                s_master_heartbeat.consecutive_failures;
            if (s_master_heartbeat.consecutive_failures >=
                DRIVE_MASTER_HEARTBEAT_TX_FAILURE_LIMIT) {
                HeartbeatLatchFault(
                    0U, CANOPEN_HEARTBEAT_FAULT_MASTER_TX);
            }
        }
    }

    for (node_id = LEFT_NODE_ID; node_id <= RIGHT_NODE_ID; node_id++) {
        CanopenHeartbeatState_t *heartbeat = &s_heartbeat[node_id];

        if (!heartbeat->configured || !heartbeat->qualified ||
            heartbeat->fault_latched) {
            continue;
        }
        if ((uint32_t)(now_ms - heartbeat->last_heartbeat_ms) >
            DRIVE_HEARTBEAT_STALE_TIMEOUT_MS) {
            IncrementSaturated(
                &s_runtime_diag.heartbeat_stale_count[node_id]);
            HeartbeatLatchFault(
                node_id, CANOPEN_HEARTBEAT_FAULT_STALE);
        }
    }
    CanTrace_RuntimeEnd(CAN_TRACE_RUNTIME_HEARTBEAT, HAL_GetTick());
}

bool Canopen_HeartbeatIsQualified(uint8_t node_id)
{
    if ((node_id != LEFT_NODE_ID) && (node_id != RIGHT_NODE_ID)) {
        return false;
    }
    return s_heartbeat[node_id].configured &&
           s_heartbeat[node_id].qualified &&
           !s_heartbeat[node_id].fault_latched;
}

bool Canopen_TakeHeartbeatFault(uint8_t *node_id,
                                CanopenHeartbeatFaultReason_t *reason)
{
    if (!s_heartbeat_fault_pending) {
        return false;
    }
    if (node_id != NULL) {
        *node_id = s_heartbeat_fault_node;
    }
    if (reason != NULL) {
        *reason = s_heartbeat_fault_reason;
    }
    s_heartbeat_fault_pending = false;
    return true;
}

bool Canopen_HeartbeatFaultPending(void)
{
    return s_heartbeat_fault_pending ||
           s_master_heartbeat.fault_latched ||
           s_heartbeat[LEFT_NODE_ID].fault_latched ||
           s_heartbeat[RIGHT_NODE_ID].fault_latched;
}

bool Canopen_TakeLiftFaultEmcy(uint16_t *error_code)
{
    if (!s_lift_fault_emcy_pending) {
        return false;
    }
    if (error_code != NULL) {
        *error_code = s_lift_fault_emcy_code;
    }
    s_lift_fault_emcy_pending = false;
    return true;
}

CanopenRuntimeDiagnostics_t Canopen_GetRuntimeDiagnostics(void)
{
    CanopenRuntimeDiagnostics_t result = s_runtime_diag;

    result.heartbeat_fault_node = s_heartbeat_fault_node;
    result.heartbeat_pending_reason = (uint8_t)s_heartbeat_fault_reason;
    result.heartbeat_fault_pending =
        Canopen_HeartbeatFaultPending() ? 1U : 0U;
    result.feedback_pending_mask =
        (s_feedback_request[0].pending ? 0x01U : 0U) |
        (s_feedback_request[1].pending ? 0x02U : 0U);
    result.target_pending_mask =
        (s_target_request[0].pending ? 0x01U : 0U) |
        (s_target_request[1].pending ? 0x02U : 0U);
    {
        uint8_t node_id;

        for (node_id = 1U; node_id < 4U; node_id++) {
            result.sdo_state[node_id] =
                (uint8_t)s_sdo_transaction[node_id].state;
            result.sdo_operation[node_id] =
                (uint8_t)s_sdo_transaction[node_id].operation;
            result.sdo_owner[node_id] =
                (uint8_t)s_sdo_transaction[node_id].owner;
        }
    }
    return result;
}

#ifdef AMR_HOST_TEST
void Canopen_TestResetState(void)
{
    s_boot_sdo_observing = false;
    s_boot_sdo_fenced = false;
    memset(&s_boot_sdo_evidence, 0, sizeof(s_boot_sdo_evidence));
    memset(&s_last_sdo_response, 0, sizeof(s_last_sdo_response));
    memset(s_feedback_request, 0, sizeof(s_feedback_request));
    memset(s_target_request, 0, sizeof(s_target_request));
    memset(s_wheel_velocity, 0, sizeof(s_wheel_velocity));
    memset(s_wheel_velocity_sdo, 0, sizeof(s_wheel_velocity_sdo));
    memset(s_wheel_position, 0, sizeof(s_wheel_position));
    memset(s_wheel_tpdo_shadow, 0, sizeof(s_wheel_tpdo_shadow));
    memset(&s_wheel_tpdo_pair, 0, sizeof(s_wheel_tpdo_pair));
    memset(s_wheel_tpdo_committed_node_sequence, 0,
           sizeof(s_wheel_tpdo_committed_node_sequence));
    memset(s_wheel_tpdo_arm_baseline_sequence, 0,
           sizeof(s_wheel_tpdo_arm_baseline_sequence));
    s_wheel_tpdo_pair.state = CANOPEN_TPDO_PAIR_NO_DATA;
    s_wheel_tpdo_arm_start_ms = 0U;
    memset(s_wheel_velocity_from_tpdo, 0,
           sizeof(s_wheel_velocity_from_tpdo));
    memset(s_wheel_tpdo_configured, 0,
           sizeof(s_wheel_tpdo_configured));
    memset(&s_wheel_tpdo_config_diagnostic, 0,
           sizeof(s_wheel_tpdo_config_diagnostic));
    memset(&s_runtime_diag, 0, sizeof(s_runtime_diag));
    s_rpdo_zero_wait = false;
    memset(s_heartbeat, 0, sizeof(s_heartbeat));
    memset(&s_master_heartbeat, 0, sizeof(s_master_heartbeat));
    memset(s_sdo_transaction, 0, sizeof(s_sdo_transaction));
    s_runtime_fault_pending = false;
    s_feedback_continuity_fault_pending = false;
    s_heartbeat_fault_pending = false;
    s_heartbeat_fault_node = 0U;
    s_heartbeat_fault_reason = CANOPEN_HEARTBEAT_FAULT_NONE;
    s_lift_fault_emcy_pending = false;
    s_lift_fault_emcy_code = 0U;
    s_emergency_zero_status = CANOPEN_EMERGENCY_ZERO_FAILED;
    s_emergency_zero_ack_mask = 0U;
    s_emergency_zero_confirmation_needed_mask = 0U;
    s_wheel_feedback_transport = CANOPEN_WHEEL_FEEDBACK_TPDO_SHADOW;
    s_wheel_feedback_authority = CANOPEN_WHEEL_AUTHORITY_SDO;
    s_last_wheel_tpdo_active_fault = "NONE";
    s_last_fault_monitor_defer_ms = 0U;
    s_fault_monitor_defer_recorded = false;
    s_sdo_quiet = true;
}

void Canopen_TestSetWheelFeedbackTransport(
    CanopenWheelFeedbackTransport_t transport)
{
    uint8_t slot;

    s_wheel_feedback_transport = transport;
    if (transport == CANOPEN_WHEEL_FEEDBACK_TPDO_ACTIVE) {
        s_wheel_feedback_authority = CANOPEN_WHEEL_AUTHORITY_TPDO;
        return;
    }
    s_wheel_feedback_authority = CANOPEN_WHEEL_AUTHORITY_SDO;
    for (slot = 0U; slot < 2U; slot++) {
        s_wheel_velocity[slot] = s_wheel_velocity_sdo[slot];
        s_wheel_velocity_from_tpdo[slot] = false;
    }
}
#endif

bool SDO_Write_U8_Timeout_Node(const CanopenNode_t *node,
                              uint16_t index,
                              uint8_t subindex,
                              uint8_t value,
                              uint32_t timeout_ms)
{
    uint8_t data[8] = {
        0x2FU, (uint8_t)(index & 0xFFU), (uint8_t)((index >> 8) & 0xFFU),
        subindex, value, 0U, 0U, 0U
    };
    if (!StartBlockingSdo(node, index, subindex,
                          CANOPEN_SDO_OP_BLOCKING_WRITE, 0U, timeout_ms,
                          data, "write-u8")) {
        return false;
    }
    return Wait_SDO_Response(node, index, subindex, timeout_ms);
}

bool SDO_Write_U8_Node(const CanopenNode_t *node,
                       uint16_t index,
                       uint8_t subindex,
                       uint8_t value)
{
    return SDO_Write_U8_Timeout_Node(node, index, subindex, value, 500U);
}

bool SDO_Write_I8_Node(const CanopenNode_t *node,
                       uint16_t index,
                       uint8_t subindex,
                       int8_t value)
{
    return SDO_Write_U8_Node(node, index, subindex, (uint8_t)value);
}

bool SDO_Write_I8_Timeout_Node(const CanopenNode_t *node,
                              uint16_t index,
                              uint8_t subindex,
                              int8_t value,
                              uint32_t timeout_ms)
{
    return SDO_Write_U8_Timeout_Node(node, index, subindex,
                                    (uint8_t)value, timeout_ms);
}

bool SDO_Write_U16_Timeout_Node(const CanopenNode_t *node,
                               uint16_t index,
                               uint8_t subindex,
                               uint16_t value,
                               uint32_t timeout_ms)
{
    uint8_t data[8] = {
        0x2BU, (uint8_t)(index & 0xFFU), (uint8_t)((index >> 8) & 0xFFU),
        subindex, (uint8_t)(value & 0xFFU),
        (uint8_t)((value >> 8) & 0xFFU), 0U, 0U
    };
    if (!StartBlockingSdo(node, index, subindex,
                          CANOPEN_SDO_OP_BLOCKING_WRITE, 0U, timeout_ms,
                          data, "write-u16")) {
        return false;
    }
    return Wait_SDO_Response(node, index, subindex, timeout_ms);
}

bool SDO_Write_U16_Node(const CanopenNode_t *node,
                        uint16_t index,
                        uint8_t subindex,
                        uint16_t value)
{
    return SDO_Write_U16_Timeout_Node(node, index, subindex, value, 500U);
}

bool SDO_Write_I32_Timeout_Node(const CanopenNode_t *node,
                               uint16_t index,
                               uint8_t subindex,
                               int32_t value,
                               uint32_t timeout_ms)
{
    uint32_t raw = (uint32_t)value;
    uint8_t data[8] = {
        0x23U, (uint8_t)(index & 0xFFU), (uint8_t)((index >> 8) & 0xFFU),
        subindex, (uint8_t)(raw & 0xFFU), (uint8_t)((raw >> 8) & 0xFFU),
        (uint8_t)((raw >> 16) & 0xFFU), (uint8_t)((raw >> 24) & 0xFFU)
    };
    if (!StartBlockingSdo(node, index, subindex,
                          CANOPEN_SDO_OP_BLOCKING_WRITE, 0U, timeout_ms,
                          data, "write-i32")) {
        return false;
    }
    return Wait_SDO_Response(node, index, subindex, timeout_ms);
}

bool SDO_Write_I32_Node(const CanopenNode_t *node,
                        uint16_t index,
                        uint8_t subindex,
                        int32_t value)
{
    return SDO_Write_I32_Timeout_Node(node, index, subindex, value, 500U);
}

bool SDO_Write_U32_Node(const CanopenNode_t *node,
                        uint16_t index,
                        uint8_t subindex,
                        uint32_t value)
{
    return SDO_Write_I32_Node(node, index, subindex, (int32_t)value);
}

bool SDO_Read_U16_Timeout_Node(const CanopenNode_t *node,
                              uint16_t index,
                              uint8_t subindex,
                              uint16_t *out_value,
                              uint32_t timeout_ms)
{
    uint8_t data[8] = {
        0x40U, (uint8_t)(index & 0xFFU), (uint8_t)((index >> 8) & 0xFFU),
        subindex, 0U, 0U, 0U, 0U
    };
    if ((out_value == NULL) ||
        !StartBlockingSdo(node, index, subindex,
                          CANOPEN_SDO_OP_BLOCKING_READ, 2U, timeout_ms,
                          data, "read-u16")) {
        return false;
    }
    if (!Wait_SDO_Response(node, index, subindex, timeout_ms) ||
        !s_last_sdo_response.valid || (s_last_sdo_response.len < 6U)) {
        return false;
    }
    *out_value = (uint16_t)s_last_sdo_response.data[4] |
                 ((uint16_t)s_last_sdo_response.data[5] << 8);
    return true;
}

bool SDO_Read_U16_Node(const CanopenNode_t *node,
                       uint16_t index,
                       uint8_t subindex,
                       uint16_t *out_value)
{
    return SDO_Read_U16_Timeout_Node(node, index, subindex, out_value, 500U);
}

bool SDO_Read_I8_Timeout_Node(const CanopenNode_t *node,
                             uint16_t index,
                             uint8_t subindex,
                             int8_t *out_value,
                             uint32_t timeout_ms)
{
    uint8_t data[8] = {
        0x40U, (uint8_t)(index & 0xFFU), (uint8_t)((index >> 8) & 0xFFU),
        subindex, 0U, 0U, 0U, 0U
    };
    if ((out_value == NULL) ||
        !StartBlockingSdo(node, index, subindex,
                          CANOPEN_SDO_OP_BLOCKING_READ, 1U, timeout_ms,
                          data, "read-i8")) {
        return false;
    }
    if (!Wait_SDO_Response(node, index, subindex, timeout_ms) ||
        !s_last_sdo_response.valid || (s_last_sdo_response.len < 5U)) {
        return false;
    }
    *out_value = (int8_t)s_last_sdo_response.data[4];
    return true;
}

bool SDO_Read_I8_Node(const CanopenNode_t *node,
                      uint16_t index,
                      uint8_t subindex,
                      int8_t *out_value)
{
    return SDO_Read_I8_Timeout_Node(node, index, subindex, out_value, 500U);
}

bool SDO_Read_U16_Quiet_Node(const CanopenNode_t *node,
                             uint16_t index,
                             uint8_t subindex,
                             uint16_t *out_value)
{
    bool old_quiet = s_sdo_quiet;
    bool ok;

    s_sdo_quiet = true;
    ok = SDO_Read_U16_Timeout_Node(node, index, subindex, out_value, 20U);
    s_sdo_quiet = old_quiet;
    return ok;
}

bool SDO_Read_I8_Quiet_Node(const CanopenNode_t *node,
                            uint16_t index,
                            uint8_t subindex,
                            int8_t *out_value)
{
    bool old_quiet = s_sdo_quiet;
    bool ok;

    s_sdo_quiet = true;
    ok = SDO_Read_I8_Timeout_Node(node, index, subindex, out_value, 20U);
    s_sdo_quiet = old_quiet;
    return ok;
}

bool SDO_Read_I32_Timeout_Node(const CanopenNode_t *node,
                               uint16_t index,
                               uint8_t subindex,
                               int32_t *out_value,
                               uint32_t timeout_ms)
{
    uint8_t data[8] = {
        0x40U, (uint8_t)(index & 0xFFU), (uint8_t)((index >> 8) & 0xFFU),
        subindex, 0U, 0U, 0U, 0U
    };
    if ((out_value == NULL) ||
        !StartBlockingSdo(node, index, subindex,
                          CANOPEN_SDO_OP_BLOCKING_READ, 4U, timeout_ms,
                          data, "read-i32")) {
        return false;
    }
    if (!Wait_SDO_Response(node, index, subindex, timeout_ms) ||
        !s_last_sdo_response.valid || (s_last_sdo_response.len < 8U)) {
        return false;
    }
    *out_value = DecodeI32LittleEndian(&s_last_sdo_response.data[4]);
    return true;
}

bool SDO_Read_I32_Quiet_Node(const CanopenNode_t *node,
                             uint16_t index,
                             uint8_t subindex,
                             int32_t *out_value)
{
    bool old_quiet = s_sdo_quiet;
    bool ok;

    s_sdo_quiet = true;
    ok = SDO_Read_I32_Timeout_Node(node, index, subindex, out_value, 20U);
    s_sdo_quiet = old_quiet;
    return ok;
}

bool SDO_Write_I8(uint8_t node_id, uint16_t index, uint8_t subindex, int8_t value)
{
    CanopenNode_t node = CanopenNode_OnFDCAN1(node_id);
    return SDO_Write_I8_Node(&node, index, subindex, value);
}

bool SDO_Write_U16(uint8_t node_id, uint16_t index, uint8_t subindex, uint16_t value)
{
    CanopenNode_t node = CanopenNode_OnFDCAN1(node_id);
    return SDO_Write_U16_Node(&node, index, subindex, value);
}

bool SDO_Write_I32(uint8_t node_id, uint16_t index, uint8_t subindex, int32_t value)
{
    CanopenNode_t node = CanopenNode_OnFDCAN1(node_id);
    return SDO_Write_I32_Node(&node, index, subindex, value);
}

bool SDO_Write_U32(uint8_t node_id, uint16_t index, uint8_t subindex, uint32_t value)
{
    CanopenNode_t node = CanopenNode_OnFDCAN1(node_id);
    return SDO_Write_U32_Node(&node, index, subindex, value);
}

bool SDO_Read_U16(uint8_t node_id, uint16_t index, uint8_t subindex, uint16_t *out_value)
{
    CanopenNode_t node = CanopenNode_OnFDCAN1(node_id);
    return SDO_Read_U16_Node(&node, index, subindex, out_value);
}

bool SDO_Read_I8(uint8_t node_id, uint16_t index, uint8_t subindex, int8_t *out_value)
{
    CanopenNode_t node = CanopenNode_OnFDCAN1(node_id);
    return SDO_Read_I8_Node(&node, index, subindex, out_value);
}

bool SDO_Read_I32_Timeout(uint8_t node_id,
                          uint16_t index,
                          uint8_t subindex,
                          int32_t *out_value,
                          uint32_t timeout_ms)
{
    CanopenNode_t node = CanopenNode_OnFDCAN1(node_id);
    return SDO_Read_I32_Timeout_Node(&node, index, subindex, out_value, timeout_ms);
}

bool SDO_Read_I32(uint8_t node_id, uint16_t index, uint8_t subindex, int32_t *out_value)
{
    return SDO_Read_I32_Timeout(node_id, index, subindex, out_value, 500U);
}

bool SDO_Read_I32_Quiet(uint8_t node_id, uint16_t index, uint8_t subindex, int32_t *out_value)
{
    bool old_quiet = s_sdo_quiet;
    bool ok;

    s_sdo_quiet = true;
    ok = SDO_Read_I32_Timeout(node_id, index, subindex, out_value, 20U);
    s_sdo_quiet = old_quiet;
    return ok;
}
