/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : can_trace.c
  * @brief          : Bounded FDCAN1/SDO/runtime flight recorder.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "can_trace.h"

#include "jetson_tx.h"

#include <limits.h>
#include <string.h>

#define CAN_TRACE_FDCAN_FLAG_WARNING 0x01U
#define CAN_TRACE_FDCAN_FLAG_PASSIVE 0x02U
#define CAN_TRACE_FDCAN_FLAG_BUS_OFF 0x04U
#define CAN_TRACE_PROTOCOL_POLL_MS     10U
#define CAN_TRACE_STATUS_LINE_COUNT    13U

typedef enum
{
    CAN_TRACE_EXPORT_IDLE = 0,
    CAN_TRACE_EXPORT_STATUS,
    CAN_TRACE_EXPORT_DUMP
} CanTraceExportState_t;

typedef struct
{
    CanTraceRuntimeStats_t stats;
    uint32_t last_start_ms;
    uint32_t active_start_ms;
    bool last_start_valid;
    bool active;
} CanTraceRuntimeSlot_t;

typedef struct
{
    CanTraceExportState_t state;
    uint32_t nonce;
    uint16_t item_index;
    uint16_t item_total;
    uint8_t status_line;
    bool ready_line_pending;
    bool automatic;
    bool pending_manual_dump;
    uint32_t pending_manual_nonce;
    bool pending_status;
    uint32_t pending_status_nonce;
} CanTraceExport_t;

static CanTraceEvent_t s_events[CAN_TRACE_BUFFER_CAPACITY];
static CanTraceRuntimeSlot_t s_runtime[CAN_TRACE_RUNTIME_COUNT];
static FDCAN_HandleTypeDef *s_fdcan;
static CanTraceState_t s_state;
static CanTraceTriggerReason_t s_trigger_reason;
static uint16_t s_head;
static uint16_t s_count;
static uint16_t s_pre_trigger_count;
static uint16_t s_snapshot_id;
static uint32_t s_trigger_ms;
static uint32_t s_freeze_ms;
static uint32_t s_overwritten_or_dropped;
static uint32_t s_dropped_normal;
static uint32_t s_dropped_critical;
static uint32_t s_last_protocol_poll_ms;
static uint8_t s_drive_state;
static uint8_t s_lift_state;
static bool s_error_passive_seen;
static bool s_bus_off_seen;
static bool s_automatic_dump_completed;
static CanTraceExport_t s_export;

_Static_assert(sizeof(CanTraceEvent_t) == 40U,
               "CanTraceEvent_t ARM/host layout changed");

static void CanTrace_Increment(uint32_t *value)
{
    if (*value != UINT32_MAX) {
        (*value)++;
    }
}

static uint8_t CanTrace_ClampU8(uint32_t value)
{
    return (value > UINT8_MAX) ? UINT8_MAX : (uint8_t)value;
}

static uint8_t CanTrace_DlcBytes(uint32_t dlc)
{
    static const uint8_t lengths[16] = {
        0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U,
        8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U
    };
    uint32_t code = dlc & 0x0FU;

    return lengths[code];
}

static const char *CanTrace_StateName(CanTraceState_t state)
{
    switch (state) {
        case CAN_TRACE_STATE_ROLLING: return "ROLLING";
        case CAN_TRACE_STATE_POST_TRIGGER: return "POST";
        case CAN_TRACE_STATE_FROZEN: return "FROZEN";
        default: return "UNKNOWN";
    }
}

const char *CanTrace_EventName(CanTraceEventType_t event)
{
    switch (event) {
        case CAN_TRACE_EVENT_TX: return "TX";
        case CAN_TRACE_EVENT_RX: return "RX";
        case CAN_TRACE_EVENT_RX_ERROR: return "RX_ERR";
        case CAN_TRACE_EVENT_SDO_START: return "SDO_START";
        case CAN_TRACE_EVENT_SDO_ENQUEUE_OK: return "SDO_ENQ_OK";
        case CAN_TRACE_EVENT_SDO_ENQUEUE_FAILED: return "SDO_ENQ_FAIL";
        case CAN_TRACE_EVENT_SDO_RESPONSE: return "SDO_RESP";
        case CAN_TRACE_EVENT_SDO_REMOTE_ABORT: return "SDO_ABORT";
        case CAN_TRACE_EVENT_SDO_MALFORMED_RESPONSE: return "SDO_MALFORM";
        case CAN_TRACE_EVENT_SDO_WRONG_RESPONSE: return "SDO_WRONG";
        case CAN_TRACE_EVENT_SDO_TIMEOUT: return "SDO_TIMEOUT";
        case CAN_TRACE_EVENT_SDO_COMPLETE: return "SDO_DONE";
        case CAN_TRACE_EVENT_HEARTBEAT_STALE: return "HB_STALE";
        case CAN_TRACE_EVENT_MASTER_TX_FAILURE: return "MASTER_TX";
        case CAN_TRACE_EVENT_FDCAN_ERROR_PASSIVE: return "CAN_PASSIVE";
        case CAN_TRACE_EVENT_FDCAN_BUS_OFF: return "CAN_BUS_OFF";
        case CAN_TRACE_EVENT_COMM_TRANSITION_FAILURE: return "COMM_FAIL";
        case CAN_TRACE_EVENT_RUNTIME_OVERRUN: return "RUNTIME";
        default: return "UNKNOWN";
    }
}

const char *CanTrace_OwnerName(CanTraceTxOwner_t owner)
{
    switch (owner) {
        case CAN_TRACE_TX_WHEEL_SDO: return "WHL_SDO";
        case CAN_TRACE_TX_LIFT_SDO: return "LFT_SDO";
        case CAN_TRACE_TX_MASTER_HEARTBEAT: return "M_HB";
        case CAN_TRACE_TX_NMT: return "NMT";
        case CAN_TRACE_TX_EMERGENCY_ZERO: return "EZERO";
        case CAN_TRACE_TX_NODE_GUARD: return "N_GUARD";
        case CAN_TRACE_TX_OTHER: return "OTHER";
        default: return "UNKNOWN";
    }
}

const char *CanTrace_TriggerName(CanTraceTriggerReason_t reason)
{
    switch (reason) {
        case CAN_TRACE_TRIGGER_NONE: return "NONE";
        case CAN_TRACE_TRIGGER_SDO_TIMEOUT: return "SDO_TIMEOUT";
        case CAN_TRACE_TRIGGER_MASTER_TX: return "MASTER_TX";
        case CAN_TRACE_TRIGGER_HEARTBEAT_STALE: return "HEARTBEAT_STALE";
        case CAN_TRACE_TRIGGER_TX_BUSY: return "HAL_BUSY";
        case CAN_TRACE_TRIGGER_TX_ERROR: return "HAL_ERROR";
        case CAN_TRACE_TRIGGER_ERROR_PASSIVE: return "ERROR_PASSIVE";
        case CAN_TRACE_TRIGGER_BUS_OFF: return "BUS_OFF";
        case CAN_TRACE_TRIGGER_SDO_PROTOCOL: return "SDO_PROTOCOL";
        case CAN_TRACE_TRIGGER_COMM_TRANSITION: return "COMM_TRANSITION";
        default: return "UNKNOWN";
    }
}

static const char *CanTrace_RuntimeName(CanTraceRuntimeChannel_t channel)
{
    switch (channel) {
        case CAN_TRACE_RUNTIME_MAIN_LOOP: return "MAIN";
        case CAN_TRACE_RUNTIME_CANOPEN_RX: return "RX_SERVICE";
        case CAN_TRACE_RUNTIME_HEARTBEAT: return "HEARTBEAT";
        case CAN_TRACE_RUNTIME_DRIVE_UPDATE: return "DRIVE";
        case CAN_TRACE_RUNTIME_LIFT_UPDATE: return "LIFT";
        case CAN_TRACE_RUNTIME_BLOCKING_SDO: return "SDO_WAIT";
        default: return "UNKNOWN";
    }
}

static void CanTrace_StartAutomaticDump(void)
{
    if ((s_export.state != CAN_TRACE_EXPORT_IDLE) ||
        s_automatic_dump_completed) {
        return;
    }
    s_export.state = CAN_TRACE_EXPORT_DUMP;
    s_export.nonce = 0U;
    s_export.item_index = 0U;
    s_export.item_total = s_count;
    s_export.ready_line_pending = true;
    s_export.automatic = true;
}

static void CanTrace_Freeze(uint32_t now_ms)
{
    if (s_state != CAN_TRACE_STATE_POST_TRIGGER) {
        return;
    }
    s_state = CAN_TRACE_STATE_FROZEN;
    s_freeze_ms = now_ms;
    CanTrace_StartAutomaticDump();
}

static void CanTrace_MaybeFreeze(uint32_t now_ms)
{
    if ((s_state == CAN_TRACE_STATE_POST_TRIGGER) &&
        ((uint32_t)(now_ms - s_trigger_ms) >=
         CAN_TRACE_POST_TRIGGER_MS)) {
        CanTrace_Freeze(now_ms);
    }
}

static void CanTrace_ArmTrigger(CanTraceTriggerReason_t reason,
                                uint32_t now_ms)
{
    if ((reason == CAN_TRACE_TRIGGER_NONE) ||
        (s_state != CAN_TRACE_STATE_ROLLING)) {
        return;
    }
    s_state = CAN_TRACE_STATE_POST_TRIGGER;
    s_trigger_reason = reason;
    s_trigger_ms = now_ms;
    s_pre_trigger_count = s_count;
    s_snapshot_id++;
    if (s_snapshot_id == 0U) {
        s_snapshot_id = 1U;
    }
}

static bool CanTrace_EventIsCritical(const CanTraceEvent_t *event)
{
    if (event == NULL) {
        return false;
    }
    if ((event->event == CAN_TRACE_EVENT_TX) &&
        (event->hal_status != HAL_OK)) {
        return true;
    }
    switch ((CanTraceEventType_t)event->event) {
        case CAN_TRACE_EVENT_RX_ERROR:
        case CAN_TRACE_EVENT_SDO_ENQUEUE_FAILED:
        case CAN_TRACE_EVENT_SDO_REMOTE_ABORT:
        case CAN_TRACE_EVENT_SDO_MALFORMED_RESPONSE:
        case CAN_TRACE_EVENT_SDO_WRONG_RESPONSE:
        case CAN_TRACE_EVENT_SDO_TIMEOUT:
        case CAN_TRACE_EVENT_HEARTBEAT_STALE:
        case CAN_TRACE_EVENT_MASTER_TX_FAILURE:
        case CAN_TRACE_EVENT_FDCAN_ERROR_PASSIVE:
        case CAN_TRACE_EVENT_FDCAN_BUS_OFF:
        case CAN_TRACE_EVENT_COMM_TRANSITION_FAILURE:
            return true;
        default:
            return false;
    }
}

static void CanTrace_CountDropped(bool critical)
{
    CanTrace_Increment(&s_overwritten_or_dropped);
    if (critical) {
        CanTrace_Increment(&s_dropped_critical);
    } else {
        CanTrace_Increment(&s_dropped_normal);
    }
}

static void CanTrace_RemoveChronological(uint16_t remove_index)
{
    uint16_t index;

    for (index = remove_index; (uint16_t)(index + 1U) < s_count;
         index++) {
        uint16_t destination = (uint16_t)((s_head + index) %
            CAN_TRACE_BUFFER_CAPACITY);
        uint16_t source = (uint16_t)((s_head + index + 1U) %
            CAN_TRACE_BUFFER_CAPACITY);

        s_events[destination] = s_events[source];
    }
    if (s_count > 0U) {
        s_count--;
    }
}

static bool CanTrace_EvictPostTriggerNormal(void)
{
    uint16_t index;

    for (index = s_pre_trigger_count; index < s_count; index++) {
        uint16_t slot = (uint16_t)((s_head + index) %
            CAN_TRACE_BUFFER_CAPACITY);

        if (!CanTrace_EventIsCritical(&s_events[slot])) {
            CanTrace_RemoveChronological(index);
            CanTrace_CountDropped(false);
            return true;
        }
    }
    return false;
}

static bool CanTrace_AppendWithSlot(CanTraceEvent_t event,
                                    CanTraceTriggerReason_t trigger,
                                    uint16_t *stored_slot)
{
    uint16_t limit;
    uint16_t tail;
    bool critical = CanTrace_EventIsCritical(&event);
    bool post_window_complete;

    if (s_state == CAN_TRACE_STATE_FROZEN) {
        return false;
    }
    /* Arm before appending the trigger so a full 128-entry rolling history
     * remains intact and the trigger consumes the first post-trigger slot. */
    if ((s_state == CAN_TRACE_STATE_ROLLING) &&
        (trigger != CAN_TRACE_TRIGGER_NONE)) {
        CanTrace_ArmTrigger(trigger, event.timestamp_ms);
    }
    post_window_complete =
        (s_state == CAN_TRACE_STATE_POST_TRIGGER) &&
        ((uint32_t)(event.timestamp_ms - s_trigger_ms) >=
         CAN_TRACE_POST_TRIGGER_MS);
    if (s_state == CAN_TRACE_STATE_ROLLING) {
        limit = CAN_TRACE_PRE_TRIGGER_CAPACITY;
    } else if (critical) {
        limit = CAN_TRACE_BUFFER_CAPACITY;
    } else {
        limit = CAN_TRACE_BUFFER_CAPACITY -
            CAN_TRACE_POST_CRITICAL_RESERVE;
    }
    if (s_count >= limit) {
        if (s_state == CAN_TRACE_STATE_ROLLING) {
            s_head = (uint16_t)((s_head + 1U) %
                                CAN_TRACE_BUFFER_CAPACITY);
            s_count--;
            CanTrace_CountDropped(false);
        } else if (critical &&
                   (s_count >= CAN_TRACE_BUFFER_CAPACITY) &&
                   CanTrace_EvictPostTriggerNormal()) {
            /* The evicted normal event makes room for this fault event. */
        } else {
            CanTrace_CountDropped(critical);
            if (post_window_complete && critical) {
                CanTrace_Freeze(event.timestamp_ms);
            }
            return false;
        }
    }
    event.drive_state = s_drive_state;
    event.lift_state = s_lift_state;
    tail = (uint16_t)((s_head + s_count) % CAN_TRACE_BUFFER_CAPACITY);
    s_events[tail] = event;
    s_count++;
    if (stored_slot != NULL) {
        *stored_slot = tail;
    }
    /* A boundary fault is part of the 500 ms capture. Normal traffic never
     * closes the window; CanTrace_Service performs the ordinary time freeze. */
    if (post_window_complete && critical) {
        CanTrace_Freeze(event.timestamp_ms);
    }
    return true;
}

static void CanTrace_Append(CanTraceEvent_t event,
                            CanTraceTriggerReason_t trigger)
{
    (void)CanTrace_AppendWithSlot(event, trigger, NULL);
}

static void CanTrace_ReadFdcan(FDCAN_HandleTypeDef *hfdcan,
                               CanTraceEvent_t *event)
{
    FDCAN_ProtocolStatusTypeDef protocol = {0};
    FDCAN_ErrorCountersTypeDef counters = {0};
    HAL_StatusTypeDef protocol_status;
    HAL_StatusTypeDef counter_status;

    if ((hfdcan == NULL) || (event == NULL)) {
        return;
    }
    protocol_status = HAL_FDCAN_GetProtocolStatus(hfdcan, &protocol);
    counter_status = HAL_FDCAN_GetErrorCounters(hfdcan, &counters);
    event->hal_error = HAL_FDCAN_GetError(hfdcan);
    if (protocol_status == HAL_OK) {
        event->lec = CanTrace_ClampU8(protocol.LastErrorCode);
        event->fdcan_flags =
            ((protocol.Warning != 0U) ?
                CAN_TRACE_FDCAN_FLAG_WARNING : 0U) |
            ((protocol.ErrorPassive != 0U) ?
                CAN_TRACE_FDCAN_FLAG_PASSIVE : 0U) |
            ((protocol.BusOff != 0U) ?
                CAN_TRACE_FDCAN_FLAG_BUS_OFF : 0U);
    }
    if (counter_status == HAL_OK) {
        event->tec = CanTrace_ClampU8(counters.TxErrorCnt);
        event->rec = CanTrace_ClampU8(counters.RxErrorCnt);
    }
    event->fdcan_valid = ((protocol_status == HAL_OK) &&
                          (counter_status == HAL_OK)) ? 1U : 0U;
}

static void CanTrace_RecordControllerFaults(const CanTraceEvent_t *sample)
{
    CanTraceEvent_t event;

    if (sample == NULL) {
        return;
    }
    if (((sample->fdcan_flags & CAN_TRACE_FDCAN_FLAG_PASSIVE) != 0U) &&
        !s_error_passive_seen) {
        event = *sample;
        event.event = CAN_TRACE_EVENT_FDCAN_ERROR_PASSIVE;
        event.owner = CAN_TRACE_TX_OTHER;
        CanTrace_Append(event, CAN_TRACE_TRIGGER_ERROR_PASSIVE);
    }
    if (((sample->fdcan_flags & CAN_TRACE_FDCAN_FLAG_BUS_OFF) != 0U) &&
        !s_bus_off_seen) {
        event = *sample;
        event.event = CAN_TRACE_EVENT_FDCAN_BUS_OFF;
        event.owner = CAN_TRACE_TX_OTHER;
        CanTrace_Append(event, CAN_TRACE_TRIGGER_BUS_OFF);
    }
    s_error_passive_seen =
        (sample->fdcan_flags & CAN_TRACE_FDCAN_FLAG_PASSIVE) != 0U;
    s_bus_off_seen =
        (sample->fdcan_flags & CAN_TRACE_FDCAN_FLAG_BUS_OFF) != 0U;
}

void CanTrace_Init(FDCAN_HandleTypeDef *hfdcan, uint32_t now_ms)
{
    memset(s_events, 0, sizeof(s_events));
    memset(s_runtime, 0, sizeof(s_runtime));
    memset(&s_export, 0, sizeof(s_export));
    s_fdcan = hfdcan;
    s_state = CAN_TRACE_STATE_ROLLING;
    s_trigger_reason = CAN_TRACE_TRIGGER_NONE;
    s_head = 0U;
    s_count = 0U;
    s_pre_trigger_count = 0U;
    s_snapshot_id = 0U;
    s_trigger_ms = 0U;
    s_freeze_ms = 0U;
    s_overwritten_or_dropped = 0U;
    s_dropped_normal = 0U;
    s_dropped_critical = 0U;
    s_last_protocol_poll_ms = now_ms;
    s_drive_state = 0U;
    s_lift_state = 0U;
    s_error_passive_seen = false;
    s_bus_off_seen = false;
    s_automatic_dump_completed = false;
}

void CanTrace_SetSubsystemStates(uint32_t drive_state, uint32_t lift_state)
{
    s_drive_state = CanTrace_ClampU8(drive_state);
    s_lift_state = CanTrace_ClampU8(lift_state);
}

HAL_StatusTypeDef CanTrace_SubmitTx(
    FDCAN_HandleTypeDef *hfdcan,
    const FDCAN_TxHeaderTypeDef *header,
    const uint8_t *data,
    CanTraceTxOwner_t owner,
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    bool sdo_write)
{
    CanTraceEvent_t event = {0};
    HAL_StatusTypeDef status;
    CanTraceTriggerReason_t trigger = CAN_TRACE_TRIGGER_NONE;

    if ((hfdcan == NULL) || (header == NULL) || (data == NULL)) {
        return HAL_ERROR;
    }
    if (s_fdcan == NULL) {
        s_fdcan = hfdcan;
    }
    event.timestamp_ms = HAL_GetTick();
    event.event = CAN_TRACE_EVENT_TX;
    event.owner = (uint8_t)owner;
    event.node_id = node_id;
    event.can_id = header->Identifier;
    event.dlc = CanTrace_DlcBytes(header->DataLength);
    event.index = index;
    event.subindex = subindex;
    event.sdo_write = sdo_write ? 1U : 0U;
    event.tx_free_before = CanTrace_ClampU8(
        HAL_FDCAN_GetTxFifoFreeLevel(hfdcan));
    status = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, header, data);
    event.hal_status = (uint8_t)status;
    event.tx_free_after = CanTrace_ClampU8(
        HAL_FDCAN_GetTxFifoFreeLevel(hfdcan));
    CanTrace_ReadFdcan(hfdcan, &event);
    if (status == HAL_BUSY) {
        trigger = CAN_TRACE_TRIGGER_TX_BUSY;
    } else if (status != HAL_OK) {
        trigger = CAN_TRACE_TRIGGER_TX_ERROR;
    }
    CanTrace_Append(event, trigger);
    CanTrace_RecordControllerFaults(&event);
    return status;
}

static bool CanTrace_IsSdoResponse(uint32_t can_id)
{
    return (can_id >= 0x581U) && (can_id <= 0x5FFU);
}

static bool CanTrace_IsTpdo(uint32_t can_id)
{
    return (can_id >= 0x181U) && (can_id <= 0x1FFU);
}

static bool CanTrace_UpdateTpdoAggregate(uint32_t now_ms,
                                         uint32_t can_id,
                                         uint8_t dlc,
                                         uint8_t node_id)
{
    uint16_t offset;

    if (s_state == CAN_TRACE_STATE_FROZEN) {
        return true;
    }
    for (offset = 0U; offset < s_count; offset++) {
        uint16_t chronological = (uint16_t)(s_count - 1U - offset);
        uint16_t slot = (uint16_t)((s_head + chronological) %
            CAN_TRACE_BUFFER_CAPACITY);
        CanTraceEvent_t *event = &s_events[slot];

        if ((event->event == CAN_TRACE_EVENT_RX) &&
            (event->detail == CAN_TRACE_RX_DETAIL_TPDO_AGGREGATE) &&
            (event->can_id == can_id) &&
            (event->node_id == node_id)) {
            if ((uint32_t)(now_ms - event->elapsed_ms) <
                CAN_TRACE_TPDO_AGGREGATE_MS) {
                event->timestamp_ms = now_ms;
                event->dlc = dlc;
                CanTrace_Increment(&event->code);
                return true;
            }
            break;
        }
    }
    return false;
}

void CanTrace_RecordRx(uint32_t now_ms,
                       uint32_t can_id,
                       uint8_t dlc,
                       uint8_t node_id)
{
    CanTraceEvent_t event = {0};
    bool sdo_response = CanTrace_IsSdoResponse(can_id);
    bool tpdo = CanTrace_IsTpdo(can_id);

    if (sdo_response && (dlc == 8U)) {
        /* A successful SDO is represented once by TX and once by SDO_DONE. */
        return;
    }
    if (tpdo && (dlc == 4U) &&
        CanTrace_UpdateTpdoAggregate(now_ms, can_id, dlc, node_id)) {
        return;
    }

    event.timestamp_ms = now_ms;
    event.event = ((sdo_response && (dlc != 8U)) ||
                   (tpdo && (dlc != 4U))) ?
        CAN_TRACE_EVENT_RX_ERROR : CAN_TRACE_EVENT_RX;
    event.owner = CAN_TRACE_TX_OTHER;
    event.node_id = node_id;
    event.can_id = can_id;
    event.dlc = dlc;
    if (event.event == CAN_TRACE_EVENT_RX_ERROR) {
        event.detail = CAN_TRACE_RX_DETAIL_INVALID_DLC;
        CanTrace_ReadFdcan(s_fdcan, &event);
        CanTrace_Append(event, sdo_response ?
            CAN_TRACE_TRIGGER_SDO_PROTOCOL :
            CAN_TRACE_TRIGGER_COMM_TRANSITION);
    } else {
        if (tpdo) {
            event.detail = CAN_TRACE_RX_DETAIL_TPDO_AGGREGATE;
            event.elapsed_ms = now_ms;
            event.code = 1U;
        }
        CanTrace_Append(event, CAN_TRACE_TRIGGER_NONE);
    }
}

void CanTrace_RecordRxError(uint32_t now_ms,
                            HAL_StatusTypeDef hal_status)
{
    CanTraceEvent_t event = {0};

    event.timestamp_ms = now_ms;
    event.event = CAN_TRACE_EVENT_RX_ERROR;
    event.owner = CAN_TRACE_TX_OTHER;
    event.hal_status = (uint8_t)hal_status;
    CanTrace_ReadFdcan(s_fdcan, &event);
    CanTrace_Append(event, CAN_TRACE_TRIGGER_NONE);
    CanTrace_RecordControllerFaults(&event);
}

void CanTrace_RecordTpdoTransition(uint32_t now_ms,
                                   uint32_t can_id,
                                   uint8_t node_id,
                                   CanTraceRxDetail_t detail,
                                   uint32_t gap_ms)
{
    CanTraceEvent_t event = {0};
    bool critical = (detail == CAN_TRACE_RX_DETAIL_TPDO_GAP) ||
                    (detail == CAN_TRACE_RX_DETAIL_TPDO_STALE);

    event.timestamp_ms = now_ms;
    event.event = critical ? CAN_TRACE_EVENT_RX_ERROR : CAN_TRACE_EVENT_RX;
    event.owner = CAN_TRACE_TX_OTHER;
    event.node_id = node_id;
    event.can_id = can_id;
    event.dlc = 4U;
    event.detail = (uint8_t)detail;
    event.code = gap_ms;
    if (critical) {
        CanTrace_ReadFdcan(s_fdcan, &event);
    }
    CanTrace_Append(event, critical ?
        CAN_TRACE_TRIGGER_COMM_TRANSITION : CAN_TRACE_TRIGGER_NONE);
}

void CanTrace_RecordSdo(CanTraceEventType_t event_type,
                        CanTraceTxOwner_t owner,
                        uint8_t node_id,
                        uint16_t index,
                        uint8_t subindex,
                        bool write,
                        uint32_t start_ms,
                        uint32_t now_ms,
                        HAL_StatusTypeDef enqueue_status,
                        CanTraceSdoReason_t reason,
                        uint32_t code)
{
    CanTraceEvent_t event = {0};
    CanTraceTriggerReason_t trigger = CAN_TRACE_TRIGGER_NONE;

    if ((event_type == CAN_TRACE_EVENT_SDO_START) ||
        (event_type == CAN_TRACE_EVENT_SDO_ENQUEUE_OK) ||
        (event_type == CAN_TRACE_EVENT_SDO_RESPONSE) ||
        ((event_type == CAN_TRACE_EVENT_SDO_COMPLETE) &&
         (reason != CAN_TRACE_SDO_INTERNAL_NONE))) {
        return;
    }

    event.timestamp_ms = now_ms;
    event.event = (uint8_t)event_type;
    event.owner = (uint8_t)owner;
    event.node_id = node_id;
    event.can_id = 0x600U + node_id;
    event.dlc = 8U;
    event.index = index;
    event.subindex = subindex;
    event.sdo_write = write ? 1U : 0U;
    event.elapsed_ms = (uint32_t)(now_ms - start_ms);
    event.hal_status = (uint8_t)enqueue_status;
    event.detail = (uint8_t)reason;
    event.code = code;
    if (event_type == CAN_TRACE_EVENT_SDO_TIMEOUT) {
        trigger = CAN_TRACE_TRIGGER_SDO_TIMEOUT;
    } else if (event_type == CAN_TRACE_EVENT_SDO_REMOTE_ABORT) {
        trigger = CAN_TRACE_TRIGGER_SDO_PROTOCOL;
    } else if ((event_type == CAN_TRACE_EVENT_SDO_MALFORMED_RESPONSE) ||
               (event_type == CAN_TRACE_EVENT_SDO_WRONG_RESPONSE)) {
        trigger = CAN_TRACE_TRIGGER_SDO_PROTOCOL;
    }
    if (CanTrace_EventIsCritical(&event)) {
        CanTrace_ReadFdcan(s_fdcan, &event);
    }
    CanTrace_Append(event, trigger);
}

void CanTrace_RecordHeartbeatFault(uint8_t node_id,
                                   bool master_tx_failure,
                                   uint32_t now_ms,
                                   uint32_t detail)
{
    CanTraceEvent_t event = {0};

    event.timestamp_ms = now_ms;
    event.event = master_tx_failure ?
        CAN_TRACE_EVENT_MASTER_TX_FAILURE :
        CAN_TRACE_EVENT_HEARTBEAT_STALE;
    event.owner = master_tx_failure ?
        CAN_TRACE_TX_MASTER_HEARTBEAT : CAN_TRACE_TX_OTHER;
    event.node_id = node_id;
    event.code = detail;
    CanTrace_ReadFdcan(s_fdcan, &event);
    CanTrace_Append(event, master_tx_failure ?
                    CAN_TRACE_TRIGGER_MASTER_TX :
                    CAN_TRACE_TRIGGER_HEARTBEAT_STALE);
}

void CanTrace_RecordCommunicationFailure(uint8_t node_id,
                                         uint32_t now_ms,
                                         uint32_t detail)
{
    CanTraceEvent_t event = {0};

    event.timestamp_ms = now_ms;
    event.event = CAN_TRACE_EVENT_COMM_TRANSITION_FAILURE;
    event.owner = CAN_TRACE_TX_OTHER;
    event.node_id = node_id;
    event.code = detail;
    CanTrace_ReadFdcan(s_fdcan, &event);
    CanTrace_Append(event, CAN_TRACE_TRIGGER_COMM_TRANSITION);
}

static void CanTrace_RuntimeUpdate(CanTraceRuntimeChannel_t channel,
                                   uint32_t duration_ms)
{
    CanTraceRuntimeStats_t *stats;

    if (channel >= CAN_TRACE_RUNTIME_COUNT) {
        return;
    }
    stats = &s_runtime[channel].stats;
    stats->current_ms = duration_ms;
    if (duration_ms > stats->maximum_ms) {
        stats->maximum_ms = duration_ms;
    }
    if (duration_ms > 5U) {
        CanTrace_Increment(&stats->over_5ms_count);
    }
    if (duration_ms > 20U) {
        CanTrace_Increment(&stats->over_20ms_count);
    }
    if (duration_ms > 100U) {
        CanTrace_Increment(&stats->over_100ms_count);
    }
    if (duration_ms > 350U) {
        CanTrace_Increment(&stats->over_350ms_count);
    }
    if (duration_ms > 5U) {
        CanTraceEvent_t event = {0};

        event.timestamp_ms = HAL_GetTick();
        event.event = CAN_TRACE_EVENT_RUNTIME_OVERRUN;
        event.owner = CAN_TRACE_TX_OTHER;
        event.detail = (uint8_t)channel;
        event.elapsed_ms = duration_ms;
        CanTrace_Append(event, CAN_TRACE_TRIGGER_NONE);
    }
}

void CanTrace_RuntimeBegin(CanTraceRuntimeChannel_t channel,
                           uint32_t now_ms)
{
    CanTraceRuntimeSlot_t *slot;

    if (channel >= CAN_TRACE_RUNTIME_COUNT) {
        return;
    }
    slot = &s_runtime[channel];
    if (slot->last_start_valid) {
        uint32_t gap = (uint32_t)(now_ms - slot->last_start_ms);

        slot->stats.current_gap_ms = gap;
        if (gap > slot->stats.maximum_gap_ms) {
            slot->stats.maximum_gap_ms = gap;
        }
    }
    slot->last_start_ms = now_ms;
    slot->last_start_valid = true;
    slot->active_start_ms = now_ms;
    slot->active = true;
}

void CanTrace_RuntimeEnd(CanTraceRuntimeChannel_t channel,
                         uint32_t now_ms)
{
    CanTraceRuntimeSlot_t *slot;

    if (channel >= CAN_TRACE_RUNTIME_COUNT) {
        return;
    }
    slot = &s_runtime[channel];
    if (!slot->active) {
        return;
    }
    slot->active = false;
    CanTrace_RuntimeUpdate(
        channel, (uint32_t)(now_ms - slot->active_start_ms));
}

void CanTrace_RuntimeObserve(CanTraceRuntimeChannel_t channel,
                             uint32_t start_ms,
                             uint32_t end_ms)
{
    CanTrace_RuntimeUpdate(channel, (uint32_t)(end_ms - start_ms));
}

CanTraceStatus_t CanTrace_GetStatus(void)
{
    CanTraceStatus_t result;

    result.state = s_state;
    result.trigger_reason = s_trigger_reason;
    result.snapshot_id = s_snapshot_id;
    result.trigger_ms = s_trigger_ms;
    result.freeze_ms = s_freeze_ms;
    result.overwritten_or_dropped = s_overwritten_or_dropped;
    result.dropped_normal = s_dropped_normal;
    result.dropped_critical = s_dropped_critical;
    result.entry_count = s_count;
    result.dump_active = s_export.state == CAN_TRACE_EXPORT_DUMP;
    result.automatic_dump_completed = s_automatic_dump_completed;
    return result;
}

CanTraceRuntimeStats_t CanTrace_GetRuntimeStats(
    CanTraceRuntimeChannel_t channel)
{
    CanTraceRuntimeStats_t empty = {0};

    if (channel >= CAN_TRACE_RUNTIME_COUNT) {
        return empty;
    }
    return s_runtime[channel].stats;
}

bool CanTrace_GetEvent(uint16_t chronological_index,
                       CanTraceEvent_t *event)
{
    uint16_t slot;

    if ((event == NULL) || (chronological_index >= s_count)) {
        return false;
    }
    slot = (uint16_t)((s_head + chronological_index) %
                      CAN_TRACE_BUFFER_CAPACITY);
    *event = s_events[slot];
    return true;
}

bool CanTrace_RequestStatus(uint32_t request_nonce)
{
    if (request_nonce == 0U) {
        return false;
    }
    if (s_export.state != CAN_TRACE_EXPORT_IDLE) {
        s_export.pending_status = true;
        s_export.pending_status_nonce = request_nonce;
        return true;
    }
    s_export.state = CAN_TRACE_EXPORT_STATUS;
    s_export.nonce = request_nonce;
    s_export.status_line = 0U;
    s_export.automatic = false;
    return true;
}

static void CanTrace_StartDump(uint32_t request_nonce, bool automatic)
{
    s_export.state = CAN_TRACE_EXPORT_DUMP;
    s_export.nonce = request_nonce;
    s_export.item_index = 0U;
    s_export.item_total =
        (s_state == CAN_TRACE_STATE_FROZEN) ? s_count : 0U;
    s_export.ready_line_pending = automatic;
    s_export.automatic = automatic;
}

bool CanTrace_RequestDump(uint32_t request_nonce)
{
    if (request_nonce == 0U) {
        return false;
    }
    if (s_export.state != CAN_TRACE_EXPORT_IDLE) {
        s_export.pending_manual_dump = true;
        s_export.pending_manual_nonce = request_nonce;
        return true;
    }
    CanTrace_StartDump(request_nonce, false);
    return true;
}

static bool CanTrace_SendStatusLine(void)
{
    CanTraceRuntimeStats_t stats;
    CanTraceRuntimeChannel_t channel;

    if (s_export.status_line == 0U) {
        return JetsonTx_Printf(
            JETSON_TX_PRIORITY_DEBUG,
            "DRIVE_CAN_TRACE_STATUS 1/13 snap=%u state=%s reason=%s entries=%u dn=%lu dc=%lu trigger=%lu freeze=%lu req=%08lX\r\n",
            (unsigned)s_snapshot_id,
            CanTrace_StateName(s_state),
            CanTrace_TriggerName(s_trigger_reason),
            (unsigned)s_count,
            (unsigned long)s_dropped_normal,
            (unsigned long)s_dropped_critical,
            (unsigned long)s_trigger_ms,
            (unsigned long)s_freeze_ms,
            (unsigned long)s_export.nonce);
    }
    channel = (CanTraceRuntimeChannel_t)((s_export.status_line - 1U) / 2U);
    stats = CanTrace_GetRuntimeStats(channel);
    if ((s_export.status_line & 1U) != 0U) {
        return JetsonTx_Printf(
            JETSON_TX_PRIORITY_DEBUG,
            "DRIVE_CAN_TRACE_STATUS %u/13 snap=%u rt=%s cur=%lu max=%lu c5=%lu c20=%lu c100=%lu c350=%lu req=%08lX\r\n",
            (unsigned)(s_export.status_line + 1U),
            (unsigned)s_snapshot_id,
            CanTrace_RuntimeName(channel),
            (unsigned long)stats.current_ms,
            (unsigned long)stats.maximum_ms,
            (unsigned long)stats.over_5ms_count,
            (unsigned long)stats.over_20ms_count,
            (unsigned long)stats.over_100ms_count,
            (unsigned long)stats.over_350ms_count,
            (unsigned long)s_export.nonce);
    }
    return JetsonTx_Printf(
        JETSON_TX_PRIORITY_DEBUG,
        "DRIVE_CAN_TRACE_STATUS %u/13 snap=%u rt=%s gap=%lu gapmax=%lu req=%08lX\r\n",
        (unsigned)(s_export.status_line + 1U),
        (unsigned)s_snapshot_id,
        CanTrace_RuntimeName(channel),
        (unsigned long)stats.current_gap_ms,
        (unsigned long)stats.maximum_gap_ms,
        (unsigned long)s_export.nonce);
}

static bool CanTrace_SendDumpLine(void)
{
    if (s_export.ready_line_pending) {
        return JetsonTx_Printf(
            JETSON_TX_PRIORITY_DEBUG,
            "DRIVE_CAN_TRACE_READY snap=%u reason=%s entries=%u dn=%lu dc=%lu\r\n",
            (unsigned)s_snapshot_id,
            CanTrace_TriggerName(s_trigger_reason),
            (unsigned)s_export.item_total,
            (unsigned long)s_dropped_normal,
            (unsigned long)s_dropped_critical);
    }
    if (s_export.item_index == 0U) {
        return JetsonTx_Printf(
            JETSON_TX_PRIORITY_DEBUG,
            "DRIVE_CAN_TRACE_BEGIN snap=%u total=%u req=%08lX\r\n",
            (unsigned)s_snapshot_id,
            (unsigned)s_export.item_total,
            (unsigned long)s_export.nonce);
    }
    if (s_export.item_index <= s_export.item_total) {
        CanTraceEvent_t event;
        uint16_t event_index = (uint16_t)(s_export.item_index - 1U);

        if (!CanTrace_GetEvent(event_index, &event)) {
            return false;
        }
        return JetsonTx_Printf(
            JETSON_TX_PRIORITY_DEBUG,
            "DRIVE_CAN_TRACE i=%u/%u sn=%u t=%lu e=%s o=%s n=%u id=%03lX d=%u h=%u q=%u/%u x=%08lX c=%u/%u/%u/%u fdv=%u s=%u/%u z=%04X/%02X/%u r=%u m=%lu v=%08lX\r\n",
            (unsigned)s_export.item_index,
            (unsigned)s_export.item_total,
            (unsigned)s_snapshot_id,
            (unsigned long)event.timestamp_ms,
            CanTrace_EventName((CanTraceEventType_t)event.event),
            CanTrace_OwnerName((CanTraceTxOwner_t)event.owner),
            (unsigned)event.node_id,
            (unsigned long)event.can_id,
            (unsigned)event.dlc,
            (unsigned)event.hal_status,
            (unsigned)event.tx_free_before,
            (unsigned)event.tx_free_after,
            (unsigned long)event.hal_error,
            (unsigned)event.lec,
            (unsigned)event.tec,
            (unsigned)event.rec,
            (unsigned)event.fdcan_flags,
            (unsigned)event.fdcan_valid,
            (unsigned)event.drive_state,
            (unsigned)event.lift_state,
            (unsigned)event.index,
            (unsigned)event.subindex,
            (unsigned)event.sdo_write,
            (unsigned)event.detail,
            (unsigned long)event.elapsed_ms,
            (unsigned long)event.code);
    }
    return JetsonTx_Printf(
        JETSON_TX_PRIORITY_DEBUG,
        "DRIVE_CAN_TRACE_END snap=%u total=%u dn=%lu dc=%lu req=%08lX\r\n",
        (unsigned)s_snapshot_id,
        (unsigned)s_export.item_total,
        (unsigned long)s_dropped_normal,
        (unsigned long)s_dropped_critical,
        (unsigned long)s_export.nonce);
}

static void CanTrace_FinishExport(void)
{
    bool was_automatic = s_export.automatic;

    s_export.state = CAN_TRACE_EXPORT_IDLE;
    s_export.automatic = false;
    if (was_automatic) {
        s_automatic_dump_completed = true;
    }
    if (s_export.pending_manual_dump) {
        uint32_t nonce = s_export.pending_manual_nonce;

        s_export.pending_manual_dump = false;
        CanTrace_StartDump(nonce, false);
    } else if (s_export.pending_status) {
        uint32_t nonce = s_export.pending_status_nonce;

        s_export.pending_status = false;
        (void)CanTrace_RequestStatus(nonce);
    }
}

void CanTrace_Service(uint32_t now_ms)
{
    CanTraceEvent_t controller = {0};
    uint32_t lines_sent = 0U;

    CanTrace_MaybeFreeze(now_ms);
    if ((s_fdcan != NULL) &&
        ((uint32_t)(now_ms - s_last_protocol_poll_ms) >=
         CAN_TRACE_PROTOCOL_POLL_MS)) {
        s_last_protocol_poll_ms = now_ms;
        controller.timestamp_ms = now_ms;
        CanTrace_ReadFdcan(s_fdcan, &controller);
        CanTrace_RecordControllerFaults(&controller);
    }
    if ((s_state == CAN_TRACE_STATE_FROZEN) &&
        (s_export.state == CAN_TRACE_EXPORT_IDLE) &&
        !s_automatic_dump_completed) {
        CanTrace_StartAutomaticDump();
    }
    while ((lines_sent < CAN_TRACE_DUMP_LINES_PER_SERVICE) &&
           (s_export.state != CAN_TRACE_EXPORT_IDLE)) {
        bool queued;

        if (s_export.state == CAN_TRACE_EXPORT_STATUS) {
            queued = CanTrace_SendStatusLine();
            if (!queued) {
                break;
            }
            s_export.status_line++;
            if (s_export.status_line >= CAN_TRACE_STATUS_LINE_COUNT) {
                CanTrace_FinishExport();
            }
        } else {
            queued = CanTrace_SendDumpLine();
            if (!queued) {
                break;
            }
            if (s_export.ready_line_pending) {
                s_export.ready_line_pending = false;
            } else {
                s_export.item_index++;
                if (s_export.item_index >
                    (uint16_t)(s_export.item_total + 1U)) {
                    CanTrace_FinishExport();
                }
            }
        }
        lines_sent++;
    }
}

#ifdef AMR_HOST_TEST
void CanTrace_TestReset(uint32_t now_ms)
{
    CanTrace_Init(s_fdcan, now_ms);
}
#endif
