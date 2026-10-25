/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : canopen.h
  * @brief          : CANopen SDO helper interfaces.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef CANOPEN_H
#define CANOPEN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define DRIVE_MASTER_HEARTBEAT_NODE_ID              127U
#define DRIVE_MASTER_HEARTBEAT_COB_ID             0x77FU
#define DRIVE_MASTER_HEARTBEAT_STATE               0x05U
#define DRIVE_MASTER_HEARTBEAT_PERIOD_MS             100U
#define DRIVE_MASTER_HEARTBEAT_TX_FAILURE_LIMIT        3U
#define DRIVE_KINCO_HEARTBEAT_PRODUCER_MS             100U
#define DRIVE_HEARTBEAT_CONSUMER_TIMEOUT_MS            350U
#define DRIVE_HEARTBEAT_STALE_TIMEOUT_MS               350U
#define DRIVE_HEARTBEAT_QUALIFY_FRAMES                   2U
#define DRIVE_KINCO_HEARTBEAT_CONSUMER_VALUE \
    (((uint32_t)DRIVE_MASTER_HEARTBEAT_NODE_ID << 16) | \
     (uint32_t)DRIVE_HEARTBEAT_CONSUMER_TIMEOUT_MS)

typedef struct
{
    bool valid;
    uint8_t node_id;
    uint8_t command;
    uint8_t len;
    uint16_t index;
    uint8_t subindex;
    uint32_t sequence;
    uint8_t data[8];
} SDO_Response_t;

typedef struct
{
    FDCAN_HandleTypeDef *hfdcan;
    uint8_t node_id;
} CanopenNode_t;

typedef struct
{
    int32_t value;
    uint32_t timestamp_ms;
    uint32_t sequence;
    bool valid;
} CanopenValueSample_t;

typedef enum
{
    CANOPEN_RUNTIME_SUBMIT_OK = 0,
    CANOPEN_RUNTIME_SUBMIT_BUSY,
    CANOPEN_RUNTIME_SUBMIT_ERROR
} CanopenRuntimeSubmitResult_t;

typedef enum
{
    CANOPEN_SDO_TX_IDLE = 0,
    CANOPEN_SDO_TX_ACTIVE,
    CANOPEN_SDO_TX_COMPLETED,
    CANOPEN_SDO_TX_QUARANTINE
} CanopenSdoTransactionState_t;

typedef enum
{
    CANOPEN_SDO_OP_NONE = 0,
    CANOPEN_SDO_OP_BLOCKING_READ,
    CANOPEN_SDO_OP_BLOCKING_WRITE,
    CANOPEN_SDO_OP_ASYNC_READ,
    CANOPEN_SDO_OP_ASYNC_WRITE,
    CANOPEN_SDO_OP_RUNTIME_VELOCITY,
    CANOPEN_SDO_OP_RUNTIME_POSITION,
    CANOPEN_SDO_OP_RUNTIME_TARGET,
    CANOPEN_SDO_OP_EMERGENCY_ZERO
} CanopenSdoOperation_t;

typedef enum
{
    CANOPEN_SDO_OWNER_NONE = 0,
    CANOPEN_SDO_OWNER_BLOCKING,
    CANOPEN_SDO_OWNER_LIFT_ACTION,
    CANOPEN_SDO_OWNER_NORMAL_FEEDBACK,
    CANOPEN_SDO_OWNER_STOP_FEEDBACK,
    CANOPEN_SDO_OWNER_FAULT_MONITOR,
    CANOPEN_SDO_OWNER_RUNTIME_TARGET,
    CANOPEN_SDO_OWNER_EMERGENCY_ZERO,
    CANOPEN_SDO_OWNER_SAFETY_ENABLE,
    CANOPEN_SDO_OWNER_SAFETY_CONFIRM,
    CANOPEN_SDO_OWNER_SAFETY_ROLLBACK
} CanopenSdoOwner_t;

typedef enum
{
    CANOPEN_ASYNC_START_OK = 0,
    CANOPEN_ASYNC_START_BUSY,
    CANOPEN_ASYNC_START_ERROR
} CanopenAsyncStartResult_t;

typedef enum
{
    CANOPEN_ASYNC_RESULT_IDLE = 0,
    CANOPEN_ASYNC_RESULT_PENDING,
    CANOPEN_ASYNC_RESULT_SUCCESS,
    CANOPEN_ASYNC_RESULT_TIMEOUT,
    CANOPEN_ASYNC_RESULT_ABORT,
    CANOPEN_ASYNC_RESULT_PROTOCOL_ERROR,
    CANOPEN_ASYNC_RESULT_OWNER_MISMATCH
} CanopenAsyncResult_t;

typedef enum
{
    CANOPEN_BOOT_SDO_FAULT_NONE = 0,
    CANOPEN_BOOT_SDO_FAULT_ABORT,
    CANOPEN_BOOT_SDO_FAULT_STATUS,
    CANOPEN_BOOT_SDO_FAULT_PROTOCOL,
    CANOPEN_BOOT_SDO_FAULT_EMCY
} CanopenBootSdoFaultReason_t;

typedef struct
{
    CanopenBootSdoFaultReason_t hard_fault_reason;
    uint8_t node_id;
    uint16_t statusword;
    uint32_t abort_code;
    uint16_t emcy_code;
} CanopenBootSdoEvidence_t;

typedef enum
{
    CANOPEN_EMERGENCY_ZERO_FAILED = 0,
    CANOPEN_EMERGENCY_ZERO_ENQUEUED_UNCONFIRMED,
    CANOPEN_EMERGENCY_ZERO_PENDING_ACK,
    CANOPEN_EMERGENCY_ZERO_ACKNOWLEDGED
} CanopenEmergencyZeroStatus_t;

typedef enum
{
    CANOPEN_WHEEL_TARGET_BOUNDED_SYNC_SDO = 0, /* historical diagnostics */
    CANOPEN_WHEEL_TARGET_RPDO = 1
} CanopenWheelTargetTransport_t;

typedef enum
{
    CANOPEN_WHEEL_FEEDBACK_SDO_POLL = 0,
    CANOPEN_WHEEL_FEEDBACK_TPDO_SHADOW_CONFIGURING,
    CANOPEN_WHEEL_FEEDBACK_TPDO_SHADOW,
    CANOPEN_WHEEL_FEEDBACK_TPDO_ACTIVE_ARMING,
    CANOPEN_WHEEL_FEEDBACK_TPDO_ACTIVE,
    CANOPEN_WHEEL_FEEDBACK_TPDO_FAULT
} CanopenWheelFeedbackTransport_t;

typedef enum
{
    CANOPEN_WHEEL_AUTHORITY_SDO = 0,
    CANOPEN_WHEEL_AUTHORITY_TPDO
} CanopenWheelFeedbackAuthority_t;

typedef enum
{
    CANOPEN_TPDO_SAMPLE_NO_DATA = 0,
    CANOPEN_TPDO_SAMPLE_FRESH,
    CANOPEN_TPDO_SAMPLE_STALE
} CanopenWheelTpdoSampleState_t;

typedef enum
{
    CANOPEN_TPDO_PAIR_NO_DATA = 0,
    CANOPEN_TPDO_PAIR_FRESH,
    CANOPEN_TPDO_PAIR_STALE,
    CANOPEN_TPDO_PAIR_FAULT
} CanopenWheelTpdoPairState_t;

typedef struct
{
    int32_t raw;
    uint32_t timestamp_ms;
    uint32_t sequence;
    bool seen;
    bool valid;
    /* Lifetime count of every exact wheel TPDO COB-ID received, including
     * frames rejected by the commissioning gates below. */
    uint32_t frame_count;
    /* Lifetime count of exact wheel TPDO frames rejected while that node's
     * TPDO configuration is incomplete. */
    uint32_t ignored_unconfigured_frame_count;
    /* Lifetime count of configured-node frames rejected because feedback
     * transport was neither TPDO_SHADOW nor TPDO_ACTIVE. */
    uint32_t ignored_transport_mode_frame_count;
    /* Lifetime count of configured, valid-transport frames rejected because
     * their DLC did not equal DRIVE_WHEEL_TPDO_EXPECTED_DLC. */
    uint32_t invalid_dlc_count;
    /* Measurement-epoch maximum; reset whenever configuration starts. */
    uint32_t max_interarrival_gap_ms;
    /* Lifetime freshness-transition diagnostics; configuration does not
     * reset these counters. */
    uint32_t stale_transition_count;
    uint32_t recovery_transition_count;
    CanopenWheelTpdoSampleState_t state;
} CanopenWheelTpdoSample_t;

typedef struct
{
    int32_t left_raw;
    int32_t right_raw;
    uint32_t left_timestamp_ms;
    uint32_t right_timestamp_ms;
    uint32_t left_sequence;
    uint32_t right_sequence;
    uint32_t timestamp_ms;
    uint32_t sequence;
    uint32_t pair_skew_ms;
    uint32_t last_coherent_pair_timestamp_ms;
    bool valid;
    CanopenWheelTpdoPairState_t state;
} CanopenWheelTpdoPair_t;

typedef enum
{
    CANOPEN_TPDO_CONFIG_SDO_NOT_RUN = 0,
    CANOPEN_TPDO_CONFIG_SDO_SUCCESS,
    CANOPEN_TPDO_CONFIG_SDO_TIMEOUT,
    CANOPEN_TPDO_CONFIG_SDO_ABORT,
    CANOPEN_TPDO_CONFIG_SDO_ERROR
} CanopenWheelTpdoConfigSdoResult_t;

typedef enum
{
    CANOPEN_TPDO_CONFIG_RESULT_NONE = 0,
    CANOPEN_TPDO_CONFIG_RESULT_SUCCESS,
    CANOPEN_TPDO_CONFIG_RESULT_INVALID_NODE,
    CANOPEN_TPDO_CONFIG_RESULT_SDO_FAILURE,
    CANOPEN_TPDO_CONFIG_RESULT_READBACK_MISMATCH,
    CANOPEN_TPDO_CONFIG_RESULT_NMT_START_FAILED
} CanopenWheelTpdoConfigResult_t;

typedef struct
{
    uint8_t node_id;
    uint8_t step;
    uint16_t index;
    uint8_t subindex;
    CanopenWheelTpdoConfigSdoResult_t sdo_result;
    CanopenWheelTpdoConfigResult_t result;
    uint32_t abort_code;
    uint32_t expected_value;
    uint32_t actual_value;
} CanopenWheelTpdoConfigDiagnostic_t;

typedef enum
{
    CANOPEN_FEEDBACK_DISPATCH_ONLY = 0,
    CANOPEN_FEEDBACK_NORMAL_MOTION,
    CANOPEN_FEEDBACK_STOP_CONFIRM_VELOCITY_ONLY,
    CANOPEN_FEEDBACK_FAULT_MONITOR_VELOCITY_ONLY
} CanopenWheelFeedbackSchedule_t;

typedef enum
{
    CANOPEN_HEARTBEAT_FAULT_NONE = 0,
    CANOPEN_HEARTBEAT_FAULT_STALE,
    CANOPEN_HEARTBEAT_FAULT_BOOTUP,
    CANOPEN_HEARTBEAT_FAULT_LENGTH,
    CANOPEN_HEARTBEAT_FAULT_STATE,
    CANOPEN_HEARTBEAT_FAULT_MASTER_TX
} CanopenHeartbeatFaultReason_t;

typedef enum
{
    CANOPEN_TARGET_FAILURE_NONE = 0,
    CANOPEN_TARGET_FAILURE_TX,
    CANOPEN_TARGET_FAILURE_TIMEOUT,
    CANOPEN_TARGET_FAILURE_SDO_ABORT,
    CANOPEN_TARGET_FAILURE_PROTOCOL,
    CANOPEN_TARGET_FAILURE_PREEMPT,
    CANOPEN_TARGET_FAILURE_EMCY
} CanopenTargetFailureReason_t;

typedef struct
{
    uint32_t rx_frame_count;
    uint32_t sdo_response_count;
    uint32_t tpdo_frame_count;
    uint32_t heartbeat_frame_count;
    uint32_t emcy_frame_count;
    uint32_t unhandled_frame_count;
    uint32_t rx_error_count;
    uint32_t wheel_feedback_request_count;
    uint32_t wheel_velocity_request_count;
    uint32_t runtime_sdo_606c_poll_count;
    uint32_t wheel_position_request_count;
    uint32_t stop_velocity_request_count;
    uint32_t fault_velocity_request_count;
    uint32_t fault_velocity_timeout_count;
    uint32_t safety_request_preemption_count;
    uint32_t fault_monitor_deferred_count;
    uint32_t wheel_feedback_success_count;
    uint32_t wheel_feedback_failure_count;
    uint32_t wheel_target_submit_count;
    uint32_t wheel_target_complete_count;
    uint32_t wheel_target_failure_count;
    uint8_t last_wheel_target_failure_node;
    uint8_t last_wheel_target_failure_reason;
    uint32_t emergency_zero_count;
    uint32_t emergency_zero_ack_count;
    uint32_t sdo_timeout_count;
    uint32_t sdo_quarantine_count;
    uint32_t sdo_late_response_count;
    uint32_t sdo_unexpected_response_count;
    uint32_t sdo_abort_count;
    uint32_t last_sdo_abort_code;
    uint8_t last_sdo_abort_node;
    uint32_t pdo_config_attempt_count;
    uint32_t pdo_config_success_count;
    uint32_t pdo_config_failure_count;
    uint32_t pdo_rollback_count;
    uint32_t last_heartbeat_ms[4];
    uint8_t last_heartbeat_state[4];
    uint32_t heartbeat_sequence[4];
    uint8_t heartbeat_configured[4];
    uint8_t heartbeat_qualification_active[4];
    uint8_t heartbeat_qualified[4];
    uint32_t heartbeat_baseline_sequence[4];
    uint8_t heartbeat_qualification_count[4];
    uint32_t heartbeat_receive_count[4];
    uint32_t heartbeat_stale_count[4];
    uint32_t heartbeat_bootup_count[4];
    uint32_t heartbeat_invalid_count[4];
    uint8_t heartbeat_fault_reason[4];
    uint8_t heartbeat_fault_node;
    uint8_t heartbeat_pending_reason;
    uint8_t heartbeat_fault_pending;
    uint8_t master_heartbeat_started;
    uint8_t master_heartbeat_fault_latched;
    uint32_t master_heartbeat_tx_attempt_count;
    uint32_t master_heartbeat_tx_success_count;
    uint32_t master_heartbeat_tx_failure_count;
    uint32_t master_heartbeat_consecutive_failures;
    uint32_t master_heartbeat_last_success_ms;
    uint16_t last_emcy_code[4];
    uint32_t last_emcy_ms[4];
    uint32_t emcy_sequence[4];
    uint8_t feedback_pending_mask;
    uint8_t target_pending_mask;
    uint8_t sdo_state[4];
    uint8_t sdo_operation[4];
    uint8_t sdo_owner[4];
} CanopenRuntimeDiagnostics_t;

extern SDO_Response_t s_last_sdo_response;

/* Bounded runtime dispatcher with independent wheel target and feedback
 * transports. Normal target authority is RPDO; TPDO is
 * feedback-only and is qualified automatically during Drive startup. */
void Canopen_ProcessRx(uint32_t now_ms, uint32_t frame_budget);

/* Boot-only lifecycle. Fence blocks wheel SDO clients while leaving Lift,
 * EMCY, heartbeat and PDO dispatch intact. Resume retires software requests;
 * if RX remains after its bounded dispatch, it returns false and keeps the
 * fence active for the next call. Only true permits retry admission, and
 * the boot owner must check hard faults and independent bilateral status.
 * These APIs do not cancel shared hardware FIFO entries or prove wire identity. */
void Canopen_BootSdoBegin(void);
void Canopen_BootSdoFence(uint32_t now_ms);
bool Canopen_BootSdoResume(uint32_t now_ms);
void Canopen_BootSdoEnd(void);
CanopenBootSdoEvidence_t Canopen_GetBootSdoEvidence(void);
void Canopen_WheelRuntimeService(uint32_t now_ms,
                                 CanopenWheelFeedbackSchedule_t schedule);
bool Canopen_StartRemoteNode(uint8_t node_id);
bool Canopen_ConfigureWheelTpdoFeedbackNode(uint8_t node_id);
bool Canopen_WheelTpdoConfigured(uint8_t node_id);
CanopenWheelTpdoConfigDiagnostic_t Canopen_GetWheelTpdoConfigDiagnostic(void);
CanopenWheelTargetTransport_t Canopen_GetWheelTargetTransport(void);
CanopenWheelFeedbackTransport_t Canopen_GetWheelFeedbackTransport(void);
CanopenWheelFeedbackAuthority_t Canopen_GetWheelFeedbackAuthority(void);
const char *Canopen_WheelFeedbackTransportName(
    CanopenWheelFeedbackTransport_t transport);
const char *Canopen_WheelFeedbackAuthorityName(
    CanopenWheelFeedbackAuthority_t authority);
const char *Canopen_WheelTpdoSampleStateName(
    CanopenWheelTpdoSampleState_t state);
const char *Canopen_WheelTpdoPairStateName(CanopenWheelTpdoPairState_t state);
bool Canopen_GetWheelTpdoShadowSample(uint8_t node_id,
                                      uint32_t now_ms,
                                      CanopenWheelTpdoSample_t *sample);
bool Canopen_GetWheelTpdoPair(uint32_t now_ms,
                              CanopenWheelTpdoPair_t *pair);
bool Canopen_BeginWheelTpdoActiveArm(uint32_t now_ms);
void Canopen_AbortWheelTpdoActiveArm(void);
void Canopen_LatchWheelTpdoStartupFault(const char *reason);
bool Canopen_DisableWheelTpdoActive(void);
bool Canopen_RuntimeSdo606cPollEnabled(void);
const char *Canopen_GetLastWheelTpdoActiveFault(void);
bool Canopen_GetWheelSdoVelocitySample(uint8_t node_id,
                                       CanopenValueSample_t *sample);
bool Canopen_GetWheelVelocitySample(uint8_t node_id,
                                    CanopenValueSample_t *sample);
bool Canopen_GetWheelPositionSample(uint8_t node_id,
                                    CanopenValueSample_t *sample);
CanopenRuntimeSubmitResult_t Canopen_SubmitWheelTargetPair(
    int32_t left_velocity_dec,
    int32_t right_velocity_dec,
    uint32_t now_ms);
CanopenEmergencyZeroStatus_t Canopen_EmergencyZeroWheelTargets(void);
CanopenEmergencyZeroStatus_t Canopen_GetEmergencyZeroStatus(void);
CanopenAsyncStartResult_t Canopen_AsyncWriteU8(
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    uint8_t value,
    CanopenSdoOwner_t owner,
    uint32_t now_ms,
    uint32_t timeout_ms);
CanopenAsyncStartResult_t Canopen_AsyncWriteU16(
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    uint16_t value,
    CanopenSdoOwner_t owner,
    uint32_t now_ms,
    uint32_t timeout_ms);
CanopenAsyncStartResult_t Canopen_AsyncWriteI8(
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    int8_t value,
    CanopenSdoOwner_t owner,
    uint32_t now_ms,
    uint32_t timeout_ms);
CanopenAsyncStartResult_t Canopen_AsyncWriteI32(
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    int32_t value,
    CanopenSdoOwner_t owner,
    uint32_t now_ms,
    uint32_t timeout_ms);
CanopenAsyncStartResult_t Canopen_AsyncWriteU32(
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    uint32_t value,
    CanopenSdoOwner_t owner,
    uint32_t now_ms,
    uint32_t timeout_ms);
CanopenAsyncStartResult_t Canopen_AsyncReadU16(
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    CanopenSdoOwner_t owner,
    uint32_t now_ms,
    uint32_t timeout_ms);
CanopenAsyncStartResult_t Canopen_AsyncReadU8(
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    CanopenSdoOwner_t owner,
    uint32_t now_ms,
    uint32_t timeout_ms);
CanopenAsyncStartResult_t Canopen_AsyncReadU32(
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    CanopenSdoOwner_t owner,
    uint32_t now_ms,
    uint32_t timeout_ms);
CanopenAsyncStartResult_t Canopen_AsyncReadI8(
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    CanopenSdoOwner_t owner,
    uint32_t now_ms,
    uint32_t timeout_ms);
CanopenAsyncResult_t Canopen_AsyncPoll(
    uint8_t node_id,
    CanopenSdoOwner_t owner,
    uint32_t now_ms,
    uint32_t *value,
    uint32_t *abort_code);
/* As AsyncPoll, with the original response dispatch tick for a completed
 * transaction. Outputs are unchanged for pending/timeout/idle/error states
 * without a received completion. A valid tick may be zero or wrap normally. */
CanopenAsyncResult_t Canopen_AsyncPollWithTimestamp(
    uint8_t node_id,
    CanopenSdoOwner_t owner,
    uint32_t now_ms,
    uint32_t *value,
    uint32_t *abort_code,
    uint32_t *rx_ms_out);
void Canopen_CancelAsyncOwner(CanopenSdoOwner_t owner, uint32_t now_ms);
void Canopen_RecordFaultMonitorDeferred(uint32_t now_ms);
bool Canopen_TakeWheelRuntimeFault(void);
bool Canopen_TakeWheelFeedbackContinuityFault(void);
bool Canopen_WheelRuntimeFaultPending(void);
bool Canopen_WheelFeedbackContinuityFaultPending(void);
void Canopen_HeartbeatResetForExplicitEnable(uint32_t now_ms);
bool Canopen_HeartbeatSetConfigured(uint8_t node_id, uint32_t now_ms);
void Canopen_HeartbeatService(uint32_t now_ms);
bool Canopen_HeartbeatIsQualified(uint8_t node_id);
bool Canopen_TakeHeartbeatFault(uint8_t *node_id,
                                CanopenHeartbeatFaultReason_t *reason);
bool Canopen_HeartbeatFaultPending(void);
/* Main-context event for a nonzero CANopen EMCY from Lift node 3. */
bool Canopen_TakeLiftFaultEmcy(uint16_t *error_code);
CanopenRuntimeDiagnostics_t Canopen_GetRuntimeDiagnostics(void);

bool SDO_Write_U8_Node(const CanopenNode_t *node,
                       uint16_t index,
                       uint8_t subindex,
                       uint8_t value);
bool SDO_Write_U8_Timeout_Node(const CanopenNode_t *node,
                               uint16_t index,
                               uint8_t subindex,
                               uint8_t value,
                               uint32_t timeout_ms);
bool SDO_Write_I8_Node(const CanopenNode_t *node,
                       uint16_t index,
                       uint8_t subindex,
                       int8_t value);
bool SDO_Write_I8_Timeout_Node(const CanopenNode_t *node,
                               uint16_t index,
                               uint8_t subindex,
                               int8_t value,
                               uint32_t timeout_ms);
bool SDO_Write_U16_Node(const CanopenNode_t *node,
                        uint16_t index,
                        uint8_t subindex,
                        uint16_t value);
bool SDO_Write_U16_Timeout_Node(const CanopenNode_t *node,
                                uint16_t index,
                                uint8_t subindex,
                                uint16_t value,
                                uint32_t timeout_ms);
bool SDO_Write_I32_Node(const CanopenNode_t *node,
                        uint16_t index,
                        uint8_t subindex,
                        int32_t value);
bool SDO_Write_I32_Timeout_Node(const CanopenNode_t *node,
                                uint16_t index,
                                uint8_t subindex,
                                int32_t value,
                                uint32_t timeout_ms);
bool SDO_Write_U32_Node(const CanopenNode_t *node,
                        uint16_t index,
                        uint8_t subindex,
                        uint32_t value);
bool SDO_Read_U16_Node(const CanopenNode_t *node,
                       uint16_t index,
                       uint8_t subindex,
                       uint16_t *out_value);
bool SDO_Read_U16_Timeout_Node(const CanopenNode_t *node,
                              uint16_t index,
                              uint8_t subindex,
                              uint16_t *out_value,
                              uint32_t timeout_ms);
bool SDO_Read_I8_Node(const CanopenNode_t *node,
                      uint16_t index,
                      uint8_t subindex,
                      int8_t *out_value);
bool SDO_Read_I8_Timeout_Node(const CanopenNode_t *node,
                             uint16_t index,
                             uint8_t subindex,
                             int8_t *out_value,
                             uint32_t timeout_ms);
bool SDO_Read_I8_Quiet_Node(const CanopenNode_t *node,
                           uint16_t index,
                           uint8_t subindex,
                           int8_t *out_value);
bool SDO_Read_U16_Quiet_Node(const CanopenNode_t *node,
                             uint16_t index,
                             uint8_t subindex,
                             uint16_t *out_value);
bool SDO_Read_I32_Timeout_Node(const CanopenNode_t *node,
                               uint16_t index,
                               uint8_t subindex,
                               int32_t *out_value,
                               uint32_t timeout_ms);
bool SDO_Read_I32_Quiet_Node(const CanopenNode_t *node,
                             uint16_t index,
                             uint8_t subindex,
                             int32_t *out_value);

bool SDO_Write_I8(uint8_t node_id, uint16_t index, uint8_t subindex, int8_t value);
bool SDO_Write_U16(uint8_t node_id, uint16_t index, uint8_t subindex, uint16_t value);
bool SDO_Write_I32(uint8_t node_id, uint16_t index, uint8_t subindex, int32_t value);
bool SDO_Write_U32(uint8_t node_id, uint16_t index, uint8_t subindex, uint32_t value);
bool SDO_Read_U16(uint8_t node_id, uint16_t index, uint8_t subindex, uint16_t *out_value);
bool SDO_Read_I8(uint8_t node_id, uint16_t index, uint8_t subindex, int8_t *out_value);
bool SDO_Read_I32_Timeout(uint8_t node_id,
                          uint16_t index,
                          uint8_t subindex,
                          int32_t *out_value,
                          uint32_t timeout_ms);
bool SDO_Read_I32(uint8_t node_id, uint16_t index, uint8_t subindex, int32_t *out_value);
bool SDO_Read_I32_Quiet(uint8_t node_id, uint16_t index, uint8_t subindex, int32_t *out_value);

#ifdef AMR_HOST_TEST
void Canopen_TestResetState(void);
void Canopen_TestSetWheelFeedbackTransport(
    CanopenWheelFeedbackTransport_t transport);
#endif

#ifdef __cplusplus
}
#endif

#endif /* CANOPEN_H */
