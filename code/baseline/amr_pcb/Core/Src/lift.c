/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : lift.c
  * @brief          : Lift/top actuator state machine and commands.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "lift.h"
#include "app_config.h"
#include "canopen.h"
#include "diagnostics.h"
#include "drive.h"
#include "firmware_identity.h"
#include "jetson_tx.h"
#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef enum
{
    LIFT_STATE_UNKNOWN = 0,
    LIFT_STATE_RECOVERY_REQUIRED,
    LIFT_STATE_IDLE_DOWN,
    LIFT_STATE_IDLE_UP,
    LIFT_STATE_MOVING_UP,
    LIFT_STATE_MOVING_DOWN,
    LIFT_STATE_RECOVERING_UP,
    LIFT_STATE_RECOVERING_DOWN,
    LIFT_STATE_STOPPED,
    LIFT_STATE_FAULT
} LiftState_t;

typedef enum
{
    LIFT_RECOVERY_PHASE_NONE = 0,
    LIFT_RECOVERY_PHASE_JOG,
    LIFT_RECOVERY_PHASE_SENSOR_HOMING
} LiftRecoveryPhase_t;

typedef enum
{
    HOME_PREFLIGHT = 0,
    HOME_CLEAR_FAULT_ASSERT,
    HOME_CLEAR_FAULT_RELEASE,
    HOME_WAIT_READY,
    HOME_CONFIG_SEARCH_SPEED,
    HOME_CONFIG_FINAL_SPEED,
    HOME_CONFIG_ACCEL,
    HOME_REQUEST_MODE,
    HOME_SWITCH_ON,
    HOME_ENABLE_OPERATION,
    HOME_WAIT_MODE_DISPLAY,
    HOME_PREPARE_START_EDGE,
    HOME_START_EDGE,
    HOME_RUNNING,
    HOME_COMPLETE,
    HOME_FAILED
} LiftHomePhase_t;

typedef enum
{
    LIFT_PENDING_NONE = 0,
    LIFT_PENDING_HOME,
    LIFT_PENDING_MOVE_UP,
    LIFT_PENDING_MOVE_DOWN,
    LIFT_PENDING_JOG_UP,
    LIFT_PENDING_JOG_DOWN,
    LIFT_PENDING_FAULT_RESET,
    LIFT_PENDING_CANCEL_STOP
} LiftPendingAction_t;

typedef enum
{
    LIFT_ACTION_PHASE_NONE = 0,
    LIFT_ACTION_WAIT_BASE_STOP,
    LIFT_ACTION_READ_STATUS,
    LIFT_ACTION_READ_ERROR,
    LIFT_ACTION_PROFILE_WRITE_MODE,
    LIFT_ACTION_PROFILE_WRITE_TARGET,
    LIFT_ACTION_PROFILE_WRITE_VELOCITY,
    LIFT_ACTION_PROFILE_WRITE_ACCEL,
    LIFT_ACTION_PROFILE_WRITE_DECEL,
    LIFT_ACTION_JOG_WRITE_MODE,
    LIFT_ACTION_JOG_WRITE_ZERO,
    LIFT_ACTION_JOG_WRITE_ACCEL,
    LIFT_ACTION_JOG_WRITE_DECEL,
    LIFT_ACTION_CIA402_WRITE_0000,
    LIFT_ACTION_CIA402_WAIT_0000,
    LIFT_ACTION_CIA402_WRITE_0006,
    LIFT_ACTION_CIA402_WAIT_0006,
    LIFT_ACTION_CIA402_WRITE_0007,
    LIFT_ACTION_CIA402_WAIT_0007,
    LIFT_ACTION_CIA402_WRITE_000F,
    LIFT_ACTION_CIA402_WAIT_000F,
    LIFT_ACTION_PROFILE_WRITE_START,
    LIFT_ACTION_JOG_WRITE_TARGET,
    LIFT_ACTION_RESET_ASSERT,
    LIFT_ACTION_RESET_WAIT_ASSERT,
    LIFT_ACTION_RESET_RELEASE,
    LIFT_ACTION_RESET_WAIT_SETTLE,
    LIFT_ACTION_RESET_VERIFY_STATUS,
    LIFT_ACTION_RESET_VERIFY_ERROR,
    LIFT_ACTION_CANCEL_WRITE_JOG_ZERO,
    LIFT_ACTION_CANCEL_WRITE_DECEL,
    LIFT_ACTION_CANCEL_WRITE_DISABLE
} LiftActionPhase_t;

typedef enum
{
    LIFT_ACTION_STEP_WAIT = 0,
    LIFT_ACTION_STEP_DONE,
    LIFT_ACTION_STEP_FAILED
} LiftActionStepResult_t;

typedef enum
{
    LIFT_FAULT_REASON_NONE = 0,
    LIFT_FAULT_REASON_NODE3_COMM_TIMEOUT = 0xE101,
    LIFT_FAULT_REASON_DRIVE_FAULT = 0xE102,
    LIFT_FAULT_REASON_CIA402_READY_TIMEOUT = 0xE103,
    LIFT_FAULT_REASON_HOMING_MODE_DISPLAY_TIMEOUT = 0xE104,
    LIFT_FAULT_REASON_HOMING_START_FAILED = 0xE105,
    LIFT_FAULT_REASON_HOMING_ERROR = 0xE106,
    LIFT_FAULT_REASON_HOMING_TIMEOUT = 0xE107,
    LIFT_FAULT_REASON_STALE_REFERENCE_STATE = 0xE108,
    LIFT_FAULT_REASON_HOMING_MODE_LOSS = 0xE109,
    LIFT_FAULT_REASON_POSITION_READ_FAILED = 0xE10A,
    LIFT_FAULT_REASON_CIA402_SWITCHED_ON_TIMEOUT = 0xE10B,
    LIFT_FAULT_REASON_CIA402_OPERATION_ENABLED_TIMEOUT = 0xE10C,
    LIFT_FAULT_REASON_ABORTED_BY_STOP = 0xE10D,
    LIFT_FAULT_REASON_BASE_STOP_FAILED = 0xE10E,
    LIFT_FAULT_REASON_ACTION_START_FAILED = 0xE10F
} LiftFaultReason_t;

typedef struct
{
    bool status_ok;
    uint16_t statusword;
    bool error_ok;
    uint16_t error_code;
} LiftDriveSnapshot_t;

typedef struct
{
    LiftPendingAction_t action;
    LiftActionPhase_t phase;
    uint32_t phase_start_ms;
    int32_t target_pul;
    LiftState_t moving_state;
    bool upward;
    bool request_pending;
    uint16_t statusword;
    uint16_t error_code;
} LiftPendingContext_t;

extern FDCAN_HandleTypeDef hfdcan1;

static const CanopenNode_t s_lift_node = { &hfdcan1, LIFT_NODE_ID };
static LiftState_t s_lift_state = LIFT_STATE_RECOVERY_REQUIRED;
static LiftRecoveryPhase_t s_lift_recovery_phase = LIFT_RECOVERY_PHASE_NONE;
static bool s_lift_homed = false;
static int32_t s_lift_pos_pul = 0;
static int32_t s_lift_target_pul = LIFT_DOWN_PUL;
static uint32_t s_lift_fault_code = 0U;
static uint32_t s_lift_move_start_ms = 0U;
static uint32_t s_lift_last_poll_ms = 0U;
static uint32_t s_lift_health_last_poll_ms = 0U;
static uint8_t s_lift_health_missed_count = 0U;
static uint32_t s_lift_recovery_start_ms = 0U;
static uint32_t s_lift_recovery_last_poll_ms = 0U;
static uint32_t s_lift_last_target_mismatch_ms = 0U;
static LiftFaultReason_t s_lift_fault_reason = LIFT_FAULT_REASON_NONE;
static LiftHomePhase_t s_lift_home_phase = HOME_FAILED;
static uint32_t s_lift_home_phase_start_ms = 0U;
static uint32_t s_lift_home_last_poll_ms = 0U;
static bool s_lift_home_action_started = false;
static bool s_lift_home_request_pending = false;
static bool s_lift_homing_start_edge_issued = false;
static int8_t s_lift_home_last_displayed_mode = 0;
static bool s_lift_home_display_seen = false;
static uint16_t s_homing_pre_start_statusword = 0U;
static bool s_homing_reference_seen_clear = false;
static uint32_t s_lift_jog_last_command_ms = 0U;
static bool s_lift_jog_upward = true;
static LiftPendingContext_t s_lift_pending;

/* Protocol-v4 host transaction binding for LIFT_FAULT_RESET and LIFT_HOME.
 * Each accepted command gets a per-command monotonic txn= and echoes the
 * host req= nonce on every STARTED/PENDING/COMPLETE/FAILED line, so a
 * stale response still queued in the TX manager can never satisfy a new
 * command. Legacy unbound lines remain as diagnostics only. */
static uint32_t s_lift_txn_counter;
static uint32_t s_lift_reset_txn;
static uint32_t s_lift_reset_nonce;
static bool s_lift_reset_bound;
static uint32_t s_lift_home_txn;
static uint32_t s_lift_home_nonce;
static bool s_lift_home_bound;
static uint32_t s_lift_command_nonce;

void Lift_SetCommandRequestNonce(uint32_t request_nonce)
{
    s_lift_command_nonce = request_nonce;
}

static uint32_t Lift_ActiveBoundTxn(void)
{
    if (s_lift_reset_bound) {
        return s_lift_reset_txn;
    }
    if (s_lift_home_bound) {
        return s_lift_home_txn;
    }
    return 0U;
}

static void Lift_EmitResetFailed(const char *reason)
{
    if (!s_lift_reset_bound) {
        return;
    }
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_STATUS,
        "LIFT_FAULT_RESET state=FAILED reason=%s txn=%lu req=%08lX\r\n",
        reason,
        (unsigned long)s_lift_reset_txn,
        (unsigned long)s_lift_reset_nonce);
    s_lift_reset_bound = false;
}

static void Lift_EmitHomeFailed(const char *reason)
{
    if (!s_lift_home_bound) {
        return;
    }
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_STATUS,
        "LIFT_HOME state=FAILED reason=%s txn=%lu req=%08lX\r\n",
        reason,
        (unsigned long)s_lift_home_txn,
        (unsigned long)s_lift_home_nonce);
    s_lift_home_bound = false;
}

typedef struct
{
    bool status_ok;
    uint16_t statusword;
    bool error_ok;
    uint16_t error_code;
    bool display_ok;
    int8_t display;
} LiftRecoveryDiag_t;

static bool Lift_ReadStatusQuiet(uint16_t *statusword);
static bool Lift_ReadActualPositionQuiet(int32_t *position_pul);
static bool Lift_ResetFault(void);
static void Lift_ServicePendingAction(uint32_t now);
static void Lift_SendJetsonStatusLine(void);
static void Lift_SetFaultReasonWithoutDriveRead(LiftFaultReason_t reason);

static bool Lift_IsProfileMoving(void)
{
    return (s_lift_state == LIFT_STATE_MOVING_UP) ||
           (s_lift_state == LIFT_STATE_MOVING_DOWN);
}

static bool Lift_IsRecoveryActive(void)
{
    return s_lift_recovery_phase != LIFT_RECOVERY_PHASE_NONE;
}

static bool Lift_IsSensorHomingActive(void)
{
    return s_lift_recovery_phase == LIFT_RECOVERY_PHASE_SENSOR_HOMING;
}

static bool Statusword_ReferenceFound(uint16_t statusword)
{
    return (statusword & (1U << 15)) != 0U;
}

static bool Statusword_HasFault(uint16_t statusword)
{
    return (statusword & (1U << 3)) != 0U;
}

static bool Statusword_HasHomingError(uint16_t statusword)
{
    return (statusword & LIFT_HOMING_ERROR_STATUS_MASK) != 0U;
}

static const char *Lift_FaultReasonName(LiftFaultReason_t reason)
{
    switch (reason) {
        case LIFT_FAULT_REASON_NONE: return "NONE";
        case LIFT_FAULT_REASON_NODE3_COMM_TIMEOUT: return "NODE3_COMM_TIMEOUT";
        case LIFT_FAULT_REASON_DRIVE_FAULT: return "DRIVE_FAULT";
        case LIFT_FAULT_REASON_CIA402_READY_TIMEOUT: return "CIA402_READY_TIMEOUT";
        case LIFT_FAULT_REASON_HOMING_MODE_DISPLAY_TIMEOUT: return "HOMING_MODE_DISPLAY_TIMEOUT";
        case LIFT_FAULT_REASON_HOMING_START_FAILED: return "HOMING_START_FAILED";
        case LIFT_FAULT_REASON_HOMING_ERROR: return "HOMING_ERROR";
        case LIFT_FAULT_REASON_HOMING_TIMEOUT: return "HOMING_TIMEOUT";
        case LIFT_FAULT_REASON_STALE_REFERENCE_STATE: return "STALE_REFERENCE_STATE";
        case LIFT_FAULT_REASON_HOMING_MODE_LOSS: return "HOMING_MODE_LOSS";
        case LIFT_FAULT_REASON_POSITION_READ_FAILED: return "POSITION_READ_FAILED";
        case LIFT_FAULT_REASON_CIA402_SWITCHED_ON_TIMEOUT: return "CIA402_SWITCHED_ON_TIMEOUT";
        case LIFT_FAULT_REASON_CIA402_OPERATION_ENABLED_TIMEOUT: return "CIA402_OPERATION_ENABLED_TIMEOUT";
        case LIFT_FAULT_REASON_ABORTED_BY_STOP: return "ABORTED_BY_STOP";
        case LIFT_FAULT_REASON_BASE_STOP_FAILED: return "BASE_STOP_FAILED";
        case LIFT_FAULT_REASON_ACTION_START_FAILED: return "ACTION_START_FAILED";
        default: return "UNKNOWN";
    }
}

static const char *Lift_StateName(LiftState_t state)
{
    switch (state) {
        case LIFT_STATE_UNKNOWN: return "UNKNOWN";
        case LIFT_STATE_RECOVERY_REQUIRED: return "RECOVERY_REQUIRED";
        case LIFT_STATE_IDLE_DOWN: return "IDLE_DOWN";
        case LIFT_STATE_IDLE_UP: return "IDLE_UP";
        case LIFT_STATE_MOVING_UP: return "MOVING_UP";
        case LIFT_STATE_MOVING_DOWN: return "MOVING_DOWN";
        case LIFT_STATE_RECOVERING_UP: return "RECOVERING_UP";
        case LIFT_STATE_RECOVERING_DOWN: return "RECOVERING_DOWN";
        case LIFT_STATE_STOPPED: return "STOPPED";
        case LIFT_STATE_FAULT: return "FAULT";
        default: return "UNKNOWN";
    }
}

static const char *Lift_RecoveryPhaseName(LiftRecoveryPhase_t phase)
{
    if (s_lift_pending.action != LIFT_PENDING_NONE) {
        return (s_lift_pending.phase == LIFT_ACTION_WAIT_BASE_STOP) ?
            "WAIT_BASE_STOP" : "ACTION_START";
    }
    switch (phase) {
        case LIFT_RECOVERY_PHASE_NONE: return "NONE";
        case LIFT_RECOVERY_PHASE_JOG: return "JOG";
        case LIFT_RECOVERY_PHASE_SENSOR_HOMING:
            switch (s_lift_home_phase) {
                case HOME_PREFLIGHT: return "HOME_PREFLIGHT";
                case HOME_CLEAR_FAULT_ASSERT: return "HOME_CLEAR_FAULT_ASSERT";
                case HOME_CLEAR_FAULT_RELEASE: return "HOME_CLEAR_FAULT_RELEASE";
                case HOME_WAIT_READY: return "HOME_WAIT_READY";
                case HOME_CONFIG_SEARCH_SPEED: return "HOME_CONFIG_SEARCH_SPEED";
                case HOME_CONFIG_FINAL_SPEED: return "HOME_CONFIG_FINAL_SPEED";
                case HOME_CONFIG_ACCEL: return "HOME_CONFIG_ACCEL";
                case HOME_REQUEST_MODE: return "HOME_REQUEST_MODE";
                case HOME_SWITCH_ON: return "HOME_SWITCH_ON";
                case HOME_ENABLE_OPERATION: return "HOME_ENABLE_OPERATION";
                case HOME_WAIT_MODE_DISPLAY: return "HOME_WAIT_MODE_DISPLAY";
                case HOME_PREPARE_START_EDGE: return "HOME_PREPARE_START_EDGE";
                case HOME_START_EDGE: return "HOME_START_EDGE";
                case HOME_RUNNING: return "HOME_RUNNING";
                case HOME_COMPLETE: return "HOME_COMPLETE";
                case HOME_FAILED: return "HOME_FAILED";
                default: return "SENSOR_HOMING";
            }
        default: return "UNKNOWN";
    }
}

static void Lift_SetState(LiftState_t new_state)
{
    if (s_lift_state != new_state) {
        uart_printf("LIFT STATE: %s -> %s\r\n",
                    Lift_StateName(s_lift_state),
                    Lift_StateName(new_state));
        s_lift_state = new_state;
    }
}

bool Lift_IsMoving(void)
{
    return (s_lift_pending.action != LIFT_PENDING_NONE) ||
           Lift_IsProfileMoving() ||
           Lift_IsRecoveryActive();
}

bool Lift_IsFaulted(void)
{
    return s_lift_state == LIFT_STATE_FAULT;
}

uint32_t Lift_GetStateForDiagnostics(void)
{
    return (uint32_t)s_lift_state;
}

static float Lift_PulsesToMm(int32_t pulses)
{
    return (float)pulses / LIFT_PULSES_PER_MM;
}

static long Lift_RoundedMmFromPul(int32_t pulses)
{
    float mm = Lift_PulsesToMm(pulses);

    if (mm >= 0.0f) {
        return (long)(mm + 0.5f);
    }

    return (long)(mm - 0.5f);
}

static int32_t Lift_AbsI32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int32_t Lift_PositionTolerancePul(void)
{
    return (int32_t)(LIFT_POS_TOL_MM * LIFT_PULSES_PER_MM);
}

static LiftState_t Lift_ClassifyRelativePosition(int32_t rel_pul)
{
    if (Lift_AbsI32(rel_pul - LIFT_DOWN_PUL) <= Lift_PositionTolerancePul()) {
        return LIFT_STATE_IDLE_DOWN;
    }

    if (Lift_AbsI32(rel_pul - LIFT_UP_PUL) <= Lift_PositionTolerancePul()) {
        return LIFT_STATE_IDLE_UP;
    }

    return LIFT_STATE_STOPPED;
}

static int32_t Lift_JogSpeedPulS(bool upward)
{
    int32_t speed_pul_s = (int32_t)(LIFT_SERVICE_JOG_SPEED_MM_S * LIFT_PULSES_PER_MM);

    if (!upward) {
        speed_pul_s = -speed_pul_s;
    }

    return speed_pul_s;
}

static uint32_t Lift_JogAccelPulS2(void)
{
    return (uint32_t)(LIFT_SERVICE_JOG_ACCEL_MM_S2 * LIFT_PULSES_PER_MM);
}

static int8_t Lift_JogVelocityMode(void)
{
    return (LIFT_JOG_USE_KINCO_MINUS3_MODE != 0U) ? (int8_t)-3 : (int8_t)3;
}

static void Lift_FormatFaultText(char *buf, size_t buf_size)
{
    if (s_lift_fault_code != 0U) {
        (void)snprintf(buf, buf_size, "0x%lX", (unsigned long)s_lift_fault_code);
    } else {
        (void)snprintf(buf, buf_size, "0");
    }
}

static void Lift_FormatI8Text(char *buf, size_t buf_size, bool ok, int8_t value)
{
    if (ok) {
        (void)snprintf(buf, buf_size, "%d", (int)value);
    } else {
        (void)snprintf(buf, buf_size, "-");
    }
}

static void Lift_FormatU16HexText(char *buf, size_t buf_size, bool ok, uint16_t value)
{
    if (ok) {
        (void)snprintf(buf, buf_size, "0x%04X", value);
    } else {
        (void)snprintf(buf, buf_size, "-");
    }
}

static void Lift_ReadRecoveryDiag(LiftRecoveryDiag_t *diag)
{
    diag->statusword = 0U;
    diag->error_code = 0U;
    diag->display = 0;
    diag->status_ok = Lift_ReadStatusQuiet(&diag->statusword);
    diag->error_ok = SDO_Read_U16_Quiet_Node(&s_lift_node,
                                             0x603F,
                                             0x00,
                                             &diag->error_code);
    diag->display_ok = SDO_Read_I8_Quiet_Node(&s_lift_node, 0x6061, 0x00, &diag->display);
}

static void Lift_PrintRecoveryFailure(const char *message,
                                      uint32_t now,
                                      const LiftRecoveryDiag_t *diag)
{
    char sw_text[16];
    char error_text[16];
    char display_text[16];

    Lift_FormatU16HexText(sw_text, sizeof(sw_text), diag->status_ok, diag->statusword);
    Lift_FormatU16HexText(error_text, sizeof(error_text), diag->error_ok, diag->error_code);
    Lift_FormatI8Text(display_text, sizeof(display_text), diag->display_ok, diag->display);

    JetsonTx_Printf(JETSON_TX_PRIORITY_CRITICAL,
                "LIFT recovery fail msg=\"%s\" phase=%s elapsed=%lu sw=%s error=%s display=%s\r\n",
                message,
                Lift_RecoveryPhaseName(s_lift_recovery_phase),
                (unsigned long)(now - s_lift_recovery_start_ms),
                sw_text,
                error_text,
                display_text);
}

static void Lift_RecordFault(uint32_t fallback_code)
{
    uint16_t error_code = 0U;

    if (SDO_Read_U16_Quiet_Node(&s_lift_node, 0x603F, 0x00, &error_code) &&
        (error_code != 0U)) {
        s_lift_fault_code = error_code;
    } else {
        s_lift_fault_code = fallback_code;
    }
}

static void Lift_SetFaultReason(LiftFaultReason_t reason)
{
    s_lift_fault_reason = reason;
    Lift_RecordFault((uint32_t)reason);
    JetsonTx_Printf(JETSON_TX_PRIORITY_CRITICAL,
                    "LIFT reason=%s\r\n", Lift_FaultReasonName(reason));
}

static void Lift_ClearFaultReason(void)
{
    s_lift_fault_reason = LIFT_FAULT_REASON_NONE;
    s_lift_fault_code = 0U;
}

static void Lift_ReadDriveSnapshot(LiftDriveSnapshot_t *snapshot)
{
    snapshot->statusword = 0U;
    snapshot->error_code = 0U;
    snapshot->status_ok = SDO_Read_U16_Quiet_Node(&s_lift_node,
                                                  0x6041,
                                                  0x00,
                                                  &snapshot->statusword);
    snapshot->error_ok = SDO_Read_U16_Quiet_Node(&s_lift_node,
                                                 0x603F,
                                                 0x00,
                                                 &snapshot->error_code);
}

static bool Lift_ReadStatusQuiet(uint16_t *statusword)
{
    bool ok = SDO_Read_U16_Quiet_Node(&s_lift_node, 0x6041, 0x00, statusword);
    if (ok) {
        s_lift_health_missed_count = 0U;
    }
    return ok;
}

static void Lift_RecordHealthReadFailure(void)
{
    if (s_lift_health_missed_count < UINT8_MAX) {
        s_lift_health_missed_count++;
    }
    if (s_lift_health_missed_count >= LIFT_HEALTH_MISSED_LIMIT) {
        /* The node may have lost power: do not perform another blocking SDO
         * read while publishing the communication fault. */
        Lift_SetFaultReasonWithoutDriveRead(
            LIFT_FAULT_REASON_NODE3_COMM_TIMEOUT);
        Lift_SetState(LIFT_STATE_FAULT);
        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_CRITICAL,
            "LIFT health state=FAULT reason=NODE3_COMM_TIMEOUT misses=%u\r\n",
            (unsigned)s_lift_health_missed_count);
        Lift_SendJetsonStatusLine();
    }
}

static void Lift_UpdateIdleHealth(uint32_t now)
{
    uint16_t statusword = 0U;

    if (s_lift_state == LIFT_STATE_FAULT) {
        return;
    }
    if ((now - s_lift_health_last_poll_ms) < LIFT_HEALTH_POLL_MS) {
        return;
    }
    s_lift_health_last_poll_ms = now;

    if (Lift_ReadStatusQuiet(&statusword)) {
        if (Statusword_HasFault(statusword)) {
            Lift_RecordFault(0x0001U);
            Lift_SetState(LIFT_STATE_FAULT);
            Lift_SendJetsonStatusLine();
        }
        return;
    }

    Lift_RecordHealthReadFailure();
}

static bool Lift_ReadActualPositionQuiet(int32_t *position_pul)
{
    return SDO_Read_I32_Quiet_Node(&s_lift_node, 0x6064, 0x00, position_pul);
}

static void Lift_RefreshCachedStatus(void)
{
    uint16_t statusword = 0U;
    int32_t position_pul = 0;

    if (Lift_ReadStatusQuiet(&statusword) && Statusword_HasFault(statusword)) {
        Lift_RecordFault(0x0001U);
        Lift_SetState(LIFT_STATE_FAULT);
    }

    if (s_lift_homed && Lift_ReadActualPositionQuiet(&position_pul)) {
        s_lift_pos_pul = position_pul;
        if (!Lift_IsProfileMoving() &&
            !Lift_IsRecoveryActive() &&
            (s_lift_state != LIFT_STATE_FAULT)) {
            Lift_SetState(Lift_ClassifyRelativePosition(s_lift_pos_pul));
        }
    }
}

static void Lift_SendJetsonStatusLine(void)
{
    char fault_text[16];
    char msg[128];

    Lift_FormatFaultText(fault_text, sizeof(fault_text));

    int len = snprintf(msg, sizeof(msg),
                       "LIFT %s %ld %ld %ld %u %s\r\n",
                       Lift_StateName(s_lift_state),
                       Lift_RoundedMmFromPul(s_lift_pos_pul),
                       Lift_RoundedMmFromPul(s_lift_target_pul),
                       (long)s_lift_pos_pul,
                       s_lift_homed ? 1U : 0U,
                       fault_text);

    if (len > 0) {
        if (len > (int)sizeof(msg)) {
            len = sizeof(msg) - 1;
        }
        JetsonTxPriority_t priority = (s_lift_state == LIFT_STATE_FAULT) ?
            JETSON_TX_PRIORITY_CRITICAL : JETSON_TX_PRIORITY_STATUS;
        (void)JetsonTx_Send(priority, msg, (uint16_t)len);
    }
}

static void Lift_SendRawStatus(void)
{
    uint16_t statusword = 0U;
    uint16_t error_code = 0U;
    int8_t requested_mode = 0;
    int8_t display = 0;
    int8_t homing_method = 0;
    bool status_ok = false;
    bool error_ok = false;
    bool requested_mode_ok = false;
    bool display_ok = false;
    bool homing_method_ok = false;
    char sw_text[16];
    char error_text[16];
    char requested_mode_text[16];
    char display_text[16];
    char homing_method_text[16];

    /* Do not let a diagnostic command block the cooperative base-stop
     * preflight. The wire format remains unchanged; unavailable fields are
     * rendered as '-'. */
    if (s_lift_pending.phase != LIFT_ACTION_WAIT_BASE_STOP) {
        status_ok = SDO_Read_U16_Quiet_Node(&s_lift_node,
                                            0x6041,
                                            0x00,
                                            &statusword);
        error_ok = SDO_Read_U16_Quiet_Node(&s_lift_node,
                                           0x603F,
                                           0x00,
                                           &error_code);
        requested_mode_ok = SDO_Read_I8_Quiet_Node(&s_lift_node,
                                                   0x6060,
                                                   0x00,
                                                   &requested_mode);
        display_ok = SDO_Read_I8_Quiet_Node(&s_lift_node,
                                            0x6061,
                                            0x00,
                                            &display);
        homing_method_ok = SDO_Read_I8_Quiet_Node(&s_lift_node,
                                                  0x6098,
                                                  0x00,
                                                  &homing_method);
    }

    Lift_FormatU16HexText(sw_text, sizeof(sw_text), status_ok, statusword);
    Lift_FormatU16HexText(error_text, sizeof(error_text), error_ok, error_code);
    Lift_FormatI8Text(requested_mode_text,
                      sizeof(requested_mode_text),
                      requested_mode_ok,
                      requested_mode);
    Lift_FormatI8Text(display_text, sizeof(display_text), display_ok, display);
    Lift_FormatI8Text(homing_method_text,
                      sizeof(homing_method_text),
                      homing_method_ok,
                      homing_method);

    /* The %.31s caps make both complete CRLF-terminated lines provably fit in
     * uart_printf's 160-byte buffer (152 and 156 bytes worst case). */
    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                "LIFT_RAW 1/2 fw=%.31s state=%.31s phase=%.31s homed=%u home_method=SENSOR\r\n",
                FIRMWARE_VARIANT,
                Lift_StateName(s_lift_state),
                Lift_RecoveryPhaseName(s_lift_recovery_phase),
                s_lift_homed ? 1U : 0U);
    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                "LIFT_RAW 2/2 statusword=%s error_code=%s requested_mode=%s displayed_mode=%s homing_method=%s reason=%.39s\r\n",
                sw_text,
                error_text,
                requested_mode_text,
                display_text,
                homing_method_text,
                Lift_FaultReasonName(s_lift_fault_reason));
}

static void Lift_SendScaleInfo(void)
{
    long pulses_per_rev_x10 = (long)(LIFT_PULSES_PER_MOTOR_REV * 10.0f);
    long reducer_x10 = (long)(LIFT_REDUCER_RATIO * 10.0f);
    long screw_lead_um = (long)(LIFT_SCREW_LEAD_MM * 1000.0f);
    long counts_per_mm_x1000 = (long)(LIFT_PULSES_PER_MM * 1000.0f);

    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                "LIFT_SCALE_INFO 1/2 pprev_x10=%ld reducer_x10=%ld screw_lead_um=%ld cpm_x1000=%ld calibrated=0\r\n",
                pulses_per_rev_x10,
                reducer_x10,
                screw_lead_um,
                counts_per_mm_x1000);
    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                "LIFT_SCALE_INFO 2/2 profile_pul_s=%lu accel_pul_s2=%lu decel_pul_s2=%lu verify_6064_travel=1\r\n",
                (unsigned long)LIFT_PROFILE_VELOCITY_PUL_S,
                (unsigned long)LIFT_PROFILE_ACCEL_PUL_S2,
                (unsigned long)LIFT_PROFILE_DECEL_PUL_S2);
}

void Lift_SendJetsonStatusNow(void)
{
    if (s_lift_pending.phase != LIFT_ACTION_WAIT_BASE_STOP) {
        Lift_RefreshCachedStatus();
    }
    Lift_SendJetsonStatusLine();
}

void Lift_SendJetsonStatus_100ms(void)
{
    static uint32_t last_status_ms = 0U;
    uint32_t now = HAL_GetTick();

    if ((now - last_status_ms) < LIFT_STATUS_PERIOD_MS) {
        return;
    }
    last_status_ms = now;

    Lift_SendJetsonStatusLine();
}

static bool Lift_WriteControlword(uint16_t controlword)
{
    return SDO_Write_U16_Timeout_Node(&s_lift_node,
                                      0x6040,
                                      0x00,
                                      controlword,
                                      LIFT_ACTION_SDO_TIMEOUT_MS);
}

static bool Lift_RequestDecelStop(void)
{
    if (Lift_WriteControlword(0x010FU)) {
        return true;
    }

    return Lift_WriteControlword(0x0002U);
}

static bool Lift_StopRecoveryJog(void)
{
    bool ok = SDO_Write_I32_Timeout_Node(&s_lift_node,
                                         0x60FF,
                                         0x00,
                                         0,
                                         LIFT_ACTION_SDO_TIMEOUT_MS);

    if (!Lift_RequestDecelStop()) {
        ok = false;
    }

    return ok;
}

static void Lift_SetActionPhase(LiftActionPhase_t phase, uint32_t now)
{
    s_lift_pending.phase = phase;
    s_lift_pending.phase_start_ms = now;
    s_lift_pending.request_pending = false;
    /* Bound-transaction progress. Phase names are the bounded numeric
     * pending-action phase ids (P<n>). */
    if (s_lift_reset_bound &&
        (s_lift_pending.action == LIFT_PENDING_FAULT_RESET)) {
        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_STATUS,
            "LIFT_FAULT_RESET state=PENDING phase=P%u txn=%lu req=%08lX\r\n",
            (unsigned)phase,
            (unsigned long)s_lift_reset_txn,
            (unsigned long)s_lift_reset_nonce);
    } else if (s_lift_home_bound &&
               (s_lift_pending.action == LIFT_PENDING_HOME)) {
        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_STATUS,
            "LIFT_HOME state=PENDING phase=P%u txn=%lu req=%08lX\r\n",
            (unsigned)phase,
            (unsigned long)s_lift_home_txn,
            (unsigned long)s_lift_home_nonce);
    }
}

static LiftActionStepResult_t Lift_AsyncPollAction(uint32_t now,
                                                   uint32_t *value)
{
    uint32_t abort_code = 0U;
    CanopenAsyncResult_t result = Canopen_AsyncPoll(
        LIFT_NODE_ID,
        CANOPEN_SDO_OWNER_LIFT_ACTION,
        now,
        value,
        &abort_code);

    if (result == CANOPEN_ASYNC_RESULT_PENDING) {
        return LIFT_ACTION_STEP_WAIT;
    }
    s_lift_pending.request_pending = false;
    if (result == CANOPEN_ASYNC_RESULT_SUCCESS) {
        return LIFT_ACTION_STEP_DONE;
    }
    JetsonTx_Printf(JETSON_TX_PRIORITY_CRITICAL,
                "LIFT action SDO failed phase=%u result=%u abort=0x%08lX\r\n",
                (unsigned)s_lift_pending.phase,
                (unsigned)result,
                (unsigned long)abort_code);
    return LIFT_ACTION_STEP_FAILED;
}

static LiftActionStepResult_t Lift_AsyncStarted(
    CanopenAsyncStartResult_t result,
    uint32_t now)
{
    if (result == CANOPEN_ASYNC_START_OK) {
        s_lift_pending.request_pending = true;
        return LIFT_ACTION_STEP_WAIT;
    }
    if ((result == CANOPEN_ASYNC_START_BUSY) &&
        ((uint32_t)(now - s_lift_pending.phase_start_ms) <
         LIFT_CIA402_STATE_TIMEOUT_MS)) {
        return LIFT_ACTION_STEP_WAIT;
    }
    return LIFT_ACTION_STEP_FAILED;
}

static LiftActionStepResult_t Lift_AsyncReadU16(uint16_t index,
                                                uint32_t now,
                                                uint32_t *value)
{
    if (s_lift_pending.request_pending) {
        return Lift_AsyncPollAction(now, value);
    }
    return Lift_AsyncStarted(
        Canopen_AsyncReadU16(LIFT_NODE_ID,
                             index,
                             0x00,
                             CANOPEN_SDO_OWNER_LIFT_ACTION,
                             now,
                             LIFT_ACTION_SDO_TIMEOUT_MS),
        now);
}

static LiftActionStepResult_t Lift_AsyncWriteU16(uint16_t index,
                                                 uint16_t value,
                                                 uint32_t now)
{
    if (s_lift_pending.request_pending) {
        return Lift_AsyncPollAction(now, NULL);
    }
    return Lift_AsyncStarted(
        Canopen_AsyncWriteU16(LIFT_NODE_ID,
                              index,
                              0x00,
                              value,
                              CANOPEN_SDO_OWNER_LIFT_ACTION,
                              now,
                              LIFT_ACTION_SDO_TIMEOUT_MS),
        now);
}

static LiftActionStepResult_t Lift_AsyncWriteI8(uint16_t index,
                                                int8_t value,
                                                uint32_t now)
{
    if (s_lift_pending.request_pending) {
        return Lift_AsyncPollAction(now, NULL);
    }
    return Lift_AsyncStarted(
        Canopen_AsyncWriteI8(LIFT_NODE_ID,
                             index,
                             0x00,
                             value,
                             CANOPEN_SDO_OWNER_LIFT_ACTION,
                             now,
                             LIFT_ACTION_SDO_TIMEOUT_MS),
        now);
}

static LiftActionStepResult_t Lift_AsyncWriteI32(uint16_t index,
                                                 int32_t value,
                                                 uint32_t now)
{
    if (s_lift_pending.request_pending) {
        return Lift_AsyncPollAction(now, NULL);
    }
    return Lift_AsyncStarted(
        Canopen_AsyncWriteI32(LIFT_NODE_ID,
                              index,
                              0x00,
                              value,
                              CANOPEN_SDO_OWNER_LIFT_ACTION,
                              now,
                              LIFT_ACTION_SDO_TIMEOUT_MS),
        now);
}

static bool Lift_PendingIsProfile(void)
{
    return (s_lift_pending.action == LIFT_PENDING_MOVE_UP) ||
           (s_lift_pending.action == LIFT_PENDING_MOVE_DOWN);
}

static void Lift_ClearPendingAction(uint32_t now)
{
    Canopen_CancelAsyncOwner(CANOPEN_SDO_OWNER_LIFT_ACTION, now);
    memset(&s_lift_pending, 0, sizeof(s_lift_pending));
    Drive_CancelConfirmedStopForLift();
}

static void Lift_SetFaultReasonWithoutDriveRead(LiftFaultReason_t reason)
{
    s_lift_fault_reason = reason;
    s_lift_fault_code = (uint32_t)reason;
    JetsonTx_Printf(JETSON_TX_PRIORITY_CRITICAL,
                    "LIFT reason=%s\r\n", Lift_FaultReasonName(reason));
}

static void Lift_FailPendingAction(LiftFaultReason_t reason,
                                   const char *message,
                                   uint32_t now,
                                   bool drive_read_allowed)
{
    LiftPendingAction_t failing_action = s_lift_pending.action;

    Lift_ClearPendingAction(now);
    if (drive_read_allowed) {
        Lift_SetFaultReason(reason);
    } else {
        Lift_SetFaultReasonWithoutDriveRead(reason);
    }
    Lift_SetState(LIFT_STATE_FAULT);
    JetsonTx_Printf(JETSON_TX_PRIORITY_CRITICAL,
                    "LIFT pending action failed msg=%s\r\n", message);
    if (failing_action == LIFT_PENDING_FAULT_RESET) {
        Lift_EmitResetFailed(Lift_FaultReasonName(reason));
    } else if (failing_action == LIFT_PENDING_HOME) {
        Lift_EmitHomeFailed(Lift_FaultReasonName(reason));
    }
    Lift_SendJetsonStatusLine();
}

static bool Lift_QueueAction(LiftPendingAction_t action,
                             int32_t target_pul,
                             LiftState_t moving_state,
                             bool upward)
{
    DriveConfirmedStopResult_t stop_result;
    uint32_t now = HAL_GetTick();

    if ((s_lift_pending.action != LIFT_PENDING_NONE) ||
        Lift_IsProfileMoving() || Lift_IsRecoveryActive()) {
        uart_printf("LIFT command rejected: action already pending/moving\r\n");
        Lift_SendJetsonStatusLine();
        return false;
    }

    memset(&s_lift_pending, 0, sizeof(s_lift_pending));
    s_lift_pending.action = action;
    s_lift_pending.target_pul = target_pul;
    s_lift_pending.moving_state = moving_state;
    s_lift_pending.upward = upward;
    Lift_SetActionPhase(LIFT_ACTION_WAIT_BASE_STOP, now);
    stop_result = Drive_RequestConfirmedStopForLift();
    if (stop_result == DRIVE_CONFIRMED_STOP_FAILED) {
        Lift_FailPendingAction(LIFT_FAULT_REASON_BASE_STOP_FAILED,
                               "base confirmed-stop request failed",
                               now,
                               false);
        return false;
    }
    if (stop_result == DRIVE_CONFIRMED_STOP_OK) {
        Lift_SetActionPhase(LIFT_ACTION_READ_STATUS, now);
    }
    uart_printf("LIFT action queued action=%u phase=%s\r\n",
                (unsigned)action,
                Lift_RecoveryPhaseName(s_lift_recovery_phase));
    Lift_SendJetsonStatusLine();
    return true;
}

static void Lift_BeginHomePhase(LiftHomePhase_t phase, uint32_t now)
{
    s_lift_home_phase = phase;
    s_lift_home_phase_start_ms = now;
    s_lift_home_last_poll_ms = 0U;
    s_lift_home_action_started = false;
    s_lift_home_request_pending = false;
    if (s_lift_home_bound &&
        (phase != HOME_COMPLETE) && (phase != HOME_FAILED)) {
        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_STATUS,
            "LIFT_HOME state=PENDING phase=H%u txn=%lu req=%08lX\r\n",
            (unsigned)phase,
            (unsigned long)s_lift_home_txn,
            (unsigned long)s_lift_home_nonce);
    }
}

static bool Lift_HomePollDue(uint32_t now, uint32_t poll_ms)
{
    if ((now - s_lift_home_last_poll_ms) < poll_ms) {
        return false;
    }
    s_lift_home_last_poll_ms = now;
    return true;
}

static bool Lift_HomePhaseTimedOut(uint32_t now, uint32_t timeout_ms)
{
    return (now - s_lift_home_phase_start_ms) >= timeout_ms;
}

static void Lift_PrintModeDisplayFailure(void)
{
    LiftDriveSnapshot_t snapshot;
    char display_text[16];
    char status_text[16];
    char error_text[16];

    Lift_ReadDriveSnapshot(&snapshot);
    Lift_FormatI8Text(display_text,
                      sizeof(display_text),
                      s_lift_home_display_seen,
                      s_lift_home_last_displayed_mode);
    Lift_FormatU16HexText(status_text,
                          sizeof(status_text),
                          snapshot.status_ok,
                          snapshot.statusword);
    Lift_FormatU16HexText(error_text,
                          sizeof(error_text),
                          snapshot.error_ok,
                          snapshot.error_code);
    uart_printf("%s requested_mode=6 displayed_mode=%s statusword=%s error_code=%s\r\n",
                Lift_FaultReasonName(s_lift_fault_reason),
                display_text,
                status_text,
                error_text);
}

static bool Lift_StopSensorHoming(void)
{
    if (s_lift_homing_start_edge_issued) {
        return Lift_RequestDecelStop();
    }

    /* Shutdown cannot advance a pre-start drive into Operation Enabled. */
    return Lift_WriteControlword(0x0006U);
}

static void Lift_CompleteSensorHoming(uint16_t statusword)
{
    int32_t position_pul = 0;

    if (!Lift_WriteControlword(0x000FU)) {
        s_lift_recovery_phase = LIFT_RECOVERY_PHASE_NONE;
        s_lift_homed = false;
        Lift_SetFaultReason(LIFT_FAULT_REASON_NODE3_COMM_TIMEOUT);
        Lift_SetState(LIFT_STATE_FAULT);
        JetsonTx_Printf(JETSON_TX_PRIORITY_CRITICAL,
                        "NODE3_COMM_TIMEOUT unable_to_clear_start_bit\r\n");
        Lift_EmitHomeFailed("NODE3_COMM_TIMEOUT");
        Lift_SendJetsonStatusLine();
        return;
    }

    s_lift_recovery_phase = LIFT_RECOVERY_PHASE_NONE;

    if (!Lift_ReadActualPositionQuiet(&position_pul)) {
        s_lift_homed = false;
        Lift_SetFaultReason(LIFT_FAULT_REASON_POSITION_READ_FAILED);
        Lift_SetState(LIFT_STATE_FAULT);
        JetsonTx_Printf(JETSON_TX_PRIORITY_CRITICAL,
                        "POSITION_READ_FAILED statusword=0x%04X\r\n", statusword);
        Lift_EmitHomeFailed("POSITION_READ_FAILED");
        Lift_SendJetsonStatusLine();
        return;
    }

    s_lift_homed = true;
    s_lift_pos_pul = position_pul;
    s_lift_target_pul = position_pul;
    Lift_ClearFaultReason();
    Lift_SetState(Lift_ClassifyRelativePosition(s_lift_pos_pul));
    uart_printf("LIFT sensor homing complete fresh_cycle=1 statusword=0x%04X pos6064=%ld\r\n",
                statusword,
                (long)s_lift_pos_pul);
    if (s_lift_home_bound) {
        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_STATUS,
            "LIFT_HOME state=COMPLETE homed=1 txn=%lu req=%08lX\r\n",
            (unsigned long)s_lift_home_txn,
            (unsigned long)s_lift_home_nonce);
        s_lift_home_bound = false;
    }
    Lift_SendJetsonStatusLine();
}

static void Lift_FailSensorHoming(LiftFaultReason_t reason,
                                  const char *message,
                                  bool retry_if_statusword_clear)
{
    LiftRecoveryDiag_t diag;
    bool drive_fault = false;
    bool stop_ok = false;
    bool was_sensor_homing = Lift_IsSensorHomingActive();

    Canopen_CancelAsyncOwner(CANOPEN_SDO_OWNER_LIFT_ACTION, HAL_GetTick());
    s_lift_home_request_pending = false;
    /* Deassert the homing start bit before diagnostic SDO traffic. */
    stop_ok = was_sensor_homing ? Lift_StopSensorHoming() : Lift_StopRecoveryJog();
    if (!stop_ok) {
        reason = LIFT_FAULT_REASON_NODE3_COMM_TIMEOUT;
    }
    Lift_ReadRecoveryDiag(&diag);
    drive_fault = (diag.status_ok && Statusword_HasFault(diag.statusword)) ||
                  (diag.error_ok && (diag.error_code != 0U));
    if (drive_fault) {
        reason = LIFT_FAULT_REASON_DRIVE_FAULT;
    } else if (!diag.status_ok || !diag.error_ok) {
        reason = LIFT_FAULT_REASON_NODE3_COMM_TIMEOUT;
    }
    Lift_SetFaultReason(reason);
    Lift_PrintRecoveryFailure(message, HAL_GetTick(), &diag);
    s_lift_home_phase = HOME_FAILED;
    s_lift_recovery_phase = LIFT_RECOVERY_PHASE_NONE;
    s_lift_homed = false;

    if (!stop_ok || drive_fault || !retry_if_statusword_clear ||
        !diag.status_ok || !diag.error_ok) {
        Lift_SetState(LIFT_STATE_FAULT);
    } else {
        s_lift_fault_code = 0U;
        Lift_SetState(LIFT_STATE_RECOVERY_REQUIRED);
    }

    /* This homing ATTEMPT is over either way; the bound transaction must
     * report a terminal failure with the final reason. */
    Lift_EmitHomeFailed(Lift_FaultReasonName(reason));
    Lift_SendJetsonStatusLine();
}

static void Lift_StartSensorHomingConfirmed(uint32_t now)
{
    s_lift_homed = false;
    s_lift_recovery_phase = LIFT_RECOVERY_PHASE_SENSOR_HOMING;
    s_lift_recovery_start_ms = now;
    s_lift_recovery_last_poll_ms = 0U;
    s_homing_pre_start_statusword = 0U;
    s_homing_reference_seen_clear = false;
    s_lift_homing_start_edge_issued = false;
    s_lift_home_last_displayed_mode = 0;
    s_lift_home_display_seen = false;
    Lift_ClearFaultReason();
    Lift_BeginHomePhase(HOME_PREFLIGHT, now);
    Lift_SetState(LIFT_STATE_RECOVERING_UP);
    uart_printf("LIFT sensor homing started after base confirmed-stop\r\n");
    Lift_SendJetsonStatusLine();
}

static LiftActionStepResult_t Lift_HomeAsyncWriteU32(uint16_t index,
                                                     uint8_t subindex,
                                                     uint32_t value,
                                                     uint32_t now)
{
    if (s_lift_home_request_pending) {
        uint32_t abort_code = 0U;
        CanopenAsyncResult_t result = Canopen_AsyncPoll(
            LIFT_NODE_ID,
            CANOPEN_SDO_OWNER_LIFT_ACTION,
            now,
            NULL,
            &abort_code);

        if (result == CANOPEN_ASYNC_RESULT_PENDING) {
            return LIFT_ACTION_STEP_WAIT;
        }
        s_lift_home_request_pending = false;
        if (result == CANOPEN_ASYNC_RESULT_SUCCESS) {
            return LIFT_ACTION_STEP_DONE;
        }
        JetsonTx_Printf(JETSON_TX_PRIORITY_CRITICAL,
                        "LIFT homing SDO failed result=%u abort=0x%08lX\r\n",
                        (unsigned)result,
                        (unsigned long)abort_code);
        return LIFT_ACTION_STEP_FAILED;
    }

    switch (Canopen_AsyncWriteU32(LIFT_NODE_ID,
                                  index,
                                  subindex,
                                  value,
                                  CANOPEN_SDO_OWNER_LIFT_ACTION,
                                  now,
                                  LIFT_ACTION_SDO_TIMEOUT_MS)) {
    case CANOPEN_ASYNC_START_OK:
        s_lift_home_request_pending = true;
        return LIFT_ACTION_STEP_WAIT;
    case CANOPEN_ASYNC_START_BUSY:
        return Lift_HomePhaseTimedOut(now, LIFT_CIA402_STATE_TIMEOUT_MS) ?
            LIFT_ACTION_STEP_FAILED : LIFT_ACTION_STEP_WAIT;
    default:
        return LIFT_ACTION_STEP_FAILED;
    }
}

static void Lift_ReportHomingConfigFailure(uint16_t index,
                                           uint8_t subindex,
                                           uint32_t value)
{
    JetsonTx_Printf(JETSON_TX_PRIORITY_CRITICAL,
                    "LIFT homing config failed object=0x%04X:%02X value=%lu\r\n",
                    index,
                    subindex,
                    (unsigned long)value);
}

static bool Lift_StartSensorHoming(void)
{
    return Lift_QueueAction(LIFT_PENDING_HOME,
                            s_lift_target_pul,
                            LIFT_STATE_RECOVERING_UP,
                            true);
}

static void Lift_UpdateSensorHoming(uint32_t now)
{
    uint16_t statusword = 0U;
    uint16_t error_code = 0U;
    int8_t displayed_mode = 0;
    uint16_t controlword = 0U;
    LiftDriveSnapshot_t snapshot;
    LiftActionStepResult_t step;

    if (!Lift_IsSensorHomingActive()) {
        return;
    }

    switch (s_lift_home_phase) {
    case HOME_PREFLIGHT:
        Lift_ReadDriveSnapshot(&snapshot);
        if (!snapshot.status_ok || !snapshot.error_ok) {
            Lift_FailSensorHoming(LIFT_FAULT_REASON_NODE3_COMM_TIMEOUT,
                                  "preflight read failed",
                                  false);
            return;
        }

        if (Statusword_HasFault(snapshot.statusword)) {
            if (!Lift_WriteControlword(0x0080U)) {
                Lift_FailSensorHoming(LIFT_FAULT_REASON_NODE3_COMM_TIMEOUT,
                                      "fault reset assert failed",
                                      false);
                return;
            }
            Lift_BeginHomePhase(HOME_CLEAR_FAULT_ASSERT, now);
            return;
        }

        if (snapshot.error_code != 0U) {
            Lift_FailSensorHoming(LIFT_FAULT_REASON_DRIVE_FAULT,
                                  "drive error code active",
                                  false);
            return;
        }

        Lift_BeginHomePhase(HOME_WAIT_READY, now);
        return;

    case HOME_CLEAR_FAULT_ASSERT:
        if ((now - s_lift_home_phase_start_ms) < LIFT_FAULT_RESET_ASSERT_MS) {
            return;
        }
        if (!Lift_WriteControlword(0x0000U)) {
            Lift_FailSensorHoming(LIFT_FAULT_REASON_NODE3_COMM_TIMEOUT,
                                  "fault reset release failed",
                                  false);
            return;
        }
        Lift_BeginHomePhase(HOME_CLEAR_FAULT_RELEASE, now);
        return;

    case HOME_CLEAR_FAULT_RELEASE:
        if ((now - s_lift_home_phase_start_ms) < LIFT_FAULT_RESET_SETTLE_MS) {
            return;
        }
        Lift_ReadDriveSnapshot(&snapshot);
        if (!snapshot.status_ok || !snapshot.error_ok) {
            Lift_FailSensorHoming(LIFT_FAULT_REASON_NODE3_COMM_TIMEOUT,
                                  "fault reset verify read failed",
                                  false);
            return;
        }
        if (Statusword_HasFault(snapshot.statusword) || (snapshot.error_code != 0U)) {
            Lift_FailSensorHoming(LIFT_FAULT_REASON_DRIVE_FAULT,
                                  "drive fault still active",
                                  false);
            return;
        }
        Lift_BeginHomePhase(HOME_WAIT_READY, now);
        return;

    case HOME_WAIT_READY:
        if (!s_lift_home_action_started) {
            if (!Lift_WriteControlword(0x0006U)) {
                Lift_FailSensorHoming(LIFT_FAULT_REASON_NODE3_COMM_TIMEOUT,
                                      "ready command failed",
                                      false);
                return;
            }
            s_lift_home_action_started = true;
        }
        if (!Lift_HomePollDue(now, LIFT_CIA402_STATE_POLL_MS)) {
            return;
        }
        if (!Lift_ReadStatusQuiet(&statusword)) {
            if (Lift_HomePhaseTimedOut(now, LIFT_CIA402_STATE_TIMEOUT_MS)) {
                Lift_FailSensorHoming(LIFT_FAULT_REASON_NODE3_COMM_TIMEOUT,
                                      "ready status timeout",
                                      false);
            }
            return;
        }
        if (Statusword_HasFault(statusword)) {
            Lift_FailSensorHoming(LIFT_FAULT_REASON_DRIVE_FAULT,
                                  "drive fault while readying",
                                  false);
            return;
        }
        if ((statusword & 0x006FU) == 0x0021U) {
            Lift_BeginHomePhase(HOME_CONFIG_SEARCH_SPEED, now);
            return;
        }
        if (Lift_HomePhaseTimedOut(now, LIFT_CIA402_STATE_TIMEOUT_MS)) {
            Lift_FailSensorHoming(LIFT_FAULT_REASON_CIA402_READY_TIMEOUT,
                                  "ready status timeout",
                                  false);
        }
        return;

    case HOME_CONFIG_SEARCH_SPEED:
        step = Lift_HomeAsyncWriteU32(0x6099U,
                                      0x01U,
                                      LIFT_PROFILE_VELOCITY_PUL_S,
                                      now);
        if (step == LIFT_ACTION_STEP_WAIT) {
            return;
        }
        if (step == LIFT_ACTION_STEP_FAILED) {
            Lift_ReportHomingConfigFailure(0x6099U,
                                           0x01U,
                                           LIFT_PROFILE_VELOCITY_PUL_S);
            Lift_FailSensorHoming(LIFT_FAULT_REASON_NODE3_COMM_TIMEOUT,
                                  "homing search-speed write failed",
                                  false);
            return;
        }
        Lift_BeginHomePhase(HOME_CONFIG_FINAL_SPEED, now);
        return;

    case HOME_CONFIG_FINAL_SPEED:
        step = Lift_HomeAsyncWriteU32(0x6099U,
                                      0x02U,
                                      LIFT_PROFILE_VELOCITY_PUL_S,
                                      now);
        if (step == LIFT_ACTION_STEP_WAIT) {
            return;
        }
        if (step == LIFT_ACTION_STEP_FAILED) {
            Lift_ReportHomingConfigFailure(0x6099U,
                                           0x02U,
                                           LIFT_PROFILE_VELOCITY_PUL_S);
            Lift_FailSensorHoming(LIFT_FAULT_REASON_NODE3_COMM_TIMEOUT,
                                  "homing final-speed write failed",
                                  false);
            return;
        }
        Lift_BeginHomePhase(HOME_CONFIG_ACCEL, now);
        return;

    case HOME_CONFIG_ACCEL:
        step = Lift_HomeAsyncWriteU32(0x609AU,
                                      0x00U,
                                      LIFT_PROFILE_ACCEL_PUL_S2,
                                      now);
        if (step == LIFT_ACTION_STEP_WAIT) {
            return;
        }
        if (step == LIFT_ACTION_STEP_FAILED) {
            Lift_ReportHomingConfigFailure(0x609AU,
                                           0x00U,
                                           LIFT_PROFILE_ACCEL_PUL_S2);
            Lift_FailSensorHoming(LIFT_FAULT_REASON_NODE3_COMM_TIMEOUT,
                                  "homing acceleration write failed",
                                  false);
            return;
        }
        uart_printf("LIFT homing config search=%lu final=%lu accel=%lu source=normal_profile\r\n",
                    (unsigned long)LIFT_PROFILE_VELOCITY_PUL_S,
                    (unsigned long)LIFT_PROFILE_VELOCITY_PUL_S,
                    (unsigned long)LIFT_PROFILE_ACCEL_PUL_S2);
        Lift_BeginHomePhase(HOME_REQUEST_MODE, now);
        return;

    case HOME_REQUEST_MODE:
        if (!SDO_Write_I8_Timeout_Node(&s_lift_node,
                                       0x6060,
                                       0x00,
                                       6,
                                       LIFT_ACTION_SDO_TIMEOUT_MS)) {
            Lift_FailSensorHoming(LIFT_FAULT_REASON_NODE3_COMM_TIMEOUT,
                                  "homing mode request failed",
                                  false);
            return;
        }
        Lift_BeginHomePhase(HOME_SWITCH_ON, now);
        return;

    case HOME_SWITCH_ON:
        if (!s_lift_home_action_started) {
            if (!Lift_WriteControlword(0x0007U)) {
                Lift_FailSensorHoming(LIFT_FAULT_REASON_NODE3_COMM_TIMEOUT,
                                      "switch-on command failed",
                                      false);
                return;
            }
            s_lift_home_action_started = true;
        }
        if (!Lift_HomePollDue(now, LIFT_CIA402_STATE_POLL_MS)) {
            return;
        }
        if (!Lift_ReadStatusQuiet(&statusword)) {
            if (Lift_HomePhaseTimedOut(now, LIFT_CIA402_STATE_TIMEOUT_MS)) {
                Lift_FailSensorHoming(LIFT_FAULT_REASON_NODE3_COMM_TIMEOUT,
                                      "switch-on status timeout",
                                      false);
            }
            return;
        }
        if (Statusword_HasFault(statusword)) {
            Lift_FailSensorHoming(LIFT_FAULT_REASON_DRIVE_FAULT,
                                  "drive fault while switching on",
                                  false);
            return;
        }
        if ((statusword & 0x006FU) == 0x0023U) {
            Lift_BeginHomePhase(HOME_ENABLE_OPERATION, now);
            return;
        }
        if (Lift_HomePhaseTimedOut(now, LIFT_CIA402_STATE_TIMEOUT_MS)) {
            Lift_FailSensorHoming(LIFT_FAULT_REASON_CIA402_SWITCHED_ON_TIMEOUT,
                                  "switched-on status timeout",
                                  false);
        }
        return;

    case HOME_ENABLE_OPERATION:
        if (!s_lift_home_action_started) {
            if (!Lift_WriteControlword(0x000FU)) {
                Lift_FailSensorHoming(LIFT_FAULT_REASON_NODE3_COMM_TIMEOUT,
                                      "operation-enable command failed",
                                      false);
                return;
            }
            s_lift_home_action_started = true;
        }
        if (!Lift_HomePollDue(now, LIFT_CIA402_STATE_POLL_MS)) {
            return;
        }
        if (!Lift_ReadStatusQuiet(&statusword)) {
            if (Lift_HomePhaseTimedOut(now, LIFT_CIA402_STATE_TIMEOUT_MS)) {
                Lift_FailSensorHoming(LIFT_FAULT_REASON_NODE3_COMM_TIMEOUT,
                                      "operation-enabled status timeout",
                                      false);
            }
            return;
        }
        if (Statusword_HasFault(statusword)) {
            Lift_FailSensorHoming(LIFT_FAULT_REASON_DRIVE_FAULT,
                                  "drive fault while enabling operation",
                                  false);
            return;
        }
        if ((statusword & 0x006FU) == 0x0027U) {
            s_lift_home_display_seen = false;
            s_lift_home_last_displayed_mode = 0;
            Lift_BeginHomePhase(HOME_WAIT_MODE_DISPLAY, now);
            return;
        }
        if (Lift_HomePhaseTimedOut(now, LIFT_CIA402_STATE_TIMEOUT_MS)) {
            Lift_FailSensorHoming(LIFT_FAULT_REASON_CIA402_OPERATION_ENABLED_TIMEOUT,
                                  "operation-enabled status timeout",
                                  false);
        }
        return;

    case HOME_WAIT_MODE_DISPLAY:
        if (!Lift_HomePollDue(now, LIFT_HOMING_MODE_POLL_MS)) {
            return;
        }
        if (Lift_ReadStatusQuiet(&statusword) && Statusword_HasFault(statusword)) {
            Lift_FailSensorHoming(LIFT_FAULT_REASON_DRIVE_FAULT,
                                  "drive fault while waiting mode display",
                                  false);
            return;
        }
        if (SDO_Read_I8_Quiet_Node(&s_lift_node, 0x6061, 0x00, &displayed_mode)) {
            s_lift_home_display_seen = true;
            s_lift_home_last_displayed_mode = displayed_mode;
            if (displayed_mode == 6) {
                Lift_BeginHomePhase(HOME_PREPARE_START_EDGE, now);
                return;
            }
        }
        if (Lift_HomePhaseTimedOut(now, LIFT_HOMING_MODE_DISPLAY_TIMEOUT_MS)) {
            Lift_SetFaultReason(LIFT_FAULT_REASON_HOMING_MODE_DISPLAY_TIMEOUT);
            Lift_PrintModeDisplayFailure();
            Lift_FailSensorHoming(LIFT_FAULT_REASON_HOMING_MODE_DISPLAY_TIMEOUT,
                                  "mode display timeout",
                                  false);
        }
        return;

    case HOME_PREPARE_START_EDGE:
        if (!s_lift_home_action_started) {
            if (!Lift_WriteControlword(0x000FU)) {
                Lift_FailSensorHoming(LIFT_FAULT_REASON_NODE3_COMM_TIMEOUT,
                                      "start-bit-low command failed",
                                      false);
                return;
            }
            s_lift_home_action_started = true;
        }
        if (!Lift_HomePollDue(now, LIFT_HOMING_START_POLL_MS)) {
            return;
        }
        if (!SDO_Read_U16_Quiet_Node(&s_lift_node, 0x6040, 0x00, &controlword)) {
            if (Lift_HomePhaseTimedOut(now, LIFT_HOMING_START_TIMEOUT_MS)) {
                Lift_FailSensorHoming(LIFT_FAULT_REASON_NODE3_COMM_TIMEOUT,
                                      "start-bit-low read timeout",
                                      false);
            }
            return;
        }
        if ((controlword & 0x0010U) != 0U) {
            if (Lift_HomePhaseTimedOut(now, LIFT_HOMING_START_TIMEOUT_MS)) {
                Lift_FailSensorHoming(LIFT_FAULT_REASON_HOMING_START_FAILED,
                                      "start bit stuck high",
                                      false);
            }
            return;
        }
        Lift_ReadDriveSnapshot(&snapshot);
        if (!snapshot.status_ok || !snapshot.error_ok) {
            Lift_FailSensorHoming(LIFT_FAULT_REASON_NODE3_COMM_TIMEOUT,
                                  "pre-start snapshot failed",
                                  false);
            return;
        }
        if (Statusword_HasFault(snapshot.statusword) || (snapshot.error_code != 0U)) {
            Lift_FailSensorHoming(LIFT_FAULT_REASON_DRIVE_FAULT,
                                  "pre-start drive fault",
                                  false);
            return;
        }
        if ((snapshot.statusword & 0x006FU) != 0x0027U) {
            Lift_FailSensorHoming(LIFT_FAULT_REASON_CIA402_OPERATION_ENABLED_TIMEOUT,
                                  "pre-start operation state lost",
                                  false);
            return;
        }
        s_homing_pre_start_statusword = snapshot.statusword;
        s_homing_reference_seen_clear = !Statusword_ReferenceFound(snapshot.statusword);
        Lift_BeginHomePhase(HOME_START_EDGE, now);
        return;

    case HOME_START_EDGE:
        if (!s_lift_home_action_started) {
            if (!Lift_WriteControlword(0x001FU)) {
                Lift_FailSensorHoming(LIFT_FAULT_REASON_NODE3_COMM_TIMEOUT,
                                      "start-edge command failed",
                                      false);
                return;
            }
            s_lift_homing_start_edge_issued = true;
            s_lift_home_action_started = true;
        }
        if (!Lift_HomePollDue(now, LIFT_HOMING_START_POLL_MS)) {
            return;
        }
        if (!SDO_Read_U16_Quiet_Node(&s_lift_node, 0x6040, 0x00, &controlword)) {
            if (Lift_HomePhaseTimedOut(now, LIFT_HOMING_START_TIMEOUT_MS)) {
                Lift_FailSensorHoming(LIFT_FAULT_REASON_NODE3_COMM_TIMEOUT,
                                      "start-bit-high read timeout",
                                      false);
            }
            return;
        }
        if ((controlword & 0x0010U) == 0U) {
            if (Lift_HomePhaseTimedOut(now, LIFT_HOMING_START_TIMEOUT_MS)) {
                Lift_FailSensorHoming(LIFT_FAULT_REASON_HOMING_START_FAILED,
                                      "start bit did not rise",
                                      false);
            }
            return;
        }
        if (Lift_ReadStatusQuiet(&statusword)) {
            if (Statusword_HasFault(statusword)) {
                Lift_FailSensorHoming(LIFT_FAULT_REASON_DRIVE_FAULT,
                                      "drive fault after start edge",
                                      false);
                return;
            }
            if (Statusword_HasHomingError(statusword)) {
                Lift_FailSensorHoming(LIFT_FAULT_REASON_HOMING_ERROR,
                                      "homing error after start edge",
                                      false);
                return;
            }
            if (!Statusword_ReferenceFound(statusword)) {
                if (!s_homing_reference_seen_clear) {
                    uart_printf("LIFT homing fresh-cycle reference_clear observed immediate=1\r\n");
                }
                s_homing_reference_seen_clear = true;
            }
        }
        s_lift_recovery_start_ms = now;
        s_lift_recovery_last_poll_ms = 0U;
        Lift_BeginHomePhase(HOME_RUNNING, now);
        uart_printf("LIFT sensor homing started pre_start_statusword=0x%04X pre_reference=%u\r\n",
                    s_homing_pre_start_statusword,
                    Statusword_ReferenceFound(s_homing_pre_start_statusword) ? 1U : 0U);
        Lift_SendJetsonStatusLine();
        return;

    case HOME_RUNNING:
        if ((now - s_lift_recovery_last_poll_ms) < LIFT_HOMING_POLL_MS) {
            return;
        }
        s_lift_recovery_last_poll_ms = now;

        if (!Lift_ReadStatusQuiet(&statusword)) {
            Lift_FailSensorHoming(LIFT_FAULT_REASON_NODE3_COMM_TIMEOUT,
                                  "status read failed",
                                  false);
            return;
        }

        if (Statusword_HasFault(statusword)) {
            Lift_FailSensorHoming(LIFT_FAULT_REASON_DRIVE_FAULT,
                                  "drive status fault",
                                  false);
            return;
        }

        if (Statusword_HasHomingError(statusword)) {
            Lift_FailSensorHoming(LIFT_FAULT_REASON_HOMING_ERROR,
                                  "homing status error",
                                  false);
            return;
        }

        if (!Statusword_ReferenceFound(statusword)) {
            if (!s_homing_reference_seen_clear) {
                uart_printf("LIFT homing fresh-cycle reference_clear observed\r\n");
            }
            s_homing_reference_seen_clear = true;
        }

        if (!SDO_Read_U16_Quiet_Node(&s_lift_node, 0x603F, 0x00, &error_code)) {
            Lift_FailSensorHoming(LIFT_FAULT_REASON_NODE3_COMM_TIMEOUT,
                                  "error code read failed",
                                  false);
            return;
        }

        if (error_code != 0U) {
            Lift_FailSensorHoming(LIFT_FAULT_REASON_DRIVE_FAULT,
                                  "drive error code active",
                                  false);
            return;
        }

        if (!SDO_Read_I8_Quiet_Node(&s_lift_node, 0x6061, 0x00, &displayed_mode)) {
            Lift_FailSensorHoming(LIFT_FAULT_REASON_NODE3_COMM_TIMEOUT,
                                  "mode display read failed",
                                  false);
            return;
        }

        if (displayed_mode != 6) {
            Lift_FailSensorHoming(LIFT_FAULT_REASON_HOMING_MODE_LOSS,
                                  "homing mode lost",
                                  false);
            return;
        }

        if ((now - s_lift_recovery_start_ms) >= LIFT_HOMING_TIMEOUT_MS) {
            if (Statusword_ReferenceFound(s_homing_pre_start_statusword) &&
                !s_homing_reference_seen_clear) {
                Lift_FailSensorHoming(LIFT_FAULT_REASON_STALE_REFERENCE_STATE,
                                      "pre-set reference never cleared",
                                      true);
            } else {
                Lift_FailSensorHoming(LIFT_FAULT_REASON_HOMING_TIMEOUT,
                                      "homing timeout",
                                      true);
            }
            return;
        }

        if (!Statusword_ReferenceFound(statusword)) {
            return;
        }

        if (s_homing_reference_seen_clear) {
            s_lift_home_phase = HOME_COMPLETE;
            Lift_CompleteSensorHoming(statusword);
        }
        return;

    case HOME_COMPLETE:
    case HOME_FAILED:
    default:
        return;
    }
}

static void Lift_UpdateRecovery(uint32_t now)
{
    uint16_t statusword = 0U;

    if (Lift_IsSensorHomingActive()) {
        Lift_UpdateSensorHoming(now);
        return;
    }

    if (s_lift_recovery_phase != LIFT_RECOVERY_PHASE_JOG) {
        return;
    }

    if ((now - s_lift_jog_last_command_ms) >= LIFT_SERVICE_JOG_WATCHDOG_MS) {
        (void)Lift_StopRecoveryJog();
        s_lift_recovery_phase = LIFT_RECOVERY_PHASE_NONE;
        Lift_RefreshCachedStatus();
        Lift_SetState(s_lift_homed ? LIFT_STATE_STOPPED : LIFT_STATE_RECOVERY_REQUIRED);
        Lift_SendJetsonStatusLine();
        return;
    }

    if ((now - s_lift_recovery_last_poll_ms) < LIFT_HOMING_POLL_MS) {
        return;
    }
    s_lift_recovery_last_poll_ms = now;

    if (!Lift_ReadStatusQuiet(&statusword)) {
        Lift_FailSensorHoming(LIFT_FAULT_REASON_NODE3_COMM_TIMEOUT,
                              "jog status read failed",
                              false);
        return;
    }

    if (Statusword_HasFault(statusword)) {
        Lift_FailSensorHoming(LIFT_FAULT_REASON_DRIVE_FAULT,
                              "jog drive status fault",
                              false);
    }
}

static bool Lift_ResetFault(void)
{
    uart_printf("LIFT_FAULT_RESET requested\r\n");
    return Lift_QueueAction(LIFT_PENDING_FAULT_RESET,
                            s_lift_target_pul,
                            s_lift_state,
                            true);
}

static bool Lift_StartProfileMove(int32_t target_pul, LiftState_t moving_state)
{
    LiftPendingAction_t action =
        (moving_state == LIFT_STATE_MOVING_UP) ?
        LIFT_PENDING_MOVE_UP : LIFT_PENDING_MOVE_DOWN;

    if (!s_lift_homed) {
        uart_printf("LIFT move rejected: LIFT_HOME required first\r\n");
        Lift_SendJetsonStatusLine();
        return false;
    }

    if (!Lift_QueueAction(action,
                          target_pul,
                          moving_state,
                          moving_state == LIFT_STATE_MOVING_UP)) {
        return false;
    }
    s_lift_fault_code = 0U;
    s_lift_target_pul = target_pul;
    return true;
}

static void Lift_StopCommand(void)
{
    bool ok;
    uint32_t now = HAL_GetTick();

    if (s_lift_pending.action != LIFT_PENDING_NONE) {
        LiftPendingAction_t cancelled_action = s_lift_pending.action;

        if (s_lift_pending.phase == LIFT_ACTION_WAIT_BASE_STOP) {
            Lift_ClearPendingAction(now);
            uart_printf("LIFT pending action cancelled before base confirmed-stop\r\n");
            if (cancelled_action == LIFT_PENDING_FAULT_RESET) {
                Lift_EmitResetFailed("ABORTED_BY_STOP");
            } else if (cancelled_action == LIFT_PENDING_HOME) {
                Lift_EmitHomeFailed("ABORTED_BY_STOP");
            }
            Lift_SendJetsonStatusLine();
            return;
        }
        Canopen_CancelAsyncOwner(CANOPEN_SDO_OWNER_LIFT_ACTION, now);
        Drive_CancelConfirmedStopForLift();
        memset(&s_lift_pending, 0, sizeof(s_lift_pending));
        s_lift_pending.action = LIFT_PENDING_CANCEL_STOP;
        Lift_SetActionPhase(LIFT_ACTION_CANCEL_WRITE_JOG_ZERO, now);
        uart_printf("LIFT pending start cancelled; safe-stop sequence queued\r\n");
        if (cancelled_action == LIFT_PENDING_FAULT_RESET) {
            Lift_EmitResetFailed("ABORTED_BY_STOP");
        } else if (cancelled_action == LIFT_PENDING_HOME) {
            Lift_EmitHomeFailed("ABORTED_BY_STOP");
        }
        return;
    }

    if (Lift_IsRecoveryActive()) {
        if (Lift_IsSensorHomingActive()) {
            Canopen_CancelAsyncOwner(CANOPEN_SDO_OWNER_LIFT_ACTION, now);
            s_lift_home_request_pending = false;
            ok = Lift_StopSensorHoming();
            Lift_SetFaultReason(LIFT_FAULT_REASON_ABORTED_BY_STOP);
            s_lift_home_phase = HOME_FAILED;
            Lift_EmitHomeFailed("ABORTED_BY_STOP");
        } else {
            ok = Lift_StopRecoveryJog();
        }
        s_lift_recovery_phase = LIFT_RECOVERY_PHASE_NONE;
        Lift_RefreshCachedStatus();

        if (ok && (s_lift_state != LIFT_STATE_FAULT)) {
            Lift_SetState(s_lift_homed ? LIFT_STATE_STOPPED : LIFT_STATE_RECOVERY_REQUIRED);
        } else if (!ok) {
            Lift_RecordFault(0x0001U);
            Lift_SetState(LIFT_STATE_FAULT);
        }

        Lift_SendJetsonStatusLine();
        return;
    }

    ok = Lift_RequestDecelStop();
    Lift_RefreshCachedStatus();
    s_lift_target_pul = s_lift_pos_pul;

    if (ok && (s_lift_state != LIFT_STATE_FAULT)) {
        Lift_SetState(s_lift_homed ? LIFT_STATE_STOPPED : LIFT_STATE_RECOVERY_REQUIRED);
    } else if (!ok) {
        Lift_RecordFault(0x0001U);
        Lift_SetState(LIFT_STATE_FAULT);
    }

    Lift_SendJetsonStatusLine();
}

void Lift_RequestCommunicationStop(void)
{
    /* This API is consumed from the main loop, never directly from a UART ISR.
     * Keep communication-loss handling identical to an explicit LIFT_STOP so
     * pending actions, recovery motion, and profile motion share one stop path.
     * An idle lift needs no CANopen transaction; returning here keeps a brief
     * RX discontinuity bounded and avoids turning it into a blocking stop. */
    if (!Lift_IsMoving()) {
        return;
    }
    Lift_StopCommand();
}

static bool Lift_StartSlowJog(bool upward)
{
    uint32_t now = HAL_GetTick();

    if (Lift_IsRecoveryActive()) {
        if ((s_lift_recovery_phase == LIFT_RECOVERY_PHASE_JOG) &&
            (s_lift_jog_upward == upward)) {
            s_lift_jog_last_command_ms = now;
            return true;
        }
        uart_printf("LIFT jog rejected: recovery/homing active\r\n");
        Lift_SendJetsonStatusLine();
        return false;
    }
    return Lift_QueueAction(upward ? LIFT_PENDING_JOG_UP :
                                      LIFT_PENDING_JOG_DOWN,
                            s_lift_target_pul,
                            upward ? LIFT_STATE_RECOVERING_UP :
                                     LIFT_STATE_RECOVERING_DOWN,
                            upward);
}

static void Lift_CompletePendingProfile(uint32_t now)
{
    LiftState_t moving_state = s_lift_pending.moving_state;

    Lift_ClearPendingAction(now);
    Lift_ClearFaultReason();
    s_lift_health_missed_count = 0U;
    Lift_SetState(moving_state);
    s_lift_move_start_ms = now;
    s_lift_last_poll_ms = 0U;
    s_lift_last_target_mismatch_ms = 0U;
    uart_printf("LIFT profile move started after base confirmed-stop\r\n");
    Lift_SendJetsonStatusLine();
}

static void Lift_CompletePendingJog(uint32_t now)
{
    bool upward = s_lift_pending.upward;

    Lift_ClearPendingAction(now);
    Lift_ClearFaultReason();
    s_lift_recovery_phase = LIFT_RECOVERY_PHASE_JOG;
    s_lift_recovery_start_ms = now;
    s_lift_recovery_last_poll_ms = 0U;
    s_lift_jog_last_command_ms = now;
    s_lift_jog_upward = upward;
    Lift_SetState(upward ? LIFT_STATE_RECOVERING_UP :
                           LIFT_STATE_RECOVERING_DOWN);
    uart_printf("LIFT jog started after base confirmed-stop\r\n");
    Lift_SendJetsonStatusLine();
}

static void Lift_CompletePendingReset(uint32_t now)
{
    Lift_ClearPendingAction(now);
    s_lift_health_missed_count = 0U;
    s_lift_health_last_poll_ms = now;
    Lift_ClearFaultReason();
    Lift_SetState(s_lift_homed ?
                  Lift_ClassifyRelativePosition(s_lift_pos_pul) :
                  LIFT_STATE_RECOVERY_REQUIRED);
    uart_printf("LIFT_FAULT_RESET complete\r\n");
    if (s_lift_reset_bound) {
        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_STATUS,
            "LIFT_FAULT_RESET state=COMPLETE txn=%lu req=%08lX\r\n",
            (unsigned long)s_lift_reset_txn,
            (unsigned long)s_lift_reset_nonce);
        s_lift_reset_bound = false;
    }
    Lift_SendJetsonStatusLine();
}

static void Lift_CompletePendingCancel(uint32_t now)
{
    Lift_ClearPendingAction(now);
    s_lift_target_pul = s_lift_pos_pul;
    Lift_SetState(s_lift_homed ? LIFT_STATE_STOPPED :
                                  LIFT_STATE_RECOVERY_REQUIRED);
    uart_printf("LIFT pending start cancelled and node3 stop submitted\r\n");
    Lift_SendJetsonStatusLine();
}

static void Lift_ServicePendingAction(uint32_t now)
{
    LiftActionStepResult_t step;
    DriveConfirmedStopResult_t stop_result;
    uint32_t value = 0U;

    if (s_lift_pending.action == LIFT_PENDING_NONE) {
        return;
    }

    if (s_lift_pending.phase == LIFT_ACTION_WAIT_BASE_STOP) {
        stop_result = Drive_RequestConfirmedStopForLift();
        if (stop_result == DRIVE_CONFIRMED_STOP_PENDING) {
            return;
        }
        if (stop_result == DRIVE_CONFIRMED_STOP_FAILED) {
            Lift_FailPendingAction(LIFT_FAULT_REASON_BASE_STOP_FAILED,
                                   "base confirmed-stop failed",
                                   now,
                                   false);
            return;
        }
        if (stop_result != DRIVE_CONFIRMED_STOP_OK) {
            Lift_FailPendingAction(LIFT_FAULT_REASON_BASE_STOP_FAILED,
                                   "invalid base confirmed-stop result",
                                   now,
                                   false);
            return;
        }
        if (s_lift_pending.action == LIFT_PENDING_HOME) {
            Lift_ClearPendingAction(now);
            Lift_StartSensorHomingConfirmed(now);
            return;
        }
        Lift_SetActionPhase(LIFT_ACTION_READ_STATUS, now);
        return;
    }

    if (s_lift_pending.action == LIFT_PENDING_HOME) {
        Lift_ClearPendingAction(now);
        Lift_StartSensorHomingConfirmed(now);
        return;
    }

    switch (s_lift_pending.phase) {
    case LIFT_ACTION_READ_STATUS:
        step = Lift_AsyncReadU16(0x6041U, now, &value);
        if (step == LIFT_ACTION_STEP_WAIT) {
            return;
        }
        if (step == LIFT_ACTION_STEP_FAILED) {
            break;
        }
        s_lift_pending.statusword = (uint16_t)value;
        if (s_lift_pending.action == LIFT_PENDING_FAULT_RESET) {
            Lift_SetActionPhase(LIFT_ACTION_READ_ERROR, now);
            return;
        }
        if (Statusword_HasFault(s_lift_pending.statusword)) {
            Lift_FailPendingAction(LIFT_FAULT_REASON_DRIVE_FAULT,
                                   "node3 fault before motion start",
                                   now,
                                   true);
            return;
        }
        Lift_SetActionPhase(Lift_PendingIsProfile() ?
                           LIFT_ACTION_PROFILE_WRITE_MODE :
                           LIFT_ACTION_JOG_WRITE_MODE,
                           now);
        return;

    case LIFT_ACTION_READ_ERROR:
        step = Lift_AsyncReadU16(0x603FU, now, &value);
        if (step == LIFT_ACTION_STEP_WAIT) {
            return;
        }
        if (step == LIFT_ACTION_STEP_FAILED) {
            break;
        }
        s_lift_pending.error_code = (uint16_t)value;
        if (Statusword_HasFault(s_lift_pending.statusword)) {
            Lift_SetActionPhase(LIFT_ACTION_RESET_ASSERT, now);
            return;
        }
        if (s_lift_pending.error_code != 0U) {
            Lift_FailPendingAction(LIFT_FAULT_REASON_DRIVE_FAULT,
                                   "node3 error without resettable status fault",
                                   now,
                                   true);
            return;
        }
        uart_printf("LIFT_FAULT_RESET no active drive fault; controlword unchanged\r\n");
        Lift_CompletePendingReset(now);
        return;

    case LIFT_ACTION_PROFILE_WRITE_MODE:
        step = Lift_AsyncWriteI8(0x6060U, 1, now);
        if (step == LIFT_ACTION_STEP_DONE) {
            Lift_SetActionPhase(LIFT_ACTION_PROFILE_WRITE_TARGET, now);
            return;
        }
        if (step == LIFT_ACTION_STEP_WAIT) return;
        break;

    case LIFT_ACTION_PROFILE_WRITE_TARGET:
        step = Lift_AsyncWriteI32(0x607AU, s_lift_pending.target_pul, now);
        if (step == LIFT_ACTION_STEP_DONE) {
            Lift_SetActionPhase(LIFT_ACTION_PROFILE_WRITE_VELOCITY, now);
            return;
        }
        if (step == LIFT_ACTION_STEP_WAIT) return;
        break;

    case LIFT_ACTION_PROFILE_WRITE_VELOCITY:
        step = Lift_AsyncWriteI32(0x6081U,
                                  (int32_t)LIFT_PROFILE_VELOCITY_PUL_S,
                                  now);
        if (step == LIFT_ACTION_STEP_DONE) {
            Lift_SetActionPhase(LIFT_ACTION_PROFILE_WRITE_ACCEL, now);
            return;
        }
        if (step == LIFT_ACTION_STEP_WAIT) return;
        break;

    case LIFT_ACTION_PROFILE_WRITE_ACCEL:
        step = Lift_AsyncWriteI32(0x6083U,
                                  (int32_t)LIFT_PROFILE_ACCEL_PUL_S2,
                                  now);
        if (step == LIFT_ACTION_STEP_DONE) {
            Lift_SetActionPhase(LIFT_ACTION_PROFILE_WRITE_DECEL, now);
            return;
        }
        if (step == LIFT_ACTION_STEP_WAIT) return;
        break;

    case LIFT_ACTION_PROFILE_WRITE_DECEL:
        step = Lift_AsyncWriteI32(0x6084U,
                                  (int32_t)LIFT_PROFILE_DECEL_PUL_S2,
                                  now);
        if (step == LIFT_ACTION_STEP_DONE) {
            Lift_SetActionPhase(LIFT_ACTION_CIA402_WRITE_0000, now);
            return;
        }
        if (step == LIFT_ACTION_STEP_WAIT) return;
        break;

    case LIFT_ACTION_JOG_WRITE_MODE:
        step = Lift_AsyncWriteI8(0x6060U, Lift_JogVelocityMode(), now);
        if (step == LIFT_ACTION_STEP_DONE) {
            Lift_SetActionPhase(LIFT_ACTION_JOG_WRITE_ZERO, now);
            return;
        }
        if (step == LIFT_ACTION_STEP_WAIT) return;
        break;

    case LIFT_ACTION_JOG_WRITE_ZERO:
        step = Lift_AsyncWriteI32(0x60FFU, 0, now);
        if (step == LIFT_ACTION_STEP_DONE) {
            Lift_SetActionPhase(LIFT_ACTION_JOG_WRITE_ACCEL, now);
            return;
        }
        if (step == LIFT_ACTION_STEP_WAIT) return;
        break;

    case LIFT_ACTION_JOG_WRITE_ACCEL:
        step = Lift_AsyncWriteI32(0x6083U,
                                  (int32_t)Lift_JogAccelPulS2(),
                                  now);
        if (step == LIFT_ACTION_STEP_DONE) {
            Lift_SetActionPhase(LIFT_ACTION_JOG_WRITE_DECEL, now);
            return;
        }
        if (step == LIFT_ACTION_STEP_WAIT) return;
        break;

    case LIFT_ACTION_JOG_WRITE_DECEL:
        step = Lift_AsyncWriteI32(0x6084U,
                                  (int32_t)Lift_JogAccelPulS2(),
                                  now);
        if (step == LIFT_ACTION_STEP_DONE) {
            Lift_SetActionPhase(LIFT_ACTION_CIA402_WRITE_0000, now);
            return;
        }
        if (step == LIFT_ACTION_STEP_WAIT) return;
        break;

    case LIFT_ACTION_CIA402_WRITE_0000:
        step = Lift_AsyncWriteU16(0x6040U, 0x0000U, now);
        if (step == LIFT_ACTION_STEP_DONE) {
            Lift_SetActionPhase(LIFT_ACTION_CIA402_WAIT_0000, now);
            return;
        }
        if (step == LIFT_ACTION_STEP_WAIT) return;
        break;

    case LIFT_ACTION_CIA402_WAIT_0000:
        if ((uint32_t)(now - s_lift_pending.phase_start_ms) <
            LIFT_ACTION_CIA402_STEP_DELAY_MS) return;
        Lift_SetActionPhase(LIFT_ACTION_CIA402_WRITE_0006, now);
        return;

    case LIFT_ACTION_CIA402_WRITE_0006:
        step = Lift_AsyncWriteU16(0x6040U, 0x0006U, now);
        if (step == LIFT_ACTION_STEP_DONE) {
            Lift_SetActionPhase(LIFT_ACTION_CIA402_WAIT_0006, now);
            return;
        }
        if (step == LIFT_ACTION_STEP_WAIT) return;
        break;

    case LIFT_ACTION_CIA402_WAIT_0006:
        if ((uint32_t)(now - s_lift_pending.phase_start_ms) <
            LIFT_ACTION_CIA402_STEP_DELAY_MS) return;
        Lift_SetActionPhase(LIFT_ACTION_CIA402_WRITE_0007, now);
        return;

    case LIFT_ACTION_CIA402_WRITE_0007:
        step = Lift_AsyncWriteU16(0x6040U, 0x0007U, now);
        if (step == LIFT_ACTION_STEP_DONE) {
            Lift_SetActionPhase(LIFT_ACTION_CIA402_WAIT_0007, now);
            return;
        }
        if (step == LIFT_ACTION_STEP_WAIT) return;
        break;

    case LIFT_ACTION_CIA402_WAIT_0007:
        if ((uint32_t)(now - s_lift_pending.phase_start_ms) <
            LIFT_ACTION_CIA402_STEP_DELAY_MS) return;
        Lift_SetActionPhase(LIFT_ACTION_CIA402_WRITE_000F, now);
        return;

    case LIFT_ACTION_CIA402_WRITE_000F:
        step = Lift_AsyncWriteU16(0x6040U, 0x000FU, now);
        if (step == LIFT_ACTION_STEP_DONE) {
            Lift_SetActionPhase(LIFT_ACTION_CIA402_WAIT_000F, now);
            return;
        }
        if (step == LIFT_ACTION_STEP_WAIT) return;
        break;

    case LIFT_ACTION_CIA402_WAIT_000F:
        if ((uint32_t)(now - s_lift_pending.phase_start_ms) <
            LIFT_ACTION_CIA402_STEP_DELAY_MS) return;
        Lift_SetActionPhase(Lift_PendingIsProfile() ?
                           LIFT_ACTION_PROFILE_WRITE_START :
                           LIFT_ACTION_JOG_WRITE_TARGET,
                           now);
        return;

    case LIFT_ACTION_PROFILE_WRITE_START:
        step = Lift_AsyncWriteU16(0x6040U, 0x001FU, now);
        if (step == LIFT_ACTION_STEP_DONE) {
            Lift_CompletePendingProfile(now);
            return;
        }
        if (step == LIFT_ACTION_STEP_WAIT) return;
        break;

    case LIFT_ACTION_JOG_WRITE_TARGET:
        step = Lift_AsyncWriteI32(0x60FFU,
                                  Lift_JogSpeedPulS(s_lift_pending.upward),
                                  now);
        if (step == LIFT_ACTION_STEP_DONE) {
            Lift_CompletePendingJog(now);
            return;
        }
        if (step == LIFT_ACTION_STEP_WAIT) return;
        break;

    case LIFT_ACTION_RESET_ASSERT:
        step = Lift_AsyncWriteU16(0x6040U, 0x0080U, now);
        if (step == LIFT_ACTION_STEP_DONE) {
            Lift_SetActionPhase(LIFT_ACTION_RESET_WAIT_ASSERT, now);
            return;
        }
        if (step == LIFT_ACTION_STEP_WAIT) return;
        break;

    case LIFT_ACTION_RESET_WAIT_ASSERT:
        if ((uint32_t)(now - s_lift_pending.phase_start_ms) <
            LIFT_FAULT_RESET_ASSERT_MS) return;
        Lift_SetActionPhase(LIFT_ACTION_RESET_RELEASE, now);
        return;

    case LIFT_ACTION_RESET_RELEASE:
        step = Lift_AsyncWriteU16(0x6040U, 0x0000U, now);
        if (step == LIFT_ACTION_STEP_DONE) {
            Lift_SetActionPhase(LIFT_ACTION_RESET_WAIT_SETTLE, now);
            return;
        }
        if (step == LIFT_ACTION_STEP_WAIT) return;
        break;

    case LIFT_ACTION_RESET_WAIT_SETTLE:
        if ((uint32_t)(now - s_lift_pending.phase_start_ms) <
            LIFT_FAULT_RESET_SETTLE_MS) return;
        Lift_SetActionPhase(LIFT_ACTION_RESET_VERIFY_STATUS, now);
        return;

    case LIFT_ACTION_RESET_VERIFY_STATUS:
        step = Lift_AsyncReadU16(0x6041U, now, &value);
        if (step == LIFT_ACTION_STEP_DONE) {
            s_lift_pending.statusword = (uint16_t)value;
            Lift_SetActionPhase(LIFT_ACTION_RESET_VERIFY_ERROR, now);
            return;
        }
        if (step == LIFT_ACTION_STEP_WAIT) return;
        break;

    case LIFT_ACTION_RESET_VERIFY_ERROR:
        step = Lift_AsyncReadU16(0x603FU, now, &value);
        if (step == LIFT_ACTION_STEP_WAIT) return;
        if (step == LIFT_ACTION_STEP_FAILED) break;
        s_lift_pending.error_code = (uint16_t)value;
        if (Statusword_HasFault(s_lift_pending.statusword) ||
            (s_lift_pending.error_code != 0U)) {
            Lift_FailPendingAction(LIFT_FAULT_REASON_DRIVE_FAULT,
                                   "node3 fault reset verification failed",
                                   now,
                                   true);
            return;
        }
        Lift_CompletePendingReset(now);
        return;

    case LIFT_ACTION_CANCEL_WRITE_JOG_ZERO:
        step = Lift_AsyncWriteI32(0x60FFU, 0, now);
        if (step == LIFT_ACTION_STEP_WAIT) return;
        Lift_SetActionPhase(LIFT_ACTION_CANCEL_WRITE_DECEL, now);
        return;

    case LIFT_ACTION_CANCEL_WRITE_DECEL:
        step = Lift_AsyncWriteU16(0x6040U, 0x010FU, now);
        if (step == LIFT_ACTION_STEP_WAIT) return;
        if (step == LIFT_ACTION_STEP_DONE) {
            Lift_CompletePendingCancel(now);
            return;
        }
        Lift_SetActionPhase(LIFT_ACTION_CANCEL_WRITE_DISABLE, now);
        return;

    case LIFT_ACTION_CANCEL_WRITE_DISABLE:
        step = Lift_AsyncWriteU16(0x6040U, 0x0002U, now);
        if (step == LIFT_ACTION_STEP_WAIT) return;
        if (step == LIFT_ACTION_STEP_DONE) {
            Lift_CompletePendingCancel(now);
            return;
        }
        Lift_FailPendingAction(LIFT_FAULT_REASON_ACTION_START_FAILED,
                               "cancel stop submission failed",
                               now,
                               true);
        return;

    case LIFT_ACTION_PHASE_NONE:
    default:
        break;
    }

    Lift_FailPendingAction(LIFT_FAULT_REASON_ACTION_START_FAILED,
                           "node3 action phase failed",
                           now,
                           true);
}

void Lift_Update(void)
{
    uint32_t now = HAL_GetTick();
    uint16_t emcy_code = 0U;
    uint16_t statusword = 0U;
    int32_t position_pul = 0;
    bool position_valid = false;
    bool target_bit_set = false;
    bool position_at_target = false;

    if (Canopen_TakeLiftFaultEmcy(&emcy_code)) {
        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_CRITICAL,
            "LIFT EMCY node=%u code=0x%04X action=FAULT\r\n",
            (unsigned)LIFT_NODE_ID, (unsigned)emcy_code);
        if (s_lift_pending.action != LIFT_PENDING_NONE) {
            Lift_FailPendingAction(LIFT_FAULT_REASON_DRIVE_FAULT,
                                   "node3 EMCY", now, false);
        } else if (Lift_IsRecoveryActive()) {
            Lift_FailSensorHoming(LIFT_FAULT_REASON_DRIVE_FAULT,
                                  "node3 EMCY", false);
        } else {
            Lift_SetFaultReasonWithoutDriveRead(LIFT_FAULT_REASON_DRIVE_FAULT);
            s_lift_fault_code = emcy_code;
            Lift_SetState(LIFT_STATE_FAULT);
            Lift_SendJetsonStatusLine();
        }
        return;
    }

    if (s_lift_pending.action != LIFT_PENDING_NONE) {
        Lift_ServicePendingAction(now);
        return;
    }

    if (Lift_IsRecoveryActive()) {
        Lift_UpdateRecovery(now);
        return;
    }

    if (!Lift_IsProfileMoving()) {
        Lift_UpdateIdleHealth(now);
        return;
    }

    if ((now - s_lift_last_poll_ms) < 20U) {
        return;
    }
    s_lift_last_poll_ms = now;

    if (!Lift_ReadStatusQuiet(&statusword)) {
        Lift_RecordHealthReadFailure();
        if (s_lift_state == LIFT_STATE_FAULT) {
            return;
        }
        if ((now - s_lift_move_start_ms) >= LIFT_MOVE_TIMEOUT_MS) {
            (void)Lift_RequestDecelStop();
            Lift_RecordFault(0x0001U);
            Lift_SetState(LIFT_STATE_FAULT);
            Lift_SendJetsonStatusLine();
        }
        return;
    }

    if (Statusword_HasFault(statusword)) {
        Lift_RecordFault(0x0001U);
        Lift_SetState(LIFT_STATE_FAULT);
        Lift_SendJetsonStatusLine();
        return;
    }

    position_valid = Lift_ReadActualPositionQuiet(&position_pul);
    if (position_valid) {
        s_lift_pos_pul = position_pul;
    }

    target_bit_set = ((statusword & (1U << 10)) != 0U);
    position_at_target = position_valid &&
        (Lift_AbsI32(s_lift_target_pul - s_lift_pos_pul) <= Lift_PositionTolerancePul());

    if (target_bit_set && !position_at_target &&
        ((now - s_lift_last_target_mismatch_ms) >= 1000U)) {
        s_lift_last_target_mismatch_ms = now;
        uart_printf("LIFT target bit set but position mismatch pos=%ld target=%ld\r\n",
                    (long)s_lift_pos_pul,
                    (long)s_lift_target_pul);
    }

    if (position_at_target) {
        (void)Lift_WriteControlword(0x000FU);
        s_lift_pos_pul = s_lift_target_pul;
        if (s_lift_target_pul == LIFT_DOWN_PUL) {
            Lift_SetState(LIFT_STATE_IDLE_DOWN);
        } else {
            Lift_SetState(LIFT_STATE_IDLE_UP);
        }
        Lift_SendJetsonStatusLine();
        return;
    }

    if ((now - s_lift_move_start_ms) >= LIFT_MOVE_TIMEOUT_MS) {
        (void)Lift_RequestDecelStop();
        Lift_RecordFault(0x0001U);
        Lift_SetState(LIFT_STATE_FAULT);
        JetsonTx_Printf(JETSON_TX_PRIORITY_CRITICAL,
                        "LIFT move timeout\r\n");
        Lift_SendJetsonStatusLine();
    }
}

bool Lift_HandleCommand(const char *cmd)
{
    if ((strcmp(cmd, "LIFT_FAULT_RESET") == 0) ||
        (strcmp(cmd, "LIFT_RST") == 0) ||
        (strcmp(cmd, "FAULT_RST") == 0)) {
        uint32_t nonce = s_lift_command_nonce;
        uint32_t txn;

        if ((s_lift_pending.action != LIFT_PENDING_NONE) ||
            Lift_IsProfileMoving() || Lift_IsRecoveryActive()) {
            /* Another lift operation is in flight. Bind an honest BUSY
             * failure to the NEW request; the active transaction keeps
             * its own binding and continues unchanged. */
            txn = ++s_lift_txn_counter;
            (void)JetsonTx_Printf(
                JETSON_TX_PRIORITY_STATUS,
                "LIFT_FAULT_RESET state=FAILED reason=BUSY active_txn=%lu txn=%lu req=%08lX\r\n",
                (unsigned long)Lift_ActiveBoundTxn(),
                (unsigned long)txn,
                (unsigned long)nonce);
            uart_printf("LIFT command rejected: action already pending/moving\r\n");
            Lift_SendJetsonStatusLine();
            return true;
        }
        s_lift_reset_txn = ++s_lift_txn_counter;
        s_lift_reset_nonce = nonce;
        s_lift_reset_bound = true;
        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_STATUS,
            "LIFT_FAULT_RESET state=STARTED txn=%lu req=%08lX\r\n",
            (unsigned long)s_lift_reset_txn,
            (unsigned long)nonce);
        (void)Lift_ResetFault();
        return true;
    }

    if ((strcmp(cmd, "LIFT_RAW_STATUS") == 0) ||
        (strcmp(cmd, "LIFT_RAW") == 0)) {
        Lift_SendRawStatus();
        return true;
    }

    if (strcmp(cmd, "LIFT_SCALE_INFO") == 0) {
        Lift_SendScaleInfo();
        return true;
    }

    if ((strcmp(cmd, "LIFT_HOME") == 0) ||
        (strcmp(cmd, "LIFT_RECOVER") == 0) ||
        (strcmp(cmd, "LIFT_RECOVER_TOP") == 0)) {
        uint32_t nonce = s_lift_command_nonce;
        uint32_t txn;

        if ((s_lift_pending.action != LIFT_PENDING_NONE) ||
            Lift_IsProfileMoving() || Lift_IsRecoveryActive()) {
            txn = ++s_lift_txn_counter;
            (void)JetsonTx_Printf(
                JETSON_TX_PRIORITY_STATUS,
                "LIFT_HOME state=FAILED reason=BUSY active_txn=%lu txn=%lu req=%08lX\r\n",
                (unsigned long)Lift_ActiveBoundTxn(),
                (unsigned long)txn,
                (unsigned long)nonce);
            uart_printf("LIFT command rejected: action already pending/moving\r\n");
            Lift_SendJetsonStatusLine();
            return true;
        }
        s_lift_home_txn = ++s_lift_txn_counter;
        s_lift_home_nonce = nonce;
        s_lift_home_bound = true;
        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_STATUS,
            "LIFT_HOME state=STARTED txn=%lu req=%08lX\r\n",
            (unsigned long)s_lift_home_txn,
            (unsigned long)nonce);
        (void)Lift_StartSensorHoming();
        return true;
    }

    if ((strcmp(cmd, "LIFT_JOG_UP_SLOW") == 0) ||
        (strcmp(cmd, "LIFT_JOG_UP") == 0)) {
        (void)Lift_StartSlowJog(true);
        return true;
    }

    if ((strcmp(cmd, "LIFT_JOG_DOWN_SLOW") == 0) ||
        (strcmp(cmd, "LIFT_JOG_DN") == 0)) {
        (void)Lift_StartSlowJog(false);
        return true;
    }

    if ((strcmp(cmd, "LIFT_JOG_STOP") == 0) ||
        (strcmp(cmd, "LIFT_JOG_STP") == 0)) {
        Lift_StopCommand();
        return true;
    }

    if (strcmp(cmd, "LIFT_UP") == 0) {
        (void)Lift_StartProfileMove(LIFT_UP_PUL, LIFT_STATE_MOVING_UP);
        return true;
    }

    if (strcmp(cmd, "LIFT_DOWN") == 0) {
        (void)Lift_StartProfileMove(LIFT_DOWN_PUL, LIFT_STATE_MOVING_DOWN);
        return true;
    }

    if (strcmp(cmd, "LIFT_STOP") == 0) {
        Lift_StopCommand();
        return true;
    }

    if (strcmp(cmd, "LIFT_STATUS") == 0) {
        Lift_SendJetsonStatusNow();
        return true;
    }

    return false;
}

#ifdef AMR_HOST_TEST
void Lift_TestResetState(void)
{
    s_lift_state = LIFT_STATE_RECOVERY_REQUIRED;
    s_lift_recovery_phase = LIFT_RECOVERY_PHASE_NONE;
    s_lift_homed = false;
    s_lift_pos_pul = 0;
    s_lift_target_pul = LIFT_DOWN_PUL;
    s_lift_fault_code = 0U;
    s_lift_move_start_ms = 0U;
    s_lift_last_poll_ms = 0U;
    s_lift_health_last_poll_ms = 0U;
    s_lift_health_missed_count = 0U;
    s_lift_recovery_start_ms = 0U;
    s_lift_recovery_last_poll_ms = 0U;
    s_lift_last_target_mismatch_ms = 0U;
    s_lift_fault_reason = LIFT_FAULT_REASON_NONE;
    s_lift_home_phase = HOME_FAILED;
    s_lift_home_phase_start_ms = 0U;
    s_lift_home_last_poll_ms = 0U;
    s_lift_home_action_started = false;
    s_lift_home_request_pending = false;
    s_lift_homing_start_edge_issued = false;
    s_lift_home_last_displayed_mode = 0;
    s_lift_home_display_seen = false;
    s_homing_pre_start_statusword = 0U;
    s_homing_reference_seen_clear = false;
    s_lift_jog_last_command_ms = 0U;
    s_lift_jog_upward = true;
    memset(&s_lift_pending, 0, sizeof(s_lift_pending));
    s_lift_txn_counter = 0U;
    s_lift_reset_txn = 0U;
    s_lift_reset_nonce = 0U;
    s_lift_reset_bound = false;
    s_lift_home_txn = 0U;
    s_lift_home_nonce = 0U;
    s_lift_home_bound = false;
    s_lift_command_nonce = 0U;
    Drive_CancelConfirmedStopForLift();
}

void Lift_TestSetMoving(uint32_t start_ms)
{
    s_lift_state = LIFT_STATE_MOVING_UP;
    s_lift_move_start_ms = start_ms;
    s_lift_last_poll_ms = start_ms;
}

void Lift_TestSetPendingForEmcy(void)
{
    s_lift_pending.action = LIFT_PENDING_MOVE_UP;
    s_lift_pending.phase = LIFT_ACTION_WAIT_BASE_STOP;
}

void Lift_TestSetRecoveryForEmcy(void)
{
    s_lift_recovery_phase = LIFT_RECOVERY_PHASE_JOG;
    s_lift_state = LIFT_STATE_RECOVERING_UP;
}

void Lift_TestSetHomed(bool homed, int32_t position_pul)
{
    s_lift_homed = homed;
    s_lift_pos_pul = position_pul;
    s_lift_target_pul = position_pul;
    s_lift_state = homed ? Lift_ClassifyRelativePosition(position_pul) :
                           LIFT_STATE_RECOVERY_REQUIRED;
}

uint32_t Lift_TestPendingAction(void)
{
    return (uint32_t)s_lift_pending.action;
}

uint32_t Lift_TestPendingPhase(void)
{
    return (uint32_t)s_lift_pending.phase;
}

uint32_t Lift_TestState(void)
{
    return (uint32_t)s_lift_state;
}

uint32_t Lift_TestFaultReason(void)
{
    return (uint32_t)s_lift_fault_reason;
}
#endif
