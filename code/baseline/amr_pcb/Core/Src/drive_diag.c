/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : drive_diag.c
  * @brief          : Private Drive module implementation.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "drive.h"
#include "app_config.h"
#include "canopen.h"
#include "diagnostics.h"
#include "drive_internal.h"
#include "wheel_pdo_config.h"
#include "firmware_identity.h"
#include "jetson_tx.h"
#include "main.h"
#include "motion_control.h"
#include "odom.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

static uint32_t Drive_TpdoDiagnosticAge(uint32_t now_ms,
                                        bool seen,
                                        uint32_t timestamp_ms);

static void Print_Statusword_Bits(uint16_t statusword)
{
    uart_printf("bits: ready=%u switched_on=%u operation_enabled=%u fault=%u\r\n",
                (statusword & (1U << 0)) ? 1U : 0U,
                (statusword & (1U << 1)) ? 1U : 0U,
                (statusword & (1U << 2)) ? 1U : 0U,
                (statusword & (1U << 3)) ? 1U : 0U);
    uart_printf("bits: voltage_enabled=%u quick_stop=%u switch_on_disabled=%u warning=%u\r\n",
                (statusword & (1U << 4)) ? 1U : 0U,
                (statusword & (1U << 5)) ? 1U : 0U,
                (statusword & (1U << 6)) ? 1U : 0U,
                (statusword & (1U << 7)) ? 1U : 0U);
    uart_printf("bits: remote=%u target_reached=%u limit=%u commutation_found=%u\r\n",
                (statusword & (1U << 9)) ? 1U : 0U,
                (statusword & (1U << 10)) ? 1U : 0U,
                (statusword & (1U << 11)) ? 1U : 0U,
                (statusword & (1U << 14)) ? 1U : 0U);
}

uint16_t Read_And_Print_Statusword(uint8_t node_id)
{
    uint16_t statusword = 0U;

    if (!SDO_Read_U16(node_id, 0x6041, 0x00, &statusword)) {
        uart_printf("Node %u statusword read failed\r\n", node_id);
        return 0U;
    }

    if (DEBUG_STATUSWORD) {
        uart_printf("Node %u statusword 0x6041=0x%04X\r\n", node_id, statusword);
        Print_Statusword_Bits(statusword);
    }
    return statusword;
}

void Read_And_Print_ActualSpeed(uint8_t node_id)
{
    int32_t actual_speed = 0;

    if (Kinco_ReadActualSpeed(node_id, &actual_speed)) {
        int32_t raw_rpm = Motion_RoundFloatToI32(
            Motion_RawVelocityDecToMotorRPM(actual_speed));
        int32_t logical_rpm = Motion_RoundFloatToI32(
            Motion_RawVelocityDecToLogicalMotorRPM(node_id, actual_speed));

        uart_printf("Node %u actual speed raw=%ld DEC, raw_rpm=%ld, logical_rpm=%ld\r\n",
                    node_id,
                    (long)actual_speed,
                    (long)raw_rpm,
                    (long)logical_rpm);
    } else {
        uart_printf("Node %u actual speed read failed\r\n", node_id);
    }
}

static void Read_And_Print_ActualPosition(uint8_t node_id)
{
    int32_t actual_position = 0;

    if (Kinco_ReadActualPosition(node_id, &actual_position)) {
        uart_printf("Node %u actual position 0x6063=%ld counts\r\n",
                    node_id, (long)actual_position);
    } else {
        uart_printf("Node %u actual position read failed\r\n", node_id);
    }
}

bool Odom_UpdateAndPrint(void)
{
    return Odom_Update(true);
}

uint32_t Drive_OdomFaultStatus(void)
{
#if DRIVE_BASIC_BRINGUP_PROFILE
    /*
     * In the basic profile the wire-level fault flag means a hard Drive
     * fault.  Stale feedback and historical diagnostic latches remain
     * visible through diagnostics but cannot masquerade as a motion fault.
     */
    return (s_drive.core.state == DRIVE_STATE_FAULT) ? 1U : 0U;
#else
    uint32_t now_ms = HAL_GetTick();
    bool feedback_trusted =
        s_drive.feedback.status_left_seen && s_drive.feedback.status_right_seen &&
        s_drive.feedback.status_left_valid && s_drive.feedback.status_right_valid &&
        ((uint32_t)(now_ms - s_drive.feedback.status_left_timestamp_ms) <=
         DRIVE_WHEEL_FEEDBACK_STALE_MS) &&
        ((uint32_t)(now_ms - s_drive.feedback.status_right_timestamp_ms) <=
         DRIVE_WHEEL_FEEDBACK_STALE_MS);

    return ((s_drive.core.state == DRIVE_STATE_FAULT) ||
            s_tracking_stop_latched ||
            s_disable_motion_inhibit_latched ||
            s_drive.safety.transition.active ||
            !feedback_trusted) ? 1U : 0U;
#endif
}

void Drive_ModeConfirmPrint(const char *state)
{
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_DEBUG,
        "DRIVE_MODE_REQUEST state=%s node=%u obj=6060 req=%d readback=%d valid=%u attempt=%lu elapsed=%lu sdo=%u abort=%08lX\r\n",
        state,
        (unsigned)s_drive.safety.transition.mode_confirm_node_id,
        (int)s_drive.safety.transition.requested_mode,
        (int)s_drive.safety.transition.last_mode_readback,
        s_drive.safety.transition.mode_readback_valid ? 1U : 0U,
        (unsigned long)s_drive.safety.transition.mode_confirm_attempt,
        (unsigned long)s_drive.safety.transition.mode_confirm_elapsed_ms,
        (unsigned)s_drive.safety.transition.mode_confirm_last_sdo_result,
        (unsigned long)s_drive.safety.transition.mode_confirm_last_abort_code);
}

void Drive_ModeConfirmPrintTimeout(void)
{
    CanopenRuntimeDiagnostics_t diag = Canopen_GetRuntimeDiagnostics();
    uint16_t statusword = 0U;
    bool statusword_valid = false;

    if (s_drive.safety.transition.mode_confirm_node_id == LEFT_NODE_ID) {
        statusword = s_drive.statusword.left_statusword;
        statusword_valid = s_drive.statusword.left_statusword_valid &&
            (s_drive.statusword.left_statusword_generation ==
             s_drive.safety.transition.generation);
    } else if (s_drive.safety.transition.mode_confirm_node_id ==
               RIGHT_NODE_ID) {
        statusword = s_drive.statusword.right_statusword;
        statusword_valid = s_drive.statusword.right_statusword_valid &&
            (s_drive.statusword.right_statusword_generation ==
             s_drive.safety.transition.generation);
    }
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_STATUS,
        "DRIVE_MODE_REQUEST state=TIMEOUT node=%u obj=6060 req=%d readback=%d valid=%u attempts=%lu elapsed=%lu sw=0x%04X/%u sdo=%u sdo_to=%lu abort=%08lX\r\n",
        (unsigned)s_drive.safety.transition.mode_confirm_node_id,
        (int)s_drive.safety.transition.requested_mode,
        (int)s_drive.safety.transition.last_mode_readback,
        s_drive.safety.transition.mode_readback_valid ? 1U : 0U,
        (unsigned long)s_drive.safety.transition.mode_confirm_attempt,
        (unsigned long)s_drive.safety.transition.mode_confirm_elapsed_ms,
        (unsigned)statusword,
        statusword_valid ? 1U : 0U,
        (unsigned)s_drive.safety.transition.mode_confirm_last_sdo_result,
        (unsigned long)diag.sdo_timeout_count,
        (unsigned long)s_drive.safety.transition.mode_confirm_last_abort_code);
}

void Drive_EnableStatePrint(uint16_t controlword,
                                   uint16_t statusword,
                                   uint16_t expected_state)
{
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_DEBUG,
        "DRIVE_ENABLE_STATE node=%u command=%04X sw=0x%04X ds402=0x%02X expected=0x%02X fault=%u op=%u\r\n",
        (unsigned)s_drive.safety.transition.enable_node_id,
        (unsigned)controlword,
        (unsigned)statusword,
        (unsigned)Statusword_Ds402State(statusword),
        (unsigned)expected_state,
        Statusword_HasFault(statusword) ? 1U : 0U,
        Statusword_IsOperationEnabled(statusword) ? 1U : 0U);
}

const char *Drive_SafetyFailureName(DriveSafetyFailureReason_t reason)
{
    switch (reason) {
        case DRIVE_SAFETY_FAILURE_ZERO_SUBMIT: return "ZERO_SUBMIT";
        case DRIVE_SAFETY_FAILURE_ZERO_ACK: return "ZERO_ACK";
        case DRIVE_SAFETY_FAILURE_ZERO_FEEDBACK: return "ZERO_FEEDBACK";
        case DRIVE_SAFETY_FAILURE_SDO_TIMEOUT: return "SDO_TIMEOUT";
        case DRIVE_SAFETY_FAILURE_SDO_ABORT: return "SDO_ABORT";
        case DRIVE_SAFETY_FAILURE_SDO_PROTOCOL: return "SDO_PROTOCOL";
        case DRIVE_SAFETY_FAILURE_NODE_FAULT: return "NODE_FAULT";
        case DRIVE_SAFETY_FAILURE_STATUS_FAULT: return "STATUS_FAULT";
        case DRIVE_SAFETY_FAILURE_HEARTBEAT: return "HEARTBEAT";
        case DRIVE_SAFETY_FAILURE_ENABLE_CONFIRM: return "ENABLE_CONFIRM";
        case DRIVE_SAFETY_FAILURE_DISABLE_CONFIRM: return "DISABLE_CONFIRM";
        case DRIVE_SAFETY_FAILURE_RUNTIME_CAN: return "RUNTIME_CAN";
        case DRIVE_SAFETY_FAILURE_TPDO_FEEDBACK:
            return "TPDO_ACTIVE_FEEDBACK_STALE";
        case DRIVE_SAFETY_FAILURE_OVERALL_TIMEOUT: return "OVERALL_TIMEOUT";
        case DRIVE_SAFETY_FAILURE_NONE:
        default: return "NONE";
    }
}

#if FIRMWARE_BUILD_PROFILE_ID == FIRMWARE_PROFILE_BENCH_DEBUG
static const char *Drive_SafetyTypeName(DriveSafetyTransitionType_t type)
{
    switch (type) {
        case DRIVE_SAFETY_STOP: return "STOP";
        case DRIVE_SAFETY_DISABLE: return "DISABLE";
        case DRIVE_SAFETY_ENABLE: return "ENABLE";
        case DRIVE_SAFETY_ENABLE_ROLLBACK: return "EN_ROLLBACK";
        case DRIVE_SAFETY_DISABLE_ROLLBACK: return "DIS_ROLLBACK";
        case DRIVE_SAFETY_INIT_DISABLE: return "INIT_DISABLE";
        case DRIVE_SAFETY_NONE:
        default: return "NONE";
    }
}

static const char *Drive_SafetyPhaseName(DriveSafetyPhase_t phase)
{
    switch (phase) {
        case DRIVE_SAFETY_PHASE_ZERO_START: return "ZERO_START";
        case DRIVE_SAFETY_PHASE_ZERO_WAIT_ACK: return "ZERO_ACK";
        case DRIVE_SAFETY_PHASE_ZERO_WAIT_MEASURED: return "ZERO_MEASURED";
        case DRIVE_SAFETY_PHASE_ENABLE_READ_INITIAL_STATUS: return "EN_STATUS0";
        case DRIVE_SAFETY_PHASE_ENABLE_HB_WRITE_100C: return "EN_HB_W_100C";
        case DRIVE_SAFETY_PHASE_ENABLE_HB_READ_100C: return "EN_HB_R_100C";
        case DRIVE_SAFETY_PHASE_ENABLE_HB_WRITE_100D: return "EN_HB_W_100D";
        case DRIVE_SAFETY_PHASE_ENABLE_HB_READ_100D: return "EN_HB_R_100D";
        case DRIVE_SAFETY_PHASE_ENABLE_HB_WRITE_1017: return "EN_HB_W_1017";
        case DRIVE_SAFETY_PHASE_ENABLE_HB_READ_1017: return "EN_HB_R_1017";
        case DRIVE_SAFETY_PHASE_ENABLE_HB_WRITE_1016_01: return "EN_HB_W_1016_01";
        case DRIVE_SAFETY_PHASE_ENABLE_HB_READ_1016_01: return "EN_HB_R_1016_01";
        case DRIVE_SAFETY_PHASE_ENABLE_HB_WRITE_6007: return "EN_HB_W_6007";
        case DRIVE_SAFETY_PHASE_ENABLE_HB_READ_6007: return "EN_HB_R_6007";
        case DRIVE_SAFETY_PHASE_ENABLE_HB_QUALIFY: return "EN_HB_QUALIFY";
        case DRIVE_SAFETY_PHASE_ENABLE_SET_MODE: return "EN_SET_MODE";
        case DRIVE_SAFETY_PHASE_ENABLE_READ_MODE_REQUEST:
            return "EN_READ_MODE_REQ";
        case DRIVE_SAFETY_PHASE_ENABLE_WRITE_ZERO: return "EN_ZERO";
        case DRIVE_SAFETY_PHASE_ENABLE_WRITE_0006: return "EN_W_0006";
        case DRIVE_SAFETY_PHASE_ENABLE_READ_0006_STATUS: return "EN_R_0006";
        case DRIVE_SAFETY_PHASE_ENABLE_WRITE_0007: return "EN_W_0007";
        case DRIVE_SAFETY_PHASE_ENABLE_READ_0007_STATUS: return "EN_R_0007";
        case DRIVE_SAFETY_PHASE_ENABLE_WRITE_000F: return "EN_W_000F";
        case DRIVE_SAFETY_PHASE_ENABLE_READ_000F_STATUS: return "EN_R_000F";
        case DRIVE_SAFETY_PHASE_ENABLE_WRITE_002F: return "EN_W_002F";
        case DRIVE_SAFETY_PHASE_ENABLE_READ_002F_STATUS: return "EN_R_002F";
        case DRIVE_SAFETY_PHASE_ENABLE_NODE_DONE: return "EN_NODE_DONE";
        case DRIVE_SAFETY_PHASE_ENABLE_FINAL_STATUS_LEFT:
            return "EN_FINAL_LEFT";
        case DRIVE_SAFETY_PHASE_ENABLE_FINAL_STATUS_RIGHT:
            return "EN_FINAL_RIGHT";
        case DRIVE_SAFETY_PHASE_DISABLE_WRITE_LEFT: return "DIS_WRITE_L";
        case DRIVE_SAFETY_PHASE_DISABLE_WRITE_RIGHT: return "DIS_WRITE_R";
        case DRIVE_SAFETY_PHASE_DISABLE_STATUS_WAIT: return "DIS_WAIT";
        case DRIVE_SAFETY_PHASE_DISABLE_STATUS_READ_LEFT: return "DIS_READ_L";
        case DRIVE_SAFETY_PHASE_DISABLE_STATUS_READ_RIGHT: return "DIS_READ_R";
        case DRIVE_SAFETY_PHASE_NONE:
        default: return "NONE";
    }
}
#endif

uint32_t Drive_DiagnosticAge(uint32_t now_ms,
                                    bool valid,
                                    uint32_t timestamp_ms)
{
    return valid ? (uint32_t)(now_ms - timestamp_ms) : UINT32_MAX;
}

static void Drive_FillFaultSnapshot(DriveFaultSnapshot_t *snapshot,
                                    DriveSafetyFailureReason_t reason,
                                    uint8_t node_id,
                                    uint32_t now_ms,
                                    bool valid)
{
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->valid = valid;
    snapshot->event_ms = now_ms;
    snapshot->state = s_drive.core.state;
    snapshot->transition_type = s_drive.safety.transition.type;
    snapshot->phase = s_drive.safety.transition.phase;
    snapshot->reason = reason;
    snapshot->node_id = node_id;
    snapshot->left_statusword = s_drive.statusword.left_statusword;
    snapshot->right_statusword = s_drive.statusword.right_statusword;
    snapshot->left_statusword_valid = s_drive.statusword.left_statusword_valid;
    snapshot->right_statusword_valid = s_drive.statusword.right_statusword_valid;
    snapshot->left_statusword_timestamp_ms =
        s_drive.statusword.left_statusword_timestamp_ms;
    snapshot->right_statusword_timestamp_ms =
        s_drive.statusword.right_statusword_timestamp_ms;
    snapshot->transition_abort_code =
        s_drive.safety.transition.last_abort_code;
    snapshot->transition_timeout_count =
        s_drive.safety.transition.transient_timeout_count;
    snapshot->mode_confirm_attempt =
        s_drive.safety.transition.mode_confirm_attempt;
    snapshot->mode_confirm_elapsed_ms =
        s_drive.safety.transition.mode_confirm_elapsed_ms;
    snapshot->mode_confirm_last_abort_code =
        s_drive.safety.transition.mode_confirm_last_abort_code;
    snapshot->enable_0006_statusword =
        s_drive.safety.transition.enable_0006_statusword;
    snapshot->enable_0007_statusword =
        s_drive.safety.transition.enable_0007_statusword;
    snapshot->enable_000f_statusword =
        s_drive.safety.transition.enable_000f_statusword;
    snapshot->requested_mode = s_drive.safety.transition.requested_mode;
    snapshot->last_mode_readback =
        s_drive.safety.transition.last_mode_readback;
    snapshot->mode_confirm_node_id =
        s_drive.safety.transition.mode_confirm_node_id;
    snapshot->mode_confirm_last_sdo_result =
        s_drive.safety.transition.mode_confirm_last_sdo_result;
    snapshot->enable_0006_statusword_valid =
        s_drive.safety.transition.enable_0006_statusword_valid;
    snapshot->enable_0007_statusword_valid =
        s_drive.safety.transition.enable_0007_statusword_valid;
    snapshot->enable_000f_statusword_valid =
        s_drive.safety.transition.enable_000f_statusword_valid;
    snapshot->mode_readback_valid =
        s_drive.safety.transition.mode_readback_valid;
    snapshot->hardware_enable_confirmed = s_hardware_enable_confirmed;
    snapshot->hardware_disable_confirmed = s_hardware_disable_confirmed;
    snapshot->motion_inhibit_latched = s_disable_motion_inhibit_latched;
    snapshot->tracking_stop_latched = s_tracking_stop_latched;
    snapshot->canopen = Canopen_GetRuntimeDiagnostics();
    if ((reason == DRIVE_SAFETY_FAILURE_SDO_ABORT) &&
        (snapshot->transition_abort_code == 0U) &&
        (snapshot->canopen.last_sdo_abort_code != 0U)) {
        snapshot->transition_abort_code =
            snapshot->canopen.last_sdo_abort_code;
    }
}

#if FIRMWARE_BUILD_PROFILE_ID == FIRMWARE_PROFILE_BENCH_DEBUG
static void Drive_PrintFaultSnapshot(const DriveFaultSnapshot_t *snapshot,
                                     JetsonTxPriority_t priority,
                                     uint32_t now_ms,
                                     unsigned snap,
                                     uint32_t request_nonce)
{
    uint32_t left_sw_age = Drive_DiagnosticAge(
        now_ms, snapshot->left_statusword_valid,
        snapshot->left_statusword_timestamp_ms);
    uint32_t right_sw_age = Drive_DiagnosticAge(
        now_ms, snapshot->right_statusword_valid,
        snapshot->right_statusword_timestamp_ms);
    uint32_t left_emcy_age = Drive_DiagnosticAge(
        now_ms, snapshot->canopen.emcy_sequence[LEFT_NODE_ID] != 0U,
        snapshot->canopen.last_emcy_ms[LEFT_NODE_ID]);
    uint32_t right_emcy_age = Drive_DiagnosticAge(
        now_ms, snapshot->canopen.emcy_sequence[RIGHT_NODE_ID] != 0U,
        snapshot->canopen.last_emcy_ms[RIGHT_NODE_ID]);
    uint32_t left_hb_age = Drive_DiagnosticAge(
        now_ms, snapshot->canopen.heartbeat_sequence[LEFT_NODE_ID] != 0U,
        snapshot->canopen.last_heartbeat_ms[LEFT_NODE_ID]);
    uint32_t right_hb_age = Drive_DiagnosticAge(
        now_ms, snapshot->canopen.heartbeat_sequence[RIGHT_NODE_ID] != 0U,
        snapshot->canopen.last_heartbeat_ms[RIGHT_NODE_ID]);
    uint32_t master_tx_age = Drive_DiagnosticAge(
        now_ms, snapshot->canopen.master_heartbeat_tx_success_count != 0U,
        snapshot->canopen.master_heartbeat_last_success_ms);

    (void)JetsonTx_Printf(
        priority,
        "DRIVE_FAULT_EVENT event=%u cur=%s evt=%s type=%s phase=%s reason=%s node=%u age=%lu snap=%u req=%08lX\r\n",
        snapshot->valid ? 1U : 0U,
        Drive_StateName(s_drive.core.state),
        Drive_StateName(snapshot->state),
        Drive_SafetyTypeName(snapshot->transition_type),
        Drive_SafetyPhaseName(snapshot->phase),
        Drive_SafetyFailureName(snapshot->reason),
        snapshot->node_id,
        (unsigned long)(now_ms - snapshot->event_ms),
        snap, (unsigned long)request_nonce);
    (void)JetsonTx_Printf(
        priority,
        "DRIVE_FAULT_STATUS L=0x%04X/%u/%lu R=0x%04X/%u/%lu en_node=%u sw6=0x%04X/%u sw7=0x%04X/%u swf=0x%04X/%u abort=0x%08lX sdo_to=%lu trans_to=%lu snap=%u req=%08lX\r\n",
        snapshot->left_statusword,
        snapshot->left_statusword_valid ? 1U : 0U,
        (unsigned long)left_sw_age,
        snapshot->right_statusword,
        snapshot->right_statusword_valid ? 1U : 0U,
        (unsigned long)right_sw_age,
        (unsigned)snapshot->mode_confirm_node_id,
        snapshot->enable_0006_statusword,
        snapshot->enable_0006_statusword_valid ? 1U : 0U,
        snapshot->enable_0007_statusword,
        snapshot->enable_0007_statusword_valid ? 1U : 0U,
        snapshot->enable_000f_statusword,
        snapshot->enable_000f_statusword_valid ? 1U : 0U,
        (unsigned long)snapshot->transition_abort_code,
        (unsigned long)snapshot->canopen.sdo_timeout_count,
        (unsigned long)snapshot->transition_timeout_count,
        snap, (unsigned long)request_nonce);
    (void)JetsonTx_Printf(
        priority,
        "DRIVE_FAULT_EMCY L=0x%04X/%lu/%lu R=0x%04X/%lu/%lu mr=%u/%d/%d/%u/%lu/%lu/%u/%08lX snap=%u req=%08lX\r\n",
        snapshot->canopen.last_emcy_code[LEFT_NODE_ID],
        (unsigned long)snapshot->canopen.emcy_sequence[LEFT_NODE_ID],
        (unsigned long)left_emcy_age,
        snapshot->canopen.last_emcy_code[RIGHT_NODE_ID],
        (unsigned long)snapshot->canopen.emcy_sequence[RIGHT_NODE_ID],
        (unsigned long)right_emcy_age,
        (unsigned)snapshot->mode_confirm_node_id,
        (int)snapshot->requested_mode,
        (int)snapshot->last_mode_readback,
        snapshot->mode_readback_valid ? 1U : 0U,
        (unsigned long)snapshot->mode_confirm_attempt,
        (unsigned long)snapshot->mode_confirm_elapsed_ms,
        (unsigned)snapshot->mode_confirm_last_sdo_result,
        (unsigned long)snapshot->mode_confirm_last_abort_code,
        snap, (unsigned long)request_nonce);
    (void)JetsonTx_Printf(
        priority,
        "DRIVE_FAULT_HB mode=HEARTBEAT master=%u/0x%03X/%u tx=%lu/%lu/%lu/%lu age=%lu fault=%u/%u en=%u dis=%u inhibit=%u tracking=%u snap=%u req=%08lX\r\n",
        DRIVE_MASTER_HEARTBEAT_NODE_ID,
        DRIVE_MASTER_HEARTBEAT_COB_ID,
        DRIVE_MASTER_HEARTBEAT_PERIOD_MS,
        (unsigned long)snapshot->canopen.master_heartbeat_tx_attempt_count,
        (unsigned long)snapshot->canopen.master_heartbeat_tx_success_count,
        (unsigned long)snapshot->canopen.master_heartbeat_tx_failure_count,
        (unsigned long)snapshot->canopen.master_heartbeat_consecutive_failures,
        (unsigned long)master_tx_age,
        snapshot->canopen.heartbeat_fault_node,
        snapshot->canopen.heartbeat_pending_reason,
        snapshot->hardware_enable_confirmed ? 1U : 0U,
        snapshot->hardware_disable_confirmed ? 1U : 0U,
        snapshot->motion_inhibit_latched ? 1U : 0U,
        snapshot->tracking_stop_latched ? 1U : 0U,
        snap, (unsigned long)request_nonce);
    (void)JetsonTx_Printf(
        priority,
        "DRIVE_FAULT_HB_NODE node=%u state=0x%02X age=%lu rx=%lu configured=%u qualified=%u stale=%lu boot=%lu invalid=%lu reason=%u snap=%u req=%08lX\r\n",
        LEFT_NODE_ID,
        snapshot->canopen.last_heartbeat_state[LEFT_NODE_ID],
        (unsigned long)left_hb_age,
        (unsigned long)snapshot->canopen.heartbeat_receive_count[LEFT_NODE_ID],
        snapshot->canopen.heartbeat_configured[LEFT_NODE_ID],
        snapshot->canopen.heartbeat_qualified[LEFT_NODE_ID],
        (unsigned long)snapshot->canopen.heartbeat_stale_count[LEFT_NODE_ID],
        (unsigned long)snapshot->canopen.heartbeat_bootup_count[LEFT_NODE_ID],
        (unsigned long)snapshot->canopen.heartbeat_invalid_count[LEFT_NODE_ID],
        snapshot->canopen.heartbeat_fault_reason[LEFT_NODE_ID],
        snap, (unsigned long)request_nonce);
    (void)JetsonTx_Printf(
        priority,
        "DRIVE_FAULT_HB_NODE node=%u state=0x%02X age=%lu rx=%lu configured=%u qualified=%u stale=%lu boot=%lu invalid=%lu reason=%u snap=%u req=%08lX\r\n",
        RIGHT_NODE_ID,
        snapshot->canopen.last_heartbeat_state[RIGHT_NODE_ID],
        (unsigned long)right_hb_age,
        (unsigned long)snapshot->canopen.heartbeat_receive_count[RIGHT_NODE_ID],
        snapshot->canopen.heartbeat_configured[RIGHT_NODE_ID],
        snapshot->canopen.heartbeat_qualified[RIGHT_NODE_ID],
        (unsigned long)snapshot->canopen.heartbeat_stale_count[RIGHT_NODE_ID],
        (unsigned long)snapshot->canopen.heartbeat_bootup_count[RIGHT_NODE_ID],
        (unsigned long)snapshot->canopen.heartbeat_invalid_count[RIGHT_NODE_ID],
        snapshot->canopen.heartbeat_fault_reason[RIGHT_NODE_ID],
        snap, (unsigned long)request_nonce);
}
#endif

static uint32_t Drive_ZeroEvidenceAgeMs(uint32_t now_ms)
{
    uint32_t left_age;
    uint32_t right_age;

    if (!s_drive.evidence.zero_speed.valid) {
        return UINT32_MAX;
    }
    left_age = (uint32_t)(now_ms -
                          s_drive.evidence.zero_speed.left_timestamp_ms);
    right_age = (uint32_t)(now_ms -
                           s_drive.evidence.zero_speed.right_timestamp_ms);
    return (left_age > right_age) ? left_age : right_age;
}

#if FIRMWARE_BUILD_PROFILE_ID == FIRMWARE_PROFILE_BENCH_DEBUG
static void Drive_PrintFaultCurrent(uint32_t now_ms,
                                    unsigned snap,
                                    uint32_t request_nonce)
{
    bool zero_current = Drive_ZeroSpeedEvidenceIsCurrent(now_ms);

    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_DEBUG,
        "DRIVE_FAULT_CURRENT state=%s active=%u type=%u phase=%u en=%u dis=%u drive=%u tracking=%u inhibit=%u zero=%u age=%lu gen=%lu/%lu snap=%u req=%08lX\r\n",
        Drive_StateName(s_drive.core.state),
        s_drive.safety.transition.active ? 1U : 0U,
        (unsigned)s_drive.safety.transition.type,
        (unsigned)s_drive.safety.transition.phase,
        s_hardware_enable_confirmed ? 1U : 0U,
        s_hardware_disable_confirmed ? 1U : 0U,
        s_drive.core.enabled ? 1U : 0U,
        s_tracking_stop_latched ? 1U : 0U,
        s_disable_motion_inhibit_latched ? 1U : 0U,
        zero_current ? 1U : 0U,
        (unsigned long)Drive_ZeroEvidenceAgeMs(now_ms),
        (unsigned long)s_drive.evidence.zero_speed.generation,
        (unsigned long)s_drive.evidence.safety_generation,
        snap, (unsigned long)request_nonce);
}
#endif

void Drive_RecordFaultEvent(DriveSafetyFailureReason_t reason,
                                   uint8_t node_id,
                                   uint32_t now_ms)
{
    Drive_FillFaultSnapshot(&s_drive.diagnostic.last_fault_snapshot, reason,
                            node_id, now_ms, true);
}

void Drive_PrintFaultDebug(uint32_t request_nonce)
{
#if FIRMWARE_BUILD_PROFILE_ID == FIRMWARE_PROFILE_BENCH_DEBUG
    DriveFaultSnapshot_t live_snapshot;
    uint32_t now_ms = HAL_GetTick();
    /* Every line of one response shares snap= and echoes req= so a
     * host can never stitch lines from different print cycles (or a
     * stale queued response) into one diagnostic snapshot. */
    unsigned snap =
        (unsigned)(++s_drive.diagnostic.fault_debug_snapshot_sequence);

    Drive_PrintFaultCurrent(now_ms, snap, request_nonce);
    Drive_PrintFaultResetDiagnostic(now_ms, snap, request_nonce);
    if (s_drive.diagnostic.last_fault_snapshot.valid) {
        Drive_PrintFaultSnapshot(&s_drive.diagnostic.last_fault_snapshot,
                                 JETSON_TX_PRIORITY_DEBUG, now_ms,
                                 snap, request_nonce);
    } else {
        Drive_FillFaultSnapshot(&live_snapshot,
                                s_drive.safety.transition.failure_reason,
                                s_drive.safety.transition.failure_node_id,
                                now_ms, false);
        Drive_PrintFaultSnapshot(&live_snapshot,
                                 JETSON_TX_PRIORITY_DEBUG, now_ms,
                                 snap, request_nonce);
    }
#else
    (void)request_nonce;
#endif
}

void Drive_PrintHeartbeatCompatibilityDebug(uint32_t request_nonce)
{
    unsigned snap =
        (unsigned)(++s_drive.diagnostic.heartbeat_snapshot_sequence);

    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_DEBUG,
        "DRIVE_NODE_GUARD active=0 mode=HEARTBEAT snap=%u req=%08lX\r\n",
        snap, (unsigned long)request_nonce);
}

void Drive_PrintHeartbeatDebug(uint32_t request_nonce)
{
    CanopenRuntimeDiagnostics_t diag = Canopen_GetRuntimeDiagnostics();
    uint32_t now_ms = HAL_GetTick();
    uint32_t master_age = Drive_DiagnosticAge(
        now_ms, diag.master_heartbeat_tx_success_count != 0U,
        diag.master_heartbeat_last_success_ms);
    unsigned snap =
        (unsigned)(++s_drive.diagnostic.heartbeat_snapshot_sequence);
    uint8_t node_id;

    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_DEBUG,
        "DRIVE_HEARTBEAT 1/9 mode=HEARTBEAT master_node=%u cob_id=0x%03X state=0x%02X period_ms=%u failure_limit=%u snap=%u req=%08lX\r\n",
        DRIVE_MASTER_HEARTBEAT_NODE_ID,
        DRIVE_MASTER_HEARTBEAT_COB_ID,
        DRIVE_MASTER_HEARTBEAT_STATE,
        DRIVE_MASTER_HEARTBEAT_PERIOD_MS,
        DRIVE_MASTER_HEARTBEAT_TX_FAILURE_LIMIT,
        snap, (unsigned long)request_nonce);
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_DEBUG,
        "DRIVE_HEARTBEAT 2/9 tx_attempt=%lu tx_success=%lu tx_failures=%lu snap=%u req=%08lX\r\n",
        (unsigned long)diag.master_heartbeat_tx_attempt_count,
        (unsigned long)diag.master_heartbeat_tx_success_count,
        (unsigned long)diag.master_heartbeat_tx_failure_count,
        snap, (unsigned long)request_nonce);
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_DEBUG,
        "DRIVE_HEARTBEAT 3/9 consecutive_failures=%lu last_tx_age_ms=%lu fault_pending=%u fault_node=%u fault_reason=%u snap=%u req=%08lX\r\n",
        (unsigned long)diag.master_heartbeat_consecutive_failures,
        (unsigned long)master_age,
        diag.heartbeat_fault_pending,
        diag.heartbeat_fault_node,
        diag.heartbeat_pending_reason,
        snap, (unsigned long)request_nonce);

    for (node_id = LEFT_NODE_ID; node_id <= RIGHT_NODE_ID; node_id++) {
        uint8_t state = diag.last_heartbeat_state[node_id];
        bool valid_state = (state == 0x04U) || (state == 0x05U) ||
                           (state == 0x7FU);
        uint32_t age_ms = Drive_DiagnosticAge(
            now_ms, valid_state &&
                    (diag.heartbeat_receive_count[node_id] != 0U),
            diag.last_heartbeat_ms[node_id]);

        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_DEBUG,
            "DRIVE_HEARTBEAT %u/9 node=%u configured=%u qualification_active=%u qualified=%u producer_1017=%u consumer_1016_01=0x%08lX state=0x%02X age_ms=%lu snap=%u req=%08lX\r\n",
            (unsigned)((node_id == LEFT_NODE_ID) ? 4U : 7U),
            (unsigned)node_id,
            (unsigned)diag.heartbeat_configured[node_id],
            (unsigned)diag.heartbeat_qualification_active[node_id],
            (unsigned)diag.heartbeat_qualified[node_id],
            DRIVE_KINCO_HEARTBEAT_PRODUCER_MS,
            (unsigned long)DRIVE_KINCO_HEARTBEAT_CONSUMER_VALUE,
            state,
            (unsigned long)age_ms,
            snap, (unsigned long)request_nonce);
        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_DEBUG,
            "DRIVE_HEARTBEAT %u/9 node=%u rx_count=%lu sequence=%lu baseline_sequence=%lu qualification_count=%u snap=%u req=%08lX\r\n",
            (unsigned)((node_id == LEFT_NODE_ID) ? 5U : 8U),
            (unsigned)node_id,
            (unsigned long)diag.heartbeat_receive_count[node_id],
            (unsigned long)diag.heartbeat_sequence[node_id],
            (unsigned long)diag.heartbeat_baseline_sequence[node_id],
            (unsigned)diag.heartbeat_qualification_count[node_id],
            snap, (unsigned long)request_nonce);
        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_DEBUG,
            "DRIVE_HEARTBEAT %u/9 node=%u stale_count=%lu bootup_count=%lu invalid_count=%lu fault_reason=%u snap=%u req=%08lX\r\n",
            (unsigned)((node_id == LEFT_NODE_ID) ? 6U : 9U),
            (unsigned)node_id,
            (unsigned long)diag.heartbeat_stale_count[node_id],
            (unsigned long)diag.heartbeat_bootup_count[node_id],
            (unsigned long)diag.heartbeat_invalid_count[node_id],
            (unsigned)diag.heartbeat_fault_reason[node_id],
            snap, (unsigned long)request_nonce);
    }
}

void Drive_PrintFeedback(void)
{
    uint16_t statusword = Read_And_Print_Statusword(LEFT_NODE_ID);

    if (s_last_sdo_response.valid) {
        Drive_RecordStatusword(LEFT_NODE_ID, statusword,
                               HAL_GetTick(), 0U);
    } else {
        s_drive.statusword.left_statusword_valid = false;
        s_drive.statusword.left_statusword_generation = 0U;
    }
    Read_And_Print_ActualSpeed(LEFT_NODE_ID);
    Read_And_Print_ActualPosition(LEFT_NODE_ID);

    statusword = Read_And_Print_Statusword(RIGHT_NODE_ID);
    if (s_last_sdo_response.valid) {
        Drive_RecordStatusword(RIGHT_NODE_ID, statusword,
                               HAL_GetTick(), 0U);
    } else {
        s_drive.statusword.right_statusword_valid = false;
        s_drive.statusword.right_statusword_generation = 0U;
    }
    Read_And_Print_ActualSpeed(RIGHT_NODE_ID);
    Read_And_Print_ActualPosition(RIGHT_NODE_ID);

    if (g_odom.initialized) {
        uart_printf("ODOM current x=%ld mm y=%ld mm theta=%ld deg totalL=%ld mm totalR=%ld mm\r\n",
                    (long)g_odom.x_mm,
                    (long)g_odom.y_mm,
                    (long)(g_odom.theta_rad * 180.0f / ODOM_PI),
                    (long)g_odom.total_left_mm,
                    (long)g_odom.total_right_mm);
    }
}

void Drive_PrintBaseSafeDebug(uint32_t request_nonce)
{
    uint32_t now_ms = HAL_GetTick();
    CanopenRuntimeDiagnostics_t diag = Canopen_GetRuntimeDiagnostics();
    bool left_fresh = Drive_StatuswordEvidenceIsFresh(
        s_drive.statusword.left_statusword_valid,
        s_drive.statusword.left_statusword_timestamp_ms,
        s_drive.statusword.left_statusword_generation,
        now_ms);
    bool right_fresh = Drive_StatuswordEvidenceIsFresh(
        s_drive.statusword.right_statusword_valid,
        s_drive.statusword.right_statusword_timestamp_ms,
        s_drive.statusword.right_statusword_generation,
        now_ms);
    bool zero_current = Drive_ZeroSpeedEvidenceIsCurrent(now_ms);
    bool safe = Drive_BaseHardwareSafeForLift(now_ms);
    unsigned snap;

    /* All three frames of one report carry the same snap= and echo the
     * host req=, so a parser can never stitch frames from two different
     * snapshots (or a stale queued report) into one piece of evidence. */
    s_drive.diagnostic.base_safe_snapshot_sequence++;
    snap = (unsigned)s_drive.diagnostic.base_safe_snapshot_sequence;
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_DEBUG,
        "DRIVE_BASE_SAFE safe=%u state=%s active=%u en=%u dis=%u drive=%u target_zero=%u pending=%02X zero=%u snap=%u req=%08lX\r\n",
        safe ? 1U : 0U,
        Drive_StateName(s_drive.core.state),
        (s_drive.safety.transition.active || s_drive.fault_reset.transaction.active) ? 1U : 0U,
        s_hardware_enable_confirmed ? 1U : 0U,
        s_hardware_disable_confirmed ? 1U : 0U,
        s_drive.core.enabled ? 1U : 0U,
        Drive_LastCommandTargetIsZero() ? 1U : 0U,
        diag.target_pending_mask,
        zero_current ? 1U : 0U,
        snap, (unsigned long)request_nonce);
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_DEBUG,
        "DRIVE_BASE_SAFE_SW valid=%u/%u fresh=%u/%u sw=%04X/%04X age=%lu/%lu seq=%lu/%lu gen=%lu/%lu snap=%u req=%08lX\r\n",
        s_drive.statusword.left_statusword_valid ? 1U : 0U,
        s_drive.statusword.right_statusword_valid ? 1U : 0U,
        left_fresh ? 1U : 0U,
        right_fresh ? 1U : 0U,
        (unsigned)s_drive.statusword.left_statusword,
        (unsigned)s_drive.statusword.right_statusword,
        (unsigned long)Drive_DiagnosticAge(
            now_ms, s_drive.statusword.left_statusword_valid,
            s_drive.statusword.left_statusword_timestamp_ms),
        (unsigned long)Drive_DiagnosticAge(
            now_ms, s_drive.statusword.right_statusword_valid,
            s_drive.statusword.right_statusword_timestamp_ms),
        (unsigned long)s_drive.statusword.left_statusword_sequence,
        (unsigned long)s_drive.statusword.right_statusword_sequence,
        (unsigned long)s_drive.statusword.left_statusword_generation,
        (unsigned long)s_drive.statusword.right_statusword_generation,
        snap, (unsigned long)request_nonce);
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_DEBUG,
        "DRIVE_STOP_CONFIRM 1/2 state=%s reject=%s vel=%ld/%ld seq=%lu/%lu pair_age=%lu skew=%lu snap=%u req=%08lX\r\n",
        Drive_StableZeroStateName(),
        Drive_StableZeroRejectName(),
        (long)Motion_RoundFloatToI32(
            s_drive.stable_zero.left_velocity_mm_s),
        (long)Motion_RoundFloatToI32(
            s_drive.stable_zero.right_velocity_mm_s),
        (unsigned long)s_drive.stable_zero.last_left_sequence,
        (unsigned long)s_drive.stable_zero.last_right_sequence,
        (unsigned long)s_drive.stable_zero.pair_age_ms,
        (unsigned long)s_drive.stable_zero.pair_skew_ms,
        snap, (unsigned long)request_nonce);
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_DEBUG,
        "DRIVE_STOP_CONFIRM 2/2 stable=%lu dwell=%lu settle=%lu overall=%lu snap=%u req=%08lX\r\n",
        (unsigned long)s_drive.stable_zero.stable_sample_count,
        (unsigned long)s_drive.stable_zero.stable_dwell_ms,
        (unsigned long)s_drive.stable_zero.settle_elapsed_ms,
        (unsigned long)s_drive.stable_zero.overall_elapsed_ms,
        snap, (unsigned long)request_nonce);
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_DEBUG,
        "DRIVE_BASE_SAFE_ZERO valid=%u age=%lu gen=%lu/%lu snap=%u req=%08lX\r\n",
        zero_current ? 1U : 0U,
        (unsigned long)Drive_ZeroEvidenceAgeMs(now_ms),
        (unsigned long)s_drive.evidence.zero_speed.generation,
        (unsigned long)s_drive.evidence.safety_generation,
        snap, (unsigned long)request_nonce);
}

static const char *Drive_LiftStatuswordRefreshPhaseName(void)
{
    if ((s_drive.lift_stop.request.phase ==
         DRIVE_LIFT_STOP_PHASE_ZERO_SETTLE) ||
        (s_drive.lift_stop.request.phase ==
         DRIVE_LIFT_STOP_PHASE_ZERO_CONFIRM)) {
        return Drive_StableZeroStateName();
    }
    return (s_drive.lift_stop.request.phase ==
            DRIVE_LIFT_STOP_PHASE_STATUS_LEFT) ?
        "LEFT_STATUS" : "RIGHT_STATUS";
}

void Drive_ReportBaseSafeRefreshPending(void)
{
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_STATUS,
        "DRIVE_BASE_SAFE_REFRESH state=PENDING phase=%s txn=%lu req=%08lX\r\n",
        Drive_LiftStatuswordRefreshPhaseName(),
        (unsigned long)s_drive.lift_stop.request.txn,
        (unsigned long)s_drive.lift_stop.request.request_nonce);
}

#if ENABLE_JETSON_UART_MODE
static int32_t Drive_RpmToMilliRpm(float rpm)
{
    float milli_rpm = rpm * 1000.0f;

    if (isnan(milli_rpm)) {
        return 0;
    }
    if (milli_rpm >= (float)INT32_MAX) {
        return INT32_MAX;
    }
    if (milli_rpm <= (float)INT32_MIN) {
        return INT32_MIN;
    }
    return Motion_RoundFloatToI32(milli_rpm);
}

void Drive_SendJetsonStatus_100ms(void)
{
    uint32_t now = HAL_GetTick();
    DriveWheelFeedbackPair_t authoritative_pair;
    CanopenWheelFeedbackAuthority_t authority =
        Canopen_GetWheelFeedbackAuthority();

    if ((now - s_drive.timing.status_last_ms) < JETSON_STATUS_PERIOD_MS) {
        return;
    }
    s_drive.timing.status_last_ms = now;

    (void)Drive_GetAuthoritativeWheelFeedbackPair(
        now, &authoritative_pair);
    if (authority == CANOPEN_WHEEL_AUTHORITY_TPDO) {
        CanopenWheelTpdoSample_t left_tpdo = {0};
        CanopenWheelTpdoSample_t right_tpdo = {0};

        (void)Canopen_GetWheelTpdoShadowSample(
            LEFT_NODE_ID, now, &left_tpdo);
        (void)Canopen_GetWheelTpdoShadowSample(
            RIGHT_NODE_ID, now, &right_tpdo);
        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_STATUS,
            "WHEEL_FEEDBACK_HEALTH v=1 mode=%s authority=TPDO pair_valid=%u pair_state=%s pair_seq=%lu pair_age_ms=%lu pair_skew_ms=%lu sdo606c=%u\r\n",
            Canopen_WheelFeedbackTransportName(
                Canopen_GetWheelFeedbackTransport()),
            authoritative_pair.pair_valid ? 1U : 0U,
            (authoritative_pair.pair_state == DRIVE_WHEEL_PAIR_FRESH) ?
                "FRESH" :
                ((authoritative_pair.pair_state == DRIVE_WHEEL_PAIR_FAULT) ?
                 "FAULT" :
                 ((authoritative_pair.pair_state == DRIVE_WHEEL_PAIR_STALE) ?
                  "STALE" : "NO_DATA")),
            (unsigned long)authoritative_pair.pair_sequence,
            (unsigned long)authoritative_pair.pair_age_ms,
            (unsigned long)authoritative_pair.pair_skew_ms,
            Canopen_RuntimeSdo606cPollEnabled() ? 1U : 0U);
        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_STATUS,
            "WHEEL_FEEDBACK_HEALTH_NODE v=1 node1_seq=%lu node1_age_ms=%lu node2_seq=%lu node2_age_ms=%lu\r\n",
            (unsigned long)left_tpdo.sequence,
            (unsigned long)Drive_TpdoDiagnosticAge(
                now, left_tpdo.seen, left_tpdo.timestamp_ms),
            (unsigned long)right_tpdo.sequence,
            (unsigned long)Drive_TpdoDiagnosticAge(
                now, right_tpdo.seen, right_tpdo.timestamp_ms));
        if (!authoritative_pair.pair_valid) {
            if (s_drive.tpdo_active.odom_stale_suppressed_count != UINT32_MAX) {
                s_drive.tpdo_active.odom_stale_suppressed_count++;
            }
            return;
        }
    }

    uint32_t fault = Drive_OdomFaultStatus();
    const char *drive_state = Drive_StateName(s_drive.core.state);
    int32_t theta_mrad = (int32_t)(g_odom.theta_rad * 1000.0f);
    float left_authoritative_rpm = authoritative_pair.pair_valid ?
        Motion_RawVelocityDecToLogicalMotorRPM(
            LEFT_NODE_ID, authoritative_pair.left_raw_velocity) : 0.0f;
    float right_authoritative_rpm = authoritative_pair.pair_valid ?
        Motion_RawVelocityDecToLogicalMotorRPM(
            RIGHT_NODE_ID, authoritative_pair.right_raw_velocity) : 0.0f;

    if (authoritative_pair.pair_valid) {
        int32_t left_mrpm = Drive_RpmToMilliRpm(left_authoritative_rpm);
        int32_t right_mrpm = Drive_RpmToMilliRpm(right_authoritative_rpm);

        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_STATUS,
            "ODOM %ld %ld %ld %ld %ld %s %lu\r\n",
            (long)g_odom.x_mm,
            (long)g_odom.y_mm,
            (long)theta_mrad,
            (long)left_mrpm,
            (long)right_mrpm,
            drive_state,
            (unsigned long)fault);
    }
}
#endif

#if ENABLE_JETSON_UART_MODE
void Drive_PrintYawPiStatus(void)
{
    JetsonTx_Printf(JETSON_TX_PRIORITY_STATUS,
                "YAW_PI enabled=%u default=%u status=OK mode=%s active=%u primed=%u feedback_sequence=%lu\r\n",
                s_yaw_pi_enabled ? 1U : 0U,
                (DRIVE_YAW_RATE_PI_DEFAULT_ENABLED != 0U) ? 1U : 0U,
                Motion_YawControlModeName(s_drive.control.yaw.pi.mode),
                s_drive.control.yaw.pi.active ? 1U : 0U,
                s_drive.control.yaw.pi.feedback_sequence_valid ? 1U : 0U,
                (unsigned long)s_drive.feedback.chassis.sequence);
    JetsonTx_Printf(JETSON_TX_PRIORITY_STATUS,
                "YAW_PI tracking feedback_sequence_valid=%u last_consumed_feedback_sequence=%lu\r\n",
                s_drive.control.tracking.supervisor.feedback_sequence_valid ? 1U : 0U,
                (unsigned long)s_drive.control.tracking.supervisor.last_feedback_sequence);
}
#endif

#if ENABLE_JETSON_UART_MODE
void Drive_PrintYawRateDiagnostics(uint32_t request_nonce)
{
    CanopenRuntimeDiagnostics_t can_diag = Canopen_GetRuntimeDiagnostics();
    uint32_t now = HAL_GetTick();
    /* All fifteen lines of one response share snap= and echo req= so a
     * host can never stitch lines from different snapshots together. */
    unsigned snap =
        (unsigned)(++s_drive.diagnostic.yaw_snapshot_sequence);
    uint32_t pair_elapsed_ms = (uint32_t)(
        s_drive.feedback.motion.last_pair_timestamp_ms -
        s_drive.feedback.motion.first_pair_timestamp_ms);
    uint32_t pi_elapsed_ms = (uint32_t)(
        s_drive.control.yaw.pi.last_accepted_feedback_timestamp_ms -
        s_drive.feedback.motion.first_pair_timestamp_ms);
    uint32_t pair_rate_x10 = 0U;
    uint32_t pi_rate_x10 = 0U;
    uint32_t submit_rate_x10 = 0U;
    uint32_t pi_dt_min_ms = (s_drive.control.yaw.pi.update_count != 0U) ?
        s_drive.control.yaw.pi.dt_min_ms : 0U;
    uint32_t left_age_ms = s_drive.feedback.status_left_seen ?
        (uint32_t)(now - s_drive.feedback.status_left_timestamp_ms) : UINT32_MAX;
    uint32_t right_age_ms = s_drive.feedback.status_right_seen ?
        (uint32_t)(now - s_drive.feedback.status_right_timestamp_ms) : UINT32_MAX;
    bool left_feedback_stale = !s_drive.feedback.status_left_valid ||
        (left_age_ms > DRIVE_WHEEL_FEEDBACK_STALE_MS);
    bool right_feedback_stale = !s_drive.feedback.status_right_valid ||
        (right_age_ms > DRIVE_WHEEL_FEEDBACK_STALE_MS);
    char left_feedback_state = !s_drive.feedback.status_left_seen ? 'N' :
        (!s_drive.feedback.status_left_valid ? 'I' :
         (left_feedback_stale ? 'S' : 'F'));
    char right_feedback_state = !s_drive.feedback.status_right_seen ? 'N' :
        (!s_drive.feedback.status_right_valid ? 'I' :
         (right_feedback_stale ? 'S' : 'F'));
    /* Units: P_term and I_term are mrad/s, I_state is mrad, and all
     * raw/requested/proposed/submitted corrections are mrad/s. */
    float i_term_mrad_s = s_yaw_rate_pi_config.ki *
                          s_drive.control.yaw.pi.integral_mrad;

    if ((s_drive.feedback.motion.chassis_sequence > 1U) &&
        (pair_elapsed_ms != 0U)) {
        pair_rate_x10 = (uint32_t)(
            ((uint64_t)(s_drive.feedback.motion.chassis_sequence - 1U) * 10000ULL) /
            pair_elapsed_ms);
    }
    if ((s_drive.control.yaw.pi.update_count != 0U) && (pi_elapsed_ms != 0U)) {
        pi_rate_x10 = (uint32_t)(
            ((uint64_t)s_drive.control.yaw.prepare_count * 10000ULL) /
            pi_elapsed_ms);
        submit_rate_x10 = (uint32_t)(
            ((uint64_t)s_drive.control.actuator.successful_submit_count * 10000ULL) /
            pi_elapsed_ms);
    }

    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                "YAW_CTRL 1/15 req=%ld/%ld body=%ld/%ld nominal=%ld/%ld env=%u lim=%u/%u snap=%u req=%08lX\r\n",
                (long)s_drive.control.requested_linear_mm_s,
                (long)s_drive.control.requested_angular_mrad_s,
                (long)s_drive.control.target_linear_mm_s,
                (long)s_drive.control.target_angular_mrad_s,
                (long)s_drive.control.nominal_feasible_linear_mm_s,
                (long)s_drive.control.nominal_feasible_angular_mrad_s,
                s_drive.control.actuator.body_command.nominal_envelope_used ? 1U : 0U,
                s_drive.control.actuator.body_command.linear_limited ? 1U : 0U,
                s_drive.control.actuator.body_command.angular_limited ? 1U : 0U,
                snap, (unsigned long)request_nonce);
    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                "YAW_CTRL 2/15 env hard_rpm/wheel=%ld/%ld nominal_wheel=%ld reserve=%ld yaw_auth_pos/neg=%ld/%ld snap=%u req=%08lX\r\n",
                (long)s_drive.control.actuator.command_envelope.hard_motor_rpm,
                (long)s_drive.control.actuator.command_envelope.hard_wheel_mm_s,
                (long)s_drive.control.actuator.command_envelope.nominal_wheel_mm_s,
                (long)s_drive.control.actuator.command_envelope.reserved_yaw_correction_mrad_s,
                (long)s_drive.control.actuator.baseline_targets.remaining_positive_yaw_authority_mrad_s,
                (long)s_drive.control.actuator.baseline_targets.remaining_negative_yaw_authority_mrad_s,
                snap, (unsigned long)request_nonce);
    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                "YAW_CTRL 3/15 yaw_mode=%s pi_enabled=%u corrected=%ld committed=%ld measured=%ld error=%ld snap=%u req=%08lX\r\n",
                Motion_YawControlModeName(s_drive.control.yaw.pi.mode),
                s_yaw_pi_enabled ? 1U : 0U,
                (long)s_drive.control.yaw.corrected_angular_mrad_s,
                (long)(s_drive.control.actuator.state.valid ?
                    s_drive.control.actuator.state.feasible_angular_mrad_s : 0.0f),
                (long)s_drive.feedback.chassis.angular_velocity_mrad_s,
                (long)s_drive.control.yaw.pi.error_mrad_s,
                snap, (unsigned long)request_nonce);
    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                "YAW_CTRL 4/15 P_term=%ld I_state=%ld I_term=%ld raw/req/proposed/submitted=%ld/%ld/%ld/%ld snap=%u req=%08lX\r\n",
                (long)s_drive.control.yaw.pi.p_term_mrad_s,
                (long)s_drive.control.yaw.pi.integral_mrad,
                (long)i_term_mrad_s,
                (long)s_drive.control.yaw.pi.raw_correction_mrad_s,
                (long)s_drive.control.yaw.pi.requested_correction_mrad_s,
                (long)s_drive.control.yaw.proposed_submitted_correction_mrad_s,
                (long)s_drive.control.yaw.submitted_correction_mrad_s,
                snap, (unsigned long)request_nonce);
    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                "YAW_CTRL 5/15 rate10 p/pi/s=%lu/%lu/%lu cnt p/pi/s/r/b=%lu/%lu/%lu/%lu/%lu snap=%u req=%08lX\r\n",
                (unsigned long)pair_rate_x10,
                (unsigned long)pi_rate_x10,
                (unsigned long)submit_rate_x10,
                (unsigned long)s_drive.feedback.motion.chassis_sequence,
                (unsigned long)s_drive.control.yaw.prepare_count,
                (unsigned long)s_drive.control.actuator.successful_submit_count,
                (unsigned long)s_drive.control.yaw.feedback_pair_replaced_count,
                (unsigned long)s_drive.control.actuator.state.busy_reject_count,
                snap, (unsigned long)request_nonce);
    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                "YAW_CTRL 6/15 dt last/min/max/avg=%ld/%lu/%lu/%ld short/long/reset=%lu/%lu/%lu snap=%u req=%08lX\r\n",
                (long)(s_drive.control.yaw.pi.dt_s * 1000.0f),
                (unsigned long)pi_dt_min_ms,
                (unsigned long)s_drive.control.yaw.pi.dt_max_ms,
                (long)s_drive.control.yaw.pi.dt_average_ms,
                (unsigned long)s_drive.control.yaw.pi.short_dt_reject_count,
                (unsigned long)s_drive.control.yaw.pi.long_gap_reset_count,
                (unsigned long)s_drive.control.yaw.pi.reset_count,
                snap, (unsigned long)request_nonce);
    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                "YAW_CTRL 7/15 hard pre_rpm=%ld/%ld feasible=%ld/%ld scale=%ld sat=%u snap=%u req=%08lX\r\n",
                (long)s_drive.control.actuator.final_targets.prelimit_left_motor_rpm,
                (long)s_drive.control.actuator.final_targets.prelimit_right_motor_rpm,
                (long)s_drive.control.actuator.final_targets.left_motor_rpm,
                (long)s_drive.control.actuator.final_targets.right_motor_rpm,
                (long)(s_drive.control.actuator.final_targets.saturation_scale * 1000.0f),
                s_drive.control.actuator.final_targets.saturated ? 1U : 0U,
                snap, (unsigned long)request_nonce);
    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                "YAW_CTRL 8/15 final rpm=%ld/%ld dec=%ld/%ld joint/dt/rate/hard=%ld/%lu/%u/%u snap=%u req=%08lX\r\n",
                (long)s_drive.control.actuator.final_proposal.left_motor_rpm,
                (long)s_drive.control.actuator.final_proposal.right_motor_rpm,
                (long)s_drive.control.actuator.final_proposal.left_velocity_dec,
                (long)s_drive.control.actuator.final_proposal.right_velocity_dec,
                (long)(s_drive.control.actuator.final_proposal.joint_scale * 1000.0f),
                (unsigned long)s_drive.control.actuator.final_proposal.dt_ms,
                s_drive.control.actuator.final_proposal.rate_limited ? 1U : 0U,
                s_drive.control.actuator.final_proposal.hard_limited ? 1U : 0U,
                snap, (unsigned long)request_nonce);
    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                "YAW_CTRL 9/15 submitted valid=%u rpm=%ld/%ld body=%ld/%ld ok/busy=%lu/%lu tx_busy=%u snap=%u req=%08lX\r\n",
                s_drive.control.actuator.state.valid ? 1U : 0U,
                (long)s_drive.control.actuator.state.left_motor_rpm,
                (long)s_drive.control.actuator.state.right_motor_rpm,
                (long)s_drive.control.actuator.state.feasible_linear_mm_s,
                (long)s_drive.control.actuator.state.feasible_angular_mrad_s,
                (unsigned long)s_drive.control.actuator.state.submit_count,
                (unsigned long)s_drive.control.actuator.state.busy_reject_count,
                s_drive.control.tracking.limits.transport_busy ? 1U : 0U,
                snap, (unsigned long)request_nonce);
    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                "YAW_CTRL 10/15 fb=%u/%u seq=%lu age=%lu/%lu skew=%lu sup=%lu/%lu/%lu snap=%u req=%08lX\r\n",
                s_drive.feedback.chassis.valid ? 1U : 0U,
                s_drive.feedback.chassis.stale ? 1U : 0U,
                (unsigned long)s_drive.feedback.chassis.sequence,
                (unsigned long)left_age_ms,
                (unsigned long)right_age_ms,
                (unsigned long)s_drive.feedback.chassis.pair_skew_ms,
                (unsigned long)s_drive.control.tracking.supervisor.invalid_feedback_observation_count,
                (unsigned long)s_drive.control.tracking.supervisor.stale_feedback_observation_count,
                (unsigned long)s_drive.control.tracking.supervisor.skew_feedback_observation_count,
                snap, (unsigned long)request_nonce);
    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                "YAW_CTRL 11/15 feedback_sequence_valid=%u last_consumed_feedback_sequence=%lu reject stale/skew=%lu/%lu snap=%u req=%08lX\r\n",
                s_drive.control.tracking.supervisor.feedback_sequence_valid ? 1U : 0U,
                (unsigned long)s_drive.control.tracking.supervisor.last_feedback_sequence,
                (unsigned long)s_drive.feedback.motion.rejected_stale_pair_count,
                (unsigned long)s_drive.feedback.motion.rejected_skew_pair_count,
                snap, (unsigned long)request_nonce);
    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                "YAW_CTRL 12/15 track=%s reason=%s last=%s latch=%u cnt=%u/%u/%u snap=%u req=%08lX\r\n",
                Motion_TrackingStateName(s_drive.control.tracking.supervisor.state),
                Motion_TrackingReasonName(s_drive.control.tracking.supervisor.reason),
                Motion_TrackingReasonName(s_drive.control.tracking.supervisor.last_stop_reason),
                s_tracking_stop_latched ? 1U : 0U,
                s_drive.control.tracking.supervisor.large_error_consecutive,
                s_drive.control.tracking.supervisor.wrong_sign_consecutive,
                s_drive.control.tracking.supervisor.divergence_consecutive,
                snap, (unsigned long)request_nonce);
    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                "YAW_CTRL 13/15 obs err/corr/head/hard/rate/busy/sign/div=%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu snap=%u req=%08lX\r\n",
                (unsigned long)s_drive.control.tracking.supervisor.large_error_observation_count,
                (unsigned long)s_drive.control.tracking.supervisor.correction_limit_observation_count,
                (unsigned long)s_drive.control.tracking.supervisor.nominal_headroom_observation_count,
                (unsigned long)s_drive.control.tracking.supervisor.hard_limit_observation_count,
                (unsigned long)s_drive.control.tracking.supervisor.rate_limit_observation_count,
                (unsigned long)s_drive.control.tracking.supervisor.transport_busy_observation_count,
                (unsigned long)s_drive.control.tracking.supervisor.wrong_sign_observation_count,
                (unsigned long)s_drive.control.tracking.supervisor.divergence_observation_count,
                snap, (unsigned long)request_nonce);
    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                "YAW_CTRL 14/15 tr=%s fb=%lu/%lu tx=%lu/%lu/%lu p=%02X/%02X own=%02X/%02X pd=%lu/%lu snap=%u req=%08lX\r\n",
                Canopen_WheelFeedbackTransportName(
                    Canopen_GetWheelFeedbackTransport()),
                (unsigned long)can_diag.wheel_feedback_success_count,
                (unsigned long)can_diag.wheel_feedback_failure_count,
                (unsigned long)can_diag.wheel_target_submit_count,
                (unsigned long)can_diag.wheel_target_complete_count,
                (unsigned long)can_diag.wheel_target_failure_count,
                can_diag.feedback_pending_mask,
                can_diag.target_pending_mask,
                can_diag.sdo_owner[LEFT_NODE_ID],
                can_diag.sdo_owner[RIGHT_NODE_ID],
                (unsigned long)can_diag.safety_request_preemption_count,
                (unsigned long)can_diag.fault_monitor_deferred_count,
                snap, (unsigned long)request_nonce);
    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                "YAW_CTRL 15/15 sdo=%lu/%lu/%lu/%lu so=%02X%02X/%02X%02X fm=%c a=%lu/%lu v=%c%c r/t=%lu/%lu snap=%u req=%08lX\r\n",
                (unsigned long)can_diag.sdo_timeout_count,
                (unsigned long)can_diag.sdo_quarantine_count,
                (unsigned long)can_diag.sdo_late_response_count,
                (unsigned long)can_diag.sdo_unexpected_response_count,
                can_diag.sdo_state[LEFT_NODE_ID],
                can_diag.sdo_operation[LEFT_NODE_ID],
                can_diag.sdo_state[RIGHT_NODE_ID],
                can_diag.sdo_operation[RIGHT_NODE_ID],
                (s_drive.core.state == DRIVE_STATE_FAULT) ? '1' : '0',
                (unsigned long)left_age_ms,
                (unsigned long)right_age_ms,
                left_feedback_state,
                right_feedback_state,
                (unsigned long)can_diag.fault_velocity_request_count,
                (unsigned long)can_diag.fault_velocity_timeout_count,
                snap, (unsigned long)request_nonce);
}
#endif

static const char *Drive_TpdoConfigSdoResultName(
    CanopenWheelTpdoConfigSdoResult_t result)
{
    switch (result) {
        case CANOPEN_TPDO_CONFIG_SDO_NOT_RUN: return "NOT_RUN";
        case CANOPEN_TPDO_CONFIG_SDO_SUCCESS: return "SUCCESS";
        case CANOPEN_TPDO_CONFIG_SDO_TIMEOUT: return "TIMEOUT";
        case CANOPEN_TPDO_CONFIG_SDO_ABORT: return "ABORT";
        case CANOPEN_TPDO_CONFIG_SDO_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

static const char *Drive_TpdoConfigResultName(
    CanopenWheelTpdoConfigResult_t result)
{
    switch (result) {
        case CANOPEN_TPDO_CONFIG_RESULT_SUCCESS: return "SUCCESS";
        case CANOPEN_TPDO_CONFIG_RESULT_INVALID_NODE: return "INVALID_NODE";
        case CANOPEN_TPDO_CONFIG_RESULT_SDO_FAILURE: return "SDO_FAILURE";
        case CANOPEN_TPDO_CONFIG_RESULT_READBACK_MISMATCH:
            return "READBACK_MISMATCH";
        case CANOPEN_TPDO_CONFIG_RESULT_NMT_START_FAILED:
            return "NMT_START_FAILED";
        case CANOPEN_TPDO_CONFIG_RESULT_NONE:
        default: return "NONE";
    }
}

static bool Drive_ConfigureWheelTpdoNodes(
    bool configure_left,
    bool configure_right,
    CanopenWheelTpdoConfigDiagnostic_t *diagnostic)
{
    if ((configure_left &&
         !Canopen_ConfigureWheelTpdoFeedbackNode(LEFT_NODE_ID)) ||
        (configure_right &&
         !Canopen_ConfigureWheelTpdoFeedbackNode(RIGHT_NODE_ID))) {
        if (diagnostic != NULL) {
            *diagnostic = Canopen_GetWheelTpdoConfigDiagnostic();
        }
        return false;
    }
    return true;
}

bool Drive_CommandConfigureWheelTpdo(uint8_t node_id,
                                     uint32_t request_nonce)
{
    bool configure_left = (node_id == 0U) || (node_id == LEFT_NODE_ID);
    bool configure_right = (node_id == 0U) || (node_id == RIGHT_NODE_ID);
    const char *node_name = (node_id == 0U) ? "all" :
        ((node_id == LEFT_NODE_ID) ? "1" : "2");
    CanopenWheelTpdoConfigDiagnostic_t diagnostic;

    if (!configure_left && !configure_right) {
        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_STATUS,
            "DRIVE_TPDO_CONFIG state=FAILED reason=INVALID_NODE node=%u req=%08lX\r\n",
            (unsigned)node_id, (unsigned long)request_nonce);
        return false;
    }
    if ((Canopen_GetWheelFeedbackAuthority() ==
         CANOPEN_WHEEL_AUTHORITY_TPDO) ||
        s_drive.tpdo_active.enable_command_active) {
        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_STATUS,
            "DRIVE_TPDO_CONFIG state=FAILED reason=ACTIVE_AUTHORITY node=%s req=%08lX\r\n",
            node_name, (unsigned long)request_nonce);
        return false;
    }
    if (s_drive.core.enabled || s_hardware_enable_confirmed) {
        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_STATUS,
            "DRIVE_TPDO_CONFIG state=FAILED reason=DRIVE_ENABLED node=%s req=%08lX\r\n",
            node_name, (unsigned long)request_nonce);
        return false;
    }
    if (!s_hardware_disable_confirmed) {
        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_STATUS,
            "DRIVE_TPDO_CONFIG state=FAILED reason=HARDWARE_DISABLE_UNCONFIRMED node=%s req=%08lX\r\n",
            node_name, (unsigned long)request_nonce);
        return false;
    }
    if (s_drive.safety.transition.active ||
        s_drive.lift_stop.request.active) {
        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_STATUS,
            "DRIVE_TPDO_CONFIG state=FAILED reason=TRANSITION_ACTIVE node=%s req=%08lX\r\n",
            node_name, (unsigned long)request_nonce);
        return false;
    }
    if (s_drive.fault_reset.transaction.active) {
        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_STATUS,
            "DRIVE_TPDO_CONFIG state=FAILED reason=FAULT_RESET_ACTIVE node=%s req=%08lX\r\n",
            node_name, (unsigned long)request_nonce);
        return false;
    }
    if (Drive_LiftIsMoving()) {
        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_STATUS,
            "DRIVE_TPDO_CONFIG state=FAILED reason=LIFT_MOVING node=%s req=%08lX\r\n",
            node_name, (unsigned long)request_nonce);
        return false;
    }
    if (!Drive_LastCommandTargetIsZero()) {
        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_STATUS,
            "DRIVE_TPDO_CONFIG state=FAILED reason=TARGET_NONZERO node=%s req=%08lX\r\n",
            node_name, (unsigned long)request_nonce);
        return false;
    }

    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_STATUS,
        "DRIVE_TPDO_CONFIG state=STARTED node=%s req=%08lX\r\n",
        node_name, (unsigned long)request_nonce);
    if (!Drive_ConfigureWheelTpdoNodes(
            configure_left, configure_right, &diagnostic)) {
        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_STATUS,
            "DRIVE_TPDO_CONFIG state=FAILED reason=%s node=%u step=%u index=%04X sub=%02X sdo=%s abort=%08lX expected=%08lX actual=%08lX req=%08lX\r\n",
            Drive_TpdoConfigResultName(diagnostic.result),
            (unsigned)diagnostic.node_id,
            (unsigned)diagnostic.step,
            (unsigned)diagnostic.index,
            (unsigned)diagnostic.subindex,
            Drive_TpdoConfigSdoResultName(diagnostic.sdo_result),
            (unsigned long)diagnostic.abort_code,
            (unsigned long)diagnostic.expected_value,
            (unsigned long)diagnostic.actual_value,
            (unsigned long)request_nonce);
        return false;
    }

    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_STATUS,
        "DRIVE_TPDO_CONFIG state=COMPLETE node=%s configured=%u/%u mode=%s req=%08lX\r\n",
        node_name,
        Canopen_WheelTpdoConfigured(LEFT_NODE_ID) ? 1U : 0U,
        Canopen_WheelTpdoConfigured(RIGHT_NODE_ID) ? 1U : 0U,
        Canopen_WheelFeedbackTransportName(
            Canopen_GetWheelFeedbackTransport()),
        (unsigned long)request_nonce);
    return true;
}

static uint32_t Drive_TpdoDiagnosticAge(uint32_t now_ms,
                                        bool seen,
                                        uint32_t timestamp_ms)
{
    return seen ? (uint32_t)(now_ms - timestamp_ms) : UINT32_MAX;
}

static void Drive_PrintWheelTpdoComparison(
    unsigned line_number,
    uint8_t node_id,
    uint32_t now_ms,
    const CanopenValueSample_t *sdo,
    const CanopenWheelTpdoSample_t *tpdo,
    uint32_t request_nonce)
{
    bool sdo_seen = sdo->sequence != 0U;
    int64_t difference = (sdo_seen && tpdo->seen) ?
        (int64_t)tpdo->raw - (int64_t)sdo->value : 0;

    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_STATUS,
        "DRIVE_TPDO_STATUS %u/5 n=%u sdo=%ld sdo_age=%lu tpdo=%ld tpdo_age=%lu diff=%lld req=%08lX\r\n",
        line_number, (unsigned)node_id, (long)sdo->value,
        (unsigned long)Drive_TpdoDiagnosticAge(
            now_ms, sdo_seen, sdo->timestamp_ms),
        (long)tpdo->raw,
        (unsigned long)Drive_TpdoDiagnosticAge(
            now_ms, tpdo->seen, tpdo->timestamp_ms),
        (long long)difference,
        (unsigned long)request_nonce);
}

void Drive_PrintWheelTpdoStatus(uint32_t request_nonce)
{
    uint32_t now_ms = HAL_GetTick();
    CanopenWheelTpdoSample_t left_tpdo = {0};
    CanopenWheelTpdoSample_t right_tpdo = {0};
    CanopenValueSample_t left_sdo = {0};
    CanopenValueSample_t right_sdo = {0};

    (void)Canopen_GetWheelTpdoShadowSample(
        LEFT_NODE_ID, now_ms, &left_tpdo);
    (void)Canopen_GetWheelTpdoShadowSample(
        RIGHT_NODE_ID, now_ms, &right_tpdo);
    (void)Canopen_GetWheelSdoVelocitySample(LEFT_NODE_ID, &left_sdo);
    (void)Canopen_GetWheelSdoVelocitySample(RIGHT_NODE_ID, &right_sdo);

    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_STATUS,
        "DRIVE_TPDO_STATUS 1/5 mode=%s configured=%u/%u cob=%03X/%03X mapping=%08lX/%08lX event_ms=%u type=%02X inhibit=%u req=%08lX\r\n",
        Canopen_WheelFeedbackTransportName(
            Canopen_GetWheelFeedbackTransport()),
        Canopen_WheelTpdoConfigured(LEFT_NODE_ID) ? 1U : 0U,
        Canopen_WheelTpdoConfigured(RIGHT_NODE_ID) ? 1U : 0U,
        DRIVE_WHEEL_TPDO_LEFT_COB_ID,
        DRIVE_WHEEL_TPDO_RIGHT_COB_ID,
        (unsigned long)DRIVE_WHEEL_TPDO_MAP_ENTRY,
        (unsigned long)DRIVE_WHEEL_TPDO_MAP_ENTRY,
        DRIVE_WHEEL_TPDO_EVENT_TIMER_MS,
        DRIVE_WHEEL_TPDO_TRANSMISSION_TYPE,
        DRIVE_WHEEL_TPDO_INHIBIT_TIME,
        (unsigned long)request_nonce);
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_STATUS,
        "DRIVE_TPDO_STATUS 2/5 n=1 st=%s raw=%ld a=%lu q=%lu f=%lu iu=%lu im=%lu g=%lu d=%lu t=%lu r=%lu req=%08lX\r\n",
        Canopen_WheelTpdoSampleStateName(left_tpdo.state),
        (long)left_tpdo.raw,
        (unsigned long)Drive_TpdoDiagnosticAge(
            now_ms, left_tpdo.seen, left_tpdo.timestamp_ms),
        (unsigned long)left_tpdo.sequence,
        (unsigned long)left_tpdo.frame_count,
        (unsigned long)left_tpdo.ignored_unconfigured_frame_count,
        (unsigned long)left_tpdo.ignored_transport_mode_frame_count,
        (unsigned long)left_tpdo.max_interarrival_gap_ms,
        (unsigned long)left_tpdo.invalid_dlc_count,
        (unsigned long)left_tpdo.stale_transition_count,
        (unsigned long)left_tpdo.recovery_transition_count,
        (unsigned long)request_nonce);
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_STATUS,
        "DRIVE_TPDO_STATUS 3/5 n=2 st=%s raw=%ld a=%lu q=%lu f=%lu iu=%lu im=%lu g=%lu d=%lu t=%lu r=%lu req=%08lX\r\n",
        Canopen_WheelTpdoSampleStateName(right_tpdo.state),
        (long)right_tpdo.raw,
        (unsigned long)Drive_TpdoDiagnosticAge(
            now_ms, right_tpdo.seen, right_tpdo.timestamp_ms),
        (unsigned long)right_tpdo.sequence,
        (unsigned long)right_tpdo.frame_count,
        (unsigned long)right_tpdo.ignored_unconfigured_frame_count,
        (unsigned long)right_tpdo.ignored_transport_mode_frame_count,
        (unsigned long)right_tpdo.max_interarrival_gap_ms,
        (unsigned long)right_tpdo.invalid_dlc_count,
        (unsigned long)right_tpdo.stale_transition_count,
        (unsigned long)right_tpdo.recovery_transition_count,
        (unsigned long)request_nonce);
    Drive_PrintWheelTpdoComparison(
        4U, LEFT_NODE_ID, now_ms, &left_sdo, &left_tpdo, request_nonce);
    Drive_PrintWheelTpdoComparison(
        5U, RIGHT_NODE_ID, now_ms, &right_sdo, &right_tpdo, request_nonce);
}

#ifdef AMR_HOST_TEST
static bool s_tpdo_test_estop_active;
#endif

static bool Drive_TpdoEstopActive(void)
{
    /* This hardware profile intentionally has no MCU E-stop control gate.
     * Physical PD15 is reported independently by estop_status.c; it does not
     * feed this TPDO gate or change Drive/Lift recovery policy.
     * The host-only override exists solely to prove the fail-closed gate
     * without changing the production hardware semantics. */
#ifdef AMR_HOST_TEST
    return s_tpdo_test_estop_active;
#else
    return false;
#endif
}

static uint32_t Drive_TpdoPairSkew(
    const CanopenWheelTpdoSample_t *left,
    const CanopenWheelTpdoSample_t *right)
{
    int32_t delta = (int32_t)(left->timestamp_ms - right->timestamp_ms);

    return (delta >= 0) ? (uint32_t)delta : (uint32_t)(-(int64_t)delta);
}

/* Configuration and feedback arming need the same current disabled-safety
 * boundary. In BASIC, tracking/inhibit are retained history after reset and
 * verified DISABLE; only explicit successful ENABLE retires them. Requiring
 * those flags clear here would make the pre-ENABLE PDO qualification cyclic.
 * Do not clear latches, reset faults or enable hardware in this predicate. */
static const char *Drive_TpdoDisabledGate(void)
{
    if (s_drive.core.state == DRIVE_STATE_FAULT) {
        return "DRIVE_FAULT_ACTIVE";
    }
    if (s_drive.core.enabled) {
        return "DRIVE_ENABLED";
    }
    if (!s_hardware_disable_confirmed) {
        return "HARDWARE_DISABLE_UNCONFIRMED";
    }
    if (s_hardware_enable_confirmed) {
        return "HARDWARE_ENABLE_CONFIRMED";
    }
    if (s_drive.safety.transition.active ||
        s_drive.lift_stop.request.active ||
        s_drive.fault_reset.transaction.active) {
        return "DRIVE_TRANSITION_ACTIVE";
    }
    if (!Drive_LastCommandTargetIsZero()) {
        return "TARGET_NONZERO";
    }
    if (Drive_LiftIsMoving()) {
        return "LIFT_NOT_STOPPED";
    }
    if (Drive_TpdoEstopActive()) {
        return "ESTOP_ACTIVE";
    }
    if (s_drive.core.state != DRIVE_STATE_DISABLED) {
        return "DRIVE_NOT_DISABLED";
    }
    /* HeartbeatFaultPending also includes history retired only by explicit
     * ENABLE. The main loop consumes new heartbeat events before this gate;
     * PDO commissioning still requires a new NMT response and fresh pairs. */
    if (Canopen_WheelRuntimeFaultPending()) {
        return "CURRENT_CAN_FAULT";
    }
    if (s_drive.statusword.left_statusword_valid &&
        (Statusword_HasFault(s_drive.statusword.left_statusword) ||
         Statusword_IsOperationEnabled(s_drive.statusword.left_statusword))) {
        return "NODE1_STATUS_UNSAFE";
    }
    if (s_drive.statusword.right_statusword_valid &&
        (Statusword_HasFault(s_drive.statusword.right_statusword) ||
         Statusword_IsOperationEnabled(s_drive.statusword.right_statusword))) {
        return "NODE2_STATUS_UNSAFE";
    }
    FDCAN_ProtocolStatusTypeDef bus = {0};
    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &bus) != HAL_OK || bus.BusOff) {
        return "BUS_OFF_OR_UNREADABLE";
    }
    if (Canopen_GetWheelFeedbackTransport() == CANOPEN_WHEEL_FEEDBACK_TPDO_FAULT) {
        return "TPDO_FAULT_LATCHED";
    }
#if !DRIVE_BASIC_BRINGUP_PROFILE
    if (s_tracking_stop_latched || s_disable_motion_inhibit_latched) {
        return "TRACKING_OR_INHIBIT";
    }
#endif
    return NULL;
}

static const char *Drive_TpdoActiveEnableGate(uint32_t now_ms)
{
    CanopenWheelTpdoSample_t left = {0};
    CanopenWheelTpdoSample_t right = {0};
    const char *reason = Drive_TpdoDisabledGate();

    if (reason != NULL) { return reason; }
    if (!Canopen_WheelTpdoConfigured(LEFT_NODE_ID)) {
        return "NODE1_NOT_CONFIGURED";
    }
    if (!Canopen_WheelTpdoConfigured(RIGHT_NODE_ID)) {
        return "NODE2_NOT_CONFIGURED";
    }
    (void)Canopen_GetWheelTpdoShadowSample(LEFT_NODE_ID, now_ms, &left);
    (void)Canopen_GetWheelTpdoShadowSample(RIGHT_NODE_ID, now_ms, &right);
    if (!left.valid || (left.state != CANOPEN_TPDO_SAMPLE_FRESH) ||
        ((uint32_t)(now_ms - left.timestamp_ms) >
         DRIVE_WHEEL_FEEDBACK_STALE_MS)) {
        return "NODE1_NOT_FRESH";
    }
    if (!right.valid || (right.state != CANOPEN_TPDO_SAMPLE_FRESH) ||
        ((uint32_t)(now_ms - right.timestamp_ms) >
         DRIVE_WHEEL_FEEDBACK_STALE_MS)) {
        return "NODE2_NOT_FRESH";
    }
    if (Drive_TpdoPairSkew(&left, &right) >
        DRIVE_WHEEL_FEEDBACK_MAX_SKEW_MS) {
        return "PAIR_SKEW";
    }
    if ((Canopen_GetWheelFeedbackTransport() !=
         CANOPEN_WHEEL_FEEDBACK_TPDO_SHADOW) ||
        (Canopen_GetWheelFeedbackAuthority() !=
         CANOPEN_WHEEL_AUTHORITY_SDO)) {
        return "MODE_NOT_SHADOW";
    }
    return NULL;
}

static bool Drive_TpdoStartupCanConfigure(void)
{
    return Drive_TpdoDisabledGate() == NULL;
}

void Drive_PrintPdoStartupDiagnostic(uint32_t now_ms, uint32_t snap, uint32_t req)
{
    const char *state = "UNKNOWN", *blocked = "UNKNOWN_STATE";
    switch (s_drive.tpdo_active.startup_state) {
        case DRIVE_TPDO_STARTUP_WAIT_SAFE:
            state = "WAIT_SAFE";
            blocked = Drive_TpdoDisabledGate();
            break;
        case DRIVE_TPDO_STARTUP_WAIT_FRESH:
            state = "WAIT_FRESH";
            blocked = Drive_TpdoActiveEnableGate(now_ms);
            break;
        case DRIVE_TPDO_STARTUP_ARMING:
            state = "ARMING";
            blocked = Drive_TpdoDisabledGate();
            if (blocked == NULL) { blocked = "WAIT_POST_ARM_PAIR"; }
            break;
        case DRIVE_TPDO_STARTUP_COMPLETE:
            state = "COMPLETE";
            blocked = (Canopen_GetWheelFeedbackTransport() != CANOPEN_WHEEL_FEEDBACK_TPDO_ACTIVE) ?
                "FEEDBACK_NOT_ACTIVE" : (WheelPdoConfig_Ready(now_ms) ? NULL : "RPDO_NOT_READY");
            break;
        case DRIVE_TPDO_STARTUP_FAILED:
            state = "FAILED";
            blocked = Canopen_GetLastWheelTpdoActiveFault();
            break;
        default: break;
    }
    /* Always compiled, including RELEASE_TEST. Reads only: no state/epoch
     * changes. History flags remain observable even after communication is ready. */
    (void)JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
        "DRIVE_RPDO 9/9 startup=%s blocked=%s tracking=%u inhibit=%u drive=%s hw_dis=%u basic=%u snap=%lu req=%08lX\r\n",
        state, blocked != NULL ? blocked : "NONE",
        s_tracking_stop_latched ? 1U : 0U, s_disable_motion_inhibit_latched ? 1U : 0U,
        Drive_StateName(s_drive.core.state), s_hardware_disable_confirmed ? 1U : 0U,
        (unsigned)DRIVE_BASIC_BRINGUP_PROFILE, (unsigned long)snap, (unsigned long)req);
}

static void Drive_TpdoStartupFail(const char *reason)
{
    Canopen_LatchWheelTpdoStartupFault(reason);
    Drive_ResetYawRateControl(true);
    s_drive.tpdo_active.startup_state = DRIVE_TPDO_STARTUP_FAILED;
    s_drive.tpdo_active.enable_command_active = false;
    s_drive.tpdo_active.enable_is_startup = false;
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_CRITICAL,
        "DRIVE_TPDO_STARTUP state=FAILED reason=%s authority=TPDO sdo606c=0\r\n",
        (reason != NULL) ? reason : "TPDO_STARTUP_FAILED");
}

static bool Drive_BeginWheelTpdoActiveArm(uint32_t now_ms,
                                          uint32_t request_nonce,
                                          bool startup)
{
    if (!Canopen_BeginWheelTpdoActiveArm(now_ms)) {
        return false;
    }

    /* This is the only feedback-epoch reset for Active arming and occurs only
     * after every side-effect-free precondition above has passed. */
    Drive_ResetYawRateControl(true);
    s_drive.feedback.odom_authoritative_pair_sequence = 0U;
    s_drive.feedback.odom_authoritative_pair_timestamp_ms = 0U;
    s_drive.tpdo_active.enable_command_active = true;
    s_drive.tpdo_active.enable_is_startup = startup;
    s_drive.tpdo_active.enable_request_nonce = request_nonce;
    s_drive.tpdo_active.enable_start_ms = now_ms;
    return true;
}

bool Drive_CommandWheelTpdoActiveEnable(uint32_t request_nonce)
{
    uint32_t now_ms = HAL_GetTick();
    const char *reason;

    if (s_drive.tpdo_active.enable_command_active) {
        reason = "ACTIVE_COMMAND_BUSY";
    } else {
        reason = Drive_TpdoActiveEnableGate(now_ms);
    }
    if (reason != NULL) {
        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_STATUS,
            "DRIVE_TPDO_ACTIVE state=FAILED req=%08lX reason=%s\r\n",
            (unsigned long)request_nonce, reason);
        return false;
    }
    if (!Drive_BeginWheelTpdoActiveArm(
            now_ms, request_nonce, false)) {
        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_STATUS,
            "DRIVE_TPDO_ACTIVE state=FAILED req=%08lX reason=INTERNAL\r\n",
            (unsigned long)request_nonce);
        return false;
    }
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_STATUS,
        "DRIVE_TPDO_ACTIVE state=STARTED req=%08lX\r\n",
        (unsigned long)request_nonce);
    return true;
}

void Drive_ServiceWheelTpdoActiveCommand(uint32_t now_ms)
{
    uint32_t request_nonce;
    bool startup;

    if (!s_drive.tpdo_active.enable_command_active) {
        return;
    }
    request_nonce = s_drive.tpdo_active.enable_request_nonce;
    startup = s_drive.tpdo_active.enable_is_startup;
    if ((Canopen_GetWheelFeedbackTransport() ==
         CANOPEN_WHEEL_FEEDBACK_TPDO_ACTIVE) &&
        (Canopen_GetWheelFeedbackAuthority() ==
         CANOPEN_WHEEL_AUTHORITY_TPDO)) {
        s_drive.tpdo_active.enable_command_active = false;
        s_drive.tpdo_active.enable_is_startup = false;
        if (startup) {
            s_drive.tpdo_active.startup_state =
                DRIVE_TPDO_STARTUP_COMPLETE;
            (void)JetsonTx_Printf(
                JETSON_TX_PRIORITY_STATUS,
                "DRIVE_TPDO_STARTUP state=COMPLETE mode=TPDO_ACTIVE authority=TPDO sdo606c=0\r\n");
        } else {
            (void)JetsonTx_Printf(
                JETSON_TX_PRIORITY_STATUS,
                "DRIVE_TPDO_ACTIVE state=COMPLETE req=%08lX mode=TPDO_ACTIVE authority=TPDO\r\n",
                (unsigned long)request_nonce);
        }
        return;
    }
    if ((uint32_t)(now_ms - s_drive.tpdo_active.enable_start_ms) >
        DRIVE_WHEEL_TPDO_ACTIVE_ARM_TIMEOUT_MS) {
        Canopen_AbortWheelTpdoActiveArm();
        s_drive.tpdo_active.enable_command_active = false;
        s_drive.tpdo_active.enable_is_startup = false;
        if (startup) {
            Drive_TpdoStartupFail("TPDO_STARTUP_ARM_TIMEOUT");
        } else {
            (void)JetsonTx_Printf(
                JETSON_TX_PRIORITY_STATUS,
                "DRIVE_TPDO_ACTIVE state=FAILED req=%08lX reason=ARM_TIMEOUT\r\n",
                (unsigned long)request_nonce);
        }
    }
}

bool Drive_ServiceWheelTpdoStartup(uint32_t now_ms)
{
    const char *reason;
    CanopenWheelTpdoConfigDiagnostic_t diagnostic = {0};

    if ((Canopen_GetWheelFeedbackTransport() ==
         CANOPEN_WHEEL_FEEDBACK_TPDO_ACTIVE) &&
        (Canopen_GetWheelFeedbackAuthority() ==
         CANOPEN_WHEEL_AUTHORITY_TPDO)) {
        if (s_drive.tpdo_active.startup_state !=
            DRIVE_TPDO_STARTUP_COMPLETE) {
            s_drive.tpdo_active.startup_state =
                DRIVE_TPDO_STARTUP_COMPLETE;
            (void)JetsonTx_Printf(
                JETSON_TX_PRIORITY_STATUS,
                "DRIVE_TPDO_STARTUP state=COMPLETE mode=TPDO_ACTIVE authority=TPDO sdo606c=0\r\n");
        }
        return true;
    }

    switch (s_drive.tpdo_active.startup_state) {
        case DRIVE_TPDO_STARTUP_WAIT_SAFE:
            if (!Drive_TpdoStartupCanConfigure()) {
                return false;
            }
            (void)JetsonTx_Printf(
                JETSON_TX_PRIORITY_STATUS,
                "DRIVE_TPDO_STARTUP state=STARTED\r\n");
            if (!WheelPdoConfig_Configure()) {
                Drive_TpdoStartupFail(WheelPdoConfig_FailureReason());
                return false;
            }
            if (!Drive_ConfigureWheelTpdoNodes(true, true, &diagnostic)) {
                (void)JetsonTx_Printf(
                    JETSON_TX_PRIORITY_CRITICAL,
                    "DRIVE_TPDO_STARTUP_CONFIG state=FAILED result=%s node=%u step=%u index=%04X sub=%02X sdo=%s abort=%08lX expected=%08lX actual=%08lX\r\n",
                    Drive_TpdoConfigResultName(diagnostic.result),
                    (unsigned)diagnostic.node_id,
                    (unsigned)diagnostic.step,
                    (unsigned)diagnostic.index,
                    (unsigned)diagnostic.subindex,
                    Drive_TpdoConfigSdoResultName(diagnostic.sdo_result),
                    (unsigned long)diagnostic.abort_code,
                    (unsigned long)diagnostic.expected_value,
                    (unsigned long)diagnostic.actual_value);
                Drive_TpdoStartupFail("TPDO_STARTUP_CONFIG_FAILED");
                return false;
            }
            s_drive.tpdo_active.startup_qualify_start_ms = HAL_GetTick();
            s_drive.tpdo_active.startup_state =
                DRIVE_TPDO_STARTUP_WAIT_FRESH;
            (void)JetsonTx_Printf(
                JETSON_TX_PRIORITY_STATUS,
                "DRIVE_TPDO_STARTUP state=WAIT_FRESH configured=1/1\r\n");
            return false;

        case DRIVE_TPDO_STARTUP_WAIT_FRESH:
            if (s_drive.tpdo_active.enable_command_active) {
                return false;
            }
            reason = Drive_TpdoActiveEnableGate(now_ms);
            if (reason == NULL) {
                if (!Drive_BeginWheelTpdoActiveArm(now_ms, 0U, true)) {
                    Drive_TpdoStartupFail("TPDO_STARTUP_ARM_FAILED");
                    return false;
                }
                s_drive.tpdo_active.startup_state =
                    DRIVE_TPDO_STARTUP_ARMING;
                (void)JetsonTx_Printf(
                    JETSON_TX_PRIORITY_STATUS,
                    "DRIVE_TPDO_STARTUP state=ARMING\r\n");
                return false;
            }
            if ((uint32_t)(now_ms -
                s_drive.tpdo_active.startup_qualify_start_ms) >
                DRIVE_WHEEL_TPDO_ACTIVE_ARM_TIMEOUT_MS) {
                Drive_TpdoStartupFail(reason);
            }
            return false;

        case DRIVE_TPDO_STARTUP_ARMING:
        case DRIVE_TPDO_STARTUP_FAILED:
            return false;

        case DRIVE_TPDO_STARTUP_COMPLETE:
        default:
            return false;
    }
}

static const char *Drive_TpdoActiveDisableGate(void)
{
    if (s_drive.core.enabled) {
        return "DRIVE_ENABLED";
    }
    if (!s_hardware_disable_confirmed) {
        return "HARDWARE_DISABLE_UNCONFIRMED";
    }
    if (s_hardware_enable_confirmed) {
        return "HARDWARE_ENABLE_CONFIRMED";
    }
    if (s_drive.safety.transition.active ||
        s_drive.lift_stop.request.active ||
        s_drive.fault_reset.transaction.active ||
        s_drive.tpdo_active.enable_command_active) {
        return "DRIVE_TRANSITION_ACTIVE";
    }
    if (!Drive_LastCommandTargetIsZero()) {
        return "TARGET_NONZERO";
    }
    if (Drive_LiftIsMoving()) {
        return "LIFT_NOT_STOPPED";
    }
    if (Canopen_GetWheelFeedbackAuthority() !=
        CANOPEN_WHEEL_AUTHORITY_TPDO) {
        return "NOT_TPDO_ACTIVE";
    }
    return NULL;
}

bool Drive_CommandWheelTpdoActiveDisable(uint32_t request_nonce)
{
    const char *reason = Drive_TpdoActiveDisableGate();

    if (reason != NULL) {
        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_STATUS,
            "DRIVE_TPDO_ACTIVE_DISABLE state=FAILED req=%08lX reason=%s\r\n",
            (unsigned long)request_nonce, reason);
        return false;
    }
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_STATUS,
        "DRIVE_TPDO_ACTIVE_DISABLE state=STARTED req=%08lX\r\n",
        (unsigned long)request_nonce);
    if (!Canopen_DisableWheelTpdoActive()) {
        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_STATUS,
            "DRIVE_TPDO_ACTIVE_DISABLE state=FAILED req=%08lX reason=INTERNAL\r\n",
            (unsigned long)request_nonce);
        return false;
    }
    Drive_ResetYawRateControl(true);
    s_drive.feedback.odom_authoritative_pair_sequence = 0U;
    s_drive.feedback.odom_authoritative_pair_timestamp_ms = 0U;
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_STATUS,
        "DRIVE_TPDO_ACTIVE_DISABLE state=COMPLETE req=%08lX mode=TPDO_SHADOW authority=SDO\r\n",
        (unsigned long)request_nonce);
    return true;
}

void Drive_PrintWheelTpdoActiveStatus(uint32_t request_nonce)
{
    uint32_t now_ms = HAL_GetTick();
    CanopenWheelTpdoSample_t left = {0};
    CanopenWheelTpdoSample_t right = {0};
    CanopenWheelTpdoPair_t pair = {0};
    CanopenRuntimeDiagnostics_t diag = Canopen_GetRuntimeDiagnostics();
    CanopenWheelFeedbackAuthority_t authority =
        Canopen_GetWheelFeedbackAuthority();
    uint32_t pair_age_ms;

    (void)Canopen_GetWheelTpdoShadowSample(LEFT_NODE_ID, now_ms, &left);
    (void)Canopen_GetWheelTpdoShadowSample(RIGHT_NODE_ID, now_ms, &right);
    (void)Canopen_GetWheelTpdoPair(now_ms, &pair);
    pair_age_ms = pair.valid ?
        (uint32_t)(now_ms - pair.timestamp_ms) : UINT32_MAX;
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_STATUS,
        "DRIVE_TPDO_ACTIVE_STATUS 1/8 mode=%s authority=%s odom_authority=%s configured=%u/%u req=%08lX\r\n",
        Canopen_WheelFeedbackTransportName(
            Canopen_GetWheelFeedbackTransport()),
        Canopen_WheelFeedbackAuthorityName(authority),
        Canopen_WheelFeedbackAuthorityName(authority),
        Canopen_WheelTpdoConfigured(LEFT_NODE_ID) ? 1U : 0U,
        Canopen_WheelTpdoConfigured(RIGHT_NODE_ID) ? 1U : 0U,
        (unsigned long)request_nonce);
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_STATUS,
        "DRIVE_TPDO_ACTIVE_STATUS 2/8 drive_state=%s software_enabled=%u hardware_enable_confirmed=%u hardware_disable_confirmed=%u req=%08lX\r\n",
        Drive_StateName(s_drive.core.state), s_drive.core.enabled ? 1U : 0U,
        s_hardware_enable_confirmed ? 1U : 0U,
        s_hardware_disable_confirmed ? 1U : 0U,
        (unsigned long)request_nonce);
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_STATUS,
        "DRIVE_TPDO_ACTIVE_STATUS 3/8 target_zero=%u lift_stopped=%u transition_active=%u fault_active=%u estop_active=%u req=%08lX\r\n",
        Drive_LastCommandTargetIsZero() ? 1U : 0U,
        Drive_LiftIsMoving() ? 0U : 1U,
        (s_drive.safety.transition.active ||
         s_drive.fault_reset.transaction.active ||
         s_drive.tpdo_active.enable_command_active) ? 1U : 0U,
        (s_drive.core.state == DRIVE_STATE_FAULT) ? 1U : 0U,
        Drive_TpdoEstopActive() ? 1U : 0U,
        (unsigned long)request_nonce);
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_STATUS,
        "DRIVE_TPDO_ACTIVE_STATUS 4/8 node=1 state=%s raw=%ld norm_mm_s=%ld seq=%lu age_ms=%lu max_gap_ms=%lu req=%08lX\r\n",
        Canopen_WheelTpdoSampleStateName(left.state), (long)left.raw,
        (long)Motion_RoundFloatToI32(
            Motion_LogicalMotorRPMToWheelVelocityMmS(
                Motion_RawVelocityDecToLogicalMotorRPM(
                    LEFT_NODE_ID, left.raw))),
        (unsigned long)left.sequence,
        (unsigned long)Drive_TpdoDiagnosticAge(
            now_ms, left.seen, left.timestamp_ms),
        (unsigned long)left.max_interarrival_gap_ms,
        (unsigned long)request_nonce);
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_STATUS,
        "DRIVE_TPDO_ACTIVE_STATUS 5/8 node=2 state=%s raw=%ld norm_mm_s=%ld seq=%lu age_ms=%lu max_gap_ms=%lu req=%08lX\r\n",
        Canopen_WheelTpdoSampleStateName(right.state), (long)right.raw,
        (long)Motion_RoundFloatToI32(
            Motion_LogicalMotorRPMToWheelVelocityMmS(
                Motion_RawVelocityDecToLogicalMotorRPM(
                    RIGHT_NODE_ID, right.raw))),
        (unsigned long)right.sequence,
        (unsigned long)Drive_TpdoDiagnosticAge(
            now_ms, right.seen, right.timestamp_ms),
        (unsigned long)right.max_interarrival_gap_ms,
        (unsigned long)request_nonce);
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_STATUS,
        "DRIVE_TPDO_ACTIVE_STATUS 6/8 pair_valid=%u pair_state=%s pair_seq=%lu pair_age_ms=%lu pair_skew_ms=%lu req=%08lX\r\n",
        pair.valid ? 1U : 0U,
        Canopen_WheelTpdoPairStateName(pair.state),
        (unsigned long)pair.sequence, (unsigned long)pair_age_ms,
        (unsigned long)pair.pair_skew_ms,
        (unsigned long)request_nonce);
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_STATUS,
        "DRIVE_TPDO_ACTIVE_STATUS 7/8 runtime_sdo_606c_poll_enabled=%u runtime_sdo_606c_poll_count=%lu last_fault=%s req=%08lX\r\n",
        Canopen_RuntimeSdo606cPollEnabled() ? 1U : 0U,
        (unsigned long)diag.runtime_sdo_606c_poll_count,
        Canopen_GetLastWheelTpdoActiveFault(),
        (unsigned long)request_nonce);
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_STATUS,
        "DRIVE_TPDO_ACTIVE_STATUS 8/8 stop_reject=%s reject_new/invalid=%lu/%lu odom_suppressed=%lu req=%08lX\r\n",
        (s_drive.tpdo_active.last_stop_reject_reason != NULL) ?
            s_drive.tpdo_active.last_stop_reject_reason : "NONE",
        (unsigned long)s_drive.tpdo_active.stop_pair_not_new_count,
        (unsigned long)s_drive.tpdo_active.stop_pair_invalid_count,
        (unsigned long)s_drive.tpdo_active.odom_stale_suppressed_count,
        (unsigned long)request_nonce);
}

#ifdef AMR_HOST_TEST
void Drive_TestSetTpdoEstopActive(bool active)
{
    s_tpdo_test_estop_active = active;
}

bool Drive_TestTpdoActiveCommandPending(void)
{
    return s_drive.tpdo_active.enable_command_active;
}
#endif

void Drive_PrintHelp(void)
{
    uart_printf("Commands:\r\n");
    uart_printf("  help\r\n");
    uart_printf("  enable\r\n");
    uart_printf("  disable\r\n");
    uart_printf("  stop\r\n");
    uart_printf("  status\r\n");
    uart_printf("  fwd <rpm> <ms>\r\n");
    uart_printf("  back <rpm> <ms>\r\n");
    uart_printf("  spin <rpm> <ms>\r\n");
    uart_printf("  raw <left_rpm> <right_rpm> <ms>\r\n");
    uart_printf("  node1 <rpm> <ms>\r\n");
    uart_printf("  node2 <rpm> <ms>\r\n");
    uart_printf("  odom\r\n");
    uart_printf("  odom_reset\r\n");
    uart_printf("  fwd_odom <rpm> <ms>\r\n");
    uart_printf("  spin_odom <rpm> <ms>\r\n");
    uart_printf("  raw_odom <left_rpm> <right_rpm> <ms>\r\n");
    uart_printf("  move_mm <distance_mm> <max_rpm>\r\n");
    uart_printf("  turn_deg <angle_deg> <max_rpm>\r\n");
    uart_printf("  cmd_vel <linear_mm_s> <angular_mrad_s>\r\n");
    uart_printf("Jetson UART protocol: CMD_VEL <linear_mm_s> <angular_mrad_s>\r\n");
    uart_printf("Limits: abs(rpm)<=%d duration<=%lu ms\r\n",
                MAX_TEST_MOTOR_RPM,
                (unsigned long)MAX_TEST_DURATION_MS);
}
