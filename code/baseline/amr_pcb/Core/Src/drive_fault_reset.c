/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : drive_fault_reset.c
  * @brief          : Private Drive module implementation.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "drive.h"
#include "app_config.h"
#include "canopen.h"
#include "diagnostics.h"
#include "drive_internal.h"
#include "jetson_tx.h"
#include "main.h"
#include "motion_control.h"
#include "odom.h"
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

static const char *Drive_FaultResetPhaseName(DriveFaultResetPhase_t phase)
{
    switch (phase) {
        case DRIVE_FAULT_RESET_PHASE_PRECHECK_ZERO_FEEDBACK:
            return "ZEROCHK";
        case DRIVE_FAULT_RESET_PHASE_READ_STATUS: return "READ_STATUS";
        case DRIVE_FAULT_RESET_PHASE_READ_ERROR_CODE: return "READ_603F";
        case DRIVE_FAULT_RESET_PHASE_READ_ERROR_REGISTER: return "READ_1001";
        case DRIVE_FAULT_RESET_PHASE_WRITE_0086: return "WRITE_0086";
        case DRIVE_FAULT_RESET_PHASE_WAIT_RESET: return "WAIT_RESET";
        case DRIVE_FAULT_RESET_PHASE_WRITE_0006: return "WRITE_0006";
        case DRIVE_FAULT_RESET_PHASE_READ_VERIFY: return "READ_VERIFY";
        case DRIVE_FAULT_RESET_PHASE_NODE_DONE: return "NODE_DONE";
        case DRIVE_FAULT_RESET_PHASE_NONE:
        default: return "NONE";
    }
}

static const char *Drive_FaultResetFailureName(
    DriveFaultResetFailure_t reason)
{
    switch (reason) {
        case DRIVE_FAULT_RESET_FAILURE_PRECONDITION: return "PRECONDITION";
        case DRIVE_FAULT_RESET_FAILURE_OPERATION_ENABLED:
            return "OPERATION_ENABLED";
        case DRIVE_FAULT_RESET_FAILURE_SDO_TIMEOUT: return "SDO_TIMEOUT";
        case DRIVE_FAULT_RESET_FAILURE_SDO_ABORT: return "SDO_ABORT";
        case DRIVE_FAULT_RESET_FAILURE_SDO_PROTOCOL: return "SDO_PROTOCOL";
        case DRIVE_FAULT_RESET_FAILURE_PERSISTENT_FAULT:
            return "PERSISTENT_FAULT";
        case DRIVE_FAULT_RESET_FAILURE_NODE_FAULT: return "NODE_FAULT";
        case DRIVE_FAULT_RESET_FAILURE_RUNTIME_CAN: return "RUNTIME_CAN";
        case DRIVE_FAULT_RESET_FAILURE_OVERALL_TIMEOUT:
            return "OVERALL_TIMEOUT";
        case DRIVE_FAULT_RESET_FAILURE_ZERO_FEEDBACK_TIMEOUT:
            return "ZERO_FEEDBACK_TMO";
        case DRIVE_FAULT_RESET_FAILURE_ZERO_FEEDBACK_NONZERO:
            return "NONZERO_FEEDBACK";
        case DRIVE_FAULT_RESET_FAILURE_NONE:
        default: return "NONE";
    }
}

static const char *Drive_FaultResetResultName(DriveFaultResetResult_t result)
{
    switch (result) {
        case DRIVE_FAULT_RESET_RESULT_PENDING: return "PENDING";
        case DRIVE_FAULT_RESET_RESULT_COMPLETE: return "COMPLETE";
        case DRIVE_FAULT_RESET_RESULT_FAILED: return "FAILED";
        case DRIVE_FAULT_RESET_RESULT_IDLE:
        default: return "IDLE";
    }
}

static void Drive_FaultResetRecordStatus(uint8_t node_id,
                                         uint16_t statusword,
                                         uint32_t now_ms)
{
    DriveFaultResetNodeDiagnostic_t *node = &s_drive.fault_reset.transaction.node[node_id];

    node->statusword = statusword;
    node->statusword_valid = true;
    node->statusword_timestamp_ms = now_ms;
    node->statusword_sequence++;
    if (node->statusword_sequence == 0U) {
        node->statusword_sequence++;
    }
    /* Reset-only evidence must not satisfy the normal DISABLE generation. */
    Drive_RecordStatusword(node_id, statusword, now_ms, 0U);
}

static void Drive_FaultResetRecordErrorCode(uint8_t node_id,
                                            uint16_t error_code,
                                            uint32_t now_ms)
{
    DriveFaultResetNodeDiagnostic_t *node = &s_drive.fault_reset.transaction.node[node_id];

    node->error_code = error_code;
    node->error_code_valid = true;
    node->error_code_timestamp_ms = now_ms;
    node->error_code_sequence++;
    if (node->error_code_sequence == 0U) {
        node->error_code_sequence++;
    }
}

static void Drive_FaultResetRecordErrorRegister(uint8_t node_id,
                                                uint8_t error_register,
                                                uint32_t now_ms)
{
    DriveFaultResetNodeDiagnostic_t *node = &s_drive.fault_reset.transaction.node[node_id];

    node->error_register = error_register;
    node->error_register_valid = true;
    node->error_register_timestamp_ms = now_ms;
    node->error_register_sequence++;
    if (node->error_register_sequence == 0U) {
        node->error_register_sequence++;
    }
}

static void Drive_PrintFaultResetNodeDiagnostic(
    const char *side,
    uint8_t node_id,
    uint32_t now_ms,
    unsigned snap,
    uint32_t request_nonce)
{
    const DriveFaultResetNodeDiagnostic_t *node =
        &s_drive.fault_reset.transaction.node[node_id];

    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_DEBUG,
        "DRIVE_FAULT_RESET_%s node=%u sw=%04X/%u/%lu/%lu error=%04X/%u/%lu/%lu reg=%02X/%u/%lu/%lu reset=%u cleared=%u snap=%u req=%08lX\r\n",
        side,
        (unsigned)node_id,
        (unsigned)node->statusword,
        node->statusword_valid ? 1U : 0U,
        (unsigned long)Drive_DiagnosticAge(
            now_ms, node->statusword_valid,
            node->statusword_timestamp_ms),
        (unsigned long)node->statusword_sequence,
        (unsigned)node->error_code,
        node->error_code_valid ? 1U : 0U,
        (unsigned long)Drive_DiagnosticAge(
            now_ms, node->error_code_valid,
            node->error_code_timestamp_ms),
        (unsigned long)node->error_code_sequence,
        (unsigned)node->error_register,
        node->error_register_valid ? 1U : 0U,
        (unsigned long)Drive_DiagnosticAge(
            now_ms, node->error_register_valid,
            node->error_register_timestamp_ms),
        (unsigned long)node->error_register_sequence,
        node->reset_written ? 1U : 0U,
        node->cleared ? 1U : 0U,
        snap, (unsigned long)request_nonce);
}

void Drive_PrintFaultResetDiagnostic(uint32_t now_ms,
                                            unsigned snap,
                                            uint32_t request_nonce)
{
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_DEBUG,
        "DRIVE_FAULT_RESET_DIAG active=%u result=%s phase=%s node=%u failure=%s fail_node=%u abort=0x%08lX timeouts=%lu cleared=%u/%u snap=%u req=%08lX\r\n",
        s_drive.fault_reset.transaction.active ? 1U : 0U,
        Drive_FaultResetResultName(s_drive.fault_reset.transaction.result),
        Drive_FaultResetPhaseName(s_drive.fault_reset.transaction.phase),
        (unsigned)s_drive.fault_reset.transaction.node_id,
        Drive_FaultResetFailureName(s_drive.fault_reset.transaction.failure_reason),
        (unsigned)s_drive.fault_reset.transaction.failure_node_id,
        (unsigned long)s_drive.fault_reset.transaction.last_abort_code,
        (unsigned long)s_drive.fault_reset.transaction.transient_timeout_count,
        s_drive.fault_reset.transaction.node[LEFT_NODE_ID].cleared ? 1U : 0U,
        s_drive.fault_reset.transaction.node[RIGHT_NODE_ID].cleared ? 1U : 0U,
        snap, (unsigned long)request_nonce);
    Drive_PrintFaultResetNodeDiagnostic("LEFT", LEFT_NODE_ID, now_ms,
                                        snap, request_nonce);
    Drive_PrintFaultResetNodeDiagnostic("RIGHT", RIGHT_NODE_ID, now_ms,
                                        snap, request_nonce);
}

static void Drive_FaultResetSetPhase(DriveFaultResetPhase_t phase,
                                     uint32_t now_ms)
{
    s_drive.fault_reset.transaction.phase = phase;
    s_drive.fault_reset.transaction.phase_start_ms = now_ms;
    s_drive.fault_reset.transaction.action_attempts = 0U;
    s_drive.fault_reset.transaction.request_pending = false;
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_STATUS,
        "DRIVE_FAULT_RESET_SAFE state=PENDING phase=%s node=%u gen=%lu req=%08lX\r\n",
        Drive_FaultResetPhaseName(phase),
        (unsigned)s_drive.fault_reset.transaction.node_id,
        (unsigned long)s_drive.fault_reset.transaction.generation,
        (unsigned long)s_drive.fault_reset.transaction.request_nonce);
}

void Drive_FaultResetFail(DriveFaultResetFailure_t reason,
                                 uint8_t node_id,
                                 uint32_t now_ms)
{
    const DriveFaultResetNodeDiagnostic_t *node;
    bool preserve_precheck_state =
        (s_drive.fault_reset.transaction.phase ==
         DRIVE_FAULT_RESET_PHASE_PRECHECK_ZERO_FEEDBACK);

    if (s_drive.fault_reset.transaction.request_pending) {
        Canopen_CancelAsyncOwner(CANOPEN_SDO_OWNER_SAFETY_CONFIRM,
                                 now_ms);
    }
    if (node_id == 0U) {
        node_id = s_drive.fault_reset.transaction.pending_node_id;
    }
    if ((node_id != LEFT_NODE_ID) && (node_id != RIGHT_NODE_ID)) {
        node_id = 0U;
    }
    node = &s_drive.fault_reset.transaction.node[node_id];
    s_drive.fault_reset.transaction.active = false;
    s_drive.fault_reset.transaction.request_pending = false;
    s_drive.fault_reset.transaction.result = DRIVE_FAULT_RESET_RESULT_FAILED;
    s_drive.fault_reset.transaction.failure_reason = reason;
    s_drive.fault_reset.transaction.failure_node_id = node_id;
    if (!preserve_precheck_state) {
        DriveCore_SetSoftwareEnabled(false);
        DriveCore_SetHardwareEnableConfirmed(false);
        DriveCore_SetHardwareDisableConfirmed(false);
        DriveCore_SetState(DRIVE_STATE_FAULT);
    }
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_CRITICAL,
        "DRIVE_FAULT_RESET_SAFE state=FAILED reason=%s node=%u phase=%s abort=0x%08lX timeouts=%lu error=0x%04X/%u reg=0x%02X/%u gen=%lu req=%08lX\r\n",
        Drive_FaultResetFailureName(reason),
        (unsigned)node_id,
        Drive_FaultResetPhaseName(s_drive.fault_reset.transaction.phase),
        (unsigned long)s_drive.fault_reset.transaction.last_abort_code,
        (unsigned long)s_drive.fault_reset.transaction.transient_timeout_count,
        (unsigned)node->error_code,
        node->error_code_valid ? 1U : 0U,
        (unsigned)node->error_register,
        node->error_register_valid ? 1U : 0U,
        (unsigned long)s_drive.fault_reset.transaction.generation,
        (unsigned long)s_drive.fault_reset.transaction.request_nonce);
}

static void Drive_FaultResetComplete(void)
{
    s_drive.fault_reset.transaction.active = false;
    s_drive.fault_reset.transaction.request_pending = false;
    s_drive.fault_reset.transaction.result = DRIVE_FAULT_RESET_RESULT_COMPLETE;
    s_drive.fault_reset.transaction.failure_reason = DRIVE_FAULT_RESET_FAILURE_NONE;
    s_drive.fault_reset.transaction.failure_node_id = 0U;
    DriveCore_SetSoftwareEnabled(false);
    DriveCore_SetHardwareEnableConfirmed(false);
#ifndef AMR_HOST_TEST
    DriveCore_SetHardwareDisableConfirmed(true);
#endif
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_STATUS,
        "DRIVE_FAULT_RESET_SAFE state=COMPLETE cleared=%u/%u gen=%lu req=%08lX\r\n",
        s_drive.fault_reset.transaction.node[LEFT_NODE_ID].cleared ? 1U : 0U,
        s_drive.fault_reset.transaction.node[RIGHT_NODE_ID].cleared ? 1U : 0U,
        (unsigned long)s_drive.fault_reset.transaction.generation,
        (unsigned long)s_drive.fault_reset.transaction.request_nonce);
}

static DriveSafetySdoActionResult_t Drive_FaultResetServiceSdoAction(
    uint32_t now_ms,
    uint8_t node_id,
    DriveSafetySdoAction_t action,
    uint16_t index,
    uint32_t write_value,
    uint32_t *read_value)
{
    const CanopenSdoOwner_t owner = CANOPEN_SDO_OWNER_SAFETY_CONFIRM;

    if ((int32_t)(now_ms - s_drive.fault_reset.transaction.next_action_ms) < 0) {
        return DRIVE_SDO_ACTION_WAIT;
    }
    if (!s_drive.fault_reset.transaction.request_pending) {
        CanopenAsyncStartResult_t start_result;

        if (action == DRIVE_SDO_ACTION_READ_U16) {
            start_result = Canopen_AsyncReadU16(
                node_id, index, 0x00U, owner, now_ms,
                DRIVE_SAFETY_SDO_TIMEOUT_MS);
        } else if (action == DRIVE_SDO_ACTION_READ_I8) {
            start_result = Canopen_AsyncReadI8(
                node_id, index, 0x00U, owner, now_ms,
                DRIVE_SAFETY_SDO_TIMEOUT_MS);
        } else if (action == DRIVE_SDO_ACTION_WRITE_U16) {
            start_result = Canopen_AsyncWriteU16(
                node_id, index, 0x00U, (uint16_t)write_value,
                owner, now_ms, DRIVE_SAFETY_SDO_TIMEOUT_MS);
        } else {
            return DRIVE_SDO_ACTION_ERROR;
        }
        if (start_result == CANOPEN_ASYNC_START_BUSY) {
            return DRIVE_SDO_ACTION_WAIT;
        }
        if (start_result != CANOPEN_ASYNC_START_OK) {
            return DRIVE_SDO_ACTION_ERROR;
        }
        s_drive.fault_reset.transaction.request_pending = true;
        s_drive.fault_reset.transaction.pending_node_id = node_id;
        s_drive.fault_reset.transaction.action_attempts++;
        return DRIVE_SDO_ACTION_WAIT;
    }
    if (s_drive.fault_reset.transaction.pending_node_id != node_id) {
        return DRIVE_SDO_ACTION_ERROR;
    }
    {
        uint32_t value = 0U;
        uint32_t abort_code = 0U;
        CanopenAsyncResult_t poll_result = Canopen_AsyncPoll(
            node_id, owner, now_ms, &value, &abort_code);

        if (poll_result == CANOPEN_ASYNC_RESULT_PENDING) {
            return DRIVE_SDO_ACTION_WAIT;
        }
        s_drive.fault_reset.transaction.request_pending = false;
        if (poll_result == CANOPEN_ASYNC_RESULT_SUCCESS) {
            if (read_value != NULL) {
                *read_value = value;
            }
            return DRIVE_SDO_ACTION_SUCCESS;
        }
        if (poll_result == CANOPEN_ASYNC_RESULT_TIMEOUT) {
            s_drive.fault_reset.transaction.transient_timeout_count++;
            s_drive.fault_reset.transaction.next_action_ms = now_ms + DRIVE_SAFETY_RETRY_MS;
            return DRIVE_SDO_ACTION_TIMEOUT;
        }
        if (poll_result == CANOPEN_ASYNC_RESULT_ABORT) {
            s_drive.fault_reset.transaction.last_abort_code = abort_code;
            return DRIVE_SDO_ACTION_ABORT;
        }
        return DRIVE_SDO_ACTION_ERROR;
    }
}

static bool Drive_FaultResetActionMustFail(
    DriveSafetySdoActionResult_t result,
    uint32_t now_ms)
{
    DriveFaultResetFailure_t reason;

    if ((result == DRIVE_SDO_ACTION_WAIT) ||
        (result == DRIVE_SDO_ACTION_SUCCESS)) {
        return false;
    }
    if ((result == DRIVE_SDO_ACTION_TIMEOUT) &&
        (s_drive.fault_reset.transaction.action_attempts <
         DRIVE_SAFETY_SDO_MAX_ATTEMPTS)) {
        return false;
    }
    if (result == DRIVE_SDO_ACTION_TIMEOUT) {
        reason = DRIVE_FAULT_RESET_FAILURE_SDO_TIMEOUT;
    } else if (result == DRIVE_SDO_ACTION_ABORT) {
        reason = DRIVE_FAULT_RESET_FAILURE_SDO_ABORT;
    } else {
        reason = DRIVE_FAULT_RESET_FAILURE_SDO_PROTOCOL;
    }
    Drive_FaultResetFail(reason, s_drive.fault_reset.transaction.node_id, now_ms);
    return true;
}

static bool Drive_FaultResetCanDispatch(uint32_t now_ms)
{
    CanopenRuntimeDiagnostics_t diag;

    if (!s_drive.fault_reset.transaction.active) {
        return false;
    }
    if ((uint32_t)(now_ms - s_drive.fault_reset.transaction.start_ms) >
        DRIVE_ENABLE_TRANSITION_TIMEOUT_MS) {
        Drive_FaultResetFail(
            DRIVE_FAULT_RESET_FAILURE_OVERALL_TIMEOUT,
            s_drive.fault_reset.transaction.node_id, now_ms);
        return false;
    }
    diag = Canopen_GetRuntimeDiagnostics();
    {
        DriveEmcyDelta_t left_delta = Drive_ClassifyEmcyDelta(
            &diag, LEFT_NODE_ID, s_drive.fault_reset.transaction.left_emcy_sequence);
        DriveEmcyDelta_t right_delta = Drive_ClassifyEmcyDelta(
            &diag, RIGHT_NODE_ID, s_drive.fault_reset.transaction.right_emcy_sequence);

        if ((left_delta == DRIVE_EMCY_DELTA_FAULT) ||
            (right_delta == DRIVE_EMCY_DELTA_FAULT)) {
            uint8_t node_id =
                (left_delta == DRIVE_EMCY_DELTA_FAULT) ?
                LEFT_NODE_ID : RIGHT_NODE_ID;

            Drive_FaultResetFail(DRIVE_FAULT_RESET_FAILURE_NODE_FAULT,
                                 node_id, now_ms);
            return false;
        }
        /* A single EMCY 0x0000 per node is that node's reset notification
         * (expected after writing 0x0086); accept it and track the new
         * sequence so a later real EMCY is still detected. */
        s_drive.fault_reset.transaction.left_emcy_sequence =
            diag.emcy_sequence[LEFT_NODE_ID];
        s_drive.fault_reset.transaction.right_emcy_sequence =
            diag.emcy_sequence[RIGHT_NODE_ID];
    }
    return true;
}

static bool Drive_FaultResetFeedbackPairIsFresh(uint32_t now_ms)
{
    const MotionChassisFeedback_t *feedback = &s_drive.feedback.chassis;

    return feedback->valid && !feedback->stale &&
           feedback->left.valid && feedback->right.valid &&
           ((uint32_t)(now_ms - feedback->left.timestamp_ms) <=
            DRIVE_WHEEL_FEEDBACK_STALE_MS) &&
           ((uint32_t)(now_ms - feedback->right.timestamp_ms) <=
            DRIVE_WHEEL_FEEDBACK_STALE_MS) &&
           (feedback->pair_skew_ms <=
            DRIVE_WHEEL_FEEDBACK_MAX_SKEW_MS);
}

static uint8_t Drive_FaultResetPrecheckMissingNode(void)
{
    if (s_drive.feedback.motion.left.sequence ==
        s_drive.fault_reset.transaction.zero_feedback_left_start_sequence) {
        return LEFT_NODE_ID;
    }
    if (s_drive.feedback.motion.right.sequence ==
        s_drive.fault_reset.transaction.zero_feedback_right_start_sequence) {
        return RIGHT_NODE_ID;
    }
    return 0U;
}

static void Drive_FaultResetServiceZeroFeedbackPrecheck(uint32_t now_ms)
{
    const MotionChassisFeedback_t *feedback = &s_drive.feedback.chassis;
    CanopenRuntimeDiagnostics_t diag = Canopen_GetRuntimeDiagnostics();
    bool pair_advanced =
        (feedback->left.sequence !=
         s_drive.fault_reset.transaction.zero_feedback_left_start_sequence) &&
        (feedback->right.sequence !=
         s_drive.fault_reset.transaction.zero_feedback_right_start_sequence);

    if (diag.wheel_feedback_failure_count !=
        s_drive.fault_reset.transaction.zero_feedback_failure_count_start) {
        Drive_FaultResetFail(
            DRIVE_FAULT_RESET_FAILURE_RUNTIME_CAN,
            Drive_FaultResetPrecheckMissingNode(), now_ms);
        return;
    }

    if (pair_advanced && Drive_FaultResetFeedbackPairIsFresh(now_ms)) {
        if (Drive_FeedbackPairIsFreshZero(now_ms)) {
            /* This is the original acceptance boundary: only now is prior
             * DISABLED confirmation invalidated before READ_STATUS begins. */
            DriveCore_SetHardwareDisableConfirmed(false);
            Drive_FaultResetSetPhase(
                DRIVE_FAULT_RESET_PHASE_READ_STATUS, now_ms);
            return;
        }
        Drive_FaultResetFail(
            DRIVE_FAULT_RESET_FAILURE_ZERO_FEEDBACK_NONZERO,
            (fabsf(feedback->left.wheel_velocity_mm_s) >
             DRIVE_STOP_WHEEL_THRESHOLD_MM_S) ? LEFT_NODE_ID :
            ((fabsf(feedback->right.wheel_velocity_mm_s) >
              DRIVE_STOP_WHEEL_THRESHOLD_MM_S) ? RIGHT_NODE_ID : 0U),
            now_ms);
        return;
    }

    if ((uint32_t)(now_ms -
                   s_drive.fault_reset.transaction.phase_start_ms) >=
        DRIVE_FAULT_RESET_ZERO_FEEDBACK_TIMEOUT_MS) {
        Drive_FaultResetFail(
            DRIVE_FAULT_RESET_FAILURE_ZERO_FEEDBACK_TIMEOUT,
            Drive_FaultResetPrecheckMissingNode(), now_ms);
    }
}

static void Drive_FaultResetServiceReadStatus(uint32_t now_ms)
{
    uint32_t value = 0U;
    DriveSafetySdoActionResult_t action_result =
        Drive_FaultResetServiceSdoAction(
            now_ms, s_drive.fault_reset.transaction.node_id,
            DRIVE_SDO_ACTION_READ_U16, 0x6041U, 0U, &value);

    if (action_result == DRIVE_SDO_ACTION_SUCCESS) {
        uint16_t statusword = (uint16_t)value;

        Drive_FaultResetRecordStatus(
            s_drive.fault_reset.transaction.node_id, statusword, now_ms);
        if (Statusword_IsOperationEnabled(statusword)) {
            Drive_FaultResetFail(
                DRIVE_FAULT_RESET_FAILURE_OPERATION_ENABLED,
                s_drive.fault_reset.transaction.node_id, now_ms);
        } else if (!Statusword_HasFault(statusword)) {
            s_drive.fault_reset.transaction.node[s_drive.fault_reset.transaction.node_id].cleared = true;
            Drive_FaultResetSetPhase(
                DRIVE_FAULT_RESET_PHASE_NODE_DONE, now_ms);
        } else {
            Drive_FaultResetSetPhase(
                DRIVE_FAULT_RESET_PHASE_READ_ERROR_CODE, now_ms);
        }
        return;
    }
    (void)Drive_FaultResetActionMustFail(action_result, now_ms);
}

static void Drive_FaultResetServiceReadErrorCode(uint32_t now_ms)
{
    uint32_t value = 0U;
    DriveSafetySdoActionResult_t action_result =
        Drive_FaultResetServiceSdoAction(
            now_ms, s_drive.fault_reset.transaction.node_id,
            DRIVE_SDO_ACTION_READ_U16, 0x603FU, 0U, &value);

    if (action_result == DRIVE_SDO_ACTION_SUCCESS) {
        Drive_FaultResetRecordErrorCode(
            s_drive.fault_reset.transaction.node_id, (uint16_t)value, now_ms);
        Drive_FaultResetSetPhase(
            DRIVE_FAULT_RESET_PHASE_READ_ERROR_REGISTER, now_ms);
        return;
    }
    (void)Drive_FaultResetActionMustFail(action_result, now_ms);
}

static void Drive_FaultResetServiceReadErrorRegister(uint32_t now_ms)
{
    uint32_t value = 0U;
    DriveSafetySdoActionResult_t action_result =
        Drive_FaultResetServiceSdoAction(
            now_ms, s_drive.fault_reset.transaction.node_id,
            DRIVE_SDO_ACTION_READ_I8, 0x1001U, 0U, &value);

    if (action_result == DRIVE_SDO_ACTION_SUCCESS) {
        const DriveFaultResetNodeDiagnostic_t *node;

        Drive_FaultResetRecordErrorRegister(
            s_drive.fault_reset.transaction.node_id, (uint8_t)value, now_ms);
        node = &s_drive.fault_reset.transaction.node[
            s_drive.fault_reset.transaction.node_id];
        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_CRITICAL,
            "DRIVE_FAULT_CAPTURE node=%u status=0x%04X error603F=0x%04X error1001=0x%02X req=%08lX\r\n",
            (unsigned)s_drive.fault_reset.transaction.node_id,
            (unsigned)node->statusword,
            (unsigned)node->error_code,
            (unsigned)node->error_register,
            (unsigned long)s_drive.fault_reset.transaction.request_nonce);
        Drive_FaultResetSetPhase(
            DRIVE_FAULT_RESET_PHASE_WRITE_0086, now_ms);
        return;
    }
    (void)Drive_FaultResetActionMustFail(action_result, now_ms);
}

static void Drive_FaultResetServiceWrite0086(uint32_t now_ms)
{
    DriveSafetySdoActionResult_t action_result =
        Drive_FaultResetServiceSdoAction(
            now_ms, s_drive.fault_reset.transaction.node_id,
            DRIVE_SDO_ACTION_WRITE_U16, 0x6040U, 0x0086U, NULL);

    if (action_result == DRIVE_SDO_ACTION_SUCCESS) {
        s_drive.fault_reset.transaction.node[s_drive.fault_reset.transaction.node_id].reset_written = true;
        Drive_FaultResetSetPhase(
            DRIVE_FAULT_RESET_PHASE_WAIT_RESET, now_ms);
        s_drive.fault_reset.transaction.next_action_ms =
            now_ms + DRIVE_ENABLE_STEP_SHORT_DELAY_MS;
        return;
    }
    (void)Drive_FaultResetActionMustFail(action_result, now_ms);
}

static void Drive_FaultResetServiceWaitReset(uint32_t now_ms)
{
    if ((int32_t)(now_ms - s_drive.fault_reset.transaction.next_action_ms) >= 0) {
        Drive_FaultResetSetPhase(
            DRIVE_FAULT_RESET_PHASE_WRITE_0006, now_ms);
    }
}

static void Drive_FaultResetServiceWrite0006(uint32_t now_ms)
{
    DriveSafetySdoActionResult_t action_result =
        Drive_FaultResetServiceSdoAction(
            now_ms, s_drive.fault_reset.transaction.node_id,
            DRIVE_SDO_ACTION_WRITE_U16, 0x6040U, 0x0006U, NULL);

    if (action_result == DRIVE_SDO_ACTION_SUCCESS) {
        Drive_FaultResetSetPhase(
            DRIVE_FAULT_RESET_PHASE_READ_VERIFY, now_ms);
        s_drive.fault_reset.transaction.next_action_ms =
            now_ms + DRIVE_ENABLE_STEP_SHORT_DELAY_MS;
        return;
    }
    (void)Drive_FaultResetActionMustFail(action_result, now_ms);
}

static void Drive_FaultResetServiceReadVerify(uint32_t now_ms)
{
    uint32_t value = 0U;
    DriveSafetySdoActionResult_t action_result =
        Drive_FaultResetServiceSdoAction(
            now_ms, s_drive.fault_reset.transaction.node_id,
            DRIVE_SDO_ACTION_READ_U16, 0x6041U, 0U, &value);

    if (action_result == DRIVE_SDO_ACTION_SUCCESS) {
        uint16_t statusword = (uint16_t)value;

        Drive_FaultResetRecordStatus(
            s_drive.fault_reset.transaction.node_id, statusword, now_ms);
        if (Statusword_IsOperationEnabled(statusword)) {
            Drive_FaultResetFail(
                DRIVE_FAULT_RESET_FAILURE_OPERATION_ENABLED,
                s_drive.fault_reset.transaction.node_id, now_ms);
        } else if (Statusword_HasFault(statusword)) {
            Drive_FaultResetFail(
                DRIVE_FAULT_RESET_FAILURE_PERSISTENT_FAULT,
                s_drive.fault_reset.transaction.node_id, now_ms);
        } else {
            s_drive.fault_reset.transaction.node[s_drive.fault_reset.transaction.node_id].cleared = true;
            Drive_FaultResetSetPhase(
                DRIVE_FAULT_RESET_PHASE_NODE_DONE, now_ms);
        }
        return;
    }
    (void)Drive_FaultResetActionMustFail(action_result, now_ms);
}

static void Drive_FaultResetServiceNodeDone(uint32_t now_ms)
{
    if (s_drive.fault_reset.transaction.node_id == LEFT_NODE_ID) {
        s_drive.fault_reset.transaction.node_id = RIGHT_NODE_ID;
        Drive_FaultResetSetPhase(
            DRIVE_FAULT_RESET_PHASE_READ_STATUS, now_ms);
    } else {
        Drive_FaultResetComplete();
    }
}

void Drive_FaultResetStep(uint32_t now_ms)
{
    if (!Drive_FaultResetCanDispatch(now_ms)) {
        return;
    }

    switch (s_drive.fault_reset.transaction.phase) {
        case DRIVE_FAULT_RESET_PHASE_PRECHECK_ZERO_FEEDBACK:
            Drive_FaultResetServiceZeroFeedbackPrecheck(now_ms);
            return;
        case DRIVE_FAULT_RESET_PHASE_READ_STATUS:
            Drive_FaultResetServiceReadStatus(now_ms);
            return;
        case DRIVE_FAULT_RESET_PHASE_READ_ERROR_CODE:
            Drive_FaultResetServiceReadErrorCode(now_ms);
            return;
        case DRIVE_FAULT_RESET_PHASE_READ_ERROR_REGISTER:
            Drive_FaultResetServiceReadErrorRegister(now_ms);
            return;
        case DRIVE_FAULT_RESET_PHASE_WRITE_0086:
            Drive_FaultResetServiceWrite0086(now_ms);
            return;
        case DRIVE_FAULT_RESET_PHASE_WAIT_RESET:
            Drive_FaultResetServiceWaitReset(now_ms);
            return;
        case DRIVE_FAULT_RESET_PHASE_WRITE_0006:
            Drive_FaultResetServiceWrite0006(now_ms);
            return;
        case DRIVE_FAULT_RESET_PHASE_READ_VERIFY:
            Drive_FaultResetServiceReadVerify(now_ms);
            return;
        case DRIVE_FAULT_RESET_PHASE_NODE_DONE:
            Drive_FaultResetServiceNodeDone(now_ms);
            return;
        case DRIVE_FAULT_RESET_PHASE_NONE:
        default:
            Drive_FaultResetFail(
                DRIVE_FAULT_RESET_FAILURE_SDO_PROTOCOL,
                s_drive.fault_reset.transaction.node_id, now_ms);
            return;
    }
}

static bool Drive_FaultResetReject(DriveFaultResetFailure_t reason,
                                   uint8_t node_id,
                                   uint32_t request_nonce)
{
    memset(&s_drive.fault_reset.transaction, 0, sizeof(s_drive.fault_reset.transaction));
    s_drive.fault_reset.transaction.result = DRIVE_FAULT_RESET_RESULT_FAILED;
    s_drive.fault_reset.transaction.failure_reason = reason;
    s_drive.fault_reset.transaction.failure_node_id = node_id;
    s_drive.fault_reset.transaction.generation = ++s_drive.fault_reset.generation;
    s_drive.fault_reset.transaction.request_nonce = request_nonce;
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_STATUS,
        "DRIVE_FAULT_RESET_SAFE state=FAILED reason=%s node=%u phase=NONE abort=0x00000000 timeouts=0 error=0x0000/0 reg=0x00/0 gen=%lu req=%08lX\r\n",
        Drive_FaultResetFailureName(reason), (unsigned)node_id,
        (unsigned long)s_drive.fault_reset.transaction.generation,
        (unsigned long)request_nonce);
    return false;
}

bool Drive_CommandFaultResetSafe(uint32_t request_nonce)
{
    uint32_t now_ms = HAL_GetTick();
    CanopenRuntimeDiagnostics_t diag = Canopen_GetRuntimeDiagnostics();
    const MotionChassisFeedback_t *feedback = &s_drive.feedback.chassis;
    DriveState_t state;
    bool transition_active;
    bool lift_stop_request_active;
    bool lift_moving;
    bool target_zero;
    bool feedback_pair_fresh;
    bool feedback_pair_fresh_zero;
    bool hardware_enable_confirmed;
    bool hardware_disable_confirmed;
    bool software_enabled;
    bool tracking_latched;
    bool motion_inhibit_latched;
    bool state_allowed;
    bool fresh_statusword_fault;
    bool non_feedback_precondition_failed;
    uint32_t left_feedback_age_ms;
    uint32_t right_feedback_age_ms;

    if (s_drive.fault_reset.transaction.active) {
        /* A re-issue while active gets its OWN terminal BUSY failure bound
         * to the new nonce; the active transaction continues unchanged and
         * keeps its own binding. */
        uint32_t busy_gen = ++s_drive.fault_reset.generation;

        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_STATUS,
            "DRIVE_FAULT_RESET_SAFE state=FAILED reason=BUSY active_gen=%lu gen=%lu req=%08lX\r\n",
            (unsigned long)s_drive.fault_reset.transaction.generation,
            (unsigned long)busy_gen,
            (unsigned long)request_nonce);
        return false;
    }
    /* FAULT state is authoritative admission. A cached statusword may also
     * admit reset only while it is fresh and its hardware fault bit is set.
     * The transaction still reads a new 0x6041 from each node before acting. */
    state = s_drive.core.state;
    transition_active = s_drive.safety.transition.active;
    lift_stop_request_active = s_drive.lift_stop.request.active;
    lift_moving = Drive_LiftIsMoving();
    target_zero = Drive_LastCommandTargetIsZero();
    feedback_pair_fresh = Drive_FaultResetFeedbackPairIsFresh(now_ms);
    feedback_pair_fresh_zero = Drive_FeedbackPairIsFreshZero(now_ms);
    hardware_enable_confirmed = s_hardware_enable_confirmed;
    hardware_disable_confirmed = s_hardware_disable_confirmed;
    software_enabled = s_drive.core.enabled;
    tracking_latched = s_tracking_stop_latched;
    motion_inhibit_latched = s_disable_motion_inhibit_latched;
    fresh_statusword_fault =
        (s_drive.statusword.left_statusword_valid &&
         ((uint32_t)(now_ms -
                     s_drive.statusword.left_statusword_timestamp_ms) <=
          DRIVE_WHEEL_FEEDBACK_STALE_MS) &&
         Statusword_HasFault(s_drive.statusword.left_statusword)) ||
        (s_drive.statusword.right_statusword_valid &&
         ((uint32_t)(now_ms -
                     s_drive.statusword.right_statusword_timestamp_ms) <=
          DRIVE_WHEEL_FEEDBACK_STALE_MS) &&
         Statusword_HasFault(s_drive.statusword.right_statusword));
    state_allowed = (state == DRIVE_STATE_FAULT) ||
                    fresh_statusword_fault;
    non_feedback_precondition_failed =
        transition_active || lift_stop_request_active || lift_moving ||
        !target_zero || (diag.target_pending_mask != 0U) ||
        hardware_enable_confirmed || software_enabled || !state_allowed;
    left_feedback_age_ms = Drive_DiagnosticAge(
        now_ms, feedback->left.valid, feedback->left.timestamp_ms);
    right_feedback_age_ms = Drive_DiagnosticAge(
        now_ms, feedback->right.valid, feedback->right.timestamp_ms);

    if (non_feedback_precondition_failed ||
        (feedback_pair_fresh && !feedback_pair_fresh_zero)) {
        /* Compact keys are intentional: this complete snapshot must fit one
         * 192-byte bounded Jetson frame. Values are captured above once and
         * are the same values used by the admission decision. */
        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_DEBUG,
            "DRIVE_FAULT_RESET_PRE st=%u ta=%u lsa=%u lm=%u tz=%u pm=%02X fz=%u cv=%u cs=%u lv=%u rv=%u la=%lu ra=%lu sk=%lu wl=%.1f wr=%.1f he=%u se=%u hd=%u tl=%u mi=%u req=%08lX\r\n",
            (unsigned)state,
            transition_active ? 1U : 0U,
            lift_stop_request_active ? 1U : 0U,
            lift_moving ? 1U : 0U,
            target_zero ? 1U : 0U,
            (unsigned)diag.target_pending_mask,
            feedback_pair_fresh_zero ? 1U : 0U,
            feedback->valid ? 1U : 0U,
            feedback->stale ? 1U : 0U,
            feedback->left.valid ? 1U : 0U,
            feedback->right.valid ? 1U : 0U,
            (unsigned long)left_feedback_age_ms,
            (unsigned long)right_feedback_age_ms,
            (unsigned long)feedback->pair_skew_ms,
            (double)feedback->left.wheel_velocity_mm_s,
            (double)feedback->right.wheel_velocity_mm_s,
            hardware_enable_confirmed ? 1U : 0U,
            software_enabled ? 1U : 0U,
            hardware_disable_confirmed ? 1U : 0U,
            tracking_latched ? 1U : 0U,
            motion_inhibit_latched ? 1U : 0U,
            (unsigned long)request_nonce);
        return Drive_FaultResetReject(
            DRIVE_FAULT_RESET_FAILURE_PRECONDITION, 0U, request_nonce);
    }

    memset(&s_drive.fault_reset.transaction, 0, sizeof(s_drive.fault_reset.transaction));
    s_drive.fault_reset.transaction.active = true;
    s_drive.fault_reset.transaction.result = DRIVE_FAULT_RESET_RESULT_PENDING;
    s_drive.fault_reset.transaction.start_ms = now_ms;
    s_drive.fault_reset.transaction.node_id = LEFT_NODE_ID;
    s_drive.fault_reset.transaction.generation = ++s_drive.fault_reset.generation;
    s_drive.fault_reset.transaction.request_nonce = request_nonce;
    s_drive.fault_reset.transaction.left_emcy_sequence =
        diag.emcy_sequence[LEFT_NODE_ID];
    s_drive.fault_reset.transaction.right_emcy_sequence =
        diag.emcy_sequence[RIGHT_NODE_ID];
    s_drive.fault_reset.transaction.zero_feedback_left_start_sequence =
        s_drive.feedback.motion.left.sequence;
    s_drive.fault_reset.transaction.zero_feedback_right_start_sequence =
        s_drive.feedback.motion.right.sequence;
    s_drive.fault_reset.transaction.zero_feedback_failure_count_start =
        diag.wheel_feedback_failure_count;
    DriveCore_SetSoftwareEnabled(false);
    DriveCore_SetHardwareEnableConfirmed(false);
    if (feedback_pair_fresh_zero) {
        /* A normal DISABLE must establish fresh bilateral disable evidence
         * after this reset-only command, even if prior confirmation existed. */
        DriveCore_SetHardwareDisableConfirmed(false);
    }
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_STATUS,
        "DRIVE_FAULT_RESET_SAFE state=STARTED gen=%lu req=%08lX\r\n",
        (unsigned long)s_drive.fault_reset.transaction.generation,
        (unsigned long)request_nonce);
    Drive_FaultResetSetPhase(
        feedback_pair_fresh_zero ?
        DRIVE_FAULT_RESET_PHASE_READ_STATUS :
        DRIVE_FAULT_RESET_PHASE_PRECHECK_ZERO_FEEDBACK,
        now_ms);
    return true;
}

#ifdef AMR_HOST_TEST
bool Drive_TestFaultResetActive(void)
{
    return s_drive.fault_reset.transaction.active;
}
#endif

#ifdef AMR_HOST_TEST
uint32_t Drive_TestFaultResetPhase(void)
{
    return (uint32_t)s_drive.fault_reset.transaction.phase;
}
#endif

#ifdef AMR_HOST_TEST
uint32_t Drive_TestFaultResetFailureReason(void)
{
    return (uint32_t)s_drive.fault_reset.transaction.failure_reason;
}
#endif

#ifdef AMR_HOST_TEST
uint32_t Drive_TestFaultResetGeneration(void)
{
    return s_drive.fault_reset.transaction.generation;
}
#endif
