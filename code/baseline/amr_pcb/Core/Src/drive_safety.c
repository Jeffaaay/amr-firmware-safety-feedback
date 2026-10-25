/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : drive_safety.c
  * @brief          : Private Drive module implementation.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "drive.h"
#include "app_config.h"
#include "canopen.h"
#include "diagnostics.h"
#include "drive_internal.h"
#include "drive_target.h"
#include "drive_lift_stop_diag.h"
#include "wheel_pdo_config.h"
#include "wheel_rpdo_tx.h"
#include "jetson_tx.h"
#include "main.h"
#include "motion_control.h"
#include "odom.h"
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

bool Drive_BootInProgress(void)
{
    return (s_drive.boot.state == DRIVE_BOOT_FIRST_ATTEMPT) ||
           (s_drive.boot.state == DRIVE_BOOT_WAIT_RETRY) ||
           (s_drive.boot.state == DRIVE_BOOT_SECOND_ATTEMPT);
}

static bool Drive_BootStatusDisabled(uint16_t statusword)
{
    /* Defined DS402 non-fault, non-operation-enabled states. Not-ready (0)
     * and fault-reaction are not proof that shutdown completed. */
    uint16_t state = Statusword_Ds402State(statusword);
    return ((statusword & 0x004FU) == 0x0040U) ||
           (state == 0x0021U) || (state == 0x0023U);
}

static DriveSafetyFailureReason_t Drive_BootKnownFailure(uint8_t *node)
{
    FDCAN_ProtocolStatusTypeDef bus = {0};
    CanopenBootSdoEvidence_t evidence = Canopen_GetBootSdoEvidence();
    CanopenRuntimeDiagnostics_t diag = Canopen_GetRuntimeDiagnostics();
    *node = 0U;
    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &bus) != HAL_OK) {
        return DRIVE_SAFETY_FAILURE_RUNTIME_CAN;
    }
    if (bus.BusOff != 0U) {
        s_drive.boot.bus_off = true;
        return DRIVE_SAFETY_FAILURE_RUNTIME_CAN;
    }
    if (evidence.hard_fault_reason != CANOPEN_BOOT_SDO_FAULT_NONE) {
        *node = evidence.node_id;
        if (evidence.hard_fault_reason == CANOPEN_BOOT_SDO_FAULT_STATUS) {
            Drive_RecordStatusword(*node, evidence.statusword, HAL_GetTick(),
                                    s_drive.evidence.safety_generation);
            return DRIVE_SAFETY_FAILURE_STATUS_FAULT;
        }
        if (evidence.hard_fault_reason == CANOPEN_BOOT_SDO_FAULT_ABORT) {
            s_drive.safety.transition.last_abort_code = evidence.abort_code;
            return DRIVE_SAFETY_FAILURE_SDO_ABORT;
        }
        if (evidence.hard_fault_reason == CANOPEN_BOOT_SDO_FAULT_EMCY) {
            return DRIVE_SAFETY_FAILURE_NODE_FAULT;
        }
        return DRIVE_SAFETY_FAILURE_SDO_PROTOCOL;
    }
    /* Resume can dispatch new events after the normal loop's fault consumers
     * have run. Inspect sticky pending flags without consuming or clearing them. */
    if (Canopen_HeartbeatFaultPending()) {
        *node = diag.heartbeat_fault_node;
        return DRIVE_SAFETY_FAILURE_HEARTBEAT;
    }
    if (Canopen_WheelRuntimeFaultPending()) {
        *node = diag.last_wheel_target_failure_node;
        return DRIVE_SAFETY_FAILURE_RUNTIME_CAN;
    }
    if (Canopen_WheelFeedbackContinuityFaultPending()) {
        return DRIVE_SAFETY_FAILURE_TPDO_FEEDBACK;
    }
    for (uint8_t id = LEFT_NODE_ID; id <= RIGHT_NODE_ID; ++id) {
        if (diag.last_emcy_code[id] != 0U) {
            *node = id;
            return DRIVE_SAFETY_FAILURE_NODE_FAULT;
        }
    }
    if ((s_drive.statusword.left_statusword_valid &&
         Statusword_HasFault(s_drive.statusword.left_statusword)) ||
        (s_drive.statusword.right_statusword_valid &&
         Statusword_HasFault(s_drive.statusword.right_statusword))) {
        *node = (s_drive.statusword.left_statusword_valid &&
                 Statusword_HasFault(s_drive.statusword.left_statusword)) ?
                LEFT_NODE_ID : RIGHT_NODE_ID;
        return DRIVE_SAFETY_FAILURE_STATUS_FAULT;
    }
    return DRIVE_SAFETY_FAILURE_NONE;
}

static void Drive_BootLogFailure(DriveSafetyFailureReason_t reason,
                                 const char *classification)
{
    (void)JetsonTx_Printf(JETSON_TX_PRIORITY_CRITICAL,
        "DRIVE_BOOT attempt=%u node=%u result=%s bus_off=%u final=%s\r\n",
        (unsigned)(s_drive.boot.retries + 1U),
        (unsigned)s_drive.safety.transition.failure_node_id,
        Drive_SafetyFailureName(reason), s_drive.boot.bus_off ? 1U : 0U,
        classification);
    (void)JetsonTx_Printf(JETSON_TX_PRIORITY_CRITICAL,
        "DRIVE_BOOT sw_left=%04X valid_left=%u sw_right=%04X valid_right=%u abort=%08lX\r\n",
        (unsigned)s_drive.statusword.left_statusword,
        s_drive.statusword.left_statusword_valid ? 1U : 0U,
        (unsigned)s_drive.statusword.right_statusword,
        s_drive.statusword.right_statusword_valid ? 1U : 0U,
        (unsigned long)s_drive.safety.transition.last_abort_code);
}

static bool Drive_BootTryWait(DriveSafetyFailureReason_t reason, uint32_t now_ms)
{
    if ((s_drive.boot.state != DRIVE_BOOT_FIRST_ATTEMPT) ||
        (s_drive.core.state == DRIVE_STATE_FAULT) ||
        (s_drive.safety.transition.type != DRIVE_SAFETY_INIT_DISABLE) ||
        !s_drive.safety.transition.active || !s_drive.boot.saw_timeout ||
        s_drive.boot.saw_nondisabled_status ||
        (s_drive.boot.retries >= DRIVE_BOOT_MAX_RETRIES) ||
        ((reason != DRIVE_SAFETY_FAILURE_SDO_TIMEOUT) &&
         (reason != DRIVE_SAFETY_FAILURE_DISABLE_CONFIRM))) {
        return false;
    }
    s_drive.safety.transition.failure_node_id = s_drive.boot.timeout_node;
    Drive_BootLogFailure(DRIVE_SAFETY_FAILURE_SDO_TIMEOUT, "RETRY_ONCE");
    Canopen_BootSdoFence(now_ms);
    s_drive.safety.transition.active = false;
    s_drive.safety.transition.request_pending = false;
    s_drive.boot.state = DRIVE_BOOT_WAIT_RETRY;
    s_drive.boot.retry_started_ms = now_ms;
    DriveCore_SetSoftwareEnabled(false);
    DriveCore_SetHardwareEnableConfirmed(false);
    DriveCore_SetHardwareDisableConfirmed(false);
    Drive_ClearMotionCommandStateOnly();
    Drive_InvalidateActuatorBaseline();
    Drive_InvalidateZeroSpeedEvidence();
    Drive_ResetYawRateControl(true);
    memset(&s_drive.statusword, 0, sizeof(s_drive.statusword));
    DriveCore_SetState(DRIVE_STATE_INIT);
    (void)JetsonTx_Printf(JETSON_TX_PRIORITY_STATUS,
        "DRIVE_BOOT state=WAIT_RETRY delay_ms=%lu\r\n",
        (unsigned long)DRIVE_BOOT_RETRY_DELAY_MS);
    return true;
}

void Drive_BootService(uint32_t now_ms)
{
    uint8_t node;
    DriveSafetyFailureReason_t failure;
    if (!Drive_BootInProgress()) { return; }
    Canopen_BootSdoBegin();
    failure = Drive_BootKnownFailure(&node);
    if (failure != DRIVE_SAFETY_FAILURE_NONE) {
        s_drive.safety.transition.type = DRIVE_SAFETY_INIT_DISABLE;
        s_drive.safety.transition.failure_node_id = node;
        Drive_SafetyEscalate(failure, now_ms);
        return;
    }
    if (s_drive.boot.state == DRIVE_BOOT_WAIT_RETRY) {
        bool resumed;

        if ((uint32_t)(now_ms - s_drive.boot.retry_started_ms) <
            DRIVE_BOOT_RETRY_DELAY_MS) { return; }
        resumed = Canopen_BootSdoResume(now_ms);
        /* Resume dispatched queued frames; do not rebaseline over new faults. */
        failure = Drive_BootKnownFailure(&node);
        if (failure != DRIVE_SAFETY_FAILURE_NONE) {
            s_drive.safety.transition.failure_node_id = node;
            Drive_SafetyEscalate(failure, now_ms);
            return;
        }
        if (!resumed) { return; }
        s_drive.boot.retries++;
        s_drive.boot.state = DRIVE_BOOT_SECOND_ATTEMPT;
        s_drive.boot.started = false;
    }
    if (!s_drive.boot.started && !s_drive.safety.transition.active &&
        !s_drive.fault_reset.transaction.active) {
        s_drive.boot.started = true;
        s_drive.boot.saw_timeout = false;
        s_drive.boot.saw_nondisabled_status = false;
        memset(&s_drive.statusword, 0, sizeof(s_drive.statusword));
        Drive_InvalidateZeroSpeedEvidence();
        (void)JetsonTx_Printf(JETSON_TX_PRIORITY_STATUS,
            "DRIVE_BOOT attempt=%u state=START\r\n",
            (unsigned)(s_drive.boot.retries + 1U));
        if (!Drive_StartSafetyTransition(DRIVE_SAFETY_INIT_DISABLE, false, now_ms)) {
            s_drive.safety.transition.type = DRIVE_SAFETY_INIT_DISABLE;
            Drive_SafetyEscalate(DRIVE_SAFETY_FAILURE_SDO_PROTOCOL, now_ms);
        }
    }
}

static bool Drive_DisabledEvidenceReady(uint32_t now_ms)
{
    return s_drive.statusword.left_statusword_valid &&
        s_drive.statusword.right_statusword_valid &&
        (s_drive.statusword.left_statusword_generation == s_drive.safety.transition.generation) &&
        (s_drive.statusword.right_statusword_generation == s_drive.safety.transition.generation) &&
        ((s_drive.safety.transition.type != DRIVE_SAFETY_INIT_DISABLE) ||
         (Drive_BootStatusDisabled(s_drive.statusword.left_statusword) &&
          Drive_BootStatusDisabled(s_drive.statusword.right_statusword))) &&
        ((uint32_t)(now_ms - s_drive.statusword.left_statusword_timestamp_ms) <= DRIVE_WHEEL_FEEDBACK_STALE_MS) &&
        ((uint32_t)(now_ms - s_drive.statusword.right_statusword_timestamp_ms) <= DRIVE_WHEEL_FEEDBACK_STALE_MS);
}

void Drive_InvalidateZeroSpeedEvidence(void)
{
    s_drive.evidence.zero_speed.valid = false;
    s_drive.evidence.zero_speed.reason = DRIVE_ZERO_EVIDENCE_NONE;
}

uint32_t Drive_BeginSafetyGeneration(void)
{
    DriveTarget_Cancel(HAL_GetTick());
    s_drive.evidence.safety_generation++;
    if (s_drive.evidence.safety_generation == 0U) {
        s_drive.evidence.safety_generation++;
    }
    Drive_InvalidateZeroSpeedEvidence();
    return s_drive.evidence.safety_generation;
}

static bool Drive_FeedbackPairIsFresh(uint32_t now_ms)
{
    DriveWheelFeedbackPair_t pair;

    return Drive_GetAuthoritativeWheelFeedbackPair(now_ms, &pair);
}

bool Drive_FeedbackPairIsFreshZero(uint32_t now_ms)
{
    DriveWheelFeedbackPair_t pair;

    return Drive_FeedbackPairIsFresh(now_ms) &&
           Drive_GetAuthoritativeWheelFeedbackPair(now_ms, &pair) &&
           (fabsf(pair.normalized_left_velocity_mm_s) <=
            DRIVE_STOP_WHEEL_THRESHOLD_MM_S) &&
           (fabsf(pair.normalized_right_velocity_mm_s) <=
            DRIVE_STOP_WHEEL_THRESHOLD_MM_S);
}

const char *Drive_StableZeroStateName(void)
{
    switch (s_drive.stable_zero.state) {
        case DRIVE_STABLE_ZERO_SETTLING: return "SETTLING";
        case DRIVE_STABLE_ZERO_CONFIRMING: return "CONFIRMING";
        case DRIVE_STABLE_ZERO_CONFIRMED: return "CONFIRMED";
        case DRIVE_STABLE_ZERO_IDLE:
        default: return "IDLE";
    }
}

const char *Drive_StableZeroRejectName(void)
{
    switch (s_drive.stable_zero.reject_reason) {
        case DRIVE_STABLE_ZERO_REJECT_PAIR_INVALID: return "PAIR_INVALID";
        case DRIVE_STABLE_ZERO_REJECT_PAIR_NOT_NEW: return "PAIR_NOT_NEW";
        case DRIVE_STABLE_ZERO_REJECT_LEFT_SPEED: return "LEFT_SPEED";
        case DRIVE_STABLE_ZERO_REJECT_RIGHT_SPEED: return "RIGHT_SPEED";
        case DRIVE_STABLE_ZERO_REJECT_PAIR_SKEW: return "PAIR_SKEW";
        case DRIVE_STABLE_ZERO_REJECT_TIMEOUT: return "TIMEOUT";
        case DRIVE_STABLE_ZERO_REJECT_NONE:
        default: return "NONE";
    }
}

static void Drive_StableZeroSetReject(
    DriveStableZeroRejectReason_t reason)
{
    s_drive.stable_zero.reject_reason = reason;
    s_drive.tpdo_active.last_stop_reject_reason =
        Drive_StableZeroRejectName();
    if (reason == DRIVE_STABLE_ZERO_REJECT_PAIR_NOT_NEW) {
        if (s_drive.tpdo_active.stop_pair_not_new_count != UINT32_MAX) {
            s_drive.tpdo_active.stop_pair_not_new_count++;
        }
    } else if ((reason == DRIVE_STABLE_ZERO_REJECT_PAIR_INVALID) ||
               (reason == DRIVE_STABLE_ZERO_REJECT_PAIR_SKEW)) {
        if (s_drive.tpdo_active.stop_pair_invalid_count != UINT32_MAX) {
            s_drive.tpdo_active.stop_pair_invalid_count++;
        }
    }
}

static void Drive_StableZeroUpdateElapsed(uint32_t now_ms)
{
    DriveStableZeroTracker_t *tracker = &s_drive.stable_zero;

    tracker->overall_elapsed_ms = (uint32_t)(now_ms - tracker->start_ms);
    if (tracker->state == DRIVE_STABLE_ZERO_SETTLING) {
        tracker->settle_elapsed_ms =
            (uint32_t)(now_ms - tracker->settle_start_ms);
    }
    tracker->stable_dwell_ms = (tracker->stable_sample_count != 0U) ?
        (uint32_t)(now_ms - tracker->stable_start_ms) : 0U;
}

static void Drive_StableZeroBreakConfirmation(
    uint32_t now_ms,
    DriveStableZeroRejectReason_t reason)
{
    DriveStableZeroTracker_t *tracker = &s_drive.stable_zero;

    tracker->stable_sample_count = 0U;
    tracker->stable_start_ms = 0U;
    tracker->stable_dwell_ms = 0U;
    if (tracker->settle_allowed &&
        (tracker->state == DRIVE_STABLE_ZERO_CONFIRMING)) {
        tracker->state = DRIVE_STABLE_ZERO_SETTLING;
        tracker->settle_start_ms = now_ms;
        tracker->settle_elapsed_ms = 0U;
    }
    Drive_StableZeroSetReject(reason);
}

void Drive_StableZeroStart(uint32_t now_ms, bool settle_allowed)
{
    DriveWheelFeedbackPair_t pair = {0};
    DriveStableZeroTracker_t *tracker = &s_drive.stable_zero;

    (void)Drive_GetAuthoritativeWheelFeedbackPair(now_ms, &pair);
    memset(tracker, 0, sizeof(*tracker));
    tracker->active = true;
    tracker->settle_allowed = settle_allowed;
    tracker->state = settle_allowed ?
        DRIVE_STABLE_ZERO_SETTLING : DRIVE_STABLE_ZERO_CONFIRMING;
    tracker->reject_reason = DRIVE_STABLE_ZERO_REJECT_PAIR_NOT_NEW;
    tracker->start_ms = now_ms;
    tracker->settle_start_ms = now_ms;
    tracker->confirm_start_ms = now_ms;
    tracker->baseline_pair_sequence = pair.pair_sequence;
    tracker->baseline_left_sequence = pair.left_node_sequence;
    tracker->baseline_right_sequence = pair.right_node_sequence;
    tracker->last_pair_sequence = pair.pair_sequence;
    tracker->last_left_sequence = pair.left_node_sequence;
    tracker->last_right_sequence = pair.right_node_sequence;
    tracker->last_skew_reject_count =
        s_drive.feedback.motion.rejected_skew_pair_count;
    tracker->left_velocity_mm_s = pair.normalized_left_velocity_mm_s;
    tracker->right_velocity_mm_s = pair.normalized_right_velocity_mm_s;
    tracker->pair_age_ms = pair.pair_age_ms;
    tracker->pair_skew_ms = pair.pair_skew_ms;
    s_drive.tpdo_active.last_stop_reject_reason = "PAIR_NOT_NEW";
}

void Drive_StableZeroMarkTimeout(uint32_t now_ms)
{
    if (s_drive.stable_zero.state == DRIVE_STABLE_ZERO_IDLE) {
        return;
    }
    Drive_StableZeroUpdateElapsed(now_ms);
    s_drive.stable_zero.active = false;
    Drive_StableZeroSetReject(DRIVE_STABLE_ZERO_REJECT_TIMEOUT);
}

DriveStableZeroResult_t Drive_StableZeroUpdate(uint32_t now_ms)
{
    DriveWheelFeedbackPair_t pair = {0};
    DriveStableZeroTracker_t *tracker = &s_drive.stable_zero;
    bool pair_valid;
    bool pair_new;
    bool left_zero;
    bool right_zero;

    if (!tracker->active) {
        return (tracker->state == DRIVE_STABLE_ZERO_CONFIRMED) ?
            DRIVE_STABLE_ZERO_COMPLETE : DRIVE_STABLE_ZERO_TIMED_OUT;
    }

    Drive_StableZeroUpdateElapsed(now_ms);
    if ((tracker->overall_elapsed_ms >
         DRIVE_NORMAL_ZERO_OVERALL_TIMEOUT_MS) ||
        ((tracker->state == DRIVE_STABLE_ZERO_SETTLING) &&
         (tracker->settle_elapsed_ms >
          DRIVE_NORMAL_ZERO_SETTLE_TIMEOUT_MS)) ||
        ((tracker->state == DRIVE_STABLE_ZERO_CONFIRMING) &&
         ((uint32_t)(now_ms - tracker->confirm_start_ms) >
          (tracker->settle_allowed ?
           DRIVE_NORMAL_ZERO_CONFIRM_TIMEOUT_MS :
           DRIVE_STOP_CONFIRM_TIMEOUT_MS)))) {
        Drive_StableZeroMarkTimeout(now_ms);
        return DRIVE_STABLE_ZERO_TIMED_OUT;
    }

    pair_valid = Drive_GetAuthoritativeWheelFeedbackPair(now_ms, &pair);
    tracker->left_velocity_mm_s = pair.normalized_left_velocity_mm_s;
    tracker->right_velocity_mm_s = pair.normalized_right_velocity_mm_s;
    tracker->pair_age_ms = pair.pair_age_ms;
    tracker->pair_skew_ms = pair.pair_skew_ms;

    if (s_drive.feedback.motion.rejected_skew_pair_count !=
        tracker->last_skew_reject_count) {
        uint32_t left_ms = s_drive.feedback.motion.left.timestamp_ms;
        uint32_t right_ms = s_drive.feedback.motion.right.timestamp_ms;

        tracker->last_skew_reject_count =
            s_drive.feedback.motion.rejected_skew_pair_count;
        tracker->pair_skew_ms = (left_ms >= right_ms) ?
            (left_ms - right_ms) : (right_ms - left_ms);
        Drive_StableZeroBreakConfirmation(
            now_ms, DRIVE_STABLE_ZERO_REJECT_PAIR_SKEW);
        return DRIVE_STABLE_ZERO_PENDING;
    }
    if (pair.pair_skew_ms > DRIVE_WHEEL_FEEDBACK_MAX_SKEW_MS) {
        Drive_StableZeroBreakConfirmation(
            now_ms, DRIVE_STABLE_ZERO_REJECT_PAIR_SKEW);
        return DRIVE_STABLE_ZERO_PENDING;
    }
    if (!pair_valid) {
        Drive_StableZeroBreakConfirmation(
            now_ms, DRIVE_STABLE_ZERO_REJECT_PAIR_INVALID);
        return DRIVE_STABLE_ZERO_PENDING;
    }
    if ((pair.left_node_sequence == tracker->baseline_left_sequence) ||
        (pair.right_node_sequence == tracker->baseline_right_sequence)) {
        Drive_StableZeroSetReject(DRIVE_STABLE_ZERO_REJECT_PAIR_NOT_NEW);
        return DRIVE_STABLE_ZERO_PENDING;
    }

    pair_new =
        (pair.pair_sequence != tracker->last_pair_sequence) &&
        (pair.left_node_sequence != tracker->last_left_sequence) &&
        (pair.right_node_sequence != tracker->last_right_sequence);
    if (!pair_new) {
        Drive_StableZeroSetReject(DRIVE_STABLE_ZERO_REJECT_PAIR_NOT_NEW);
        return DRIVE_STABLE_ZERO_PENDING;
    }

    tracker->last_pair_sequence = pair.pair_sequence;
    tracker->last_left_sequence = pair.left_node_sequence;
    tracker->last_right_sequence = pair.right_node_sequence;
    left_zero = fabsf(pair.normalized_left_velocity_mm_s) <=
        DRIVE_STOP_WHEEL_THRESHOLD_MM_S;
    right_zero = fabsf(pair.normalized_right_velocity_mm_s) <=
        DRIVE_STOP_WHEEL_THRESHOLD_MM_S;
    if (!left_zero || !right_zero) {
        tracker->stable_sample_count = 0U;
        tracker->stable_start_ms = 0U;
        tracker->stable_dwell_ms = 0U;
        if (tracker->settle_allowed &&
            (tracker->state == DRIVE_STABLE_ZERO_CONFIRMING)) {
            tracker->state = DRIVE_STABLE_ZERO_SETTLING;
            tracker->settle_start_ms = now_ms;
            tracker->settle_elapsed_ms = 0U;
        }
        Drive_StableZeroSetReject(!left_zero ?
            DRIVE_STABLE_ZERO_REJECT_LEFT_SPEED :
            DRIVE_STABLE_ZERO_REJECT_RIGHT_SPEED);
        return DRIVE_STABLE_ZERO_PENDING;
    }

    if (tracker->state == DRIVE_STABLE_ZERO_SETTLING) {
        tracker->settle_elapsed_ms =
            (uint32_t)(now_ms - tracker->settle_start_ms);
        tracker->state = DRIVE_STABLE_ZERO_CONFIRMING;
        tracker->confirm_start_ms = now_ms;
    }
    if (tracker->stable_sample_count == 0U) {
        tracker->stable_start_ms = now_ms;
    }
    if (tracker->stable_sample_count != UINT32_MAX) {
        tracker->stable_sample_count++;
    }
    tracker->stable_dwell_ms =
        (uint32_t)(now_ms - tracker->stable_start_ms);
    Drive_StableZeroSetReject(DRIVE_STABLE_ZERO_REJECT_NONE);
    if ((tracker->stable_sample_count >=
         DRIVE_STOP_ZERO_STABLE_SAMPLE_COUNT) &&
        (tracker->stable_dwell_ms >= DRIVE_STOP_ZERO_DWELL_MS)) {
        if (s_drive.evidence.post_disable_zero_proof_required) {
            if ((tracker->last_pair_sequence ==
                 s_drive.evidence.post_disable_baseline_pair_sequence) ||
                (tracker->last_left_sequence ==
                 s_drive.evidence.post_disable_baseline_left_sequence) ||
                (tracker->last_right_sequence ==
                 s_drive.evidence.post_disable_baseline_right_sequence)) {
                Drive_StableZeroSetReject(
                    DRIVE_STABLE_ZERO_REJECT_PAIR_NOT_NEW);
                return DRIVE_STABLE_ZERO_PENDING;
            }
            s_drive.evidence.post_disable_zero_proof_required = false;
        }
        tracker->active = false;
        tracker->state = DRIVE_STABLE_ZERO_CONFIRMED;
        return DRIVE_STABLE_ZERO_COMPLETE;
    }
    return DRIVE_STABLE_ZERO_PENDING;
}

bool Drive_StopHasStableMeasuredZero(uint32_t now_ms)
{
    return Drive_StableZeroUpdate(now_ms) == DRIVE_STABLE_ZERO_COMPLETE;
}

void Drive_RecordZeroSpeedEvidence(
    uint32_t generation,
    uint32_t now_ms,
    DriveZeroEvidenceReason_t reason)
{
    if ((generation == 0U) ||
        (generation != s_drive.evidence.safety_generation) ||
        (reason == DRIVE_ZERO_EVIDENCE_NONE) ||
        s_drive.evidence.post_disable_zero_proof_required ||
        !Drive_FeedbackPairIsFreshZero(now_ms)) {
        Drive_InvalidateZeroSpeedEvidence();
        return;
    }

    s_drive.evidence.zero_speed.valid = true;
    s_drive.evidence.zero_speed.generation = generation;
    s_drive.evidence.zero_speed.left_sequence =
        s_drive.feedback.chassis.left.sequence;
    s_drive.evidence.zero_speed.right_sequence =
        s_drive.feedback.chassis.right.sequence;
    s_drive.evidence.zero_speed.left_timestamp_ms =
        s_drive.feedback.chassis.left.timestamp_ms;
    s_drive.evidence.zero_speed.right_timestamp_ms =
        s_drive.feedback.chassis.right.timestamp_ms;
    s_drive.evidence.zero_speed.reason = reason;
}

bool Drive_ZeroSpeedEvidenceIsCurrent(uint32_t now_ms)
{
    return s_drive.evidence.zero_speed.valid &&
           !s_drive.evidence.post_disable_zero_proof_required &&
           (s_drive.evidence.zero_speed.generation == s_drive.evidence.safety_generation) &&
           (s_drive.evidence.zero_speed.left_sequence ==
            s_drive.feedback.chassis.left.sequence) &&
           (s_drive.evidence.zero_speed.right_sequence ==
            s_drive.feedback.chassis.right.sequence) &&
           (s_drive.evidence.zero_speed.left_timestamp_ms ==
            s_drive.feedback.chassis.left.timestamp_ms) &&
           (s_drive.evidence.zero_speed.right_timestamp_ms ==
            s_drive.feedback.chassis.right.timestamp_ms) &&
           !Canopen_WheelRuntimeFaultPending() &&
           !Canopen_WheelFeedbackContinuityFaultPending() &&
           Drive_FeedbackPairIsFreshZero(now_ms);
}

static bool Drive_WheelStatuswordBitsConfirmDisabled(void)
{
    const uint16_t unsafe_mask = (1U << 2) | (1U << 3);

    return s_drive.statusword.left_statusword_valid && s_drive.statusword.right_statusword_valid &&
           ((s_drive.statusword.left_statusword & unsafe_mask) == 0U) &&
           ((s_drive.statusword.right_statusword & unsafe_mask) == 0U);
}

bool Drive_StatuswordEvidenceIsFresh(bool valid,
                                            uint32_t timestamp_ms,
                                            uint32_t generation,
                                            uint32_t now_ms)
{
    return valid && (generation == s_drive.evidence.safety_generation) &&
           ((uint32_t)(now_ms - timestamp_ms) <=
            DRIVE_LIFT_STATUSWORD_FRESH_MS);
}

static bool Drive_WheelStatuswordsConfirmDisabled(uint32_t now_ms)
{
    return Drive_WheelStatuswordBitsConfirmDisabled() &&
           (s_drive.statusword.left_statusword_sequence != 0U) &&
           (s_drive.statusword.right_statusword_sequence != 0U) &&
           Drive_StatuswordEvidenceIsFresh(
               s_drive.statusword.left_statusword_valid,
               s_drive.statusword.left_statusword_timestamp_ms,
               s_drive.statusword.left_statusword_generation,
               now_ms) &&
           Drive_StatuswordEvidenceIsFresh(
               s_drive.statusword.right_statusword_valid,
               s_drive.statusword.right_statusword_timestamp_ms,
               s_drive.statusword.right_statusword_generation,
               now_ms);
}

void Drive_MaintainZeroSpeedEvidence(uint32_t now_ms)
{
    DriveWheelFeedbackPair_t pair;
    bool pair_valid = Drive_GetAuthoritativeWheelFeedbackPair(now_ms, &pair);
    bool latest_left_nonzero = pair_valid &&
        (fabsf(pair.normalized_left_velocity_mm_s) >
         DRIVE_STOP_WHEEL_THRESHOLD_MM_S);
    bool latest_right_nonzero = pair_valid &&
        (fabsf(pair.normalized_right_velocity_mm_s) >
         DRIVE_STOP_WHEEL_THRESHOLD_MM_S);

    if (!s_drive.evidence.zero_speed.valid) {
        /* Maintenance can extend an already proven stop, never recreate one
         * from a single zero frame after invalidation. The requesting safety
         * transaction must run the unchanged 3-pair / 100 ms tracker again. */
        return;
    }
    if (latest_left_nonzero || latest_right_nonzero ||
        !Drive_FeedbackPairIsFreshZero(now_ms) ||
        Canopen_WheelRuntimeFaultPending() ||
        Canopen_WheelFeedbackContinuityFaultPending()) {
        Drive_InvalidateZeroSpeedEvidence();
        return;
    }

    if ((s_drive.feedback.chassis.left.sequence !=
         s_drive.evidence.zero_speed.left_sequence) ||
        (s_drive.feedback.chassis.right.sequence !=
         s_drive.evidence.zero_speed.right_sequence)) {
        Drive_RecordZeroSpeedEvidence(s_drive.evidence.zero_speed.generation,
                                      now_ms,
                                      s_drive.evidence.zero_speed.reason);
    }
}

void Drive_InvalidateActuatorBaseline(void)
{
    Motion_ActuatorStateInvalidate(&s_drive.control.actuator.state);
    Drive_InvalidateZeroSpeedEvidence();
}

bool Statusword_HasFault(uint16_t statusword)
{
    return (statusword & (1U << 3)) != 0U;
}

bool Statusword_IsOperationEnabled(uint16_t statusword)
{
    return (statusword & (1U << 2)) != 0U;
}

uint16_t Statusword_Ds402State(uint16_t statusword)
{
    return statusword & 0x006FU;
}

static bool Statusword_IsDs402State(uint16_t statusword,
                                    uint16_t expected_state)
{
    return Statusword_Ds402State(statusword) == expected_state;
}

DriveEmcyDelta_t Drive_ClassifyEmcyDelta(
    const CanopenRuntimeDiagnostics_t *diag,
    uint8_t node_id,
    uint32_t recorded_sequence)
{
    uint32_t delta = diag->emcy_sequence[node_id] - recorded_sequence;

    if (delta == 0U) {
        return DRIVE_EMCY_DELTA_NONE;
    }
    if ((delta == 1U) && (diag->last_emcy_code[node_id] == 0x0000U)) {
        return DRIVE_EMCY_DELTA_RESET_NOTIFICATION;
    }
    return DRIVE_EMCY_DELTA_FAULT;
}

static void Drive_ModeConfirmInitialize(uint8_t node_id)
{
    s_drive.safety.transition.requested_mode = DRIVE_KINCO_VELOCITY_MODE;
    s_drive.safety.transition.last_mode_readback = 0;
    s_drive.safety.transition.mode_readback_valid = false;
    s_drive.safety.transition.mode_confirm_attempt = 0U;
    s_drive.safety.transition.mode_confirm_start_tick = 0U;
    s_drive.safety.transition.mode_confirm_next_poll_tick = 0U;
    s_drive.safety.transition.mode_confirm_elapsed_ms = 0U;
    s_drive.safety.transition.mode_confirm_last_sdo_result =
        DRIVE_MODE_CONFIRM_SDO_NONE;
    s_drive.safety.transition.mode_confirm_last_abort_code = 0U;
    s_drive.safety.transition.mode_confirm_node_id = node_id;
}

void Drive_EnableNodeEvidenceInitialize(uint8_t node_id)
{
    Drive_ModeConfirmInitialize(node_id);
    s_drive.safety.transition.enable_0006_statusword = 0U;
    s_drive.safety.transition.enable_0007_statusword = 0U;
    s_drive.safety.transition.enable_000f_statusword = 0U;
    s_drive.safety.transition.enable_0006_statusword_valid = false;
    s_drive.safety.transition.enable_0007_statusword_valid = false;
    s_drive.safety.transition.enable_000f_statusword_valid = false;
}

static void Drive_ModeConfirmStartWindow(uint32_t now_ms)
{
    s_drive.safety.transition.mode_confirm_start_tick = now_ms;
    s_drive.safety.transition.mode_confirm_next_poll_tick = now_ms;
    s_drive.safety.transition.mode_confirm_elapsed_ms = 0U;
    s_drive.safety.transition.next_action_ms = now_ms;
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_DEBUG,
        "DRIVE_MODE_REQUEST state=START node=%u obj=6060 req=%d elapsed=0\r\n",
        (unsigned)s_drive.safety.transition.mode_confirm_node_id,
        (int)s_drive.safety.transition.requested_mode);
}

void Drive_RecordStatusword(uint8_t node_id,
                                   uint16_t statusword,
                                   uint32_t now_ms,
                                   uint32_t generation)
{
    if (node_id == LEFT_NODE_ID) {
        s_drive.statusword.left_statusword = statusword;
        s_drive.statusword.left_statusword_valid = true;
        s_drive.statusword.left_statusword_timestamp_ms = now_ms;
        s_drive.statusword.left_statusword_sequence++;
        if (s_drive.statusword.left_statusword_sequence == 0U) {
            s_drive.statusword.left_statusword_sequence++;
        }
        s_drive.statusword.left_statusword_generation = generation;
    } else if (node_id == RIGHT_NODE_ID) {
        s_drive.statusword.right_statusword = statusword;
        s_drive.statusword.right_statusword_valid = true;
        s_drive.statusword.right_statusword_timestamp_ms = now_ms;
        s_drive.statusword.right_statusword_sequence++;
        if (s_drive.statusword.right_statusword_sequence == 0U) {
            s_drive.statusword.right_statusword_sequence++;
        }
        s_drive.statusword.right_statusword_generation = generation;
    }
}

bool Drive_SafetyIsRollback(void)
{
    return s_drive.safety.transition.active &&
        ((s_drive.safety.transition.type == DRIVE_SAFETY_ENABLE_ROLLBACK) ||
         (s_drive.safety.transition.type == DRIVE_SAFETY_DISABLE_ROLLBACK));
}

bool Drive_SafetyNeedsVelocityPolling(void)
{
    if (!s_drive.safety.transition.active || Drive_SafetyIsRollback()) {
        return false;
    }
    return (s_drive.safety.transition.phase == DRIVE_SAFETY_PHASE_ZERO_WAIT_ACK) ||
           (s_drive.safety.transition.phase ==
            DRIVE_SAFETY_PHASE_ZERO_WAIT_SETTLE) ||
           (s_drive.safety.transition.phase ==
            DRIVE_SAFETY_PHASE_ZERO_WAIT_MEASURED);
}

static CanopenSdoOwner_t Drive_SafetySdoOwner(void)
{
    if (Drive_SafetyIsRollback()) {
        return CANOPEN_SDO_OWNER_SAFETY_ROLLBACK;
    }
    if (s_drive.safety.transition.type == DRIVE_SAFETY_ENABLE) {
        return CANOPEN_SDO_OWNER_SAFETY_ENABLE;
    }
    return CANOPEN_SDO_OWNER_SAFETY_CONFIRM;
}

void Drive_SafetySetPhase(DriveSafetyPhase_t phase,
                                 uint32_t now_ms)
{
    s_drive.safety.transition.phase = phase;
    s_drive.safety.transition.phase_start_ms = now_ms;
    s_drive.safety.transition.action_attempts = 0U;
    s_drive.safety.transition.request_pending = false;
    s_drive.safety.transition.next_action_ms = now_ms;
}

static DriveSafetySdoActionResult_t Drive_SafetyServiceSdoAction(
    uint32_t now_ms,
    uint8_t node_id,
    DriveSafetySdoAction_t action,
    uint16_t index,
    uint8_t subindex,
    uint32_t write_value,
    uint32_t *read_value)
{
    CanopenSdoOwner_t owner = Drive_SafetySdoOwner();

    if ((int32_t)(now_ms - s_drive.safety.transition.next_action_ms) < 0) {
        return DRIVE_SDO_ACTION_WAIT;
    }
    if (!s_drive.safety.transition.request_pending) {
        CanopenAsyncStartResult_t start_result;

        switch (action) {
            case DRIVE_SDO_ACTION_READ_U16:
                start_result = Canopen_AsyncReadU16(
                    node_id, index, subindex, owner, now_ms,
                    DRIVE_SAFETY_SDO_TIMEOUT_MS);
                break;
            case DRIVE_SDO_ACTION_READ_U8:
                start_result = Canopen_AsyncReadU8(
                    node_id, index, subindex, owner, now_ms,
                    DRIVE_SAFETY_SDO_TIMEOUT_MS);
                break;
            case DRIVE_SDO_ACTION_READ_U32:
                start_result = Canopen_AsyncReadU32(
                    node_id, index, subindex, owner, now_ms,
                    DRIVE_SAFETY_SDO_TIMEOUT_MS);
                break;
            case DRIVE_SDO_ACTION_READ_I8:
                start_result = Canopen_AsyncReadI8(
                    node_id, index, subindex, owner, now_ms,
                    DRIVE_SAFETY_SDO_TIMEOUT_MS);
                break;
            case DRIVE_SDO_ACTION_WRITE_U8:
                start_result = Canopen_AsyncWriteU8(
                    node_id, index, subindex, (uint8_t)write_value,
                    owner, now_ms, DRIVE_SAFETY_SDO_TIMEOUT_MS);
                break;
            case DRIVE_SDO_ACTION_WRITE_U16:
                start_result = Canopen_AsyncWriteU16(
                    node_id, index, subindex, (uint16_t)write_value,
                    owner, now_ms, DRIVE_SAFETY_SDO_TIMEOUT_MS);
                break;
            case DRIVE_SDO_ACTION_WRITE_U32:
                start_result = Canopen_AsyncWriteU32(
                    node_id, index, subindex, write_value,
                    owner, now_ms, DRIVE_SAFETY_SDO_TIMEOUT_MS);
                break;
            case DRIVE_SDO_ACTION_WRITE_I8:
                start_result = Canopen_AsyncWriteI8(
                    node_id, index, subindex, (int8_t)write_value,
                    owner, now_ms, DRIVE_SAFETY_SDO_TIMEOUT_MS);
                break;
            case DRIVE_SDO_ACTION_WRITE_I32:
            default:
                start_result = Canopen_AsyncWriteI32(
                    node_id, index, subindex, (int32_t)write_value,
                    owner, now_ms, DRIVE_SAFETY_SDO_TIMEOUT_MS);
                break;
        }
        if (start_result == CANOPEN_ASYNC_START_BUSY) {
            return DRIVE_SDO_ACTION_WAIT;
        }
        if (start_result != CANOPEN_ASYNC_START_OK) {
            return DRIVE_SDO_ACTION_ERROR;
        }
        s_drive.safety.transition.request_pending = true;
        s_drive.safety.transition.pending_node_id = node_id;
        s_drive.safety.transition.pending_owner = owner;
        s_drive.safety.transition.action_attempts++;
        return DRIVE_SDO_ACTION_WAIT;
    }

    if ((s_drive.safety.transition.pending_node_id != node_id) ||
        (s_drive.safety.transition.pending_owner != owner)) {
        return DRIVE_SDO_ACTION_ERROR;
    }
    {
        uint32_t value = 0U;
        uint32_t abort_code = 0U;
        uint32_t rx_ms = 0U;
        CanopenAsyncResult_t poll_result = Canopen_AsyncPollWithTimestamp(
            node_id, owner, now_ms, &value, &abort_code, &rx_ms);

        if (poll_result == CANOPEN_ASYNC_RESULT_PENDING) {
            return DRIVE_SDO_ACTION_WAIT;
        }
        s_drive.safety.transition.request_pending = false;
        if (poll_result == CANOPEN_ASYNC_RESULT_SUCCESS) {
            s_drive.safety.transition.last_sdo_rx_ms = rx_ms;
            if (read_value != NULL) {
                *read_value = value;
            }
            return DRIVE_SDO_ACTION_SUCCESS;
        }
        if (poll_result == CANOPEN_ASYNC_RESULT_TIMEOUT) {
            s_drive.safety.transition.transient_timeout_count++;
            if (s_drive.safety.transition.type == DRIVE_SAFETY_INIT_DISABLE) {
                s_drive.boot.saw_timeout = true;
                s_drive.boot.timeout_node = node_id;
            }
            Drive_InvalidateZeroSpeedEvidence();
            s_drive.safety.transition.next_action_ms =
                now_ms + DRIVE_SAFETY_RETRY_MS;
            return DRIVE_SDO_ACTION_TIMEOUT;
        }
        if (poll_result == CANOPEN_ASYNC_RESULT_ABORT) {
            s_drive.safety.transition.last_abort_code = abort_code;
            return DRIVE_SDO_ACTION_ABORT;
        }
        return DRIVE_SDO_ACTION_ERROR;
    }
}

static bool Drive_SafetyStatuswordNeedsRefresh(uint16_t statusword,
                                              uint32_t now_ms)
{
    /* A completed SDO can wait while another cooperative client blocks.
     * Keep its RX age; an old fault still faults, but an old safe state must
     * be re-read within the unchanged transition/confirmation deadlines. */
    if (!Statusword_HasFault(statusword) &&
        ((uint32_t)(now_ms - s_drive.safety.transition.last_sdo_rx_ms) >
         DRIVE_WHEEL_FEEDBACK_STALE_MS)) {
        s_drive.safety.transition.next_action_ms =
            now_ms + DRIVE_SAFETY_RETRY_MS;
        return true;
    }
    return false;
}

static void Drive_SafetyFinishFault(DriveSafetyFailureReason_t reason)
{
    if (s_drive.safety.transition.request_pending) {
        Canopen_CancelAsyncOwner(s_drive.safety.transition.pending_owner,
                                 HAL_GetTick());
    }
    s_drive.safety.transition.active = false;
    s_drive.safety.transition.request_pending = false;
    DriveCore_SetSoftwareEnabled(false);
    DriveCore_SetHardwareEnableConfirmed(false);
    DriveCore_SetTrackingStopLatched(true);
    DriveCore_SetMotionInhibitLatched(true);
    DriveCore_SetTrackingStopIgnoreLogged(false);
    Drive_ClearMotionCommandStateOnly();
    Drive_InvalidateActuatorBaseline();
    Drive_InvalidateZeroSpeedEvidence();
    Drive_ResetYawRateControl(true);
    DriveCore_SetState(DRIVE_STATE_FAULT);
    JetsonTx_Printf(JETSON_TX_PRIORITY_CRITICAL,
                "DRIVE safety transition failed reason=%s abort=0x%08lX transient_to=%lu\r\n",
                Drive_SafetyFailureName(reason),
                (unsigned long)s_drive.safety.transition.last_abort_code,
                (unsigned long)s_drive.safety.transition.transient_timeout_count);
}

void Drive_SafetyBeginRollback(
    DriveSafetyTransitionType_t rollback_type,
    DriveSafetyFailureReason_t reason,
    uint32_t now_ms)
{
    bool explicit_recovery = s_drive.safety.transition.explicit_recovery;
    bool had_tracking_latch = s_drive.safety.transition.had_tracking_latch ||
                              s_tracking_stop_latched;
    bool had_disable_latch = s_drive.safety.transition.had_disable_latch ||
                             s_disable_motion_inhibit_latched;
    uint8_t failure_node_id = s_drive.safety.transition.failure_node_id;
    uint32_t last_abort_code = s_drive.safety.transition.last_abort_code;
    uint32_t transient_timeout_count =
        s_drive.safety.transition.transient_timeout_count;
    uint32_t mode_confirm_attempt =
        s_drive.safety.transition.mode_confirm_attempt;
    uint32_t mode_confirm_start_tick =
        s_drive.safety.transition.mode_confirm_start_tick;
    uint32_t mode_confirm_next_poll_tick =
        s_drive.safety.transition.mode_confirm_next_poll_tick;
    uint32_t mode_confirm_elapsed_ms =
        s_drive.safety.transition.mode_confirm_elapsed_ms;
    uint32_t mode_confirm_last_abort_code =
        s_drive.safety.transition.mode_confirm_last_abort_code;
    uint16_t enable_0006_statusword =
        s_drive.safety.transition.enable_0006_statusword;
    uint16_t enable_0007_statusword =
        s_drive.safety.transition.enable_0007_statusword;
    uint16_t enable_000f_statusword =
        s_drive.safety.transition.enable_000f_statusword;
    int8_t requested_mode = s_drive.safety.transition.requested_mode;
    int8_t last_mode_readback =
        s_drive.safety.transition.last_mode_readback;
    uint8_t mode_confirm_node_id =
        s_drive.safety.transition.mode_confirm_node_id;
    DriveModeConfirmSdoResult_t mode_confirm_last_sdo_result =
        s_drive.safety.transition.mode_confirm_last_sdo_result;
    bool enable_0006_statusword_valid =
        s_drive.safety.transition.enable_0006_statusword_valid;
    bool enable_0007_statusword_valid =
        s_drive.safety.transition.enable_0007_statusword_valid;
    bool enable_000f_statusword_valid =
        s_drive.safety.transition.enable_000f_statusword_valid;
    bool mode_readback_valid =
        s_drive.safety.transition.mode_readback_valid;
    CanopenRuntimeDiagnostics_t diag = Canopen_GetRuntimeDiagnostics();

    if (!s_drive.diagnostic.last_fault_snapshot.valid ||
        (s_drive.diagnostic.last_fault_snapshot.event_ms != now_ms) ||
        (s_drive.diagnostic.last_fault_snapshot.reason != reason) ||
        (s_drive.diagnostic.last_fault_snapshot.node_id != failure_node_id)) {
        Drive_RecordFaultEvent(reason, failure_node_id, now_ms);
    }
    if (s_drive.safety.transition.request_pending) {
        Canopen_CancelAsyncOwner(s_drive.safety.transition.pending_owner, now_ms);
    }
    memset(&s_drive.safety.transition, 0, sizeof(s_drive.safety.transition));
    s_drive.safety.transition.active = true;
    s_drive.safety.transition.type = rollback_type;
    s_drive.safety.transition.failure_reason = reason;
    s_drive.safety.transition.start_ms = now_ms;
    s_drive.safety.transition.generation = Drive_BeginSafetyGeneration();
    s_drive.safety.transition.left_start_sequence =
        s_drive.feedback.motion.left.sequence;
    s_drive.safety.transition.right_start_sequence =
        s_drive.feedback.motion.right.sequence;
    s_drive.safety.transition.left_emcy_sequence =
        diag.emcy_sequence[LEFT_NODE_ID];
    s_drive.safety.transition.right_emcy_sequence =
        diag.emcy_sequence[RIGHT_NODE_ID];
    s_drive.safety.transition.explicit_recovery = explicit_recovery;
    s_drive.safety.transition.had_tracking_latch = had_tracking_latch;
    s_drive.safety.transition.had_disable_latch = had_disable_latch;
    s_drive.safety.transition.failure_node_id = failure_node_id;
    s_drive.safety.transition.last_abort_code = last_abort_code;
    s_drive.safety.transition.transient_timeout_count =
        transient_timeout_count;
    s_drive.safety.transition.mode_confirm_attempt = mode_confirm_attempt;
    s_drive.safety.transition.mode_confirm_start_tick =
        mode_confirm_start_tick;
    s_drive.safety.transition.mode_confirm_next_poll_tick =
        mode_confirm_next_poll_tick;
    s_drive.safety.transition.mode_confirm_elapsed_ms =
        mode_confirm_elapsed_ms;
    s_drive.safety.transition.mode_confirm_last_abort_code =
        mode_confirm_last_abort_code;
    s_drive.safety.transition.enable_0006_statusword =
        enable_0006_statusword;
    s_drive.safety.transition.enable_0007_statusword =
        enable_0007_statusword;
    s_drive.safety.transition.enable_000f_statusword =
        enable_000f_statusword;
    s_drive.safety.transition.requested_mode = requested_mode;
    s_drive.safety.transition.last_mode_readback = last_mode_readback;
    s_drive.safety.transition.mode_confirm_node_id = mode_confirm_node_id;
    s_drive.safety.transition.mode_confirm_last_sdo_result =
        mode_confirm_last_sdo_result;
    s_drive.safety.transition.enable_0006_statusword_valid =
        enable_0006_statusword_valid;
    s_drive.safety.transition.enable_0007_statusword_valid =
        enable_0007_statusword_valid;
    s_drive.safety.transition.enable_000f_statusword_valid =
        enable_000f_statusword_valid;
    s_drive.safety.transition.mode_readback_valid = mode_readback_valid;
    Drive_SafetySetPhase(DRIVE_SAFETY_PHASE_ZERO_START, now_ms);
    DriveCore_SetSoftwareEnabled(false);
    DriveCore_SetHardwareEnableConfirmed(false);
    DriveCore_SetHardwareDisableConfirmed(false);
    DriveCore_SetTrackingStopLatched(true);
    DriveCore_SetMotionInhibitLatched(true);
    DriveCore_SetTrackingStopIgnoreLogged(false);
    Drive_ClearMotionCommandStateOnly();
    Drive_InvalidateActuatorBaseline();
    Drive_ResetYawRateControl(true);
    DriveCore_SetState(DRIVE_STATE_FAULT);
    JetsonTx_Printf(JETSON_TX_PRIORITY_CRITICAL,
                "DRIVE safety rollback started reason=%s\r\n",
                Drive_SafetyFailureName(reason));
}

void Drive_EnterHardFault(DriveSafetyFailureReason_t reason,
                          uint8_t node_id,
                          uint32_t now_ms)
{
    s_drive.safety.transition.failure_node_id = node_id;
    DriveCore_SetSoftwareEnabled(false);
    DriveCore_SetHardwareEnableConfirmed(false);
    Drive_ClearMotionCommandStateOnly();
    DriveCore_SetState(DRIVE_STATE_FAULT);
    Drive_SafetyBeginRollback(
        DRIVE_SAFETY_DISABLE_ROLLBACK, reason, now_ms);
}

void Drive_SafetyEscalate(DriveSafetyFailureReason_t reason,
                                 uint32_t now_ms)
{
    if (Drive_BootInProgress()) {
        uint8_t hard_node = 0U;
        DriveSafetyFailureReason_t hard_reason = Drive_BootKnownFailure(&hard_node);
        if (hard_reason != DRIVE_SAFETY_FAILURE_NONE) {
            reason = hard_reason;
            s_drive.safety.transition.failure_node_id = hard_node;
        }
    }
    if (s_drive.safety.transition.failure_node_id == 0U) {
        if (s_drive.safety.transition.pending_node_id != 0U) {
            s_drive.safety.transition.failure_node_id =
                s_drive.safety.transition.pending_node_id;
        } else if ((reason == DRIVE_SAFETY_FAILURE_SDO_ABORT) &&
                   (Canopen_GetRuntimeDiagnostics().last_sdo_abort_node !=
                    0U)) {
            s_drive.safety.transition.failure_node_id =
                Canopen_GetRuntimeDiagnostics().last_sdo_abort_node;
        }
    }
    if (Drive_BootTryWait(reason, now_ms)) { return; }
    if (Drive_BootInProgress()) {
        Drive_BootLogFailure(reason, "FAULT_LATCHED");
    }
    s_drive.safety.transition.failure_reason = reason;
    Drive_RecordFaultEvent(reason,
                           s_drive.safety.transition.failure_node_id,
                           now_ms);
    if (s_drive.safety.transition.type == DRIVE_SAFETY_ENABLE) {
        Drive_SafetyBeginRollback(DRIVE_SAFETY_ENABLE_ROLLBACK,
                                  reason, now_ms);
    } else if ((s_drive.safety.transition.type == DRIVE_SAFETY_STOP) ||
               (s_drive.safety.transition.type == DRIVE_SAFETY_DISABLE)) {
        Drive_SafetyBeginRollback(DRIVE_SAFETY_DISABLE_ROLLBACK,
                                  reason, now_ms);
    } else {
        Drive_SafetyFinishFault(reason);
    }
}

static bool Drive_SafetyActionMustFail(
    DriveSafetySdoActionResult_t result,
    uint32_t now_ms)
{
    DriveSafetyFailureReason_t reason;

    if ((result == DRIVE_SDO_ACTION_WAIT) ||
        (result == DRIVE_SDO_ACTION_SUCCESS)) {
        return false;
    }
    if ((result == DRIVE_SDO_ACTION_TIMEOUT) &&
        (s_drive.safety.transition.action_attempts <
         DRIVE_SAFETY_SDO_MAX_ATTEMPTS)) {
        return false;
    }
    if (result == DRIVE_SDO_ACTION_TIMEOUT) {
        reason = DRIVE_SAFETY_FAILURE_SDO_TIMEOUT;
    } else if (result == DRIVE_SDO_ACTION_ABORT) {
        reason = DRIVE_SAFETY_FAILURE_SDO_ABORT;
    } else {
        reason = DRIVE_SAFETY_FAILURE_SDO_PROTOCOL;
    }
    Drive_SafetyEscalate(reason, now_ms);
    return true;
}

static void Drive_SafetyBeginDisableControlwords(uint32_t now_ms)
{
    DriveCore_SetHardwareEnableConfirmed(false);
    DriveCore_SetHardwareDisableConfirmed(false);
    Drive_SafetySetPhase(DRIVE_SAFETY_PHASE_DISABLE_WRITE_LEFT, now_ms);
}

static bool Drive_EnableFinalStatusIsValid(uint32_t now_ms,
                                           uint8_t *failed_node_out)
{
    bool left_fresh =
        s_drive.statusword.left_statusword_valid &&
        (s_drive.statusword.left_statusword_generation ==
         s_drive.safety.transition.generation) &&
        ((uint32_t)(now_ms -
                    s_drive.statusword.left_statusword_timestamp_ms) <=
         DRIVE_WHEEL_FEEDBACK_STALE_MS);
    bool right_fresh =
        s_drive.statusword.right_statusword_valid &&
        (s_drive.statusword.right_statusword_generation ==
         s_drive.safety.transition.generation) &&
        ((uint32_t)(now_ms -
                    s_drive.statusword.right_statusword_timestamp_ms) <=
         DRIVE_WHEEL_FEEDBACK_STALE_MS);

    if (!left_fresh ||
        Statusword_HasFault(s_drive.statusword.left_statusword) ||
        !Statusword_IsOperationEnabled(
            s_drive.statusword.left_statusword)) {
        *failed_node_out = LEFT_NODE_ID;
        return false;
    }
    if (!right_fresh ||
        Statusword_HasFault(s_drive.statusword.right_statusword) ||
        !Statusword_IsOperationEnabled(
            s_drive.statusword.right_statusword)) {
        *failed_node_out = RIGHT_NODE_ID;
        return false;
    }
    *failed_node_out = 0U;
    return true;
}

static void Drive_SafetyCompleteEnable(uint32_t now_ms)
{
    uint8_t failed_node = 0U;

    if (!WheelPdoConfig_Ready(now_ms) ||
        !WheelRpdoTx_Rearm(now_ms) ||
        !Drive_EnableFinalStatusIsValid(now_ms, &failed_node) ||
        !Canopen_HeartbeatIsQualified(LEFT_NODE_ID) ||
        !Canopen_HeartbeatIsQualified(RIGHT_NODE_ID) ||
        Canopen_HeartbeatFaultPending() ||
        Canopen_WheelRuntimeFaultPending()) {
        s_drive.safety.transition.failure_node_id = failed_node;
        Drive_SafetyEscalate(
            DRIVE_SAFETY_FAILURE_ENABLE_CONFIRM, now_ms);
        return;
    }

    s_drive.command.last_left_cmd_rpm = 0;
    s_drive.command.last_right_cmd_rpm = 0;
    s_drive.command.last_left_cmd_dec = 0;
    s_drive.command.last_right_cmd_dec = 0;
    Motion_ActuatorStateCommitZero(
        &s_drive.control.actuator.state,
        now_ms);

    /* Historical tracking and generic inhibit latches are diagnostic only in
     * the basic profile. A successful explicit DS402 enable establishes the
     * current controllable state directly. */
    DriveCore_SetTrackingStopLatched(false);
    DriveCore_SetMotionInhibitLatched(false);
    DriveCore_SetTrackingStopIgnoreLogged(false);
    Motion_TrackingSupervisorReset(
        &s_drive.control.tracking.supervisor, true);
    s_drive.safety.transition.active = false;
    DriveCore_SetSoftwareEnabled(true);
    DriveCore_SetHardwareEnableConfirmed(true);
    DriveCore_SetHardwareDisableConfirmed(false);
    DriveCore_SetState(DRIVE_STATE_ENABLED_IDLE);
}

static void Drive_SafetyCompleteZero(uint32_t now_ms)
{
    DriveZeroEvidenceReason_t reason = DRIVE_ZERO_EVIDENCE_STOP;

    if (s_drive.safety.transition.type == DRIVE_SAFETY_DISABLE) {
        reason = DRIVE_ZERO_EVIDENCE_DISABLE;
    } else if (s_drive.safety.transition.type == DRIVE_SAFETY_ENABLE) {
        reason = DRIVE_ZERO_EVIDENCE_ENABLE;
    }
    Motion_ActuatorStateCommitZero(&s_drive.control.actuator.state, now_ms);
    Drive_RecordZeroSpeedEvidence(s_drive.safety.transition.generation,
                                  now_ms, reason);
    s_drive.command.last_left_cmd_rpm = 0;
    s_drive.command.last_right_cmd_rpm = 0;
    s_drive.command.last_left_cmd_dec = 0;
    s_drive.command.last_right_cmd_dec = 0;

    if (s_drive.safety.transition.type == DRIVE_SAFETY_STOP) {
        bool can_idle = s_drive.safety.transition.was_software_enabled &&
            s_hardware_enable_confirmed &&
            !s_tracking_stop_latched &&
            !s_disable_motion_inhibit_latched &&
            Drive_LastCommandTargetIsZero() &&
            Drive_ZeroSpeedEvidenceIsCurrent(now_ms);
        bool can_remain_disabled =
            s_drive.safety.transition.was_hardware_disabled_confirmed &&
            s_hardware_disable_confirmed &&
            Drive_ZeroSpeedEvidenceIsCurrent(now_ms);

        s_drive.safety.transition.active = false;
        if (can_idle && WheelRpdoTx_Rearm(now_ms)) {
            DriveCore_SetSoftwareEnabled(true);
            DriveCore_SetState(DRIVE_STATE_ENABLED_IDLE);
        } else if (can_remain_disabled) {
            DriveCore_SetSoftwareEnabled(false);
            DriveCore_SetState(DRIVE_STATE_DISABLED);
        } else {
            DriveCore_SetSoftwareEnabled(false);
            DriveCore_SetMotionInhibitLatched(true);
            DriveCore_SetState(DRIVE_STATE_FAULT);
        }
        return;
    }
    if (s_drive.safety.transition.type == DRIVE_SAFETY_DISABLE) {
        Drive_SafetyBeginDisableControlwords(now_ms);
        return;
    }
    if (s_drive.safety.transition.type == DRIVE_SAFETY_ENABLE) {
        Drive_SafetyEscalate(
            DRIVE_SAFETY_FAILURE_SDO_PROTOCOL, now_ms);
    }
}

static void Drive_SafetyCompleteDisable(uint32_t now_ms)
{
    bool rollback = Drive_SafetyIsRollback();
    DriveWheelFeedbackPair_t post_disable_pair = {0};
    DriveSafetyFailureReason_t original_reason =
        s_drive.safety.transition.failure_reason;

    (void)Drive_GetAuthoritativeWheelFeedbackPair(
        now_ms, &post_disable_pair);
    DriveCore_SetHardwareEnableConfirmed(false);
    DriveCore_SetHardwareDisableConfirmed(true);
    DriveCore_SetSoftwareEnabled(false);
    s_drive.command.last_left_cmd_rpm = 0;
    s_drive.command.last_right_cmd_rpm = 0;
    s_drive.command.last_left_cmd_dec = 0;
    s_drive.command.last_right_cmd_dec = 0;
    /* Removing holding torque creates a new physical-motion epoch. Evidence
     * collected before the disabled statuswords completed cannot authorize
     * Lift, and the unchanged cached pair cannot recreate it. */
    Drive_InvalidateZeroSpeedEvidence();
    s_drive.evidence.post_disable_baseline_pair_sequence =
        post_disable_pair.pair_sequence;
    s_drive.evidence.post_disable_baseline_left_sequence =
        post_disable_pair.left_node_sequence;
    s_drive.evidence.post_disable_baseline_right_sequence =
        post_disable_pair.right_node_sequence;
    s_drive.evidence.post_disable_zero_proof_required = true;
    Motion_ActuatorStateInvalidate(&s_drive.control.actuator.state);
    s_drive.safety.transition.active = false;
    if (rollback) {
        DriveCore_SetTrackingStopLatched(true);
        DriveCore_SetMotionInhibitLatched(true);
        DriveCore_SetState(DRIVE_STATE_FAULT);
        JetsonTx_Printf(JETSON_TX_PRIORITY_CRITICAL,
                    "DRIVE rollback disable confirmed; state remains FAULT reason=%s\r\n",
                    Drive_SafetyFailureName(original_reason));
    } else {
        if (s_drive.safety.transition.type == DRIVE_SAFETY_INIT_DISABLE) {
            s_drive.boot.state = DRIVE_BOOT_READY;
            Drive_ClearMotionCommandStateOnly();
            Canopen_BootSdoEnd();
            (void)JetsonTx_Printf(JETSON_TX_PRIORITY_STATUS,
                "DRIVE_BOOT attempt=%u state=DISABLED_CONFIRMED\r\n",
                (unsigned)(s_drive.boot.retries + 1U));
        }
        DriveCore_SetState(DRIVE_STATE_DISABLED);
        uart_printf("Drive disabled\r\n");
    }
}

static void Drive_SafetyServiceEnableAction(
    uint32_t now_ms,
    DriveSafetySdoAction_t action,
    uint16_t index,
    uint8_t subindex,
    uint32_t write_value,
    DriveSafetyPhase_t success_phase,
    uint32_t delay_ms)
{
    DriveSafetySdoActionResult_t result = Drive_SafetyServiceSdoAction(
        now_ms, s_drive.safety.transition.enable_node_id, action,
        index, subindex, write_value, NULL);

    if (result == DRIVE_SDO_ACTION_SUCCESS) {
        Drive_SafetySetPhase(success_phase, now_ms);
        s_drive.safety.transition.next_action_ms = now_ms + delay_ms;
        return;
    }
    (void)Drive_SafetyActionMustFail(result, now_ms);
}

static void Drive_HeartbeatConfigFail(uint32_t now_ms)
{
    s_drive.safety.transition.failure_node_id =
        s_drive.safety.transition.enable_node_id;
    Drive_SafetyEscalate(
        DRIVE_SAFETY_FAILURE_HEARTBEAT, now_ms);
}

static void Drive_SafetyServiceHeartbeatRead(
    uint32_t now_ms,
    DriveSafetySdoAction_t action,
    uint16_t index,
    uint8_t subindex,
    uint32_t expected_value,
    DriveSafetyPhase_t success_phase,
    bool mark_configured)
{
    uint32_t value = 0U;
    DriveSafetySdoActionResult_t result =
        Drive_SafetyServiceSdoAction(
            now_ms, s_drive.safety.transition.enable_node_id,
            action, index, subindex, 0U, &value);

    if (result == DRIVE_SDO_ACTION_SUCCESS) {
        if ((value != expected_value) ||
            (mark_configured &&
             !Canopen_HeartbeatSetConfigured(
                  s_drive.safety.transition.enable_node_id, now_ms))) {
            Drive_HeartbeatConfigFail(now_ms);
            return;
        }
        Drive_SafetySetPhase(success_phase, now_ms);
        return;
    }
    (void)Drive_SafetyActionMustFail(result, now_ms);
}

static bool Drive_SafetyCanDispatch(
    uint32_t now_ms,
    CanopenRuntimeDiagnostics_t *diag)
{
    uint32_t overall_timeout_ms;

    if (!s_drive.safety.transition.active) {
        return false;
    }
    overall_timeout_ms =
        (s_drive.safety.transition.type == DRIVE_SAFETY_STOP) ?
        DRIVE_NORMAL_ZERO_OVERALL_TIMEOUT_MS :
        (s_drive.safety.transition.type == DRIVE_SAFETY_ENABLE) ?
        DRIVE_ENABLE_TRANSITION_TIMEOUT_MS :
        (Drive_SafetyIsRollback() ?
         DRIVE_ROLLBACK_TRANSITION_TIMEOUT_MS :
         (((s_drive.safety.transition.type == DRIVE_SAFETY_DISABLE) ||
           (s_drive.safety.transition.type == DRIVE_SAFETY_INIT_DISABLE)) ?
          DRIVE_DISABLE_TRANSITION_TIMEOUT_MS :
          DRIVE_STOP_CONFIRM_TIMEOUT_MS));
    if ((uint32_t)(now_ms - s_drive.safety.transition.start_ms) >
        overall_timeout_ms) {
        if (s_drive.safety.transition.type == DRIVE_SAFETY_STOP) {
            Drive_StableZeroMarkTimeout(now_ms);
        }
        Drive_SafetyEscalate(DRIVE_SAFETY_FAILURE_OVERALL_TIMEOUT,
                             now_ms);
        return false;
    }

    *diag = Canopen_GetRuntimeDiagnostics();
    {
        DriveEmcyDelta_t left_delta = Drive_ClassifyEmcyDelta(
            diag, LEFT_NODE_ID, s_drive.safety.transition.left_emcy_sequence);
        DriveEmcyDelta_t right_delta = Drive_ClassifyEmcyDelta(
            diag, RIGHT_NODE_ID, s_drive.safety.transition.right_emcy_sequence);
        bool node_fault = (left_delta == DRIVE_EMCY_DELTA_FAULT) ||
                          (right_delta == DRIVE_EMCY_DELTA_FAULT);

        /* A single benign EMCY 0x0000 (error reset notification) advances
         * the tracked sequence without failing the transition. */
        s_drive.safety.transition.left_emcy_sequence =
            diag->emcy_sequence[LEFT_NODE_ID];
        s_drive.safety.transition.right_emcy_sequence =
            diag->emcy_sequence[RIGHT_NODE_ID];
        if (node_fault) {
            s_drive.safety.transition.failure_node_id =
                (left_delta == DRIVE_EMCY_DELTA_FAULT) ?
                LEFT_NODE_ID : RIGHT_NODE_ID;
            if (!Drive_SafetyIsRollback()) {
                Drive_SafetyEscalate(DRIVE_SAFETY_FAILURE_NODE_FAULT,
                                     now_ms);
                return false;
            }
            s_drive.safety.transition.failure_reason =
                DRIVE_SAFETY_FAILURE_NODE_FAULT;
            Drive_RecordFaultEvent(
                DRIVE_SAFETY_FAILURE_NODE_FAULT,
                s_drive.safety.transition.failure_node_id, now_ms);
        }
    }
    return true;
}

static void Drive_SafetyServiceZeroStart(
    uint32_t now_ms,
    const CanopenRuntimeDiagnostics_t *diag)
{
    CanopenEmergencyZeroStatus_t zero_status;

    s_drive.safety.transition.zero_start_ms = now_ms;
    s_drive.safety.transition.zero_start_abort_count =
        diag->sdo_abort_count;
    s_drive.safety.transition.left_start_sequence =
        s_drive.feedback.motion.left.sequence;
    s_drive.safety.transition.right_start_sequence =
        s_drive.feedback.motion.right.sequence;
    zero_status = Canopen_EmergencyZeroWheelTargets();
    if (zero_status == CANOPEN_EMERGENCY_ZERO_FAILED) {
        if (Drive_SafetyIsRollback()) {
            Drive_SafetyBeginDisableControlwords(now_ms);
        } else {
            Drive_SafetyEscalate(
                DRIVE_SAFETY_FAILURE_ZERO_SUBMIT, now_ms);
        }
        return;
    }
    Drive_SafetySetPhase(DRIVE_SAFETY_PHASE_ZERO_WAIT_ACK,
                         now_ms);
}

static void Drive_SafetyServiceZeroWaitAck(
    uint32_t now_ms,
    const CanopenRuntimeDiagnostics_t *diag)
{
    CanopenEmergencyZeroStatus_t zero_status =
        Canopen_GetEmergencyZeroStatus();

    if (zero_status == CANOPEN_EMERGENCY_ZERO_ACKNOWLEDGED) {
        if (Drive_SafetyIsRollback()) {
            Drive_SafetyBeginDisableControlwords(now_ms);
        } else {
            s_drive.safety.transition.left_start_sequence =
                s_drive.feedback.motion.left.sequence;
            s_drive.safety.transition.right_start_sequence =
                s_drive.feedback.motion.right.sequence;
            Drive_StableZeroStart(
                now_ms,
                s_drive.safety.transition.type == DRIVE_SAFETY_STOP);
            Drive_SafetySetPhase(
                (s_drive.safety.transition.type == DRIVE_SAFETY_STOP) ?
                DRIVE_SAFETY_PHASE_ZERO_WAIT_SETTLE :
                DRIVE_SAFETY_PHASE_ZERO_WAIT_MEASURED,
                now_ms);
        }
        return;
    }
    if (zero_status == CANOPEN_EMERGENCY_ZERO_FAILED) {
        DriveSafetyFailureReason_t reason =
            (diag->sdo_abort_count !=
             s_drive.safety.transition.zero_start_abort_count) ?
            DRIVE_SAFETY_FAILURE_SDO_ABORT :
            DRIVE_SAFETY_FAILURE_ZERO_ACK;

        if (reason == DRIVE_SAFETY_FAILURE_SDO_ABORT) {
            s_drive.safety.transition.last_abort_code =
                diag->last_sdo_abort_code;
        }
        if (Drive_SafetyIsRollback()) {
            Drive_SafetyBeginDisableControlwords(now_ms);
        } else {
            Drive_SafetyEscalate(reason, now_ms);
        }
        return;
    }
    if ((uint32_t)(now_ms - s_drive.safety.transition.zero_start_ms) >
        (Drive_SafetyIsRollback() ?
         DRIVE_ROLLBACK_ZERO_ACK_TIMEOUT_MS :
         DRIVE_STOP_CONFIRM_TIMEOUT_MS)) {
        if (Drive_SafetyIsRollback()) {
            Drive_SafetyBeginDisableControlwords(now_ms);
        } else {
            Drive_SafetyEscalate(
                DRIVE_SAFETY_FAILURE_ZERO_ACK, now_ms);
        }
    }
}

static void Drive_SafetyServiceZeroWaitMeasured(uint32_t now_ms)
{
    DriveStableZeroResult_t result = Drive_StableZeroUpdate(now_ms);

    if (s_drive.stable_zero.state == DRIVE_STABLE_ZERO_SETTLING) {
        Drive_SafetySetPhase(
            DRIVE_SAFETY_PHASE_ZERO_WAIT_SETTLE, now_ms);
        return;
    }
    if (result == DRIVE_STABLE_ZERO_COMPLETE) {
        Drive_SafetyCompleteZero(now_ms);
    } else if (result == DRIVE_STABLE_ZERO_TIMED_OUT) {
        Drive_SafetyEscalate(
            DRIVE_SAFETY_FAILURE_ZERO_FEEDBACK, now_ms);
    }
}

static void Drive_SafetyServiceZeroWaitSettle(uint32_t now_ms)
{
    DriveStableZeroResult_t result = Drive_StableZeroUpdate(now_ms);

    if (s_drive.stable_zero.state == DRIVE_STABLE_ZERO_CONFIRMING) {
        Drive_SafetySetPhase(
            DRIVE_SAFETY_PHASE_ZERO_WAIT_MEASURED, now_ms);
    }
    if (result == DRIVE_STABLE_ZERO_COMPLETE) {
        Drive_SafetyCompleteZero(now_ms);
    } else if (result == DRIVE_STABLE_ZERO_TIMED_OUT) {
        Drive_SafetyEscalate(
            DRIVE_SAFETY_FAILURE_ZERO_FEEDBACK, now_ms);
    }
}

static void Drive_SafetyServiceEnableReadInitialStatus(uint32_t now_ms)
{
    uint32_t value = 0U;
    DriveSafetySdoActionResult_t result =
        Drive_SafetyServiceSdoAction(
            now_ms, s_drive.safety.transition.enable_node_id,
            DRIVE_SDO_ACTION_READ_U16, 0x6041U, 0U, 0U, &value);

    if (result == DRIVE_SDO_ACTION_SUCCESS) {
        uint16_t statusword = (uint16_t)value;

        Drive_RecordStatusword(
            s_drive.safety.transition.enable_node_id,
            statusword, s_drive.safety.transition.last_sdo_rx_ms,
            s_drive.safety.transition.generation);
        if (Drive_SafetyStatuswordNeedsRefresh(statusword, now_ms)) {
            return;
        }
        if (Statusword_HasFault(statusword)) {
            s_drive.safety.transition.failure_node_id =
                s_drive.safety.transition.enable_node_id;
            Drive_SafetyEscalate(
                DRIVE_SAFETY_FAILURE_STATUS_FAULT, now_ms);
        } else {
            Drive_SafetySetPhase(
                DRIVE_SAFETY_PHASE_ENABLE_HB_WRITE_100C,
                now_ms);
        }
        return;
    }
    (void)Drive_SafetyActionMustFail(result, now_ms);
}

static void Drive_SafetyServiceEnableSetMode(uint32_t now_ms)
{
    DriveSafetySdoActionResult_t result =
        Drive_SafetyServiceSdoAction(
            now_ms, s_drive.safety.transition.enable_node_id,
            DRIVE_SDO_ACTION_WRITE_I8, 0x6060U, 0U,
            (uint32_t)(uint8_t)
                s_drive.safety.transition.requested_mode,
            NULL);

    if (result == DRIVE_SDO_ACTION_SUCCESS) {
        Drive_ModeConfirmInitialize(
            s_drive.safety.transition.enable_node_id);
        Drive_ModeConfirmStartWindow(now_ms);
        Drive_SafetySetPhase(
            DRIVE_SAFETY_PHASE_ENABLE_READ_MODE_REQUEST, now_ms);
        return;
    }
    (void)Drive_SafetyActionMustFail(result, now_ms);
}

static void Drive_SafetyServiceEnableReadModeRequest(uint32_t now_ms)
{
    uint32_t value = 0U;
    DriveSafetySdoActionResult_t result;

    s_drive.safety.transition.mode_confirm_elapsed_ms =
        (uint32_t)(now_ms -
                   s_drive.safety.transition.mode_confirm_start_tick);
    if (!s_drive.safety.transition.request_pending &&
        (s_drive.safety.transition.mode_confirm_elapsed_ms >=
         DRIVE_MODE_REQUEST_TIMEOUT_MS)) {
        s_drive.safety.transition.failure_node_id =
            s_drive.safety.transition.mode_confirm_node_id;
        Drive_ModeConfirmPrintTimeout();
        Drive_SafetyEscalate(
            DRIVE_SAFETY_FAILURE_ENABLE_CONFIRM, now_ms);
        return;
    }
    if (!s_drive.safety.transition.request_pending) {
        s_drive.safety.transition.next_action_ms =
            s_drive.safety.transition.mode_confirm_next_poll_tick;
    }
    result = Drive_SafetyServiceSdoAction(
        now_ms, s_drive.safety.transition.enable_node_id,
        DRIVE_SDO_ACTION_READ_I8, 0x6060U, 0U, 0U, &value);
    s_drive.safety.transition.mode_confirm_elapsed_ms =
        (uint32_t)(now_ms -
                   s_drive.safety.transition.mode_confirm_start_tick);

    if (result == DRIVE_SDO_ACTION_SUCCESS) {
        s_drive.safety.transition.mode_confirm_last_sdo_result =
            DRIVE_MODE_CONFIRM_SDO_SUCCESS;
        s_drive.safety.transition.mode_confirm_attempt++;
        s_drive.safety.transition.last_mode_readback =
            (int8_t)(value & 0xFFU);
        s_drive.safety.transition.mode_readback_valid = true;
        if (s_drive.safety.transition.mode_readback_valid &&
            (s_drive.safety.transition.last_mode_readback ==
             s_drive.safety.transition.requested_mode) &&
            (s_drive.safety.transition.requested_mode == -3) &&
            (s_drive.safety.transition.mode_confirm_elapsed_ms <=
             DRIVE_MODE_REQUEST_TIMEOUT_MS)) {
            Drive_ModeConfirmPrint("COMPLETE");
            Drive_SafetySetPhase(
                DRIVE_SAFETY_PHASE_ENABLE_WRITE_ZERO, now_ms);
        } else if (
            s_drive.safety.transition.mode_confirm_elapsed_ms >=
            DRIVE_MODE_REQUEST_TIMEOUT_MS) {
            s_drive.safety.transition.failure_node_id =
                s_drive.safety.transition.mode_confirm_node_id;
            Drive_ModeConfirmPrintTimeout();
            Drive_SafetyEscalate(
                DRIVE_SAFETY_FAILURE_ENABLE_CONFIRM, now_ms);
        } else {
            Drive_ModeConfirmPrint("POLL");
            s_drive.safety.transition.action_attempts = 0U;
            s_drive.safety.transition.mode_confirm_next_poll_tick =
                now_ms + DRIVE_MODE_REQUEST_POLL_INTERVAL_MS;
            s_drive.safety.transition.next_action_ms =
                s_drive.safety.transition.mode_confirm_next_poll_tick;
        }
        return;
    }
    if (result == DRIVE_SDO_ACTION_WAIT) {
        if (s_drive.safety.transition.request_pending &&
            (s_drive.safety.transition.mode_confirm_elapsed_ms >=
             DRIVE_MODE_REQUEST_TIMEOUT_MS)) {
            Canopen_CancelAsyncOwner(
                s_drive.safety.transition.pending_owner, now_ms);
            s_drive.safety.transition.request_pending = false;
            s_drive.safety.transition.mode_confirm_last_sdo_result =
                DRIVE_MODE_CONFIRM_SDO_TIMEOUT;
            s_drive.safety.transition.failure_node_id =
                s_drive.safety.transition.mode_confirm_node_id;
            Drive_ModeConfirmPrintTimeout();
            Drive_SafetyEscalate(
                DRIVE_SAFETY_FAILURE_ENABLE_CONFIRM, now_ms);
        }
        return;
    }
    if (result == DRIVE_SDO_ACTION_TIMEOUT) {
        s_drive.safety.transition.mode_confirm_last_sdo_result =
            DRIVE_MODE_CONFIRM_SDO_TIMEOUT;
        s_drive.safety.transition.mode_confirm_next_poll_tick =
            s_drive.safety.transition.next_action_ms;
        if (s_drive.safety.transition.mode_confirm_elapsed_ms >=
            DRIVE_MODE_REQUEST_TIMEOUT_MS) {
            s_drive.safety.transition.failure_node_id =
                s_drive.safety.transition.mode_confirm_node_id;
            Drive_ModeConfirmPrintTimeout();
            Drive_SafetyEscalate(
                DRIVE_SAFETY_FAILURE_ENABLE_CONFIRM, now_ms);
            return;
        }
    } else if (result == DRIVE_SDO_ACTION_ABORT) {
        s_drive.safety.transition.mode_confirm_last_sdo_result =
            DRIVE_MODE_CONFIRM_SDO_ABORT;
        s_drive.safety.transition.mode_confirm_last_abort_code =
            s_drive.safety.transition.last_abort_code;
        s_drive.safety.transition.failure_node_id =
            s_drive.safety.transition.mode_confirm_node_id;
        Drive_ModeConfirmPrint("FAILED");
    } else {
        s_drive.safety.transition.mode_confirm_last_sdo_result =
            DRIVE_MODE_CONFIRM_SDO_ERROR;
        s_drive.safety.transition.failure_node_id =
            s_drive.safety.transition.mode_confirm_node_id;
        Drive_ModeConfirmPrint("FAILED");
    }
    if ((result == DRIVE_SDO_ACTION_TIMEOUT) &&
        (s_drive.safety.transition.action_attempts >=
         DRIVE_SAFETY_SDO_MAX_ATTEMPTS)) {
        Drive_ModeConfirmPrint("FAILED");
    }
    (void)Drive_SafetyActionMustFail(result, now_ms);
}

static void Drive_SafetyServiceEnableReadStatus(uint32_t now_ms)
{
    uint32_t value = 0U;
    DriveSafetySdoActionResult_t result =
        Drive_SafetyServiceSdoAction(
            now_ms, s_drive.safety.transition.enable_node_id,
            DRIVE_SDO_ACTION_READ_U16, 0x6041U, 0U, 0U, &value);

    if (result == DRIVE_SDO_ACTION_SUCCESS) {
        uint16_t statusword = (uint16_t)value;
        DriveSafetyPhase_t phase = s_drive.safety.transition.phase;

        Drive_RecordStatusword(
            s_drive.safety.transition.enable_node_id,
            statusword, s_drive.safety.transition.last_sdo_rx_ms,
            s_drive.safety.transition.generation);
        if (Drive_SafetyStatuswordNeedsRefresh(statusword, now_ms)) {
            return;
        }
        if (phase ==
            DRIVE_SAFETY_PHASE_ENABLE_READ_0006_STATUS) {
            s_drive.safety.transition.enable_0006_statusword =
                statusword;
            s_drive.safety.transition.enable_0006_statusword_valid =
                true;
            Drive_EnableStatePrint(0x0006U, statusword, 0x0021U);
        } else if (phase ==
                   DRIVE_SAFETY_PHASE_ENABLE_READ_0007_STATUS) {
            s_drive.safety.transition.enable_0007_statusword =
                statusword;
            s_drive.safety.transition.enable_0007_statusword_valid =
                true;
            Drive_EnableStatePrint(0x0007U, statusword, 0x0023U);
        } else if (phase ==
                   DRIVE_SAFETY_PHASE_ENABLE_READ_000F_STATUS) {
            s_drive.safety.transition.enable_000f_statusword =
                statusword;
            s_drive.safety.transition.enable_000f_statusword_valid =
                true;
            Drive_EnableStatePrint(0x000FU, statusword, 0x0027U);
        }
        if (Statusword_HasFault(statusword)) {
            s_drive.safety.transition.failure_node_id =
                s_drive.safety.transition.enable_node_id;
            Drive_SafetyEscalate(
                DRIVE_SAFETY_FAILURE_STATUS_FAULT, now_ms);
        } else if ((phase ==
                    DRIVE_SAFETY_PHASE_ENABLE_READ_0006_STATUS) &&
                   (Statusword_IsOperationEnabled(statusword) ||
                    !Statusword_IsDs402State(statusword,
                                            0x0021U))) {
            s_drive.safety.transition.failure_node_id =
                s_drive.safety.transition.enable_node_id;
            Drive_SafetyEscalate(
                DRIVE_SAFETY_FAILURE_ENABLE_CONFIRM, now_ms);
        } else if ((phase ==
                    DRIVE_SAFETY_PHASE_ENABLE_READ_0007_STATUS) &&
                   (Statusword_IsOperationEnabled(statusword) ||
                    !Statusword_IsDs402State(statusword,
                                            0x0023U))) {
            s_drive.safety.transition.failure_node_id =
                s_drive.safety.transition.enable_node_id;
            Drive_SafetyEscalate(
                DRIVE_SAFETY_FAILURE_ENABLE_CONFIRM, now_ms);
        } else if ((phase ==
                    DRIVE_SAFETY_PHASE_ENABLE_READ_000F_STATUS) &&
                   (!Statusword_IsOperationEnabled(statusword) ||
                    !Statusword_IsDs402State(statusword,
                                            0x0027U))) {
            s_drive.safety.transition.failure_node_id =
                s_drive.safety.transition.enable_node_id;
            Drive_SafetyEscalate(
                DRIVE_SAFETY_FAILURE_ENABLE_CONFIRM, now_ms);
        } else if ((phase ==
                    DRIVE_SAFETY_PHASE_ENABLE_READ_002F_STATUS) &&
                   ((statusword & (1U << 2)) == 0U)) {
            s_drive.safety.transition.failure_node_id =
                s_drive.safety.transition.enable_node_id;
            Drive_SafetyEscalate(
                DRIVE_SAFETY_FAILURE_ENABLE_CONFIRM, now_ms);
        } else if (phase ==
                   DRIVE_SAFETY_PHASE_ENABLE_READ_0006_STATUS) {
            Drive_SafetySetPhase(
                DRIVE_SAFETY_PHASE_ENABLE_WRITE_0007, now_ms);
        } else if (phase ==
                   DRIVE_SAFETY_PHASE_ENABLE_READ_0007_STATUS) {
            Drive_SafetySetPhase(
                DRIVE_SAFETY_PHASE_ENABLE_WRITE_000F, now_ms);
        } else if (phase ==
                   DRIVE_SAFETY_PHASE_ENABLE_READ_000F_STATUS) {
            Drive_SafetySetPhase(
                DRIVE_SAFETY_PHASE_ENABLE_WRITE_002F, now_ms);
        } else {
            Drive_SafetySetPhase(
                DRIVE_SAFETY_PHASE_ENABLE_NODE_DONE, now_ms);
        }
        return;
    }
    (void)Drive_SafetyActionMustFail(result, now_ms);
}

static void Drive_SafetyServiceEnableNodeDone(uint32_t now_ms)
{
    if (s_drive.safety.transition.enable_node_id == LEFT_NODE_ID) {
        s_drive.safety.transition.enable_node_id = RIGHT_NODE_ID;
        Drive_EnableNodeEvidenceInitialize(RIGHT_NODE_ID);
        Drive_SafetySetPhase(
            DRIVE_SAFETY_PHASE_ENABLE_READ_INITIAL_STATUS,
            now_ms);
    } else {
        s_drive.safety.transition.enable_node_id = LEFT_NODE_ID;
        Drive_SafetySetPhase(
            DRIVE_SAFETY_PHASE_ENABLE_FINAL_STATUS_LEFT, now_ms);
    }
}

static void Drive_SafetyServiceEnableFinalStatus(uint32_t now_ms)
{
    bool left = s_drive.safety.transition.phase ==
                DRIVE_SAFETY_PHASE_ENABLE_FINAL_STATUS_LEFT;
    uint8_t node_id = left ? LEFT_NODE_ID : RIGHT_NODE_ID;
    uint32_t value = 0U;
    DriveSafetySdoActionResult_t result =
        Drive_SafetyServiceSdoAction(
            now_ms, node_id, DRIVE_SDO_ACTION_READ_U16,
            0x6041U, 0U, 0U, &value);

    if (result == DRIVE_SDO_ACTION_SUCCESS) {
        uint16_t statusword = (uint16_t)value;

        Drive_RecordStatusword(
            node_id, statusword, s_drive.safety.transition.last_sdo_rx_ms,
            s_drive.safety.transition.generation);
        if (Drive_SafetyStatuswordNeedsRefresh(statusword, now_ms)) {
            return;
        }
        if (Statusword_HasFault(statusword) ||
            !Statusword_IsOperationEnabled(statusword)) {
            s_drive.safety.transition.failure_node_id = node_id;
            Drive_SafetyEscalate(
                Statusword_HasFault(statusword) ?
                    DRIVE_SAFETY_FAILURE_STATUS_FAULT :
                    DRIVE_SAFETY_FAILURE_ENABLE_CONFIRM,
                now_ms);
        } else if (left) {
            s_drive.safety.transition.enable_node_id = RIGHT_NODE_ID;
            Drive_SafetySetPhase(
                DRIVE_SAFETY_PHASE_ENABLE_FINAL_STATUS_RIGHT,
                now_ms);
        } else {
            Drive_SafetyCompleteEnable(now_ms);
        }
        return;
    }
    (void)Drive_SafetyActionMustFail(result, now_ms);
}

static void Drive_SafetyServiceDisableWrite(uint32_t now_ms)
{
    uint8_t node_id =
        (s_drive.safety.transition.phase ==
         DRIVE_SAFETY_PHASE_DISABLE_WRITE_LEFT) ?
        LEFT_NODE_ID : RIGHT_NODE_ID;
    DriveSafetySdoActionResult_t result =
        Drive_SafetyServiceSdoAction(
            now_ms, node_id, DRIVE_SDO_ACTION_WRITE_U16,
            0x6040U, 0U, 0x0006U, NULL);

    if (result == DRIVE_SDO_ACTION_SUCCESS) {
        if (node_id == LEFT_NODE_ID) {
            Drive_SafetySetPhase(
                DRIVE_SAFETY_PHASE_DISABLE_WRITE_RIGHT, now_ms);
        } else {
            s_drive.safety.transition.status_start_ms = now_ms;
            s_drive.safety.transition.status_attempts = 0U;
            Drive_SafetySetPhase(
                DRIVE_SAFETY_PHASE_DISABLE_STATUS_WAIT, now_ms);
        }
        return;
    }
    (void)Drive_SafetyActionMustFail(result, now_ms);
}

static void Drive_SafetyServiceDisableStatusWait(uint32_t now_ms)
{
    if ((s_drive.safety.transition.status_attempts >=
         DRIVE_DISABLE_STATUS_MAX_ATTEMPTS) ||
        ((uint32_t)(now_ms -
                    s_drive.safety.transition.status_start_ms) >=
         DRIVE_DISABLE_STATUS_TIMEOUT_MS)) {
        Drive_SafetyEscalate(
            DRIVE_SAFETY_FAILURE_DISABLE_CONFIRM, now_ms);
    } else if ((int32_t)(now_ms -
                         s_drive.safety.transition.next_action_ms) >= 0) {
        Drive_SafetySetPhase(
            DRIVE_SAFETY_PHASE_DISABLE_STATUS_READ_LEFT,
            now_ms);
    }
}

static void Drive_SafetyServiceDisableStatusRead(uint32_t now_ms)
{
    bool left = s_drive.safety.transition.phase ==
                DRIVE_SAFETY_PHASE_DISABLE_STATUS_READ_LEFT;
    uint8_t node_id = left ? LEFT_NODE_ID : RIGHT_NODE_ID;
    uint32_t value = 0U;
    bool boot_confirm = s_drive.safety.transition.type == DRIVE_SAFETY_INIT_DISABLE;
    DriveSafetySdoActionResult_t result =
        Drive_SafetyServiceSdoAction(
            now_ms, node_id, DRIVE_SDO_ACTION_READ_U16,
            0x6041U, 0U, 0U, &value);

    if (result == DRIVE_SDO_ACTION_SUCCESS) {
        uint16_t statusword = (uint16_t)value;

        Drive_RecordStatusword(node_id, statusword,
                               s_drive.safety.transition.last_sdo_rx_ms,
                               s_drive.safety.transition.generation);
        if (boot_confirm && !Drive_BootStatusDisabled(statusword)) {
            s_drive.boot.saw_nondisabled_status = true;
        }
        if (Drive_SafetyStatuswordNeedsRefresh(statusword, now_ms)) {
            s_drive.safety.transition.status_attempts++;
            Drive_SafetySetPhase(DRIVE_SAFETY_PHASE_DISABLE_STATUS_WAIT, now_ms);
            s_drive.safety.transition.next_action_ms =
                now_ms + DRIVE_DISABLE_STATUS_POLL_MS;
            return;
        }
        if (left) {
            if (Statusword_HasFault(statusword)) {
                s_drive.safety.transition.failure_node_id = LEFT_NODE_ID;
                Drive_SafetyEscalate(
                    DRIVE_SAFETY_FAILURE_STATUS_FAULT, now_ms);
            } else {
                Drive_SafetySetPhase(
                    DRIVE_SAFETY_PHASE_DISABLE_STATUS_READ_RIGHT,
                    now_ms);
            }
            return;
        }
        if (Statusword_HasFault(statusword)) {
            s_drive.safety.transition.failure_node_id = RIGHT_NODE_ID;
            Drive_SafetyEscalate(
                DRIVE_SAFETY_FAILURE_STATUS_FAULT, now_ms);
            return;
        }
        s_drive.safety.transition.status_attempts++;
        if (((s_drive.statusword.left_statusword & (1U << 2)) == 0U) &&
            ((s_drive.statusword.right_statusword & (1U << 2)) == 0U) &&
            Drive_DisabledEvidenceReady(now_ms)) {
            if (s_drive.safety.transition.type == DRIVE_SAFETY_INIT_DISABLE) {
                uint8_t hard_node = 0U;
                DriveSafetyFailureReason_t failure = Drive_BootKnownFailure(&hard_node);
                if (failure != DRIVE_SAFETY_FAILURE_NONE) {
                    s_drive.safety.transition.failure_node_id = hard_node;
                    Drive_SafetyEscalate(failure, now_ms);
                    return;
                }
                /* These are matched responses in the current software
                 * transaction. Standard SDO has no nonce: an arbitrarily
                 * delayed same-object response can still look identical. */
                for (uint8_t id = LEFT_NODE_ID; id <= RIGHT_NODE_ID; ++id) {
                    (void)JetsonTx_Printf(JETSON_TX_PRIORITY_STATUS,
                        "DRIVE_BOOT attempt=%u node=%u result=OK statusword=%04X\r\n",
                        (unsigned)(s_drive.boot.retries + 1U), (unsigned)id,
                        (unsigned)(id == LEFT_NODE_ID ? s_drive.statusword.left_statusword :
                                                       s_drive.statusword.right_statusword));
                }
            }
            Drive_SafetyCompleteDisable(now_ms);
        } else {
            Drive_SafetySetPhase(
                DRIVE_SAFETY_PHASE_DISABLE_STATUS_WAIT, now_ms);
            s_drive.safety.transition.next_action_ms = now_ms + DRIVE_DISABLE_STATUS_POLL_MS;
        }
        return;
    }
    if (result == DRIVE_SDO_ACTION_TIMEOUT) {
        if (s_drive.safety.transition.type == DRIVE_SAFETY_INIT_DISABLE) {
            /* Missing boot feedback can take the one delayed retry; valid
             * non-disabled states retain the existing bounded polling path.
             * Runtime STOP/disable polling is unchanged. */
            Drive_SafetyEscalate(DRIVE_SAFETY_FAILURE_SDO_TIMEOUT, now_ms);
            return;
        }
        s_drive.safety.transition.status_attempts++;
        Drive_SafetySetPhase(
            DRIVE_SAFETY_PHASE_DISABLE_STATUS_WAIT, now_ms);
        s_drive.safety.transition.next_action_ms =
            now_ms + DRIVE_DISABLE_STATUS_POLL_MS;
        return;
    }
    if ((result == DRIVE_SDO_ACTION_ABORT) ||
        (result == DRIVE_SDO_ACTION_ERROR)) {
        (void)Drive_SafetyActionMustFail(result, now_ms);
    }
}

static void Drive_SafetyServiceHeartbeatQualification(uint32_t now_ms)
{
    if (Canopen_HeartbeatIsQualified(
            s_drive.safety.transition.enable_node_id)) {
        Drive_SafetySetPhase(
            DRIVE_SAFETY_PHASE_ENABLE_SET_MODE, now_ms);
    } else if (Canopen_HeartbeatFaultPending()) {
        Drive_HeartbeatConfigFail(now_ms);
    }
}

void Drive_SafetyTransitionStep(uint32_t now_ms)
{
    CanopenRuntimeDiagnostics_t diag;

    if (!Drive_SafetyCanDispatch(now_ms, &diag)) {
        return;
    }

    WheelRpdoTx_Service(now_ms);
    if (!WheelRpdoTx_Quiescent()) {
        /* No zero/disable can overtake an old nonzero RPDO still in hardware. */
        return;
    }
    switch (s_drive.safety.transition.phase) {
        case DRIVE_SAFETY_PHASE_ZERO_START:
            Drive_SafetyServiceZeroStart(now_ms, &diag); return;
        case DRIVE_SAFETY_PHASE_ZERO_WAIT_ACK:
            Drive_SafetyServiceZeroWaitAck(now_ms, &diag); return;
        case DRIVE_SAFETY_PHASE_ZERO_WAIT_SETTLE:
            Drive_SafetyServiceZeroWaitSettle(now_ms); return;
        case DRIVE_SAFETY_PHASE_ZERO_WAIT_MEASURED:
            Drive_SafetyServiceZeroWaitMeasured(now_ms); return;
        case DRIVE_SAFETY_PHASE_ENABLE_READ_INITIAL_STATUS:
            Drive_SafetyServiceEnableReadInitialStatus(now_ms); return;
        case DRIVE_SAFETY_PHASE_ENABLE_HB_WRITE_100C:
            Drive_SafetyServiceEnableAction(
                now_ms, DRIVE_SDO_ACTION_WRITE_U16, 0x100CU, 0U, 0U,
                DRIVE_SAFETY_PHASE_ENABLE_HB_READ_100C, 0U);
            return;
        case DRIVE_SAFETY_PHASE_ENABLE_HB_READ_100C:
            Drive_SafetyServiceHeartbeatRead(
                now_ms, DRIVE_SDO_ACTION_READ_U16, 0x100CU, 0U, 0U,
                DRIVE_SAFETY_PHASE_ENABLE_HB_WRITE_100D, false);
            return;
        case DRIVE_SAFETY_PHASE_ENABLE_HB_WRITE_100D:
            Drive_SafetyServiceEnableAction(
                now_ms, DRIVE_SDO_ACTION_WRITE_U8, 0x100DU, 0U, 0U,
                DRIVE_SAFETY_PHASE_ENABLE_HB_READ_100D, 0U);
            return;
        case DRIVE_SAFETY_PHASE_ENABLE_HB_READ_100D:
            Drive_SafetyServiceHeartbeatRead(
                now_ms, DRIVE_SDO_ACTION_READ_U8, 0x100DU, 0U, 0U,
                DRIVE_SAFETY_PHASE_ENABLE_HB_WRITE_1017, false);
            return;
        case DRIVE_SAFETY_PHASE_ENABLE_HB_WRITE_1017:
            Drive_SafetyServiceEnableAction(
                now_ms, DRIVE_SDO_ACTION_WRITE_U16, 0x1017U, 0U,
                DRIVE_KINCO_HEARTBEAT_PRODUCER_MS,
                DRIVE_SAFETY_PHASE_ENABLE_HB_READ_1017, 0U);
            return;
        case DRIVE_SAFETY_PHASE_ENABLE_HB_READ_1017:
            Drive_SafetyServiceHeartbeatRead(
                now_ms, DRIVE_SDO_ACTION_READ_U16, 0x1017U, 0U,
                DRIVE_KINCO_HEARTBEAT_PRODUCER_MS,
                DRIVE_SAFETY_PHASE_ENABLE_HB_WRITE_1016_01, false);
            return;
        case DRIVE_SAFETY_PHASE_ENABLE_HB_WRITE_1016_01:
            Drive_SafetyServiceEnableAction(
                now_ms, DRIVE_SDO_ACTION_WRITE_U32, 0x1016U, 0x01U,
                DRIVE_KINCO_HEARTBEAT_CONSUMER_VALUE,
                DRIVE_SAFETY_PHASE_ENABLE_HB_READ_1016_01, 0U);
            return;
        case DRIVE_SAFETY_PHASE_ENABLE_HB_READ_1016_01:
            Drive_SafetyServiceHeartbeatRead(
                now_ms, DRIVE_SDO_ACTION_READ_U32, 0x1016U, 0x01U,
                DRIVE_KINCO_HEARTBEAT_CONSUMER_VALUE,
                DRIVE_SAFETY_PHASE_ENABLE_HB_WRITE_6007, false);
            return;
        case DRIVE_SAFETY_PHASE_ENABLE_HB_WRITE_6007:
            Drive_SafetyServiceEnableAction(
                now_ms, DRIVE_SDO_ACTION_WRITE_U16, 0x6007U, 0U, 1U,
                DRIVE_SAFETY_PHASE_ENABLE_HB_READ_6007, 0U);
            return;
        case DRIVE_SAFETY_PHASE_ENABLE_HB_READ_6007:
            Drive_SafetyServiceHeartbeatRead(
                now_ms, DRIVE_SDO_ACTION_READ_U16, 0x6007U, 0U, 1U,
                DRIVE_SAFETY_PHASE_ENABLE_HB_QUALIFY, true);
            return;
        case DRIVE_SAFETY_PHASE_ENABLE_HB_QUALIFY:
            Drive_SafetyServiceHeartbeatQualification(now_ms); return;
        case DRIVE_SAFETY_PHASE_ENABLE_SET_MODE:
            Drive_SafetyServiceEnableSetMode(now_ms); return;
        case DRIVE_SAFETY_PHASE_ENABLE_READ_MODE_REQUEST:
            Drive_SafetyServiceEnableReadModeRequest(now_ms); return;
        case DRIVE_SAFETY_PHASE_ENABLE_WRITE_ZERO:
            Drive_SafetyServiceEnableAction(
                now_ms, DRIVE_SDO_ACTION_WRITE_I32, 0x60FFU, 0U, 0U,
                DRIVE_SAFETY_PHASE_ENABLE_WRITE_0006, 0U);
            return;
        case DRIVE_SAFETY_PHASE_ENABLE_WRITE_0006:
            Drive_SafetyServiceEnableAction(
                now_ms, DRIVE_SDO_ACTION_WRITE_U16, 0x6040U, 0U, 0x0006U,
                DRIVE_SAFETY_PHASE_ENABLE_READ_0006_STATUS, DRIVE_ENABLE_STEP_SHORT_DELAY_MS);
            return;
        case DRIVE_SAFETY_PHASE_ENABLE_READ_0006_STATUS:
        case DRIVE_SAFETY_PHASE_ENABLE_READ_0007_STATUS:
        case DRIVE_SAFETY_PHASE_ENABLE_READ_000F_STATUS:
        case DRIVE_SAFETY_PHASE_ENABLE_READ_002F_STATUS:
            Drive_SafetyServiceEnableReadStatus(now_ms); return;
        case DRIVE_SAFETY_PHASE_ENABLE_WRITE_0007:
            Drive_SafetyServiceEnableAction(
                now_ms, DRIVE_SDO_ACTION_WRITE_U16, 0x6040U, 0U, 0x0007U,
                DRIVE_SAFETY_PHASE_ENABLE_READ_0007_STATUS, DRIVE_ENABLE_STEP_SHORT_DELAY_MS);
            return;
        case DRIVE_SAFETY_PHASE_ENABLE_WRITE_000F:
            Drive_SafetyServiceEnableAction(
                now_ms, DRIVE_SDO_ACTION_WRITE_U16, 0x6040U, 0U, 0x000FU,
                DRIVE_SAFETY_PHASE_ENABLE_READ_000F_STATUS, DRIVE_ENABLE_STEP_LONG_DELAY_MS);
            return;
        case DRIVE_SAFETY_PHASE_ENABLE_WRITE_002F:
            Drive_SafetyServiceEnableAction(
                now_ms, DRIVE_SDO_ACTION_WRITE_U16, 0x6040U, 0U, 0x002FU,
                DRIVE_SAFETY_PHASE_ENABLE_READ_002F_STATUS, DRIVE_ENABLE_STEP_LONG_DELAY_MS);
            return;
        case DRIVE_SAFETY_PHASE_ENABLE_NODE_DONE:
            Drive_SafetyServiceEnableNodeDone(now_ms); return;
        case DRIVE_SAFETY_PHASE_ENABLE_FINAL_STATUS_LEFT:
        case DRIVE_SAFETY_PHASE_ENABLE_FINAL_STATUS_RIGHT:
            Drive_SafetyServiceEnableFinalStatus(now_ms); return;
        case DRIVE_SAFETY_PHASE_DISABLE_WRITE_LEFT:
        case DRIVE_SAFETY_PHASE_DISABLE_WRITE_RIGHT:
            Drive_SafetyServiceDisableWrite(now_ms); return;
        case DRIVE_SAFETY_PHASE_DISABLE_STATUS_WAIT:
            Drive_SafetyServiceDisableStatusWait(now_ms); return;
        case DRIVE_SAFETY_PHASE_DISABLE_STATUS_READ_LEFT:
        case DRIVE_SAFETY_PHASE_DISABLE_STATUS_READ_RIGHT:
            Drive_SafetyServiceDisableStatusRead(now_ms); return;
        case DRIVE_SAFETY_PHASE_NONE:
        default:
            Drive_SafetyEscalate(DRIVE_SAFETY_FAILURE_SDO_PROTOCOL,
                                 now_ms);
            return;
    }
}

DriveStopResult_t Drive_StopBoth(void)
{
    uint32_t now_ms = HAL_GetTick();
    if (Drive_BootInProgress()) {
        Drive_ClearMotionCommand();
        return DRIVE_STOP_PENDING;
    }
    DriveSafetyTransitionType_t requested_type =
        (s_drive.core.state == DRIVE_STATE_INIT) ?
        DRIVE_SAFETY_INIT_DISABLE : DRIVE_SAFETY_STOP;

    if (s_drive.safety.transition.active) {
        (void)Drive_StartSafetyTransition(requested_type, false,
                                          now_ms);
        return DRIVE_STOP_PENDING;
    }
    if (!Drive_StartSafetyTransition(requested_type, false, now_ms)) {
        return DRIVE_STOP_FAILED;
    }
    Drive_SafetyTransitionStep(now_ms);
    if (requested_type == DRIVE_SAFETY_INIT_DISABLE) {
        return DRIVE_STOP_PENDING;
    }
    return (s_drive.safety.transition.active &&
            (s_drive.safety.transition.type == DRIVE_SAFETY_STOP)) ?
        DRIVE_STOP_ZERO_ACCEPTED : DRIVE_STOP_FAILED;
}

bool Drive_EnsureEnabled(void)
{
    return Drive_NormalMotionAllowed();
}

DriveDisableResult_t Drive_DisableBoth(void)
{
    uint32_t now_ms = HAL_GetTick();
    if (Drive_BootInProgress()) {
        Drive_ClearMotionCommand();
        return DRIVE_DISABLE_PENDING;
    }

    if (s_drive.safety.transition.active) {
        if ((s_drive.safety.transition.type == DRIVE_SAFETY_DISABLE) ||
            (s_drive.safety.transition.type == DRIVE_SAFETY_DISABLE_ROLLBACK) ||
            (s_drive.safety.transition.type == DRIVE_SAFETY_INIT_DISABLE)) {
            return DRIVE_DISABLE_PENDING;
        }
        if ((s_drive.safety.transition.type == DRIVE_SAFETY_STOP) &&
            Drive_StartSafetyTransition(DRIVE_SAFETY_DISABLE, false,
                                        now_ms)) {
            return DRIVE_DISABLE_PENDING;
        }
        return DRIVE_DISABLE_FAILED;
    }
    if (!Drive_StartSafetyTransition(
            (s_drive.core.state == DRIVE_STATE_INIT) ?
            DRIVE_SAFETY_INIT_DISABLE : DRIVE_SAFETY_DISABLE, false,
                                     now_ms)) {
        return DRIVE_DISABLE_FAILED;
    }
    Drive_SafetyTransitionStep(now_ms);
    return (s_drive.safety.transition.active &&
            ((s_drive.safety.transition.type == DRIVE_SAFETY_DISABLE) ||
             (s_drive.safety.transition.type == DRIVE_SAFETY_INIT_DISABLE))) ?
        DRIVE_DISABLE_PENDING : DRIVE_DISABLE_FAILED;
}

void Drive_HandleFault(uint8_t fault_node_id)
{
    uint32_t now_ms = HAL_GetTick();

    if (!s_drive.safety.transition.active) {
        Drive_RecordFaultEvent(DRIVE_SAFETY_FAILURE_NODE_FAULT,
                               fault_node_id, now_ms);
    }
    JetsonTx_Printf(JETSON_TX_PRIORITY_CRITICAL,
                "FAULT DETECTED node=%u, STARTING NONBLOCKING ROLLBACK\r\n",
                fault_node_id);
    DriveCore_SetSoftwareEnabled(false);
    DriveCore_SetTrackingStopLatched(true);
    DriveCore_SetMotionInhibitLatched(true);
    s_drive.safety.transition.failure_node_id = fault_node_id;
    DriveCore_SetState(DRIVE_STATE_FAULT);
    if (s_drive.safety.transition.active) {
        if (Drive_SafetyIsRollback()) {
            s_drive.safety.transition.failure_reason =
                DRIVE_SAFETY_FAILURE_NODE_FAULT;
            Drive_RecordFaultEvent(DRIVE_SAFETY_FAILURE_NODE_FAULT,
                                   fault_node_id, now_ms);
        } else {
            Drive_SafetyEscalate(
                DRIVE_SAFETY_FAILURE_NODE_FAULT, now_ms);
        }
    } else {
        Drive_SafetyBeginRollback(
            DRIVE_SAFETY_DISABLE_ROLLBACK,
            DRIVE_SAFETY_FAILURE_NODE_FAULT, now_ms);
    }
    Drive_ResetYawRateControl(true);
}

void Drive_CommandStop(void)
{
    uint32_t now_ms = HAL_GetTick();
    Drive_ClearMotionCommand();
    Drive_InvalidateActuatorBaseline();
    if (Drive_BootInProgress()) { return; }
    if (s_drive.core.state == DRIVE_STATE_INIT) {
        (void)Drive_DisableBoth();
        return;
    }
    /* STOP admission is asynchronous. Never publish idle/zero from an enqueue.
     * ZERO_START waits for RPDO cancellation, SDO ACK, then bilateral TPDO zero. */
    if (!Drive_StartSafetyTransition(DRIVE_SAFETY_STOP, false, now_ms)) {
        Drive_EnterHardFault(DRIVE_SAFETY_FAILURE_ZERO_SUBMIT, 0U, now_ms);
    }
}

void Drive_RequestCommunicationFaultStop(void)
{
    /* Parser damage and temporary command-link loss discard the damaged
     * frame. Loss of fresh valid CMD_VEL is a watchdog-style soft stop, not
     * a persistent Drive fault latch. A real zero-transaction failure is
     * still promoted to the normal hard-fault cleanup by Drive_CommandStop. */
#ifdef AMR_HOST_TEST
    if (s_communication_fault_stop_request_count != UINT32_MAX) {
        s_communication_fault_stop_request_count++;
    }
#endif
    Drive_CommandStop();
}

CommandResult_t Drive_CommandDisableChecked(void)
{
    DriveDisableResult_t result;

    Drive_ClearMotionCommand();
    s_drive.command.jetson_cmd_active = false;
    result = Drive_DisableBoth();
    if (result == DRIVE_DISABLE_FAILED) {
        /* FAILED means nothing was dispatched (for example a conflicting
         * ENABLE or fault-reset transaction is in flight). The CMD_ACK
         * must report the real cause instead of a hardwired accept. */
        if (s_drive.fault_reset.transaction.active || s_drive.safety.transition.active) {
            return CMD_RESULT_TRANSITION_BUSY;
        }
        return CMD_RESULT_INTERNAL_ERROR;
    }
    return CMD_RESULT_ACCEPTED;
}

bool Drive_CommandDisable(void)
{
    return Drive_CommandDisableChecked() == CMD_RESULT_ACCEPTED;
}

CommandResult_t Drive_CommandEnableChecked(void)
{
    uint32_t now_ms = HAL_GetTick();
    DriveWheelFeedbackPair_t authoritative_pair;

    if (!WheelPdoConfig_Ready(now_ms) ||
        (Canopen_GetWheelFeedbackAuthority() !=
         CANOPEN_WHEEL_AUTHORITY_TPDO) ||
        (Canopen_GetWheelFeedbackTransport() !=
         CANOPEN_WHEEL_FEEDBACK_TPDO_ACTIVE) ||
        !Drive_GetAuthoritativeWheelFeedbackPair(
            now_ms, &authoritative_pair)) {
        uart_printf("DRIVE ENABLE rejected: RPDO/TPDO qualification not ready\r\n");
        return CMD_RESULT_FAULT_LATCHED;
    }
    if (Drive_BootInProgress()) { return CMD_RESULT_TRANSITION_BUSY; }
    if (s_drive.core.enabled &&
        ((s_drive.core.state == DRIVE_STATE_ENABLED_IDLE) ||
         (s_drive.core.state == DRIVE_STATE_RUNNING))) {
        /* Healthy repeated ENABLE is a nonce-acknowledged no-op. It must not
         * restart Heartbeat qualification, resend mode/controlwords, clear
         * any condition, or disturb a live target. */
        uart_printf("DRIVE ENABLE already healthy; accepted no-op\r\n");
        return CMD_RESULT_ACCEPTED;
    }
    if (Drive_LiftIsMoving()) {
        uart_printf("DRIVE ENABLE rejected: lift interlock active\r\n");
        return CMD_RESULT_LIFT_ACTIVE;
    }
    if (s_drive.core.state == DRIVE_STATE_FAULT) {
        uart_printf("DRIVE ENABLE rejected: explicit fault reset required\r\n");
        return CMD_RESULT_FAULT_LATCHED;
    }
    if (s_drive.fault_reset.transaction.active) {
        uart_printf("DRIVE ENABLE rejected: fault reset still pending\r\n");
        return CMD_RESULT_TRANSITION_BUSY;
    }
    if (s_drive.safety.transition.active) {
        uart_printf("DRIVE ENABLE rejected: safety transition still pending\r\n");
        return CMD_RESULT_TRANSITION_BUSY;
    }
    if (s_drive.core.state != DRIVE_STATE_DISABLED) {
        uart_printf("DRIVE ENABLE rejected: drive is not disabled\r\n");
        return CMD_RESULT_NOT_ENABLED;
    }
    if (!Drive_StartSafetyTransition(DRIVE_SAFETY_ENABLE, true, now_ms)) {
        /* Every screened precondition was just re-checked above; an entry
         * refusal here is an unexpected internal state. */
        Drive_ResetYawRateControl(true);
        return CMD_RESULT_INTERNAL_ERROR;
    }
    return CMD_RESULT_ACCEPTED;
}

bool Drive_CommandEnable(void)
{
    return Drive_CommandEnableChecked() == CMD_RESULT_ACCEPTED;
}

static bool Drive_BaseHardwareStaticSafeForLift(void)
{
    CanopenRuntimeDiagnostics_t diag = Canopen_GetRuntimeDiagnostics();

    return !s_drive.safety.transition.active &&
           !s_drive.fault_reset.transaction.active &&
           !DriveTarget_Pending() && WheelRpdoTx_Quiescent() &&
           s_hardware_disable_confirmed &&
           !s_hardware_enable_confirmed &&
           !s_drive.core.enabled &&
           Drive_LastCommandTargetIsZero() &&
           (diag.target_pending_mask == 0U);
}

static bool Drive_BaseHardwareSafeExceptStatusForLift(uint32_t now_ms)
{
    return Drive_BaseHardwareStaticSafeForLift() &&
           Drive_ZeroSpeedEvidenceIsCurrent(now_ms);
}

bool Drive_BaseHardwareSafeForLift(uint32_t now_ms)
{
    return Drive_BaseHardwareSafeExceptStatusForLift(now_ms) &&
           Drive_WheelStatuswordsConfirmDisabled(now_ms);
}

static bool Drive_NormalBaseStopSafeForLift(uint32_t now_ms)
{
    return !s_drive.safety.transition.active &&
           !s_drive.fault_reset.transaction.active &&
           !DriveTarget_Pending() && WheelRpdoTx_Quiescent() &&
           Drive_LastCommandTargetIsZero() &&
           Drive_ZeroSpeedEvidenceIsCurrent(now_ms) &&
           (s_drive.core.state != DRIVE_STATE_FAULT) &&
           !s_tracking_stop_latched &&
           !s_disable_motion_inhibit_latched;
}

static void Drive_ResetLiftStopRequest(uint32_t now_ms)
{
    if (s_drive.lift_stop.request.request_pending) {
        Canopen_CancelAsyncOwner(CANOPEN_SDO_OWNER_SAFETY_CONFIRM,
                                 now_ms);
    }
    memset(&s_drive.lift_stop.request, 0, sizeof(s_drive.lift_stop.request));
}

static bool Drive_LiftStopIsZeroPhase(void)
{
    return (s_drive.lift_stop.request.phase ==
            DRIVE_LIFT_STOP_PHASE_ZERO_SETTLE) ||
           (s_drive.lift_stop.request.phase ==
            DRIVE_LIFT_STOP_PHASE_ZERO_CONFIRM);
}

static uint8_t Drive_LiftStatuswordRefreshNode(void)
{
    if (Drive_LiftStopIsZeroPhase()) {
        if (s_drive.stable_zero.reject_reason ==
            DRIVE_STABLE_ZERO_REJECT_LEFT_SPEED) {
            return LEFT_NODE_ID;
        }
        if (s_drive.stable_zero.reject_reason ==
            DRIVE_STABLE_ZERO_REJECT_RIGHT_SPEED) {
            return RIGHT_NODE_ID;
        }
        if (s_drive.stable_zero.last_left_sequence ==
            s_drive.stable_zero.baseline_left_sequence) {
            return LEFT_NODE_ID;
        }
        if (s_drive.stable_zero.last_right_sequence ==
            s_drive.stable_zero.baseline_right_sequence) {
            return RIGHT_NODE_ID;
        }
        return 0U;
    }
    return (s_drive.lift_stop.request.phase ==
            DRIVE_LIFT_STOP_PHASE_STATUS_LEFT) ?
        LEFT_NODE_ID : RIGHT_NODE_ID;
}

bool Drive_BaseSafeNeedsVelocityPolling(void)
{
    return s_drive.lift_stop.request.active &&
           Drive_LiftStopIsZeroPhase();
}

static DriveConfirmedStopResult_t Drive_FailLiftStatuswordRefresh(
    uint32_t now_ms,
    const char *reason,
    uint8_t node_id)
{
    DriveLiftStopDiag_Report(now_ms, "FAILED", reason, node_id,
                             Drive_BaseHardwareStaticSafeForLift(), true);
    if (s_drive.lift_stop.request.explicit_base_safe_refresh) {
        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_STATUS,
            "DRIVE_BASE_SAFE_REFRESH state=FAILED reason=%s node=%u txn=%lu req=%08lX\r\n",
            reason, (unsigned)node_id,
            (unsigned long)s_drive.lift_stop.request.txn,
            (unsigned long)s_drive.lift_stop.request.request_nonce);
    }
    Drive_ResetLiftStopRequest(now_ms);
    return DRIVE_CONFIRMED_STOP_FAILED;
}

DriveConfirmedStopResult_t Drive_ServiceLiftStatuswordRefresh(
    uint32_t now_ms)
{
    uint8_t node_id;
    uint32_t value = 0U;
    uint32_t abort_code = 0U;
    uint32_t rx_ms = 0U;

    if (!s_drive.lift_stop.request.active ||
        (!Drive_LiftStopIsZeroPhase() &&
         (s_drive.lift_stop.request.phase !=
          DRIVE_LIFT_STOP_PHASE_STATUS_LEFT) &&
         (s_drive.lift_stop.request.phase !=
          DRIVE_LIFT_STOP_PHASE_STATUS_RIGHT))) {
        return Drive_FailLiftStatuswordRefresh(now_ms, "INVALID_PHASE", 0U);
    }
    if (s_drive.lift_stop.request.generation != s_drive.evidence.safety_generation) {
        return Drive_FailLiftStatuswordRefresh(
            now_ms, "GENERATION", Drive_LiftStatuswordRefreshNode());
    }
    if (!Drive_BaseHardwareStaticSafeForLift()) {
        return Drive_FailLiftStatuswordRefresh(
            now_ms, "STATIC_UNSAFE", Drive_LiftStatuswordRefreshNode());
    }
    if (!Drive_LiftStopIsZeroPhase() &&
         !s_drive.lift_stop.request.explicit_base_safe_refresh &&
         !Drive_ZeroSpeedEvidenceIsCurrent(now_ms)) {
        return Drive_FailLiftStatuswordRefresh(
            now_ms, "ZERO_EVIDENCE_LOST", Drive_LiftStatuswordRefreshNode());
    }
    if (!Drive_LiftStopIsZeroPhase() &&
        ((uint32_t)(now_ms - s_drive.lift_stop.request.phase_start_ms) >
         DRIVE_DISABLE_STATUS_TIMEOUT_MS)) {
        return Drive_FailLiftStatuswordRefresh(
            now_ms, "STATUS_TIMEOUT", Drive_LiftStatuswordRefreshNode());
    }

    if (Drive_LiftStopIsZeroPhase()) {
        DriveLiftStopDiag_Report(now_ms, "PENDING", "ZERO_CHECK", 0U,
                                 true, false);
        DriveStableZeroResult_t zero_result =
            Drive_StableZeroUpdate(now_ms);
        uint32_t expected_phase =
            (s_drive.stable_zero.state == DRIVE_STABLE_ZERO_CONFIRMING) ?
            DRIVE_LIFT_STOP_PHASE_ZERO_CONFIRM :
            DRIVE_LIFT_STOP_PHASE_ZERO_SETTLE;

        if (zero_result == DRIVE_STABLE_ZERO_TIMED_OUT) {
            const char *reason = s_drive.stable_zero.overall_elapsed_ms >
                DRIVE_NORMAL_ZERO_OVERALL_TIMEOUT_MS ? "ZERO_OVERALL_TIMEOUT" :
                (s_drive.stable_zero.state == DRIVE_STABLE_ZERO_SETTLING ?
                 "ZERO_SETTLE_TIMEOUT" : "ZERO_CONFIRM_TIMEOUT");
            return Drive_FailLiftStatuswordRefresh(
                now_ms, reason,
                Drive_LiftStatuswordRefreshNode());
        }
        if (zero_result != DRIVE_STABLE_ZERO_COMPLETE) {
            if (s_drive.lift_stop.request.phase != expected_phase) {
                s_drive.lift_stop.request.phase = expected_phase;
                Drive_ReportBaseSafeRefreshPending();
            }
            return DRIVE_CONFIRMED_STOP_PENDING;
        }
        Drive_RecordZeroSpeedEvidence(
            s_drive.lift_stop.request.generation,
            now_ms, DRIVE_ZERO_EVIDENCE_DISABLE);
        if (!Drive_ZeroSpeedEvidenceIsCurrent(now_ms)) {
            return Drive_FailLiftStatuswordRefresh(
                now_ms, "ZERO_RECORD_REJECTED", 0U);
        }
        s_drive.lift_stop.request.phase =
            DRIVE_LIFT_STOP_PHASE_STATUS_LEFT;
        s_drive.lift_stop.request.phase_start_ms = now_ms;
        Drive_ReportBaseSafeRefreshPending();
        return DRIVE_CONFIRMED_STOP_PENDING;
    }

    node_id = Drive_LiftStatuswordRefreshNode();
    DriveLiftStopDiag_Report(now_ms, "PENDING", "STATUS_READ", node_id,
                             true, false);
    if (!s_drive.lift_stop.request.request_pending) {
        CanopenAsyncStartResult_t start_result = Canopen_AsyncReadU16(
            node_id, 0x6041U, 0x00U,
            CANOPEN_SDO_OWNER_SAFETY_CONFIRM,
            now_ms, DRIVE_SAFETY_SDO_TIMEOUT_MS);

        if (start_result == CANOPEN_ASYNC_START_BUSY) {
            return DRIVE_CONFIRMED_STOP_PENDING;
        }
        if (start_result != CANOPEN_ASYNC_START_OK) {
            return Drive_FailLiftStatuswordRefresh(
                now_ms, "SDO_START", node_id);
        }
        s_drive.lift_stop.request.request_pending = true;
        return DRIVE_CONFIRMED_STOP_PENDING;
    }

    {
        CanopenAsyncResult_t poll_result = Canopen_AsyncPollWithTimestamp(
            node_id, CANOPEN_SDO_OWNER_SAFETY_CONFIRM,
            now_ms, &value, &abort_code, &rx_ms);

        if (poll_result == CANOPEN_ASYNC_RESULT_PENDING) {
            return DRIVE_CONFIRMED_STOP_PENDING;
        }
        s_drive.lift_stop.request.request_pending = false;
        if (poll_result != CANOPEN_ASYNC_RESULT_SUCCESS) {
            const char *reason = "PROTOCOL";

            JetsonTx_Printf(JETSON_TX_PRIORITY_STATUS,
                "DRIVE_LIFT_STOP_SDO node=%u result=%u abort=%08lX gen=%lu\r\n",
                (unsigned)node_id, (unsigned)poll_result, (unsigned long)abort_code,
                (unsigned long)s_drive.lift_stop.request.generation);
            if (poll_result == CANOPEN_ASYNC_RESULT_TIMEOUT) {
                reason = "SDO_TIMEOUT";
            } else if (poll_result == CANOPEN_ASYNC_RESULT_ABORT) {
                reason = "SDO_ABORT";
            }
            return Drive_FailLiftStatuswordRefresh(
                now_ms, reason, node_id);
        }
    }

    Drive_RecordStatusword(node_id, (uint16_t)value, rx_ms,
                           s_drive.lift_stop.request.generation);
    if (node_id == LEFT_NODE_ID) {
        s_drive.lift_stop.request.phase = DRIVE_LIFT_STOP_PHASE_STATUS_RIGHT;
        if (s_drive.lift_stop.request.explicit_base_safe_refresh) {
            Drive_ReportBaseSafeRefreshPending();
        }
        return DRIVE_CONFIRMED_STOP_PENDING;
    }

    {
        bool safe = Drive_BaseHardwareSafeForLift(now_ms);
        bool explicit_refresh =
            s_drive.lift_stop.request.explicit_base_safe_refresh;
        uint32_t generation = s_drive.lift_stop.request.generation;

        const char *reason = safe ? "CONFIRMED" :
            (!Drive_WheelStatuswordBitsConfirmDisabled() ? "STATUS_NOT_DISABLED" :
             (!Drive_WheelStatuswordsConfirmDisabled(now_ms) ? "STATUS_STALE" :
              "ZERO_EVIDENCE_LOST"));
        DriveLiftStopDiag_Report(now_ms, safe ? "COMPLETE" : "FAILED", reason, 0U,
            Drive_BaseHardwareStaticSafeForLift(), true);

        if (explicit_refresh) {
            (void)JetsonTx_Printf(
                JETSON_TX_PRIORITY_STATUS,
                "DRIVE_BASE_SAFE_REFRESH state=COMPLETE safe=%u gen=%lu txn=%lu req=%08lX\r\n",
                safe ? 1U : 0U, (unsigned long)generation,
                (unsigned long)s_drive.lift_stop.request.txn,
                (unsigned long)s_drive.lift_stop.request.request_nonce);
        }
        Drive_ResetLiftStopRequest(now_ms);
        return safe ? DRIVE_CONFIRMED_STOP_OK :
                      DRIVE_CONFIRMED_STOP_FAILED;
    }
}

static DriveConfirmedStopResult_t Drive_StartLiftStatuswordRefresh(
    uint32_t now_ms,
    bool explicit_base_safe_refresh,
    uint32_t txn,
    uint32_t request_nonce)
{
    memset(&s_drive.lift_stop.request, 0, sizeof(s_drive.lift_stop.request));
    s_drive.lift_stop.request.active = true;
    s_drive.lift_stop.request.phase =
        !Drive_ZeroSpeedEvidenceIsCurrent(now_ms) ?
        DRIVE_LIFT_STOP_PHASE_ZERO_SETTLE :
        DRIVE_LIFT_STOP_PHASE_STATUS_LEFT;
    s_drive.lift_stop.request.generation = s_drive.evidence.safety_generation;
    s_drive.lift_stop.request.start_ms = now_ms;
    s_drive.lift_stop.request.phase_start_ms = now_ms;
    s_drive.lift_stop.request.txn = txn;
    s_drive.lift_stop.request.request_nonce = request_nonce;
    s_drive.lift_stop.request.explicit_base_safe_refresh =
        explicit_base_safe_refresh;
    if (Drive_LiftStopIsZeroPhase()) {
        Drive_StableZeroStart(now_ms, true);
    }
    DriveLiftStopDiag_Reset();
    DriveLiftStopDiag_Report(now_ms, "STARTED", "EVIDENCE_REFRESH", 0U,
                             Drive_BaseHardwareStaticSafeForLift(), true);
    return Drive_ServiceLiftStatuswordRefresh(now_ms);
}

bool Drive_RequestBaseSafeRefresh(uint32_t request_nonce)
{
    uint32_t now_ms = HAL_GetTick();
    uint32_t txn = ++s_drive.lift_stop.base_safe_refresh_txn;
    DriveConfirmedStopResult_t result;

    /* Every command outcome — BUSY, immediate COMPLETE, PRECONDITION, or
     * asynchronous refresh — is bound to this command by a per-command
     * monotonic txn= plus the echoed host req=. gen= remains the safety
     * generation of the evidence, which two different transactions may
     * legitimately share. */
    if (s_drive.lift_stop.request.active) {
        /* A refresh or lift confirmed-stop transaction is already in
         * flight. This command dispatches nothing; report an honest,
         * bound BUSY failure without touching the active transaction. */
        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_STATUS,
            "DRIVE_BASE_SAFE_REFRESH state=FAILED reason=BUSY active_txn=%lu txn=%lu req=%08lX\r\n",
            (unsigned long)s_drive.lift_stop.request.txn,
            (unsigned long)txn, (unsigned long)request_nonce);
        return false;
    }
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_STATUS,
        "DRIVE_BASE_SAFE_REFRESH state=STARTED gen=%lu txn=%lu req=%08lX\r\n",
        (unsigned long)s_drive.evidence.safety_generation,
        (unsigned long)txn, (unsigned long)request_nonce);
    if (Drive_BaseHardwareSafeForLift(now_ms)) {
        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_STATUS,
            "DRIVE_BASE_SAFE_REFRESH state=COMPLETE safe=1 gen=%lu txn=%lu req=%08lX\r\n",
            (unsigned long)s_drive.evidence.safety_generation,
            (unsigned long)txn, (unsigned long)request_nonce);
        return true;
    }
    if (!Drive_BaseHardwareStaticSafeForLift()) {
        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_STATUS,
            "DRIVE_BASE_SAFE_REFRESH state=FAILED reason=PRECONDITION node=0 txn=%lu req=%08lX\r\n",
            (unsigned long)txn, (unsigned long)request_nonce);
        return false;
    }

    result = Drive_StartLiftStatuswordRefresh(now_ms, true, txn,
                                              request_nonce);
    if (result == DRIVE_CONFIRMED_STOP_PENDING) {
        Drive_ReportBaseSafeRefreshPending();
    }
    return result != DRIVE_CONFIRMED_STOP_FAILED;
}

DriveConfirmedStopResult_t Drive_RequestConfirmedStopForLift(void)
{
    uint32_t now_ms = HAL_GetTick();
    DriveStopResult_t stop_result;

    if (!s_drive.lift_stop.request.active &&
        (Drive_BaseHardwareSafeForLift(now_ms) ||
         Drive_NormalBaseStopSafeForLift(now_ms))) {
        DriveLiftStopDiag_Reset();
        DriveLiftStopDiag_Report(now_ms, "COMPLETE", "CURRENT_EVIDENCE", 0U,
                                 Drive_BaseHardwareStaticSafeForLift(), true);
        return DRIVE_CONFIRMED_STOP_OK;
    }

    if (s_drive.lift_stop.request.active) {
        if (s_drive.lift_stop.request.explicit_base_safe_refresh) {
            return DRIVE_CONFIRMED_STOP_PENDING;
        }
        if (Drive_LiftStopIsZeroPhase() ||
            (s_drive.lift_stop.request.phase ==
             DRIVE_LIFT_STOP_PHASE_STATUS_LEFT) ||
            (s_drive.lift_stop.request.phase ==
             DRIVE_LIFT_STOP_PHASE_STATUS_RIGHT)) {
            return Drive_ServiceLiftStatuswordRefresh(now_ms);
        }
        if (s_drive.safety.transition.active) {
            if (s_drive.safety.transition.generation != s_drive.lift_stop.request.generation) {
                return Drive_FailLiftStatuswordRefresh(now_ms, "SAFETY_GENERATION", 0U);
            }
            DriveLiftStopDiag_Report(now_ms, "PENDING", "SAFETY_TRANSITION", 0U,
                                     Drive_BaseHardwareStaticSafeForLift(), false);
            return DRIVE_CONFIRMED_STOP_PENDING;
        }
        {
            uint32_t request_generation = s_drive.lift_stop.request.generation;
            bool generation_matches =
                (s_drive.safety.transition.generation ==
                 request_generation) &&
                (s_drive.evidence.safety_generation ==
                 request_generation);
            bool matching_disable_completed =
                generation_matches &&
                (s_drive.lift_stop.request.phase ==
                 DRIVE_LIFT_STOP_PHASE_WAIT_SAFETY) &&
                (s_drive.safety.transition.type == DRIVE_SAFETY_DISABLE) &&
                (s_drive.core.state == DRIVE_STATE_DISABLED) &&
                s_hardware_disable_confirmed &&
                Drive_BaseHardwareStaticSafeForLift();

            /* A Lift request may overlap the real DISABLE transaction. Its
             * pre-disable zero proof is deliberately retired when disabled
             * statuswords complete, so hand the same implicit request into
             * the shared post-disable settle/confirm/statusword flow. */
            if (matching_disable_completed) {
                return Drive_StartLiftStatuswordRefresh(
                    now_ms, false, 0U, 0U);
            }

            if (generation_matches &&
                (s_drive.evidence.zero_speed.generation ==
                 request_generation) &&
                (Drive_BaseHardwareSafeForLift(now_ms) ||
                 Drive_NormalBaseStopSafeForLift(now_ms))) {
                DriveLiftStopDiag_Report(now_ms, "COMPLETE", "SAFETY_CONFIRMED", 0U,
                                         Drive_BaseHardwareStaticSafeForLift(), true);
                Drive_ResetLiftStopRequest(now_ms);
                return DRIVE_CONFIRMED_STOP_OK;
            }
            return Drive_FailLiftStatuswordRefresh(now_ms,
                generation_matches ? "STOP_EVIDENCE_LOST" : "SAFETY_GENERATION", 0U);
        }
    }

    if (Drive_BaseHardwareStaticSafeForLift()) {
        return Drive_StartLiftStatuswordRefresh(now_ms, false, 0U, 0U);
    }

    if ((s_drive.core.state == DRIVE_STATE_FAULT) &&
        !s_drive.safety.transition.active) {
        return Drive_FailLiftStatuswordRefresh(now_ms, "DRIVE_FAULT", 0U);
    }

    Drive_ClearMotionCommand();
    s_drive.command.jetson_cmd_active = false;
    stop_result = Drive_StopBoth();
    if ((stop_result == DRIVE_STOP_FAILED) ||
        !s_drive.safety.transition.active) {
        return Drive_FailLiftStatuswordRefresh(now_ms, "STOP_START_FAILED", 0U);
    }
    s_drive.lift_stop.request.active = true;
    s_drive.lift_stop.request.phase = DRIVE_LIFT_STOP_PHASE_WAIT_SAFETY;
    s_drive.lift_stop.request.generation = s_drive.safety.transition.generation;
    s_drive.lift_stop.request.start_ms = now_ms;
    DriveLiftStopDiag_Reset();
    DriveLiftStopDiag_Report(now_ms, "STARTED", "SAFETY_TRANSITION", 0U,
                             Drive_BaseHardwareStaticSafeForLift(), true);
    return DRIVE_CONFIRMED_STOP_PENDING;
}

void Drive_CancelConfirmedStopForLift(void)
{
    Drive_ResetLiftStopRequest(HAL_GetTick());
}

void Drive_LatchHeartbeatFault(void)
{
    Drive_InvalidateZeroSpeedEvidence();
    Drive_MarkStatusFeedbackInvalid();
    DriveCore_SetSoftwareEnabled(false);
    DriveCore_SetHardwareEnableConfirmed(false);
    DriveCore_SetHardwareDisableConfirmed(false);
    DriveCore_SetTrackingStopLatched(true);
    DriveCore_SetMotionInhibitLatched(true);
    DriveCore_SetTrackingStopIgnoreLogged(false);
    Drive_ClearMotionCommandStateOnly();
    Drive_ResetYawRateControl(true);
}

#ifdef AMR_HOST_TEST
void Drive_TestServiceStopConfirmation(uint32_t now_ms)
{
    Drive_SafetyTransitionStep(now_ms);
}
#endif

#ifdef AMR_HOST_TEST
bool Drive_TestStopIsPending(void)
{
    return s_drive.safety.transition.active &&
        ((s_drive.safety.transition.type == DRIVE_SAFETY_STOP) ||
         (s_drive.safety.transition.type == DRIVE_SAFETY_DISABLE));
}
#endif

#ifdef AMR_HOST_TEST
bool Drive_TestSafetyTransitionActive(void)
{
    return s_drive.safety.transition.active;
}
#endif

#ifdef AMR_HOST_TEST
uint32_t Drive_TestSafetyTransitionType(void)
{
    return (uint32_t)s_drive.safety.transition.type;
}
#endif

#ifdef AMR_HOST_TEST
uint32_t Drive_TestSafetyTransitionPhase(void)
{
    return (uint32_t)s_drive.safety.transition.phase;
}
#endif

#ifdef AMR_HOST_TEST
uint32_t Drive_TestSafetyTransitionGeneration(void)
{
    return s_drive.safety.transition.generation;
}
#endif

#ifdef AMR_HOST_TEST
uint32_t Drive_TestSafetyTransientTimeoutCount(void)
{
    return s_drive.safety.transition.transient_timeout_count;
}
#endif

#ifdef AMR_HOST_TEST
uint32_t Drive_TestStableZeroSampleCount(void)
{
    return s_drive.stable_zero.stable_sample_count;
}

uint32_t Drive_TestStableZeroState(void)
{
    return (uint32_t)s_drive.stable_zero.state;
}

uint32_t Drive_TestStableZeroRejectReason(void)
{
    return (uint32_t)s_drive.stable_zero.reject_reason;
}
#endif

#ifdef AMR_HOST_TEST
bool Drive_TestHardwareEnableConfirmed(void)
{
    return s_hardware_enable_confirmed;
}
#endif

#ifdef AMR_HOST_TEST
bool Drive_TestHardwareDisableConfirmed(void)
{
    return s_hardware_disable_confirmed;
}
#endif

#ifdef AMR_HOST_TEST
void Drive_TestSetHardwareDisableConfirmed(bool confirmed)
{
    DriveCore_SetHardwareDisableConfirmed(confirmed);
}
#endif

#ifdef AMR_HOST_TEST
void Drive_TestSetLastCommandTargetDec(int32_t left, int32_t right)
{
    s_drive.command.last_left_cmd_dec = left;
    s_drive.command.last_right_cmd_dec = right;
}
#endif

#ifdef AMR_HOST_TEST
void Drive_TestSetStatusword(uint8_t node_id,
                             uint16_t statusword,
                             bool valid)
{
    if (valid) {
        Drive_RecordStatusword(node_id, statusword, HAL_GetTick(),
                               s_drive.evidence.safety_generation);
        return;
    }
    if (node_id == LEFT_NODE_ID) {
        s_drive.statusword.left_statusword_valid = false;
    } else if (node_id == RIGHT_NODE_ID) {
        s_drive.statusword.right_statusword_valid = false;
    }
}
#endif

#ifdef AMR_HOST_TEST
void Drive_TestSetStatuswordTimestamp(uint8_t node_id,
                                      uint32_t timestamp_ms)
{
    if (node_id == LEFT_NODE_ID) {
        s_drive.statusword.left_statusword_timestamp_ms = timestamp_ms;
    } else if (node_id == RIGHT_NODE_ID) {
        s_drive.statusword.right_statusword_timestamp_ms = timestamp_ms;
    }
}
#endif

#ifdef AMR_HOST_TEST
void Drive_TestSetStatuswordGeneration(uint8_t node_id,
                                       uint32_t generation)
{
    if (node_id == LEFT_NODE_ID) {
        s_drive.statusword.left_statusword_generation = generation;
    } else if (node_id == RIGHT_NODE_ID) {
        s_drive.statusword.right_statusword_generation = generation;
    }
}
#endif

#ifdef AMR_HOST_TEST
bool Drive_TestBaseHardwareSafeForLift(uint32_t now_ms)
{
    return Drive_BaseHardwareSafeForLift(now_ms);
}
#endif

#ifdef AMR_HOST_TEST
bool Drive_TestStatuswordsConfirmDisabled(uint32_t now_ms)
{
    return Drive_WheelStatuswordsConfirmDisabled(now_ms);
}

bool Drive_TestPostDisableZeroProofRequired(void)
{
    return s_drive.evidence.post_disable_zero_proof_required;
}
#endif

#ifdef AMR_HOST_TEST
bool Drive_TestBaseSafeRefreshActive(void)
{
    return s_drive.lift_stop.request.active &&
           s_drive.lift_stop.request.explicit_base_safe_refresh;
}

DriveConfirmedStopResult_t Drive_TestServiceLiftStopRequest(uint32_t now_ms)
{
    return Drive_ServiceLiftStatuswordRefresh(now_ms);
}
#endif

#ifdef AMR_HOST_TEST
bool Drive_TestZeroEvidenceValid(void)
{
    return Drive_ZeroSpeedEvidenceIsCurrent(HAL_GetTick());
}
#endif

#ifdef AMR_HOST_TEST
uint32_t Drive_TestZeroEvidenceGeneration(void)
{
    return s_drive.evidence.zero_speed.generation;
}
#endif

#ifdef AMR_HOST_TEST
uint32_t Drive_TestZeroEvidenceReason(void)
{
    return (uint32_t)s_drive.evidence.zero_speed.reason;
}
#endif
