/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : drive_control.c
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

bool s_yaw_pi_enabled =
    (DRIVE_YAW_RATE_PI_DEFAULT_ENABLED != 0U);

const MotionYawRatePIConfig_t s_yaw_rate_pi_config = {
    .kp = DRIVE_YAW_RATE_KP,
    .ki = DRIVE_YAW_RATE_KI,
    .tracking_gain = DRIVE_YAW_RATE_TRACKING_GAIN,
    .integral_limit_mrad = DRIVE_YAW_RATE_INTEGRAL_LIMIT_MRAD,
    .correction_limit_mrad_s = DRIVE_YAW_RATE_CORRECTION_LIMIT_MRAD_S,
    .zero_target_threshold_mrad_s = DRIVE_YAW_RATE_ZERO_THRESHOLD_MRAD_S,
    .zero_target_exit_threshold_mrad_s = DRIVE_YAW_RATE_ZERO_EXIT_MRAD_S,
    .zero_yaw_kp = DRIVE_ZERO_YAW_KP,
    .zero_yaw_correction_limit_mrad_s =
        DRIVE_ZERO_YAW_CORRECTION_LIMIT_MRAD_S,
    .zero_yaw_feedback_deadband_mrad_s =
        DRIVE_ZERO_YAW_FEEDBACK_DEADBAND_MRAD_S,
    .stopped_target_threshold_mrad_s =
        DRIVE_ZERO_YAW_STOP_TARGET_MRAD_S,
    .stopped_linear_threshold_mm_s = DRIVE_ZERO_YAW_STOP_LINEAR_MM_S,
    .stopped_wheel_threshold_mm_s = DRIVE_ZERO_YAW_STOP_WHEEL_MM_S,
    .minimum_feedback_dt_ms = DRIVE_YAW_RATE_MIN_DT_MS,
    .maximum_feedback_dt_ms = DRIVE_YAW_RATE_MAX_DT_MS,
    .reentry_valid_updates = DRIVE_YAW_RATE_REENTRY_VALID_UPDATES
};
static const MotionTrackingConfig_t s_tracking_config = {
    .command_step_threshold_mrad_s = DRIVE_TRACKING_COMMAND_STEP_MRAD_S,
    .active_target_threshold_mrad_s = DRIVE_TRACKING_ACTIVE_TARGET_MRAD_S,
    .large_error_threshold_mrad_s = DRIVE_TRACKING_LARGE_ERROR_MRAD_S,
    .wrong_sign_actual_threshold_mrad_s =
        DRIVE_TRACKING_WRONG_SIGN_ACTUAL_MRAD_S,
    .divergence_rise_threshold_mrad_s =
        DRIVE_TRACKING_DIVERGENCE_RISE_MRAD_S,
    .submitted_target_step_threshold_mrad_s =
        DRIVE_TRACKING_SUBMITTED_STEP_MRAD_S,
    .grace_ms = DRIVE_TRACKING_GRACE_MS,
    .large_error_duration_ms = DRIVE_TRACKING_LARGE_ERROR_DURATION_MS,
    .saturation_duration_ms = DRIVE_TRACKING_SATURATION_DURATION_MS,
    .large_error_consecutive_limit =
        DRIVE_TRACKING_LARGE_ERROR_CONSECUTIVE,
    .wrong_sign_consecutive_limit =
        DRIVE_TRACKING_WRONG_SIGN_CONSECUTIVE,
    .divergence_consecutive_limit =
        DRIVE_TRACKING_DIVERGENCE_CONSECUTIVE,
    .recovery_consecutive_limit = DRIVE_TRACKING_RECOVERY_CONSECUTIVE
};










void Drive_EnsureMotionControlInitialized(void)
{
    if (s_drive.control.actuator.initialized) {
        return;
    }

    Motion_FeedbackInit(&s_drive.feedback.motion);
    Motion_YawRatePIInit(&s_drive.control.yaw.pi);
    Motion_TrackingSupervisorInit(&s_drive.control.tracking.supervisor);
    Motion_ActuatorStateInit(&s_drive.control.actuator.state);
    Motion_CalculateCommandEnvelope((float)MAX_TEST_MOTOR_RPM,
                                    DRIVE_YAW_RATE_CORRECTION_LIMIT_MRAD_S,
                                    &s_drive.control.actuator.command_envelope);
    s_drive.control.actuator.initialized = true;
}

static void Drive_CacheStatusFeedback(uint8_t node_id,
                                      const CanopenValueSample_t *sample)
{
    int32_t logical_rpm;

    if ((sample == NULL) || !sample->valid) {
        return;
    }
    logical_rpm = Motion_RoundFloatToI32(
        Motion_RawVelocityDecToLogicalMotorRPM(node_id, sample->value));
    if (node_id == LEFT_NODE_ID) {
        s_drive.feedback.status_left_rpm = logical_rpm;
        s_drive.feedback.status_left_timestamp_ms = sample->timestamp_ms;
        s_drive.feedback.status_left_seen = true;
        s_drive.feedback.status_left_valid = true;
    } else if (node_id == RIGHT_NODE_ID) {
        s_drive.feedback.status_right_rpm = logical_rpm;
        s_drive.feedback.status_right_timestamp_ms = sample->timestamp_ms;
        s_drive.feedback.status_right_seen = true;
        s_drive.feedback.status_right_valid = true;
    }
}

bool Drive_GetAuthoritativeWheelFeedbackPair(
    uint32_t now_ms,
    DriveWheelFeedbackPair_t *pair)
{
    const MotionChassisFeedback_t *feedback = &s_drive.feedback.chassis;
    CanopenWheelFeedbackAuthority_t authority =
        Canopen_GetWheelFeedbackAuthority();

    if (pair == NULL) {
        return false;
    }
    memset(pair, 0, sizeof(*pair));
    pair->authority_source =
        (authority == CANOPEN_WHEEL_AUTHORITY_TPDO) ?
        DRIVE_WHEEL_AUTHORITY_TPDO : DRIVE_WHEEL_AUTHORITY_SDO;
    pair->left_raw_velocity = feedback->left.raw_velocity_dec;
    pair->right_raw_velocity = feedback->right.raw_velocity_dec;
    pair->normalized_left_velocity_mm_s =
        feedback->left.wheel_velocity_mm_s;
    pair->normalized_right_velocity_mm_s =
        feedback->right.wheel_velocity_mm_s;
    pair->pair_timestamp_ms = feedback->timestamp_ms;
    pair->pair_age_ms = feedback->valid ?
        (uint32_t)(now_ms - feedback->timestamp_ms) : UINT32_MAX;
    pair->pair_sequence = feedback->sequence;
    pair->left_node_sequence = feedback->left.sequence;
    pair->right_node_sequence = feedback->right.sequence;
    pair->pair_skew_ms = feedback->pair_skew_ms;
    pair->pair_valid = feedback->valid && !feedback->stale &&
        (pair->pair_age_ms <= DRIVE_WHEEL_FEEDBACK_STALE_MS) &&
        (feedback->pair_skew_ms <= DRIVE_WHEEL_FEEDBACK_MAX_SKEW_MS);
    pair->pair_state = pair->pair_valid ? DRIVE_WHEEL_PAIR_FRESH :
        ((authority == CANOPEN_WHEEL_AUTHORITY_TPDO) ?
         DRIVE_WHEEL_PAIR_FAULT :
         ((feedback->left.valid || feedback->right.valid) ?
          DRIVE_WHEEL_PAIR_STALE : DRIVE_WHEEL_PAIR_NO_DATA));
    return pair->pair_valid;
}

void Drive_MarkStatusFeedbackInvalid(void)
{
    s_drive.feedback.status_left_valid = false;
    s_drive.feedback.status_right_valid = false;
}

void Drive_ResetYawRateControl(bool invalidate_feedback)
{
    Drive_EnsureMotionControlInitialized();
    Motion_YawRatePIReset(&s_drive.control.yaw.pi);
    memset(&s_drive.control.yaw.proposal, 0, sizeof(s_drive.control.yaw.proposal));
    s_drive.feedback.pair_pending = false;
    s_drive.control.yaw.corrected_angular_mrad_s = 0.0f;
    s_drive.control.yaw.submitted_correction_mrad_s = 0.0f;
    if (!s_tracking_stop_latched) {
        Motion_TrackingSupervisorReset(&s_drive.control.tracking.supervisor, true);
    }
    if (invalidate_feedback) {
        Drive_InvalidateZeroSpeedEvidence();
        Motion_FeedbackInvalidate(&s_drive.feedback.motion);
        Drive_MarkStatusFeedbackInvalid();
        s_drive.feedback.issue_active = false;
        memset(&s_drive.feedback.chassis, 0, sizeof(s_drive.feedback.chassis));
        s_drive.feedback.chassis.stale = true;
    }
}

static void Drive_CommitKnownActuatorTarget(float left_motor_rpm,
                                            float right_motor_rpm,
                                            int32_t left_velocity_dec,
                                            int32_t right_velocity_dec,
                                            uint32_t timestamp_ms)
{
    MotionActuatorProposal_t committed = {0};
    float left_wheel_mm_s =
        Motion_LogicalMotorRPMToWheelVelocityMmS(left_motor_rpm);
    float right_wheel_mm_s =
        Motion_LogicalMotorRPMToWheelVelocityMmS(right_motor_rpm);

    committed.left_motor_rpm = left_motor_rpm;
    committed.right_motor_rpm = right_motor_rpm;
    committed.left_velocity_dec = left_velocity_dec;
    committed.right_velocity_dec = right_velocity_dec;
    committed.feasible_linear_mm_s =
        (left_wheel_mm_s + right_wheel_mm_s) * 0.5f;
    committed.feasible_angular_mrad_s =
        Motion_WheelVelocitiesToAngularMradS(left_wheel_mm_s,
                                             right_wheel_mm_s);
    Motion_ActuatorCommit(&s_drive.control.actuator.state, &committed, timestamp_ms);
}

static void Drive_UpdateWheelFeedback(
    uint32_t now_ms,
    CanopenWheelFeedbackSchedule_t schedule)
{
    CanopenValueSample_t left_sample;
    CanopenValueSample_t right_sample;
    CanopenValueSample_t left_position;
    CanopenValueSample_t right_position;
    MotionChassisFeedback_t evaluated_feedback;

    Canopen_WheelRuntimeService(now_ms, schedule);

    if (Canopen_GetWheelVelocitySample(LEFT_NODE_ID, &left_sample) &&
        (left_sample.sequence != s_drive.feedback.motion.left.sequence)) {
        Drive_CacheStatusFeedback(LEFT_NODE_ID, &left_sample);
        Motion_FeedbackUpdateRawVelocity(&s_drive.feedback.motion,
                                         LEFT_NODE_ID,
                                         left_sample.value,
                                         left_sample.timestamp_ms,
                                         left_sample.sequence);
    }
    if (Canopen_GetWheelVelocitySample(RIGHT_NODE_ID, &right_sample) &&
        (right_sample.sequence != s_drive.feedback.motion.right.sequence)) {
        Drive_CacheStatusFeedback(RIGHT_NODE_ID, &right_sample);
        Motion_FeedbackUpdateRawVelocity(&s_drive.feedback.motion,
                                         RIGHT_NODE_ID,
                                         right_sample.value,
                                         right_sample.timestamp_ms,
                                         right_sample.sequence);
    }

    evaluated_feedback = Motion_FeedbackEvaluate(
        &s_drive.feedback.motion,
        now_ms,
        DRIVE_WHEEL_FEEDBACK_STALE_MS,
        DRIVE_WHEEL_FEEDBACK_MAX_SKEW_MS);
    if (evaluated_feedback.new_sample) {
        s_drive.control.yaw.waiting_for_new_pair = false;
        if (s_drive.feedback.pair_pending &&
            (evaluated_feedback.sequence != s_drive.feedback.chassis.sequence) &&
            (s_drive.control.yaw.feedback_pair_replaced_count != UINT32_MAX)) {
            s_drive.control.yaw.feedback_pair_replaced_count++;
        }
        s_drive.feedback.pair_pending = true;
    }
    s_drive.feedback.chassis = evaluated_feedback;
    Drive_MaintainZeroSpeedEvidence(now_ms);

    if ((Canopen_GetWheelFeedbackAuthority() ==
         CANOPEN_WHEEL_AUTHORITY_TPDO) &&
        evaluated_feedback.valid && !evaluated_feedback.stale &&
        evaluated_feedback.new_sample &&
        (evaluated_feedback.sequence !=
         s_drive.feedback.odom_authoritative_pair_sequence)) {
        if (s_drive.feedback.odom_authoritative_pair_sequence != 0U) {
            uint32_t dt_ms = (uint32_t)(
                evaluated_feedback.timestamp_ms -
                s_drive.feedback.odom_authoritative_pair_timestamp_ms);

            Odom_UpdateFromWheelVelocities(
                evaluated_feedback.left.wheel_velocity_mm_s,
                evaluated_feedback.right.wheel_velocity_mm_s,
                dt_ms, false);
        }
        s_drive.feedback.odom_authoritative_pair_sequence =
            evaluated_feedback.sequence;
        s_drive.feedback.odom_authoritative_pair_timestamp_ms =
            evaluated_feedback.timestamp_ms;
    } else if ((Canopen_GetWheelFeedbackAuthority() ==
                CANOPEN_WHEEL_AUTHORITY_SDO) &&
        g_odom.initialized &&
        Canopen_GetWheelPositionSample(LEFT_NODE_ID, &left_position) &&
        Canopen_GetWheelPositionSample(RIGHT_NODE_ID, &right_position) &&
        (left_position.sequence != s_drive.feedback.odom_left_position_sequence) &&
        (right_position.sequence != s_drive.feedback.odom_right_position_sequence)) {
        int32_t timestamp_delta =
            (int32_t)(left_position.timestamp_ms - right_position.timestamp_ms);
        uint32_t timestamp_skew = (timestamp_delta >= 0) ?
            (uint32_t)timestamp_delta : (uint32_t)(-(int64_t)timestamp_delta);

        if (timestamp_skew <= DRIVE_WHEEL_FEEDBACK_MAX_SKEW_MS) {
            Odom_UpdateFromRawPositions(left_position.value,
                                        right_position.value,
                                        false);
            s_drive.feedback.odom_left_position_sequence = left_position.sequence;
            s_drive.feedback.odom_right_position_sequence = right_position.sequence;
        }
    }
}

#if ENABLE_JETSON_UART_MODE
static bool Drive_YawControlAllowed(void)
{
    return s_yaw_pi_enabled && s_drive.core.enabled &&
           (s_drive.core.state != DRIVE_STATE_FAULT) &&
           (s_drive.core.state != DRIVE_STATE_DISABLED) &&
           (s_drive.core.state != DRIVE_STATE_DISABLING) &&
           !Drive_LiftIsMoving() && !s_tracking_stop_latched &&
           !s_disable_motion_inhibit_latched;
}
#endif

#if ENABLE_JETSON_UART_MODE
static void Drive_ProcessFeedbackHealth(void)
{
    bool rejected_stale_candidate;
    bool rejected_skew_candidate;

    rejected_stale_candidate =
        (s_drive.feedback.motion.rejected_stale_pair_count !=
         s_drive.feedback.observed_stale_pair_reject_count);
    rejected_skew_candidate =
        (s_drive.feedback.motion.rejected_skew_pair_count !=
         s_drive.feedback.observed_skew_pair_reject_count);
    s_drive.feedback.observed_stale_pair_reject_count =
        s_drive.feedback.motion.rejected_stale_pair_count;
    s_drive.feedback.observed_skew_pair_reject_count =
        s_drive.feedback.motion.rejected_skew_pair_count;
    if (rejected_stale_candidate || rejected_skew_candidate) {
        Drive_InvalidateZeroSpeedEvidence();
        if (s_drive.safety.transition.active) {
            s_drive.safety.transition.left_start_sequence =
                s_drive.feedback.chassis.left.sequence;
            s_drive.safety.transition.right_start_sequence =
                s_drive.feedback.chassis.right.sequence;
        }
        Motion_TrackingSupervisorObserveFeedbackIssue(
            &s_drive.control.tracking.supervisor,
            rejected_stale_candidate,
            rejected_skew_candidate);
        s_drive.feedback.issue_active = true;
    }

    if (!s_drive.feedback.chassis.valid || s_drive.feedback.chassis.stale) {
        if (!s_drive.feedback.issue_active) {
            Motion_TrackingSupervisorObserveFeedbackIssue(
                &s_drive.control.tracking.supervisor,
                s_drive.feedback.chassis.stale,
                false);
        }
        s_drive.feedback.issue_active = true;
        if (s_drive.control.yaw.pi.feedback_sequence_valid || s_drive.control.yaw.pi.active) {
            Motion_YawRatePIReset(&s_drive.control.yaw.pi);
            s_drive.feedback.pair_pending = false;
            s_drive.control.yaw.corrected_angular_mrad_s =
                s_drive.control.nominal_feasible_angular_mrad_s;
            s_drive.control.yaw.submitted_correction_mrad_s = 0.0f;
        }
        if (!s_tracking_stop_latched) {
            Motion_TrackingSupervisorReset(&s_drive.control.tracking.supervisor, true);
        }
        return;
    }
    s_drive.feedback.issue_active = false;
}
#endif

bool Kinco_ReadActualPosition(uint8_t node_id, int32_t *pos_out)
{
    return SDO_Read_I32(node_id, 0x6063, 0x00, pos_out);
}

bool Kinco_ReadActualSpeed(uint8_t node_id, int32_t *speed_out)
{
    return SDO_Read_I32(node_id, 0x606C, 0x00, speed_out);
}

static bool Kinco_ReadActualPositionQuiet(uint8_t node_id, int32_t *pos_out)
{
    return SDO_Read_I32_Quiet(node_id, 0x6063, 0x00, pos_out);
}

static int32_t clamp_i32(int32_t v, int32_t min_v, int32_t max_v)
{
    if (v < min_v) {
        return min_v;
    }

    if (v > max_v) {
        return max_v;
    }

    return v;
}

bool Odom_Reset(void)
{
    int32_t left_pos = 0;
    int32_t right_pos = 0;
    CanopenValueSample_t sample;

    if (!Kinco_ReadActualPosition(LEFT_NODE_ID, &left_pos)) {
        uart_printf("ODOM RESET failed: left position read failed\r\n");
        return false;
    }

    if (!Kinco_ReadActualPosition(RIGHT_NODE_ID, &right_pos)) {
        uart_printf("ODOM RESET failed: right position read failed\r\n");
        return false;
    }

    Odom_ResetFromRawPositions(left_pos, right_pos);
    if (Canopen_GetWheelPositionSample(LEFT_NODE_ID, &sample)) {
        s_drive.feedback.odom_left_position_sequence = sample.sequence;
    }
    if (Canopen_GetWheelPositionSample(RIGHT_NODE_ID, &sample)) {
        s_drive.feedback.odom_right_position_sequence = sample.sequence;
    }
    return true;
}

bool Odom_Update(bool print)
{
    int32_t left_pos = 0;
    int32_t right_pos = 0;

    if (!g_odom.initialized) {
        return Odom_Reset();
    }

    bool left_ok = print ?
        Kinco_ReadActualPosition(LEFT_NODE_ID, &left_pos) :
        Kinco_ReadActualPositionQuiet(LEFT_NODE_ID, &left_pos);
    if (!left_ok) {
        if (print) {
            uart_printf("ODOM update failed: left position read failed\r\n");
        }
        return false;
    }

    bool right_ok = print ?
        Kinco_ReadActualPosition(RIGHT_NODE_ID, &right_pos) :
        Kinco_ReadActualPositionQuiet(RIGHT_NODE_ID, &right_pos);
    if (!right_ok) {
        if (print) {
            uart_printf("ODOM update failed: right position read failed\r\n");
        }
        return false;
    }

    Odom_UpdateFromRawPositions(left_pos, right_pos, print);
    return true;
}

static bool Kinco_SetMotorRPM_Ex(uint8_t node_id, int32_t motor_rpm, bool write_controlword)
{
    int32_t velocity_dec = Motion_RawMotorRPMToVelocityDec(motor_rpm);

    if (DEBUG_CMD_VEL && write_controlword) {
        uart_printf("Kinco set RPM node=%u motor_rpm=%ld velocity_dec=%ld ctrl=%u\r\n",
                    node_id,
                    (long)motor_rpm,
                    (long)velocity_dec,
                    write_controlword ? 1U : 0U);
    }

    if (!SDO_Write_I32(node_id, 0x60FF, 0x00, velocity_dec)) {
        return false;
    }

    if (!write_controlword) {
        return true;
    }

    return SDO_Write_U16(node_id, 0x6040, 0x00, 0x002F);
}

static bool Kinco_SetMotorRPM(uint8_t node_id, int32_t motor_rpm)
{
    return Kinco_SetMotorRPM_Ex(node_id, motor_rpm, true);
}

static bool Kinco_SetMotorRPM_TargetOnly(uint8_t node_id, int32_t motor_rpm)
{
    return Kinco_SetMotorRPM_Ex(node_id, motor_rpm, false);
}

bool Kinco_StopNode(uint8_t node_id)
{
    if (DEBUG_CMD_VEL) {
        uart_printf("Kinco fast stop Node %u\r\n", node_id);
    }

    return SDO_Write_I32(node_id, 0x60FF, 0x00, 0);
}

void Drive_ClearMotionCommandStateOnly(void)
{
    DriveTarget_Cancel(HAL_GetTick());
#if ENABLE_JETSON_UART_MODE
    s_drive.control.requested_linear_mm_s = 0.0f;
    s_drive.control.requested_angular_mrad_s = 0.0f;
    s_drive.control.target_linear_mm_s = 0.0f;
    s_drive.control.target_angular_mrad_s = 0.0f;
    s_drive.control.nominal_feasible_linear_mm_s = 0.0f;
    s_drive.control.nominal_feasible_angular_mrad_s = 0.0f;
    s_drive.control.new_cmd_vel_pending = false;
#endif
    s_drive.command.jetson_cmd_active = false;
}

bool Drive_StartSafetyTransition(DriveSafetyTransitionType_t type,
                                        bool explicit_recovery,
                                        uint32_t now_ms)
{
    CanopenRuntimeDiagnostics_t diag;

    Drive_EnsureMotionControlInitialized();
    if (Drive_BootInProgress() &&
        ((type != DRIVE_SAFETY_INIT_DISABLE) ||
         (s_drive.boot.state == DRIVE_BOOT_WAIT_RETRY) ||
         !s_drive.boot.started)) {
        return false;
    }
    if ((type == DRIVE_SAFETY_INIT_DISABLE) && !Drive_BootInProgress()) {
        return false;
    }
    if ((type == DRIVE_SAFETY_ENABLE) &&
        Drive_LiftIsMoving()) {
        return false;
    }
    if (s_drive.fault_reset.transaction.active) {
        return false;
    }
    if (s_drive.safety.transition.active) {
        if (((type == DRIVE_SAFETY_DISABLE) ||
             (type == DRIVE_SAFETY_INIT_DISABLE)) &&
            (s_drive.safety.transition.type == DRIVE_SAFETY_STOP)) {
            s_drive.safety.transition.type = type;
            if (s_drive.safety.transition.normal_controlled_zero) {
                s_drive.safety.transition.normal_controlled_zero = false;
                Drive_StableZeroStart(now_ms, false);
                Drive_SafetySetPhase(
                    DRIVE_SAFETY_PHASE_ZERO_WAIT_MEASURED, now_ms);
            }
            DriveCore_SetSoftwareEnabled(false);
            DriveCore_SetHardwareDisableConfirmed(false);
            DriveCore_SetState(DRIVE_STATE_DISABLING);
            return true;
        }
        if (type == DRIVE_SAFETY_STOP) {
            if (s_drive.safety.transition.type == DRIVE_SAFETY_ENABLE) {
                Drive_SafetyBeginRollback(
                    DRIVE_SAFETY_ENABLE_ROLLBACK,
                    DRIVE_SAFETY_FAILURE_RUNTIME_CAN, now_ms);
            }
            return true;
        }
        return false;
    }

    if (type == DRIVE_SAFETY_ENABLE) {
        bool safe_source_state =
            s_drive.core.state == DRIVE_STATE_DISABLED;

        if (!explicit_recovery || !safe_source_state) {
            return false;
        }
    }
    diag = Canopen_GetRuntimeDiagnostics();
    memset(&s_drive.safety.transition, 0, sizeof(s_drive.safety.transition));
    s_drive.safety.transition.active = true;
    s_drive.safety.transition.type = type;
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
    s_drive.safety.transition.had_tracking_latch = s_tracking_stop_latched;
    s_drive.safety.transition.had_disable_latch =
        s_disable_motion_inhibit_latched;
    s_drive.safety.transition.was_software_enabled = s_drive.core.enabled;
    s_drive.safety.transition.was_hardware_disabled_confirmed =
        s_hardware_disable_confirmed;
    Drive_ClearMotionCommandStateOnly();
    Drive_ResetYawRateControl(true);

    if (type == DRIVE_SAFETY_ENABLE) {
        DriveCore_SetSoftwareEnabled(false);
        DriveCore_SetHardwareEnableConfirmed(false);
        DriveCore_SetHardwareDisableConfirmed(false);
        DriveCore_SetState(DRIVE_STATE_ENABLING);
        Canopen_HeartbeatResetForExplicitEnable(now_ms);
        s_drive.safety.transition.enable_node_id = LEFT_NODE_ID;
        Drive_EnableNodeEvidenceInitialize(LEFT_NODE_ID);
        Drive_InvalidateActuatorBaseline();
        Drive_SafetySetPhase(
            DRIVE_SAFETY_PHASE_ENABLE_READ_INITIAL_STATUS, now_ms);
    } else if (type == DRIVE_SAFETY_INIT_DISABLE) {
        DriveCore_SetSoftwareEnabled(false);
        DriveCore_SetHardwareEnableConfirmed(false);
        DriveCore_SetHardwareDisableConfirmed(false);
        Drive_InvalidateActuatorBaseline();
        DriveCore_SetState(DRIVE_STATE_DISABLING);
        Drive_SafetySetPhase(
            DRIVE_SAFETY_PHASE_DISABLE_WRITE_LEFT, now_ms);
    } else {
        Drive_SafetySetPhase(DRIVE_SAFETY_PHASE_ZERO_START, now_ms);
        if (type == DRIVE_SAFETY_DISABLE) {
            DriveCore_SetSoftwareEnabled(false);
            DriveCore_SetHardwareDisableConfirmed(false);
            DriveCore_SetState(DRIVE_STATE_DISABLING);
        } else if (s_drive.core.state != DRIVE_STATE_FAULT) {
            DriveCore_SetState(DRIVE_STATE_STOPPING);
        }
    }
    return true;
}

static bool Drive_SetMotorRPM_Ex(int32_t left_motor_rpm,
                                 int32_t right_motor_rpm,
                                 bool write_controlword)
{
    int32_t left_cmd = LEFT_MOTOR_FORWARD_SIGN * left_motor_rpm;
    int32_t right_cmd = RIGHT_MOTOR_FORWARD_SIGN * right_motor_rpm;
    bool target_zero = (left_motor_rpm == 0) &&
                       (right_motor_rpm == 0);

    if (target_zero) {
        return Drive_StopBoth() != DRIVE_STOP_FAILED;
    }
    if (s_drive.safety.transition.active || s_drive.fault_reset.transaction.active ||
        !Drive_EnsureEnabled()) {
        return false;
    }
    Drive_InvalidateZeroSpeedEvidence();

    if (DEBUG_CMD_VEL && write_controlword) {
        uart_printf("Drive_SetMotorRPM logical L=%ld R=%ld -> raw L=%ld R=%ld ctrl=%u\r\n",
                    (long)left_motor_rpm,
                    (long)right_motor_rpm,
                    (long)left_cmd,
                    (long)right_cmd,
                    write_controlword ? 1U : 0U);
    }

    bool ok_left = write_controlword ?
        Kinco_SetMotorRPM(LEFT_NODE_ID, left_cmd) :
        Kinco_SetMotorRPM_TargetOnly(LEFT_NODE_ID, left_cmd);
    bool ok_right = write_controlword ?
        Kinco_SetMotorRPM(RIGHT_NODE_ID, right_cmd) :
        Kinco_SetMotorRPM_TargetOnly(RIGHT_NODE_ID, right_cmd);

    if (ok_left && ok_right) {
        s_drive.command.last_left_cmd_rpm = left_motor_rpm;
        s_drive.command.last_right_cmd_rpm = right_motor_rpm;
        s_drive.command.last_left_cmd_dec = Motion_RawMotorRPMToVelocityDec(left_cmd);
        s_drive.command.last_right_cmd_dec = Motion_RawMotorRPMToVelocityDec(right_cmd);
        Drive_CommitKnownActuatorTarget((float)left_motor_rpm,
                                        (float)right_motor_rpm,
                                        s_drive.command.last_left_cmd_dec,
                                        s_drive.command.last_right_cmd_dec,
                                        HAL_GetTick());
        DriveCore_SetState(DRIVE_STATE_RUNNING);
    } else {
        uint8_t failed_node = ok_left ? RIGHT_NODE_ID : LEFT_NODE_ID;

        JetsonTx_Printf(
            JETSON_TX_PRIORITY_CRITICAL,
            "DRIVE motion SDO failed node=%u index=0x60FF reason=WRITE; zero+disable cleanup\r\n",
            (unsigned)failed_node);
        Drive_EnterHardFault(
            DRIVE_SAFETY_FAILURE_RUNTIME_CAN,
            failed_node, HAL_GetTick());
    }

    return ok_left && ok_right;
}

bool Drive_SetMotorRPM(int32_t left_motor_rpm, int32_t right_motor_rpm)
{
    return Drive_SetMotorRPM_Ex(left_motor_rpm, right_motor_rpm, true);
}

int32_t Clamp_TestMotorRPM(int32_t rpm)
{
    int32_t clamped = clamp_i32(rpm, -MAX_TEST_MOTOR_RPM, MAX_TEST_MOTOR_RPM);

    if (clamped != rpm) {
        uart_printf("WARNING: requested RPM %ld exceeds max %d, clamping\r\n",
                    (long)rpm, MAX_TEST_MOTOR_RPM);
    }

    return clamped;
}

uint32_t Clamp_TestDurationMS(uint32_t duration_ms)
{
    if (duration_ms > MAX_TEST_DURATION_MS) {
        uart_printf("WARNING: requested duration %lu ms exceeds max %lu ms, clamping\r\n",
                    (unsigned long)duration_ms,
                    (unsigned long)MAX_TEST_DURATION_MS);
        return MAX_TEST_DURATION_MS;
    }

    return duration_ms;
}

bool Drive_RunNodeMove(uint8_t node_id, int32_t motor_rpm, uint32_t duration_ms)
{
    uint32_t start = HAL_GetTick();

    if (!Kinco_SetMotorRPM(node_id, motor_rpm)) {
        (void)Kinco_StopNode(node_id);
        return false;
    }

    while ((HAL_GetTick() - start) < duration_ms) {
        uint16_t statusword = Read_And_Print_Statusword(node_id);
        bool status_valid = s_last_sdo_response.valid;
        Read_And_Print_ActualSpeed(node_id);

        if (status_valid && Statusword_HasFault(statusword)) {
            Drive_HandleFault(node_id);
            return false;
        }

        HAL_Delay(500);
    }

    (void)Kinco_StopNode(node_id);
    return true;
}

bool Drive_RunBothMove(int32_t left_motor_rpm,
                              int32_t right_motor_rpm,
                              uint32_t duration_ms)
{
    uint32_t start = HAL_GetTick();

    if (!Drive_SetMotorRPM(left_motor_rpm, right_motor_rpm)) {
        Drive_StopBoth();
        return false;
    }

    while ((HAL_GetTick() - start) < duration_ms) {
        Drive_PrintFeedback();

        if (s_drive.statusword.left_statusword_valid && Statusword_HasFault(s_drive.statusword.left_statusword)) {
            Drive_HandleFault(LEFT_NODE_ID);
            return false;
        }

        if (s_drive.statusword.right_statusword_valid && Statusword_HasFault(s_drive.statusword.right_statusword)) {
            Drive_HandleFault(RIGHT_NODE_ID);
            return false;
        }

        HAL_Delay(500);
    }

    Drive_StopBoth();
    return true;
}

bool Drive_RunBothMoveOdom(int32_t left_motor_rpm,
                                  int32_t right_motor_rpm,
                                  uint32_t duration_ms)
{
    uint32_t start = HAL_GetTick();

    if (!Drive_SetMotorRPM(left_motor_rpm, right_motor_rpm)) {
        Drive_StopBoth();
        return false;
    }

    while ((HAL_GetTick() - start) < duration_ms) {
        Drive_PrintFeedback();

        if (s_drive.statusword.left_statusword_valid && Statusword_HasFault(s_drive.statusword.left_statusword)) {
            Drive_HandleFault(LEFT_NODE_ID);
            return false;
        }

        if (s_drive.statusword.right_statusword_valid && Statusword_HasFault(s_drive.statusword.right_statusword)) {
            Drive_HandleFault(RIGHT_NODE_ID);
            return false;
        }

        (void)Odom_UpdateAndPrint();

        HAL_Delay(500);
    }

    return true;
}

static bool Drive_CheckFaultsAndStop(void)
{
    uint16_t statusword = Read_And_Print_Statusword(LEFT_NODE_ID);

    if (s_last_sdo_response.valid) {
        Drive_RecordStatusword(LEFT_NODE_ID, statusword,
                               HAL_GetTick(), 0U);
    } else {
        s_drive.statusword.left_statusword_valid = false;
        s_drive.statusword.left_statusword_generation = 0U;
    }
    if (s_drive.statusword.left_statusword_valid && Statusword_HasFault(s_drive.statusword.left_statusword)) {
        Drive_HandleFault(LEFT_NODE_ID);
        return true;
    }

    statusword = Read_And_Print_Statusword(RIGHT_NODE_ID);
    if (s_last_sdo_response.valid) {
        Drive_RecordStatusword(RIGHT_NODE_ID, statusword,
                               HAL_GetTick(), 0U);
    } else {
        s_drive.statusword.right_statusword_valid = false;
        s_drive.statusword.right_statusword_generation = 0U;
    }
    if (s_drive.statusword.right_statusword_valid && Statusword_HasFault(s_drive.statusword.right_statusword)) {
        Drive_HandleFault(RIGHT_NODE_ID);
        return true;
    }

    return false;
}

int32_t Clamp_ClosedLoopMaxRPM(int32_t max_rpm)
{
    int32_t rpm = Clamp_TestMotorRPM(max_rpm);

    if (rpm < 0) {
        rpm = -rpm;
    }

    return rpm;
}

static int32_t Select_Move_CommandRPM(float abs_error_mm, int32_t max_rpm)
{
    if (abs_error_mm > 150.0f) {
        return max_rpm;
    }

    if (abs_error_mm > 50.0f) {
        return clamp_i32(max_rpm, 0, 50);
    }

    if (abs_error_mm > MOVE_FINAL_CREEP_BAND_MM) {
        return clamp_i32(max_rpm, 0, 20);
    }

    if (abs_error_mm > MOVE_TARGET_REACHED_MM) {
        return clamp_i32(max_rpm, 0, FINAL_CREEP_RPM);
    }

    return 0;
}

static int32_t Select_Turn_CommandRPM(float abs_error_deg, int32_t max_rpm)
{
    if (abs_error_deg > 30.0f) {
        return max_rpm;
    }

    if (abs_error_deg > 10.0f) {
        return clamp_i32(max_rpm, 0, 50);
    }

    if (abs_error_deg > TURN_FINAL_CREEP_BAND_DEG) {
        return clamp_i32(max_rpm, 0, 20);
    }

    if (abs_error_deg > TURN_TARGET_REACHED_DEG) {
        return clamp_i32(max_rpm, 0, FINAL_CREEP_RPM);
    }

    return 0;
}

void Drive_MoveMM_ClosedLoop(float target_mm, int32_t max_rpm)
{
    uint32_t start = 0U;
    bool already_stopped = false;

    max_rpm = Clamp_ClosedLoopMaxRPM(max_rpm);

    uart_printf("CMD START: move_mm target=%ld mm max_rpm=%ld\r\n",
                (long)target_mm, (long)max_rpm);

    if (max_rpm == 0) {
        uart_printf("move_mm aborted: max_rpm must be nonzero\r\n");
        goto done;
    }

    if (!Odom_Reset()) {
        uart_printf("move_mm aborted: Odom_Reset failed\r\n");
        goto done;
    }

    if (!Drive_EnsureEnabled()) {
        uart_printf("move_mm aborted: Drive_EnsureEnabled failed\r\n");
        goto done;
    }

    start = HAL_GetTick();
    while ((HAL_GetTick() - start) < MAX_TEST_DURATION_MS) {
        if (!Odom_UpdateAndPrint()) {
            uart_printf("move_mm aborted: odom update failed\r\n");
            break;
        }

        float error_mm = target_mm - g_odom.x_mm;
        float abs_error_mm = fabsf(error_mm);

        uart_printf("move_mm error=%ld mm target=%ld current_x=%ld\r\n",
                    (long)error_mm,
                    (long)target_mm,
                    (long)g_odom.x_mm);

        if (abs_error_mm < MOVE_TARGET_REACHED_MM) {
            uart_printf("move_mm target reached error=%ld mm threshold=%ld mm\r\n",
                        (long)error_mm,
                        (long)MOVE_TARGET_REACHED_MM);
            break;
        }

        if (Drive_CheckFaultsAndStop()) {
            already_stopped = true;
            goto done;
        }

        int32_t rpm = Select_Move_CommandRPM(abs_error_mm, max_rpm);
        if (rpm == 0) {
            uart_printf("move_mm final stop band error=%ld mm threshold=%ld mm\r\n",
                        (long)error_mm,
                        (long)MOVE_TARGET_REACHED_MM);
            break;
        }

        if (error_mm > 0.0f) {
            if (!Drive_SetMotorRPM(rpm, rpm)) {
                uart_printf("move_mm aborted: Drive_SetMotorRPM failed\r\n");
                break;
            }
        } else {
            if (!Drive_SetMotorRPM(-rpm, -rpm)) {
                uart_printf("move_mm aborted: Drive_SetMotorRPM failed\r\n");
                break;
            }
        }

        HAL_Delay(100);
    }

    if ((HAL_GetTick() - start) >= MAX_TEST_DURATION_MS) {
        uart_printf("MOVE_MM TIMEOUT\r\n");
    }

done:
    if (!already_stopped) {
        Drive_StopBoth();
    }
    (void)Odom_UpdateAndPrint();
    uart_printf("CMD DONE stop_confirmation=PENDING\r\n");
}

void Drive_TurnDeg_ClosedLoop(float target_deg, int32_t max_rpm)
{
    uint32_t start = 0U;
    bool already_stopped = false;
    float target_rad = target_deg * ODOM_PI / 180.0f;

    max_rpm = Clamp_ClosedLoopMaxRPM(max_rpm);

    uart_printf("CMD START: turn_deg target=%ld deg max_rpm=%ld\r\n",
                (long)target_deg, (long)max_rpm);

    if (max_rpm == 0) {
        uart_printf("turn_deg aborted: max_rpm must be nonzero\r\n");
        goto done;
    }

    if (!Odom_Reset()) {
        uart_printf("turn_deg aborted: Odom_Reset failed\r\n");
        goto done;
    }

    if (!Drive_EnsureEnabled()) {
        uart_printf("turn_deg aborted: Drive_EnsureEnabled failed\r\n");
        goto done;
    }

    start = HAL_GetTick();
    while ((HAL_GetTick() - start) < MAX_TEST_DURATION_MS) {
        if (!Odom_UpdateAndPrint()) {
            uart_printf("turn_deg aborted: odom update failed\r\n");
            break;
        }

        float error_rad = Odom_NormalizeAngleRad(target_rad - g_odom.theta_rad);
        float error_deg = error_rad * 180.0f / ODOM_PI;
        float abs_error_deg = fabsf(error_deg);

        uart_printf("turn_deg error=%ld deg target=%ld current=%ld\r\n",
                    (long)error_deg,
                    (long)target_deg,
                    (long)(g_odom.theta_rad * 180.0f / ODOM_PI));

        if (abs_error_deg < TURN_TARGET_REACHED_DEG) {
            uart_printf("turn_deg target reached error_x100=%ld deg threshold_x100=%ld deg\r\n",
                        (long)(error_deg * 100.0f),
                        (long)(TURN_TARGET_REACHED_DEG * 100.0f));
            break;
        }

        if (Drive_CheckFaultsAndStop()) {
            already_stopped = true;
            goto done;
        }

        int32_t rpm = Select_Turn_CommandRPM(abs_error_deg, max_rpm);
        if (rpm == 0) {
            uart_printf("turn_deg final stop band error_x100=%ld deg threshold_x100=%ld deg\r\n",
                        (long)(error_deg * 100.0f),
                        (long)(TURN_TARGET_REACHED_DEG * 100.0f));
            break;
        }

        if (error_rad < 0.0f) {
            if (!Drive_SetMotorRPM(rpm, -rpm)) {
                uart_printf("turn_deg aborted: Drive_SetMotorRPM failed\r\n");
                break;
            }
        } else {
            if (!Drive_SetMotorRPM(-rpm, rpm)) {
                uart_printf("turn_deg aborted: Drive_SetMotorRPM failed\r\n");
                break;
            }
        }

        HAL_Delay(100);
    }

    if ((HAL_GetTick() - start) >= MAX_TEST_DURATION_MS) {
        uart_printf("TURN_DEG TIMEOUT\r\n");
    }

done:
    if (!already_stopped) {
        Drive_StopBoth();
    }
    (void)Odom_UpdateAndPrint();
    uart_printf("CMD DONE stop_confirmation=PENDING\r\n");
}









static CanopenRuntimeSubmitResult_t Drive_SubmitControlledZeroPair(uint32_t now_ms)
{
    if (s_drive.safety.transition.active || s_drive.fault_reset.transaction.active ||
        !Drive_NormalMotionAllowed()) {
        return CANOPEN_RUNTIME_SUBMIT_BUSY;
    }
    /* Acceptance starts the existing SDO zero/ACK/measured-zero state machine.
     * It is neither an RPDO completion nor confirmation that either wheel stopped. */
    return Drive_StartSafetyTransition(DRIVE_SAFETY_STOP, false, now_ms) ?
        CANOPEN_RUNTIME_SUBMIT_OK : CANOPEN_RUNTIME_SUBMIT_ERROR;
}

void Drive_ClearMotionCommand(void)
{
    Drive_ClearMotionCommandStateOnly();
    Drive_ResetYawRateControl(false);
}

static bool Drive_CommandTargetChanged(float old_linear_mm_s,
                                       float old_angular_mrad_s,
                                       float new_linear_mm_s,
                                       float new_angular_mrad_s)
{
    return (fabsf(new_linear_mm_s - old_linear_mm_s) >=
            DRIVE_CMD_CHANGE_LINEAR_EPSILON_MM_S) ||
           (fabsf(new_angular_mrad_s - old_angular_mrad_s) >=
            DRIVE_CMD_CHANGE_ANGULAR_EPSILON_MRAD_S);
}

static bool Drive_ShouldResetYawPI(float old_linear_mm_s,
                                   float old_angular_mrad_s,
                                   float new_linear_mm_s,
                                   float new_angular_mrad_s)
{
    bool old_zero_mode =
        fabsf(old_angular_mrad_s) <= DRIVE_YAW_RATE_ZERO_THRESHOLD_MRAD_S;
    bool new_zero_mode =
        fabsf(new_angular_mrad_s) <= DRIVE_YAW_RATE_ZERO_THRESHOLD_MRAD_S;
    bool significant_sign_reversal =
        ((old_angular_mrad_s > 0.0f) && (new_angular_mrad_s < 0.0f)) ||
        ((old_angular_mrad_s < 0.0f) && (new_angular_mrad_s > 0.0f));

    (void)old_linear_mm_s;
    (void)new_linear_mm_s;
    significant_sign_reversal = significant_sign_reversal &&
        (fabsf(old_angular_mrad_s) >=
         DRIVE_YAW_PI_RETARGET_SIGN_MIN_MRAD_S) &&
        (fabsf(new_angular_mrad_s) >=
         DRIVE_YAW_PI_RETARGET_SIGN_MIN_MRAD_S);

    return (!old_zero_mode && new_zero_mode) ||
           significant_sign_reversal ||
           (fabsf(new_angular_mrad_s - old_angular_mrad_s) >=
            DRIVE_YAW_PI_RETARGET_RESET_STEP_MRAD_S);
}

void Drive_CommandSetCmdVel(float linear_mm_s, float angular_mrad_s)
{
    if (Drive_BootInProgress()) {
        Drive_ClearMotionCommandStateOnly();
        return;
    }
#if ENABLE_JETSON_UART_MODE
    MotionBodyCommand_t body_command;
    bool requested_zero = Drive_CmdVelIsZero(linear_mm_s, angular_mrad_s);
    bool target_changed;
    bool reset_yaw_pi;

    if (!requested_zero && !Drive_NormalMotionAllowed()) {
        uart_printf(
            "CMD_VEL ignored: state=%s hard_fault=%u lift=%u\r\n",
            Drive_StateName(s_drive.core.state),
            (s_drive.core.state == DRIVE_STATE_FAULT) ? 1U : 0U,
            Drive_LiftIsMoving() ? 1U : 0U);
        return;
    }

    Drive_EnsureMotionControlInitialized();
    /* Idle time is not acceleration time. Anchor shaping at command arrival,
     * including tick==0, without changing the real TX commit timestamp. */
    if (!requested_zero && !s_drive.command.jetson_cmd_active &&
        !DriveTarget_Pending() && Drive_LastCommandTargetIsZero() &&
        s_drive.control.actuator.state.valid) {
        DriveTarget_Hold(HAL_GetTick());
    }
    if (s_yaw_pi_enabled) {
        Motion_LimitBodyCommandWithYawHeadroom(linear_mm_s,
                                               angular_mrad_s,
                                               &s_drive.control.actuator.command_envelope,
                                               &body_command);
    } else {
        Motion_LimitBodyCommand(linear_mm_s,
                                angular_mrad_s,
                                s_drive.control.actuator.command_envelope.hard_linear_mm_s,
                                s_drive.control.actuator.command_envelope.hard_angular_mrad_s,
                                &body_command);
    }
    s_drive.control.actuator.body_command = body_command;
    target_changed = Drive_CommandTargetChanged(
        s_drive.control.target_linear_mm_s,
        s_drive.control.target_angular_mrad_s,
        body_command.limited_linear_mm_s,
        body_command.limited_angular_mrad_s);
    reset_yaw_pi = target_changed && Drive_ShouldResetYawPI(
        s_drive.control.target_linear_mm_s,
        s_drive.control.target_angular_mrad_s,
        body_command.limited_linear_mm_s,
        body_command.limited_angular_mrad_s);

    s_drive.control.requested_linear_mm_s = linear_mm_s;
    s_drive.control.requested_angular_mrad_s = angular_mrad_s;
    s_drive.control.target_linear_mm_s = body_command.limited_linear_mm_s;
    s_drive.control.target_angular_mrad_s = body_command.limited_angular_mrad_s;
    if (reset_yaw_pi) {
        Motion_YawRatePIReset(&s_drive.control.yaw.pi);
        if (s_drive.control.yaw.retarget_reset_count != UINT32_MAX) {
            s_drive.control.yaw.retarget_reset_count++;
        }
    }
    if (target_changed) {
        s_drive.feedback.pair_pending = !s_drive.control.yaw.waiting_for_new_pair &&
                                  s_drive.feedback.chassis.valid &&
                                  !s_drive.feedback.chassis.stale;
    }
    s_drive.command.last_jetson_cmd_ms = HAL_GetTick();
    s_drive.command.jetson_cmd_active = !Drive_CmdVelIsZero(s_drive.control.target_linear_mm_s,
                                              s_drive.control.target_angular_mrad_s);
    s_drive.control.new_cmd_vel_pending = true;

    if (DEBUG_CMD_VEL && target_changed) {
        uart_printf("CMD_VEL requested=%ld/%ld body_limited=%ld/%ld nominal=%u limited=%u/%u\r\n",
                    (long)s_drive.control.requested_linear_mm_s,
                    (long)s_drive.control.requested_angular_mrad_s,
                    (long)s_drive.control.target_linear_mm_s,
                    (long)s_drive.control.target_angular_mrad_s,
                    body_command.nominal_envelope_used ? 1U : 0U,
                    body_command.linear_limited ? 1U : 0U,
                    body_command.angular_limited ? 1U : 0U);
    }
#else
    (void)linear_mm_s;
    (void)angular_mrad_s;
#endif
}

bool Drive_CommandSetYawPiEnabled(bool enabled)
{
    bool motion_is_idle =
        !s_drive.command.jetson_cmd_active && !Drive_LiftIsMoving() &&
        !s_drive.safety.transition.active && !s_drive.fault_reset.transaction.active &&
        (s_drive.core.state != DRIVE_STATE_RUNNING) &&
        Drive_LastCommandTargetIsZero();

#if ENABLE_JETSON_UART_MODE
    motion_is_idle = motion_is_idle &&
        Drive_CmdVelIsZero(s_drive.control.requested_linear_mm_s,
                           s_drive.control.requested_angular_mrad_s) &&
        Drive_CmdVelIsZero(s_drive.control.target_linear_mm_s,
                           s_drive.control.target_angular_mrad_s);
#endif
    if (!motion_is_idle) {
        return false;
    }
#if DRIVE_BASIC_BRINGUP_PROFILE && \
    (!defined(AMR_HOST_TEST) || \
     !defined(AMR_HOST_TEST_SYNC_PI_ENABLED))
    if (enabled) {
        return false;
    }
#endif
    if (s_yaw_pi_enabled == enabled) {
        return true;
    }

    Drive_EnsureMotionControlInitialized();
    s_yaw_pi_enabled = enabled;
    Motion_YawRatePIReset(&s_drive.control.yaw.pi);
    memset(&s_drive.control.yaw.proposal, 0, sizeof(s_drive.control.yaw.proposal));
    Motion_TrackingSupervisorReset(&s_drive.control.tracking.supervisor, true);
    memset(&s_drive.control.tracking.limits, 0, sizeof(s_drive.control.tracking.limits));
    s_drive.feedback.pair_pending = false;
    s_drive.control.yaw.waiting_for_new_pair = true;
    s_drive.control.yaw.corrected_angular_mrad_s = 0.0f;
    s_drive.control.yaw.submitted_correction_mrad_s = 0.0f;
    s_drive.control.yaw.proposed_submitted_correction_mrad_s = 0.0f;
    return true;
}

static bool Drive_IdleAutoDisable_Update(uint32_t now, bool target_zero)
{
#if DRIVE_IDLE_AUTO_DISABLE_ENABLE
    static uint32_t idle_start_ms = 0U;

    if ((s_drive.core.state == DRIVE_STATE_ENABLED_IDLE) && target_zero) {
        if (idle_start_ms == 0U) {
            idle_start_ms = now;
            return false;
        }

        if ((now - idle_start_ms) >= DRIVE_IDLE_AUTO_DISABLE_MS) {
            if (DEBUG_CMD_VEL) {
                uart_printf("DRIVE idle auto-disable after %lu ms\r\n",
                            (unsigned long)DRIVE_IDLE_AUTO_DISABLE_MS);
            }

            (void)Drive_DisableBoth();
            idle_start_ms = 0U;
            return true;
        }

        return false;
    }

    idle_start_ms = 0U;
#else
    (void)now;
    (void)target_zero;
#endif

    return false;
}

void Drive_ApplyCmdVel(float linear_mm_s, float angular_mrad_s)
{
    /* All body commands use the one production controller/transport. */
    Drive_CommandSetCmdVel(linear_mm_s, angular_mrad_s);
}

#if ENABLE_JETSON_UART_MODE
static CanopenWheelFeedbackSchedule_t Drive_ControlFeedbackSchedule(
    uint32_t now,
    bool feedback_request_window,
    bool runtime_target_pending)
{
    CanopenWheelFeedbackSchedule_t feedback_schedule =
        CANOPEN_FEEDBACK_DISPATCH_ONLY;

    if (Drive_SafetyNeedsVelocityPolling()) {
        feedback_schedule =
            CANOPEN_FEEDBACK_STOP_CONFIRM_VELOCITY_ONLY;
    } else if (s_drive.safety.transition.active) {
        feedback_schedule = CANOPEN_FEEDBACK_DISPATCH_ONLY;
        if (s_drive.core.state == DRIVE_STATE_FAULT) {
            Canopen_RecordFaultMonitorDeferred(now);
        }
    } else if (s_drive.fault_reset.transaction.active &&
               (s_drive.fault_reset.transaction.phase ==
                DRIVE_FAULT_RESET_PHASE_PRECHECK_ZERO_FEEDBACK)) {
        feedback_schedule =
            CANOPEN_FEEDBACK_STOP_CONFIRM_VELOCITY_ONLY;
    } else if (s_drive.fault_reset.transaction.active) {
        feedback_schedule = CANOPEN_FEEDBACK_DISPATCH_ONLY;
        Canopen_RecordFaultMonitorDeferred(now);
    } else if (Drive_BaseSafeNeedsVelocityPolling()) {
        feedback_schedule =
            CANOPEN_FEEDBACK_STOP_CONFIRM_VELOCITY_ONLY;
    } else if (s_drive.core.state == DRIVE_STATE_FAULT) {
        feedback_schedule =
            CANOPEN_FEEDBACK_FAULT_MONITOR_VELOCITY_ONLY;
    } else if (runtime_target_pending) {
        feedback_schedule = CANOPEN_FEEDBACK_DISPATCH_ONLY;
    } else if (feedback_request_window) {
        feedback_schedule = CANOPEN_FEEDBACK_NORMAL_MOTION;
    }
    return feedback_schedule;
}
#endif

#if ENABLE_JETSON_UART_MODE

#endif

#if ENABLE_JETSON_UART_MODE
static bool Drive_ControlConsumeHeartbeatFault(uint32_t now)
{
    {
        uint8_t heartbeat_fault_node = 0U;
        CanopenHeartbeatFaultReason_t heartbeat_fault_reason =
            CANOPEN_HEARTBEAT_FAULT_NONE;

        if (Canopen_TakeHeartbeatFault(
                &heartbeat_fault_node, &heartbeat_fault_reason)) {
            Drive_LatchHeartbeatFault();
            if (s_drive.fault_reset.transaction.active) {
                Drive_FaultResetFail(
                    DRIVE_FAULT_RESET_FAILURE_RUNTIME_CAN,
                    heartbeat_fault_node, now);
                return true;
            }
            if (s_drive.safety.transition.active) {
                s_drive.safety.transition.failure_node_id =
                    heartbeat_fault_node;
                if (Drive_SafetyIsRollback()) {
                    s_drive.safety.transition.failure_reason =
                        DRIVE_SAFETY_FAILURE_HEARTBEAT;
                } else {
                    Drive_SafetyEscalate(
                        DRIVE_SAFETY_FAILURE_HEARTBEAT, now);
                }
                return true;
            }
            DriveCore_SetState(DRIVE_STATE_FAULT);
            JetsonTx_Printf(
                JETSON_TX_PRIORITY_CRITICAL,
                "DRIVE Heartbeat fault node=%u reason=%u; entering nonblocking rollback\r\n",
                (unsigned)heartbeat_fault_node,
                (unsigned)heartbeat_fault_reason);
            Drive_SafetyBeginRollback(
                DRIVE_SAFETY_DISABLE_ROLLBACK,
                DRIVE_SAFETY_FAILURE_HEARTBEAT, now);
            return true;
        }
    }
    return false;
}
#endif

#if ENABLE_JETSON_UART_MODE
static const char *Drive_TargetFailureReasonName(uint8_t reason)
{
    switch ((CanopenTargetFailureReason_t)reason) {
        case CANOPEN_TARGET_FAILURE_TX: return "TX";
        case CANOPEN_TARGET_FAILURE_TIMEOUT: return "TIMEOUT";
        case CANOPEN_TARGET_FAILURE_SDO_ABORT: return "SDO_ABORT";
        case CANOPEN_TARGET_FAILURE_PROTOCOL: return "PROTOCOL";
        case CANOPEN_TARGET_FAILURE_PREEMPT: return "PREEMPT";
        case CANOPEN_TARGET_FAILURE_EMCY: return "EMCY";
        case CANOPEN_TARGET_FAILURE_NONE:
        default: return "UNKNOWN";
    }
}

static bool Drive_ControlConsumeWheelRuntimeFault(uint32_t now)
{
    if (Canopen_TakeWheelRuntimeFault()) {
        CanopenRuntimeDiagnostics_t diag =
            Canopen_GetRuntimeDiagnostics();

        Drive_InvalidateZeroSpeedEvidence();
        if (s_drive.fault_reset.transaction.active) {
            Drive_FaultResetFail(
                DRIVE_FAULT_RESET_FAILURE_RUNTIME_CAN,
                s_drive.fault_reset.transaction.node_id, now);
            return true;
        }
        if (s_drive.safety.transition.active) {
            if (Drive_SafetyIsRollback()) {
                s_drive.safety.transition.failure_reason =
                    DRIVE_SAFETY_FAILURE_RUNTIME_CAN;
            } else {
                Drive_SafetyEscalate(
                    DRIVE_SAFETY_FAILURE_RUNTIME_CAN, now);
            }
            return true;
        }
        JetsonTx_Printf(
            JETSON_TX_PRIORITY_CRITICAL,
            "DRIVE runtime target failed node=%u index=0x60FF reason=%s; zero+disable cleanup\r\n",
            (unsigned)diag.last_wheel_target_failure_node,
            Drive_TargetFailureReasonName(
                diag.last_wheel_target_failure_reason));
        Drive_EnterHardFault(
            DRIVE_SAFETY_FAILURE_RUNTIME_CAN,
            diag.last_wheel_target_failure_node, now);
        Drive_ResetYawRateControl(false);
        return true;
    }
    return false;
}
#endif

#if ENABLE_JETSON_UART_MODE
static bool Drive_ControlConsumeFeedbackContinuityFault(uint32_t now)
{
    if (Canopen_TakeWheelFeedbackContinuityFault()) {
        Drive_InvalidateZeroSpeedEvidence();
        Drive_MarkStatusFeedbackInvalid();
        if (s_drive.fault_reset.transaction.active) {
            Drive_FaultResetFail(
                DRIVE_FAULT_RESET_FAILURE_RUNTIME_CAN,
                s_drive.fault_reset.transaction.node_id, now);
            return true;
        }
        if (s_drive.safety.transition.active) {
            if (Drive_SafetyIsRollback()) {
                s_drive.safety.transition.failure_reason =
                    DRIVE_SAFETY_FAILURE_RUNTIME_CAN;
            } else {
                Drive_SafetyEscalate(
                    (Canopen_GetWheelFeedbackAuthority() ==
                     CANOPEN_WHEEL_AUTHORITY_TPDO) ?
                    DRIVE_SAFETY_FAILURE_TPDO_FEEDBACK :
                    DRIVE_SAFETY_FAILURE_ZERO_FEEDBACK,
                    now);
            }
            return true;
        }
        if (Canopen_GetWheelFeedbackAuthority() ==
            CANOPEN_WHEEL_AUTHORITY_TPDO) {
            (void)JetsonTx_Printf(
                JETSON_TX_PRIORITY_CRITICAL,
                "DRIVE TPDO Active feedback lost reason=%s; zero+disable cleanup\r\n",
                Canopen_GetLastWheelTpdoActiveFault());
            Drive_EnterHardFault(
                DRIVE_SAFETY_FAILURE_TPDO_FEEDBACK, 0U, now);
            Motion_YawRatePIReset(&s_drive.control.yaw.pi);
            s_drive.feedback.pair_pending = false;
            s_drive.control.yaw.submitted_correction_mrad_s = 0.0f;
            return true;
        }
        Motion_YawRatePIReset(&s_drive.control.yaw.pi);
        s_drive.feedback.pair_pending = false;
        s_drive.control.yaw.submitted_correction_mrad_s = 0.0f;
    }
    return false;
}
#endif

#if ENABLE_JETSON_UART_MODE
static bool Drive_ControlServiceSafetyTransactions(uint32_t now)
{
    Drive_ProcessFeedbackHealth();

    if (s_drive.lift_stop.request.active &&
        s_drive.lift_stop.request.explicit_base_safe_refresh) {
        (void)Drive_ServiceLiftStatuswordRefresh(now);
    }

    Drive_FaultResetStep(now);
    if (s_drive.fault_reset.transaction.active) {
        return true;
    }
    Drive_SafetyTransitionStep(now);
    if (s_drive.safety.transition.active) {
        return true;
    }
#if !DRIVE_BASIC_BRINGUP_PROFILE
    if ((s_drive.core.state == DRIVE_STATE_ENABLED_IDLE) &&
        Drive_CmdVelIsZero(s_drive.control.target_linear_mm_s,
                           s_drive.control.target_angular_mrad_s) &&
        Drive_LastCommandTargetIsZero() &&
        !Drive_ZeroSpeedEvidenceIsCurrent(now)) {
        (void)Drive_StartSafetyTransition(
            DRIVE_SAFETY_STOP, false, now);
        return true;
    }
    if (s_tracking_stop_latched || s_disable_motion_inhibit_latched) {
        DriveCore_SetSoftwareEnabled(false);
        if (!((s_drive.core.state == DRIVE_STATE_DISABLED) &&
              s_hardware_disable_confirmed)) {
            DriveCore_SetState(DRIVE_STATE_FAULT);
        }
        return true;
    }
#endif
    return false;
}
#endif

#if ENABLE_JETSON_UART_MODE
static bool Drive_ControlHandleJetsonWatchdog(uint32_t now)
{
    if (s_drive.command.jetson_cmd_active &&
        ((now - s_drive.command.last_jetson_cmd_ms) > JETSON_CMD_TIMEOUT_MS)) {
        if ((s_drive.core.state == DRIVE_STATE_RUNNING) ||
            DriveTarget_Pending() || !Drive_LastCommandTargetIsZero()) {
            uart_printf("JETSON WATCHDOG TIMEOUT, STOPPING\r\n");
            Drive_CommandStop();
        }

        s_drive.control.requested_linear_mm_s = 0.0f;
        s_drive.control.requested_angular_mrad_s = 0.0f;
        s_drive.control.target_linear_mm_s = 0.0f;
        s_drive.control.target_angular_mrad_s = 0.0f;
        s_drive.control.nominal_feasible_linear_mm_s = 0.0f;
        s_drive.control.nominal_feasible_angular_mrad_s = 0.0f;
        s_drive.control.new_cmd_vel_pending = false;
        s_drive.command.jetson_cmd_active = false;
        Drive_ResetYawRateControl(false);
        s_drive.timing.control_last_can_write_ms = now;
        return true;
    }
    return false;
}
#endif

#if ENABLE_JETSON_UART_MODE
static bool Drive_ControlHandleMovingLift(uint32_t now)
{
    if (Drive_LiftIsMoving()) {
        if (DEBUG_CMD_VEL && s_drive.control.new_cmd_vel_pending) {
            uart_printf("CMD_VEL ignored: lift is moving\r\n");
        }
        if ((s_drive.core.state == DRIVE_STATE_RUNNING) ||
            DriveTarget_Pending() || !Drive_LastCommandTargetIsZero()) {
            Drive_CommandStop();
        }
        s_drive.control.requested_linear_mm_s = 0.0f;
        s_drive.control.requested_angular_mrad_s = 0.0f;
        s_drive.control.target_linear_mm_s = 0.0f;
        s_drive.control.target_angular_mrad_s = 0.0f;
        s_drive.control.nominal_feasible_linear_mm_s = 0.0f;
        s_drive.control.nominal_feasible_angular_mrad_s = 0.0f;
        s_drive.control.new_cmd_vel_pending = false;
        s_drive.command.jetson_cmd_active = false;
        /* Lift activity inhibits wheel commands, not measurement. In
         * WAIT_BASE_STOP the Lift consumer runs after this function and needs
         * the fresh TPDO pair / stable-zero proof just updated above. Actual
         * stale/skew/fault paths still invalidate that proof independently. */
        Drive_ResetYawRateControl(false);
        s_drive.timing.control_last_can_write_ms = now;
        return true;
    }
    return false;
}
#endif

#if ENABLE_JETSON_UART_MODE
static bool Drive_LinearDirectionReversalInProgress(void)
{
    float current_linear_mm_s =
        s_drive.control.actuator.state.feasible_linear_mm_s;
    float target_linear_mm_s = s_drive.control.target_linear_mm_s;

    return s_drive.control.actuator.state.valid &&
        (((current_linear_mm_s > 0.0f) &&
          (target_linear_mm_s < 0.0f)) ||
         ((current_linear_mm_s < 0.0f) &&
          (target_linear_mm_s > 0.0f)));
}

static float Drive_SelectLinearSlewLimitMmS2(void)
{
    float current_linear_mm_s =
        s_drive.control.actuator.state.feasible_linear_mm_s;
    float target_linear_mm_s = s_drive.control.target_linear_mm_s;

    if (s_drive.control.actuator.state.valid &&
        (((current_linear_mm_s > 0.0f) &&
          (target_linear_mm_s < current_linear_mm_s)) ||
         ((current_linear_mm_s < 0.0f) &&
          (target_linear_mm_s > current_linear_mm_s)))) {
        return LINEAR_DECEL_LIMIT_MM_S2;
    }
    return LINEAR_ACCEL_LIMIT_MM_S2;
}

static bool Drive_ControlPrepareProposal(
    uint32_t now,
    bool target_zero,
    bool nominal_feasible_zero,
    bool *pi_prepared_out,
    MotionTrackingLimits_t *proposed_limits_out,
    bool *target_changed_out,
    bool *refresh_due_out)
{
    MotionActuatorState_t shaping_state = DriveTarget_ShapingState();
    bool pi_prepared = false;
    bool linear_reversal = Drive_LinearDirectionReversalInProgress();
    float linear_slew_limit_mm_s2 = Drive_SelectLinearSlewLimitMmS2();
    float slew_target_linear_mm_s = linear_reversal ?
        0.0f : s_drive.control.target_linear_mm_s;
    float requested_correction_mrad_s =
        (s_drive.control.yaw.pi.active && Drive_YawControlAllowed()) ?
        s_drive.control.yaw.pi.requested_correction_mrad_s : 0.0f;
    float raw_correction_direction_mrad_s = requested_correction_mrad_s;
    float directional_authority_mrad_s;

    Motion_ActuatorPrepareBodyCommand(
        &shaping_state,
        slew_target_linear_mm_s,
        s_drive.control.target_angular_mrad_s,
        now,
        DRIVE_ACTUATOR_RATE_MAX_DT_MS,
        linear_slew_limit_mm_s2,
        ANGULAR_ACCEL_LIMIT_MRAD_S2,
        (float)MAX_TEST_MOTOR_RPM,
        &s_drive.control.actuator.baseline_proposal);
    Motion_CmdVelToMotorTargets(
        s_drive.control.actuator.baseline_proposal.feasible_linear_mm_s,
        s_drive.control.actuator.baseline_proposal.feasible_angular_mrad_s,
        &s_drive.control.actuator.baseline_targets);

    if (s_drive.feedback.pair_pending && Drive_YawControlAllowed()) {
        s_drive.control.yaw.proposal = Motion_YawRatePIPrepare(
            &s_drive.control.yaw.pi,
            &s_yaw_rate_pi_config,
            s_drive.control.actuator.baseline_proposal.feasible_linear_mm_s,
            s_drive.control.actuator.baseline_proposal.feasible_angular_mrad_s,
            &s_drive.feedback.chassis,
            true);
        if (s_drive.control.yaw.prepare_count != UINT32_MAX) {
            s_drive.control.yaw.prepare_count++;
        }
        if (s_drive.control.yaw.proposal.valid) {
            pi_prepared = true;
            requested_correction_mrad_s =
                s_drive.control.yaw.proposal.active ?
                s_drive.control.yaw.proposal.requested_correction_mrad_s : 0.0f;
            raw_correction_direction_mrad_s =
                s_drive.control.yaw.proposal.raw_correction_mrad_s;
        } else {
            Motion_YawRatePIRecordDiscard(
                &s_drive.control.yaw.pi, s_drive.control.yaw.proposal.status);
            s_drive.feedback.pair_pending = false;
        }
    }

    directional_authority_mrad_s = Motion_SelectDirectionalYawAuthority(
        &s_drive.control.actuator.baseline_targets, raw_correction_direction_mrad_s);
    if (requested_correction_mrad_s > directional_authority_mrad_s) {
        requested_correction_mrad_s = directional_authority_mrad_s;
    } else if (requested_correction_mrad_s <
               -directional_authority_mrad_s) {
        requested_correction_mrad_s = -directional_authority_mrad_s;
    }
    s_drive.control.yaw.corrected_angular_mrad_s = s_drive.control.target_angular_mrad_s +
                                 requested_correction_mrad_s;
    Motion_ActuatorPrepareBodyCommand(
        &shaping_state,
        slew_target_linear_mm_s,
        s_drive.control.yaw.corrected_angular_mrad_s,
        now,
        DRIVE_ACTUATOR_RATE_MAX_DT_MS,
        linear_slew_limit_mm_s2,
        ANGULAR_ACCEL_LIMIT_MRAD_S2,
        (float)MAX_TEST_MOTOR_RPM,
        &s_drive.control.actuator.final_proposal);
    Motion_CmdVelToMotorTargets(
        s_drive.control.actuator.final_proposal.feasible_linear_mm_s,
        s_drive.control.actuator.final_proposal.feasible_angular_mrad_s,
        &s_drive.control.actuator.final_targets);
    s_drive.control.yaw.proposed_submitted_correction_mrad_s =
        s_drive.control.actuator.final_proposal.feasible_angular_mrad_s -
        s_drive.control.actuator.baseline_proposal.feasible_angular_mrad_s;

    MotionTrackingLimits_t proposed_limits = {
        .correction_saturated =
            (pi_prepared && s_drive.control.yaw.proposal.correction_saturated) ||
            (fabsf(requested_correction_mrad_s) + 0.001f <
             fabsf(pi_prepared ?
                 s_drive.control.yaw.proposal.requested_correction_mrad_s :
                 s_drive.control.yaw.pi.requested_correction_mrad_s)),
        .nominal_headroom_exhausted =
            (fabsf(raw_correction_direction_mrad_s) > 0.001f) &&
            (directional_authority_mrad_s <=
             (fabsf(pi_prepared ?
                 s_drive.control.yaw.proposal.requested_correction_mrad_s :
                 s_drive.control.yaw.pi.requested_correction_mrad_s) + 0.001f)),
        .hard_wheel_saturated = s_drive.control.actuator.final_proposal.hard_limited,
        .final_rate_limited = s_drive.control.actuator.final_proposal.rate_limited,
        .transport_busy = false
    };
    bool proposed_dec_zero = Drive_TargetDecIsZero(
        s_drive.control.actuator.final_proposal.left_velocity_dec,
        s_drive.control.actuator.final_proposal.right_velocity_dec);

    if (target_zero && proposed_dec_zero) {
        s_drive.control.nominal_feasible_linear_mm_s = 0.0f;
        s_drive.control.nominal_feasible_angular_mrad_s = 0.0f;
    }

    if (proposed_dec_zero &&
        !Drive_LinearDirectionReversalInProgress()) {
        bool was_moving = (s_drive.core.state == DRIVE_STATE_RUNNING) ||
                          !Drive_LastCommandTargetIsZero();

        if (was_moving) {
            CanopenRuntimeSubmitResult_t zero_result;

            if (DEBUG_CMD_VEL) {
                uart_printf("CMD_VEL ramp reached zero, stopping drive\r\n");
            }
            zero_result = Drive_SubmitControlledZeroPair(now);
            if (zero_result == CANOPEN_RUNTIME_SUBMIT_BUSY) {
                Motion_ActuatorRejectBusy(
                    &s_drive.control.actuator.state);
                s_drive.control.tracking.limits.transport_busy = true;
                if (s_drive.control.tracking.supervisor
                        .transport_busy_observation_count != UINT32_MAX) {
                    s_drive.control.tracking.supervisor
                        .transport_busy_observation_count++;
                }
                s_drive.control.new_cmd_vel_pending = true;
                return false;
            }
            if (zero_result != CANOPEN_RUNTIME_SUBMIT_OK) {
                uint8_t failed_node = (uint8_t)
                    s_drive.diagnostic.sync_pair.last_failed_node;
                uint32_t failure_now = HAL_GetTick();

                JetsonTx_Printf(
                    JETSON_TX_PRIORITY_CRITICAL,
                    "CMD_VEL safety zero start failed node=%u index=0x60FF reason=WRITE; zero+disable cleanup\r\n",
                    (unsigned)failed_node);
                Motion_YawRatePIReset(&s_drive.control.yaw.pi);
                Drive_ClearMotionCommandStateOnly();
                Drive_EnterHardFault(
                    DRIVE_SAFETY_FAILURE_RUNTIME_CAN,
                    failed_node, failure_now);
                s_drive.timing.control_last_can_write_ms = now;
                return false;
            }
            s_drive.timing.control_last_can_write_ms = now;
        }

        if (target_zero) {
            s_drive.command.jetson_cmd_active = false;
        }

        s_drive.control.new_cmd_vel_pending = false;
        Drive_ResetYawRateControl(false);
        return false;
    }

    if (!s_drive.command.jetson_cmd_active &&
        !s_drive.control.new_cmd_vel_pending &&
        target_zero &&
        nominal_feasible_zero) {
        return false;
    }

    bool target_changed =
        (s_drive.control.actuator.final_proposal.left_velocity_dec != s_drive.command.last_left_cmd_dec) ||
        (s_drive.control.actuator.final_proposal.right_velocity_dec != s_drive.command.last_right_cmd_dec);
    bool refresh_due = (s_drive.timing.control_last_can_write_ms == 0U) ||
                       ((now - s_drive.timing.control_last_can_write_ms) >= DRIVE_CAN_REFRESH_MS);

    *pi_prepared_out = pi_prepared;
    *proposed_limits_out = proposed_limits;
    *target_changed_out = target_changed;
    *refresh_due_out = refresh_due;
    return true;
}
#endif

#if ENABLE_JETSON_UART_MODE
static void Drive_ControlSubmitProposal(
    uint32_t now, bool pi_prepared, MotionTrackingLimits_t proposed_limits,
    bool target_changed, bool refresh_due)
{
    if (!target_changed && !refresh_due && !pi_prepared) {
        DriveTarget_Hold(now);
        s_drive.control.new_cmd_vel_pending = false;
        return;
    }
    DriveTarget_Submit(now, pi_prepared, proposed_limits);
    /* Completion is serviced separately, never HAL_OK == ACK. */
}
#endif

#if ENABLE_JETSON_UART_MODE
static void Drive_ControlUpdateTrackingSupervisor(bool pi_prepared, const MotionChassisFeedback_t *feedback)
{
    if (pi_prepared &&
        (Motion_TrackingSupervisorUpdate(
             &s_drive.control.tracking.supervisor,
             &s_tracking_config,
             s_drive.control.actuator.baseline_proposal.feasible_angular_mrad_s,
             s_drive.control.actuator.final_proposal.feasible_angular_mrad_s,
             feedback,
             s_drive.control.yaw.pi.mode,
             &s_drive.control.tracking.limits) == MOTION_TRACKING_STOP_REQUESTED)) {
#if DRIVE_BASIC_BRINGUP_PROFILE
        JetsonTx_Printf(
            JETSON_TX_PRIORITY_STATUS,
            "DRIVE tracking diagnostic reason=%s error=%ld actual=%ld submitted=%ld profile=BASIC observational=1\r\n",
            Motion_TrackingReasonName(
                s_drive.control.tracking.supervisor.reason),
            (long)s_drive.control.tracking.supervisor.error_mrad_s,
            (long)feedback->angular_velocity_mrad_s,
            (long)s_drive.control.actuator.final_proposal
                .feasible_angular_mrad_s);
#else
        DriveCore_SetTrackingStopLatched(true);
        DriveCore_SetTrackingStopIgnoreLogged(false);
        JetsonTx_Printf(JETSON_TX_PRIORITY_CRITICAL,
                    "DRIVE tracking safe-stop reason=%s error=%ld actual=%ld submitted=%ld\r\n",
                    Motion_TrackingReasonName(s_drive.control.tracking.supervisor.reason),
                    (long)s_drive.control.tracking.supervisor.error_mrad_s,
                    (long)feedback->angular_velocity_mrad_s,
                    (long)s_drive.control.actuator.final_proposal.feasible_angular_mrad_s);
        Drive_ClearMotionCommandStateOnly();
        (void)Drive_StopBoth();
#endif
    }
}
#endif

#if ENABLE_JETSON_UART_MODE
void Drive_ControlLoop_Update(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t control_elapsed_ms = (uint32_t)(now - s_drive.timing.control_last_ms);
    bool feedback_request_window = false; /* No background wheel SDO in motion. */
    bool runtime_target_pending =
        Drive_NormalMotionAllowed() &&
        (s_drive.control.new_cmd_vel_pending ||
         (control_elapsed_ms >= DRIVE_CONTROL_PERIOD_MS)) &&
        (!Drive_CmdVelIsZero(s_drive.control.target_linear_mm_s,
                             s_drive.control.target_angular_mrad_s) ||
         !Drive_LastCommandTargetIsZero());
    CanopenWheelFeedbackSchedule_t feedback_schedule;
    bool target_zero;
    bool nominal_feasible_zero;
    bool pi_prepared;
    MotionTrackingLimits_t proposed_limits;
    bool target_changed;
    bool refresh_due;

    if (Drive_BootInProgress()) { Canopen_BootSdoBegin(); }

    feedback_schedule = Drive_ControlFeedbackSchedule(
        now, feedback_request_window, runtime_target_pending);
    /* This bounded dispatcher runs every cooperative-loop iteration, not only
     * at the 25 ms control cadence. Initialization and STOP confirmation may
     * request 0x606C while SDO still owns feedback; qualified TPDO authority
     * suppresses runtime 0x606C polling. */
    Drive_UpdateWheelFeedback(now, feedback_schedule);
    Drive_ServiceWheelTpdoActiveCommand(now);
    /* H1 supervision is independent of Drive state and SDO ownership. RX is
     * dispatched first so a heartbeat already in FIFO is considered before
     * the 350 ms stale-age decision; no state-specific return precedes it. */
    Canopen_HeartbeatService(now);
    if (Drive_ControlConsumeHeartbeatFault(now)) {
        return;
    }
    if (Drive_ControlConsumeWheelRuntimeFault(now)) {
        return;
    }
    if (Drive_ControlConsumeFeedbackContinuityFault(now)) {
        return;
    }
    /* RX and existing bus/heartbeat safety always run before retry admission. */
    Drive_BootService(now);
    if (s_drive.boot.state == DRIVE_BOOT_WAIT_RETRY) { return; }
    /* Watchdog and Lift preempt pending CAN frames on every cooperative turn. */
    bool motion_preempted = Drive_ControlHandleJetsonWatchdog(now);
    motion_preempted = Drive_ControlHandleMovingLift(now) || motion_preempted;
    bool completed_pi = false;
    if (DriveTarget_Service(now, &completed_pi)) {
        Drive_ControlUpdateTrackingSupervisor(completed_pi, DriveTarget_CommittedFeedback());
    }
    if (Drive_ControlServiceSafetyTransactions(now)) {
        return;
    }
    if (s_drive.boot.state == DRIVE_BOOT_WAIT_RETRY) { return; }
    /* Production startup is not complete until both configured TPDO streams
     * have formed a fresh coherent pair and become feedback authority. A
     * startup TPDO failure remains a closed gate for normal Drive control. */
    if (!Drive_ServiceWheelTpdoStartup(now)) {
        return;
    }

    /* An active Lift inhibits motion, but must not starve Drive STOP/disable
     * transactions above. Their hardware cancellation/zero proof still runs. */
    if (motion_preempted) { return; }

    if (control_elapsed_ms < DRIVE_CONTROL_PERIOD_MS) {
        return;
    }
    s_drive.timing.control_last_ms = now;
    DriveTarget_RecordControl(now);
    if (DriveTarget_Pending() || !Drive_NormalMotionAllowed()) { return; }

    target_zero = Drive_CmdVelIsZero(s_drive.control.target_linear_mm_s,
                                     s_drive.control.target_angular_mrad_s);
    if (Drive_ControlHandleJetsonWatchdog(now)) {
        return;
    }
    nominal_feasible_zero = Drive_CmdVelIsZero(
        s_drive.control.nominal_feasible_linear_mm_s,
        s_drive.control.nominal_feasible_angular_mrad_s);
    if (Drive_IdleAutoDisable_Update(now, target_zero)) {
        return;
    }
    if (!s_drive.command.jetson_cmd_active &&
        !s_drive.control.new_cmd_vel_pending &&
        target_zero &&
        nominal_feasible_zero &&
        Drive_LastCommandTargetIsZero()) {
        return;
    }
    if (Drive_ControlHandleMovingLift(now)) {
        return;
    }
    if (!Drive_ControlPrepareProposal(
            now, target_zero, nominal_feasible_zero,
            &pi_prepared, &proposed_limits,
            &target_changed, &refresh_due)) {
        return;
    }
    Drive_ControlSubmitProposal(now, pi_prepared, proposed_limits,
                                target_changed, refresh_due);
}
#endif

#ifdef AMR_HOST_TEST
void Drive_TestResetControlPrivateState(void)
{



    memset(&s_drive.diagnostic.sync_pair, 0,
           sizeof(s_drive.diagnostic.sync_pair));
}
#endif

#ifdef AMR_HOST_TEST
bool Drive_TestActuatorStateValid(void)
{
    return s_drive.control.actuator.state.valid;
}

int32_t Drive_TestLastLeftCommandDec(void)
{
    return s_drive.command.last_left_cmd_dec;
}

int32_t Drive_TestLastRightCommandDec(void)
{
    return s_drive.command.last_right_cmd_dec;
}

bool Drive_TestNewCmdVelPending(void)
{
    return s_drive.control.new_cmd_vel_pending;
}

uint32_t Drive_TestSyncPairAttemptCount(void)
{
    return s_drive.diagnostic.sync_pair.attempt_count;
}

uint32_t Drive_TestSyncPairSuccessCount(void)
{
    return s_drive.diagnostic.sync_pair.success_count;
}

uint32_t Drive_TestSyncPairFailureCount(void)
{
    return s_drive.diagnostic.sync_pair.failure_count;
}

uint32_t Drive_TestSyncPairLastFailedNode(void)
{
    return s_drive.diagnostic.sync_pair.last_failed_node;
}

uint32_t Drive_TestSyncPairLastLeftDurationMs(void)
{
    return s_drive.diagnostic.sync_pair.last_left_duration_ms;
}

uint32_t Drive_TestSyncPairLastRightDurationMs(void)
{
    return s_drive.diagnostic.sync_pair.last_right_duration_ms;
}

uint32_t Drive_TestSyncPairLastTotalDurationMs(void)
{
    return s_drive.diagnostic.sync_pair.last_total_duration_ms;
}

uint32_t Drive_TestSyncPairMaximumTotalDurationMs(void)
{
    return s_drive.diagnostic.sync_pair.maximum_total_duration_ms;
}
#endif

#ifdef AMR_HOST_TEST
void Drive_TestSetRuntimeReady(DriveState_t state,
                               float submitted_linear_mm_s,
                               float submitted_angular_mrad_s,
                               uint32_t timestamp_ms)
{
    MotionMotorTargets_t targets;
    MotionActuatorProposal_t submitted = {0};

    Drive_EnsureMotionControlInitialized();
    s_drive.boot.state = DRIVE_BOOT_READY;
    Canopen_BootSdoEnd();
    Motion_CmdVelToMotorTargets(submitted_linear_mm_s,
                                submitted_angular_mrad_s,
                                &targets);
    submitted.left_motor_rpm = targets.left_motor_rpm;
    submitted.right_motor_rpm = targets.right_motor_rpm;
    submitted.left_velocity_dec = targets.left_velocity_dec;
    submitted.right_velocity_dec = targets.right_velocity_dec;
    submitted.feasible_linear_mm_s = targets.feasible_linear_mm_s;
    submitted.feasible_angular_mrad_s = targets.feasible_angular_mrad_s;
    Motion_ActuatorCommit(&s_drive.control.actuator.state, &submitted, timestamp_ms);
    s_drive.command.last_left_cmd_rpm = Motion_RoundFloatToI32(targets.left_motor_rpm);
    s_drive.command.last_right_cmd_rpm = Motion_RoundFloatToI32(targets.right_motor_rpm);
    s_drive.command.last_left_cmd_dec = targets.left_velocity_dec;
    s_drive.command.last_right_cmd_dec = targets.right_velocity_dec;
    s_drive.control.nominal_feasible_linear_mm_s = targets.feasible_linear_mm_s;
    s_drive.control.nominal_feasible_angular_mrad_s = targets.feasible_angular_mrad_s;
    DriveCore_SetSoftwareEnabled((state == DRIVE_STATE_ENABLED_IDLE) ||
                      (state == DRIVE_STATE_RUNNING));
    DriveCore_SetHardwareEnableConfirmed(s_drive.core.enabled);
    DriveCore_SetHardwareDisableConfirmed(
        (state == DRIVE_STATE_DISABLED));
    DriveCore_WriteStateUnchecked(state);
    memset(&s_drive.safety.transition, 0, sizeof(s_drive.safety.transition));
    DriveCore_SetMotionInhibitLatched(false);
    Drive_InvalidateZeroSpeedEvidence();
    if ((state == DRIVE_STATE_ENABLED_IDLE) &&
        (fabsf(submitted_linear_mm_s) < 0.001f) &&
        (fabsf(submitted_angular_mrad_s) < 0.001f)) {
        Motion_FeedbackUpdateRawVelocity(
            &s_drive.feedback.motion, LEFT_NODE_ID, 0,
            timestamp_ms, 1U);
        Motion_FeedbackUpdateRawVelocity(
            &s_drive.feedback.motion, RIGHT_NODE_ID, 0,
            timestamp_ms, 1U);
        s_drive.feedback.chassis = Motion_FeedbackEvaluate(
            &s_drive.feedback.motion, timestamp_ms,
            DRIVE_WHEEL_FEEDBACK_STALE_MS,
            DRIVE_WHEEL_FEEDBACK_MAX_SKEW_MS);
        s_drive.feedback.status_left_rpm = 0;
        s_drive.feedback.status_right_rpm = 0;
        s_drive.feedback.status_left_timestamp_ms = timestamp_ms;
        s_drive.feedback.status_right_timestamp_ms = timestamp_ms;
        s_drive.feedback.status_left_seen = true;
        s_drive.feedback.status_right_seen = true;
        s_drive.feedback.status_left_valid = true;
        s_drive.feedback.status_right_valid = true;
        s_drive.evidence.safety_generation = 1U;
        Drive_RecordZeroSpeedEvidence(
            s_drive.evidence.safety_generation, timestamp_ms,
            DRIVE_ZERO_EVIDENCE_STOP);
    }
    g_odom.initialized = true;
}
#endif

#ifdef AMR_HOST_TEST
void Drive_TestStoreFeedbackPair(uint32_t sequence,
                                 uint32_t timestamp_ms,
                                 float left_wheel_mm_s,
                                 float right_wheel_mm_s)
{
    float wheel_circumference_mm = ODOM_PI * WHEEL_DIAMETER_MM;
    float left_logical_motor_rpm =
        left_wheel_mm_s / wheel_circumference_mm * 60.0f * GEAR_RATIO;
    float right_logical_motor_rpm =
        right_wheel_mm_s / wheel_circumference_mm * 60.0f * GEAR_RATIO;
    int32_t left_dec = Motion_RoundFloatToI32(
        (float)LEFT_MOTOR_FORWARD_SIGN * left_logical_motor_rpm *
        512.0f * ENCODER_RESOLUTION / 1875.0f);
    int32_t right_dec = Motion_RoundFloatToI32(
        (float)RIGHT_MOTOR_FORWARD_SIGN * right_logical_motor_rpm *
        512.0f * ENCODER_RESOLUTION / 1875.0f);
    MotionChassisFeedback_t evaluated;

    Motion_FeedbackUpdateRawVelocity(&s_drive.feedback.motion, LEFT_NODE_ID,
                                     left_dec, timestamp_ms, sequence);
    Motion_FeedbackUpdateRawVelocity(&s_drive.feedback.motion, RIGHT_NODE_ID,
                                     right_dec, timestamp_ms, sequence);
    evaluated = Motion_FeedbackEvaluate(&s_drive.feedback.motion, timestamp_ms,
                                        DRIVE_WHEEL_FEEDBACK_STALE_MS,
                                        DRIVE_WHEEL_FEEDBACK_MAX_SKEW_MS);
    if (evaluated.new_sample) {
        s_drive.control.yaw.waiting_for_new_pair = false;
        if (s_drive.feedback.pair_pending &&
            (s_drive.control.yaw.feedback_pair_replaced_count != UINT32_MAX)) {
            s_drive.control.yaw.feedback_pair_replaced_count++;
        }
        s_drive.feedback.pair_pending = true;
    }
    s_drive.feedback.chassis = evaluated;
    s_drive.feedback.status_left_rpm =
        Motion_RoundFloatToI32(left_logical_motor_rpm);
    s_drive.feedback.status_right_rpm =
        Motion_RoundFloatToI32(right_logical_motor_rpm);
    s_drive.feedback.status_left_timestamp_ms = timestamp_ms;
    s_drive.feedback.status_right_timestamp_ms = timestamp_ms;
    s_drive.feedback.status_left_seen = true;
    s_drive.feedback.status_right_seen = true;
    s_drive.feedback.status_left_valid = true;
    s_drive.feedback.status_right_valid = true;
    Drive_MaintainZeroSpeedEvidence(timestamp_ms);
}
#endif

#ifdef AMR_HOST_TEST
void Drive_TestStoreFeedbackNodes(uint32_t left_sequence,
                                  uint32_t right_sequence,
                                  uint32_t left_timestamp_ms,
                                  uint32_t right_timestamp_ms,
                                  float left_wheel_mm_s,
                                  float right_wheel_mm_s)
{
    float wheel_circumference_mm = ODOM_PI * WHEEL_DIAMETER_MM;
    float left_logical_motor_rpm =
        left_wheel_mm_s / wheel_circumference_mm * 60.0f * GEAR_RATIO;
    float right_logical_motor_rpm =
        right_wheel_mm_s / wheel_circumference_mm * 60.0f * GEAR_RATIO;
    int32_t left_dec = Motion_RoundFloatToI32(
        (float)LEFT_MOTOR_FORWARD_SIGN * left_logical_motor_rpm *
        512.0f * ENCODER_RESOLUTION / 1875.0f);
    int32_t right_dec = Motion_RoundFloatToI32(
        (float)RIGHT_MOTOR_FORWARD_SIGN * right_logical_motor_rpm *
        512.0f * ENCODER_RESOLUTION / 1875.0f);
    uint32_t now_ms = (left_timestamp_ms > right_timestamp_ms) ?
        left_timestamp_ms : right_timestamp_ms;

    Motion_FeedbackUpdateRawVelocity(&s_drive.feedback.motion, LEFT_NODE_ID,
                                     left_dec, left_timestamp_ms,
                                     left_sequence);
    Motion_FeedbackUpdateRawVelocity(&s_drive.feedback.motion, RIGHT_NODE_ID,
                                     right_dec, right_timestamp_ms,
                                     right_sequence);
    s_drive.feedback.chassis = Motion_FeedbackEvaluate(
        &s_drive.feedback.motion, now_ms,
        DRIVE_WHEEL_FEEDBACK_STALE_MS,
        DRIVE_WHEEL_FEEDBACK_MAX_SKEW_MS);
#if ENABLE_JETSON_UART_MODE
    Drive_ProcessFeedbackHealth();
#endif
    Drive_MaintainZeroSpeedEvidence(now_ms);
}
#endif

#ifdef AMR_HOST_TEST
void Drive_TestInvalidateFeedbackPair(void)
{
    s_drive.feedback.chassis.valid = false;
    s_drive.feedback.chassis.stale = true;
}
#endif

#ifdef AMR_HOST_TEST
void Drive_TestStoreFeedbackPairWithSkew(uint32_t sequence,
                                         uint32_t left_timestamp_ms,
                                         uint32_t right_timestamp_ms,
                                         float left_wheel_mm_s,
                                         float right_wheel_mm_s)
{
    float wheel_circumference_mm = ODOM_PI * WHEEL_DIAMETER_MM;
    float left_logical_motor_rpm =
        left_wheel_mm_s / wheel_circumference_mm * 60.0f * GEAR_RATIO;
    float right_logical_motor_rpm =
        right_wheel_mm_s / wheel_circumference_mm * 60.0f * GEAR_RATIO;
    int32_t left_dec = Motion_RoundFloatToI32(
        (float)LEFT_MOTOR_FORWARD_SIGN * left_logical_motor_rpm *
        512.0f * ENCODER_RESOLUTION / 1875.0f);
    int32_t right_dec = Motion_RoundFloatToI32(
        (float)RIGHT_MOTOR_FORWARD_SIGN * right_logical_motor_rpm *
        512.0f * ENCODER_RESOLUTION / 1875.0f);
    uint32_t now_ms = (left_timestamp_ms > right_timestamp_ms) ?
        left_timestamp_ms : right_timestamp_ms;

    Motion_FeedbackUpdateRawVelocity(&s_drive.feedback.motion, LEFT_NODE_ID,
                                     left_dec, left_timestamp_ms, sequence);
    Motion_FeedbackUpdateRawVelocity(&s_drive.feedback.motion, RIGHT_NODE_ID,
                                     right_dec, right_timestamp_ms, sequence);
    s_drive.feedback.chassis = Motion_FeedbackEvaluate(
        &s_drive.feedback.motion, now_ms,
        DRIVE_WHEEL_FEEDBACK_STALE_MS,
        DRIVE_WHEEL_FEEDBACK_MAX_SKEW_MS);
#if ENABLE_JETSON_UART_MODE
    Drive_ProcessFeedbackHealth();
#endif
    Drive_MaintainZeroSpeedEvidence(now_ms);
}
#endif

#ifdef AMR_HOST_TEST
uint32_t Drive_TestPiResetCount(void)
{
    return s_drive.control.yaw.retarget_reset_count;
}
#endif

#ifdef AMR_HOST_TEST
uint32_t Drive_TestPiLastSequence(void)
{
    return s_drive.control.yaw.pi.last_feedback_sequence;
}
#endif

#ifdef AMR_HOST_TEST
bool Drive_TestPiFeedbackSequenceValid(void)
{
    return s_drive.control.yaw.pi.feedback_sequence_valid;
}
#endif

#ifdef AMR_HOST_TEST
float Drive_TestPiIntegral(void) { return s_drive.control.yaw.pi.integral_mrad; }
#endif

#ifdef AMR_HOST_TEST
uint32_t Drive_TestPiPrepareCount(void) { return s_drive.control.yaw.prepare_count; }
#endif

#ifdef AMR_HOST_TEST
uint32_t Drive_TestSubmitCount(void) { return s_drive.control.actuator.successful_submit_count; }
#endif

#ifdef AMR_HOST_TEST
uint32_t Drive_TestFeedbackReplacedCount(void)
{
    return s_drive.control.yaw.feedback_pair_replaced_count;
}
#endif

#ifdef AMR_HOST_TEST
bool Drive_TestFeedbackPairPending(void)
{
    return s_drive.feedback.pair_pending;
}
#endif

#ifdef AMR_HOST_TEST
bool Drive_TestChassisFeedbackNewSample(void)
{
    return s_drive.feedback.chassis.new_sample;
}
#endif

#ifdef AMR_HOST_TEST
uint32_t Drive_TestTrackingLastSequence(void)
{
    return s_drive.control.tracking.supervisor.last_feedback_sequence;
}
#endif

#ifdef AMR_HOST_TEST
bool Drive_TestTrackingFeedbackSequenceValid(void)
{
    return s_drive.control.tracking.supervisor.feedback_sequence_valid;
}
#endif

#ifdef AMR_HOST_TEST
uint32_t Drive_TestTrackingState(void)
{
    return (uint32_t)s_drive.control.tracking.supervisor.state;
}
#endif

#ifdef AMR_HOST_TEST
uint32_t Drive_TestTrackingWrongSignObservationCount(void)
{
    return s_drive.control.tracking.supervisor.wrong_sign_observation_count;
}
#endif

#ifdef AMR_HOST_TEST
float Drive_TestSubmittedCorrectionMradS(void)
{
    return s_drive.control.yaw.submitted_correction_mrad_s;
}
#endif

#ifdef AMR_HOST_TEST
float Drive_TestSubmittedLinearMmS(void)
{
    return s_drive.control.actuator.state.feasible_linear_mm_s;
}
#endif

#ifdef AMR_HOST_TEST
float Drive_TestSubmittedAngularMradS(void)
{
    return s_drive.control.actuator.state.feasible_angular_mrad_s;
}
#endif

#ifdef AMR_HOST_TEST
float Drive_TestSubmittedLeftMotorRpm(void)
{
    return s_drive.control.actuator.state.left_motor_rpm;
}
#endif

#ifdef AMR_HOST_TEST
float Drive_TestSubmittedRightMotorRpm(void)
{
    return s_drive.control.actuator.state.right_motor_rpm;
}
#endif

#ifdef AMR_HOST_TEST
float Drive_TestRequestedLinearMmS(void)
{
    return s_drive.control.requested_linear_mm_s;
}
#endif

#ifdef AMR_HOST_TEST
float Drive_TestRequestedAngularMradS(void)
{
    return s_drive.control.requested_angular_mrad_s;
}
#endif

#ifdef AMR_HOST_TEST
float Drive_TestTargetLinearMmS(void)
{
    return s_drive.control.target_linear_mm_s;
}
#endif

#ifdef AMR_HOST_TEST
float Drive_TestTargetAngularMradS(void)
{
    return s_drive.control.target_angular_mrad_s;
}
#endif

#ifdef AMR_HOST_TEST
uint32_t Drive_TestLastJetsonCommandMs(void)
{
    return s_drive.command.last_jetson_cmd_ms;
}
#endif

#ifdef AMR_HOST_TEST
uint32_t Drive_TestCommunicationFaultStopRequestCount(void)
{
    return s_communication_fault_stop_request_count;
}
#endif
