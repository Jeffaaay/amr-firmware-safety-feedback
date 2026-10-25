/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : drive.c
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

void Drive_ConsoleApplyCmdVel(float linear_mm_s, float angular_mrad_s)
{
    s_drive.command.last_jetson_cmd_ms = HAL_GetTick();
    s_drive.command.jetson_cmd_active = true;
    Drive_ApplyCmdVel(linear_mm_s, angular_mrad_s);
}

#ifdef AMR_HOST_TEST
void Drive_TestResetState(void)
{
    memset(&s_drive.boot, 0, sizeof(s_drive.boot));
    s_drive.statusword.left_statusword = 0U;
    s_drive.statusword.right_statusword = 0U;
    s_drive.statusword.left_statusword_valid = false;
    s_drive.statusword.right_statusword_valid = false;
    s_drive.statusword.left_statusword_timestamp_ms = 0U;
    s_drive.statusword.right_statusword_timestamp_ms = 0U;
    s_drive.statusword.left_statusword_sequence = 0U;
    s_drive.statusword.right_statusword_sequence = 0U;
    s_drive.statusword.left_statusword_generation = 0U;
    s_drive.statusword.right_statusword_generation = 0U;
    DriveCore_SetSoftwareEnabled(false);
    DriveCore_WriteStateUnchecked(DRIVE_STATE_INIT);
    s_drive.command.last_jetson_cmd_ms = 0U;
    s_drive.command.jetson_cmd_active = false;
    s_drive.command.last_left_cmd_rpm = 0;
    s_drive.command.last_right_cmd_rpm = 0;
    s_drive.command.last_left_cmd_dec = 0;
    s_drive.command.last_right_cmd_dec = 0;
    memset(&s_drive.feedback.motion, 0, sizeof(s_drive.feedback.motion));
    memset(&s_drive.feedback.chassis, 0, sizeof(s_drive.feedback.chassis));
    memset(&s_drive.tpdo_active, 0, sizeof(s_drive.tpdo_active));
    memset(&s_drive.control.yaw.pi, 0, sizeof(s_drive.control.yaw.pi));
    memset(&s_drive.control.yaw.proposal, 0, sizeof(s_drive.control.yaw.proposal));
    s_yaw_pi_enabled = (DRIVE_YAW_RATE_PI_DEFAULT_ENABLED != 0U);
    s_drive.control.yaw.waiting_for_new_pair = false;
    memset(&s_drive.control.tracking.supervisor, 0, sizeof(s_drive.control.tracking.supervisor));
    memset(&s_drive.control.tracking.limits, 0, sizeof(s_drive.control.tracking.limits));
    memset(&s_drive.control.actuator.body_command, 0, sizeof(s_drive.control.actuator.body_command));
    memset(&s_drive.control.actuator.state, 0, sizeof(s_drive.control.actuator.state));
    memset(&s_drive.control.actuator.baseline_proposal, 0,
           sizeof(s_drive.control.actuator.baseline_proposal));
    memset(&s_drive.control.actuator.final_proposal, 0,
           sizeof(s_drive.control.actuator.final_proposal));
    memset(&s_drive.safety.transition, 0, sizeof(s_drive.safety.transition));
    memset(&s_drive.fault_reset.transaction, 0, sizeof(s_drive.fault_reset.transaction));
    s_drive.fault_reset.generation = 0U;
    memset(&s_drive.diagnostic.last_fault_snapshot, 0,
           sizeof(s_drive.diagnostic.last_fault_snapshot));
    memset(&s_drive.lift_stop.request, 0, sizeof(s_drive.lift_stop.request));
    memset(&s_drive.stable_zero, 0, sizeof(s_drive.stable_zero));
    s_drive.lift_stop.base_safe_refresh_txn = 0U;
    s_drive.diagnostic.base_safe_snapshot_sequence = 0U;
    s_drive.diagnostic.yaw_snapshot_sequence = 0U;
    s_drive.diagnostic.fault_debug_snapshot_sequence = 0U;
    s_drive.diagnostic.heartbeat_snapshot_sequence = 0U;
    s_drive.control.actuator.initialized = false;
    s_drive.feedback.pair_pending = false;
    s_drive.feedback.issue_active = false;
    s_drive.feedback.observed_stale_pair_reject_count = 0U;
    s_drive.feedback.observed_skew_pair_reject_count = 0U;
    s_drive.control.yaw.corrected_angular_mrad_s = 0.0f;
    s_drive.control.yaw.submitted_correction_mrad_s = 0.0f;
    s_drive.control.yaw.proposed_submitted_correction_mrad_s = 0.0f;
    DriveCore_SetTrackingStopLatched(false);
    DriveCore_SetTrackingStopIgnoreLogged(false);
    DriveCore_SetMotionInhibitLatched(false);
    DriveCore_SetHardwareEnableConfirmed(false);
    DriveCore_SetHardwareDisableConfirmed(false);
#ifdef AMR_HOST_TEST
    s_communication_fault_stop_request_count = 0U;
    Drive_TestSetTpdoEstopActive(false);
    Drive_TestResetControlPrivateState();
#endif
    memset(&s_drive.evidence.zero_speed, 0, sizeof(s_drive.evidence.zero_speed));
    s_drive.evidence.safety_generation = 0U;
    s_drive.evidence.post_disable_baseline_pair_sequence = 0U;
    s_drive.evidence.post_disable_baseline_left_sequence = 0U;
    s_drive.evidence.post_disable_baseline_right_sequence = 0U;
    s_drive.evidence.post_disable_zero_proof_required = false;
    s_drive.control.yaw.feedback_pair_replaced_count = 0U;
    s_drive.control.yaw.prepare_count = 0U;
    s_drive.control.yaw.retarget_reset_count = 0U;
    s_drive.control.actuator.successful_submit_count = 0U;
    s_drive.feedback.odom_left_position_sequence = 0U;
    s_drive.feedback.odom_right_position_sequence = 0U;
    s_drive.feedback.odom_authoritative_pair_sequence = 0U;
    s_drive.feedback.odom_authoritative_pair_timestamp_ms = 0U;
    s_drive.timing.control_last_ms = 0U;
    s_drive.timing.control_last_can_write_ms = 0U;
    s_drive.timing.control_last_summary_ms = 0U;
    s_drive.timing.status_last_ms = 0U;
    s_drive.feedback.status_left_rpm = 0;
    s_drive.feedback.status_right_rpm = 0;
    s_drive.feedback.status_left_timestamp_ms = 0U;
    s_drive.feedback.status_right_timestamp_ms = 0U;
    s_drive.feedback.status_left_seen = false;
    s_drive.feedback.status_right_seen = false;
    s_drive.feedback.status_left_valid = false;
    s_drive.feedback.status_right_valid = false;
#if ENABLE_JETSON_UART_MODE
    s_drive.control.requested_linear_mm_s = 0.0f;
    s_drive.control.requested_angular_mrad_s = 0.0f;
    s_drive.control.target_linear_mm_s = 0.0f;
    s_drive.control.target_angular_mrad_s = 0.0f;
    s_drive.control.nominal_feasible_linear_mm_s = 0.0f;
    s_drive.control.nominal_feasible_angular_mrad_s = 0.0f;
    s_drive.control.new_cmd_vel_pending = false;
#endif
    Drive_EnsureMotionControlInitialized();
}
#endif

void Drive_Console_MoveBoth(int32_t left_rpm,
                                   int32_t right_rpm,
                                   uint32_t duration_ms)
{
    if (Drive_EnsureEnabled()) {
        (void)Drive_RunBothMove(left_rpm, right_rpm, duration_ms);
    }
    Drive_StopBoth();
}

void Drive_Console_MoveNode(uint8_t node_id, int32_t rpm, uint32_t duration_ms)
{
    if (Drive_EnsureEnabled()) {
        (void)Drive_RunNodeMove(node_id, rpm, duration_ms);
    }
    (void)Kinco_StopNode(node_id);
}

void Drive_Console_MoveBothOdom(int32_t left_rpm,
                                       int32_t right_rpm,
                                       uint32_t duration_ms)
{
    if (!Odom_Reset()) {
        uart_printf("ODOM command aborted because Odom_Reset failed\r\n");
        return;
    }

    if (Drive_EnsureEnabled()) {
        (void)Drive_RunBothMoveOdom(left_rpm, right_rpm, duration_ms);
    }

    Drive_StopBoth();
}
