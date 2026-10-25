#include "drive_target.h"
#include "drive_internal.h"
#include "wheel_rpdo_tx.h"
#include "wheel_pdo_config.h"
#include "app_config.h"
#include "jetson_tx.h"
#include "can_trace.h"
#include <string.h>

static struct {
    bool pending, pi_prepared;
    uint32_t proposal_ms, pi_reset_count, retarget_reset_count;
    MotionActuatorProposal_t actuator, baseline;
    MotionYawRatePIProposal_t pi;
    MotionChassisFeedback_t feedback;
    MotionTrackingLimits_t limits;
    float correction;
} target;
static uint32_t control_count, last_control_ms, max_control_gap_ms;
static uint32_t last_control_gap_ms, snapshot, busy_since;
static bool control_seen, busy_wait;
/* Time consumed by normal command shaping is distinct from the last real TX
 * commit. Holding an unchanged, already transmitted target spends time only;
 * it must not bank acceleration for a later retarget or fabricate a commit. */
static uint32_t shaping_ms;
static bool shaping_valid;

void DriveTarget_Cancel(uint32_t now)
{
    target.pending = false;
    target.pi_prepared = false;
    busy_wait = false;
    shaping_valid = false;
    WheelRpdoTx_Cancel(now);
}

MotionActuatorState_t DriveTarget_ShapingState(void)
{
    MotionActuatorState_t state = s_drive.control.actuator.state;
    if (state.valid && shaping_valid) { state.timestamp_ms = shaping_ms; }
    return state;
}

void DriveTarget_Hold(uint32_t now)
{
    if (!target.pending && WheelRpdoTx_Quiescent() &&
        s_drive.control.actuator.state.valid && Drive_NormalMotionAllowed()) {
        shaping_ms = now;
        shaping_valid = true;
        busy_wait = false; /* The latest command is already satisfied. */
    }
}

bool DriveTarget_Pending(void) { return target.pending; }
const MotionChassisFeedback_t *DriveTarget_CommittedFeedback(void) { return &target.feedback; }

static void TransportFault(uint32_t now, const char *reason)
{
    WheelRpdoDiagnostic diag = WheelRpdoTx_Get();
    JetsonTx_Printf(JETSON_TX_PRIORITY_CRITICAL,
        "DRIVE_RPDO_FAULT reason=%s node=%u sent=%lu pending=%lu stop_confirmed=0\r\n",
        reason, (unsigned)diag.failed_node, (unsigned long)diag.sent_mask,
        (unsigned long)diag.pending_mask);
    DriveTarget_Cancel(now);
    Drive_ClearMotionCommandStateOnly();
    Drive_EnterHardFault(DRIVE_SAFETY_FAILURE_RUNTIME_CAN, diag.failed_node, now);
}

void DriveTarget_Submit(uint32_t now, bool pi_prepared, MotionTrackingLimits_t limits)
{
    if (target.pending) {
        /* Existing pending proposal is serviced every main-loop turn. Never
         * prepare/commit it twice; new CMD_VEL overwrites only desired state. */
        return;
    }
    if (!Drive_NormalMotionAllowed()) { return; }
    WheelRpdoSubmit result = WheelRpdoTx_Submit(
        s_drive.control.actuator.final_proposal.left_velocity_dec,
        s_drive.control.actuator.final_proposal.right_velocity_dec, now);
    if (result == WHEEL_RPDO_BUSY) {
        Motion_ActuatorRejectBusy(&s_drive.control.actuator.state);
        s_drive.control.tracking.limits.transport_busy = true;
        s_drive.control.tracking.supervisor.transport_busy_observation_count++;
        s_drive.control.new_cmd_vel_pending = true;
        if (!busy_wait) { busy_wait = true; busy_since = now; }
        if ((uint32_t)(now - busy_since) > DRIVE_RUNTIME_TARGET_TIMEOUT_MS) {
            TransportFault(now, "CAPACITY_TIMEOUT");
        }
        return;
    }
    busy_wait = false;
    if (result != WHEEL_RPDO_ACCEPTED) {
        TransportFault(now, WheelRpdoTx_Get().reason ? WheelRpdoTx_Get().reason : "NOT_QUALIFIED");
        return;
    }
    target.pending = true;
    target.pi_prepared = pi_prepared;
    target.proposal_ms = now;
    target.actuator = s_drive.control.actuator.final_proposal;
    target.baseline = s_drive.control.actuator.baseline_proposal;
    target.pi = s_drive.control.yaw.proposal;
    target.feedback = s_drive.feedback.chassis;
    target.pi_reset_count = s_drive.control.yaw.pi.reset_count;
    target.retarget_reset_count = s_drive.control.yaw.retarget_reset_count;
    target.correction = s_drive.control.yaw.proposed_submitted_correction_mrad_s;
    target.limits = limits;
    Drive_InvalidateZeroSpeedEvidence();
}

bool DriveTarget_Service(uint32_t now, bool *pi_committed)
{
    *pi_committed = false;
    WheelRpdoTx_Service(now);
    if (!target.pending) { return false; }
    if (!Drive_NormalMotionAllowed() || s_drive.safety.transition.active ||
        s_drive.fault_reset.transaction.active) {
        DriveTarget_Cancel(now);
        return false;
    }
    WheelRpdoState state = WheelRpdoTx_State();
    if ((state == WHEEL_RPDO_FAULT) || (state == WHEEL_RPDO_CANCELLING)) {
        TransportFault(now, WheelRpdoTx_Get().reason ? WheelRpdoTx_Get().reason : "CANCELLED");
        return false;
    }
    if (state != WHEEL_RPDO_SENT) { return false; }
    /* Both CAN transmissions are complete. Commit the calculated command
     * epoch, not the later observation time: dt already used proposal_ms.
     * Drive application and stopping remain proven only by fresh feedback. */
    Motion_ActuatorCommit(&s_drive.control.actuator.state, &target.actuator,
                          target.proposal_ms);
    shaping_ms = target.proposal_ms;
    shaping_valid = true;
    s_drive.control.actuator.final_proposal = target.actuator;
    s_drive.control.actuator.baseline_proposal = target.baseline;
    s_drive.command.last_left_cmd_rpm = Motion_RoundFloatToI32(target.actuator.left_motor_rpm);
    s_drive.command.last_right_cmd_rpm = Motion_RoundFloatToI32(target.actuator.right_motor_rpm);
    s_drive.command.last_left_cmd_dec = target.actuator.left_velocity_dec;
    s_drive.command.last_right_cmd_dec = target.actuator.right_velocity_dec;
    s_drive.control.nominal_feasible_linear_mm_s = target.baseline.feasible_linear_mm_s;
    s_drive.control.nominal_feasible_angular_mrad_s = target.baseline.feasible_angular_mrad_s;
    s_drive.control.yaw.submitted_correction_mrad_s = target.correction;
    if (target.pi_prepared &&
        (target.pi_reset_count == s_drive.control.yaw.pi.reset_count) &&
        (target.retarget_reset_count == s_drive.control.yaw.retarget_reset_count)) {
        Motion_YawRatePICommit(&s_drive.control.yaw.pi, &s_yaw_rate_pi_config,
            &target.pi, target.correction,
            target.limits.hard_wheel_saturated || target.limits.final_rate_limited);
        if (s_drive.feedback.chassis.sequence == target.pi.feedback_sequence) {
            s_drive.feedback.pair_pending = false;
        }
        *pi_committed = true;
    }
    s_drive.control.tracking.limits = target.limits;
    s_drive.control.actuator.successful_submit_count++;
    s_drive.timing.control_last_can_write_ms = now;
    /* Leave latest command pending until next calculation; a command may
     * have arrived while this pair was on the wire. */
    s_drive.control.new_cmd_vel_pending = true;
    DriveCore_SetState(DRIVE_STATE_RUNNING);
    target.pending = false;
    WheelRpdoTx_ConsumeSent();
    return true;
}

void DriveTarget_RecordControl(uint32_t now)
{
    last_control_gap_ms = control_seen ? (uint32_t)(now - last_control_ms) : 0U;
    if (last_control_gap_ms > max_control_gap_ms) { max_control_gap_ms = last_control_gap_ms; }
    control_seen = true;
    last_control_ms = now;
    control_count++;
}

void DriveTarget_PrintDiagnostics(uint32_t req)
{
    WheelRpdoDiagnostic d = WheelRpdoTx_Get();
    uint32_t snap = ++snapshot;
    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
        "DRIVE_RPDO 1/9 control=%lu period=%u gap=%lu max_gap=%lu queued=%lu sent=%lu state=%u ready=%u t_ms=%lu snap=%lu req=%08lX\r\n",
        (unsigned long)control_count, DRIVE_CONTROL_PERIOD_MS,
        (unsigned long)last_control_gap_ms, (unsigned long)max_control_gap_ms,
        (unsigned long)d.queued_pairs, (unsigned long)d.sent_pairs, (unsigned)d.state,
        WheelPdoConfig_Ready(HAL_GetTick()) ? 1U : 0U,
        (unsigned long)last_control_ms,
        (unsigned long)snap, (unsigned long)req);
    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
        "DRIVE_RPDO 2/9 interval_ms=%lu latency_ms=%lu max_latency=%lu skew_ticks=%lu max_skew=%lu pending=%lu snap=%lu req=%08lX\r\n",
        (unsigned long)d.last_interval_ms, (unsigned long)d.last_latency_ms,
        (unsigned long)d.max_latency_ms, (unsigned long)d.last_skew_ticks,
        (unsigned long)d.max_skew_ticks, (unsigned long)(hfdcan1.Instance->TXBRP & 3U),
        (unsigned long)snap, (unsigned long)req);
    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
        "DRIVE_RPDO 3/9 busy=%lu failed=%lu cancel=%lu bus_off=%lu cancel_timeout=%lu reason=%s snap=%lu req=%08lX\r\n",
        (unsigned long)d.busy_count, (unsigned long)d.failure_count,
        (unsigned long)d.cancel_count, (unsigned long)d.bus_off_count,
        (unsigned long)d.cancel_timeout_count, d.reason ? d.reason : "NONE",
        (unsigned long)snap, (unsigned long)req);
    for (uint8_t node = LEFT_NODE_ID; node <= RIGHT_NODE_ID; ++node) {
        WheelPdoConfigDiagnostic p = WheelPdoConfig_Get(node);
        JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
            "DRIVE_RPDO %u/9 node=%u cfg=%u op=%u epoch=%lu identity=NOT_QUERIED phase=%s expected=%08lX actual=%08lX snap=%lu req=%08lX\r\n",
            (unsigned)(node + 3U), (unsigned)node, p.configured ? 1U : 0U,
            p.operational ? 1U : 0U, (unsigned long)p.epoch,
            p.phase ? p.phase : "NOT_STARTED", (unsigned long)p.expected,
            (unsigned long)p.actual,
            (unsigned long)snap, (unsigned long)req);
        JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
            "DRIVE_RPDO %u/9 node=%u cob=%08lX type=%lu count=%lu map=%08lX step=%u idx=%04X sub=%02X abort=%08lX result=%s snap=%lu req=%08lX\r\n",
            (unsigned)(node + 5U), (unsigned)node,
            (unsigned long)p.readback[0], (unsigned long)p.readback[1],
            (unsigned long)p.readback[2], (unsigned long)p.readback[3],
            (unsigned)p.step, (unsigned)p.index, (unsigned)p.subindex,
            (unsigned long)p.abort_code, p.reason ? p.reason : "NOT_RUN",
            (unsigned long)snap, (unsigned long)req);
    }
    CanTraceRuntimeStats_t runtime = CanTrace_GetRuntimeStats(CAN_TRACE_RUNTIME_DRIVE_UPDATE);
    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
        "DRIVE_RPDO 8/9 runtime_ms=%lu max_runtime_ms=%lu runtime_gap_ms=%lu over20=%lu max_interval_ms=%lu snap=%lu req=%08lX\r\n",
        (unsigned long)runtime.current_ms, (unsigned long)runtime.maximum_ms,
        (unsigned long)runtime.current_gap_ms, (unsigned long)runtime.over_20ms_count,
        (unsigned long)d.max_interval_ms, (unsigned long)snap, (unsigned long)req);
    Drive_PrintPdoStartupDiagnostic(HAL_GetTick(), snap, req);
}
