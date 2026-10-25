/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : motion_control.c
  * @brief          : Pure differential-drive feedback and yaw-rate control.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "motion_control.h"
#include "app_config.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static float Motion_ClampFloat(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static void Motion_IncrementU32(uint32_t *value)
{
    if (*value != UINT32_MAX) {
        (*value)++;
    }
}

static void Motion_IncrementU8(uint8_t *value)
{
    if (*value != UINT8_MAX) {
        (*value)++;
    }
}

static int32_t Motion_ForwardSign(uint8_t node_id)
{
    if (node_id == LEFT_NODE_ID) {
        return LEFT_MOTOR_FORWARD_SIGN;
    }
    if (node_id == RIGHT_NODE_ID) {
        return RIGHT_MOTOR_FORWARD_SIGN;
    }
    return 1;
}

static uint32_t Motion_TimestampSkewMs(uint32_t first, uint32_t second)
{
    int32_t signed_delta = (int32_t)(first - second);

    if (signed_delta >= 0) {
        return (uint32_t)signed_delta;
    }
    return (uint32_t)(-(int64_t)signed_delta);
}

static uint32_t Motion_LaterTimestamp(uint32_t first, uint32_t second)
{
    return ((int32_t)(first - second) >= 0) ? first : second;
}

int32_t Motion_RoundFloatToI32(float value)
{
    if (value >= 0.0f) {
        return (int32_t)(value + 0.5f);
    }
    return (int32_t)(value - 0.5f);
}

int32_t Motion_ApplyMotorForwardSignI32(uint8_t node_id, int32_t raw_value)
{
    if (Motion_ForwardSign(node_id) < 0) {
        return (int32_t)(0U - (uint32_t)raw_value);
    }
    return raw_value;
}

int32_t Motion_RawMotorRPMToVelocityDec(int32_t raw_motor_rpm)
{
    int64_t numerator = (int64_t)raw_motor_rpm * 512LL * 65536LL;
    return (int32_t)(numerator / 1875LL);
}

float Motion_RawVelocityDecToMotorRPM(int32_t raw_velocity_dec)
{
    return ((float)raw_velocity_dec * 1875.0f) /
           (512.0f * ENCODER_RESOLUTION);
}

float Motion_RawVelocityDecToLogicalMotorRPM(uint8_t node_id,
                                             int32_t raw_velocity_dec)
{
    return (float)Motion_ForwardSign(node_id) *
           Motion_RawVelocityDecToMotorRPM(raw_velocity_dec);
}

float Motion_LogicalMotorRPMToWheelVelocityMmS(float logical_motor_rpm)
{
    float wheel_rpm = logical_motor_rpm / GEAR_RATIO;
    return wheel_rpm * (ODOM_PI * WHEEL_DIAMETER_MM) / 60.0f;
}

float Motion_MotorDeltaCountsToWheelDistanceMm(uint8_t node_id,
                                               int32_t raw_delta_counts)
{
    float logical_counts = (float)Motion_ApplyMotorForwardSignI32(
        node_id, raw_delta_counts);
    float motor_revolutions = logical_counts / ENCODER_RESOLUTION;
    float wheel_revolutions = motor_revolutions / GEAR_RATIO;
    return wheel_revolutions * ODOM_PI * WHEEL_DIAMETER_MM;
}

float Motion_WheelVelocitiesToAngularMradS(float left_wheel_mm_s,
                                           float right_wheel_mm_s)
{
    return ((right_wheel_mm_s - left_wheel_mm_s) / WHEEL_BASE_MM) * 1000.0f;
}

void Motion_LimitBodyCommand(float linear_mm_s,
                             float angular_mrad_s,
                             float maximum_linear_mm_s,
                             float maximum_angular_mrad_s,
                             MotionBodyCommand_t *command)
{
    if (command == NULL) {
        return;
    }

    memset(command, 0, sizeof(*command));
    command->requested_linear_mm_s = linear_mm_s;
    command->requested_angular_mrad_s = angular_mrad_s;
    command->limited_linear_mm_s = Motion_ClampFloat(
        linear_mm_s, -maximum_linear_mm_s, maximum_linear_mm_s);
    command->limited_angular_mrad_s = Motion_ClampFloat(
        angular_mrad_s, -maximum_angular_mrad_s, maximum_angular_mrad_s);
    command->linear_limited = (command->limited_linear_mm_s != linear_mm_s);
    command->angular_limited = (command->limited_angular_mrad_s != angular_mrad_s);
}

void Motion_CalculateCommandEnvelope(float hard_motor_rpm,
                                     float reserved_yaw_correction_mrad_s,
                                     MotionCommandEnvelope_t *envelope)
{
    if (envelope == NULL) {
        return;
    }
    memset(envelope, 0, sizeof(*envelope));
    envelope->hard_motor_rpm = fabsf(hard_motor_rpm);
    envelope->hard_wheel_mm_s = fabsf(
        Motion_LogicalMotorRPMToWheelVelocityMmS(
            envelope->hard_motor_rpm));
    envelope->reserved_yaw_correction_mrad_s =
        fabsf(reserved_yaw_correction_mrad_s);
    envelope->reserved_wheel_mm_s =
        (envelope->reserved_yaw_correction_mrad_s / 1000.0f) *
        WHEEL_BASE_MM * 0.5f;
    envelope->nominal_wheel_mm_s = fmaxf(
        0.0f, envelope->hard_wheel_mm_s - envelope->reserved_wheel_mm_s);
    envelope->hard_linear_mm_s = envelope->hard_wheel_mm_s;
    envelope->hard_angular_mrad_s =
        (2.0f * envelope->hard_wheel_mm_s / WHEEL_BASE_MM) * 1000.0f;
    envelope->nominal_linear_mm_s = envelope->nominal_wheel_mm_s;
    envelope->nominal_angular_mrad_s =
        (2.0f * envelope->nominal_wheel_mm_s / WHEEL_BASE_MM) * 1000.0f;
}

void Motion_LimitBodyCommandWithYawHeadroom(
    float linear_mm_s,
    float angular_mrad_s,
    const MotionCommandEnvelope_t *envelope,
    MotionBodyCommand_t *command)
{
    float yaw_wheel_mm_s;
    float allowed_linear_mm_s;

    if ((envelope == NULL) || (command == NULL)) {
        return;
    }
    memset(command, 0, sizeof(*command));
    command->requested_linear_mm_s = linear_mm_s;
    command->requested_angular_mrad_s = angular_mrad_s;
    command->nominal_envelope_used = true;
    command->limited_angular_mrad_s = Motion_ClampFloat(
        angular_mrad_s,
        -envelope->nominal_angular_mrad_s,
        envelope->nominal_angular_mrad_s);
    yaw_wheel_mm_s = fabsf(command->limited_angular_mrad_s) /
                    1000.0f * WHEEL_BASE_MM * 0.5f;
    allowed_linear_mm_s = fmaxf(
        0.0f, envelope->nominal_wheel_mm_s - yaw_wheel_mm_s);
    command->allowed_linear_at_angular_mm_s = allowed_linear_mm_s;
    command->limited_linear_mm_s = Motion_ClampFloat(
        linear_mm_s, -allowed_linear_mm_s, allowed_linear_mm_s);
    command->linear_limited = (command->limited_linear_mm_s != linear_mm_s);
    command->angular_limited =
        (command->limited_angular_mrad_s != angular_mrad_s);
}

void Motion_CmdVelToMotorTargets(float linear_mm_s,
                                 float angular_mrad_s,
                                 MotionMotorTargets_t *targets)
{
    float omega_rad_s;
    float wheel_circumference_mm;
    float maximum_magnitude;
    float scale = 1.0f;

    if (targets == NULL) {
        return;
    }

    memset(targets, 0, sizeof(*targets));
    omega_rad_s = angular_mrad_s / 1000.0f;
    targets->prelimit_left_wheel_mm_s =
        linear_mm_s - (omega_rad_s * WHEEL_BASE_MM * 0.5f);
    targets->prelimit_right_wheel_mm_s =
        linear_mm_s + (omega_rad_s * WHEEL_BASE_MM * 0.5f);
    wheel_circumference_mm = ODOM_PI * WHEEL_DIAMETER_MM;

    targets->prelimit_left_motor_rpm =
        ((targets->prelimit_left_wheel_mm_s / wheel_circumference_mm) * 60.0f) *
        GEAR_RATIO;
    targets->prelimit_right_motor_rpm =
        ((targets->prelimit_right_wheel_mm_s / wheel_circumference_mm) * 60.0f) *
        GEAR_RATIO;

    maximum_magnitude = fmaxf(fabsf(targets->prelimit_left_motor_rpm),
                              fabsf(targets->prelimit_right_motor_rpm));
    if (maximum_magnitude > (float)MAX_TEST_MOTOR_RPM) {
        /* Uniform scaling preserves curvature and is the sole wheel-limit
         * layer. Independent wheel clamps would silently alter yaw rate. */
        scale = (float)MAX_TEST_MOTOR_RPM / maximum_magnitude;
    }

    targets->left_wheel_mm_s = targets->prelimit_left_wheel_mm_s * scale;
    targets->right_wheel_mm_s = targets->prelimit_right_wheel_mm_s * scale;
    targets->left_motor_rpm = targets->prelimit_left_motor_rpm * scale;
    targets->right_motor_rpm = targets->prelimit_right_motor_rpm * scale;
    targets->feasible_linear_mm_s =
        (targets->left_wheel_mm_s + targets->right_wheel_mm_s) * 0.5f;
    targets->feasible_angular_mrad_s = Motion_WheelVelocitiesToAngularMradS(
        targets->left_wheel_mm_s, targets->right_wheel_mm_s);
    {
        float rpm_per_mrad_s =
            ((WHEEL_BASE_MM * 0.5f / 1000.0f) / wheel_circumference_mm) *
            60.0f * GEAR_RATIO;
        float positive_motor_rpm = fmaxf(
            0.0f,
            fminf(targets->left_motor_rpm + (float)MAX_TEST_MOTOR_RPM,
                  (float)MAX_TEST_MOTOR_RPM - targets->right_motor_rpm));
        float negative_motor_rpm = fmaxf(
            0.0f,
            fminf((float)MAX_TEST_MOTOR_RPM - targets->left_motor_rpm,
                  targets->right_motor_rpm + (float)MAX_TEST_MOTOR_RPM));
        targets->remaining_positive_yaw_authority_mrad_s =
            (rpm_per_mrad_s > 0.0f) ?
            (positive_motor_rpm / rpm_per_mrad_s) : 0.0f;
        targets->remaining_negative_yaw_authority_mrad_s =
            (rpm_per_mrad_s > 0.0f) ?
            (negative_motor_rpm / rpm_per_mrad_s) : 0.0f;
    }
    targets->left_velocity_dec = Motion_RoundFloatToI32(
        (float)LEFT_MOTOR_FORWARD_SIGN * targets->left_motor_rpm *
        512.0f * ENCODER_RESOLUTION / 1875.0f);
    targets->right_velocity_dec = Motion_RoundFloatToI32(
        (float)RIGHT_MOTOR_FORWARD_SIGN * targets->right_motor_rpm *
        512.0f * ENCODER_RESOLUTION / 1875.0f);
    targets->saturation_scale = scale;
    targets->saturated = (scale < 1.0f);
}

float Motion_SelectDirectionalYawAuthority(
    const MotionMotorTargets_t *targets,
    float correction_mrad_s)
{
    if (targets == NULL) {
        return 0.0f;
    }
    return (correction_mrad_s < 0.0f) ?
        targets->remaining_negative_yaw_authority_mrad_s :
        targets->remaining_positive_yaw_authority_mrad_s;
}

void Motion_ActuatorStateInit(MotionActuatorState_t *state)
{
    if (state != NULL) {
        memset(state, 0, sizeof(*state));
    }
}

void Motion_ActuatorStateCommitZero(MotionActuatorState_t *state,
                                    uint32_t timestamp_ms)
{
    uint32_t submit_count;
    uint32_t busy_reject_count;

    if (state == NULL) {
        return;
    }
    submit_count = state->submit_count;
    busy_reject_count = state->busy_reject_count;
    memset(state, 0, sizeof(*state));
    state->valid = true;
    state->timestamp_ms = timestamp_ms;
    state->submit_count = submit_count;
    state->busy_reject_count = busy_reject_count;
    Motion_IncrementU32(&state->submit_count);
}

void Motion_ActuatorStateInvalidate(MotionActuatorState_t *state)
{
    if (state != NULL) {
        state->valid = false;
    }
}

static float Motion_JointWheelFeasibilityScale(float start_left_mm_s,
                                                float start_right_mm_s,
                                                float candidate_left_mm_s,
                                                float candidate_right_mm_s,
                                                float hard_wheel_mm_s)
{
    float scale = 1.0f;
    float start[2] = { start_left_mm_s, start_right_mm_s };
    float candidate[2] = { candidate_left_mm_s, candidate_right_mm_s };
    uint8_t index;

    for (index = 0U; index < 2U; index++) {
        float delta = candidate[index] - start[index];
        float wheel_scale = 1.0f;

        if ((candidate[index] > hard_wheel_mm_s) && (delta > 0.0f)) {
            wheel_scale = (hard_wheel_mm_s - start[index]) / delta;
        } else if ((candidate[index] < -hard_wheel_mm_s) &&
                   (delta < 0.0f)) {
            wheel_scale = (-hard_wheel_mm_s - start[index]) / delta;
        }
        scale = fminf(scale, Motion_ClampFloat(wheel_scale, 0.0f, 1.0f));
    }
    return scale;
}

void Motion_ActuatorPrepareBodyCommand(
    const MotionActuatorState_t *state,
    float desired_linear_mm_s,
    float desired_angular_mrad_s,
    uint32_t now_ms,
    uint32_t maximum_dt_ms,
    float maximum_linear_accel_mm_s2,
    float maximum_angular_accel_mrad_s2,
    float hard_motor_rpm,
    MotionActuatorProposal_t *proposal)
{
    MotionMotorTargets_t final_targets;
    float previous_linear_mm_s = 0.0f;
    float previous_angular_mrad_s = 0.0f;
    float hard_wheel_mm_s;
    float hard_angular_mrad_s;
    float hard_yaw_wheel_mm_s;
    float hard_allowed_linear_mm_s;
    float candidate_linear_mm_s;
    float candidate_angular_mrad_s;
    float previous_yaw_wheel_mm_s;
    float candidate_yaw_wheel_mm_s;
    float start_left_mm_s;
    float start_right_mm_s;
    float candidate_left_mm_s;
    float candidate_right_mm_s;
    float linear_step;
    float angular_step;
    float scale;
    uint32_t dt_ms = 0U;

    if (proposal == NULL) {
        return;
    }
    memset(proposal, 0, sizeof(*proposal));
    proposal->desired_linear_mm_s = desired_linear_mm_s;
    proposal->desired_angular_mrad_s = desired_angular_mrad_s;
    hard_wheel_mm_s = fabsf(Motion_LogicalMotorRPMToWheelVelocityMmS(
        fabsf(hard_motor_rpm)));
    hard_angular_mrad_s = (2.0f * hard_wheel_mm_s / WHEEL_BASE_MM) * 1000.0f;
    proposal->hard_feasible_angular_mrad_s = Motion_ClampFloat(
        desired_angular_mrad_s, -hard_angular_mrad_s, hard_angular_mrad_s);
    hard_yaw_wheel_mm_s = fabsf(proposal->hard_feasible_angular_mrad_s) /
                         1000.0f * WHEEL_BASE_MM * 0.5f;
    hard_allowed_linear_mm_s = fmaxf(0.0f,
                                     hard_wheel_mm_s - hard_yaw_wheel_mm_s);
    proposal->hard_feasible_linear_mm_s = Motion_ClampFloat(
        desired_linear_mm_s,
        -hard_allowed_linear_mm_s,
        hard_allowed_linear_mm_s);
    proposal->hard_limited =
        (proposal->hard_feasible_linear_mm_s != desired_linear_mm_s) ||
        (proposal->hard_feasible_angular_mrad_s != desired_angular_mrad_s);

    if ((state != NULL) && state->valid) {
        previous_linear_mm_s = state->feasible_linear_mm_s;
        previous_angular_mrad_s = state->feasible_angular_mrad_s;
        dt_ms = (uint32_t)(now_ms - state->timestamp_ms);
        if (dt_ms > maximum_dt_ms) {
            dt_ms = maximum_dt_ms;
        }
    }
    proposal->dt_ms = dt_ms;
    linear_step = fabsf(maximum_linear_accel_mm_s2) *
                  ((float)dt_ms / 1000.0f);
    angular_step = fabsf(maximum_angular_accel_mrad_s2) *
                   ((float)dt_ms / 1000.0f);
    candidate_linear_mm_s = Motion_ClampFloat(
        proposal->hard_feasible_linear_mm_s,
        previous_linear_mm_s - linear_step,
        previous_linear_mm_s + linear_step);
    candidate_angular_mrad_s = Motion_ClampFloat(
        proposal->hard_feasible_angular_mrad_s,
        previous_angular_mrad_s - angular_step,
        previous_angular_mrad_s + angular_step);

    previous_yaw_wheel_mm_s = previous_angular_mrad_s / 1000.0f *
                             WHEEL_BASE_MM * 0.5f;
    candidate_yaw_wheel_mm_s = candidate_angular_mrad_s / 1000.0f *
                              WHEEL_BASE_MM * 0.5f;
    start_left_mm_s = previous_linear_mm_s - previous_yaw_wheel_mm_s;
    start_right_mm_s = previous_linear_mm_s + previous_yaw_wheel_mm_s;
    candidate_left_mm_s = candidate_linear_mm_s - candidate_yaw_wheel_mm_s;
    candidate_right_mm_s = candidate_linear_mm_s + candidate_yaw_wheel_mm_s;
    scale = Motion_JointWheelFeasibilityScale(start_left_mm_s,
                                              start_right_mm_s,
                                              candidate_left_mm_s,
                                              candidate_right_mm_s,
                                              hard_wheel_mm_s);
    proposal->feasible_linear_mm_s = previous_linear_mm_s +
        ((candidate_linear_mm_s - previous_linear_mm_s) * scale);
    proposal->feasible_angular_mrad_s = previous_angular_mrad_s +
        ((candidate_angular_mrad_s - previous_angular_mrad_s) * scale);
    proposal->joint_scale = scale;
    proposal->rate_limited =
        (proposal->feasible_linear_mm_s !=
         proposal->hard_feasible_linear_mm_s) ||
        (proposal->feasible_angular_mrad_s !=
         proposal->hard_feasible_angular_mrad_s);

    Motion_CmdVelToMotorTargets(proposal->feasible_linear_mm_s,
                                proposal->feasible_angular_mrad_s,
                                &final_targets);
    proposal->left_motor_rpm = final_targets.left_motor_rpm;
    proposal->right_motor_rpm = final_targets.right_motor_rpm;
    proposal->left_velocity_dec = final_targets.left_velocity_dec;
    proposal->right_velocity_dec = final_targets.right_velocity_dec;
}

void Motion_ActuatorCommit(MotionActuatorState_t *state,
                           const MotionActuatorProposal_t *proposal,
                           uint32_t timestamp_ms)
{
    if ((state == NULL) || (proposal == NULL)) {
        return;
    }
    state->valid = true;
    state->left_motor_rpm = proposal->left_motor_rpm;
    state->right_motor_rpm = proposal->right_motor_rpm;
    state->left_velocity_dec = proposal->left_velocity_dec;
    state->right_velocity_dec = proposal->right_velocity_dec;
    state->feasible_linear_mm_s = proposal->feasible_linear_mm_s;
    state->feasible_angular_mrad_s = proposal->feasible_angular_mrad_s;
    state->timestamp_ms = timestamp_ms;
    Motion_IncrementU32(&state->submit_count);
}

void Motion_ActuatorRejectBusy(MotionActuatorState_t *state)
{
    if (state != NULL) {
        Motion_IncrementU32(&state->busy_reject_count);
    }
}

void Motion_FeedbackInit(MotionFeedbackEstimator_t *estimator)
{
    if (estimator != NULL) {
        memset(estimator, 0, sizeof(*estimator));
    }
}

void Motion_FeedbackInvalidate(MotionFeedbackEstimator_t *estimator)
{
    if (estimator == NULL) {
        return;
    }

    estimator->left.valid = false;
    estimator->left.stale = true;
    estimator->right.valid = false;
    estimator->right.stale = true;
    estimator->paired_left.valid = false;
    estimator->paired_right.valid = false;
    estimator->pair_valid = false;
}

void Motion_FeedbackUpdateRawVelocity(MotionFeedbackEstimator_t *estimator,
                                      uint8_t node_id,
                                      int32_t raw_velocity_dec,
                                      uint32_t timestamp_ms,
                                      uint32_t sequence)
{
    MotionWheelFeedback_t *wheel;

    if (estimator == NULL) {
        return;
    }

    if (node_id == LEFT_NODE_ID) {
        wheel = &estimator->left;
    } else if (node_id == RIGHT_NODE_ID) {
        wheel = &estimator->right;
    } else {
        return;
    }

    wheel->raw_velocity_dec = raw_velocity_dec;
    wheel->logical_motor_rpm =
        Motion_RawVelocityDecToLogicalMotorRPM(node_id, raw_velocity_dec);
    wheel->wheel_velocity_mm_s =
        Motion_LogicalMotorRPMToWheelVelocityMmS(wheel->logical_motor_rpm);
    wheel->timestamp_ms = timestamp_ms;
    wheel->sequence = sequence;
    wheel->valid = true;
    wheel->stale = false;
}

MotionChassisFeedback_t Motion_FeedbackEvaluate(MotionFeedbackEstimator_t *estimator,
                                                uint32_t now_ms,
                                                uint32_t stale_limit_ms,
                                                uint32_t pair_skew_limit_ms)
{
    MotionChassisFeedback_t result;
    bool left_stale;
    bool right_stale;
    bool both_wheels_advanced;
    bool candidate_changed;
    bool candidate_coherent;
    uint32_t candidate_skew_ms;

    memset(&result, 0, sizeof(result));
    result.source = MOTION_FEEDBACK_SOURCE_WHEEL_ENCODER;
    result.stale = true;
    if (estimator == NULL) {
        return result;
    }

    left_stale = !estimator->left.valid ||
                 ((uint32_t)(now_ms - estimator->left.timestamp_ms) > stale_limit_ms);
    right_stale = !estimator->right.valid ||
                  ((uint32_t)(now_ms - estimator->right.timestamp_ms) > stale_limit_ms);
    estimator->left.stale = left_stale;
    estimator->right.stale = right_stale;

    both_wheels_advanced =
        (estimator->left.sequence != estimator->paired_left_sequence) &&
        (estimator->right.sequence != estimator->paired_right_sequence);
    candidate_changed =
        (estimator->left.sequence != estimator->evaluated_left_sequence) ||
        (estimator->right.sequence != estimator->evaluated_right_sequence);
    candidate_skew_ms = Motion_TimestampSkewMs(estimator->left.timestamp_ms,
                                               estimator->right.timestamp_ms);
    candidate_coherent = estimator->left.valid && estimator->right.valid &&
        !left_stale && !right_stale &&
        (candidate_skew_ms <= pair_skew_limit_ms);

    if (both_wheels_advanced && candidate_coherent) {
        estimator->paired_left = estimator->left;
        estimator->paired_right = estimator->right;
        estimator->paired_left_sequence = estimator->left.sequence;
        estimator->paired_right_sequence = estimator->right.sequence;
        estimator->chassis_sequence++;
        if (estimator->chassis_sequence == 1U) {
            estimator->first_pair_timestamp_ms = Motion_LaterTimestamp(
                estimator->left.timestamp_ms, estimator->right.timestamp_ms);
        }
        estimator->last_pair_timestamp_ms = Motion_LaterTimestamp(
            estimator->left.timestamp_ms, estimator->right.timestamp_ms);
        estimator->pair_valid = true;
        result.new_sample = true;
    } else if (both_wheels_advanced && candidate_changed) {
        if (left_stale || right_stale || !estimator->left.valid ||
            !estimator->right.valid) {
            Motion_IncrementU32(&estimator->rejected_stale_pair_count);
        } else if (candidate_skew_ms > pair_skew_limit_ms) {
            Motion_IncrementU32(&estimator->rejected_skew_pair_count);
        }
    }
    if (candidate_changed) {
        estimator->evaluated_left_sequence = estimator->left.sequence;
        estimator->evaluated_right_sequence = estimator->right.sequence;
    }

    if (!estimator->pair_valid) {
        return result;
    }

    result.left = estimator->paired_left;
    result.right = estimator->paired_right;
    result.timestamp_ms = Motion_LaterTimestamp(result.left.timestamp_ms,
                                                result.right.timestamp_ms);
    result.pair_skew_ms = Motion_TimestampSkewMs(result.left.timestamp_ms,
                                                result.right.timestamp_ms);
    result.sequence = estimator->chassis_sequence;
    result.stale =
        ((uint32_t)(now_ms - result.left.timestamp_ms) > stale_limit_ms) ||
        ((uint32_t)(now_ms - result.right.timestamp_ms) > stale_limit_ms);
    result.valid = !result.stale &&
                   (result.pair_skew_ms <= pair_skew_limit_ms);
    result.angular_velocity_mrad_s = Motion_WheelVelocitiesToAngularMradS(
        result.left.wheel_velocity_mm_s, result.right.wheel_velocity_mm_s);
    if (!result.valid) {
        result.new_sample = false;
    }
    return result;
}

void Motion_YawRatePIInit(MotionYawRatePI_t *controller)
{
    if (controller != NULL) {
        memset(controller, 0, sizeof(*controller));
        controller->dt_min_ms = UINT32_MAX;
    }
}

void Motion_YawRatePIReset(MotionYawRatePI_t *controller)
{
    bool had_state;
    uint32_t update_count;
    uint32_t duplicate_count;
    uint32_t short_dt_reject_count;
    uint32_t long_gap_reset_count;
    uint32_t reset_count;
    uint32_t first_update_timestamp_ms;
    uint32_t dt_min_ms;
    uint32_t dt_max_ms;
    float dt_average_ms;
    bool enabled;

    if (controller == NULL) {
        return;
    }

    had_state = controller->active || controller->feedback_sequence_valid ||
                (controller->consecutive_valid_updates != 0U) ||
                (controller->integral_mrad != 0.0f) ||
                (controller->requested_correction_mrad_s != 0.0f) ||
                (controller->submitted_correction_mrad_s != 0.0f);
    update_count = controller->update_count;
    duplicate_count = controller->duplicate_sample_count;
    short_dt_reject_count = controller->short_dt_reject_count;
    long_gap_reset_count = controller->long_gap_reset_count;
    reset_count = controller->reset_count;
    first_update_timestamp_ms = controller->first_update_timestamp_ms;
    dt_min_ms = controller->dt_min_ms;
    dt_max_ms = controller->dt_max_ms;
    dt_average_ms = controller->dt_average_ms;
    enabled = controller->enabled;
    memset(controller, 0, sizeof(*controller));
    controller->enabled = enabled;
    controller->update_count = update_count;
    controller->duplicate_sample_count = duplicate_count;
    controller->short_dt_reject_count = short_dt_reject_count;
    controller->long_gap_reset_count = long_gap_reset_count;
    controller->reset_count = reset_count;
    controller->first_update_timestamp_ms = first_update_timestamp_ms;
    controller->dt_min_ms = dt_min_ms;
    controller->dt_max_ms = dt_max_ms;
    controller->dt_average_ms = dt_average_ms;
    if (had_state) {
        Motion_IncrementU32(&controller->reset_count);
    }
}

static MotionYawControlMode_t Motion_SelectYawControlMode(
    const MotionYawRatePI_t *controller,
    const MotionYawRatePIConfig_t *config,
    float target_angular_mrad_s)
{
    if ((controller->mode == MOTION_YAW_CONTROL_ZERO_STABILIZE) &&
        (fabsf(target_angular_mrad_s) <
         config->zero_target_exit_threshold_mrad_s)) {
        return MOTION_YAW_CONTROL_ZERO_STABILIZE;
    }
    if (fabsf(target_angular_mrad_s) <=
        config->zero_target_threshold_mrad_s) {
        return MOTION_YAW_CONTROL_ZERO_STABILIZE;
    }
    return MOTION_YAW_CONTROL_NORMAL_PI;
}

MotionYawRatePIProposal_t Motion_YawRatePIPrepare(
    const MotionYawRatePI_t *controller,
    const MotionYawRatePIConfig_t *config,
    float target_linear_mm_s,
    float target_angular_mrad_s,
    const MotionChassisFeedback_t *feedback,
    bool enable)
{
    MotionYawRatePIProposal_t proposal;
    uint32_t raw_dt_ms;
    float candidate_integral;
    float candidate_raw;
    float correction_limit;
    bool pushes_correction_limit;
    bool stopped;
    bool mode_changed;

    memset(&proposal, 0, sizeof(proposal));
    proposal.status = MOTION_YAW_PI_PROPOSAL_INVALID;
    if ((controller == NULL) || (config == NULL) || (feedback == NULL)) {
        return proposal;
    }
    if (!enable || !feedback->valid || feedback->stale) {
        return proposal;
    }

    proposal.mode = Motion_SelectYawControlMode(controller, config,
                                                target_angular_mrad_s);
    proposal.feedback_sequence = feedback->sequence;
    proposal.feedback_timestamp_ms = feedback->timestamp_ms;
    if (controller->feedback_sequence_valid &&
        (feedback->sequence == controller->last_feedback_sequence)) {
        proposal.status = MOTION_YAW_PI_PROPOSAL_DUPLICATE;
        return proposal;
    }

    if (!controller->feedback_sequence_valid) {
        proposal.status = MOTION_YAW_PI_PROPOSAL_PRIME;
        proposal.valid = true;
        return proposal;
    }

    raw_dt_ms = (uint32_t)(feedback->timestamp_ms -
                           controller->last_accepted_feedback_timestamp_ms);
    proposal.dt_ms = raw_dt_ms;
    proposal.dt_s = (float)raw_dt_ms / 1000.0f;
    if (raw_dt_ms < config->minimum_feedback_dt_ms) {
        proposal.status = MOTION_YAW_PI_PROPOSAL_SHORT_DT;
        return proposal;
    }
    if (raw_dt_ms > config->maximum_feedback_dt_ms) {
        proposal.status = MOTION_YAW_PI_PROPOSAL_LONG_GAP_RESET;
        proposal.valid = true;
        return proposal;
    }

    mode_changed = (controller->mode != proposal.mode);
    if (mode_changed) {
        proposal.consecutive_valid_updates = 0U;
        proposal.candidate_integral_mrad = 0.0f;
    } else {
        proposal.consecutive_valid_updates =
            controller->consecutive_valid_updates;
        proposal.candidate_integral_mrad = controller->integral_mrad;
    }

    proposal.error_mrad_s = target_angular_mrad_s -
                            feedback->angular_velocity_mrad_s;
    if (proposal.consecutive_valid_updates < config->reentry_valid_updates) {
        Motion_IncrementU8(&proposal.consecutive_valid_updates);
        proposal.status = MOTION_YAW_PI_PROPOSAL_REENTRY;
        proposal.valid = true;
        return proposal;
    }

    proposal.status = MOTION_YAW_PI_PROPOSAL_READY;
    proposal.valid = true;
    proposal.active = true;
    if (proposal.mode == MOTION_YAW_CONTROL_ZERO_STABILIZE) {
        stopped =
            (fabsf(target_angular_mrad_s) <=
             config->stopped_target_threshold_mrad_s) &&
            (fabsf(target_linear_mm_s) <=
             config->stopped_linear_threshold_mm_s) &&
            (fabsf(feedback->left.wheel_velocity_mm_s) <=
             config->stopped_wheel_threshold_mm_s) &&
            (fabsf(feedback->right.wheel_velocity_mm_s) <=
             config->stopped_wheel_threshold_mm_s);
        proposal.candidate_integral_mrad = 0.0f;
        if (stopped ||
            (fabsf(proposal.error_mrad_s) <=
             config->zero_yaw_feedback_deadband_mrad_s)) {
            proposal.p_term_mrad_s = 0.0f;
            proposal.raw_correction_mrad_s = 0.0f;
            proposal.requested_correction_mrad_s = 0.0f;
        } else {
            proposal.p_term_mrad_s =
                config->zero_yaw_kp * proposal.error_mrad_s;
            proposal.raw_correction_mrad_s = proposal.p_term_mrad_s;
            proposal.requested_correction_mrad_s = Motion_ClampFloat(
                proposal.raw_correction_mrad_s,
                -config->zero_yaw_correction_limit_mrad_s,
                config->zero_yaw_correction_limit_mrad_s);
            proposal.correction_saturated =
                (proposal.requested_correction_mrad_s !=
                 proposal.raw_correction_mrad_s);
        }
        return proposal;
    }

    candidate_integral = Motion_ClampFloat(
        proposal.candidate_integral_mrad +
            (proposal.error_mrad_s * proposal.dt_s),
        -config->integral_limit_mrad,
        config->integral_limit_mrad);
    candidate_raw = (config->kp * proposal.error_mrad_s) +
                    (config->ki * candidate_integral);
    correction_limit = fabsf(config->correction_limit_mrad_s);
    pushes_correction_limit =
        ((candidate_raw > correction_limit) &&
         (proposal.error_mrad_s > 0.0f)) ||
        ((candidate_raw < -correction_limit) &&
         (proposal.error_mrad_s < 0.0f));
    if (!pushes_correction_limit) {
        proposal.candidate_integral_mrad = candidate_integral;
    }
    proposal.p_term_mrad_s = config->kp * proposal.error_mrad_s;
    proposal.raw_correction_mrad_s = proposal.p_term_mrad_s +
        (config->ki * proposal.candidate_integral_mrad);
    proposal.requested_correction_mrad_s = Motion_ClampFloat(
        proposal.raw_correction_mrad_s, -correction_limit, correction_limit);
    proposal.correction_saturated =
        (proposal.requested_correction_mrad_s !=
         proposal.raw_correction_mrad_s);
    return proposal;
}

void Motion_YawRatePICommit(
    MotionYawRatePI_t *controller,
    const MotionYawRatePIConfig_t *config,
    const MotionYawRatePIProposal_t *proposal,
    float submitted_correction_mrad_s,
    bool wheel_saturated)
{
    float tracking_integral_change;

    if ((controller == NULL) || (config == NULL) || (proposal == NULL) ||
        !proposal->valid) {
        return;
    }
    controller->enabled = true;
    controller->feedback_sequence_valid = true;
    controller->last_feedback_sequence = proposal->feedback_sequence;
    controller->last_accepted_feedback_timestamp_ms =
        proposal->feedback_timestamp_ms;
    controller->mode = proposal->mode;
    controller->active = proposal->active;
    controller->consecutive_valid_updates =
        proposal->consecutive_valid_updates;
    controller->dt_s = proposal->dt_s;
    controller->integral_mrad = proposal->candidate_integral_mrad;
    controller->error_mrad_s = proposal->error_mrad_s;
    controller->p_term_mrad_s = proposal->p_term_mrad_s;
    controller->raw_correction_mrad_s = proposal->raw_correction_mrad_s;
    controller->requested_correction_mrad_s =
        proposal->requested_correction_mrad_s;
    controller->submitted_correction_mrad_s = submitted_correction_mrad_s;
    controller->correction_saturated = proposal->correction_saturated;
    controller->wheel_saturated = wheel_saturated;

    if (proposal->status == MOTION_YAW_PI_PROPOSAL_LONG_GAP_RESET) {
        Motion_IncrementU32(&controller->long_gap_reset_count);
    }
    if (proposal->dt_ms > 0U) {
        Motion_IncrementU32(&controller->update_count);
        if (controller->update_count == 1U) {
            controller->first_update_timestamp_ms =
                proposal->feedback_timestamp_ms;
            controller->dt_min_ms = proposal->dt_ms;
            controller->dt_max_ms = proposal->dt_ms;
            controller->dt_average_ms = (float)proposal->dt_ms;
        } else {
            if (proposal->dt_ms < controller->dt_min_ms) {
                controller->dt_min_ms = proposal->dt_ms;
            }
            if (proposal->dt_ms > controller->dt_max_ms) {
                controller->dt_max_ms = proposal->dt_ms;
            }
            controller->dt_average_ms +=
                ((float)proposal->dt_ms - controller->dt_average_ms) /
                (float)controller->update_count;
        }
    }

    if (proposal->active &&
        (proposal->mode == MOTION_YAW_CONTROL_NORMAL_PI) &&
        (config->ki != 0.0f)) {
        /* The same proposal dt/raw correction and the correction from the
         * successfully enqueued final body target form one anti-windup event. */
        tracking_integral_change =
            (config->tracking_gain / config->ki) *
            (submitted_correction_mrad_s -
             proposal->raw_correction_mrad_s) * proposal->dt_s;
        controller->integral_mrad = Motion_ClampFloat(
            controller->integral_mrad + tracking_integral_change,
            -config->integral_limit_mrad,
            config->integral_limit_mrad);
    }
}

void Motion_YawRatePIRecordDiscard(
    MotionYawRatePI_t *controller,
    MotionYawRatePIProposalStatus_t status)
{
    if (controller == NULL) {
        return;
    }
    if (status == MOTION_YAW_PI_PROPOSAL_DUPLICATE) {
        Motion_IncrementU32(&controller->duplicate_sample_count);
    } else if (status == MOTION_YAW_PI_PROPOSAL_SHORT_DT) {
        Motion_IncrementU32(&controller->short_dt_reject_count);
    }
}

const char *Motion_YawControlModeName(MotionYawControlMode_t mode)
{
    switch (mode) {
        case MOTION_YAW_CONTROL_NORMAL_PI: return "NORMAL_PI";
        case MOTION_YAW_CONTROL_ZERO_STABILIZE: return "ZERO_P";
        case MOTION_YAW_CONTROL_OFF:
        default: return "OFF";
    }
}

void Motion_TrackingSupervisorInit(MotionTrackingSupervisor_t *supervisor)
{
    if (supervisor != NULL) {
        memset(supervisor, 0, sizeof(*supervisor));
        supervisor->state = MOTION_TRACKING_OPEN_LOOP;
    }
}

void Motion_TrackingSupervisorReset(MotionTrackingSupervisor_t *supervisor,
                                    bool preserve_history)
{
    MotionTrackingReason_t last_stop_reason;
    uint32_t degraded_event_count;
    uint32_t stop_event_count;
    uint32_t large_error_observation_count;
    uint32_t wrong_sign_observation_count;
    uint32_t divergence_observation_count;
    uint32_t wrong_sign_event_count;
    uint32_t divergence_event_count;
    uint32_t invalid_feedback_observation_count;
    uint32_t stale_feedback_observation_count;
    uint32_t skew_feedback_observation_count;
    uint32_t correction_limit_observation_count;
    uint32_t nominal_headroom_observation_count;
    uint32_t hard_limit_observation_count;
    uint32_t rate_limit_observation_count;
    uint32_t transport_busy_observation_count;

    if (supervisor == NULL) {
        return;
    }
    last_stop_reason = supervisor->last_stop_reason;
    degraded_event_count = supervisor->degraded_event_count;
    stop_event_count = supervisor->stop_event_count;
    large_error_observation_count =
        supervisor->large_error_observation_count;
    wrong_sign_observation_count =
        supervisor->wrong_sign_observation_count;
    divergence_observation_count =
        supervisor->divergence_observation_count;
    wrong_sign_event_count = supervisor->wrong_sign_event_count;
    divergence_event_count = supervisor->divergence_event_count;
    invalid_feedback_observation_count =
        supervisor->invalid_feedback_observation_count;
    stale_feedback_observation_count =
        supervisor->stale_feedback_observation_count;
    skew_feedback_observation_count =
        supervisor->skew_feedback_observation_count;
    correction_limit_observation_count =
        supervisor->correction_limit_observation_count;
    nominal_headroom_observation_count =
        supervisor->nominal_headroom_observation_count;
    hard_limit_observation_count = supervisor->hard_limit_observation_count;
    rate_limit_observation_count = supervisor->rate_limit_observation_count;
    transport_busy_observation_count =
        supervisor->transport_busy_observation_count;
    memset(supervisor, 0, sizeof(*supervisor));
    supervisor->state = MOTION_TRACKING_OPEN_LOOP;
    if (preserve_history) {
        supervisor->last_stop_reason = last_stop_reason;
        supervisor->degraded_event_count = degraded_event_count;
        supervisor->stop_event_count = stop_event_count;
        supervisor->large_error_observation_count =
            large_error_observation_count;
        supervisor->wrong_sign_observation_count =
            wrong_sign_observation_count;
        supervisor->divergence_observation_count =
            divergence_observation_count;
        supervisor->wrong_sign_event_count = wrong_sign_event_count;
        supervisor->divergence_event_count = divergence_event_count;
        supervisor->invalid_feedback_observation_count =
            invalid_feedback_observation_count;
        supervisor->stale_feedback_observation_count =
            stale_feedback_observation_count;
        supervisor->skew_feedback_observation_count =
            skew_feedback_observation_count;
        supervisor->correction_limit_observation_count =
            correction_limit_observation_count;
        supervisor->nominal_headroom_observation_count =
            nominal_headroom_observation_count;
        supervisor->hard_limit_observation_count =
            hard_limit_observation_count;
        supervisor->rate_limit_observation_count =
            rate_limit_observation_count;
        supervisor->transport_busy_observation_count =
            transport_busy_observation_count;
    }
}

void Motion_TrackingSupervisorObserveFeedbackIssue(
    MotionTrackingSupervisor_t *supervisor,
    bool stale,
    bool skew)
{
    if (supervisor == NULL) {
        return;
    }
    Motion_TrackingSupervisorReset(supervisor, true);
    Motion_IncrementU32(&supervisor->invalid_feedback_observation_count);
    if (stale) {
        Motion_IncrementU32(&supervisor->stale_feedback_observation_count);
    }
    if (skew) {
        Motion_IncrementU32(&supervisor->skew_feedback_observation_count);
    }
}

static MotionTrackingState_t Motion_TrackingRequestStop(
    MotionTrackingSupervisor_t *supervisor,
    MotionTrackingReason_t reason)
{
    supervisor->state = MOTION_TRACKING_STOP_REQUESTED;
    supervisor->reason = reason;
    supervisor->last_stop_reason = reason;
    Motion_IncrementU32(&supervisor->stop_event_count);
    if (reason == MOTION_TRACKING_REASON_WRONG_SIGN) {
        Motion_IncrementU32(&supervisor->wrong_sign_event_count);
    } else if (reason == MOTION_TRACKING_REASON_DIVERGING) {
        Motion_IncrementU32(&supervisor->divergence_event_count);
    }
    return supervisor->state;
}

MotionTrackingState_t Motion_TrackingSupervisorUpdate(
    MotionTrackingSupervisor_t *supervisor,
    const MotionTrackingConfig_t *config,
    float command_target_mrad_s,
    float submitted_target_mrad_s,
    const MotionChassisFeedback_t *feedback,
    MotionYawControlMode_t control_mode,
    const MotionTrackingLimits_t *limits)
{
    float abs_error;
    bool command_step;
    bool submitted_target_step;
    bool large_error;
    bool wrong_sign;
    bool diverging;
    bool persistent_error;
    bool persistent_saturation;
    bool authority_limited;
    bool mode_changed;

    if ((supervisor == NULL) || (config == NULL) || (feedback == NULL) ||
        (limits == NULL)) {
        return MOTION_TRACKING_OPEN_LOOP;
    }

    if (!feedback->valid || feedback->stale ||
        (fabsf(submitted_target_mrad_s) <
         config->active_target_threshold_mrad_s)) {
        Motion_TrackingSupervisorReset(supervisor, true);
        supervisor->last_command_target_mrad_s = command_target_mrad_s;
        return supervisor->state;
    }
    /* new_sample is an evaluator-call event and may be false by the later
     * actuator gate. The supervisor owns an independent sequence cursor so a
     * coherent pair is consumed exactly once regardless of loop scheduling. */
    if (supervisor->feedback_sequence_valid &&
        (feedback->sequence == supervisor->last_feedback_sequence)) {
        return supervisor->state;
    }
    supervisor->last_feedback_sequence = feedback->sequence;
    supervisor->feedback_sequence_valid = true;

    if (limits->correction_saturated) {
        Motion_IncrementU32(&supervisor->correction_limit_observation_count);
    }
    if (limits->nominal_headroom_exhausted) {
        Motion_IncrementU32(&supervisor->nominal_headroom_observation_count);
    }
    if (limits->hard_wheel_saturated) {
        Motion_IncrementU32(&supervisor->hard_limit_observation_count);
    }
    if (limits->final_rate_limited) {
        Motion_IncrementU32(&supervisor->rate_limit_observation_count);
    }
    if (limits->transport_busy) {
        Motion_IncrementU32(&supervisor->transport_busy_observation_count);
    }
    authority_limited = limits->correction_saturated ||
        limits->nominal_headroom_exhausted ||
        limits->hard_wheel_saturated ||
        limits->final_rate_limited || limits->transport_busy;
    mode_changed = (supervisor->last_control_mode != control_mode);
    supervisor->last_control_mode = control_mode;

    command_step =
        (fabsf(command_target_mrad_s - supervisor->last_command_target_mrad_s) >=
         config->command_step_threshold_mrad_s);
    submitted_target_step =
        (fabsf(submitted_target_mrad_s -
               supervisor->last_submitted_target_mrad_s) >=
         config->submitted_target_step_threshold_mrad_s);
    supervisor->last_command_target_mrad_s = command_target_mrad_s;
    supervisor->last_submitted_target_mrad_s = submitted_target_mrad_s;
    if ((supervisor->state == MOTION_TRACKING_OPEN_LOOP) || command_step ||
        submitted_target_step || mode_changed) {
        supervisor->state = MOTION_TRACKING_GRACE;
        supervisor->reason = MOTION_TRACKING_REASON_NONE;
        supervisor->grace_start_ms = feedback->timestamp_ms;
        supervisor->large_error_start_ms = 0U;
        supervisor->saturation_start_ms = 0U;
        supervisor->large_error_consecutive = 0U;
        supervisor->wrong_sign_consecutive = 0U;
        supervisor->divergence_consecutive = 0U;
        supervisor->recovery_consecutive = 0U;
        supervisor->previous_abs_error_mrad_s = 0.0f;
    }

    supervisor->error_mrad_s = submitted_target_mrad_s -
                               feedback->angular_velocity_mrad_s;
    abs_error = fabsf(supervisor->error_mrad_s);
    if ((uint32_t)(feedback->timestamp_ms - supervisor->grace_start_ms) <
        config->grace_ms) {
        supervisor->previous_abs_error_mrad_s = abs_error;
        return supervisor->state;
    }
    if (supervisor->state == MOTION_TRACKING_GRACE) {
        supervisor->state = MOTION_TRACKING_OK;
    }

    large_error = abs_error >= config->large_error_threshold_mrad_s;
    wrong_sign =
        (fabsf(submitted_target_mrad_s) >=
         config->active_target_threshold_mrad_s) &&
        (fabsf(feedback->angular_velocity_mrad_s) >=
         config->wrong_sign_actual_threshold_mrad_s) &&
        ((submitted_target_mrad_s * feedback->angular_velocity_mrad_s) < 0.0f);
    diverging = large_error &&
        (abs_error >= (supervisor->previous_abs_error_mrad_s +
                       config->divergence_rise_threshold_mrad_s));
    supervisor->previous_abs_error_mrad_s = abs_error;

    if (large_error) {
        Motion_IncrementU32(&supervisor->large_error_observation_count);
        Motion_IncrementU8(&supervisor->large_error_consecutive);
        if (supervisor->large_error_start_ms == 0U) {
            supervisor->large_error_start_ms = feedback->timestamp_ms;
        }
    } else {
        supervisor->large_error_consecutive = 0U;
        supervisor->large_error_start_ms = 0U;
    }
    if (authority_limited) {
        if (supervisor->saturation_start_ms == 0U) {
            supervisor->saturation_start_ms = feedback->timestamp_ms;
        }
    } else {
        supervisor->saturation_start_ms = 0U;
    }
    if (wrong_sign) {
        Motion_IncrementU32(&supervisor->wrong_sign_observation_count);
        Motion_IncrementU8(&supervisor->wrong_sign_consecutive);
    } else {
        supervisor->wrong_sign_consecutive = 0U;
    }
    if (diverging) {
        Motion_IncrementU32(&supervisor->divergence_observation_count);
        Motion_IncrementU8(&supervisor->divergence_consecutive);
    } else {
        supervisor->divergence_consecutive = 0U;
    }

    if (supervisor->wrong_sign_consecutive >=
        config->wrong_sign_consecutive_limit) {
        return Motion_TrackingRequestStop(supervisor,
                                          MOTION_TRACKING_REASON_WRONG_SIGN);
    }
    if (supervisor->divergence_consecutive >=
        config->divergence_consecutive_limit) {
        return Motion_TrackingRequestStop(supervisor,
                                          MOTION_TRACKING_REASON_DIVERGING);
    }

    persistent_error =
        (supervisor->large_error_consecutive >=
         config->large_error_consecutive_limit) ||
        ((supervisor->large_error_start_ms != 0U) &&
         ((uint32_t)(feedback->timestamp_ms -
                     supervisor->large_error_start_ms) >=
          config->large_error_duration_ms));
    persistent_saturation =
        (supervisor->saturation_start_ms != 0U) &&
        ((uint32_t)(feedback->timestamp_ms - supervisor->saturation_start_ms) >=
         config->saturation_duration_ms);
    if (persistent_error || persistent_saturation) {
        if (supervisor->state != MOTION_TRACKING_DEGRADED) {
            Motion_IncrementU32(&supervisor->degraded_event_count);
        }
        supervisor->state = MOTION_TRACKING_DEGRADED;
        if (persistent_error && !persistent_saturation) {
            supervisor->reason = MOTION_TRACKING_REASON_LARGE_ERROR;
        } else if (limits->transport_busy) {
            supervisor->reason = MOTION_TRACKING_REASON_TRANSPORT_BUSY;
        } else if (limits->final_rate_limited) {
            supervisor->reason = MOTION_TRACKING_REASON_RATE_LIMIT;
        } else if (limits->hard_wheel_saturated) {
            supervisor->reason = MOTION_TRACKING_REASON_HARD_LIMIT;
        } else if (limits->nominal_headroom_exhausted) {
            supervisor->reason = MOTION_TRACKING_REASON_NOMINAL_HEADROOM;
        } else {
            supervisor->reason = MOTION_TRACKING_REASON_CORRECTION_LIMIT;
        }
        supervisor->recovery_consecutive = 0U;
    } else if (supervisor->state == MOTION_TRACKING_DEGRADED) {
        Motion_IncrementU8(&supervisor->recovery_consecutive);
        if (supervisor->recovery_consecutive >=
            config->recovery_consecutive_limit) {
            supervisor->state = MOTION_TRACKING_OK;
            supervisor->reason = MOTION_TRACKING_REASON_NONE;
            supervisor->recovery_consecutive = 0U;
        }
    } else {
        supervisor->state = MOTION_TRACKING_OK;
        supervisor->reason = MOTION_TRACKING_REASON_NONE;
    }
    return supervisor->state;
}

const char *Motion_TrackingStateName(MotionTrackingState_t state)
{
    switch (state) {
        case MOTION_TRACKING_GRACE: return "GRACE";
        case MOTION_TRACKING_OK: return "OK";
        case MOTION_TRACKING_DEGRADED: return "DEGRADED";
        case MOTION_TRACKING_STOP_REQUESTED: return "STOP";
        case MOTION_TRACKING_OPEN_LOOP:
        default: return "OPEN_LOOP";
    }
}

const char *Motion_TrackingReasonName(MotionTrackingReason_t reason)
{
    switch (reason) {
        case MOTION_TRACKING_REASON_LARGE_ERROR: return "LARGE_ERROR";
        case MOTION_TRACKING_REASON_CORRECTION_LIMIT: return "CORR_LIMIT";
        case MOTION_TRACKING_REASON_NOMINAL_HEADROOM: return "NO_HEADROOM";
        case MOTION_TRACKING_REASON_HARD_LIMIT: return "HARD_LIMIT";
        case MOTION_TRACKING_REASON_RATE_LIMIT: return "RATE_LIMIT";
        case MOTION_TRACKING_REASON_TRANSPORT_BUSY: return "TX_BUSY";
        case MOTION_TRACKING_REASON_WRONG_SIGN: return "WRONG_SIGN";
        case MOTION_TRACKING_REASON_DIVERGING: return "DIVERGING";
        case MOTION_TRACKING_REASON_NONE:
        default: return "NONE";
    }
}
