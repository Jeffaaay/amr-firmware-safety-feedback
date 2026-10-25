/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : motion_control.h
  * @brief          : Pure differential-drive feedback and yaw-rate control.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    MOTION_FEEDBACK_SOURCE_NONE = 0,
    MOTION_FEEDBACK_SOURCE_WHEEL_ENCODER
} MotionFeedbackSource_t;

typedef struct
{
    int32_t raw_velocity_dec;
    float logical_motor_rpm;
    float wheel_velocity_mm_s;
    uint32_t timestamp_ms;
    uint32_t sequence;
    bool valid;
    bool stale;
} MotionWheelFeedback_t;

typedef struct
{
    MotionWheelFeedback_t left;
    MotionWheelFeedback_t right;
    MotionFeedbackSource_t source;
    float angular_velocity_mrad_s;
    uint32_t timestamp_ms;
    uint32_t pair_skew_ms;
    uint32_t sequence;
    bool valid;
    bool stale;
    bool new_sample;
} MotionChassisFeedback_t;

typedef struct
{
    MotionWheelFeedback_t left;
    MotionWheelFeedback_t right;
    MotionWheelFeedback_t paired_left;
    MotionWheelFeedback_t paired_right;
    uint32_t paired_left_sequence;
    uint32_t paired_right_sequence;
    uint32_t evaluated_left_sequence;
    uint32_t evaluated_right_sequence;
    uint32_t chassis_sequence;
    uint32_t first_pair_timestamp_ms;
    uint32_t last_pair_timestamp_ms;
    uint32_t rejected_stale_pair_count;
    uint32_t rejected_skew_pair_count;
    bool pair_valid;
} MotionFeedbackEstimator_t;

typedef struct
{
    float hard_motor_rpm;
    float hard_wheel_mm_s;
    float reserved_yaw_correction_mrad_s;
    float reserved_wheel_mm_s;
    float nominal_wheel_mm_s;
    float hard_linear_mm_s;
    float hard_angular_mrad_s;
    float nominal_linear_mm_s;
    float nominal_angular_mrad_s;
} MotionCommandEnvelope_t;

typedef struct
{
    float requested_linear_mm_s;
    float requested_angular_mrad_s;
    float limited_linear_mm_s;
    float limited_angular_mrad_s;
    float allowed_linear_at_angular_mm_s;
    bool nominal_envelope_used;
    bool linear_limited;
    bool angular_limited;
} MotionBodyCommand_t;

typedef enum
{
    MOTION_YAW_CONTROL_OFF = 0,
    MOTION_YAW_CONTROL_NORMAL_PI,
    MOTION_YAW_CONTROL_ZERO_STABILIZE
} MotionYawControlMode_t;

typedef struct
{
    float kp;
    float ki;
    float tracking_gain;
    float integral_limit_mrad;
    float correction_limit_mrad_s;
    float zero_target_threshold_mrad_s;
    float zero_target_exit_threshold_mrad_s;
    float zero_yaw_kp;
    float zero_yaw_correction_limit_mrad_s;
    float zero_yaw_feedback_deadband_mrad_s;
    float stopped_target_threshold_mrad_s;
    float stopped_linear_threshold_mm_s;
    float stopped_wheel_threshold_mm_s;
    uint32_t minimum_feedback_dt_ms;
    uint32_t maximum_feedback_dt_ms;
    uint8_t reentry_valid_updates;
} MotionYawRatePIConfig_t;

typedef struct
{
    bool enabled;
    bool active;
    bool feedback_sequence_valid;
    bool correction_saturated;
    bool wheel_saturated;
    MotionYawControlMode_t mode;
    uint8_t consecutive_valid_updates;
    uint32_t last_feedback_sequence;
    uint32_t last_accepted_feedback_timestamp_ms;
    uint32_t first_update_timestamp_ms;
    uint32_t dt_min_ms;
    uint32_t dt_max_ms;
    float dt_s;
    float dt_average_ms;
    float integral_mrad;
    float error_mrad_s;
    float p_term_mrad_s;
    float raw_correction_mrad_s;
    float requested_correction_mrad_s;
    float submitted_correction_mrad_s;
    uint32_t update_count;
    uint32_t duplicate_sample_count;
    uint32_t short_dt_reject_count;
    uint32_t long_gap_reset_count;
    uint32_t reset_count;
} MotionYawRatePI_t;

typedef enum
{
    MOTION_YAW_PI_PROPOSAL_INVALID = 0,
    MOTION_YAW_PI_PROPOSAL_PRIME,
    MOTION_YAW_PI_PROPOSAL_REENTRY,
    MOTION_YAW_PI_PROPOSAL_READY,
    MOTION_YAW_PI_PROPOSAL_LONG_GAP_RESET,
    MOTION_YAW_PI_PROPOSAL_DUPLICATE,
    MOTION_YAW_PI_PROPOSAL_SHORT_DT
} MotionYawRatePIProposalStatus_t;

typedef struct
{
    MotionYawRatePIProposalStatus_t status;
    MotionYawControlMode_t mode;
    bool valid;
    bool active;
    bool correction_saturated;
    uint8_t consecutive_valid_updates;
    uint32_t feedback_sequence;
    uint32_t feedback_timestamp_ms;
    uint32_t dt_ms;
    float dt_s;
    float candidate_integral_mrad;
    float error_mrad_s;
    float p_term_mrad_s;
    float raw_correction_mrad_s;
    float requested_correction_mrad_s;
} MotionYawRatePIProposal_t;

typedef struct
{
    float prelimit_left_wheel_mm_s;
    float prelimit_right_wheel_mm_s;
    float left_wheel_mm_s;
    float right_wheel_mm_s;
    float prelimit_left_motor_rpm;
    float prelimit_right_motor_rpm;
    float left_motor_rpm;
    float right_motor_rpm;
    int32_t left_velocity_dec;
    int32_t right_velocity_dec;
    float feasible_linear_mm_s;
    float feasible_angular_mrad_s;
    float remaining_positive_yaw_authority_mrad_s;
    float remaining_negative_yaw_authority_mrad_s;
    float saturation_scale;
    bool saturated;
} MotionMotorTargets_t;

typedef struct
{
    bool valid;
    float left_motor_rpm;
    float right_motor_rpm;
    int32_t left_velocity_dec;
    int32_t right_velocity_dec;
    float feasible_linear_mm_s;
    float feasible_angular_mrad_s;
    uint32_t timestamp_ms;
    uint32_t submit_count;
    uint32_t busy_reject_count;
} MotionActuatorState_t;

typedef struct
{
    float desired_linear_mm_s;
    float desired_angular_mrad_s;
    float hard_feasible_linear_mm_s;
    float hard_feasible_angular_mrad_s;
    float left_motor_rpm;
    float right_motor_rpm;
    int32_t left_velocity_dec;
    int32_t right_velocity_dec;
    float feasible_linear_mm_s;
    float feasible_angular_mrad_s;
    float joint_scale;
    uint32_t dt_ms;
    bool rate_limited;
    bool hard_limited;
} MotionActuatorProposal_t;

typedef enum
{
    MOTION_TRACKING_OPEN_LOOP = 0,
    MOTION_TRACKING_GRACE,
    MOTION_TRACKING_OK,
    MOTION_TRACKING_DEGRADED,
    MOTION_TRACKING_STOP_REQUESTED
} MotionTrackingState_t;

typedef enum
{
    MOTION_TRACKING_REASON_NONE = 0,
    MOTION_TRACKING_REASON_LARGE_ERROR,
    MOTION_TRACKING_REASON_CORRECTION_LIMIT,
    MOTION_TRACKING_REASON_NOMINAL_HEADROOM,
    MOTION_TRACKING_REASON_HARD_LIMIT,
    MOTION_TRACKING_REASON_RATE_LIMIT,
    MOTION_TRACKING_REASON_TRANSPORT_BUSY,
    MOTION_TRACKING_REASON_WRONG_SIGN,
    MOTION_TRACKING_REASON_DIVERGING
} MotionTrackingReason_t;

typedef struct
{
    float command_step_threshold_mrad_s;
    float active_target_threshold_mrad_s;
    float large_error_threshold_mrad_s;
    float wrong_sign_actual_threshold_mrad_s;
    float divergence_rise_threshold_mrad_s;
    float submitted_target_step_threshold_mrad_s;
    uint32_t grace_ms;
    uint32_t large_error_duration_ms;
    uint32_t saturation_duration_ms;
    uint8_t large_error_consecutive_limit;
    uint8_t wrong_sign_consecutive_limit;
    uint8_t divergence_consecutive_limit;
    uint8_t recovery_consecutive_limit;
} MotionTrackingConfig_t;

typedef struct
{
    bool correction_saturated;
    bool nominal_headroom_exhausted;
    bool hard_wheel_saturated;
    bool final_rate_limited;
    bool transport_busy;
} MotionTrackingLimits_t;

typedef struct
{
    MotionTrackingState_t state;
    MotionTrackingReason_t reason;
    MotionTrackingReason_t last_stop_reason;
    MotionYawControlMode_t last_control_mode;
    float last_command_target_mrad_s;
    float last_submitted_target_mrad_s;
    float error_mrad_s;
    float previous_abs_error_mrad_s;
    uint32_t last_feedback_sequence;
    bool feedback_sequence_valid;
    uint32_t grace_start_ms;
    uint32_t large_error_start_ms;
    uint32_t saturation_start_ms;
    uint8_t large_error_consecutive;
    uint8_t wrong_sign_consecutive;
    uint8_t divergence_consecutive;
    uint8_t recovery_consecutive;
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
} MotionTrackingSupervisor_t;

int32_t Motion_RoundFloatToI32(float value);
int32_t Motion_ApplyMotorForwardSignI32(uint8_t node_id, int32_t raw_value);
int32_t Motion_RawMotorRPMToVelocityDec(int32_t raw_motor_rpm);
float Motion_RawVelocityDecToMotorRPM(int32_t raw_velocity_dec);
float Motion_RawVelocityDecToLogicalMotorRPM(uint8_t node_id,
                                             int32_t raw_velocity_dec);
float Motion_LogicalMotorRPMToWheelVelocityMmS(float logical_motor_rpm);
float Motion_MotorDeltaCountsToWheelDistanceMm(uint8_t node_id,
                                               int32_t raw_delta_counts);
float Motion_WheelVelocitiesToAngularMradS(float left_wheel_mm_s,
                                           float right_wheel_mm_s);

void Motion_CalculateCommandEnvelope(float hard_motor_rpm,
                                     float reserved_yaw_correction_mrad_s,
                                     MotionCommandEnvelope_t *envelope);
void Motion_LimitBodyCommandWithYawHeadroom(
    float linear_mm_s,
    float angular_mrad_s,
    const MotionCommandEnvelope_t *envelope,
    MotionBodyCommand_t *command);

void Motion_LimitBodyCommand(float linear_mm_s,
                             float angular_mrad_s,
                             float maximum_linear_mm_s,
                             float maximum_angular_mrad_s,
                             MotionBodyCommand_t *command);
void Motion_CmdVelToMotorTargets(float linear_mm_s,
                                 float angular_mrad_s,
                                 MotionMotorTargets_t *targets);
float Motion_SelectDirectionalYawAuthority(
    const MotionMotorTargets_t *targets,
    float correction_mrad_s);
void Motion_ActuatorStateInit(MotionActuatorState_t *state);
void Motion_ActuatorStateCommitZero(MotionActuatorState_t *state,
                                    uint32_t timestamp_ms);
void Motion_ActuatorStateInvalidate(MotionActuatorState_t *state);
void Motion_ActuatorPrepareBodyCommand(
    const MotionActuatorState_t *state,
    float desired_linear_mm_s,
    float desired_angular_mrad_s,
    uint32_t now_ms,
    uint32_t maximum_dt_ms,
    float maximum_linear_accel_mm_s2,
    float maximum_angular_accel_mrad_s2,
    float hard_motor_rpm,
    MotionActuatorProposal_t *proposal);
void Motion_ActuatorCommit(MotionActuatorState_t *state,
                           const MotionActuatorProposal_t *proposal,
                           uint32_t timestamp_ms);
void Motion_ActuatorRejectBusy(MotionActuatorState_t *state);

void Motion_FeedbackInit(MotionFeedbackEstimator_t *estimator);
void Motion_FeedbackInvalidate(MotionFeedbackEstimator_t *estimator);
void Motion_FeedbackUpdateRawVelocity(MotionFeedbackEstimator_t *estimator,
                                      uint8_t node_id,
                                      int32_t raw_velocity_dec,
                                      uint32_t timestamp_ms,
                                      uint32_t sequence);
MotionChassisFeedback_t Motion_FeedbackEvaluate(MotionFeedbackEstimator_t *estimator,
                                                uint32_t now_ms,
                                                uint32_t stale_limit_ms,
                                                uint32_t pair_skew_limit_ms);

void Motion_YawRatePIInit(MotionYawRatePI_t *controller);
void Motion_YawRatePIReset(MotionYawRatePI_t *controller);
MotionYawRatePIProposal_t Motion_YawRatePIPrepare(
    const MotionYawRatePI_t *controller,
    const MotionYawRatePIConfig_t *config,
    float target_linear_mm_s,
    float target_angular_mrad_s,
    const MotionChassisFeedback_t *feedback,
    bool enable);
void Motion_YawRatePICommit(
    MotionYawRatePI_t *controller,
    const MotionYawRatePIConfig_t *config,
    const MotionYawRatePIProposal_t *proposal,
    float submitted_correction_mrad_s,
    bool wheel_saturated);
void Motion_YawRatePIRecordDiscard(
    MotionYawRatePI_t *controller,
    MotionYawRatePIProposalStatus_t status);
const char *Motion_YawControlModeName(MotionYawControlMode_t mode);

void Motion_TrackingSupervisorInit(MotionTrackingSupervisor_t *supervisor);
void Motion_TrackingSupervisorReset(MotionTrackingSupervisor_t *supervisor,
                                    bool preserve_history);
void Motion_TrackingSupervisorObserveFeedbackIssue(
    MotionTrackingSupervisor_t *supervisor,
    bool stale,
    bool skew);
MotionTrackingState_t Motion_TrackingSupervisorUpdate(
    MotionTrackingSupervisor_t *supervisor,
    const MotionTrackingConfig_t *config,
    float command_target_mrad_s,
    float submitted_target_mrad_s,
    const MotionChassisFeedback_t *feedback,
    MotionYawControlMode_t control_mode,
    const MotionTrackingLimits_t *limits);
const char *Motion_TrackingStateName(MotionTrackingState_t state);
const char *Motion_TrackingReasonName(MotionTrackingReason_t reason);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_CONTROL_H */
