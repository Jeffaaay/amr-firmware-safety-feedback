#ifndef DRIVE_INTERNAL_H
#define DRIVE_INTERNAL_H

#include "app_config.h"
#include "canopen.h"
#include "drive.h"
#include "motion_control.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    DRIVE_ZERO_EVIDENCE_NONE = 0,
    DRIVE_ZERO_EVIDENCE_STOP,
    DRIVE_ZERO_EVIDENCE_DISABLE,
    DRIVE_ZERO_EVIDENCE_ENABLE
} DriveZeroEvidenceReason_t;

typedef struct
{
    bool valid;
    uint32_t generation;
    uint32_t left_sequence;
    uint32_t right_sequence;
    uint32_t left_timestamp_ms;
    uint32_t right_timestamp_ms;
    DriveZeroEvidenceReason_t reason;
} DriveZeroSpeedEvidence_t;

typedef enum
{
    DRIVE_STABLE_ZERO_IDLE = 0,
    DRIVE_STABLE_ZERO_SETTLING,
    DRIVE_STABLE_ZERO_CONFIRMING,
    DRIVE_STABLE_ZERO_CONFIRMED
} DriveStableZeroState_t;

typedef enum
{
    DRIVE_STABLE_ZERO_REJECT_NONE = 0,
    DRIVE_STABLE_ZERO_REJECT_PAIR_INVALID,
    DRIVE_STABLE_ZERO_REJECT_PAIR_NOT_NEW,
    DRIVE_STABLE_ZERO_REJECT_LEFT_SPEED,
    DRIVE_STABLE_ZERO_REJECT_RIGHT_SPEED,
    DRIVE_STABLE_ZERO_REJECT_PAIR_SKEW,
    DRIVE_STABLE_ZERO_REJECT_TIMEOUT
} DriveStableZeroRejectReason_t;

typedef enum
{
    DRIVE_STABLE_ZERO_PENDING = 0,
    DRIVE_STABLE_ZERO_COMPLETE,
    DRIVE_STABLE_ZERO_TIMED_OUT
} DriveStableZeroResult_t;

typedef struct
{
    bool active;
    bool settle_allowed;
    DriveStableZeroState_t state;
    DriveStableZeroRejectReason_t reject_reason;
    uint32_t start_ms;
    uint32_t settle_start_ms;
    uint32_t confirm_start_ms;
    uint32_t stable_start_ms;
    uint32_t baseline_pair_sequence;
    uint32_t baseline_left_sequence;
    uint32_t baseline_right_sequence;
    uint32_t last_pair_sequence;
    uint32_t last_left_sequence;
    uint32_t last_right_sequence;
    uint32_t last_skew_reject_count;
    float left_velocity_mm_s;
    float right_velocity_mm_s;
    uint32_t pair_age_ms;
    uint32_t pair_skew_ms;
    uint32_t stable_sample_count;
    uint32_t stable_dwell_ms;
    uint32_t settle_elapsed_ms;
    uint32_t overall_elapsed_ms;
} DriveStableZeroTracker_t;

typedef enum
{
    DRIVE_SAFETY_NONE = 0,
    DRIVE_SAFETY_STOP,
    DRIVE_SAFETY_DISABLE,
    DRIVE_SAFETY_ENABLE,
    DRIVE_SAFETY_ENABLE_ROLLBACK,
    DRIVE_SAFETY_DISABLE_ROLLBACK,
    DRIVE_SAFETY_INIT_DISABLE
} DriveSafetyTransitionType_t;

typedef enum
{
    DRIVE_SAFETY_PHASE_NONE = 0,
    DRIVE_SAFETY_PHASE_ZERO_START,
    DRIVE_SAFETY_PHASE_ZERO_WAIT_ACK,
    DRIVE_SAFETY_PHASE_ZERO_WAIT_SETTLE,
    DRIVE_SAFETY_PHASE_ZERO_WAIT_MEASURED,
    DRIVE_SAFETY_PHASE_ENABLE_READ_INITIAL_STATUS,
    DRIVE_SAFETY_PHASE_ENABLE_HB_WRITE_100C,
    DRIVE_SAFETY_PHASE_ENABLE_HB_READ_100C,
    DRIVE_SAFETY_PHASE_ENABLE_HB_WRITE_100D,
    DRIVE_SAFETY_PHASE_ENABLE_HB_READ_100D,
    DRIVE_SAFETY_PHASE_ENABLE_HB_WRITE_1017,
    DRIVE_SAFETY_PHASE_ENABLE_HB_READ_1017,
    DRIVE_SAFETY_PHASE_ENABLE_HB_WRITE_1016_01,
    DRIVE_SAFETY_PHASE_ENABLE_HB_READ_1016_01,
    DRIVE_SAFETY_PHASE_ENABLE_HB_WRITE_6007,
    DRIVE_SAFETY_PHASE_ENABLE_HB_READ_6007,
    DRIVE_SAFETY_PHASE_ENABLE_HB_QUALIFY,
    DRIVE_SAFETY_PHASE_ENABLE_SET_MODE,
    DRIVE_SAFETY_PHASE_ENABLE_READ_MODE_REQUEST,
    DRIVE_SAFETY_PHASE_ENABLE_WRITE_ZERO,
    DRIVE_SAFETY_PHASE_ENABLE_WRITE_0006,
    DRIVE_SAFETY_PHASE_ENABLE_READ_0006_STATUS,
    DRIVE_SAFETY_PHASE_ENABLE_WRITE_0007,
    DRIVE_SAFETY_PHASE_ENABLE_READ_0007_STATUS,
    DRIVE_SAFETY_PHASE_ENABLE_WRITE_000F,
    DRIVE_SAFETY_PHASE_ENABLE_READ_000F_STATUS,
    DRIVE_SAFETY_PHASE_ENABLE_WRITE_002F,
    DRIVE_SAFETY_PHASE_ENABLE_READ_002F_STATUS,
    DRIVE_SAFETY_PHASE_ENABLE_NODE_DONE,
    DRIVE_SAFETY_PHASE_ENABLE_FINAL_STATUS_LEFT,
    DRIVE_SAFETY_PHASE_ENABLE_FINAL_STATUS_RIGHT,
    DRIVE_SAFETY_PHASE_DISABLE_WRITE_LEFT,
    DRIVE_SAFETY_PHASE_DISABLE_WRITE_RIGHT,
    DRIVE_SAFETY_PHASE_DISABLE_STATUS_WAIT,
    DRIVE_SAFETY_PHASE_DISABLE_STATUS_READ_LEFT,
    DRIVE_SAFETY_PHASE_DISABLE_STATUS_READ_RIGHT
} DriveSafetyPhase_t;

typedef enum
{
    DRIVE_SAFETY_FAILURE_NONE = 0,
    DRIVE_SAFETY_FAILURE_ZERO_SUBMIT,
    DRIVE_SAFETY_FAILURE_ZERO_ACK,
    DRIVE_SAFETY_FAILURE_ZERO_FEEDBACK,
    DRIVE_SAFETY_FAILURE_SDO_TIMEOUT,
    DRIVE_SAFETY_FAILURE_SDO_ABORT,
    DRIVE_SAFETY_FAILURE_SDO_PROTOCOL,
    DRIVE_SAFETY_FAILURE_NODE_FAULT,
    DRIVE_SAFETY_FAILURE_STATUS_FAULT,
    DRIVE_SAFETY_FAILURE_HEARTBEAT,
    DRIVE_SAFETY_FAILURE_ENABLE_CONFIRM,
    DRIVE_SAFETY_FAILURE_DISABLE_CONFIRM,
    DRIVE_SAFETY_FAILURE_RUNTIME_CAN,
    DRIVE_SAFETY_FAILURE_TPDO_FEEDBACK,
    DRIVE_SAFETY_FAILURE_OVERALL_TIMEOUT
} DriveSafetyFailureReason_t;

typedef enum
{
    DRIVE_MODE_CONFIRM_SDO_NONE = 0,
    DRIVE_MODE_CONFIRM_SDO_SUCCESS,
    DRIVE_MODE_CONFIRM_SDO_TIMEOUT,
    DRIVE_MODE_CONFIRM_SDO_ABORT,
    DRIVE_MODE_CONFIRM_SDO_ERROR
} DriveModeConfirmSdoResult_t;

typedef struct
{
    bool active;
    DriveSafetyTransitionType_t type;
    DriveSafetyPhase_t phase;
    DriveSafetyFailureReason_t failure_reason;
    uint32_t start_ms;
    uint32_t phase_start_ms;
    uint32_t status_start_ms;
    uint32_t zero_start_ms;
    uint32_t next_action_ms;
    uint32_t generation;
    uint32_t left_start_sequence;
    uint32_t right_start_sequence;
    uint32_t left_emcy_sequence;
    uint32_t right_emcy_sequence;
    uint32_t action_attempts;
    uint32_t status_attempts;
    uint32_t transient_timeout_count;
    uint32_t last_abort_code;
    uint32_t last_sdo_rx_ms; /* Completion RX time, never the consumer time. */
    uint32_t mode_confirm_attempt;
    uint32_t mode_confirm_start_tick;
    uint32_t mode_confirm_next_poll_tick;
    uint32_t mode_confirm_elapsed_ms;
    uint32_t mode_confirm_last_abort_code;
    uint32_t zero_start_abort_count;
    uint16_t enable_0006_statusword;
    uint16_t enable_0007_statusword;
    uint16_t enable_000f_statusword;
    int8_t requested_mode;
    int8_t last_mode_readback;
    uint8_t enable_node_id;
    uint8_t mode_confirm_node_id;
    uint8_t pending_node_id;
    uint8_t failure_node_id;
    CanopenSdoOwner_t pending_owner;
    DriveModeConfirmSdoResult_t mode_confirm_last_sdo_result;
    bool enable_0006_statusword_valid;
    bool enable_0007_statusword_valid;
    bool enable_000f_statusword_valid;
    bool mode_readback_valid;
    bool request_pending;
    bool explicit_recovery;
    bool normal_controlled_zero;
    bool had_tracking_latch;
    bool had_disable_latch;
    bool was_software_enabled;
    bool was_hardware_disabled_confirmed;
} DriveSafetyTransition_t;

typedef enum
{
    DRIVE_FAULT_RESET_PHASE_NONE = 0,
    DRIVE_FAULT_RESET_PHASE_READ_STATUS,
    DRIVE_FAULT_RESET_PHASE_READ_ERROR_CODE,
    DRIVE_FAULT_RESET_PHASE_READ_ERROR_REGISTER,
    DRIVE_FAULT_RESET_PHASE_WRITE_0086,
    DRIVE_FAULT_RESET_PHASE_WAIT_RESET,
    DRIVE_FAULT_RESET_PHASE_WRITE_0006,
    DRIVE_FAULT_RESET_PHASE_READ_VERIFY,
    DRIVE_FAULT_RESET_PHASE_NODE_DONE,
    DRIVE_FAULT_RESET_PHASE_PRECHECK_ZERO_FEEDBACK
} DriveFaultResetPhase_t;

typedef enum
{
    DRIVE_FAULT_RESET_FAILURE_NONE = 0,
    DRIVE_FAULT_RESET_FAILURE_PRECONDITION,
    DRIVE_FAULT_RESET_FAILURE_OPERATION_ENABLED,
    DRIVE_FAULT_RESET_FAILURE_SDO_TIMEOUT,
    DRIVE_FAULT_RESET_FAILURE_SDO_ABORT,
    DRIVE_FAULT_RESET_FAILURE_SDO_PROTOCOL,
    DRIVE_FAULT_RESET_FAILURE_PERSISTENT_FAULT,
    DRIVE_FAULT_RESET_FAILURE_NODE_FAULT,
    DRIVE_FAULT_RESET_FAILURE_RUNTIME_CAN,
    DRIVE_FAULT_RESET_FAILURE_OVERALL_TIMEOUT,
    DRIVE_FAULT_RESET_FAILURE_ZERO_FEEDBACK_TIMEOUT,
    DRIVE_FAULT_RESET_FAILURE_ZERO_FEEDBACK_NONZERO
} DriveFaultResetFailure_t;

typedef enum
{
    DRIVE_FAULT_RESET_RESULT_IDLE = 0,
    DRIVE_FAULT_RESET_RESULT_PENDING,
    DRIVE_FAULT_RESET_RESULT_COMPLETE,
    DRIVE_FAULT_RESET_RESULT_FAILED
} DriveFaultResetResult_t;

typedef struct
{
    uint16_t statusword;
    bool statusword_valid;
    uint32_t statusword_timestamp_ms;
    uint32_t statusword_sequence;
    uint16_t error_code;
    bool error_code_valid;
    uint32_t error_code_timestamp_ms;
    uint32_t error_code_sequence;
    uint8_t error_register;
    bool error_register_valid;
    uint32_t error_register_timestamp_ms;
    uint32_t error_register_sequence;
    bool cleared;
    bool reset_written;
} DriveFaultResetNodeDiagnostic_t;

typedef struct
{
    bool active;
    DriveFaultResetPhase_t phase;
    DriveFaultResetFailure_t failure_reason;
    DriveFaultResetResult_t result;
    uint32_t start_ms;
    uint32_t phase_start_ms;
    uint32_t next_action_ms;
    uint32_t action_attempts;
    uint32_t transient_timeout_count;
    uint32_t last_abort_code;
    uint32_t left_emcy_sequence;
    uint32_t right_emcy_sequence;
    uint32_t zero_feedback_left_start_sequence;
    uint32_t zero_feedback_right_start_sequence;
    uint32_t zero_feedback_failure_count_start;
    uint32_t generation;
    uint32_t request_nonce;
    uint8_t node_id;
    uint8_t pending_node_id;
    uint8_t failure_node_id;
    bool request_pending;
    DriveFaultResetNodeDiagnostic_t node[3];
} DriveFaultReset_t;

typedef struct
{
    enum
    {
        DRIVE_LIFT_STOP_PHASE_NONE = 0,
        DRIVE_LIFT_STOP_PHASE_WAIT_SAFETY,
        DRIVE_LIFT_STOP_PHASE_ZERO_SETTLE,
        DRIVE_LIFT_STOP_PHASE_ZERO_CONFIRM,
        DRIVE_LIFT_STOP_PHASE_STATUS_LEFT,
        DRIVE_LIFT_STOP_PHASE_STATUS_RIGHT
    } phase;
    bool active;
    uint32_t generation;
    uint32_t start_ms;
    uint32_t phase_start_ms;
    /* Explicit-refresh command binding: per-command monotonic txn plus the
     * host request nonce, echoed on every DRIVE_BASE_SAFE_REFRESH line.
     * Both are 0 for the implicit lift confirmed-stop path. */
    uint32_t txn;
    uint32_t request_nonce;
    bool request_pending;
    bool explicit_base_safe_refresh;
} DriveConfirmedStopRequest_t;

typedef struct
{
    bool valid;
    uint32_t event_ms;
    DriveState_t state;
    DriveSafetyTransitionType_t transition_type;
    DriveSafetyPhase_t phase;
    DriveSafetyFailureReason_t reason;
    uint8_t node_id;
    uint16_t left_statusword;
    uint16_t right_statusword;
    bool left_statusword_valid;
    bool right_statusword_valid;
    uint32_t left_statusword_timestamp_ms;
    uint32_t right_statusword_timestamp_ms;
    uint32_t transition_abort_code;
    uint32_t transition_timeout_count;
    uint32_t mode_confirm_attempt;
    uint32_t mode_confirm_elapsed_ms;
    uint32_t mode_confirm_last_abort_code;
    uint16_t enable_0006_statusword;
    uint16_t enable_0007_statusword;
    uint16_t enable_000f_statusword;
    int8_t requested_mode;
    int8_t last_mode_readback;
    uint8_t mode_confirm_node_id;
    DriveModeConfirmSdoResult_t mode_confirm_last_sdo_result;
    bool enable_0006_statusword_valid;
    bool enable_0007_statusword_valid;
    bool enable_000f_statusword_valid;
    bool mode_readback_valid;
    bool hardware_enable_confirmed;
    bool hardware_disable_confirmed;
    bool motion_inhibit_latched;
    bool tracking_stop_latched;
    CanopenRuntimeDiagnostics_t canopen;
} DriveFaultSnapshot_t;

typedef struct
{
    bool enabled;
    DriveState_t state;
    bool (*lift_moving_callback)(void);
} DriveCoreState_t;

typedef struct
{
    uint32_t last_jetson_cmd_ms;
    bool jetson_cmd_active;
    int32_t last_left_cmd_rpm;
    int32_t last_right_cmd_rpm;
    int32_t last_left_cmd_dec;
    int32_t last_right_cmd_dec;
} DriveCommandState_t;

typedef struct
{
    DriveSafetyTransition_t transition;
} DriveSafetyContext_t;

typedef enum
{
    DRIVE_BOOT_FIRST_ATTEMPT = 0,
    DRIVE_BOOT_WAIT_RETRY,
    DRIVE_BOOT_SECOND_ATTEMPT,
    DRIVE_BOOT_READY,
    DRIVE_BOOT_FAILED
} DriveBootState_t;

typedef struct
{
    DriveBootState_t state;
    bool started;
    bool saw_timeout;
    bool saw_nondisabled_status;
    bool bus_off;
    uint8_t retries;
    uint8_t timeout_node;
    uint32_t retry_started_ms;
} DriveBootContext_t;

typedef struct
{
    DriveFaultReset_t transaction;
    /* Monotonic transaction counter echoed as gen= on every
     * DRIVE_FAULT_RESET_SAFE response line. Together with the echoed host
     * req= nonce it binds responses to one specific command; a stale line
     * still queued in the TX manager can never satisfy a new command. */
    uint32_t generation;
} DriveFaultResetContext_t;

typedef struct
{
    DriveConfirmedStopRequest_t request;
    /* Per-command transaction counter for explicit DRIVE_BASE_SAFE_REFRESH.
     * Distinct from the safety generation: two refreshes under one unchanged
     * safety generation still get different txn= values. */
    uint32_t base_safe_refresh_txn;
} DriveLiftStopContext_t;

typedef struct
{
    uint16_t left_statusword;
    uint16_t right_statusword;
    bool left_statusword_valid;
    bool right_statusword_valid;
    uint32_t left_statusword_timestamp_ms;
    uint32_t right_statusword_timestamp_ms;
    uint32_t left_statusword_sequence;
    uint32_t right_statusword_sequence;
    uint32_t left_statusword_generation;
    uint32_t right_statusword_generation;
} DriveStatuswordContext_t;

typedef struct
{
    MotionFeedbackEstimator_t motion;
    MotionChassisFeedback_t chassis;
    bool pair_pending;
    bool issue_active;
    uint32_t observed_stale_pair_reject_count;
    uint32_t observed_skew_pair_reject_count;
    int32_t status_left_rpm;
    int32_t status_right_rpm;
    uint32_t status_left_timestamp_ms;
    uint32_t status_right_timestamp_ms;
    bool status_left_seen;
    bool status_right_seen;
    bool status_left_valid;
    bool status_right_valid;
    uint32_t odom_left_position_sequence;
    uint32_t odom_right_position_sequence;
    uint32_t odom_authoritative_pair_sequence;
    uint32_t odom_authoritative_pair_timestamp_ms;
} DriveFeedbackContext_t;

typedef struct
{
    enum
    {
        DRIVE_TPDO_STARTUP_WAIT_SAFE = 0,
        DRIVE_TPDO_STARTUP_WAIT_FRESH,
        DRIVE_TPDO_STARTUP_ARMING,
        DRIVE_TPDO_STARTUP_COMPLETE,
        DRIVE_TPDO_STARTUP_FAILED
    } startup_state;
    uint32_t startup_qualify_start_ms;
    bool enable_command_active;
    bool enable_is_startup;
    uint32_t enable_request_nonce;
    uint32_t enable_start_ms;
    uint32_t odom_stale_suppressed_count;
    uint32_t stop_pair_not_new_count;
    uint32_t stop_pair_invalid_count;
    const char *last_stop_reject_reason;
} DriveTpdoActiveContext_t;

typedef struct
{
    DriveZeroSpeedEvidence_t zero_speed;
    uint32_t safety_generation;
    uint32_t post_disable_baseline_pair_sequence;
    uint32_t post_disable_baseline_left_sequence;
    uint32_t post_disable_baseline_right_sequence;
    bool post_disable_zero_proof_required;
} DriveEvidenceContext_t;

typedef struct
{
    MotionYawRatePI_t pi;
    MotionYawRatePIProposal_t proposal;
    bool waiting_for_new_pair;
    float corrected_angular_mrad_s;
    float submitted_correction_mrad_s;
    float proposed_submitted_correction_mrad_s;
    uint32_t feedback_pair_replaced_count;
    uint32_t prepare_count;
    uint32_t retarget_reset_count;
} DriveYawControlContext_t;

typedef struct
{
    MotionTrackingSupervisor_t supervisor;
    MotionTrackingLimits_t limits;
} DriveTrackingContext_t;

typedef struct
{
    MotionCommandEnvelope_t command_envelope;
    MotionBodyCommand_t body_command;
    MotionActuatorState_t state;
    MotionActuatorProposal_t baseline_proposal;
    MotionActuatorProposal_t final_proposal;
    MotionMotorTargets_t baseline_targets;
    MotionMotorTargets_t final_targets;
    bool initialized;
    uint32_t successful_submit_count;
} DriveActuatorContext_t;

typedef struct
{
    DriveYawControlContext_t yaw;
    DriveTrackingContext_t tracking;
    DriveActuatorContext_t actuator;
#if ENABLE_JETSON_UART_MODE
    float requested_linear_mm_s;
    float requested_angular_mrad_s;
    float target_linear_mm_s;
    float target_angular_mrad_s;
    float nominal_feasible_linear_mm_s;
    float nominal_feasible_angular_mrad_s;
    bool new_cmd_vel_pending;
#endif
} DriveControlContext_t;

typedef struct
{
    uint32_t attempt_count;
    uint32_t success_count;
    uint32_t failure_count;
    uint32_t last_failed_node;
    uint32_t last_left_duration_ms;
    uint32_t last_right_duration_ms;
    uint32_t last_total_duration_ms;
    uint32_t maximum_total_duration_ms;
} DriveSyncPairDiagnostics_t;

typedef struct
{
    DriveFaultSnapshot_t last_fault_snapshot;
    DriveSyncPairDiagnostics_t sync_pair;
    /* Rolling snapshot ids for the multi-line DRIVE_BASE_SAFE and YAW_CTRL
     * query responses; every line of one response carries the same snap=. */
    uint16_t base_safe_snapshot_sequence;
    uint16_t yaw_snapshot_sequence;
    uint16_t fault_debug_snapshot_sequence;
    uint16_t heartbeat_snapshot_sequence;
} DriveDiagnosticContext_t;

typedef struct
{
    uint32_t control_last_ms;
    uint32_t control_last_can_write_ms;
    uint32_t control_last_summary_ms;
    uint32_t status_last_ms;
} DriveTimingContext_t;

typedef struct
{
    DriveCoreState_t core;
    DriveCommandState_t command;
    DriveSafetyContext_t safety;
    DriveBootContext_t boot;
    DriveFaultResetContext_t fault_reset;
    DriveLiftStopContext_t lift_stop;
    DriveStatuswordContext_t statusword;
    DriveFeedbackContext_t feedback;
    DriveTpdoActiveContext_t tpdo_active;
    DriveEvidenceContext_t evidence;
    DriveStableZeroTracker_t stable_zero;
    DriveControlContext_t control;
    DriveDiagnosticContext_t diagnostic;
    DriveTimingContext_t timing;
} DriveContext_t;



#define DRIVE_LIFT_STATUSWORD_FRESH_MS DRIVE_WHEEL_FEEDBACK_STALE_MS
#define DRIVE_MODE_REQUEST_POLL_INTERVAL_MS 20U
#define DRIVE_MODE_REQUEST_TIMEOUT_MS       500U
#define DRIVE_FAULT_RESET_ZERO_FEEDBACK_TIMEOUT_MS 500U

typedef enum
{
    DRIVE_SDO_ACTION_READ_U16 = 0,
    DRIVE_SDO_ACTION_READ_U8,
    DRIVE_SDO_ACTION_READ_U32,
    DRIVE_SDO_ACTION_READ_I8,
    DRIVE_SDO_ACTION_WRITE_U8,
    DRIVE_SDO_ACTION_WRITE_U16,
    DRIVE_SDO_ACTION_WRITE_U32,
    DRIVE_SDO_ACTION_WRITE_I8,
    DRIVE_SDO_ACTION_WRITE_I32
} DriveSafetySdoAction_t;

typedef enum
{
    DRIVE_SDO_ACTION_WAIT = 0,
    DRIVE_SDO_ACTION_SUCCESS,
    DRIVE_SDO_ACTION_TIMEOUT,
    DRIVE_SDO_ACTION_ABORT,
    DRIVE_SDO_ACTION_ERROR
} DriveSafetySdoActionResult_t;

typedef enum
{
    DRIVE_EMCY_DELTA_NONE = 0,
    DRIVE_EMCY_DELTA_RESET_NOTIFICATION,
    DRIVE_EMCY_DELTA_FAULT
} DriveEmcyDelta_t;

extern FDCAN_HandleTypeDef hfdcan1;

extern DriveContext_t s_drive;
extern bool s_yaw_pi_enabled;
extern const MotionYawRatePIConfig_t s_yaw_rate_pi_config;
extern bool s_tracking_stop_latched;
extern bool s_tracking_stop_ignore_logged;
extern bool s_disable_motion_inhibit_latched;
extern bool s_hardware_enable_confirmed;
extern bool s_hardware_disable_confirmed;
#ifdef AMR_HOST_TEST
extern uint32_t s_communication_fault_stop_request_count;
void Drive_TestResetControlPrivateState(void);
#endif

void DriveCore_SetHardwareDisableConfirmed(bool confirmed);

bool Drive_BootInProgress(void);
void Drive_BootService(uint32_t now_ms);

void DriveCore_SetHardwareEnableConfirmed(bool confirmed);

void DriveCore_SetMotionInhibitLatched(bool latched);

void DriveCore_SetSoftwareEnabled(bool enabled);

void DriveCore_SetState(DriveState_t new_state);

void DriveCore_SetTrackingStopIgnoreLogged(bool logged);

void DriveCore_SetTrackingStopLatched(bool latched);

void DriveCore_WriteStateUnchecked(DriveState_t state);

void Drive_ApplyCmdVel(float linear_mm_s, float angular_mrad_s);

bool Drive_BaseHardwareSafeForLift(uint32_t now_ms);

bool Drive_BaseSafeNeedsVelocityPolling(void);

uint32_t Drive_BeginSafetyGeneration(void);

DriveEmcyDelta_t Drive_ClassifyEmcyDelta(
    const CanopenRuntimeDiagnostics_t *diag,
    uint8_t node_id,
    uint32_t recorded_sequence);

void Drive_ClearMotionCommandStateOnly(void);

bool Drive_CmdVelIsZero(float linear_mm_s, float angular_mrad_s);

uint32_t Drive_DiagnosticAge(uint32_t now_ms,
                                    bool valid,
                                    uint32_t timestamp_ms);

void Drive_EnableNodeEvidenceInitialize(uint8_t node_id);

void Drive_EnterHardFault(DriveSafetyFailureReason_t reason,
                          uint8_t node_id,
                          uint32_t now_ms);

void Drive_EnableStatePrint(uint16_t controlword,
                                   uint16_t statusword,
                                   uint16_t expected_state);

void Drive_EnsureMotionControlInitialized(void);

void Drive_FaultResetFail(DriveFaultResetFailure_t reason,
                                 uint8_t node_id,
                                 uint32_t now_ms);

void Drive_FaultResetStep(uint32_t now_ms);

bool Drive_FeedbackPairIsFreshZero(uint32_t now_ms);

const char *Drive_StableZeroRejectName(void);

const char *Drive_StableZeroStateName(void);

void Drive_StableZeroStart(uint32_t now_ms, bool settle_allowed);

DriveStableZeroResult_t Drive_StableZeroUpdate(uint32_t now_ms);

void Drive_StableZeroMarkTimeout(uint32_t now_ms);

void Drive_HandleFault(uint8_t fault_node_id);

void Drive_InvalidateActuatorBaseline(void);

void Drive_InvalidateZeroSpeedEvidence(void);

bool Drive_LastCommandTargetIsZero(void);

void Drive_LatchHeartbeatFault(void);

bool Drive_LiftIsMoving(void);

bool Drive_NormalMotionAllowed(void);

void Drive_MaintainZeroSpeedEvidence(uint32_t now_ms);

void Drive_MarkStatusFeedbackInvalid(void);

void Drive_ServiceWheelTpdoActiveCommand(uint32_t now_ms);
bool Drive_ServiceWheelTpdoStartup(uint32_t now_ms);
void Drive_PrintPdoStartupDiagnostic(uint32_t now_ms, uint32_t snap, uint32_t req);

void Drive_ModeConfirmPrint(const char *state);

void Drive_ModeConfirmPrintTimeout(void);

uint32_t Drive_OdomFaultStatus(void);

void Drive_PrintFaultResetDiagnostic(uint32_t now_ms,
                                            unsigned snap,
                                            uint32_t request_nonce);

void Drive_RecordFaultEvent(DriveSafetyFailureReason_t reason,
                                   uint8_t node_id,
                                   uint32_t now_ms);

void Drive_RecordStatusword(uint8_t node_id,
                                   uint16_t statusword,
                                   uint32_t now_ms,
                                   uint32_t generation);

void Drive_RecordZeroSpeedEvidence(
    uint32_t generation,
    uint32_t now_ms,
    DriveZeroEvidenceReason_t reason);

void Drive_ReportBaseSafeRefreshPending(void);

void Drive_ResetYawRateControl(bool invalidate_feedback);

bool Drive_RunBothMove(int32_t left_motor_rpm,
                              int32_t right_motor_rpm,
                              uint32_t duration_ms);

bool Drive_RunBothMoveOdom(int32_t left_motor_rpm,
                                  int32_t right_motor_rpm,
                                  uint32_t duration_ms);

bool Drive_RunNodeMove(uint8_t node_id, int32_t motor_rpm, uint32_t duration_ms);

void Drive_SafetyBeginRollback(
    DriveSafetyTransitionType_t rollback_type,
    DriveSafetyFailureReason_t reason,
    uint32_t now_ms);

void Drive_SafetyEscalate(DriveSafetyFailureReason_t reason,
                                 uint32_t now_ms);

const char *Drive_SafetyFailureName(DriveSafetyFailureReason_t reason);

bool Drive_SafetyIsRollback(void);

bool Drive_SafetyNeedsVelocityPolling(void);

void Drive_SafetySetPhase(DriveSafetyPhase_t phase,
                                 uint32_t now_ms);

void Drive_SafetyTransitionStep(uint32_t now_ms);

DriveConfirmedStopResult_t Drive_ServiceLiftStatuswordRefresh(
    uint32_t now_ms);

bool Drive_StartSafetyTransition(DriveSafetyTransitionType_t type,
                                        bool explicit_recovery,
                                        uint32_t now_ms);

const char *Drive_StateName(DriveState_t state);

bool Drive_StatuswordEvidenceIsFresh(bool valid,
                                            uint32_t timestamp_ms,
                                            uint32_t generation,
                                            uint32_t now_ms);

bool Drive_StopHasStableMeasuredZero(uint32_t now_ms);

bool Drive_TargetDecIsZero(int32_t left_velocity_dec, int32_t right_velocity_dec);

bool Drive_ZeroSpeedEvidenceIsCurrent(uint32_t now_ms);

bool Kinco_ReadActualPosition(uint8_t node_id, int32_t *pos_out);

bool Kinco_ReadActualSpeed(uint8_t node_id, int32_t *speed_out);

bool Kinco_StopNode(uint8_t node_id);

bool Odom_Update(bool print);

void Read_And_Print_ActualSpeed(uint8_t node_id);

uint16_t Read_And_Print_Statusword(uint8_t node_id);

uint16_t Statusword_Ds402State(uint16_t statusword);

bool Statusword_HasFault(uint16_t statusword);

bool Statusword_IsOperationEnabled(uint16_t statusword);

#endif
