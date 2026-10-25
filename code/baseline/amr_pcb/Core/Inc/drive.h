/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : drive.h
  * @brief          : Differential drive control interfaces.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef DRIVE_H
#define DRIVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "command_result.h"

typedef enum
{
    DRIVE_STATE_INIT = 0,
    DRIVE_STATE_ENABLED_IDLE,
    DRIVE_STATE_RUNNING,
    DRIVE_STATE_STOPPING,
    DRIVE_STATE_DISABLING,
    DRIVE_STATE_FAULT,
    DRIVE_STATE_DISABLED,
    DRIVE_STATE_ENABLING
} DriveState_t;

/* Read-only status used by board indicators and diagnostics. */
bool Drive_IsFaulted(void);

typedef enum
{
    DRIVE_STOP_ZERO_ACCEPTED = 0,
    DRIVE_STOP_PENDING,
    DRIVE_STOP_FAILED
} DriveStopResult_t;

typedef enum
{
    DRIVE_DISABLE_CONFIRMED = 0,
    DRIVE_DISABLE_PENDING,
    DRIVE_DISABLE_FAILED
} DriveDisableResult_t;

typedef enum
{
    DRIVE_CONFIRMED_STOP_PENDING = 0,
    DRIVE_CONFIRMED_STOP_OK,
    DRIVE_CONFIRMED_STOP_FAILED
} DriveConfirmedStopResult_t;

typedef enum
{
    DRIVE_WHEEL_AUTHORITY_SDO = 0,
    DRIVE_WHEEL_AUTHORITY_TPDO
} DriveWheelFeedbackAuthority_t;

typedef enum
{
    DRIVE_WHEEL_PAIR_NO_DATA = 0,
    DRIVE_WHEEL_PAIR_FRESH,
    DRIVE_WHEEL_PAIR_STALE,
    DRIVE_WHEEL_PAIR_FAULT
} DriveWheelFeedbackPairState_t;

typedef struct
{
    int32_t left_raw_velocity;
    int32_t right_raw_velocity;
    float normalized_left_velocity_mm_s;
    float normalized_right_velocity_mm_s;
    uint32_t pair_timestamp_ms;
    uint32_t pair_age_ms;
    uint32_t pair_sequence;
    uint32_t left_node_sequence;
    uint32_t right_node_sequence;
    uint32_t pair_skew_ms;
    bool pair_valid;
    DriveWheelFeedbackPairState_t pair_state;
    DriveWheelFeedbackAuthority_t authority_source;
} DriveWheelFeedbackPair_t;

void Drive_SetLiftMovingCallback(bool (*fn)(void));

void Drive_ClearMotionCommand(void);
void Drive_CommandStop(void);
/* UART damage discards the affected command and requests the same recoverable
 * zero/IDLE stop used by the command watchdog. A zero-write failure still
 * escalates through the hard-fault cleanup contract. */
void Drive_RequestCommunicationFaultStop(void);
/* Typed results carry the real accept/reject reason for CMD_ACK; the bool
 * forms are compatibility wrappers (true == CMD_RESULT_ACCEPTED). */
CommandResult_t Drive_CommandDisableChecked(void);
CommandResult_t Drive_CommandEnableChecked(void);
bool Drive_CommandDisable(void);
bool Drive_CommandEnable(void);
bool Drive_CommandFaultResetSafe(uint32_t request_nonce);
void Drive_CommandSetCmdVel(float linear_mm_s, float angular_mrad_s);
bool Drive_CommandSetYawPiEnabled(bool enabled);
bool Drive_IsYawPiEnabled(void);
void Drive_PrintYawPiStatus(void);
bool Drive_IsMotionCommandActive(void);
DriveConfirmedStopResult_t Drive_RequestConfirmedStopForLift(void);
void Drive_CancelConfirmedStopForLift(void);
bool Drive_RequestBaseSafeRefresh(uint32_t request_nonce);
bool Drive_CommandConfigureWheelTpdo(uint8_t node_id,
                                     uint32_t request_nonce);
void Drive_PrintWheelTpdoStatus(uint32_t request_nonce);
bool Drive_CommandWheelTpdoActiveEnable(uint32_t request_nonce);
bool Drive_CommandWheelTpdoActiveDisable(uint32_t request_nonce);
void Drive_PrintWheelTpdoActiveStatus(uint32_t request_nonce);
bool Drive_GetAuthoritativeWheelFeedbackPair(
    uint32_t now_ms,
    DriveWheelFeedbackPair_t *pair);

void Drive_ControlLoop_Update(void);
void Drive_SendJetsonStatus_100ms(void);

bool Drive_EnsureEnabled(void);
DriveStopResult_t Drive_StopBoth(void);
DriveDisableResult_t Drive_DisableBoth(void);
bool Drive_SetMotorRPM(int32_t left_motor_rpm, int32_t right_motor_rpm);

void Drive_PrintFeedback(void);
void Drive_PrintYawRateDiagnostics(uint32_t request_nonce);
void Drive_PrintFaultDebug(uint32_t request_nonce);
void Drive_PrintHeartbeatCompatibilityDebug(uint32_t request_nonce);
void Drive_PrintHeartbeatDebug(uint32_t request_nonce);
void Drive_PrintBaseSafeDebug(uint32_t request_nonce);
void Drive_PrintHelp(void);

int32_t Clamp_TestMotorRPM(int32_t rpm);
uint32_t Clamp_TestDurationMS(uint32_t duration_ms);
int32_t Clamp_ClosedLoopMaxRPM(int32_t max_rpm);

bool Odom_Reset(void);
bool Odom_UpdateAndPrint(void);

void Drive_MoveMM_ClosedLoop(float target_mm, int32_t max_rpm);
void Drive_TurnDeg_ClosedLoop(float target_deg, int32_t max_rpm);
void Drive_Console_MoveBoth(int32_t left_rpm,
                            int32_t right_rpm,
                            uint32_t duration_ms);
void Drive_Console_MoveNode(uint8_t node_id, int32_t rpm, uint32_t duration_ms);
void Drive_Console_MoveBothOdom(int32_t left_rpm,
                                int32_t right_rpm,
                                uint32_t duration_ms);
void Drive_ConsoleApplyCmdVel(float linear_mm_s, float angular_mrad_s);

#ifdef AMR_HOST_TEST
void Drive_TestResetState(void);
void Drive_TestSetRuntimeReady(DriveState_t state,
                               float submitted_linear_mm_s,
                               float submitted_angular_mrad_s,
                               uint32_t timestamp_ms);
void Drive_TestStoreFeedbackPair(uint32_t sequence,
                                 uint32_t timestamp_ms,
                                 float left_wheel_mm_s,
                                 float right_wheel_mm_s);
void Drive_TestStoreFeedbackPairWithSkew(uint32_t sequence,
                                         uint32_t left_timestamp_ms,
                                         uint32_t right_timestamp_ms,
                                         float left_wheel_mm_s,
                                         float right_wheel_mm_s);
void Drive_TestStoreFeedbackNodes(uint32_t left_sequence,
                                  uint32_t right_sequence,
                                  uint32_t left_timestamp_ms,
                                  uint32_t right_timestamp_ms,
                                  float left_wheel_mm_s,
                                  float right_wheel_mm_s);
void Drive_TestInvalidateFeedbackPair(void);
void Drive_TestSetTrackingLatch(bool latched);
void Drive_TestSetDisableInhibitLatch(bool latched);
void Drive_TestServiceStopConfirmation(uint32_t now_ms);
DriveState_t Drive_TestGetState(void);
bool Drive_TestStopIsPending(void);
bool Drive_TestTrackingLatch(void);
bool Drive_TestDisableInhibitLatch(void);
bool Drive_TestIsEnabled(void);
bool Drive_TestSafetyTransitionActive(void);
bool Drive_TestFaultResetActive(void);
uint32_t Drive_TestFaultResetPhase(void);
uint32_t Drive_TestFaultResetFailureReason(void);
uint32_t Drive_TestFaultResetGeneration(void);
uint32_t Drive_TestSafetyTransitionType(void);
uint32_t Drive_TestSafetyTransitionPhase(void);
uint32_t Drive_TestSafetyTransitionGeneration(void);
uint32_t Drive_TestSafetyTransientTimeoutCount(void);
uint32_t Drive_TestStableZeroSampleCount(void);
uint32_t Drive_TestStableZeroState(void);
uint32_t Drive_TestStableZeroRejectReason(void);
uint32_t Drive_TestPiResetCount(void);
bool Drive_TestHardwareEnableConfirmed(void);
bool Drive_TestHardwareDisableConfirmed(void);
void Drive_TestSetHardwareDisableConfirmed(bool confirmed);
void Drive_TestSetLastCommandTargetDec(int32_t left, int32_t right);
void Drive_TestSetStatusword(uint8_t node_id,
                             uint16_t statusword,
                             bool valid);
void Drive_TestSetStatuswordTimestamp(uint8_t node_id,
                                      uint32_t timestamp_ms);
void Drive_TestSetStatuswordGeneration(uint8_t node_id,
                                       uint32_t generation);
bool Drive_TestBaseHardwareSafeForLift(uint32_t now_ms);
bool Drive_TestStatuswordsConfirmDisabled(uint32_t now_ms);
bool Drive_TestPostDisableZeroProofRequired(void);
bool Drive_TestBaseSafeRefreshActive(void);
DriveConfirmedStopResult_t Drive_TestServiceLiftStopRequest(uint32_t now_ms);
bool Drive_TestZeroEvidenceValid(void);
uint32_t Drive_TestZeroEvidenceGeneration(void);
uint32_t Drive_TestZeroEvidenceReason(void);
uint32_t Drive_TestOdomFaultStatus(void);
uint32_t Drive_TestPiLastSequence(void);
bool Drive_TestPiFeedbackSequenceValid(void);
float Drive_TestPiIntegral(void);
uint32_t Drive_TestPiPrepareCount(void);
uint32_t Drive_TestSubmitCount(void);
uint32_t Drive_TestFeedbackReplacedCount(void);
bool Drive_TestFeedbackPairPending(void);
bool Drive_TestChassisFeedbackNewSample(void);
uint32_t Drive_TestTrackingLastSequence(void);
bool Drive_TestTrackingFeedbackSequenceValid(void);
uint32_t Drive_TestTrackingState(void);
uint32_t Drive_TestTrackingWrongSignObservationCount(void);
float Drive_TestSubmittedCorrectionMradS(void);
float Drive_TestSubmittedLinearMmS(void);
float Drive_TestSubmittedAngularMradS(void);
float Drive_TestSubmittedLeftMotorRpm(void);
float Drive_TestSubmittedRightMotorRpm(void);
float Drive_TestRequestedLinearMmS(void);
float Drive_TestRequestedAngularMradS(void);
float Drive_TestTargetLinearMmS(void);
float Drive_TestTargetAngularMradS(void);
uint32_t Drive_TestLastJetsonCommandMs(void);
uint32_t Drive_TestCommunicationFaultStopRequestCount(void);
void Drive_TestSetTpdoEstopActive(bool active);
bool Drive_TestTpdoActiveCommandPending(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* DRIVE_H */
