/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : drive_core.c
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
#include "jetson_tx.h"
#include "main.h"
#include "motion_control.h"
#include "odom.h"
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

bool s_tracking_stop_latched;
bool s_tracking_stop_ignore_logged;
bool s_disable_motion_inhibit_latched;
bool s_hardware_enable_confirmed;
bool s_hardware_disable_confirmed;
#ifdef AMR_HOST_TEST
uint32_t s_communication_fault_stop_request_count;
#endif

DriveContext_t s_drive = {
    .core = {
        .enabled = false,
        .state = DRIVE_STATE_INIT,
        .lift_moving_callback = NULL
    },
    .command = {0},
    .safety = {
        .transition = {
            .active = false
        }
    },
    .fault_reset = {
        .transaction = {
            .active = false
        },
        .generation = 0U
    },
    .lift_stop = {
        .request = {
            .active = false
        },
        .base_safe_refresh_txn = 0U
    },
    .statusword = {
        .left_statusword = 0U
    },
    .feedback = {
        .pair_pending = false
    },
    .evidence = {
        .safety_generation = 0U
    }
};

void Drive_SetLiftMovingCallback(bool (*fn)(void))
{
    s_drive.core.lift_moving_callback = fn;
}

bool Drive_LiftIsMoving(void)
{
    return (s_drive.core.lift_moving_callback != NULL) && s_drive.core.lift_moving_callback();
}

bool Drive_NormalMotionAllowed(void)
{
    bool state_allows_motion =
        (s_drive.core.state == DRIVE_STATE_ENABLED_IDLE) ||
        (s_drive.core.state == DRIVE_STATE_RUNNING);
    bool hard_fault_latched = s_drive.core.state == DRIVE_STATE_FAULT;
    bool lift_moving = Drive_LiftIsMoving();

    return !Drive_BootInProgress() && state_allows_motion && !hard_fault_latched && !lift_moving &&
        WheelPdoConfig_Ready(HAL_GetTick()) &&
        (Canopen_GetWheelFeedbackTransport() == CANOPEN_WHEEL_FEEDBACK_TPDO_ACTIVE);
}

uint32_t Drive_GetStateForDiagnostics(void)
{
    return (uint32_t)s_drive.core.state;
}

bool Drive_IsFaulted(void)
{
    return s_drive.core.state == DRIVE_STATE_FAULT;
}

const char *Drive_StateName(DriveState_t state)
{
    switch (state) {
        case DRIVE_STATE_INIT: return "INIT";
        case DRIVE_STATE_ENABLED_IDLE: return "ENABLED_IDLE";
        case DRIVE_STATE_RUNNING: return "RUNNING";
        case DRIVE_STATE_STOPPING: return "STOPPING";
        case DRIVE_STATE_DISABLING: return "DISABLING";
        case DRIVE_STATE_FAULT: return "FAULT";
        case DRIVE_STATE_DISABLED: return "DISABLED";
        case DRIVE_STATE_ENABLING: return "ENABLING";
        default: return "UNKNOWN";
    }
}

void DriveCore_SetHardwareEnableConfirmed(bool confirmed)
{
    s_hardware_enable_confirmed = confirmed;
}

void DriveCore_SetHardwareDisableConfirmed(bool confirmed)
{
    s_hardware_disable_confirmed = confirmed;
}

void DriveCore_SetSoftwareEnabled(bool enabled)
{
    s_drive.core.enabled = enabled;
}

void DriveCore_SetTrackingStopLatched(bool latched)
{
    s_tracking_stop_latched = latched;
}

void DriveCore_SetMotionInhibitLatched(bool latched)
{
    s_disable_motion_inhibit_latched = latched;
}

void DriveCore_SetTrackingStopIgnoreLogged(bool logged)
{
    s_tracking_stop_ignore_logged = logged;
}

void DriveCore_WriteStateUnchecked(DriveState_t state)
{
    s_drive.core.state = state;
}

void DriveCore_SetState(DriveState_t new_state)
{
#if DRIVE_BASIC_BRINGUP_PROFILE
    if ((new_state == DRIVE_STATE_ENABLED_IDLE) &&
        (!s_drive.core.enabled || !s_hardware_enable_confirmed ||
         s_drive.safety.transition.active)) {
        uart_printf("DRIVE invariant rejected ENABLED_IDLE publication\r\n");
        DriveCore_SetSoftwareEnabled(false);
        new_state = DRIVE_STATE_FAULT;
    }
    if ((new_state == DRIVE_STATE_DISABLED) &&
        (s_drive.core.enabled || !s_hardware_disable_confirmed ||
         s_drive.safety.transition.active)) {
        uart_printf("DRIVE invariant rejected DISABLED publication\r\n");
        DriveCore_SetSoftwareEnabled(false);
        new_state = DRIVE_STATE_FAULT;
    }
    if ((new_state == DRIVE_STATE_RUNNING) &&
        (!s_drive.core.enabled ||
         (s_drive.core.state == DRIVE_STATE_FAULT) ||
         Drive_LiftIsMoving())) {
        uart_printf("DRIVE invariant rejected RUNNING publication\r\n");
        DriveCore_SetSoftwareEnabled(false);
        new_state = DRIVE_STATE_FAULT;
    }
#else
    if ((new_state == DRIVE_STATE_ENABLED_IDLE) &&
        (!s_drive.core.enabled || !s_hardware_enable_confirmed ||
         s_drive.safety.transition.active || s_tracking_stop_latched ||
         s_disable_motion_inhibit_latched ||
         !Drive_LastCommandTargetIsZero() ||
         !Drive_ZeroSpeedEvidenceIsCurrent(HAL_GetTick()))) {
        uart_printf("DRIVE invariant rejected ENABLED_IDLE publication\r\n");
        DriveCore_SetSoftwareEnabled(false);
        DriveCore_SetMotionInhibitLatched(true);
        new_state = DRIVE_STATE_FAULT;
    }
    if ((new_state == DRIVE_STATE_DISABLED) &&
        (s_drive.core.enabled || !s_hardware_disable_confirmed ||
         s_drive.safety.transition.active)) {
        uart_printf("DRIVE invariant rejected DISABLED publication\r\n");
        DriveCore_SetSoftwareEnabled(false);
        DriveCore_SetMotionInhibitLatched(true);
        new_state = DRIVE_STATE_FAULT;
    }
    if ((new_state == DRIVE_STATE_RUNNING) &&
        (!s_drive.core.enabled || !s_hardware_enable_confirmed ||
         s_drive.safety.transition.active || s_tracking_stop_latched ||
         s_disable_motion_inhibit_latched)) {
        uart_printf("DRIVE invariant rejected RUNNING publication\r\n");
        DriveCore_SetSoftwareEnabled(false);
        DriveCore_SetMotionInhibitLatched(true);
        new_state = DRIVE_STATE_FAULT;
    }
#endif
    if ((new_state == DRIVE_STATE_FAULT) && Drive_BootInProgress()) {
        s_drive.boot.state = DRIVE_BOOT_FAILED;
        Canopen_BootSdoEnd();
        DriveCore_SetHardwareDisableConfirmed(false);
        Drive_ClearMotionCommandStateOnly();
        Drive_InvalidateZeroSpeedEvidence();
        (void)JetsonTx_Printf(JETSON_TX_PRIORITY_CRITICAL,
            "DRIVE_BOOT attempt=%u state=FAILED final=FAULT_LATCHED\r\n",
            (unsigned)(s_drive.boot.retries + 1U));
    }
    if (s_drive.core.state != new_state) {
        uart_printf("DRIVE STATE: %s -> %s\r\n",
                    Drive_StateName(s_drive.core.state),
                    Drive_StateName(new_state));
        DriveCore_WriteStateUnchecked(new_state);

#if DRIVE_ZERO_HOLD_ON_IDLE && !DRIVE_IDLE_AUTO_DISABLE_ENABLE
        if (new_state == DRIVE_STATE_ENABLED_IDLE) {
            uart_printf("DRIVE zero-speed hold active, auto-disable disabled\r\n");
        }
#endif
    }
}

bool Drive_TargetDecIsZero(int32_t left_velocity_dec, int32_t right_velocity_dec)
{
    return (left_velocity_dec == 0) && (right_velocity_dec == 0);
}

bool Drive_LastCommandTargetIsZero(void)
{
    return (s_drive.command.last_left_cmd_rpm == 0) &&
           (s_drive.command.last_right_cmd_rpm == 0) &&
           (s_drive.command.last_left_cmd_dec == 0) &&
           (s_drive.command.last_right_cmd_dec == 0);
}

bool Drive_CmdVelIsZero(float linear_mm_s, float angular_mrad_s)
{
    return (fabsf(linear_mm_s) < 0.001f) &&
           (fabsf(angular_mrad_s) < 0.001f);
}

bool Drive_IsMotionCommandActive(void)
{
    return (s_drive.core.state == DRIVE_STATE_RUNNING) || s_drive.command.jetson_cmd_active;
}

bool Drive_IsYawPiEnabled(void)
{
    return s_yaw_pi_enabled;
}

#ifdef AMR_HOST_TEST
void Drive_TestSetTrackingLatch(bool latched)
{
    DriveCore_SetTrackingStopLatched(latched);
}
#endif

#ifdef AMR_HOST_TEST
void Drive_TestSetDisableInhibitLatch(bool latched)
{
    DriveCore_SetMotionInhibitLatched(latched);
}
#endif

#ifdef AMR_HOST_TEST
DriveState_t Drive_TestGetState(void) { return s_drive.core.state; }
#endif

#ifdef AMR_HOST_TEST
bool Drive_TestTrackingLatch(void) { return s_tracking_stop_latched; }
#endif

#ifdef AMR_HOST_TEST
bool Drive_TestDisableInhibitLatch(void)
{
    return s_disable_motion_inhibit_latched;
}
#endif

#ifdef AMR_HOST_TEST
bool Drive_TestIsEnabled(void) { return s_drive.core.enabled; }
#endif

#ifdef AMR_HOST_TEST
uint32_t Drive_TestOdomFaultStatus(void)
{
    return Drive_OdomFaultStatus();
}
#endif
