#ifndef DRIVE_TARGET_H
#define DRIVE_TARGET_H
#include "motion_control.h"
#include <stdbool.h>
#include <stdint.h>

/* Thin private bridge: one immutable in-flight proposal, latest command stays
 * in the existing Drive context. No historical target queue. */
void DriveTarget_Submit(uint32_t now, bool pi_prepared,
                        MotionTrackingLimits_t limits);
bool DriveTarget_Service(uint32_t now, bool *pi_committed);
void DriveTarget_Cancel(uint32_t now);
bool DriveTarget_Pending(void);
/* Copy the real committed state with only its calculation time substituted.
 * Hold is allowed only for an unchanged transmitted target or idle departure. */
MotionActuatorState_t DriveTarget_ShapingState(void);
void DriveTarget_Hold(uint32_t now);
const MotionChassisFeedback_t *DriveTarget_CommittedFeedback(void);
void DriveTarget_RecordControl(uint32_t now);
void DriveTarget_PrintDiagnostics(uint32_t request_nonce);
#endif
