#ifndef DRIVE_LIFT_STOP_DIAG_H
#define DRIVE_LIFT_STOP_DIAG_H
#include <stdbool.h>
#include <stdint.h>

/* Read-only, bounded UART snapshots. Never refreshes or authorizes evidence. */
void DriveLiftStopDiag_Reset(void);
void DriveLiftStopDiag_Report(uint32_t now, const char *event, const char *reason,
                              uint8_t node, bool static_safe, bool force);
#endif
