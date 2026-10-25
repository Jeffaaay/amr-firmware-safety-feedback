#ifndef WHEEL_PDO_CONFIG_H
#define WHEEL_PDO_CONFIG_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool configured, operational;
    uint8_t step, subindex;
    uint16_t index;
    uint32_t epoch, expected, actual, abort_code;
    uint32_t readback[4]; /* final COB-ID/type/map count/map entry */
    const char *phase, *reason;
} WheelPdoConfigDiagnostic;

/* Caller must have confirmed BOTH drives disabled; no persistence writes. */
bool WheelPdoConfig_Configure(void);
bool WheelPdoConfig_Ready(uint32_t now_ms);
void WheelPdoConfig_OnHeartbeat(uint8_t node, uint8_t state, uint32_t now_ms);
WheelPdoConfigDiagnostic WheelPdoConfig_Get(uint8_t node);
const char *WheelPdoConfig_FailureReason(void);
#ifdef AMR_HOST_TEST
void WheelPdoConfig_TestReset(void);
#endif
#endif
