#ifndef WHEEL_RPDO_TX_H
#define WHEEL_RPDO_TX_H
#include <stdbool.h>
#include <stdint.h>

typedef enum { WHEEL_RPDO_IDLE, WHEEL_RPDO_QUEUED, WHEEL_RPDO_SENT,
               WHEEL_RPDO_CANCELLING, WHEEL_RPDO_FAULT } WheelRpdoState;
typedef enum { WHEEL_RPDO_ACCEPTED, WHEEL_RPDO_BUSY,
               WHEEL_RPDO_REJECTED } WheelRpdoSubmit;
typedef struct {
    WheelRpdoState state;
    uint32_t queued_pairs, sent_pairs, busy_count, failure_count, cancel_count;
    uint32_t pending_mask, sent_mask, last_latency_ms, max_latency_ms;
    uint32_t last_interval_ms, max_interval_ms, last_skew_ticks, max_skew_ticks;
    uint32_t bus_off_count, cancel_timeout_count;
    uint8_t failed_node;
    const char *reason;
} WheelRpdoDiagnostic;

/* ACCEPTED means queued only. SENT is local CAN completion, never SDO ACK
 * or proof of drive application. Only one pair may be outstanding. */
WheelRpdoSubmit WheelRpdoTx_Submit(int32_t left, int32_t right, uint32_t now);
void WheelRpdoTx_Service(uint32_t now);
WheelRpdoState WheelRpdoTx_State(void);
void WheelRpdoTx_ConsumeSent(void);
void WheelRpdoTx_Cancel(uint32_t now);
bool WheelRpdoTx_Quiescent(void);
bool WheelRpdoTx_Rearm(uint32_t now);
WheelRpdoDiagnostic WheelRpdoTx_Get(void);
void WheelRpdoTx_Encode(int32_t raw, uint8_t bytes[4]);
#ifdef AMR_HOST_TEST
void WheelRpdoTx_TestReset(void);
#endif
#endif
