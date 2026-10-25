#ifndef FAKE_BUS_H
#define FAKE_BUS_H
#include <stdbool.h>
#include "main.h"
extern uint32_t tick, tick_step, fifo_free, bus_off, sdo_ops, fail_sdo, mismatch_sdo;
extern uint32_t fail_add_mask, fail_request_mask, cancel_hold, restart_sdo, safety_zero_frames;
extern uint32_t suppress_response, fail_nmt, fail_cancel;
extern uint32_t injected_abort, bad_width_sdo, initial_type, type_writes, drop_event;
extern uint32_t identity_accesses;
extern uint8_t last_sdo_node, last_sdo_sub, last_sdo_write, silent_node, drop_nmt;
extern uint16_t last_sdo_index;
extern uint16_t wheel_statusword[3];
extern uint32_t enable_controlwords, reset_controlwords;
extern bool persistent_wheel_fault;
/* Optional wire-level injection. True means the hook supplied or deliberately
 * withheld the response; false keeps the ordinary simulated device behavior. */
typedef bool (*FakeSdoHook_t)(uint8_t node, uint16_t index, uint8_t sub,
                             const uint8_t request[8]);
extern FakeSdoHook_t fake_sdo_hook;
typedef void (*FakeRxConsumedHook_t)(uint32_t id);
extern FakeRxConsumedHook_t fake_rx_consumed_hook;
extern uint32_t fake_rx_capacity;
void TestPdoStartupRecovery(void);
extern FDCAN_TxHeaderTypeDef tx_headers[2];
extern uint8_t tx_data[2][4];
void Fake_Reset(void);
void Fake_Rx(uint32_t id, const uint8_t *data, uint8_t length);
/* Raw DLC injection also models an FD frame which HAL copies before the
 * classic-CAN application rejects its unsupported DLC encoding. */
void Fake_RxWithDlc(uint32_t id, const uint8_t *data, uint8_t length, uint8_t dlc);
void Fake_Heartbeat(uint8_t node, uint8_t state);
void Fake_Tpdo(int32_t left, int32_t right);
void Fake_Complete(uint8_t slot, uint16_t timestamp);
void TestDriveReady(void);
void TestDriveReadyAt(uint32_t start);
void TestDriveRetarget(void);
void TestLiftBaseStop(void);
const char *Fake_Log(void);
void Fake_ClearLog(void);
#endif
