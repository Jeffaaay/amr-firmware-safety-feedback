/* Exclusive owner of FDCAN1 dedicated TX buffers 0/1 and their TX events.
 * SDO/NMT/heartbeat keep the separate eight-entry hardware FIFO. Cooperative
 * foreground only: neither ISR nor another module may write these buffers. */
#include "wheel_rpdo_tx.h"
#include "wheel_pdo_config.h"
#include "wheel_pdo_profile.h"
#include "app_config.h"
#include "main.h"
#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;
#define RPDO_MASK 3U
static WheelRpdoDiagnostic diag;
static uint32_t started_ms, cancel_ms, previous_sent_ms;
static bool have_sent, fault_latched, cancelling, armed;
static uint8_t event_mask;
static uint8_t requested_mask;
static bool cancel_timeout_reported;
static uint16_t timestamps[2];

static uint32_t Pending(void)
{
    return hfdcan1.Instance->TXBRP & RPDO_MASK;
}

static bool BusHealthy(void)
{
    FDCAN_ProtocolStatusTypeDef status = {0};
    return (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &status) == HAL_OK) &&
           !status.BusOff;
}

void WheelRpdoTx_Encode(int32_t raw, uint8_t bytes[4])
{
    uint32_t value = (uint32_t)raw;
    for (uint8_t i = 0U; i < 4U; ++i) {
        bytes[i] = (uint8_t)(value >> (8U * i));
    }
}

static void DrainEvents(bool record)
{
    FDCAN_TxEventFifoTypeDef event;
    /* At most eight entries; no other production frames request TX events. */
    for (uint8_t n = 0U; n < 8U; ++n) {
        if ((hfdcan1.Instance->TXEFS & FDCAN_TXEFS_EFFL) == 0U) {
            break;
        }
        if (HAL_FDCAN_GetTxEvent(&hfdcan1, &event) != HAL_OK) {
            break;
        }
        if (record && (event.MessageMarker < 2U)) {
            uint8_t slot = (uint8_t)event.MessageMarker;
            uint32_t expected_id = slot ? WHEEL_RPDO_RIGHT_COB_ID :
                                          WHEEL_RPDO_LEFT_COB_ID;
            if ((event.Identifier == expected_id) &&
                (event.DataLength == FDCAN_DLC_BYTES_4)) {
                timestamps[slot] = (uint16_t)event.TxTimestamp;
                event_mask |= (uint8_t)(1U << slot);
            }
        }
    }
}

void WheelRpdoTx_Cancel(uint32_t now)
{
    armed = false;
    if (!cancelling) {
        cancelling = true;
        cancel_ms = now;
        cancel_timeout_reported = false;
        diag.cancel_count++;
    }
    diag.state = WHEEL_RPDO_CANCELLING;
    /* Cancellation may lose arbitration against a frame already on the wire.
     * TXBRP reaching zero is the barrier BEFORE any safety SDO is queued. */
    if (Pending() != 0U) {
        if (HAL_FDCAN_AbortTxRequest(&hfdcan1, RPDO_MASK) != HAL_OK) {
            fault_latched = true;
            diag.reason = "CANCEL_REQUEST_FAILED";
        }
    }
}

static void Fail(const char *reason, uint8_t node, uint32_t now)
{
    if (!fault_latched) {
        diag.failure_count++;
    }
    fault_latched = true;
    diag.reason = reason;
    diag.failed_node = node;
    WheelRpdoTx_Cancel(now);
}

void WheelRpdoTx_Service(uint32_t now)
{
    diag.pending_mask = Pending();
    diag.sent_mask = hfdcan1.Instance->TXBTO & requested_mask;
    if (cancelling) {
        if (diag.pending_mask == 0U) {
            DrainEvents(false);
            event_mask = 0U;
            cancelling = false;
            diag.state = fault_latched ? WHEEL_RPDO_FAULT : WHEEL_RPDO_IDLE;
        } else if ((uint32_t)(now - cancel_ms) > DRIVE_RUNTIME_TARGET_TIMEOUT_MS) {
            if (!cancel_timeout_reported) {
                diag.cancel_timeout_count++;
                cancel_timeout_reported = true;
            }
            fault_latched = true;
            diag.reason = "CANCEL_TIMEOUT_STOP_UNCONFIRMED";
            /* Never bypass the barrier, even on bus-off or cancellation loss. */
        }
        return;
    }
    if (diag.state != WHEEL_RPDO_QUEUED) {
        return;
    }
    if (!BusHealthy()) {
        diag.bus_off_count++;
        Fail("BUS_OFF", 0U, now);
        return;
    }
    if (!WheelPdoConfig_Ready(now)) {
        Fail("QUALIFICATION_LOST", 0U, now);
        return;
    }
    if ((uint32_t)(now - started_ms) > DRIVE_RUNTIME_TARGET_TIMEOUT_MS) {
        Fail("TX_TIMEOUT_OR_EVENT_LOSS", 0U, now);
        return;
    }
    DrainEvents(true);
    diag.sent_mask = hfdcan1.Instance->TXBTO & RPDO_MASK;
    if ((diag.pending_mask == 0U) && (diag.sent_mask == RPDO_MASK) &&
        (event_mask == RPDO_MASK)) {
        uint16_t delta = (uint16_t)(timestamps[1] - timestamps[0]);
        /* Arbitration order is not guaranteed. Shortest signed modular gap. */
        diag.last_skew_ticks = delta > 32767U ? 65536U - delta : delta;
        if (diag.last_skew_ticks > diag.max_skew_ticks) {
            diag.max_skew_ticks = diag.last_skew_ticks;
        }
        diag.last_latency_ms = (uint32_t)(now - started_ms);
        if (diag.last_latency_ms > diag.max_latency_ms) {
            diag.max_latency_ms = diag.last_latency_ms;
        }
        diag.last_interval_ms = have_sent ? (uint32_t)(now - previous_sent_ms) : 0U;
        if (diag.last_interval_ms > diag.max_interval_ms) {
            diag.max_interval_ms = diag.last_interval_ms;
        }
        previous_sent_ms = now;
        have_sent = true;
        diag.sent_pairs++;
        diag.state = WHEEL_RPDO_SENT;
    } else if ((hfdcan1.Instance->TXBCF & RPDO_MASK) != 0U) {
        Fail("UNEXPECTED_CANCEL", 0U, now);
    }
}

WheelRpdoSubmit WheelRpdoTx_Submit(int32_t left, int32_t right, uint32_t now)
{
    const int32_t values[2] = {left, right};
    if (!armed || fault_latched || !WheelPdoConfig_Ready(now)) {
        return WHEEL_RPDO_REJECTED;
    }
    if (!BusHealthy()) {
        diag.bus_off_count++;
        Fail("BUS_OFF", 0U, now);
        return WHEEL_RPDO_REJECTED;
    }
    /* FIFO capacity is diagnostic/precheck only, never an atomicity claim.
     * Keep two shared slots free for heartbeat/safety traffic. */
    if ((diag.state != WHEEL_RPDO_IDLE) || cancelling || (Pending() != 0U) ||
        (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) < 2U)) {
        diag.busy_count++;
        return WHEEL_RPDO_BUSY;
    }
    DrainEvents(false);
    event_mask = 0U;
    requested_mask = 0U;
    diag.sent_mask = 0U;
    started_ms = now;
    for (uint8_t slot = 0U; slot < 2U; ++slot) {
        FDCAN_TxHeaderTypeDef header = {0};
        uint8_t data[4];
        uint32_t mask = 1U << slot;
        header.Identifier = slot ? WHEEL_RPDO_RIGHT_COB_ID : WHEEL_RPDO_LEFT_COB_ID;
        header.IdType = FDCAN_STANDARD_ID;
        header.TxFrameType = FDCAN_DATA_FRAME;
        header.DataLength = FDCAN_DLC_BYTES_4;
        header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        header.BitRateSwitch = FDCAN_BRS_OFF;
        header.FDFormat = FDCAN_CLASSIC_CAN;
        header.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
        header.MessageMarker = slot;
        WheelRpdoTx_Encode(values[slot], data);
        if ((HAL_FDCAN_AddMessageToTxBuffer(&hfdcan1, &header, data, mask) != HAL_OK) ||
            (HAL_FDCAN_EnableTxBufferRequest(&hfdcan1, mask) != HAL_OK)) {
            Fail(slot ? "SECOND_FRAME_FAILED" : "FIRST_FRAME_FAILED",
                 slot ? RIGHT_NODE_ID : LEFT_NODE_ID, now);
            return WHEEL_RPDO_REJECTED;
        }
        requested_mask |= (uint8_t)mask;
    }
    diag.queued_pairs++;
    diag.state = WHEEL_RPDO_QUEUED;
    return WHEEL_RPDO_ACCEPTED;
}

bool WheelRpdoTx_Quiescent(void)
{
    return (Pending() == 0U) && !cancelling &&
           (diag.state != WHEEL_RPDO_QUEUED) && (diag.state != WHEEL_RPDO_SENT);
}

bool WheelRpdoTx_Rearm(uint32_t now)
{
    WheelRpdoTx_Service(now);
    if (!WheelRpdoTx_Quiescent() || !WheelPdoConfig_Ready(now) || !BusHealthy()) {
        return false;
    }
    fault_latched = false;
    armed = true;
    diag.state = WHEEL_RPDO_IDLE;
    diag.reason = "NONE";
    return true;
}

WheelRpdoState WheelRpdoTx_State(void) { return diag.state; }
void WheelRpdoTx_ConsumeSent(void)
{
    if (diag.state == WHEEL_RPDO_SENT) { diag.state = WHEEL_RPDO_IDLE; }
}
WheelRpdoDiagnostic WheelRpdoTx_Get(void) { return diag; }
#ifdef AMR_HOST_TEST
void WheelRpdoTx_TestReset(void)
{
    memset(&diag, 0, sizeof(diag));
    started_ms = cancel_ms = previous_sent_ms = 0U;
    have_sent = fault_latched = cancelling = armed = false;
    event_mask = requested_mask = 0U;
    cancel_timeout_reported = false;
}
#endif
