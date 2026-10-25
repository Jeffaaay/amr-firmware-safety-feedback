/* Candidate RPDO2 commissioning. Existing TPDO configuration remains owned
 * by canopen.c; this module does not change units, mode, signs or geometry. */
#include "wheel_pdo_config.h"
#include "wheel_pdo_profile.h"
#include "wheel_rpdo_tx.h"
#include "canopen.h"
#include "can_trace.h"
#include "app_config.h"
#include "jetson_tx.h"
#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;
static WheelPdoConfigDiagnostic nodes[2];
static uint32_t heartbeat_ms[2], heartbeat_sequence[2];
static uint8_t nmt_state[2];
static uint32_t operation_abort_baseline;
static uint8_t failed_node;

static int Slot(uint8_t node)
{
    return node == LEFT_NODE_ID ? 0 : (node == RIGHT_NODE_ID ? 1 : -1);
}

static void Report(uint8_t node, const WheelPdoConfigDiagnostic *d)
{
    /* Startup is disabled. Bound log backpressure while servicing CAN and
     * master heartbeat so a fast SDO peer cannot overflow the UART queue. */
    uint32_t start = HAL_GetTick();
    while ((JetsonTx_GetStats().queue_depth[JETSON_TX_PRIORITY_STATUS] > 8U) &&
           ((uint32_t)(HAL_GetTick() - start) < 50U)) {
        JetsonTx_Process();
        Canopen_ProcessRx(HAL_GetTick(), DRIVE_CANOPEN_RX_BUDGET_FRAMES);
        Canopen_HeartbeatService(HAL_GetTick());
        HAL_Delay(1U);
    }
    JetsonTx_Printf(JETSON_TX_PRIORITY_STATUS,
        "WHEEL_RPDO_CONFIG node=%u step=%u phase=%s index=%04X sub=%02X expected=%08lX actual=%08lX abort=%08lX result=%s epoch=%lu\r\n",
        (unsigned)node, (unsigned)d->step, d->phase, (unsigned)d->index,
        (unsigned)d->subindex, (unsigned long)d->expected,
        (unsigned long)d->actual, (unsigned long)d->abort_code,
        d->reason, (unsigned long)d->epoch);
}

static bool FinishStep(uint8_t node, bool ok, const char *failure)
{
    WheelPdoConfigDiagnostic *d = &nodes[Slot(node)];
    d->reason = ok ? "READBACK_OK" : failure;
    if (!ok) {
        failed_node = node;
        d->configured = false;
        CanopenRuntimeDiagnostics_t runtime = Canopen_GetRuntimeDiagnostics();
        d->abort_code = ((runtime.sdo_abort_count != operation_abort_baseline) &&
                         (runtime.last_sdo_abort_node == node)) ?
            runtime.last_sdo_abort_code : 0U;
        if (d->abort_code == 0x06040041UL) { d->reason = "OBJECT_NOT_MAPPABLE"; }
    }
    Report(node, d);
    return ok;
}

static void BeginStep(uint8_t node, const char *phase, uint16_t index,
                      uint8_t subindex, uint32_t expected)
{
    WheelPdoConfigDiagnostic *d = &nodes[Slot(node)];
    d->step++;
    d->phase = phase;
    d->index = index;
    d->subindex = subindex;
    d->expected = expected;
    d->actual = d->abort_code = 0U;
    operation_abort_baseline = Canopen_GetRuntimeDiagnostics().sdo_abort_count;
}

static bool EpochMatches(uint8_t node, uint32_t expected)
{
    WheelPdoConfigDiagnostic *d = &nodes[Slot(node)];
    if (d->epoch == expected) { return true; }
    BeginStep(node, "RESTART_CHECK", 0U, 0U, expected);
    d->actual = d->epoch;
    return FinishStep(node, false, "NODE_RESTARTED");
}

static bool Read(uint8_t node, uint16_t index, uint8_t sub, uint8_t width,
                 uint32_t *value)
{
    CanopenNode_t handle = {&hfdcan1, node};
    operation_abort_baseline = Canopen_GetRuntimeDiagnostics().sdo_abort_count;
    if (width == 1U) {
        int8_t result = 0;
        bool ok = SDO_Read_I8_Timeout_Node(&handle, index, sub, &result,
                                         DRIVE_WHEEL_TPDO_CONFIG_SDO_TIMEOUT_MS);
        *value = (uint8_t)result;
        return ok && (s_last_sdo_response.command == 0x4FU);
    }
    if (width == 2U) {
        uint16_t result = 0U;
        bool ok = SDO_Read_U16_Timeout_Node(&handle, index, sub, &result,
                                          DRIVE_WHEEL_TPDO_CONFIG_SDO_TIMEOUT_MS);
        *value = result;
        return ok && (s_last_sdo_response.command == 0x4BU);
    }
    int32_t result = 0;
    bool ok = SDO_Read_I32_Timeout_Node(&handle, index, sub, &result,
                                      DRIVE_WHEEL_TPDO_CONFIG_SDO_TIMEOUT_MS);
    *value = (uint32_t)result;
    return ok && (s_last_sdo_response.command == 0x43U);
}

static bool WriteVerify(uint8_t node, uint16_t index, uint8_t sub,
                        uint8_t width, uint32_t expected)
{
    WheelPdoConfigDiagnostic *d = &nodes[Slot(node)];
    CanopenNode_t handle = {&hfdcan1, node};
    uint32_t epoch = d->epoch;
    bool ok;
    BeginStep(node, "WRITE", index, sub, expected);
    if (width == 1U) {
        ok = SDO_Write_U8_Timeout_Node(&handle, index, sub, (uint8_t)expected,
                                      DRIVE_WHEEL_TPDO_CONFIG_SDO_TIMEOUT_MS);
    } else if (width == 2U) {
        ok = SDO_Write_U16_Timeout_Node(&handle, index, sub, (uint16_t)expected,
                                       DRIVE_WHEEL_TPDO_CONFIG_SDO_TIMEOUT_MS);
    } else {
        ok = SDO_Write_I32_Timeout_Node(&handle, index, sub, (int32_t)expected,
                                       DRIVE_WHEEL_TPDO_CONFIG_SDO_TIMEOUT_MS);
    }
    if (!ok) { return FinishStep(node, false, "WRITE_FAILED"); }
    d->phase = "READBACK";
    if (!Read(node, index, sub, width, &d->actual)) {
        return FinishStep(node, false, "READ_FAILED");
    }
    if (!EpochMatches(node, epoch)) { return false; }
    return FinishStep(node, d->actual == expected, "READBACK_MISMATCH");
}

static bool PreOperational(uint8_t node)
{
    int slot = Slot(node);
    WheelPdoConfigDiagnostic *d = &nodes[slot];
    uint32_t baseline = heartbeat_sequence[slot];
    uint32_t start = HAL_GetTick();
    FDCAN_TxHeaderTypeDef tx = {0};
    uint8_t data[2] = {0x80U, node};
    tx.IdType = FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME;
    tx.DataLength = FDCAN_DLC_BYTES_2;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch = FDCAN_BRS_OFF;
    tx.FDFormat = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    BeginStep(node, "NMT_PREOP", 0U, 0U, 0x7FU);
    if (CanTrace_SubmitTx(&hfdcan1, &tx, data, CAN_TRACE_TX_NMT,
                         node, 0U, 0U, false) != HAL_OK) {
        return FinishStep(node, false, "NMT_PREOP_TX_FAILED");
    }
    do {
        uint32_t now = HAL_GetTick();
        Canopen_ProcessRx(now, DRIVE_CANOPEN_RX_BUDGET_FRAMES);
        Canopen_HeartbeatService(now);
        JetsonTx_Process();
        if ((heartbeat_sequence[slot] != baseline) && (nmt_state[slot] == 0x7FU)) {
            d->actual = 0x7FU;
            return FinishStep(node, true, "NONE");
        }
    } while ((uint32_t)(HAL_GetTick() - start) <
             DRIVE_WHEEL_TPDO_CONFIG_SDO_TIMEOUT_MS);
    d->actual = nmt_state[slot];
    return FinishStep(node, false, "NMT_PREOP_NOT_CONFIRMED");
}

static bool VerifyTransmissionType(uint8_t node)
{
    WheelPdoConfigDiagnostic *d = &nodes[Slot(node)];
    BeginStep(node, "TYPE_READ", WHEEL_RPDO_COMM_INDEX, 2U, WHEEL_RPDO_TYPE);
    if (!Read(node, d->index, 2U, 1U, &d->actual)) {
        return FinishStep(node, false, "TYPE_READ_FAILED");
    }
    if (d->actual == WHEEL_RPDO_TYPE) {
        return FinishStep(node, true, "NONE");
    }
    /* iWMC manual: default 254, immediate RPDO application. Never select 255. */
    return WriteVerify(node, WHEEL_RPDO_COMM_INDEX, 2U, 1U, WHEEL_RPDO_TYPE);
}

static bool ConfigureNode(uint8_t node)
{
    WheelPdoConfigDiagnostic *d = &nodes[Slot(node)];
    uint32_t epoch = d->epoch;
    uint32_t cob = node == LEFT_NODE_ID ? WHEEL_RPDO_LEFT_COB_ID : WHEEL_RPDO_RIGHT_COB_ID;
    bool ok = WriteVerify(node, 0x1017U, 0U, 2U, DRIVE_KINCO_HEARTBEAT_PRODUCER_MS) &&
        PreOperational(node) &&
        WriteVerify(node, WHEEL_RPDO_COMM_INDEX, 1U, 4U, cob | 0x80000000UL) &&
        WriteVerify(node, WHEEL_RPDO_MAP_INDEX, 0U, 1U, 0U) &&
        WriteVerify(node, WHEEL_RPDO_MAP_INDEX, 1U, 4U, WHEEL_RPDO_MAP_ENTRY) &&
        WriteVerify(node, WHEEL_RPDO_MAP_INDEX, 0U, 1U, 1U) &&
        VerifyTransmissionType(node) &&
        WriteVerify(node, WHEEL_RPDO_COMM_INDEX, 1U, 4U, cob);
    if (!ok || !EpochMatches(node, epoch)) { return false; }
    /* Final readback AFTER enabling the PDO, not just individual writes. */
    const uint16_t indices[] = {WHEEL_RPDO_COMM_INDEX, WHEEL_RPDO_COMM_INDEX,
                                WHEEL_RPDO_MAP_INDEX, WHEEL_RPDO_MAP_INDEX};
    const uint8_t subs[] = {1U, 2U, 0U, 1U}, widths[] = {4U, 1U, 1U, 4U};
    const uint32_t expected[] = {cob, WHEEL_RPDO_TYPE, 1U, WHEEL_RPDO_MAP_ENTRY};
    for (uint8_t i = 0U; i < 4U; ++i) {
        BeginStep(node, "FINAL_READ", indices[i], subs[i], expected[i]);
        if (!Read(node, indices[i], subs[i], widths[i], &d->actual)) {
            return FinishStep(node, false, "FINAL_READ_FAILED");
        }
        if (!EpochMatches(node, epoch)) { return false; }
        if (!FinishStep(node, d->actual == expected[i], "FINAL_MISMATCH")) { return false; }
        d->readback[i] = d->actual;
    }
    if (!EpochMatches(node, epoch)) { return false; }
    d->configured = true;
    d->reason = "CONFIGURED_WAIT_OPERATIONAL";
    return true;
}

bool WheelPdoConfig_Configure(void)
{
    failed_node = 0U;
    for (uint8_t slot = 0U; slot < 2U; ++slot) {
        uint32_t epoch = nodes[slot].epoch + 1U;
        memset(&nodes[slot], 0, sizeof(nodes[slot]));
        nodes[slot].epoch = epoch;
        nodes[slot].phase = nodes[slot].reason = "NOT_STARTED";
        JetsonTx_Printf(JETSON_TX_PRIORITY_STATUS,
            "WHEEL_IDENTITY node=%u state=NOT_QUERIED vendor=UNKNOWN product=UNKNOWN revision=UNKNOWN serial=UNKNOWN\r\n",
            (unsigned)(slot ? RIGHT_NODE_ID : LEFT_NODE_ID));
    }
    WheelRpdoTx_Cancel(HAL_GetTick());
    WheelRpdoTx_Service(HAL_GetTick());
    if (!WheelRpdoTx_Quiescent()) {
        BeginStep(LEFT_NODE_ID, "TX_CANCEL", 0U, 0U, 0U);
        nodes[0].actual = hfdcan1.Instance->TXBRP & 3U;
        return FinishStep(LEFT_NODE_ID, false, "TX_NOT_QUIESCENT");
    }
    uint32_t epochs[2] = {nodes[0].epoch, nodes[1].epoch};
    bool ok = ConfigureNode(LEFT_NODE_ID) && ConfigureNode(RIGHT_NODE_ID) &&
        EpochMatches(LEFT_NODE_ID, epochs[0]) && EpochMatches(RIGHT_NODE_ID, epochs[1]);
    if (!ok) { nodes[0].configured = nodes[1].configured = false; }
    /* Existing TPDO startup issues per-wheel NMT START after its readbacks. */
    return ok;
}

void WheelPdoConfig_OnHeartbeat(uint8_t node, uint8_t state, uint32_t now_ms)
{
    int slot = Slot(node);
    if (slot < 0) { return; }
    WheelPdoConfigDiagnostic *d = &nodes[slot];
    heartbeat_ms[slot] = now_ms;
    heartbeat_sequence[slot]++;
    nmt_state[slot] = state;
    if ((state == 0U) || (d->operational && (state != 5U))) {
        bool was_configured = d->configured;
        d->epoch++;
        d->configured = d->operational = false;
        d->reason = state == 0U ? "NODE_BOOTUP" : "NMT_NOT_OPERATIONAL";
        WheelRpdoTx_Cancel(now_ms);
        if (was_configured) {
            Canopen_LatchWheelTpdoStartupFault(d->reason);
        }
    } else if (d->configured && (state == 5U)) {
        d->operational = true;
    }
}

bool WheelPdoConfig_Ready(uint32_t now_ms)
{
    for (uint8_t slot = 0U; slot < 2U; ++slot) {
        if (!nodes[slot].configured || !nodes[slot].operational ||
            ((uint32_t)(now_ms - heartbeat_ms[slot]) > DRIVE_HEARTBEAT_STALE_TIMEOUT_MS)) {
            return false;
        }
    }
    return true;
}

WheelPdoConfigDiagnostic WheelPdoConfig_Get(uint8_t node)
{
    int slot = Slot(node);
    WheelPdoConfigDiagnostic empty = {0};
    return slot < 0 ? empty : nodes[slot];
}
const char *WheelPdoConfig_FailureReason(void)
{
    int slot = Slot(failed_node);
    return slot < 0 ? "RPDO_CONFIG_NOT_RUN" : nodes[slot].reason;
}

#ifdef AMR_HOST_TEST
void WheelPdoConfig_TestReset(void)
{
    memset(nodes, 0, sizeof(nodes));
    memset(heartbeat_ms, 0, sizeof(heartbeat_ms));
    memset(heartbeat_sequence, 0, sizeof(heartbeat_sequence));
    memset(nmt_state, 0, sizeof(nmt_state));
    failed_node = 0U;
}
#endif
