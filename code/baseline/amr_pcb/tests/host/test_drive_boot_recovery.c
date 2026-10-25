/* Executes production Drive/CANopen with adversarial wire responses. The fake
 * bus cannot establish real hardware timing, CAN ACK behavior or motor safety. */
#include "fake_bus.h"
#include "drive_internal.h"
#include "drive_target.h"
#include "wheel_pdo_config.h"
#include "wheel_rpdo_tx.h"
#include "app_config.h"
#include "lift.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t boot_started, offline_until, first_second_ms;
static unsigned offline_mask, status_drop_mask, abort_node, malformed_node;
static bool forever_offline, drop_first_only, inject_delayed;
static unsigned status_requests[3][3], disable_requests[3][3];
static unsigned second_seen, motion_writes, reset_writes, stale_injections;
static unsigned delayed_remaining[3];
static unsigned ack_fault_node, ack_fault_kind;
static uint32_t response_delay_ms;
static struct { bool active; uint32_t due; uint8_t data[8]; } delayed[3];

static unsigned Attempt(void)
{
    return s_drive.boot.state == DRIVE_BOOT_SECOND_ATTEMPT ? 2U : 1U;
}

static bool Offline(unsigned node)
{
    return (offline_mask & (1U << node)) &&
        (forever_offline || (uint32_t)(tick - boot_started) < offline_until);
}

static bool InjectSdo(uint8_t node, uint16_t index, uint8_t sub,
                      const uint8_t request[8])
{
    unsigned attempt = Attempt();
    bool read = request[0] == 0x40;
    uint32_t value = (uint32_t)request[4] | ((uint32_t)request[5] << 8) |
                     ((uint32_t)request[6] << 16) | ((uint32_t)request[7] << 24);
    if (node > 2) return false;
    if (!read && index == 0x6040) {
        if (value == 6) disable_requests[attempt][node]++;
        if (value == 0x0f || value == 0x2f) motion_writes++;
        if ((value & 0x80U) != 0) reset_writes++;
    }
    if (!read && index == 0x60ff && value != 0) motion_writes++;
    if (read && index == 0x6041) status_requests[attempt][node]++;
    if (Offline(node)) return true;
    if (ack_fault_node == node && !read && index == 0x6040) {
        uint8_t response[8] = {0x60, request[1], request[2], sub, 0, 0, 0, 0};
        uint8_t length = 8;
        if (ack_fault_kind <= 3) {
            static const uint8_t short_lengths[] = {0, 4, 5, 7};
            length = short_lengths[ack_fault_kind];
        } else if (ack_fault_kind == 4) response[0] = 0x61;
        else if (ack_fault_kind == 5) response[1] ^= 2;
        else response[3] ^= 1;
        Fake_Rx(0x580U + node, response, length);
        return true;
    }
    if (abort_node == node && index == 0x6040) {
        uint8_t response[8] = {0x80, request[1], request[2], sub, 0x11, 0, 9, 6};
        Fake_Rx(0x580U + node, response, 8);
        return true;
    }
    if (malformed_node == node && read && index == 0x6041) {
        uint8_t response[8] = {0x4f, 0x41, 0x60, sub, 0x40, 0, 0, 0};
        Fake_Rx(0x580U + node, response, 8);
        return true;
    }
    if (inject_delayed && attempt == 2 && read && index == 0x6041 &&
        (status_drop_mask & (1U << node))) {
        /* Deliver an old first-attempt response while the same node/object is
         * active in attempt two. An epoch counter alone accepts this frame. */
        if (delayed_remaining[node] != 0) {
            uint8_t response[8] = {0x4b, 0x41, 0x60, 0, 0x40, 0, 0, 0};
            Fake_Rx(0x580U + node, response, 8);
            delayed_remaining[node]--;
            stale_injections++;
        }
        return true;
    }
    if (read && index == 0x6041 && (status_drop_mask & (1U << node)) &&
        (!drop_first_only || attempt == 1)) return true;
    if (response_delay_ms && (index == 0x6040 || index == 0x6041)) {
        assert(!delayed[node].active);
        delayed[node].active = true;
        delayed[node].due = tick + response_delay_ms;
        uint8_t response[8] = {read ? 0x4b : 0x60, request[1], request[2], sub,
                              read ? (uint8_t)wheel_statusword[node] : 0,
                              read ? (uint8_t)(wheel_statusword[node] >> 8) : 0, 0, 0};
        memcpy(delayed[node].data, response, sizeof(response));
        return true;
    }
    return false;
}

static void ResetAt(uint32_t start)
{
    Fake_Reset();
    Drive_TestResetState();
    Drive_SetLiftMovingCallback(NULL);
    tick = boot_started = start;
    tick_step = 0;
    offline_until = first_second_ms = 0;
    offline_mask = status_drop_mask = abort_node = malformed_node = 0;
    forever_offline = drop_first_only = inject_delayed = false;
    second_seen = motion_writes = reset_writes = stale_injections = 0;
    memset(status_requests, 0, sizeof(status_requests));
    memset(disable_requests, 0, sizeof(disable_requests));
    memset(delayed_remaining, 0, sizeof(delayed_remaining));
    ack_fault_node = ack_fault_kind = 0;
    memset(delayed, 0, sizeof(delayed));
    response_delay_ms = 0;
    fake_sdo_hook = InjectSdo;
}

static void Inhibited(void)
{
    assert(!s_drive.core.enabled && !s_hardware_enable_confirmed);
    assert(!Drive_NormalMotionAllowed());
    assert(!DriveTarget_Pending());
    assert(WheelRpdoTx_Get().queued_pairs == 0);
    assert(motion_writes == 0 && reset_writes == 0);
    assert(enable_controlwords == 0 && reset_controlwords == 0);
}

static void Step(uint32_t elapsed, bool tpdo)
{
    tick += elapsed;
    uint32_t before = tick;
    for (unsigned node = 1; node <= 2; node++) {
        if (delayed[node].active && (int32_t)(tick - delayed[node].due) >= 0) {
            Fake_Rx(0x580U + node, delayed[node].data, 8);
            delayed[node].active = false;
        }
    }
    if (!bus_off) {
        if (!Offline(1)) Fake_Heartbeat(1, 5);
        if (!Offline(2)) Fake_Heartbeat(2, 5);
        if (tpdo && !Offline(1) && !Offline(2)) Fake_Tpdo(0, 0);
    }
    Drive_ControlLoop_Update();
    /* The scheduler is deterministic; HAL_Delay would change this clock. */
    assert(tick == before);
    if (s_drive.boot.state == DRIVE_BOOT_SECOND_ATTEMPT && !second_seen) {
        second_seen = 1;
        first_second_ms = tick;
    }
    Inhibited();
    if (s_drive.boot.state == DRIVE_BOOT_WAIT_RETRY) {
        assert(s_drive.core.state == DRIVE_STATE_INIT);
        assert(!s_hardware_disable_confirmed);
        assert(!Drive_TestZeroEvidenceValid());
        assert(!Drive_TestBaseHardwareSafeForLift(tick));
    }
}

static void UntilWait(void)
{
    for (unsigned i = 0; i < 400 && s_drive.boot.state != DRIVE_BOOT_WAIT_RETRY; i++) {
        Step(10, false);
        assert(s_drive.core.state != DRIVE_STATE_FAULT);
    }
    assert(s_drive.boot.state == DRIVE_BOOT_WAIT_RETRY);
}

static void UntilTerminal(void)
{
    for (unsigned i = 0; i < 1200 &&
         s_drive.boot.state != DRIVE_BOOT_READY &&
         s_drive.boot.state != DRIVE_BOOT_FAILED; i++) Step(10, false);
    assert(s_drive.boot.state == DRIVE_BOOT_READY ||
           s_drive.boot.state == DRIVE_BOOT_FAILED);
}

static void Ready(unsigned expected_attempt)
{
    if (s_drive.boot.state != DRIVE_BOOT_READY) {
        printf("BOOT FAILURE expected=%u state=%u first_reads=%u/%u second_reads=%u/%u elapsed=%lu\n",
            expected_attempt, (unsigned)s_drive.boot.state,
            status_requests[1][1], status_requests[1][2],
            status_requests[2][1], status_requests[2][2],
            (unsigned long)(tick - boot_started));
        puts(Fake_Log());
    }
    assert(s_drive.boot.state == DRIVE_BOOT_READY);
    assert(s_drive.core.state == DRIVE_STATE_DISABLED);
    assert(s_hardware_disable_confirmed);
    assert(s_drive.boot.retries == expected_attempt - 1);
    assert(status_requests[expected_attempt][1] > 0);
    assert(status_requests[expected_attempt][2] > 0);
    assert(disable_requests[expected_attempt][1] > 0);
    assert(disable_requests[expected_attempt][2] > 0);
    assert(s_drive.statusword.left_statusword_valid);
    assert(s_drive.statusword.right_statusword_valid);
    assert(s_drive.statusword.left_statusword_generation == s_drive.evidence.safety_generation);
    assert(s_drive.statusword.right_statusword_generation == s_drive.evidence.safety_generation);
    assert(!s_drive.command.jetson_cmd_active && !s_drive.control.new_cmd_vel_pending);
    assert(s_drive.control.target_linear_mm_s == 0 && s_drive.control.target_angular_mrad_s == 0);
    assert(!Drive_TestZeroEvidenceValid());
    assert(!Drive_TestBaseHardwareSafeForLift(tick));
    Inhibited();
}

static void Failed(unsigned retries)
{
    assert(s_drive.boot.state == DRIVE_BOOT_FAILED);
    assert(s_drive.core.state == DRIVE_STATE_FAULT);
    assert(s_drive.boot.retries == retries);
    assert(s_tracking_stop_latched && s_disable_motion_inhibit_latched);
    Inhibited();
}

static void TestImmediate(void)
{
    ResetAt(0);
    UntilTerminal();
    Ready(1);
    assert(tick < DRIVE_BOOT_RETRY_DELAY_MS && !second_seen);
    assert(strstr(Fake_Log(), "DRIVE_BOOT"));
    puts("PASS BOOT 01: immediately ready completes first attempt without 5-second delay");
}

static void TestDelayed(unsigned mask, unsigned number)
{
    ResetAt(0);
    offline_mask = mask;
    offline_until = 3000;
    UntilWait();
    uint32_t wait_start = s_drive.boot.retry_started_ms;
    UntilTerminal();
    Ready(2);
    assert(second_seen && (uint32_t)(first_second_ms - wait_start) >= 5000);
    assert((uint32_t)(first_second_ms - wait_start) < 5010);
    printf("PASS BOOT %02u: offline mask=%u ready at 3 seconds, one full bilateral retry succeeds\n", number, mask);
}

static void TestSecondAttemptOnly(void)
{
    ResetAt(0);
    offline_mask = 6;
    forever_offline = true;
    UntilWait();
    uint32_t start = s_drive.boot.retry_started_ms;
    tick = start + 4999;
    Step(0, false);
    assert(s_drive.boot.state == DRIVE_BOOT_WAIT_RETRY);
    forever_offline = false;
    Step(1, false);
    assert(s_drive.boot.state == DRIVE_BOOT_SECOND_ATTEMPT);
    UntilTerminal();
    Ready(2);
    puts("PASS BOOT 04: both drives first respond at second-attempt boundary; exact 5000 ms wait");
}

static void TestPermanentOffline(void)
{
    ResetAt(0);
    offline_mask = 6;
    forever_offline = true;
    UntilTerminal();
    Failed(1);
    /* Finish cleanup, then show later scheduler calls cannot create a third attempt. */
    for (unsigned i = 0; i < 1000; i++) Step(10, false);
    unsigned count = disable_requests[1][1] + disable_requests[1][2] +
                     disable_requests[2][1] + disable_requests[2][2];
    for (unsigned i = 0; i < 6000; i++) Step(10, false);
    Failed(1);
    assert(disable_requests[1][1] + disable_requests[1][2] +
           disable_requests[2][1] + disable_requests[2][2] == count);
    puts("PASS BOOT 05: permanently offline latches second failure; no third retry for 60 seconds");
}

static void TestHardFaults(void)
{
    for (unsigned node = 1; node <= 2; node++) {
        ResetAt(0); abort_node = node; UntilTerminal(); Failed(0);
        assert(!second_seen);
    }
    puts("PASS BOOT 06: either-node SDO Abort remains a hard fault without retry");
    for (unsigned node = 1; node <= 2; node++) {
        ResetAt(0); wheel_statusword[node] = 8; UntilTerminal(); Failed(0);
        ResetAt(0);
        uint8_t emcy[8] = {0x10, 0x23, 1, 0, 0, 0, 0, 0};
        Fake_Rx(0x80U + node, emcy, 8); UntilTerminal(); Failed(0);
        ResetAt(0); offline_mask = 6; forever_offline = true; UntilWait();
        Fake_Rx(0x80U + node, emcy, 8); UntilTerminal(); Failed(0);
    }
    puts("PASS BOOT 07: DS402 Fault and EMCY before initialization or during retry wait cannot recover");
    ResetAt(0); bus_off = 1; UntilTerminal(); Failed(0);
    ResetAt(0); offline_mask = 6; forever_offline = true; UntilWait();
    bus_off = 1; UntilTerminal(); Failed(0);
    puts("PASS BOOT 08: BUS_OFF before startup and during wait is never an ordinary timeout");
}

static void TestMissingStatus(void)
{
    for (unsigned node = 1; node <= 2; node++) {
        ResetAt(0); status_drop_mask = 1U << node; drop_first_only = true;
        UntilWait();
        assert(disable_requests[1][1] && disable_requests[1][2]);
        assert(!s_hardware_disable_confirmed);
        assert(status_requests[1][node] > 0 && !second_seen);
        UntilTerminal(); Ready(2);
        ResetAt(0); status_drop_mask = 1U << node;
        UntilTerminal(); Failed(1);
    }
    puts("PASS BOOT 09: write ACK alone never confirms disabled; unavailable status retries once then latches on second failure");
}

static void TestLateHardEvidence(void)
{
    for (unsigned node = 1; node <= 2; node++) {
        for (unsigned is_abort = 0; is_abort <= 1; is_abort++) {
            for (unsigned boundary = 0; boundary <= 1; boundary++) {
                ResetAt(0);
                offline_mask = 1U << node; forever_offline = true;
                UntilWait();
                assert(!s_drive.safety.transition.active);
                assert(!s_drive.safety.transition.request_pending);
                /* The timed-out transaction has been retired. Exercise both
                 * idle wait and the exact retry deadline with an old frame. */
                tick = s_drive.boot.retry_started_ms + (boundary ? 5000U : 200U);
                uint8_t response[8] = {0x4b, 0x41, 0x60, 0, 8, 0, 0, 0};
                if (is_abort) {
                    response[0] = 0x80;
                    response[4] = 0x11; response[5] = 0;
                    response[6] = 9; response[7] = 6;
                }
                Fake_Rx(0x580U + node, response, 8);
                Step(0, false);
                Failed(0);
                assert(!second_seen);
                assert(disable_requests[2][1] == 0 && disable_requests[2][2] == 0);
                CanopenBootSdoEvidence_t evidence = Canopen_GetBootSdoEvidence();
                assert(evidence.node_id == node);
                assert(evidence.hard_fault_reason == (is_abort ?
                    CANOPEN_BOOT_SDO_FAULT_ABORT : CANOPEN_BOOT_SDO_FAULT_STATUS));
            }
        }
    }
    puts("PASS BOOT late hard evidence: retired-transaction Abort/6041 Fault on either node fails during WAIT and at 5000 ms without second attempt");
}

static void TestCommandsAndWrap(uint32_t start, unsigned number)
{
    ResetAt(start); offline_mask = 6; forever_offline = true; UntilWait();
    uint32_t wait_start = s_drive.boot.retry_started_ms;
    for (unsigned i = 0; i < 100; i++) {
        assert(Drive_CommandEnableChecked() != CMD_RESULT_ACCEPTED);
        Drive_CommandSetCmdVel(200, 100);
        Drive_CommandStop();
        (void)Drive_CommandDisableChecked();
        Step(40, false);
        assert(s_drive.boot.state == DRIVE_BOOT_WAIT_RETRY);
        assert(s_drive.boot.retry_started_ms == wait_start);
        assert(!s_drive.command.jetson_cmd_active && !s_drive.control.new_cmd_vel_pending);
    }
    tick = wait_start + 4999;
    Step(0, false);
    assert(s_drive.boot.state == DRIVE_BOOT_WAIT_RETRY);
    forever_offline = false;
    Step(1, false);
    assert(s_drive.boot.state == DRIVE_BOOT_SECOND_ATTEMPT);
    UntilTerminal(); Ready(2);
    assert(Drive_CommandEnableChecked() != CMD_RESULT_ACCEPTED);
    for (unsigned i = 0; i < 40; i++) Step(10, true);
    assert(s_drive.core.state == DRIVE_STATE_DISABLED && !s_drive.command.jetson_cmd_active);
    assert(Canopen_GetWheelFeedbackAuthority() == CANOPEN_WHEEL_AUTHORITY_TPDO);
    assert(Canopen_GetWheelFeedbackTransport() == CANOPEN_WHEEL_FEEDBACK_TPDO_ACTIVE);
    assert(!DriveTarget_Pending() && WheelRpdoTx_Get().queued_pairs == 0);
    if (number == 13) puts("PASS BOOT 13: retry timer remains exact across HAL_GetTick wraparound");
    else {
        puts("PASS BOOT 11: repeated ENABLE/CMD_VEL cannot move or replay after recovered TPDO startup");
        puts("PASS BOOT 12: repeated STOP/DISABLE neither restart initialization nor postpone retry");
        puts("PASS BOOT 15: unavailable feedback never grants Lift zero proof; fresh TPDO authority still required");
    }
}

static void TestDelayedOldResponses(void)
{
    /* Frames already in FIFO at retry entry cannot be promoted into evidence.
     * The independent Lift SDO channel remains usable during the wheel fence. */
    ResetAt(0); offline_mask = 6; forever_offline = true; UntilWait();
    assert(!SDO_Read_U16(1, 0x6041, 0, &wheel_statusword[0]));
    uint16_t lift_status = 0;
    tick_step = 1;
    assert(SDO_Read_U16(3, 0x6041, 0, &lift_status));
    tick_step = 0;
    assert(lift_status == 0x40);
    uint8_t status[8] = {0x4b, 0x41, 0x60, 0, 0x40, 0, 0, 0};
    uint8_t ack[8] = {0x60, 0x40, 0x60, 0, 0, 0, 0, 0};
    Fake_Rx(0x581, status, 8); Fake_Rx(0x582, status, 8);
    Fake_Rx(0x581, ack, 8); Fake_Rx(0x582, ack, 8);
    tick = s_drive.boot.retry_started_ms + 5000;
    Step(0, false);
    assert(!s_drive.statusword.left_statusword_valid);
    assert(!s_drive.statusword.right_statusword_valid);
    UntilTerminal(); Failed(1);
    puts("PASS BOOT FIFO: buffered stale frames rejected and Lift SDO preserved during wheel-only fence");
}

static void TestRuntimeAndInvalidStatus(void)
{
    TestDriveReady();
    uint32_t retries = s_drive.boot.retries;
    tick += DRIVE_WHEEL_FEEDBACK_STALE_MS + 1;
    Drive_ControlLoop_Update();
    assert(s_drive.core.state == DRIVE_STATE_FAULT);
    assert(s_drive.boot.state != DRIVE_BOOT_WAIT_RETRY);
    assert(s_drive.boot.retries == retries);
    puts("PASS BOOT 14: established runtime feedback loss retains original latched-fault behavior");
    for (unsigned node = 1; node <= 2; node++) {
        ResetAt(0); wheel_statusword[node] = 0x27; UntilTerminal(); Failed(0);
        ResetAt(0); wheel_statusword[node] = 0; UntilTerminal(); Failed(0);
        ResetAt(0); malformed_node = node; UntilTerminal(); Failed(0);
    }
    puts("PASS BOOT extra: responsive enabled state and malformed status response never qualify for retry");
}

static void TestResponseLatency(void)
{
    for (unsigned latency = 10; latency <= 15; latency += 5) {
        ResetAt(0); response_delay_ms = latency;
        UntilTerminal(); Ready(1);
        for (unsigned node = 1; node <= 2; node++) {
            ResetAt(0); response_delay_ms = latency;
            offline_mask = 1U << node; offline_until = 3000;
            UntilTerminal(); Ready(2);
            assert(status_requests[1][1] == 0 && status_requests[1][2] == 0);
            ResetAt(0); response_delay_ms = latency;
            status_drop_mask = 1U << node; drop_first_only = true;
            UntilTerminal(); Ready(2);
        }
    }
    puts("PASS BOOT latency: 10/15 ms actual response delay with 10 ms loop; either-node initial write or status-read timeout recovers once");
}

static void TestDs402DisabledStates(void)
{
    /* DS402 switch-on-disabled uses mask 0x004f: bit 5 is unspecified.
     * Ready-to-switch-on and switched-on use mask 0x006f. */
    static const uint16_t allowed[] = {0x0040, 0x0060, 0x0021, 0x0023};
    static const uint16_t rejected[] = {0x0068, 0x0064};
    for (unsigned node = 1; node <= 2; node++) {
        for (unsigned i = 0; i < sizeof(allowed) / sizeof(allowed[0]); i++) {
            ResetAt(0); wheel_statusword[node] = allowed[i];
            UntilTerminal(); Ready(1);
            ResetAt(0); wheel_statusword[node] = allowed[i];
            offline_mask = 1U << node; offline_until = 3000;
            UntilTerminal(); Ready(2);
        }
        for (unsigned i = 0; i < sizeof(rejected) / sizeof(rejected[0]); i++) {
            ResetAt(0); wheel_statusword[node] = rejected[i];
            UntilTerminal(); Failed(0);
        }
    }
    puts("PASS BOOT DS402: either node 0040/0060 switch-on-disabled, 0021 ready and 0023 switched-on qualify in both attempts; 0068 fault and 0064 invalid never qualify");
}

static void TestReviewReplay(void)
{
    bool limitation_demonstrated = true;
    for (unsigned node = 1; node <= 2; node++) {
        for (unsigned extra = 1; extra <= 4; extra += 3) {
            ResetAt(0); status_drop_mask = 1U << node;
            for (unsigned i = 0; i < 400 &&
                 s_drive.boot.state != DRIVE_BOOT_WAIT_RETRY &&
                 s_drive.boot.state != DRIVE_BOOT_FAILED; i++) Step(10, false);
            if (s_drive.boot.state == DRIVE_BOOT_WAIT_RETRY) {
                /* Repeated delivery of the old upload exceeds the request
                 * count. N+1 is not a wire-level freshness guarantee. */
                delayed_remaining[node] = status_requests[1][node] + extra;
                inject_delayed = true;
                UntilTerminal();
            } else {
                uint8_t old[8] = {0x4b, 0x41, 0x60, 0, 0x40, 0, 0, 0};
                for (unsigned i = 0; i < status_requests[1][node] + extra; i++) {
                    Fake_Rx(0x580U + node, old, 8);
                    stale_injections++;
                }
                Step(10, false);
            }
            printf("KNOWN LIMITATION replay node=%u old_extra=%u state=%u retries=%u injections=%u hw_disabled=%u enabled=%u\n",
                node, extra, (unsigned)s_drive.boot.state,
                (unsigned)s_drive.boot.retries, stale_injections,
                s_hardware_disable_confirmed ? 1U : 0U, s_drive.core.enabled ? 1U : 0U);
            limitation_demonstrated = limitation_demonstrated &&
                s_drive.boot.state == DRIVE_BOOT_READY &&
                s_drive.boot.retries == 1 && second_seen &&
                s_hardware_disable_confirmed && stale_injections > 0;
            Inhibited();
        }
    }
    assert(limitation_demonstrated);
    puts("KNOWN LIMITATION: standard SDO same-node/object replay during an active retry read can produce READY and hardware-disabled confirmation; no automatic enable or motion occurred. Software tests do not resolve or prove safety against this replay risk.");
}

static void TestReviewEmcyDrain(void)
{
    bool safe = true;
    static const unsigned prefixes[] = {0, 7, 8};
    for (unsigned node = 1; node <= 2; node++) {
        for (unsigned n = 0; n < sizeof(prefixes) / sizeof(prefixes[0]); n++) {
          for (unsigned deadline = 0; deadline <= 1; deadline++) {
            ResetAt(0); offline_mask = 6; forever_offline = true; UntilWait();
            uint8_t noise[8] = {0};
            uint8_t fault[8] = {0x10, 0x23, 1, 0, 0, 0, 0, 0};
            for (unsigned i = 0; i < prefixes[n]; i++) Fake_Rx(0x777, noise, 8);
            Fake_Rx(0x80U + node, fault, 8);
            Fake_Rx(0x80U + node, noise, 8);
            tick = s_drive.boot.retry_started_ms + (deadline ? 5000U : 200U);
            Step(0, false);
            printf("REVIEW EMCY node=%u prefix=%u deadline=%u state=%u retries=%u second_writes=%u/%u\n",
                node, prefixes[n], deadline, (unsigned)s_drive.boot.state,
                (unsigned)s_drive.boot.retries, disable_requests[2][1], disable_requests[2][2]);
            safe = safe && (!deadline || s_drive.boot.state == DRIVE_BOOT_FAILED) &&
                s_drive.boot.retries == 0 && !second_seen &&
                disable_requests[2][1] == 0 && disable_requests[2][2] == 0 &&
                !strstr(Fake_Log(), "DRIVE_BOOT attempt=2 state=START");
            Step(10, false);
            safe = safe && s_drive.boot.state == DRIVE_BOOT_FAILED &&
                !strstr(Fake_Log(), "DRIVE_BOOT attempt=2 state=START");
          }
        }
    }
    assert(safe);
    puts("PASS REVIEW software-queue model (128 slots): EMCY/Clear survives same-batch and RX-budget split; this does not model hardware FIFO overflow");
}

static void TestReviewMalformedAck(void)
{
    bool safe = true;
    for (unsigned node = 1; node <= 2; node++) {
        for (unsigned kind = 1; kind <= 6; kind++) {
            ResetAt(0); ack_fault_node = node; ack_fault_kind = kind;
            UntilTerminal();
            printf("REVIEW boot ACK node=%u kind=%u state=%u retries=%u\n",
                node, kind, (unsigned)s_drive.boot.state, (unsigned)s_drive.boot.retries);
            safe = safe && s_drive.boot.state == DRIVE_BOOT_FAILED;
            if (kind <= 3) {
                safe = safe && s_drive.boot.retries == 0 && !second_seen &&
                    status_requests[1][1] == 0 && status_requests[1][2] == 0 &&
                    status_requests[2][1] == 0 && status_requests[2][2] == 0 &&
                    !strstr(Fake_Log(), "DRIVE_BOOT attempt=2 state=START");
            }
            /* Runtime async and blocking writers use the real SDO decoder. */
            ResetAt(0); ack_fault_node = node; ack_fault_kind = kind;
            assert(Canopen_AsyncWriteU16(node, 0x6040, 0, 6,
                CANOPEN_SDO_OWNER_SAFETY_CONFIRM, tick, 20) == CANOPEN_ASYNC_START_OK);
            Canopen_WheelRuntimeService(tick, CANOPEN_FEEDBACK_DISPATCH_ONLY);
            tick += 30;
            Canopen_WheelRuntimeService(tick, CANOPEN_FEEDBACK_DISPATCH_ONLY);
            uint32_t value = 0, abort_code = 0;
            CanopenAsyncResult_t result = Canopen_AsyncPoll(node,
                CANOPEN_SDO_OWNER_SAFETY_CONFIRM, tick, &value, &abort_code);
            printf("REVIEW runtime ACK node=%u kind=%u async=%u\n", node, kind, (unsigned)result);
            safe = safe && result != CANOPEN_ASYNC_RESULT_SUCCESS;
            ResetAt(0); ack_fault_node = node; ack_fault_kind = kind; tick_step = 1;
            bool success = SDO_Write_U16(node, 0x6040, 0, 6);
            printf("REVIEW blocking ACK node=%u kind=%u success=%u\n", node, kind, success ? 1U : 0U);
            safe = safe && !success;
        }
    }
    assert(safe);
    puts("PASS REVIEW: 6040 ACK lengths 4/5/7 and wrong CS/index/sub cannot succeed in boot, async or blocking runtime");
}

static void TestDiagnosticAndPollingCompatibility(void)
{
    for (unsigned node = 1; node <= 2; node++) {
        ResetAt(0);
        Canopen_BootSdoBegin();
        tick_step = 1;
        uint16_t diagnostic_status = 0;
        assert(SDO_Read_U16(node, 0x6041, 0, &diagnostic_status));
        tick_step = 0;
        assert(diagnostic_status == 0x40);
        offline_mask = 6; forever_offline = true;
        UntilWait();
        forever_offline = false;
        UntilTerminal(); Ready(2);

        for (unsigned attempt = 1; attempt <= 2; attempt++) {
            ResetAt(0);
            if (attempt == 2) {
                offline_mask = 6; forever_offline = true; UntilWait();
                forever_offline = false;
            }
            wheel_statusword[node] = 0x27;
            for (unsigned i = 0; i < 800 && status_requests[attempt][node] == 0; i++)
                Step(10, false);
            assert(status_requests[attempt][node] > 0);
            for (unsigned i = 0; i < 10; i++) Step(10, false);
            assert(s_drive.boot.state != DRIVE_BOOT_FAILED);
            wheel_statusword[node] = 0x40;
            UntilTerminal(); Ready(attempt);
            assert(status_requests[attempt][node] > 1);
        }
    }
    puts("PASS V1 compatibility: prior diagnostic 6041 does not disable the single retry; valid non-disabled status polls until the existing deadline in either attempt");
}

static uint8_t generic_sdo_width, generic_sdo_dlc;

static bool InjectGenericSdo(uint8_t node, uint16_t index, uint8_t sub,
                             const uint8_t request[8])
{
    bool read = request[0] == 0x40;
    uint8_t response[8] = {read ? (generic_sdo_width == 1 ? 0x4f : 0x4b) : 0x60,
                          (uint8_t)index, (uint8_t)(index >> 8), sub,
                          0x34, 0x12, 0, 0};
    Fake_Rx(0x580U + node, response, generic_sdo_dlc);
    return true;
}

static void TestGenericSdoDlc8(void)
{
    static const uint8_t read_lengths[] = {5, 6, 7, 8};
    static const uint8_t write_lengths[] = {4, 5, 7, 8};
    unsigned cases = 0;
    for (unsigned node = 1; node <= 3; node++) {
        for (unsigned width = 1; width <= 2; width++) {
            for (unsigned i = 0; i < sizeof(read_lengths); i++) {
                uint8_t dlc = read_lengths[i];
                ResetAt(0); fake_sdo_hook = InjectGenericSdo;
                generic_sdo_width = (uint8_t)width; generic_sdo_dlc = dlc;
                CanopenAsyncStartResult_t started = width == 1 ?
                    Canopen_AsyncReadU8((uint8_t)node, 0x2001, 0,
                        CANOPEN_SDO_OWNER_SAFETY_CONFIRM, tick, 20) :
                    Canopen_AsyncReadU16((uint8_t)node, 0x2001, 0,
                        CANOPEN_SDO_OWNER_SAFETY_CONFIRM, tick, 20);
                assert(started == CANOPEN_ASYNC_START_OK);
                Canopen_WheelRuntimeService(tick, CANOPEN_FEEDBACK_DISPATCH_ONLY);
                tick += 30;
                Canopen_WheelRuntimeService(tick, CANOPEN_FEEDBACK_DISPATCH_ONLY);
                uint32_t value = 0, abort_code = 0;
                CanopenAsyncResult_t result = Canopen_AsyncPoll((uint8_t)node,
                    CANOPEN_SDO_OWNER_SAFETY_CONFIRM, tick, &value, &abort_code);
                assert((result == CANOPEN_ASYNC_RESULT_SUCCESS) == (dlc == 8));
                if (dlc == 8) assert(value == (width == 1 ? 0x34U : 0x1234U));
                ResetAt(0); fake_sdo_hook = InjectGenericSdo; tick_step = 1;
                int8_t value8 = 0; uint16_t value16 = 0;
                bool success = width == 1 ?
                    SDO_Read_I8((uint8_t)node, 0x2001, 0, &value8) :
                    SDO_Read_U16((uint8_t)node, 0x2001, 0, &value16);
                assert(success == (dlc == 8));
                assert(s_last_sdo_response.valid == success);
                if (success) assert(width == 1 ? value8 == 0x34 : value16 == 0x1234);
                cases += 2;
            }
        }
        for (unsigned i = 0; i < sizeof(write_lengths); i++) {
            uint8_t dlc = write_lengths[i];
            ResetAt(0); fake_sdo_hook = InjectGenericSdo; generic_sdo_dlc = dlc;
            assert(Canopen_AsyncWriteU16((uint8_t)node, 0x2002, 0, 7,
                CANOPEN_SDO_OWNER_SAFETY_CONFIRM, tick, 20) == CANOPEN_ASYNC_START_OK);
            Canopen_WheelRuntimeService(tick, CANOPEN_FEEDBACK_DISPATCH_ONLY);
            tick += 30;
            Canopen_WheelRuntimeService(tick, CANOPEN_FEEDBACK_DISPATCH_ONLY);
            uint32_t value = 0, abort_code = 0;
            CanopenAsyncResult_t result = Canopen_AsyncPoll((uint8_t)node,
                CANOPEN_SDO_OWNER_SAFETY_CONFIRM, tick, &value, &abort_code);
            assert((result == CANOPEN_ASYNC_RESULT_SUCCESS) == (dlc == 8));
            ResetAt(0); fake_sdo_hook = InjectGenericSdo; tick_step = 1;
            bool success = SDO_Write_U16((uint8_t)node, 0x2002, 0, 7);
            assert(success == (dlc == 8));
            assert(s_last_sdo_response.valid == success);
            cases += 2;
        }
    }
    printf("PASS generic SDO DLC8: %u async/blocking read/write cases across Nodes 1/2/3; short 8-bit/16-bit uploads and non-6040 ACKs rejected, valid DLC8 accepted\n", cases);
}

static uint8_t oversize_node;

static bool InjectOversizeSdo(uint8_t node, uint16_t index, uint8_t sub,
                              const uint8_t request[8])
{
    if (node != oversize_node) return false;
    uint8_t response[12] = {request[0] == 0x40 ? 0x4b : 0x60,
                           (uint8_t)index, (uint8_t)(index >> 8), sub,
                           0x40, 0, 0, 0, 0, 0, 0, 0};
    /* DLC encoding 9 means 12 payload bytes. The fake HAL really copies 12
     * bytes, exercising the receive buffer as well as unsupported-DLC rejection. */
    Fake_RxWithDlc(0x580U + node, response, sizeof(response), 9);
    return true;
}

static void TestUnsupportedSdoDlc(void)
{
    bool rejected = true;
    for (unsigned node = 1; node <= 3; node++) {
        for (unsigned read = 0; read <= 1; read++) {
            ResetAt(0); fake_sdo_hook = InjectOversizeSdo; oversize_node = (uint8_t)node;
            CanopenAsyncStartResult_t started = read ?
                Canopen_AsyncReadU16((uint8_t)node, 0x2001, 0,
                    CANOPEN_SDO_OWNER_SAFETY_CONFIRM, tick, 20) :
                Canopen_AsyncWriteU16((uint8_t)node, 0x2002, 0, 7,
                    CANOPEN_SDO_OWNER_SAFETY_CONFIRM, tick, 20);
            assert(started == CANOPEN_ASYNC_START_OK);
            Canopen_WheelRuntimeService(tick, CANOPEN_FEEDBACK_DISPATCH_ONLY);
            tick += 30;
            Canopen_WheelRuntimeService(tick, CANOPEN_FEEDBACK_DISPATCH_ONLY);
            uint32_t value = 0, abort_code = 0;
            CanopenAsyncResult_t result = Canopen_AsyncPoll((uint8_t)node,
                CANOPEN_SDO_OWNER_SAFETY_CONFIRM, tick, &value, &abort_code);
            printf("SDO oversized DLC node=%u read=%u async=%u\n", node, read, (unsigned)result);
            rejected = rejected && result != CANOPEN_ASYNC_RESULT_SUCCESS;
        }
        ResetAt(0); fake_sdo_hook = InjectOversizeSdo; oversize_node = (uint8_t)node;
        tick_step = 1; uint16_t value = 0;
        bool success = SDO_Read_U16((uint8_t)node, 0x2001, 0, &value);
        rejected = rejected && !success && !s_last_sdo_response.valid;
    }
    for (unsigned node = 1; node <= 2; node++) {
        ResetAt(0); fake_sdo_hook = InjectOversizeSdo; oversize_node = (uint8_t)node;
        UntilTerminal();
        printf("SDO oversized DLC boot node=%u state=%u\n", node, (unsigned)s_drive.boot.state);
        rejected = rejected && s_drive.boot.state == DRIVE_BOOT_FAILED &&
            !s_hardware_disable_confirmed;
    }
    assert(rejected);
    puts("PASS SDO oversized DLC: raw DLC=9 with 12-byte HAL copy cannot complete async/blocking SDO or confirm boot READY");
}

static void PrepareCachedStatus(unsigned attempt, unsigned node, uint32_t start)
{
    ResetAt(start);
    if (attempt == 2) {
        offline_mask = 6; forever_offline = true; UntilWait();
        forever_offline = false;
    }
    for (unsigned i = 0; i < 1200 &&
         !(s_drive.safety.transition.phase == DRIVE_SAFETY_PHASE_DISABLE_STATUS_READ_LEFT &&
           s_drive.safety.transition.request_pending); i++) Step(10, false);
    assert(s_drive.safety.transition.phase == DRIVE_SAFETY_PHASE_DISABLE_STATUS_READ_LEFT);
    assert(s_drive.safety.transition.request_pending);
    if (node == 2) {
        /* Keep both nodes' legitimate RX times equal so the right-node
         * publication boundary is not obscured by an older left sample. */
        Canopen_ProcessRx(tick, 8);
        Step(0, false); Step(0, false);
        assert(s_drive.safety.transition.phase == DRIVE_SAFETY_PHASE_DISABLE_STATUS_READ_RIGHT);
        assert(s_drive.safety.transition.request_pending);
    }
}

static uint32_t StatusTimestamp(unsigned node)
{
    return node == 1 ? s_drive.statusword.left_statusword_timestamp_ms :
                       s_drive.statusword.right_statusword_timestamp_ms;
}

static void TestCachedStatusAge(void)
{
    static const uint32_t ages[] = {149, 150, 151, 200};
    unsigned cases = 0;
    for (unsigned attempt = 1; attempt <= 2; attempt++)
      for (unsigned node = 1; node <= 2; node++)
       for (unsigned wrap = 0; wrap <= 1; wrap++)
        for (unsigned a = 0; a < sizeof(ages) / sizeof(ages[0]); a++) {
            PrepareCachedStatus(attempt, node, 0);
            if (wrap) {
                uint32_t preparation_time = tick;
                PrepareCachedStatus(attempt, node, UINT32_MAX - 75U - preparation_time);
                assert(tick == UINT32_MAX - 75U);
            }
            uint32_t rx_ms = tick;
            unsigned requests_before = status_requests[attempt][node];
            Canopen_ProcessRx(rx_ms, 8); /* complete RX without consuming its value */
            tick = rx_ms + ages[a];
            for (unsigned i = 0; i < 3 && s_drive.boot.state != DRIVE_BOOT_READY; i++)
                Step(0, false);
            assert(StatusTimestamp(node) == rx_ms);
            if (ages[a] <= DRIVE_WHEEL_FEEDBACK_STALE_MS) {
                Ready(attempt);
                assert(status_requests[attempt][node] == requests_before);
            } else {
                assert(s_drive.boot.state != DRIVE_BOOT_READY && !s_hardware_disable_confirmed);
                assert(status_requests[attempt][node] == requests_before);
                UntilTerminal(); Ready(attempt);
                assert(status_requests[attempt][node] > requests_before);
                assert((uint32_t)(tick - StatusTimestamp(node)) <= DRIVE_WHEEL_FEEDBACK_STALE_MS);
            }
            cases++;
        }
    printf("PASS RX age: %u node/attempt/wrap cases; actual RX age 149/150 allowed, 151/200 requires fresh read before READY\n", cases);
}

static void TestLiftDiagnosticStatusAge(void)
{
    for (unsigned attempt = 1; attempt <= 2; attempt++)
      for (unsigned node = 1; node <= 2; node++) {
        PrepareCachedStatus(attempt, node, 0);
        Lift_TestResetState();
        unsigned requests_before = status_requests[attempt][node];
        uint32_t diagnostic_start = tick;
        silent_node = 3; tick_step = 1;
        assert(Lift_HandleCommand("LIFT_RAW_STATUS"));
        tick_step = 0; silent_node = 0;
        uint32_t diagnostic_end = tick;
        assert((uint32_t)(diagnostic_end - diagnostic_start) > DRIVE_WHEEL_FEEDBACK_STALE_MS);
        Step(0, false);
        assert(s_drive.boot.state != DRIVE_BOOT_READY && !s_hardware_disable_confirmed);
        assert((uint32_t)(diagnostic_end - StatusTimestamp(node)) > DRIVE_WHEEL_FEEDBACK_STALE_MS);
        assert((uint32_t)(StatusTimestamp(node) - diagnostic_start) <= 3U);
        UntilTerminal(); Ready(attempt);
        assert(status_requests[attempt][node] > requests_before);
        assert((uint32_t)(tick - StatusTimestamp(node)) <= DRIVE_WHEEL_FEEDBACK_STALE_MS);
      }
    puts("PASS RX age with real LIFT_RAW_STATUS: offline Node 3 dispatches then delays wheel completion; either node/attempt must refresh before READY");
}

static uint8_t typed_reply_width;
static bool InjectTypedUpload(uint8_t node, uint16_t index, uint8_t sub,
                              const uint8_t request[8])
{
    if (request[0] != 0x40) return false;
    uint8_t response[8] = {typed_reply_width == 1 ? 0x4f :
                          typed_reply_width == 2 ? 0x4b : 0x43,
                          (uint8_t)index, (uint8_t)(index >> 8), sub,
                          0x34, 0x12, 0x56, 0x34};
    Fake_Rx(0x580U + node, response, 8);
    return true;
}

static void TestTypedSdoWidths(void)
{
    unsigned cases = 0;
    for (unsigned observer = 0; observer <= 1; observer++)
     for (unsigned node = 1; node <= 3; node++)
      for (unsigned expected = 1; expected <= 4; expected *= 2)
       for (unsigned actual = 1; actual <= 4; actual *= 2) {
        ResetAt(0); fake_sdo_hook = InjectTypedUpload; typed_reply_width = (uint8_t)actual;
        Canopen_BootSdoBegin(); if (!observer) Canopen_BootSdoEnd();
        CanopenAsyncStartResult_t started = expected == 1 ?
            Canopen_AsyncReadU8((uint8_t)node, 0x2001, 0, CANOPEN_SDO_OWNER_SAFETY_CONFIRM, tick, 20) :
            expected == 2 ?
            Canopen_AsyncReadU16((uint8_t)node, 0x2001, 0, CANOPEN_SDO_OWNER_SAFETY_CONFIRM, tick, 20) :
            Canopen_AsyncReadU32((uint8_t)node, 0x2001, 0, CANOPEN_SDO_OWNER_SAFETY_CONFIRM, tick, 20);
        assert(started == CANOPEN_ASYNC_START_OK);
        Canopen_ProcessRx(tick, 8);
        uint32_t value = 0, abort_code = 0;
        CanopenAsyncResult_t result = Canopen_AsyncPoll((uint8_t)node,
            CANOPEN_SDO_OWNER_SAFETY_CONFIRM, tick, &value, &abort_code);
        assert(result == (actual == expected ? CANOPEN_ASYNC_RESULT_SUCCESS :
                                             CANOPEN_ASYNC_RESULT_PROTOCOL_ERROR));
        if (actual == expected) assert(value == (actual == 1 ? 0x34U : actual == 2 ? 0x1234U : 0x34561234U));
        else assert(!s_last_sdo_response.valid);
        ResetAt(0); fake_sdo_hook = InjectTypedUpload; tick_step = 1;
        Canopen_BootSdoBegin(); if (!observer) Canopen_BootSdoEnd();
        int8_t value8 = 0; uint16_t value16 = 0; int32_t value32 = 0;
        bool success = expected == 1 ? SDO_Read_I8((uint8_t)node, 0x2001, 0, &value8) :
            expected == 2 ? SDO_Read_U16((uint8_t)node, 0x2001, 0, &value16) :
                            SDO_Read_I32((uint8_t)node, 0x2001, 0, &value32);
        assert(success == (actual == expected));
        assert(s_last_sdo_response.valid == success);
        if (success) assert(expected == 1 ? value8 == 0x34 : expected == 2 ? value16 == 0x1234 : value32 == 0x34561234);
        cases += 2;
       }
    printf("PASS typed SDO widths: %u DLC8 cases, U8/U16/U32 versus 4F/4B/43, async/blocking, Nodes 1/2/3, observer on/ended; mismatches are protocol errors and invalid mailboxes\n", cases);
}

static unsigned arriving_emcy_node, arriving_emcy_count;
static void QueueEmcyWhileDraining(uint32_t id)
{
    if (id != 0x777 || arriving_emcy_count >= 2) return;
    uint8_t emcy[8] = {0};
    if (arriving_emcy_count == 0) { emcy[0] = 0x10; emcy[1] = 0x23; emcy[2] = 1; }
    arriving_emcy_count++;
    Fake_Rx(0x80U + arriving_emcy_node, emcy, 8);
}

static void TestEightSlotRxArrivals(void)
{
    assert(FDCAN_DLC_BYTES_8 == 8U && FDCAN_DLC_BYTES_4 == 4U);
    for (unsigned node = 1; node <= 2; node++)
      for (unsigned deadline = 0; deadline <= 1; deadline++) {
        ResetAt(0); offline_mask = 6; forever_offline = true; UntilWait();
        fake_rx_capacity = 8;
        arriving_emcy_node = node; arriving_emcy_count = 0;
        fake_rx_consumed_hook = QueueEmcyWhileDraining;
        uint8_t noise[8] = {0};
        for (unsigned i = 0; i < 8; i++) Fake_Rx(0x777, noise, 8);
        tick = s_drive.boot.retry_started_ms + (deadline ? 5000U : 200U);
        Step(0, false);
        assert(arriving_emcy_count == 2);
        assert(s_drive.boot.retries == 0 && !second_seen);
        if (deadline) assert(s_drive.boot.state == DRIVE_BOOT_FAILED);
        Step(10, false);
        Failed(0);
        assert(!strstr(Fake_Log(), "DRIVE_BOOT attempt=2 state=START"));
      }
    puts("PASS 8-slot RX model: Fault/Clear arrive as slots are consumed, cross the 8-frame budget and forbid retry; physical overflow/loss remains untested");
}

static unsigned resume_arrival_node, resume_arrival_prefix, resume_pop_count;
static bool resume_continuous;

static void QueueFramesInsideResume(uint32_t id)
{
    if (id != 0x777U) return;
    unsigned consumed = resume_pop_count++;
    uint8_t data[8] = {0};
    if (resume_continuous || consumed < resume_arrival_prefix) {
        /* Refill each freed slot, never exceeding the modeled capacity. In
         * the integrated case the first eight refills survive periodic RX
         * dispatch and make up the snapshot seen by BootSdoResume. */
        Fake_Rx(0x777U, data, 8);
    } else if (consumed < resume_arrival_prefix + 2U) {
        if (resume_arrival_node != 0) {
            if (consumed == resume_arrival_prefix) {
                data[0] = 0x10; data[1] = 0x23; data[2] = 1;
            }
            Fake_Rx(0x80U + resume_arrival_node, data, 8);
        } else {
            Fake_Heartbeat((uint8_t)(consumed - resume_arrival_prefix + 1U), 5);
        }
    }
}

static void PrepareResumeArrival(bool integrated, unsigned fault_node, bool continuous)
{
    ResetAt(0); offline_mask = 6; forever_offline = true; UntilWait();
    Canopen_ProcessRx(tick, 128U);
    assert(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0);
    fake_rx_capacity = 8;
    resume_arrival_node = fault_node;
    resume_arrival_prefix = integrated ? 8U : 0U;
    resume_pop_count = 0;
    resume_continuous = continuous;
    fake_rx_consumed_hook = QueueFramesInsideResume;
    uint8_t noise[8] = {0};
    for (unsigned i = 0; i < 8; i++) Fake_Rx(0x777U, noise, 8);
    tick = s_drive.boot.retry_started_ms + DRIVE_BOOT_RETRY_DELAY_MS;
}

static void ResumeArrivalTurn(bool integrated)
{
    uint32_t before = tick;
    /* Call the actual loop without Step's extra simulated heartbeat arrivals:
     * all eight FIFO slots may already be occupied at entry. */
    if (integrated) Drive_ControlLoop_Update();
    else Drive_BootService(tick);
    assert(tick == before);
    Inhibited();
}

static void DeferredResume(uint32_t wait_started, uint32_t prior_sdo_ops)
{
    assert(s_drive.boot.state == DRIVE_BOOT_WAIT_RETRY);
    assert(s_drive.core.state == DRIVE_STATE_INIT);
    assert(s_drive.boot.retries == 0 && !second_seen);
    assert(s_drive.boot.retry_started_ms == wait_started);
    assert(!s_hardware_disable_confirmed);
    assert(!s_drive.safety.transition.active && !s_drive.safety.transition.request_pending);
    assert(disable_requests[2][1] == 0 && disable_requests[2][2] == 0);
    assert(status_requests[2][1] == 0 && status_requests[2][2] == 0);
    assert(!strstr(Fake_Log(), "DRIVE_BOOT attempt=2 state=START"));
    for (unsigned node = 1; node <= 2; node++) {
        assert(Canopen_AsyncReadU16((uint8_t)node, 0x6041, 0,
            CANOPEN_SDO_OWNER_SAFETY_CONFIRM, tick, 20) == CANOPEN_ASYNC_START_BUSY);
    }
    assert(sdo_ops == prior_sdo_ops); /* The wheel fence prevented all TX. */
    Inhibited();
}

static void FinishDeferredResume(bool integrated, uint32_t wait_started)
{
    fake_rx_consumed_hook = NULL;
    forever_offline = false;
    uint32_t resumed_at = tick;
    ResumeArrivalTurn(integrated);
    assert(tick == resumed_at); /* No new five-second wait after FIFO empties. */
    assert(s_drive.boot.retry_started_ms == wait_started);
    assert(s_drive.boot.state == DRIVE_BOOT_SECOND_ATTEMPT && s_drive.boot.retries == 1);
    UntilTerminal(); Ready(2);
    const char *start = strstr(Fake_Log(), "DRIVE_BOOT attempt=2 state=START");
    assert(start && !strstr(start + 1, "DRIVE_BOOT attempt=2 state=START"));
}

static void TestResumeMidDrainArrivals(void)
{
    unsigned cases = 0;
    for (unsigned integrated = 0; integrated <= 1; integrated++) {
        for (unsigned node = 1; node <= 2; node++) {
            PrepareResumeArrival(integrated != 0, node, false);
            uint32_t wait_started = s_drive.boot.retry_started_ms;
            uint32_t prior_sdo_ops = sdo_ops;
            ResumeArrivalTurn(integrated != 0);
            assert(resume_pop_count == (integrated ? 16U : 8U));
            assert(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 2);
            DeferredResume(wait_started, prior_sdo_ops);
            assert(Canopen_GetBootSdoEvidence().hard_fault_reason == CANOPEN_BOOT_SDO_FAULT_NONE);
            Step(0, false); /* Dispatch the queued nonzero EMCY followed by Clear. */
            Failed(0);
            assert(Canopen_GetBootSdoEvidence().hard_fault_reason == CANOPEN_BOOT_SDO_FAULT_EMCY);
            assert(Canopen_GetBootSdoEvidence().node_id == node);
            assert(!strstr(Fake_Log(), "DRIVE_BOOT attempt=2 state=START"));
            assert(disable_requests[2][1] == 0 && disable_requests[2][2] == 0);
            cases++;
        }
        PrepareResumeArrival(integrated != 0, 0, false);
        uint32_t wait_started = s_drive.boot.retry_started_ms;
        uint32_t prior_sdo_ops = sdo_ops;
        ResumeArrivalTurn(integrated != 0);
        assert(resume_pop_count == (integrated ? 16U : 8U));
        assert(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 2);
        DeferredResume(wait_started, prior_sdo_ops);
        FinishDeferredResume(integrated != 0, wait_started);
        cases++;

        PrepareResumeArrival(integrated != 0, 0, true);
        wait_started = s_drive.boot.retry_started_ms;
        prior_sdo_ops = sdo_ops;
        for (unsigned turn = 0; turn < 4; turn++) {
            unsigned before = resume_pop_count;
            ResumeArrivalTurn(integrated != 0);
            /* A continuous stream must return after exactly one bounded
             * snapshot (plus the ordinary eight-frame budget in the loop). */
            assert(resume_pop_count - before == (integrated ? 16U : 8U));
            assert(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 8);
            DeferredResume(wait_started, prior_sdo_ops);
            tick += 10;
        }
        FinishDeferredResume(integrated != 0, wait_started);
        cases++;
    }
    printf("PASS Resume arrivals: %u cases, direct BootService and full control loop; either-node Fault/Clear stays fenced with retries=0/no START/no SDO, benign traffic resumes once without resetting 5s, continuous traffic returns after bounded RX passes\n", cases);
}

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    if (argc == 2) {
        if (!strcmp(argv[1], "--review-replay")) TestReviewReplay();
        else if (!strcmp(argv[1], "--review-emcy")) TestReviewEmcyDrain();
        else if (!strcmp(argv[1], "--review-ack")) TestReviewMalformedAck();
        else if (!strcmp(argv[1], "--sdo-dlc8")) TestGenericSdoDlc8();
        else if (!strcmp(argv[1], "--sdo-oversize")) TestUnsupportedSdoDlc();
        else if (!strcmp(argv[1], "--rx-age")) { TestCachedStatusAge(); TestLiftDiagnosticStatusAge(); }
        else if (!strcmp(argv[1], "--sdo-widths")) TestTypedSdoWidths();
        else if (!strcmp(argv[1], "--rx-eight-slots")) TestEightSlotRxArrivals();
        else if (!strcmp(argv[1], "--resume-arrivals")) TestResumeMidDrainArrivals();
        else assert(!"Unknown test selection");
        return 0;
    }
    TestImmediate();
    TestDelayed(2, 2); TestDelayed(4, 3);
    TestSecondAttemptOnly(); TestPermanentOffline();
    TestHardFaults(); TestLateHardEvidence(); TestDelayedOldResponses();
    TestCommandsAndWrap(0, 11);
    TestCommandsAndWrap(UINT32_MAX - 1000U, 13);
    TestRuntimeAndInvalidStatus();
    TestMissingStatus();
    TestResponseLatency();
    TestDs402DisabledStates();
    TestReviewReplay(); TestReviewEmcyDrain(); TestReviewMalformedAck();
    TestDiagnosticAndPollingCompatibility();
    TestGenericSdoDlc8();
    TestUnsupportedSdoDlc();
    TestCachedStatusAge(); TestLiftDiagnosticStatusAge();
    TestTypedSdoWidths(); TestEightSlotRxArrivals();
    TestResumeMidDrainArrivals();
    puts("PASS: V1 software regressions; same-object SDO replay remains a KNOWN LIMITATION (hardware NOT RUN)");
    return 0;
}
