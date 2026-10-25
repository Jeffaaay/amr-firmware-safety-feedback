#include "fake_bus.h"
#include "drive_internal.h"
#include "lift.h"
#include "app_config.h"
#include "wheel_rpdo_tx.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void Step(uint32_t ms, unsigned mask, int32_t left, int32_t right)
{
    uint8_t data[4];
    tick+=ms;
    Fake_Heartbeat(1,5);Fake_Heartbeat(2,5);
    if(mask&1U){WheelRpdoTx_Encode(left,data);Fake_Rx(0x181,data,4);}
    if(mask&2U){WheelRpdoTx_Encode(right,data);Fake_Rx(0x182,data,4);}
    Drive_ControlLoop_Update();Lift_Update();
}
static void Turn(void) { Step(10,3,0,0); }
static void DisabledAt(uint32_t start)
{
    TestDriveReadyAt(start);Lift_TestResetState();Drive_SetLiftMovingCallback(Lift_IsMoving);
    assert(Drive_CommandDisableChecked()==CMD_RESULT_ACCEPTED);
    for(unsigned n=0;n<100 && Drive_TestSafetyTransitionActive();n++)Turn();
    assert(Drive_TestGetState()==DRIVE_STATE_DISABLED);
    assert(Drive_TestPostDisableZeroProofRequired());
    Fake_ClearLog();
    assert(Drive_RequestBaseSafeRefresh(0x12345678));
    for(unsigned n=0;n<100 && Drive_TestBaseSafeRefreshActive();n++)Turn();
    assert(strstr(Fake_Log(),"DRIVE_BASE_SAFE_REFRESH state=COMPLETE safe=1"));
    assert(Drive_TestBaseHardwareSafeForLift(tick));
    assert(Lift_HandleCommand("LIFT_FAULT_RESET"));
    for(unsigned n=0;n<100 && Lift_TestPendingAction();n++)Turn();
    assert(strstr(Fake_Log(),"LIFT_FAULT_RESET state=COMPLETE"));
    assert(Lift_TestFaultReason()==0);
}
static void Disabled(void) { DisabledAt(0); }
static void AssertHoming(void)
{
    assert(strstr(Fake_Log(),"LIFT sensor homing started after base confirmed-stop"));
    assert(Lift_TestFaultReason()==0);
    assert(!Lift_TestPendingAction());
}
static void AssertBlocked(void)
{
    assert(!strstr(Fake_Log(),"LIFT sensor homing started after base confirmed-stop"));
    assert(Lift_TestPendingAction());
    assert(!Drive_TestBaseHardwareSafeForLift(tick));
}
static void ResetThenHome(bool service_drive, uint32_t start)
{
    DisabledAt(start);
    /* UART dispatch precedes Drive update in main.c. Cover both normal
     * servicing and a bounded 50 ms gap with fresh CAN RX but an older Drive
     * cache. The old interlock erased that cache during fault reset. */
    for(unsigned n=0;n<5;n++) {
        if(service_drive){Turn();continue;}
        tick+=10;Fake_Heartbeat(1,5);Fake_Heartbeat(2,5);Fake_Tpdo(0,0);
        Canopen_ProcessRx(tick,128);
    }
    assert(Lift_HandleCommand("LIFT_HOME"));
    for(unsigned n=0;n<120 && Lift_TestPendingAction();n++)Turn();
    if(!strstr(Fake_Log(),"LIFT sensor homing started after base confirmed-stop")) {
        printf("REPRO Lift fault=%04X zero=%u post=%u stable=%s reject=%s elapsed=%u\n%s",
            (unsigned)Lift_TestFaultReason(),Drive_TestZeroEvidenceValid()?1U:0U,
            Drive_TestPostDisableZeroProofRequired()?1U:0U,Drive_StableZeroStateName(),
            Drive_StableZeroRejectName(),(unsigned)s_drive.stable_zero.overall_elapsed_ms,Fake_Log());
        fflush(stdout);
    }
    AssertHoming();
    assert(strstr(Fake_Log(),"zero_valid="));
    assert(strstr(Fake_Log(),"post_required="));
    assert(strstr(Fake_Log(),"post_base="));
    assert(strstr(Fake_Log(),"stable_base="));
    assert(strstr(Fake_Log(),"status_gen="));
    assert(strstr(Fake_Log(),"static_safe="));
    assert(strstr(Fake_Log(),"rx_seq="));
}
static void StartFreshConfirmation(void)
{
    Disabled();
    /* Real moving feedback invalidates the proven stop. A single subsequent
     * zero must not silently recreate it in the disabled maintenance path. */
    Step(10,3,1000000,1000000);
    assert(!Drive_TestZeroEvidenceValid());
    Turn();
    assert(!Drive_TestZeroEvidenceValid());
    Fake_ClearLog();
    assert(Lift_HandleCommand("LIFT_HOME"));
    AssertBlocked();
    assert(s_drive.stable_zero.stable_sample_count==0);
}
static void TestBilateralAndDwell(void)
{
    StartFreshConfirmation();
    for(unsigned n=1;n<=3;n++) {
        Turn();AssertBlocked();
        assert(s_drive.stable_zero.stable_sample_count==n);
    }
    assert(s_drive.stable_zero.stable_dwell_ms==20);
    Step(79,3,0,0);AssertBlocked();
    assert(s_drive.stable_zero.stable_dwell_ms==99);
    Step(1,3,0,0);
    assert(s_drive.stable_zero.stable_dwell_ms==100);
    for(unsigned n=0;n<20 && Lift_TestPendingAction();n++)Turn();
    AssertHoming();

    StartFreshConfirmation();
    Turn();Step(100,3,0,0);AssertBlocked();
    assert(s_drive.stable_zero.stable_sample_count==2);
    assert(s_drive.stable_zero.stable_dwell_ms==100);
    Step(10,0,0,0);AssertBlocked();
    assert(s_drive.stable_zero.stable_sample_count==2);
    Turn();assert(s_drive.stable_zero.stable_sample_count==3);
    for(unsigned n=0;n<20 && Lift_TestPendingAction();n++)Turn();
    AssertHoming();
}
static void TestRejectedFeedback(unsigned mask, int32_t left, int32_t right,
                                 const char *reject)
{
    StartFreshConfirmation();
    for(unsigned n=0;n<120 && Lift_TestPendingAction();n++)Step(10,mask,left,right);
    assert(!Lift_TestPendingAction());
    assert(Lift_TestFaultReason()==0xE10E);
    assert(!strstr(Fake_Log(),"LIFT sensor homing started after base confirmed-stop"));
    assert(strstr(Fake_Log(),"event=FAILED"));
    if(!strstr(Fake_Log(),reject)) {
        printf("Expected %s for mask=%u left=%ld right=%ld\n%s",reject,mask,
               (long)left,(long)right,Fake_Log());fflush(stdout);
    }
    assert(strstr(Fake_Log(),reject));
    assert(!Drive_TestBaseHardwareSafeForLift(tick));
}
static void TestStopPreemption(void)
{
    StartFreshConfirmation();Turn();
    uint32_t generation=s_drive.evidence.safety_generation;
    Drive_CommandStop();
    assert(Drive_TestSafetyTransitionActive());
    assert(s_drive.evidence.safety_generation!=generation);
    Turn();
    assert(!Lift_TestPendingAction());
    assert(Lift_TestFaultReason()==0xE10E);
    assert(strstr(Fake_Log(),"reason=GENERATION"));
    assert(!strstr(Fake_Log(),"LIFT sensor homing started after base confirmed-stop"));
}
static void WaitForStatusPhase(uint32_t phase)
{
    for(unsigned n=0;n<20 && s_drive.lift_stop.request.phase!=phase;n++)Turn();
    assert(s_drive.lift_stop.request.phase==phase);
    assert(!s_drive.lift_stop.request.request_pending);
}
static void TestStatuswordFailure(uint32_t phase)
{
    StartFreshConfirmation();WaitForStatusPhase(phase);
    fail_sdo=sdo_ops+1;
    for(unsigned n=0;n<20 && Lift_TestPendingAction();n++)Turn();
    assert(!Lift_TestPendingAction());
    assert(Lift_TestFaultReason()==0xE10E);
    assert(strstr(Fake_Log(),"reason=SDO_ABORT"));
    assert(strstr(Fake_Log(),"abort=06090011"));
    assert(!strstr(Fake_Log(),"LIFT sensor homing started after base confirmed-stop"));
}
static void TestStatuswordFreshness(void)
{
    StartFreshConfirmation();WaitForStatusPhase(DRIVE_LIFT_STOP_PHASE_STATUS_RIGHT);
    /* TPDO stays current, but the earlier left statusword expires before the
     * right SDO completes. Disabled bits alone must not authorize homing. */
    for(unsigned n=0;n<(DRIVE_WHEEL_FEEDBACK_STALE_MS/10U)+1U;n++) {
        tick+=10;Fake_Heartbeat(1,5);Fake_Heartbeat(2,5);Fake_Tpdo(0,0);
        Drive_ControlLoop_Update(); /* Delay only the Lift statusword consumer. */
    }
    Turn();Turn();
    assert(!Lift_TestPendingAction());
    assert(Lift_TestFaultReason()==0xE10E);
    if(!strstr(Fake_Log(),"reason=STATUS_STALE")){puts(Fake_Log());fflush(stdout);}
    assert(strstr(Fake_Log(),"reason=STATUS_STALE"));
    assert(!strstr(Fake_Log(),"LIFT sensor homing started after base confirmed-stop"));
    assert(!Drive_TestBaseHardwareSafeForLift(tick));

    Disabled();
    for(unsigned n=0;n<20;n++)Turn();
    assert(Drive_TestZeroEvidenceValid());
    assert(!Drive_TestStatuswordsConfirmDisabled(tick));
    Fake_ClearLog();uint32_t before=sdo_ops;
    assert(Drive_RequestBaseSafeRefresh(0xabcdef01));
    for(unsigned n=0;n<20 && Drive_TestBaseSafeRefreshActive();n++)Turn();
    assert(strstr(Fake_Log(),"state=COMPLETE safe=1"));
    assert(sdo_ops==before+2); /* Only fresh bilateral 6041 reads, no fake proof. */
    assert(Drive_TestBaseHardwareSafeForLift(tick));
}
void TestLiftBaseStop(void)
{
    ResetThenHome(false,0);ResetThenHome(true,0);
    uint32_t wrap_start=UINT32_MAX-tick+25U;
    ResetThenHome(false,wrap_start);assert(tick<wrap_start);
    puts("PASS: real Lift reset + 50 ms + HOME; continuous/gapped Drive service and tick wrap");
    TestBilateralAndDwell();
    puts("PASS: invalidated zero evidence requires >=3 fresh bilateral pairs AND >=100 ms; cached pair cannot count");
    TestRejectedFeedback(0,0,0,"reason=TPDO_ACTIVE_FEEDBACK_STALE");
    TestRejectedFeedback(1,0,0,"reason=TPDO_ACTIVE_FEEDBACK_STALE");
    TestRejectedFeedback(3,1000000,0,"last_reject=LEFT_SPEED");
    TestRejectedFeedback(3,0,1000000,"last_reject=RIGHT_SPEED");
    TestStopPreemption();
    TestStatuswordFailure(DRIVE_LIFT_STOP_PHASE_STATUS_LEFT);
    TestStatuswordFailure(DRIVE_LIFT_STOP_PHASE_STATUS_RIGHT);
    TestStatuswordFreshness();
    puts("PASS: Lift rejects stale/one-sided/nonzero feedback, STOP generation change and either statusword SDO abort");
    puts("PASS: expired statusword blocks homing; fresh bilateral readback renews disabled evidence");
}
