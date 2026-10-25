#include "fake_bus.h"
#include "drive_internal.h"
#include "drive_target.h"
#include "wheel_pdo_config.h"
#include "wheel_rpdo_tx.h"
#include "app_config.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void Loop(bool tpdo)
{
    tick+=10;
    if(!bus_off){Fake_Heartbeat(1,5);Fake_Heartbeat(2,5);}
    if(tpdo)Fake_Tpdo(0,0);
    Drive_ControlLoop_Update();
    assert(!s_drive.core.enabled && !s_hardware_enable_confirmed);
    assert(enable_controlwords==0 && WheelRpdoTx_Get().queued_pairs==0);
}

static void FaultAndReset(bool persistent)
{
    Fake_Reset();Drive_TestResetState();Drive_SetLiftMovingCallback(NULL);
    Fake_Heartbeat(1,0);Fake_Heartbeat(2,0);
    bus_off=1;
    for(unsigned i=0;i<250;i++)Loop(false);
    assert(s_drive.core.state==DRIVE_STATE_FAULT);
    assert(!s_drive.safety.transition.active);
    assert(s_tracking_stop_latched && s_disable_motion_inhibit_latched);
    assert(!WheelPdoConfig_Get(1).configured);
    /* Restore only the transport/device. No direct mutation of Drive latches,
     * PDO state, disable proof or feedback caches on this recovery path. */
    bus_off=0;wheel_statusword[1]=0x08;persistent_wheel_fault=persistent;
    for(unsigned i=0;i<10;i++)Loop(false);
    assert(Drive_CommandFaultResetSafe(0x12345678));
    for(unsigned i=0;i<200 && s_drive.fault_reset.transaction.active;i++)Loop(false);
}

static void RecoverToDisable(void)
{
    FaultAndReset(false);
    assert(s_drive.fault_reset.transaction.result==DRIVE_FAULT_RESET_RESULT_COMPLETE);
    assert(reset_controlwords==1 && s_drive.core.state==DRIVE_STATE_FAULT);
    assert(s_tracking_stop_latched && s_disable_motion_inhibit_latched);
    assert(Drive_CommandDisableChecked()==CMD_RESULT_ACCEPTED);
    for(unsigned i=0;i<200 && s_drive.safety.transition.active;i++)Loop(false);
    if(s_drive.core.state!=DRIVE_STATE_DISABLED)puts(Fake_Log());
    assert(s_drive.core.state==DRIVE_STATE_DISABLED && s_hardware_disable_confirmed);
    assert(s_tracking_stop_latched && s_disable_motion_inhibit_latched);
}

static bool LiftBusy(void) { return true; }
static void PrepareGate(bool arm)
{
    Fake_Reset();Drive_TestResetState();Drive_SetLiftMovingCallback(NULL);
    Drive_TestSetRuntimeReady(DRIVE_STATE_DISABLED,0,0,tick);
    /* Fault injection fixture only. The end-to-end regression above obtains
     * these flags/proofs exclusively through real faults/reset/DISABLE. */
    DriveCore_SetTrackingStopLatched(true);DriveCore_SetMotionInhibitLatched(true);
    if(arm){
        assert(WheelPdoConfig_Configure());
        assert(Canopen_ConfigureWheelTpdoFeedbackNode(1));
        assert(Canopen_ConfigureWheelTpdoFeedbackNode(2));
        Fake_Tpdo(0,0);Canopen_ProcessRx(tick,128);
        s_drive.tpdo_active.startup_state=DRIVE_TPDO_STARTUP_WAIT_FRESH;
        s_drive.tpdo_active.startup_qualify_start_ms=tick;
    }
    tick_step=0;
}

static void GateBlocked(bool arm, const char *reason)
{
    uint32_t before=sdo_ops;
    Fake_ClearLog();
    if(arm)assert(!Drive_CommandWheelTpdoActiveEnable(0x12345678));
    assert(!Drive_ServiceWheelTpdoStartup(tick));
    Drive_PrintPdoStartupDiagnostic(tick,1,0x12345678);
    assert(strstr(Fake_Log(),reason));
    assert(sdo_ops==before && enable_controlwords==0);
    assert(s_tracking_stop_latched && s_disable_motion_inhibit_latched);
    assert(s_drive.tpdo_active.startup_state==(arm?
        DRIVE_TPDO_STARTUP_WAIT_FRESH:DRIVE_TPDO_STARTUP_WAIT_SAFE));
    if(!arm && !strcmp(reason,"DRIVE_FAULT_ACTIVE")){
        Fake_ClearLog();DriveTarget_PrintDiagnostics(0x12345678);
        FILE *file=fopen("output/host_tests/pdo_startup_blocked_snapshot.log","wb");assert(file);
        assert(fputs(Fake_Log(),file)>=0);assert(fclose(file)==0);
    }
}

static void TestCurrentHazards(void)
{
    for(unsigned arm=0;arm<2;arm++){
        PrepareGate(arm);DriveCore_WriteStateUnchecked(DRIVE_STATE_FAULT);
        GateBlocked(arm,"DRIVE_FAULT_ACTIVE");
        PrepareGate(arm);DriveCore_SetHardwareDisableConfirmed(false);
        GateBlocked(arm,"HARDWARE_DISABLE_UNCONFIRMED");
        PrepareGate(arm);DriveCore_SetHardwareEnableConfirmed(true);
        GateBlocked(arm,"HARDWARE_ENABLE_CONFIRMED");
        PrepareGate(arm);DriveCore_SetSoftwareEnabled(true);GateBlocked(arm,"DRIVE_ENABLED");
        PrepareGate(arm);s_drive.safety.transition.active=true;GateBlocked(arm,"DRIVE_TRANSITION_ACTIVE");
        PrepareGate(arm);s_drive.fault_reset.transaction.active=true;GateBlocked(arm,"DRIVE_TRANSITION_ACTIVE");
        PrepareGate(arm);s_drive.lift_stop.request.active=true;GateBlocked(arm,"DRIVE_TRANSITION_ACTIVE");
        PrepareGate(arm);Drive_TestSetLastCommandTargetDec(1,0);GateBlocked(arm,"TARGET_NONZERO");
        PrepareGate(arm);Drive_SetLiftMovingCallback(LiftBusy);GateBlocked(arm,"LIFT_NOT_STOPPED");
        PrepareGate(arm);Drive_TestSetTpdoEstopActive(true);GateBlocked(arm,"ESTOP_ACTIVE");
        PrepareGate(arm);bus_off=1;GateBlocked(arm,"BUS_OFF_OR_UNREADABLE");
        PrepareGate(arm);Drive_TestSetStatusword(1,8,true);GateBlocked(arm,"NODE1_STATUS_UNSAFE");
        PrepareGate(arm);Drive_TestSetStatusword(2,0x27,true);GateBlocked(arm,"NODE2_STATUS_UNSAFE");
        PrepareGate(arm);Drive_CommandStop();GateBlocked(arm,"DRIVE_TRANSITION_ACTIVE");
        PrepareGate(arm);Canopen_LatchWheelTpdoStartupFault("INJECTED");GateBlocked(arm,"TPDO_FAULT_LATCHED");
    }
    PrepareGate(true);tick+=DRIVE_WHEEL_FEEDBACK_STALE_MS+1;
    GateBlocked(true,"NODE1_NOT_FRESH");
    PrepareGate(true);tick+=DRIVE_WHEEL_FEEDBACK_STALE_MS+1;
    uint8_t data[4]={0};Fake_Rx(0x181,data,4);Canopen_ProcessRx(tick,128);
    GateBlocked(true,"NODE2_NOT_FRESH");
    PrepareGate(true);tick+=31;Fake_Rx(0x182,data,4);Canopen_ProcessRx(tick,128);
    GateBlocked(true,"PAIR_SKEW");
    FaultAndReset(true);
    assert(s_drive.fault_reset.transaction.result==DRIVE_FAULT_RESET_RESULT_FAILED);
    assert(s_drive.core.state==DRIVE_STATE_FAULT && !WheelPdoConfig_Get(1).configured);
    assert(s_tracking_stop_latched && s_disable_motion_inhibit_latched);
    puts("PASS: both startup gates reject current fault/unsafe hardware/transactions/STOP/Lift/E-stop/CAN; TPDO freshness/skew and persistent reset failure preserved");
}

void TestPdoStartupRecovery(void)
{
    RecoverToDisable();
    for(unsigned i=0;i<25;i++)Loop(true);
    printf("RECOVERY startup=%u drive=%s hd=%u tracking=%u inhibit=%u configured=%u/%u transport=%u\n",
        (unsigned)s_drive.tpdo_active.startup_state,Drive_StateName(s_drive.core.state),
        s_hardware_disable_confirmed,s_tracking_stop_latched,s_disable_motion_inhibit_latched,
        WheelPdoConfig_Get(1).configured,WheelPdoConfig_Get(2).configured,
        (unsigned)Canopen_GetWheelFeedbackTransport());
    fflush(stdout);
    if(s_drive.tpdo_active.startup_state!=DRIVE_TPDO_STARTUP_COMPLETE){
        Fake_ClearLog();DriveTarget_PrintDiagnostics(0x12345678);puts(Fake_Log());fflush(stdout);
    }
    assert(s_drive.tpdo_active.startup_state==DRIVE_TPDO_STARTUP_COMPLETE);
    assert(WheelPdoConfig_Ready(tick));
    assert(Canopen_GetWheelFeedbackTransport()==CANOPEN_WHEEL_FEEDBACK_TPDO_ACTIVE);
    assert(Canopen_GetWheelFeedbackAuthority()==CANOPEN_WHEEL_AUTHORITY_TPDO);
    DriveWheelFeedbackPair_t pair;
    assert(Drive_GetAuthoritativeWheelFeedbackPair(tick,&pair));
    assert(s_drive.core.state==DRIVE_STATE_DISABLED && s_hardware_disable_confirmed);
    assert(s_tracking_stop_latched && s_disable_motion_inhibit_latched);
    assert(!Drive_NormalMotionAllowed() && identity_accesses==0);
    Fake_ClearLog();DriveTarget_PrintDiagnostics(0x12345678);
    FILE *file=fopen("output/host_tests/pdo_recovery_snapshot.log","wb");assert(file);
    assert(fputs(Fake_Log(),file)>=0);assert(fclose(file)==0);
    /* A reboot after recovery cannot reuse the just-established qualification. */
    Fake_Heartbeat(1,0);Loop(false);
    assert(!WheelPdoConfig_Ready(tick));
    assert(Drive_CommandEnableChecked()==CMD_RESULT_FAULT_LATCHED);
    TestCurrentHazards();
    puts("PASS: main loop fault -> communication recovery -> reset -> DISABLE -> PDO/fresh TPDO, stays DISABLED with history intact");
}

#ifdef AMR_HOST_TEST_PDO_GATE_MAIN
int main(void)
{
    assert(!DRIVE_BASIC_BRINGUP_PROFILE);
    PrepareGate(false);GateBlocked(false,"TRACKING_OR_INHIBIT");
    PrepareGate(true);GateBlocked(true,"TRACKING_OR_INHIBIT");
    puts("PASS: non-BASIC profile retains both strict tracking/inhibit gates");
    return 0;
}
#endif
