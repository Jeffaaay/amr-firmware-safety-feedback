#include "fake_bus.h"
#include "wheel_pdo_config.h"
#include "wheel_rpdo_tx.h"
#include "drive_target.h"
#include "drive_internal.h"
#include "motion_control.h"
#include "app_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void Pump(void) { Canopen_ProcessRx(tick, 128); }
static bool AttemptConfiguration(void)
{
    bool ok=WheelPdoConfig_Configure();
    assert(identity_accesses==0);
    return ok;
}
static void Configure(void)
{
    assert(AttemptConfiguration());
    assert(!WheelPdoConfig_Ready(tick));
    Fake_Heartbeat(1,5);Fake_Heartbeat(2,5);Pump();
    tick_step=0;
    assert(WheelPdoConfig_Ready(tick));
    assert(WheelRpdoTx_Rearm(tick));
}
static void AssertBlocked(void)
{
    assert(!WheelPdoConfig_Ready(tick));
    assert(!WheelPdoConfig_Get(1).configured && !WheelPdoConfig_Get(2).configured);
    assert(Drive_CommandEnableChecked()==CMD_RESULT_FAULT_LATCHED);
}
static void SaveSnapshot(const char *path)
{
    Fake_ClearLog();DriveTarget_PrintDiagnostics(0x12345678);
    FILE *file=fopen(path,"wb");assert(file);
    assert(fputs(Fake_Log(),file)>=0);assert(fclose(file)==0);
}
static void AssertSdoFailure(void)
{
    AssertBlocked();
    WheelPdoConfigDiagnostic d=WheelPdoConfig_Get(last_sdo_node);
    assert(d.index==last_sdo_index && d.subindex==last_sdo_sub);
    assert(d.phase && strcmp(d.phase,"NOT_STARTED"));
    assert(!strcmp(WheelPdoConfig_FailureReason(),d.reason));
    assert(strstr(Fake_Log(),"phase=") && strstr(Fake_Log(),"expected="));
    assert(strstr(Fake_Log(),"actual=") && strstr(Fake_Log(),"abort="));
    assert(!strstr(Fake_Log(),"RPDO_CONFIG_FAILED"));
}
static void TestConfiguration(void)
{
    Fake_Reset();Configure();uint32_t operations=sdo_ops;
    assert(operations==34); /* 17 necessary configuration/readback SDOs per node */
    assert(type_writes==0); /* vendor default 254 is preserved */
    assert(strstr(Fake_Log(),"state=NOT_QUERIED vendor=UNKNOWN product=UNKNOWN revision=UNKNOWN serial=UNKNOWN"));
    for(unsigned node=1;node<=2;node++) {
        WheelPdoConfigDiagnostic d=WheelPdoConfig_Get((uint8_t)node);
        assert(d.readback[0]==0x300+node && d.readback[1]==254);
        assert(d.readback[2]==1 && d.readback[3]==0x60ff0020);
    }
    SaveSnapshot("output/host_tests/rpdo_no_identity_snapshot.log");
    /* Abort EVERY remaining config write/read/final verification, each wheel.
     * The fake device rejects 0x1018; no fabricated identity values exist. */
    for(uint32_t n=1;n<=operations;n++) {
        Fake_Reset();fail_sdo=n;
        assert(!AttemptConfiguration());AssertSdoFailure();
        WheelPdoConfigDiagnostic d=WheelPdoConfig_Get(last_sdo_node);
        assert(d.abort_code==injected_abort);
        assert(!strcmp(d.phase,last_sdo_write?"WRITE":
            (d.index==0x1401 && d.subindex==2 && n%17==11?"TYPE_READ":
             (n%17>=14 || n%17==0?"FINAL_READ":"READBACK"))));
    }
    unsigned mismatches=0;
    for(uint32_t n=1;n<=operations;n++) {
        Fake_Reset();mismatch_sdo=n;
        if(!AttemptConfiguration()) {
            mismatches++;AssertSdoFailure();
            WheelPdoConfigDiagnostic d=WheelPdoConfig_Get(last_sdo_node);
            assert(d.actual!=d.expected && d.abort_code==0);
            assert(strstr(d.reason,"MISMATCH"));
        }
    }
    assert(mismatches==20); /* type mismatch is repaired by verified write 254 */
    Fake_Reset();fail_nmt=1;assert(!AttemptConfiguration());AssertBlocked();
    assert(!strcmp(WheelPdoConfig_FailureReason(),"NMT_PREOP_TX_FAILED"));
    for(uint8_t node=1;node<=2;node++) {
        Fake_Reset();drop_nmt=node;assert(!AttemptConfiguration());AssertBlocked();
        WheelPdoConfigDiagnostic d=WheelPdoConfig_Get(node);
        assert(!strcmp(d.phase,"NMT_PREOP") && d.expected==0x7f && d.actual==0);
        assert(!strcmp(d.reason,"NMT_PREOP_NOT_CONFIRMED"));
        Fake_Reset();silent_node=node;assert(!AttemptConfiguration());AssertSdoFailure();
        d=WheelPdoConfig_Get(node);
        assert(!strcmp(d.phase,"WRITE") && d.index==0x1017 && d.subindex==0);
        assert(d.expected==DRIVE_KINCO_HEARTBEAT_PRODUCER_MS && d.abort_code==0);
    }
    Fake_Reset();initial_type=255;assert(AttemptConfiguration());assert(type_writes==2);
    uint32_t nondefault_operations=sdo_ops;
    for(uint32_t n=1;n<=nondefault_operations;n++) {
        Fake_Reset();initial_type=255;fail_sdo=n;
        assert(!AttemptConfiguration());AssertSdoFailure();
    }
    Fake_Reset();bad_width_sdo=2;assert(!AttemptConfiguration());AssertSdoFailure();
    assert(!strcmp(WheelPdoConfig_FailureReason(),"READ_FAILED"));
    Fake_Reset();fail_sdo=7;injected_abort=0x06040041;
    assert(!AttemptConfiguration());AssertSdoFailure();
    WheelPdoConfigDiagnostic d=WheelPdoConfig_Get(1);
    assert(!strcmp(d.reason,"OBJECT_NOT_MAPPABLE") && !strcmp(d.phase,"WRITE"));
    assert(d.index==0x1601 && d.subindex==1 && d.expected==0x60ff0020);
    assert(d.abort_code==0x06040041 && d.actual==0);
    SaveSnapshot("output/host_tests/rpdo_abort_snapshot.log");
    Fake_Reset();suppress_response=1;assert(!AttemptConfiguration());AssertSdoFailure();
    Fake_Reset();restart_sdo=operations-1;assert(!AttemptConfiguration());AssertBlocked();
    d=WheelPdoConfig_Get(1);
    assert(!strcmp(d.phase,"RESTART_CHECK") && !strcmp(d.reason,"NODE_RESTARTED"));
    assert(d.actual!=d.expected);
    Fake_Reset();Configure();uint32_t epoch=WheelPdoConfig_Get(1).epoch;
    Fake_Heartbeat(1,0);Pump();assert(!WheelPdoConfig_Ready(tick));
    assert(WheelPdoConfig_Get(1).epoch!=epoch);
    Fake_Heartbeat(1,5);Pump();assert(!WheelPdoConfig_Ready(tick));
    assert(Drive_CommandEnableChecked()==CMD_RESULT_FAULT_LATCHED);
    /* Recovery uses the same real configuration path, still without identity. */
    tick_step=1;Configure();assert(identity_accesses==0 && sdo_ops==operations*2);
    Fake_Reset();Configure();tick+=DRIVE_HEARTBEAT_STALE_TIMEOUT_MS+1;
    assert(!WheelPdoConfig_Ready(tick));
    assert(Drive_CommandEnableChecked()==CMD_RESULT_FAULT_LATCHED);
    Fake_Reset();Configure();Fake_Heartbeat(2,0x7f);Pump();assert(!WheelPdoConfig_Ready(tick));
    /* Reconfiguration cannot leave old qualification alive on cancel failure. */
    Fake_Reset();Configure();assert(WheelRpdoTx_Submit(1,2,tick)==WHEEL_RPDO_ACCEPTED);
    cancel_hold=1;assert(!AttemptConfiguration());AssertBlocked();
    assert(sdo_ops==operations && !WheelRpdoTx_Quiescent());
    assert(!strcmp(WheelPdoConfig_FailureReason(),"TX_NOT_QUIESCENT"));
    d=WheelPdoConfig_Get(1);assert(!strcmp(d.phase,"TX_CANCEL") && d.actual==3);
    puts("PASS: no 0x1018 accesses or identity data in startup/recovery; bilateral config and preserved type 254");
    puts("PASS: every SDO abort/readback failure retains phase/node/object/expected/actual/abort; NMT/offline/restart/stale block ENABLE");
}
static void StartupDisabled(void)
{
    Fake_Reset();Drive_TestResetState();Drive_SetLiftMovingCallback(NULL);
    Drive_TestSetRuntimeReady(DRIVE_STATE_DISABLED,0,0,tick);
}
static void TestAutomaticStartup(void)
{
    StartupDisabled();
    assert(!Drive_ServiceWheelTpdoStartup(tick));
    assert(s_drive.tpdo_active.startup_state==DRIVE_TPDO_STARTUP_WAIT_FRESH);
    assert(identity_accesses==0);
    Fake_Heartbeat(1,5);Fake_Heartbeat(2,5);Fake_Tpdo(0,0);Pump();
    assert(!Drive_ServiceWheelTpdoStartup(tick));
    assert(s_drive.tpdo_active.startup_state==DRIVE_TPDO_STARTUP_ARMING);
    Fake_Tpdo(0,0);Pump();Drive_ServiceWheelTpdoActiveCommand(tick);
    assert(Drive_ServiceWheelTpdoStartup(tick));
    assert(s_drive.tpdo_active.startup_state==DRIVE_TPDO_STARTUP_COMPLETE);
    assert(Canopen_GetWheelFeedbackTransport()==CANOPEN_WHEEL_FEEDBACK_TPDO_ACTIVE);
    assert(WheelPdoConfig_Ready(tick) && identity_accesses==0);
    /* Exercise the real startup caller, including propagated error reason. */
    StartupDisabled();fail_sdo=7;injected_abort=0x06040041;
    assert(!Drive_ServiceWheelTpdoStartup(tick));
    assert(s_drive.tpdo_active.startup_state==DRIVE_TPDO_STARTUP_FAILED);
    assert(strstr(Fake_Log(),"DRIVE_TPDO_STARTUP state=FAILED reason=OBJECT_NOT_MAPPABLE"));
    assert(identity_accesses==0);AssertBlocked();
    /* Successful PDO setup still cannot replace fresh bilateral feedback. */
    StartupDisabled();assert(!Drive_ServiceWheelTpdoStartup(tick));
    tick+=DRIVE_WHEEL_TPDO_ACTIVE_ARM_TIMEOUT_MS+1;
    assert(!Drive_ServiceWheelTpdoStartup(tick));
    assert(s_drive.tpdo_active.startup_state==DRIVE_TPDO_STARTUP_FAILED);
    assert(Drive_CommandEnableChecked()==CMD_RESULT_FAULT_LATCHED);
    assert(strstr(Fake_Log(),"reason=NODE1_NOT_FRESH"));
    assert(identity_accesses==0);
    puts("PASS: actual automatic Drive startup completes without identity; config cause propagates; missing TPDO blocks arming");
}
static void TestTransport(void)
{
    uint8_t bytes[4];WheelRpdoTx_Encode(-1,bytes);
    for(unsigned i=0;i<4;i++)assert(bytes[i]==255);
    WheelRpdoTx_Encode(INT32_MIN,bytes);assert(bytes[3]==128&&bytes[0]==0);
    WheelRpdoTx_Encode(0,bytes);for(unsigned i=0;i<4;i++)assert(bytes[i]==0);
    Fake_Reset();Configure();
    fifo_free=1;assert(WheelRpdoTx_Submit(-123,456,tick)==WHEEL_RPDO_BUSY);
    assert(hfdcan1.Instance->TXBRP==0);fifo_free=8;
    assert(WheelRpdoTx_Submit(-123,456,tick)==WHEEL_RPDO_ACCEPTED);
    assert(tx_headers[0].Identifier==0x301 && tx_headers[1].Identifier==0x302);
    assert(tx_headers[0].DataLength==FDCAN_DLC_BYTES_4);
    WheelRpdoTx_Encode(-123,bytes);assert(!memcmp(bytes,tx_data[0],4));
    WheelRpdoTx_Encode(456,bytes);assert(!memcmp(bytes,tx_data[1],4));
    assert(WheelRpdoTx_Submit(9,9,tick)==WHEEL_RPDO_BUSY);
    Fake_Complete(0,65530);WheelRpdoTx_Service(tick+1);assert(WheelRpdoTx_State()==WHEEL_RPDO_QUEUED);
    Fake_Complete(1,10);WheelRpdoTx_Service(tick+2);assert(WheelRpdoTx_State()==WHEEL_RPDO_SENT);
    assert(WheelRpdoTx_Get().last_skew_ticks==16);assert(WheelRpdoTx_Get().sent_pairs==1);
    WheelRpdoTx_Service(tick+3);assert(WheelRpdoTx_Get().sent_pairs==1);
    WheelRpdoTx_ConsumeSent();
    for(unsigned mask=1;mask<=2;mask++) {
        Fake_Reset();Configure();fail_add_mask=mask;
        assert(WheelRpdoTx_Submit(1,2,tick)==WHEEL_RPDO_REJECTED);
        WheelRpdoTx_Service(tick);assert(WheelRpdoTx_State()==WHEEL_RPDO_FAULT);
        assert(WheelRpdoTx_Get().sent_pairs==0);assert(hfdcan1.Instance->TXBRP==0);
        Fake_Reset();Configure();fail_request_mask=mask;
        assert(WheelRpdoTx_Submit(1,2,tick)==WHEEL_RPDO_REJECTED);
        WheelRpdoTx_Service(tick);assert(WheelRpdoTx_State()==WHEEL_RPDO_FAULT);
    }
    Fake_Reset();Configure();assert(WheelRpdoTx_Submit(1,2,tick)==WHEEL_RPDO_ACCEPTED);
    WheelRpdoTx_Service(tick+DRIVE_RUNTIME_TARGET_TIMEOUT_MS+1);
    WheelRpdoTx_Service(tick+DRIVE_RUNTIME_TARGET_TIMEOUT_MS+2);
    assert(WheelRpdoTx_State()==WHEEL_RPDO_FAULT);
    Fake_Reset();Configure();assert(WheelRpdoTx_Submit(1,2,tick)==WHEEL_RPDO_ACCEPTED);
    bus_off=1;WheelRpdoTx_Service(tick);WheelRpdoTx_Service(tick);
    assert(WheelRpdoTx_State()==WHEEL_RPDO_FAULT);assert(WheelRpdoTx_Get().bus_off_count==1);
    Fake_Reset();Configure();assert(WheelRpdoTx_Submit(1,2,tick)==WHEEL_RPDO_ACCEPTED);
    drop_event=1;Fake_Complete(0,100);Fake_Complete(1,110);
    WheelRpdoTx_Service(tick+1);assert(WheelRpdoTx_State()==WHEEL_RPDO_QUEUED);
    WheelRpdoTx_Service(tick+21);WheelRpdoTx_Service(tick+22);
    assert(WheelRpdoTx_State()==WHEEL_RPDO_FAULT);
    Fake_Reset();Configure();assert(WheelRpdoTx_Submit(1,2,tick)==WHEEL_RPDO_ACCEPTED);
    Fake_Complete(0,100);Fake_Complete(1,110);
    WheelRpdoTx_Service(tick+21);assert(WheelRpdoTx_State()!=WHEEL_RPDO_SENT);
    puts("PASS: signed/zero encoding and wheel assignment; FIFO BUSY; bilateral completion; partial queue failure; TX timeout; bus-off");
}
static void TestCancellation(void)
{
    Fake_Reset();Configure();assert(WheelRpdoTx_Submit(111,222,tick)==WHEEL_RPDO_ACCEPTED);
    cancel_hold=1;
    assert(Canopen_EmergencyZeroWheelTargets()==CANOPEN_EMERGENCY_ZERO_PENDING_ACK);
    assert(safety_zero_frames==0 && !WheelRpdoTx_Quiescent());
    /* Both already-arbitrating nonzero frames may win cancellation. Zero
     * must be sent only AFTER their TXBRP bits have cleared. */
    Fake_Complete(1,100);Fake_Complete(0,110);
    Canopen_WheelRuntimeService(tick,CANOPEN_FEEDBACK_DISPATCH_ONLY);
    assert(safety_zero_frames==2);assert(WheelRpdoTx_Get().sent_pairs==0);
    assert(WheelRpdoTx_Submit(9,9,tick)==WHEEL_RPDO_REJECTED);
    cancel_hold=0;assert(WheelRpdoTx_Rearm(tick));
    assert(WheelRpdoTx_Get().queued_pairs==1); /* no resurrection after rearm */
    Fake_Reset();Configure();assert(WheelRpdoTx_Submit(3,4,tick)==WHEEL_RPDO_ACCEPTED);
    cancel_hold=1;WheelRpdoTx_Cancel(tick);WheelRpdoTx_Service(tick+100);
    assert(!WheelRpdoTx_Quiescent());assert(!WheelRpdoTx_Rearm(tick+100));
    assert(WheelRpdoTx_Get().cancel_timeout_count==1);
    assert(Canopen_EmergencyZeroWheelTargets()==CANOPEN_EMERGENCY_ZERO_PENDING_ACK);
    assert(safety_zero_frames==0);
    Fake_Reset();Configure();assert(WheelRpdoTx_Submit(3,4,tick)==WHEEL_RPDO_ACCEPTED);
    fail_cancel=1;WheelRpdoTx_Cancel(tick);WheelRpdoTx_Service(tick);
    assert(!WheelRpdoTx_Quiescent());assert(!WheelRpdoTx_Rearm(tick));
    puts("PASS: cancellation race and hardware barrier; no zero before quiescence; no replay after recovery; cancellation timeout");
}
static float Ramp(unsigned period)
{
    MotionActuatorState_t state;MotionActuatorProposal_t p;
    Motion_ActuatorStateInit(&state);Motion_ActuatorStateCommitZero(&state,0);
    for(unsigned ms=period;ms<=100;ms+=period){
        Motion_ActuatorPrepareBodyCommand(&state,300,0,ms,200,1000,1000,MAX_TEST_MOTOR_RPM,&p);
        Motion_ActuatorCommit(&state,&p,ms);
    }
    return state.feasible_linear_mm_s;
}
static void TestTiming(void)
{
    assert(fabsf(Ramp(100)-Ramp(25))<0.0001f);
    MotionActuatorState_t s;MotionActuatorProposal_t a,b;
    Motion_ActuatorStateInit(&s);
    Motion_ActuatorPrepareBodyCommand(&s,100,0,100000,200,1000,1000,MAX_TEST_MOTOR_RPM,&a);
    assert(a.dt_ms==0 && a.feasible_linear_mm_s==0);
    Motion_ActuatorStateCommitZero(&s,UINT32_MAX-9);
    Motion_ActuatorPrepareBodyCommand(&s,100,0,15,200,1000,1000,MAX_TEST_MOTOR_RPM,&a);
    assert(a.dt_ms==25 && fabsf(a.feasible_linear_mm_s-25)<0.001f);
    Motion_ActuatorRejectBusy(&s);
    Motion_ActuatorPrepareBodyCommand(&s,100,0,15,200,1000,1000,MAX_TEST_MOTOR_RPM,&b);
    assert(a.feasible_linear_mm_s==b.feasible_linear_mm_s);
    assert(s.feasible_linear_mm_s==0);
    Motion_ActuatorPrepareBodyCommand(&s,100,0,10000,200,1000,1000,MAX_TEST_MOTOR_RPM,&b);
    assert(b.dt_ms==200);
    MotionMotorTargets_t t;Motion_CmdVelToMotorTargets(50,0,&t);
    assert(t.left_velocity_dec<0 && t.right_velocity_dec>0);
    assert(t.left_velocity_dec==-t.right_velocity_dec);
    assert(fabsf(Motion_RawVelocityDecToLogicalMotorRPM(1,t.left_velocity_dec)-t.left_motor_rpm)<0.001f);
    puts("PASS: 10/40 Hz equal real-time acceleration; first execution; rollover; delayed dt cap; BUSY retry does not commit; original signs/units");
}
void TestDriveRpdo(void);
int main(void)
{
    TestConfiguration();TestAutomaticStartup();TestTransport();TestCancellation();TestTiming();TestDriveRpdo();TestDriveRetarget();TestLiftBaseStop();TestPdoStartupRecovery();
    puts("PASS: host production-C wheel RPDO tests (hardware NOT RUN)");return 0;
}
