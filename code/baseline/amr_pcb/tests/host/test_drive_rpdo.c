#include "fake_bus.h"
#include "wheel_pdo_config.h"
#include "wheel_rpdo_tx.h"
#include "drive_target.h"
#include "drive_internal.h"
#include "app_config.h"
#include "odom.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static bool lift_moving;
static bool LiftMoving(void) { return lift_moving; }
static void Pump(void) { Canopen_ProcessRx(tick,128); }
static void Feedback(int32_t left,int32_t right)
{
    Fake_Heartbeat(1,5);Fake_Heartbeat(2,5);Fake_Tpdo(left,right);
}
static void ReadyAt(uint32_t start)
{
    DriveTarget_Cancel(tick);
    Fake_Reset();Drive_TestResetState();lift_moving=false;
    tick=start;
    Drive_SetLiftMovingCallback(LiftMoving);
    memset(&s_drive.tpdo_active,0,sizeof(s_drive.tpdo_active));
    assert(WheelPdoConfig_Configure());
    assert(Canopen_ConfigureWheelTpdoFeedbackNode(1));
    assert(Canopen_ConfigureWheelTpdoFeedbackNode(2));
    assert(identity_accesses==0); /* device has no 0x1018, including TPDO setup */
    Pump();Fake_Tpdo(0,0);Pump();
    assert(Canopen_BeginWheelTpdoActiveArm(tick));
    Fake_Tpdo(0,0);Pump();
    assert(Canopen_GetWheelFeedbackTransport()==CANOPEN_WHEEL_FEEDBACK_TPDO_ACTIVE);
    tick_step=0;assert(WheelRpdoTx_Rearm(tick));
    Canopen_HeartbeatResetForExplicitEnable(tick);
    assert(Canopen_HeartbeatSetConfigured(1,tick));
    assert(Canopen_HeartbeatSetConfigured(2,tick));
    Feedback(0,0);Pump();tick++;Feedback(0,0);Pump();
    assert(Canopen_HeartbeatIsQualified(1)&&Canopen_HeartbeatIsQualified(2));
    Drive_TestSetRuntimeReady(DRIVE_STATE_ENABLED_IDLE,0,0,tick);
    s_drive.timing.control_last_ms=tick;
    s_yaw_pi_enabled=false;
    Feedback(0,0);Drive_ControlLoop_Update();
}
static void Ready(void) { ReadyAt(0); }
static void Pending(void)
{
    Drive_CommandSetCmdVel(80,0);
    tick+=25;Feedback(0,0);Drive_ControlLoop_Update();
    assert(DriveTarget_Pending());
    assert(WheelRpdoTx_State()==WHEEL_RPDO_QUEUED);
    assert(!Drive_TestBaseHardwareSafeForLift(tick));
}
static void TestCommit(void)
{
    Ready();uint32_t commits=s_drive.control.actuator.state.submit_count;
    uint32_t sdobefore=sdo_ops;Pending();
    assert(s_drive.control.actuator.state.submit_count==commits);
    assert(sdo_ops==sdobefore);assert(s_drive.command.last_left_cmd_dec==0);
    Fake_Complete(0,100);tick++;Drive_ControlLoop_Update();
    assert(s_drive.control.actuator.state.submit_count==commits);
    Fake_Complete(1,140);tick++;Drive_ControlLoop_Update();
    assert(s_drive.control.actuator.state.submit_count==commits+1);
    assert(s_drive.command.last_left_cmd_dec<0 && s_drive.command.last_right_cmd_dec>0);
    for(unsigned i=0;i<5;i++){tick++;Drive_ControlLoop_Update();}
    assert(s_drive.control.actuator.state.submit_count==commits+1);
    assert(sdo_ops==sdobefore);
    DriveTarget_PrintDiagnostics(123);

    /* PI transaction uses a real prepared proposal and keeps newer feedback
     * pending when the older in-flight proposal commits. */
    Ready();tick+=25;
    Motion_ActuatorPrepareBodyCommand(&s_drive.control.actuator.state,80,0,tick,
        DRIVE_ACTUATOR_RATE_MAX_DT_MS,LINEAR_ACCEL_LIMIT_MM_S2,
        ANGULAR_ACCEL_LIMIT_MRAD_S2,MAX_TEST_MOTOR_RPM,&s_drive.control.actuator.final_proposal);
    s_drive.control.actuator.baseline_proposal=s_drive.control.actuator.final_proposal;
    s_drive.control.yaw.proposal=Motion_YawRatePIPrepare(&s_drive.control.yaw.pi,
        &s_yaw_rate_pi_config,25,0,&s_drive.feedback.chassis,true);
    assert(s_drive.control.yaw.proposal.valid);
    MotionTrackingLimits_t limits={0};bool pi=false;
    DriveTarget_Submit(tick,true,limits);
    assert(!s_drive.control.yaw.pi.feedback_sequence_valid);
    assert(!DriveTarget_Service(tick,&pi));assert(!pi);
    uint32_t seq=s_drive.control.yaw.proposal.feedback_sequence;
    s_drive.feedback.chassis.sequence++;s_drive.feedback.pair_pending=true;
    Fake_Complete(0,100);Fake_Complete(1,120);tick++;
    assert(DriveTarget_Service(tick,&pi)&&pi);
    assert(s_drive.control.yaw.pi.last_feedback_sequence==seq);
    assert(s_drive.feedback.pair_pending);
    assert(!DriveTarget_Service(tick,&pi));
    puts("PASS: real Drive loop queues without commit; bilateral TX commits once; PI snapshot and newer feedback preserved; no normal background SDO");
}
static void TestStopAndFaults(void)
{
    for(unsigned cause=0;cause<7;cause++){
        Ready();Pending();uint32_t commits=s_drive.control.actuator.state.submit_count;
        cancel_hold=1;
        switch(cause){
            case 0: Drive_CommandStop();break;
            case 1: s_drive.command.last_jetson_cmd_ms=tick-JETSON_CMD_TIMEOUT_MS-1;break;
            case 2: lift_moving=true;break;
            case 3: Fake_Heartbeat(1,0);break;
            case 4: {uint8_t emcy[8]={0x10,0x23};Fake_Rx(0x81,emcy,8);break;}
            case 5: bus_off=1;break;
            default: tick+=DRIVE_WHEEL_FEEDBACK_STALE_MS+1;break;
        }
        Drive_ControlLoop_Update();
        assert(!DriveTarget_Pending());assert(!s_drive.command.jetson_cmd_active);
        assert(s_drive.control.actuator.state.submit_count==commits);
        assert(!Drive_TestZeroEvidenceValid());
        assert(!Drive_TestBaseHardwareSafeForLift(tick));
        assert(!Drive_NormalMotionAllowed());assert(safety_zero_frames==0);
        Fake_Complete(0,100);Fake_Complete(1,110);cancel_hold=0;bus_off=0;
        tick++;Feedback(0,0);Drive_ControlLoop_Update();
        assert(s_drive.control.actuator.state.submit_count==commits);
        assert(WheelRpdoTx_Get().sent_pairs==0);
        uint32_t queued=WheelRpdoTx_Get().queued_pairs;
        for(unsigned i=0;i<10;i++){tick++;Feedback(0,0);Drive_ControlLoop_Update();}
        assert(WheelRpdoTx_Get().queued_pairs==queued);
        if(cause<3)assert(safety_zero_frames>=2); /* Lift cannot starve STOP */
    }
    Ready();Pending();Drive_CommandStop();
    assert(!Drive_TestZeroEvidenceValid());
    for(unsigned i=0;i<10;i++){tick+=10;Feedback(-100000,100000);Drive_ControlLoop_Update();}
    assert(!Drive_TestZeroEvidenceValid());assert(s_drive.core.state==DRIVE_STATE_STOPPING);
    /* ACK alone never confirms stopping. New coherent zero TPDO samples do. */
    for(unsigned i=0;i<70 && s_drive.safety.transition.active;i++){
        tick+=20;Feedback(0,0);Drive_ControlLoop_Update();
    }
    assert(!s_drive.safety.transition.active);
    assert(Drive_TestZeroEvidenceValid());assert(s_drive.core.state==DRIVE_STATE_ENABLED_IDLE);
    assert(WheelRpdoTx_Get().queued_pairs==1);
    puts("PASS: pending STOP/watchdog/Lift/boot-up/EMCY/bus-off preemption; no stale replay; ACK alone not stopped; fresh TPDO measured stop");
}
static void TestTpdoOdom(void)
{
    Ready();MotionMotorTargets_t t;Motion_CmdVelToMotorTargets(40,0,&t);
    memset(&g_odom,0,sizeof(g_odom));g_odom.initialized=true;
    tick+=20;Feedback(t.left_velocity_dec,t.right_velocity_dec);Drive_ControlLoop_Update();
    float before=g_odom.x_mm;
    tick+=20;Feedback(t.left_velocity_dec,t.right_velocity_dec);Drive_ControlLoop_Update();
    assert(g_odom.x_mm>before && fabsf(g_odom.y_mm)<0.001f);
    DriveWheelFeedbackPair_t p;assert(Drive_GetAuthoritativeWheelFeedbackPair(tick,&p));
    uint32_t seq=p.pair_sequence;uint8_t b[4];WheelRpdoTx_Encode(t.left_velocity_dec,b);
    tick+=10;Fake_Rx(0x181,b,4);Drive_ControlLoop_Update();
    assert(Drive_GetAuthoritativeWheelFeedbackPair(tick,&p));assert(p.pair_sequence==seq);
    tick+=31;WheelRpdoTx_Encode(t.right_velocity_dec,b);Fake_Rx(0x182,b,4);Drive_ControlLoop_Update();
    assert(Drive_GetAuthoritativeWheelFeedbackPair(tick,&p));assert(p.pair_sequence==seq);
    before=g_odom.x_mm;tick+=DRIVE_WHEEL_FEEDBACK_STALE_MS+1;
    Fake_Heartbeat(1,5);Fake_Heartbeat(2,5);Drive_ControlLoop_Update();
    assert(!Drive_GetAuthoritativeWheelFeedbackPair(tick,&p));
    assert(g_odom.x_mm==before);assert(!Drive_NormalMotionAllowed());
    assert(!Canopen_RuntimeSdo606cPollEnabled());
    assert(Drive_CommandEnableChecked()==CMD_RESULT_FAULT_LATCHED);
    puts("PASS: actual TPDO pairing requires both nodes; skew rejection; stale fail-closed/no SDO fallback; real ODOM integration");
}
static void TestCadence(void)
{
    Ready();uint32_t sdos=sdo_ops, cycles=0, start=tick;
    for(uint32_t elapsed=1;elapsed<=1000;elapsed++) {
        tick=start+elapsed;
        if((elapsed%20)==0)Feedback(0,0);
        if((elapsed%25)==1)Drive_CommandSetCmdVel(80,0);
        if(DriveTarget_Pending()){Fake_Complete(0,(uint16_t)elapsed);Fake_Complete(1,(uint16_t)(elapsed+10));}
        uint32_t previous=s_drive.timing.control_last_ms;
        Drive_ControlLoop_Update();
        if(previous!=s_drive.timing.control_last_ms)cycles++;
    }
    assert(cycles==40);assert(sdo_ops==sdos);
    assert(WheelRpdoTx_Get().sent_pairs<cycles);
    /* Long idle before the first command must not become a 200 ms ramp step. */
    Ready();
    for(unsigned i=0;i<50;i++){tick+=20;Feedback(0,0);Drive_ControlLoop_Update();}
    Drive_CommandSetCmdVel(80,0);tick+=25;Feedback(0,0);Drive_ControlLoop_Update();
    assert(DriveTarget_Pending());
    assert(s_drive.control.actuator.final_proposal.dt_ms==25);
    puts("PASS: cooperative production control executes 40 ticks/second; dedup reduces frames independently; first departure excludes idle time");
}
static void TestPiAndPermanentBusFault(void)
{
    Ready();
    for(unsigned n=0;n<5;n++) {
        s_drive.feedback.chassis.sequence++;
        s_drive.feedback.chassis.timestamp_ms+=25;
        MotionYawRatePIProposal_t p=Motion_YawRatePIPrepare(&s_drive.control.yaw.pi,
            &s_yaw_rate_pi_config,25,100,&s_drive.feedback.chassis,true);
        assert(p.valid);
        Motion_YawRatePICommit(&s_drive.control.yaw.pi,&s_yaw_rate_pi_config,&p,
                               p.requested_correction_mrad_s,false);
    }
    s_drive.feedback.chassis.sequence++;s_drive.feedback.chassis.timestamp_ms+=25;
    s_drive.control.yaw.proposal=Motion_YawRatePIPrepare(&s_drive.control.yaw.pi,
        &s_yaw_rate_pi_config,25,100,&s_drive.feedback.chassis,true);
    assert(s_drive.control.yaw.proposal.status==MOTION_YAW_PI_PROPOSAL_READY);
    uint32_t count=s_drive.control.yaw.pi.update_count;
    float integral=s_drive.control.yaw.pi.integral_mrad;
    s_drive.control.yaw.proposed_submitted_correction_mrad_s=
        s_drive.control.yaw.proposal.requested_correction_mrad_s;
    MotionTrackingLimits_t limits={0};bool committed=false;
    fifo_free=1;DriveTarget_Submit(tick,true,limits);
    assert(s_drive.control.yaw.pi.integral_mrad==integral);
    assert(s_drive.control.yaw.pi.update_count==count);
    fifo_free=8;DriveTarget_Submit(tick,true,limits);
    assert(s_drive.control.yaw.pi.update_count==count);
    Fake_Complete(0,100);Fake_Complete(1,120);tick++;
    assert(DriveTarget_Service(tick,&committed)&&committed);
    assert(s_drive.control.yaw.pi.update_count==count+1);
    assert(s_drive.control.yaw.pi.integral_mrad!=integral);
    assert(!DriveTarget_Service(tick,&committed));
    assert(s_drive.control.yaw.pi.update_count==count+1);
    /* Retarget/reset while in flight must not revive the old PI integrator. */
    DriveTarget_Submit(tick,true,limits);Motion_YawRatePIReset(&s_drive.control.yaw.pi);
    Fake_Complete(0,200);Fake_Complete(1,220);tick++;
    assert(DriveTarget_Service(tick,&committed)&&!committed);
    assert(!s_drive.control.yaw.pi.feedback_sequence_valid);
    Ready();Pending();bus_off=1;Drive_CommandStop();
    for(unsigned n=0;n<50;n++){tick+=50;Drive_ControlLoop_Update();}
    assert(!Drive_TestZeroEvidenceValid());assert(!Drive_TestHardwareDisableConfirmed());
    assert(safety_zero_frames==0);assert(!Drive_NormalMotionAllowed());
    puts("PASS: READY PI integral commits once after TX, BUSY does not integrate, retarget reset survives; persistent CAN fault never confirms parking");
}
void TestDriveRpdo(void)
{
    TestCommit();TestStopAndFaults();TestTpdoOdom();TestCadence();TestPiAndPermanentBusFault();
}

/* Shared production startup fixture for the retarget regressions. */
void TestDriveReady(void) { Ready(); }
void TestDriveReadyAt(uint32_t start) { ReadyAt(start); }
