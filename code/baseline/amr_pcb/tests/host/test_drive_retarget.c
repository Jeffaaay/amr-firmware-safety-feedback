#include "fake_bus.h"
#include "drive_internal.h"
#include "drive_target.h"
#include "wheel_rpdo_tx.h"
#include "app_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void Feedback(void)
{
    Fake_Heartbeat(1,5);Fake_Heartbeat(2,5);
    Fake_Tpdo(s_drive.command.last_left_cmd_dec,s_drive.command.last_right_cmd_dec);
}
static void Complete(void)
{
    if(DriveTarget_Pending()) {
        Fake_Complete(0,(uint16_t)(tick*500));
        Fake_Complete(1,(uint16_t)(tick*500+50));
        Drive_ControlLoop_Update();
    }
}
static void Step(float command,unsigned elapsed,bool complete)
{
    Drive_CommandSetCmdVel(command,0);
    tick+=elapsed;Feedback();Drive_ControlLoop_Update();
    if(complete)Complete();
}
static void Hold80(void)
{
    TestDriveReady();
    for(unsigned n=0;n<16;n++)Step(80,25,true);
    assert(fabsf(s_drive.control.actuator.state.feasible_linear_mm_s-80)<0.001f);
    assert(!DriveTarget_Pending());
}
static void TestReportedCases(void)
{
    bool ok=true;
    const float targets[]={200,0}, expected[]={105,55};
    for(unsigned n=0;n<2;n++) {
        Hold80();uint32_t committed=s_drive.control.actuator.state.timestamp_ms;
        Step(targets[n],25,false);
        MotionActuatorProposal_t p=s_drive.control.actuator.final_proposal;
        printf("RETARGET 80->%.0f: expected=%.0f actual=%.3f last_commit_age=%u proposal_dt=%u state=%s\n",
               targets[n],expected[n],p.feasible_linear_mm_s,(unsigned)(tick-committed),
               (unsigned)p.dt_ms,Drive_StateName(s_drive.core.state));
        ok=ok && fabsf(p.feasible_linear_mm_s-expected[n])<0.001f &&
           p.dt_ms==25 && s_drive.core.state!=DRIVE_STATE_STOPPING;
    }
    fflush(stdout);assert(ok);
}

static void TestHoldsAndSigns(void)
{
    Hold80();
    uint32_t committed=s_drive.control.actuator.state.timestamp_ms;
    uint32_t count=s_drive.control.actuator.state.submit_count;
    uint32_t pi=s_drive.control.yaw.pi.update_count;
    WheelRpdoDiagnostic before=WheelRpdoTx_Get();
    for(unsigned n=0;n<12;n++)Step(80,25,true);
    assert(s_drive.control.actuator.state.timestamp_ms==committed);
    assert(s_drive.control.actuator.state.submit_count==count);
    assert(s_drive.control.yaw.pi.update_count==pi);
    assert(WheelRpdoTx_Get().queued_pairs==before.queued_pairs);
    assert(WheelRpdoTx_Get().sent_pairs==before.sent_pairs);
    assert(DriveTarget_ShapingState().timestamp_ms==tick);
    Step(-200,25,false);
    assert(fabsf(s_drive.control.actuator.final_proposal.feasible_linear_mm_s-55)<0.001f);
    Complete();
    for(unsigned n=0;n<20;n++)Step(-80,25,true);
    assert(fabsf(s_drive.control.actuator.state.feasible_linear_mm_s+80)<0.001f);
    Step(200,25,false);
    assert(fabsf(s_drive.control.actuator.final_proposal.feasible_linear_mm_s+55)<0.001f);
    assert(s_drive.core.state==DRIVE_STATE_RUNNING);
    puts("PASS: repeated held commands spend shaping time only; TX timestamp/count/PI unchanged; positive and negative reversals slew through zero");
}

static void TestBusyAndPendingLatest(void)
{
    Hold80();
    uint32_t shaping=DriveTarget_ShapingState().timestamp_ms;
    uint32_t committed=s_drive.control.actuator.state.timestamp_ms;
    uint32_t count=s_drive.control.actuator.state.submit_count;
    fifo_free=1;Step(200,25,false);
    assert(!DriveTarget_Pending());
    assert(DriveTarget_ShapingState().timestamp_ms==shaping);
    assert(s_drive.control.actuator.state.timestamp_ms==committed);
    assert(s_drive.control.actuator.state.submit_count==count);
    assert(s_drive.control.actuator.state.feasible_linear_mm_s==80);
    fifo_free=8;Step(-80,25,false);
    assert(DriveTarget_Pending());
    assert(s_drive.control.actuator.final_proposal.dt_ms==50);
    assert(fabsf(s_drive.control.actuator.final_proposal.feasible_linear_mm_s-30)<0.001f);
    Complete();assert(s_drive.control.actuator.state.submit_count==count+1);

    Hold80();Step(200,25,false); /* 105 queued; real state still 80. */
    count=s_drive.control.actuator.state.submit_count;
    committed=s_drive.control.actuator.state.timestamp_ms;
    shaping=DriveTarget_ShapingState().timestamp_ms;
    Drive_CommandSetCmdVel(-80,0);tick+=5;Feedback();Drive_ControlLoop_Update();
    assert(DriveTarget_ShapingState().timestamp_ms==shaping);
    assert(s_drive.control.actuator.state.timestamp_ms==committed);
    assert(s_drive.control.actuator.state.submit_count==count);
    Fake_Complete(0,100);Drive_ControlLoop_Update();
    assert(s_drive.control.actuator.state.submit_count==count);
    Fake_Complete(1,150);Drive_ControlLoop_Update();
    assert(s_drive.control.actuator.state.submit_count==count+1);
    assert(fabsf(s_drive.control.actuator.state.feasible_linear_mm_s-105)<0.001f);
    tick+=20;Feedback();Drive_ControlLoop_Update();
    assert(DriveTarget_Pending());
    assert(s_drive.control.actuator.final_proposal.dt_ms==25);
    assert(fabsf(s_drive.control.actuator.final_proposal.feasible_linear_mm_s-80)<0.001f);
    Complete();assert(s_drive.control.actuator.state.submit_count==count+2);
    puts("PASS: BUSY/pending do not advance unsent execution or shaping state; retry uses latest target and real dt; each bilateral completion commits once");
}

static void TestWrapDepartureAndStop(void)
{
    TestDriveReady();uint32_t setup_elapsed=tick;
    /* Start the entire fixture near wrap; do not teleport a qualified TPDO
     * epoch backwards, which correctly causes a continuity fault. */
    TestDriveReadyAt(UINT32_MAX-setup_elapsed-350);
    for(unsigned n=0;n<16;n++)Step(80,25,true);
    assert(tick<100);Step(200,25,false);
    assert(s_drive.control.actuator.final_proposal.dt_ms==25);
    assert(fabsf(s_drive.control.actuator.final_proposal.feasible_linear_mm_s-105)<0.001f);

    TestDriveReady();
    for(unsigned n=0;n<50;n++){tick+=20;Feedback();Drive_ControlLoop_Update();}
    uint32_t committed=s_drive.control.actuator.state.timestamp_ms;
    uint32_t count=s_drive.control.actuator.state.submit_count;
    Drive_CommandSetCmdVel(200,0);
    assert(s_drive.control.actuator.state.timestamp_ms==committed);
    assert(s_drive.control.actuator.state.submit_count==count);
    Step(200,70,false); /* Scheduler delay is real time, not hardcoded 25 ms. */
    assert(s_drive.control.actuator.final_proposal.dt_ms==70);
    assert(fabsf(s_drive.control.actuator.final_proposal.feasible_linear_mm_s-70)<0.001f);

    Hold80();Step(0,25,false);
    assert(s_drive.core.state==DRIVE_STATE_RUNNING && DriveTarget_Pending());
    uint32_t now=tick;cancel_hold=1;Drive_CommandStop();
    assert(tick==now);assert(!DriveTarget_Pending());
    assert(s_drive.core.state==DRIVE_STATE_STOPPING);
    assert(WheelRpdoTx_State()==WHEEL_RPDO_CANCELLING);
    assert(!Drive_TestZeroEvidenceValid());
    puts("PASS: retarget across tick wrap; idle departure preserves TX record; delayed scheduling uses 70 ms; explicit STOP preempts ordinary deceleration immediately");
}

static void Trajectory(unsigned period,float samples[14])
{
    TestDriveReady();unsigned sample=0;
    for(unsigned elapsed=0;elapsed<1400;elapsed+=period) {
        float command=elapsed<400?80:(elapsed<800?200:(elapsed<1300?-80:0));
        Step(command,period,true);
        if((elapsed+period)%100==0) {
            /* At final zero STOP owns the zero proposal before safety ACK. */
            samples[sample++]=s_drive.core.state==DRIVE_STATE_STOPPING ?
                s_drive.control.actuator.final_proposal.feasible_linear_mm_s :
                s_drive.control.actuator.state.feasible_linear_mm_s;
        }
    }
    assert(sample==14);
}
static void TestTenFortyWithHolds(void)
{
    float ten[14],forty[14];
    const float expected[]={80,80,80,80,180,200,200,200,100,0,-80,-80,-80,0};
    Trajectory(100,ten);Trajectory(25,forty);
    for(unsigned i=0;i<14;i++) {
        assert(fabsf(ten[i]-forty[i])<0.001f);
        assert(fabsf(forty[i]-expected[i])<0.001f);
    }
    puts("PASS: production 10/40 Hz equal elapsed-time trajectory with acceleration, constant holds, reversal and ordinary zero deceleration");
}
void TestDriveRetarget(void)
{
    TestReportedCases();
    TestHoldsAndSigns();TestBusyAndPendingLatest();
    TestWrapDepartureAndStop();TestTenFortyWithHolds();
}
