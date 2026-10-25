#include "drive_lift_stop_diag.h"
#include "drive_internal.h"
#include "jetson_tx.h"
#include "app_config.h"

static uint32_t last_report_ms, snapshot;
static bool report_seen;
static const char *last_reject = "NONE";

void DriveLiftStopDiag_Reset(void)
{
    report_seen = false;
    last_reject = "NONE";
}

void DriveLiftStopDiag_Report(uint32_t now, const char *event, const char *reason,
                              uint8_t node, bool static_safe, bool force)
{
    const DriveStableZeroTracker_t *z = &s_drive.stable_zero;
    const DriveConfirmedStopRequest_t *r = &s_drive.lift_stop.request;
    DriveWheelFeedbackPair_t pair = {0};
    CanopenWheelTpdoSample_t left = {0}, right = {0};
    if (z->reject_reason != DRIVE_STABLE_ZERO_REJECT_TIMEOUT) {
        last_reject = Drive_StableZeroRejectName();
    }
    if (!force && report_seen && (uint32_t)(now-last_report_ms) < 200U) {
        return;
    }
    report_seen = true;
    last_report_ms = now;
    uint32_t snap = ++snapshot;
    bool valid = Drive_GetAuthoritativeWheelFeedbackPair(now, &pair);
    (void)Canopen_GetWheelTpdoShadowSample(LEFT_NODE_ID, now, &left);
    (void)Canopen_GetWheelTpdoShadowSample(RIGHT_NODE_ID, now, &right);
    JetsonTx_Printf(JETSON_TX_PRIORITY_STATUS,
        "DRIVE_LIFT_STOP 1/7 event=%s reason=%s node=%u gen=%lu request_gen=%lu phase=%u age=%lu snap=%lu\r\n",
        event, reason, (unsigned)node, (unsigned long)s_drive.evidence.safety_generation,
        (unsigned long)r->generation, (unsigned)r->phase,
        (unsigned long)(r->active ? now-r->start_ms : 0U), (unsigned long)snap);
    JetsonTx_Printf(JETSON_TX_PRIORITY_STATUS,
        "DRIVE_LIFT_STOP 2/7 zero_valid=%u current=%u zero_gen=%lu post_required=%u static_safe=%u hw_disabled=%u hw_enabled=%u snap=%lu\r\n",
        s_drive.evidence.zero_speed.valid ? 1U : 0U,
        Drive_ZeroSpeedEvidenceIsCurrent(now) ? 1U : 0U,
        (unsigned long)s_drive.evidence.zero_speed.generation,
        s_drive.evidence.post_disable_zero_proof_required ? 1U : 0U,
        static_safe ? 1U : 0U, s_hardware_disable_confirmed ? 1U : 0U,
        s_hardware_enable_confirmed ? 1U : 0U, (unsigned long)snap);
    JetsonTx_Printf(JETSON_TX_PRIORITY_STATUS,
        "DRIVE_LIFT_STOP 3/7 post_base=%lu/%lu/%lu stable_base=%lu/%lu/%lu current=%lu/%lu/%lu snap=%lu\r\n",
        (unsigned long)s_drive.evidence.post_disable_baseline_pair_sequence,
        (unsigned long)s_drive.evidence.post_disable_baseline_left_sequence,
        (unsigned long)s_drive.evidence.post_disable_baseline_right_sequence,
        (unsigned long)z->baseline_pair_sequence, (unsigned long)z->baseline_left_sequence,
        (unsigned long)z->baseline_right_sequence, (unsigned long)pair.pair_sequence,
        (unsigned long)pair.left_node_sequence, (unsigned long)pair.right_node_sequence,
        (unsigned long)snap);
    JetsonTx_Printf(JETSON_TX_PRIORITY_STATUS,
        "DRIVE_LIFT_STOP 4/7 stable=%s reject=%s last_reject=%s samples=%lu elapsed=%lu dwell=%lu snap=%lu\r\n",
        Drive_StableZeroStateName(), Drive_StableZeroRejectName(), last_reject,
        (unsigned long)z->stable_sample_count, (unsigned long)z->overall_elapsed_ms,
        (unsigned long)z->stable_dwell_ms, (unsigned long)snap);
    for (uint8_t n = 0U; n < 2U; ++n) {
        bool seen = n ? s_drive.statusword.right_statusword_valid : s_drive.statusword.left_statusword_valid;
        uint32_t time = n ? s_drive.statusword.right_statusword_timestamp_ms : s_drive.statusword.left_statusword_timestamp_ms;
        uint32_t gen = n ? s_drive.statusword.right_statusword_generation : s_drive.statusword.left_statusword_generation;
        uint16_t word = n ? s_drive.statusword.right_statusword : s_drive.statusword.left_statusword;
        JetsonTx_Printf(JETSON_TX_PRIORITY_STATUS,
            "DRIVE_LIFT_STOP %u/7 node=%u statusword=%04X valid=%u fresh=%u status_gen=%lu age_ms=%lu snap=%lu\r\n",
            (unsigned)(n+5U), (unsigned)(n+1U), (unsigned)word, seen ? 1U : 0U,
            Drive_StatuswordEvidenceIsFresh(seen,time,gen,now) ? 1U : 0U,
            (unsigned long)gen, (unsigned long)(seen ? now-time : UINT32_MAX), (unsigned long)snap);
    }
    JetsonTx_Printf(JETSON_TX_PRIORITY_STATUS,
        "DRIVE_LIFT_STOP 7/7 rx_seq=%lu/%lu rx_age=%lu/%lu pair_valid=%u age=%lu skew=%lu snap=%lu\r\n",
        (unsigned long)left.sequence, (unsigned long)right.sequence,
        (unsigned long)(left.seen ? now-left.timestamp_ms : UINT32_MAX),
        (unsigned long)(right.seen ? now-right.timestamp_ms : UINT32_MAX),
        valid ? 1U : 0U, (unsigned long)pair.pair_age_ms,
        (unsigned long)pair.pair_skew_ms, (unsigned long)snap);
}
