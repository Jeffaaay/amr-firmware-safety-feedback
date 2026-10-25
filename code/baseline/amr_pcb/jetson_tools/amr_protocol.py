#!/usr/bin/env python3
"""Shared AMR USART3 protocol parsers, transactions, and serial session (v5).

This module is the single production parser/transaction library used by:
  - jetson_tools/amr_test_runner.py   (identity/uart-soak/drive-reset/battery/pi-bench)
  - jetson_tools/amr_lift_recovery_test.py   (lift recovery gate)
  - jetson_tools/amr_pi_uart_hardware_test.py (pi-preflight)

Protocol v5 binding model (fail closed):
  - ODOM has one grammar. Its signed wheel fields are milli-RPM; float RPM is
    exposed only by dividing those wire integers by 1000.0.
  - The protocol-v4 request/snapshot binding rules remain unchanged.
  - The host cannot flush frames still queued inside the STM32 TX manager,
    so local RX-buffer clearing alone can never prove freshness. Every
    stateful command and structured query therefore carries a host-generated
    req=<8 hex> nonce that the firmware echoes on every response line. The
    host accepts ONLY responses echoing its own nonce.
  - Reset/refresh transactions additionally carry a per-command monotonic
    gen=/txn=; multi-line snapshots share a snap= id. A snapshot is used
    only when every declared line arrived exactly once with one snap= and
    the caller's req=.
  - v5 hosts refuse protocol 4 firmware; there is no silent degradation or
    ambiguous interpretation of ODOM wheel units.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import re
import secrets
import time


REQUIRED_PROTOCOL = 5

# --------------------------------------------------------------------------
# Strict line grammars
# --------------------------------------------------------------------------

_SNAP_TAIL = r" snap=(?P<snap>\d+) req=(?P<request>[0-9A-F]{8})$"
_REQ_TAIL = r" req=(?P<request>[0-9A-F]{8})$"

SNAP_TAIL_RE = re.compile(_SNAP_TAIL)

SUMMARY_RE = re.compile(
    r"^DRIVE_BASE_SAFE "
    r"safe=(?P<safe>\d+) "
    r"state=(?P<state>\S+) "
    r"active=(?P<active>\d+) "
    r"en=(?P<enable>\d+) "
    r"dis=(?P<disable>\d+) "
    r"drive=(?P<drive_enabled>\d+) "
    r"target_zero=(?P<target_zero>\d+) "
    r"pending=(?P<pending>[0-9A-Fa-f]{2}) "
    r"zero=(?P<zero>\d+)" + _SNAP_TAIL
)
STATUS_RE = re.compile(
    r"^DRIVE_BASE_SAFE_SW "
    r"valid=(?P<left_valid>\d+)/(?P<right_valid>\d+) "
    r"fresh=(?P<left_fresh>\d+)/(?P<right_fresh>\d+) "
    r"sw=(?P<left_status>[0-9A-Fa-f]{4})/"
    r"(?P<right_status>[0-9A-Fa-f]{4}) "
    r"age=(?P<left_age_ms>\d+)/(?P<right_age_ms>\d+) "
    r"seq=(?P<left_sequence>\d+)/(?P<right_sequence>\d+) "
    r"gen=(?P<left_generation>\d+)/(?P<right_generation>\d+)" + _SNAP_TAIL
)
ZERO_RE = re.compile(
    r"^DRIVE_BASE_SAFE_ZERO "
    r"valid=(?P<valid>\d+) "
    r"age=(?P<age_ms>\d+) "
    r"gen=(?P<generation>\d+)/(?P<safety_generation>\d+)" + _SNAP_TAIL
)
FAULT_RESET_STARTED_RE = re.compile(
    r"^DRIVE_FAULT_RESET_SAFE state=STARTED gen=(?P<generation>\d+)"
    + _REQ_TAIL
)
FAULT_RESET_PENDING_RE = re.compile(
    r"^DRIVE_FAULT_RESET_SAFE state=PENDING "
    r"phase=(?P<phase>[A-Z0-9_]+) node=(?P<node>\d+) "
    r"gen=(?P<generation>\d+)" + _REQ_TAIL
)
FAULT_RESET_COMPLETE_RE = re.compile(
    r"^DRIVE_FAULT_RESET_SAFE state=COMPLETE "
    r"cleared=(?P<left_cleared>[01])/(?P<right_cleared>[01]) "
    r"gen=(?P<generation>\d+)" + _REQ_TAIL
)
FAULT_RESET_FAILED_RE = re.compile(
    r"^DRIVE_FAULT_RESET_SAFE state=FAILED "
    r"reason=(?P<reason>[A-Z0-9_]+) node=(?P<node>\d+) "
    r"phase=(?P<phase>[A-Z0-9_]+) "
    r"abort=(?P<abort>0x[0-9A-Fa-f]{8}) "
    r"timeouts=(?P<timeouts>\d+) "
    r"error=(?P<error>0x[0-9A-Fa-f]{4})/(?P<error_valid>[01]) "
    r"reg=(?P<register>0x[0-9A-Fa-f]{2})/(?P<register_valid>[01]) "
    r"gen=(?P<generation>\d+)" + _REQ_TAIL
)
REFRESH_STARTED_RE = re.compile(
    r"^DRIVE_BASE_SAFE_REFRESH state=STARTED "
    r"gen=(?P<generation>\d+) txn=(?P<txn>\d+)" + _REQ_TAIL
)
REFRESH_PENDING_RE = re.compile(
    r"^DRIVE_BASE_SAFE_REFRESH state=PENDING "
    r"phase=(?P<phase>SETTLING|CONFIRMING|LEFT_STATUS|RIGHT_STATUS|ZERO_FEEDBACK) "
    r"txn=(?P<txn>\d+)" + _REQ_TAIL
)
REFRESH_COMPLETE_RE = re.compile(
    r"^DRIVE_BASE_SAFE_REFRESH state=COMPLETE "
    r"safe=(?P<safe>\d+) gen=(?P<generation>\d+) "
    r"txn=(?P<txn>\d+)" + _REQ_TAIL
)
REFRESH_FAILED_RE = re.compile(
    r"^DRIVE_BASE_SAFE_REFRESH state=FAILED "
    r"reason=(?P<reason>[A-Z0-9_]+) node=(?P<node>\d+) "
    r"txn=(?P<txn>\d+)" + _REQ_TAIL
)
TPDO_CONFIG_STARTED_RE = re.compile(
    r"^DRIVE_TPDO_CONFIG state=STARTED node=(?P<node>1|2|all)"
    + _REQ_TAIL
)
TPDO_CONFIG_COMPLETE_RE = re.compile(
    r"^DRIVE_TPDO_CONFIG state=COMPLETE node=(?P<node>1|2|all) "
    r"configured=(?P<left_configured>[01])/(?P<right_configured>[01]) "
    r"mode=(?P<mode>[A-Z0-9_]+)" + _REQ_TAIL
)
TPDO_CONFIG_FAILED_RE = re.compile(
    r"^DRIVE_TPDO_CONFIG state=FAILED reason=(?P<reason>[A-Z0-9_]+) "
    r"node=(?P<node>1|2|all)"
    r"(?P<details>(?: [A-Za-z0-9_]+=[^\s]+)*)" + _REQ_TAIL
)
TPDO_STATUS_1_RE = re.compile(
    r"^DRIVE_TPDO_STATUS 1/5 mode=(?P<mode>[A-Z0-9_]+) "
    r"configured=(?P<left_configured>[01])/(?P<right_configured>[01]) "
    r"cob=(?P<left_cob>[0-9A-F]{3})/(?P<right_cob>[0-9A-F]{3}) "
    r"mapping=(?P<left_mapping>[0-9A-F]{8})/"
    r"(?P<right_mapping>[0-9A-F]{8}) "
    r"event_ms=(?P<event_ms>\d+) type=(?P<transmission_type>[0-9A-F]{2}) "
    r"inhibit=(?P<inhibit>\d+)" + _REQ_TAIL
)
TPDO_STATUS_NODE_RES = (
    re.compile(
        r"^DRIVE_TPDO_STATUS 2/5 n=1 st=(?P<state>[A-Z_]+) "
        r"raw=(?P<raw>-?\d+) a=(?P<age>\d+) q=(?P<sequence>\d+) "
        r"f=(?P<frames>\d+) iu=(?P<ignored_unconfigured>\d+) "
        r"im=(?P<ignored_mode>\d+) g=(?P<max_gap>\d+) "
        r"d=(?P<invalid_dlc>\d+) t=(?P<stale>\d+) "
        r"r=(?P<recovery>\d+)" + _REQ_TAIL),
    re.compile(
        r"^DRIVE_TPDO_STATUS 3/5 n=2 st=(?P<state>[A-Z_]+) "
        r"raw=(?P<raw>-?\d+) a=(?P<age>\d+) q=(?P<sequence>\d+) "
        r"f=(?P<frames>\d+) iu=(?P<ignored_unconfigured>\d+) "
        r"im=(?P<ignored_mode>\d+) g=(?P<max_gap>\d+) "
        r"d=(?P<invalid_dlc>\d+) t=(?P<stale>\d+) "
        r"r=(?P<recovery>\d+)" + _REQ_TAIL),
)
TPDO_STATUS_COMPARE_RES = (
    re.compile(
        r"^DRIVE_TPDO_STATUS 4/5 n=1 sdo=(?P<sdo>-?\d+) "
        r"sdo_age=(?P<sdo_age>\d+) tpdo=(?P<tpdo>-?\d+) "
        r"tpdo_age=(?P<tpdo_age>\d+) diff=(?P<diff>-?\d+)"
        + _REQ_TAIL),
    re.compile(
        r"^DRIVE_TPDO_STATUS 5/5 n=2 sdo=(?P<sdo>-?\d+) "
        r"sdo_age=(?P<sdo_age>\d+) tpdo=(?P<tpdo>-?\d+) "
        r"tpdo_age=(?P<tpdo_age>\d+) diff=(?P<diff>-?\d+)"
        + _REQ_TAIL),
)
TPDO_ACTIVE_EVENT_RES = (
    re.compile(r"^DRIVE_TPDO_ACTIVE state=STARTED" + _REQ_TAIL),
    re.compile(
        r"^DRIVE_TPDO_ACTIVE state=COMPLETE req=(?P<request>[0-9A-F]{8}) "
        r"mode=(?P<mode>TPDO_ACTIVE) authority=(?P<authority>TPDO)$"),
    re.compile(
        r"^DRIVE_TPDO_ACTIVE state=FAILED req=(?P<request>[0-9A-F]{8}) "
        r"reason=(?P<reason>[A-Z0-9_]+)$"),
)
TPDO_ACTIVE_DISABLE_EVENT_RES = (
    re.compile(r"^DRIVE_TPDO_ACTIVE_DISABLE state=STARTED" + _REQ_TAIL),
    re.compile(
        r"^DRIVE_TPDO_ACTIVE_DISABLE state=COMPLETE "
        r"req=(?P<request>[0-9A-F]{8}) "
        r"mode=(?P<mode>TPDO_SHADOW) authority=(?P<authority>SDO)$"),
    re.compile(
        r"^DRIVE_TPDO_ACTIVE_DISABLE state=FAILED "
        r"req=(?P<request>[0-9A-F]{8}) "
        r"reason=(?P<reason>[A-Z0-9_]+)$"),
)
TPDO_ACTIVE_STATUS_RES = (
    re.compile(
        r"^DRIVE_TPDO_ACTIVE_STATUS 1/8 mode=(?P<mode>[A-Z0-9_]+) "
        r"authority=(?P<authority>SDO|TPDO) "
        r"odom_authority=(?P<odom_authority>SDO|TPDO) "
        r"configured=(?P<left_configured>[01])/(?P<right_configured>[01])"
        + _REQ_TAIL),
    re.compile(
        r"^DRIVE_TPDO_ACTIVE_STATUS 2/8 drive_state=(?P<drive_state>[A-Z_]+) "
        r"software_enabled=(?P<software_enabled>[01]) "
        r"hardware_enable_confirmed=(?P<hardware_enable>[01]) "
        r"hardware_disable_confirmed=(?P<hardware_disable>[01])" + _REQ_TAIL),
    re.compile(
        r"^DRIVE_TPDO_ACTIVE_STATUS 3/8 "
        r"target_zero=(?P<target_zero>[01]) lift_stopped=(?P<lift_stopped>[01]) "
        r"transition_active=(?P<transition_active>[01]) "
        r"fault_active=(?P<fault_active>[01]) estop_active=(?P<estop_active>[01])"
        + _REQ_TAIL),
    re.compile(
        r"^DRIVE_TPDO_ACTIVE_STATUS 4/8 node=1 state=(?P<state>[A-Z_]+) "
        r"raw=(?P<raw>-?\d+) norm_mm_s=(?P<normalized>-?\d+) "
        r"seq=(?P<sequence>\d+) age_ms=(?P<age>\d+) "
        r"max_gap_ms=(?P<max_gap>\d+)" + _REQ_TAIL),
    re.compile(
        r"^DRIVE_TPDO_ACTIVE_STATUS 5/8 node=2 state=(?P<state>[A-Z_]+) "
        r"raw=(?P<raw>-?\d+) norm_mm_s=(?P<normalized>-?\d+) "
        r"seq=(?P<sequence>\d+) age_ms=(?P<age>\d+) "
        r"max_gap_ms=(?P<max_gap>\d+)" + _REQ_TAIL),
    re.compile(
        r"^DRIVE_TPDO_ACTIVE_STATUS 6/8 pair_valid=(?P<pair_valid>[01]) "
        r"pair_state=(?P<pair_state>[A-Z_]+) pair_seq=(?P<pair_sequence>\d+) "
        r"pair_age_ms=(?P<pair_age>\d+) pair_skew_ms=(?P<pair_skew>\d+)"
        + _REQ_TAIL),
    re.compile(
        r"^DRIVE_TPDO_ACTIVE_STATUS 7/8 "
        r"runtime_sdo_606c_poll_enabled=(?P<poll_enabled>[01]) "
        r"runtime_sdo_606c_poll_count=(?P<poll_count>\d+) "
        r"last_fault=(?P<last_fault>[A-Z0-9_]+)" + _REQ_TAIL),
    re.compile(
        r"^DRIVE_TPDO_ACTIVE_STATUS 8/8 stop_reject=(?P<stop_reject>[A-Z0-9_]+) "
        r"reject_new/invalid=(?P<reject_new>\d+)/(?P<reject_invalid>\d+) "
        r"odom_suppressed=(?P<odom_suppressed>\d+)" + _REQ_TAIL),
)
WHEEL_FEEDBACK_HEALTH_RE = re.compile(
    r"^WHEEL_FEEDBACK_HEALTH v=1 mode=(?P<mode>[A-Z0-9_]+) "
    r"authority=(?P<authority>SDO|TPDO) pair_valid=(?P<pair_valid>[01]) "
    r"pair_state=(?P<pair_state>[A-Z_]+) pair_seq=(?P<pair_sequence>\d+) "
    r"pair_age_ms=(?P<pair_age>\d+) pair_skew_ms=(?P<pair_skew>\d+) "
    r"sdo606c=(?P<poll_enabled>[01])$"
)
WHEEL_FEEDBACK_HEALTH_NODE_RE = re.compile(
    r"^WHEEL_FEEDBACK_HEALTH_NODE v=1 "
    r"node1_seq=(?P<left_sequence>\d+) node1_age_ms=(?P<left_age>\d+) "
    r"node2_seq=(?P<right_sequence>\d+) node2_age_ms=(?P<right_age>\d+)$"
)
LIFT_STATUS_RE = re.compile(
    r"^LIFT (?P<state>[A-Z_]+) "
    r"(?P<position_mm>-?\d+) (?P<target_mm>-?\d+) "
    r"(?P<position_pulses>-?\d+) (?P<homed>[01]) "
    r"(?P<fault>0|0x[0-9A-Fa-f]+|[1-9]\d*)$"
)
CMD_ACK_RE = re.compile(
    r"^CMD_ACK (?P<command>[A-Z_]+) seq=(?P<sequence>\d+) "
    r"accepted=(?P<accepted>[01]) reason=(?P<reason>[A-Z0-9_]+)"
    + _REQ_TAIL
)
FAULT_RESET_BUSY_RE = re.compile(
    r"^DRIVE_FAULT_RESET_SAFE state=FAILED reason=BUSY "
    r"active_gen=(?P<active_gen>\d+) gen=(?P<generation>\d+)" + _REQ_TAIL
)
REFRESH_BUSY_RE = re.compile(
    r"^DRIVE_BASE_SAFE_REFRESH state=FAILED reason=BUSY "
    r"active_txn=(?P<active_txn>\d+) txn=(?P<txn>\d+)" + _REQ_TAIL
)
LIFT_RESET_STARTED_RE = re.compile(
    r"^LIFT_FAULT_RESET state=STARTED txn=(?P<txn>\d+)" + _REQ_TAIL
)
LIFT_RESET_PENDING_RE = re.compile(
    r"^LIFT_FAULT_RESET state=PENDING phase=(?P<phase>[A-Z0-9_]+) "
    r"txn=(?P<txn>\d+)" + _REQ_TAIL
)
LIFT_RESET_COMPLETE_RE = re.compile(
    r"^LIFT_FAULT_RESET state=COMPLETE txn=(?P<txn>\d+)" + _REQ_TAIL
)
LIFT_RESET_FAILED_RE = re.compile(
    r"^LIFT_FAULT_RESET state=FAILED reason=(?P<reason>[A-Z0-9_]+)"
    r"(?: active_txn=(?P<active_txn>\d+))? txn=(?P<txn>\d+)" + _REQ_TAIL
)
LIFT_HOME_STARTED_RE = re.compile(
    r"^LIFT_HOME state=STARTED txn=(?P<txn>\d+)" + _REQ_TAIL
)
LIFT_HOME_PENDING_RE = re.compile(
    r"^LIFT_HOME state=PENDING phase=(?P<phase>[A-Z0-9_]+) "
    r"txn=(?P<txn>\d+)" + _REQ_TAIL
)
LIFT_HOME_COMPLETE_RE = re.compile(
    r"^LIFT_HOME state=COMPLETE homed=(?P<homed>[01]) "
    r"txn=(?P<txn>\d+)" + _REQ_TAIL
)
LIFT_HOME_FAILED_RE = re.compile(
    r"^LIFT_HOME state=FAILED reason=(?P<reason>[A-Z0-9_]+)"
    r"(?: active_txn=(?P<active_txn>\d+))? txn=(?P<txn>\d+)" + _REQ_TAIL
)
# The eight tags of one DRIVE_FAULT_DEBUG snapshot response.
FAULT_DEBUG_TAGS = (
    "DRIVE_FAULT_CURRENT", "DRIVE_FAULT_RESET_DIAG",
    "DRIVE_FAULT_RESET_LEFT", "DRIVE_FAULT_RESET_RIGHT",
    "DRIVE_FAULT_EVENT", "DRIVE_FAULT_STATUS",
    "DRIVE_FAULT_EMCY", "DRIVE_FAULT_HB",
)
ODOM_RE = re.compile(
    r"^ODOM (?P<x_mm>-?\d+) (?P<y_mm>-?\d+) (?P<theta_mrad>-?\d+) "
    r"(?P<left_mrpm>-?\d+) (?P<right_mrpm>-?\d+) "
    r"(?P<state>[A-Z_]+) (?P<fault>\d+)$"
)
FW_IDENTITY_RE = re.compile(
    r"^FW variant=(?P<variant>\S+) build=(?P<build>\S+) "
    r"profile=(?P<profile>\S+) optimization=(?P<optimization>\S+) "
    r'date="(?P<date>[^"]*)" time=(?P<time>\S+) '
    r"protocol=(?P<protocol>\d+) txmgr=(?P<txmgr>\d+) pi=(?P<pi>\d+) "
    r"raw_test=(?P<raw_test>[01])$"
)
# Fail-closed RX diagnostics: five bounded lines sharing snap/req.
RX_DEBUG_1_RE = re.compile(
    r"^UART_RX_DEBUG 1/5 armed=(?P<armed>[01]) "
    r"rearm_pending=(?P<rearm_pending>[01]) "
    r"arm_seq=(?P<arm_attempt_sequence>\d+) "
    r"completion_seq=(?P<completion_sequence>\d+) "
    r"link_state=(?P<link_state>[A-Z_]+) "
    r"outage_age=(?P<outage_age>\d+)"
    + _SNAP_TAIL
)
RX_DEBUG_2_RE = re.compile(
    r"^UART_RX_DEBUG 2/5 "
    r"rearm_fail=(?P<rearm_failures>\d+) "
    r"rearm_recover=(?P<rearm_recoveries>\d+) "
    r"rearm_fault=(?P<rearm_fault_events>\d+) "
    r"consecutive=(?P<consecutive_failures>\d+) "
    r"last_hal=(?P<last_hal_status>\d+)"
    + _SNAP_TAIL
)
RX_DEBUG_3_RE = re.compile(
    r"^UART_RX_DEBUG 3/5 "
    r"stop_pending=(?P<immediate_stop_pending>[01]) "
    r"stop_events=(?P<immediate_stop_events>\d+) "
    r"stop_dispatch=(?P<immediate_stop_dispatches>\d+) "
    r"ring_ovf=(?P<ring_overflow>\d+) line_ovf=(?P<line_overflow>\d+)"
    + _SNAP_TAIL
)
RX_DEBUG_4_RE = re.compile(
    r"^UART_RX_DEBUG 4/5 "
    r"uart_err=(?P<uart_error>\d+) bad_ch=(?P<invalid_char>\d+) "
    r"asm_to=(?P<assembly_timeout>\d+) stale=(?P<stale_line>\d+) "
    r"damage_gen=(?P<damage_generation>\d+) desync=(?P<desync>\d+)"
    + _SNAP_TAIL
)
RX_DEBUG_5_RE = re.compile(
    r"^UART_RX_DEBUG 5/5 "
    r"byte_budget=(?P<byte_budget_hits>\d+) "
    r"cmd_budget=(?P<command_budget_hits>\d+) sync=(?P<sync>[01])"
    + _SNAP_TAIL
)
# BAT t_ms health fault age voltage_mV current_mA remaining_mAh rsoc
#     temp1_dC temp2_dC min_cell_mV max_cell_mV  (exactly 12 numeric fields)
BATTERY_RE = re.compile(
    r"^BAT (?P<t_ms>\d+) (?P<health>\d+) (?P<fault>\d+) (?P<age_ms>\d+) "
    r"(?P<voltage_mv>\d+) (?P<current_ma>-?\d+) (?P<remaining_mah>\d+) "
    r"(?P<rsoc_percent>\d+) (?P<temp1_dc>-?\d+) (?P<temp2_dc>-?\d+) "
    r"(?P<min_cell_mv>\d+) (?P<max_cell_mv>\d+)$"
)
BAT_DEBUG_1_RE = re.compile(
    r"^BAT_DEBUG 1/3 slave=(?P<slave>\d+) dir=PG0 RS485_WR1 "
    r"polarity=HIGH_TX_LOW_RX poll=(?P<requests>\d+) "
    r"resp=(?P<responses>\d+)" + _SNAP_TAIL
)
BAT_DEBUG_2_RE = re.compile(
    r"^BAT_DEBUG 2/3 timeout=(?P<timeout>\d+) crc=(?P<crc>\d+) "
    r"exception=(?P<exception>\d+) uart=(?P<uart>\d+) "
    r"age=(?P<age_ms>\d+)" + _SNAP_TAIL
)
TX_DEBUG_2_RE = re.compile(
    r"^UART_TX_DEBUG 2/6 depth=(\d+)/(\d+)/(\d+)/(\d+) "
    r"high=(\d+)/(\d+)/(\d+)/(\d+) snap=\d+ req=[0-9A-F]{8}$"
)
TX_DEBUG_4_RE = re.compile(
    r"^UART_TX_DEBUG 4/6 sent=(\d+)/(\d+)/(\d+)/(\d+) "
    r"dropped=(\d+)/(\d+)/(\d+)/(\d+) snap=\d+ req=[0-9A-F]{8}$"
)
TX_DEBUG_5_RE = re.compile(
    r"^UART_TX_DEBUG 5/6 hal_busy=(\d+) hal_error=(\d+) uart_error=(\d+) "
    r"last_error=0x[0-9A-Fa-f]+ timeout=(\d+) recovery=(\d+) "
    r"snap=\d+ req=[0-9A-F]{8}$"
)
TX_DEBUG_6_RE = re.compile(
    r"^UART_TX_DEBUG 6/6 format_overflow=(\d+) completion=(\d+) "
    r"resync=(\d+) snap=\d+ req=[0-9A-F]{8}$"
)
YAW_CTRL_3_RE = re.compile(
    r"^YAW_CTRL 3/15 yaw_mode=(?P<mode>\S+) pi_enabled=(?P<pi_enabled>[01]) "
    r"corrected=(?P<corrected>-?\d+) committed=(?P<committed>-?\d+) "
    r"measured=(?P<measured>-?\d+) error=(?P<error>-?\d+)"
    + _SNAP_TAIL
)
YAW_CTRL_4_RE = re.compile(
    r"^YAW_CTRL 4/15 P_term=(?P<p_term>-?\d+) I_state=(?P<i_state>-?\d+) "
    r"I_term=(?P<i_term>-?\d+) raw/req/proposed/submitted="
    r"(?P<raw>-?\d+)/(?P<requested>-?\d+)/(?P<proposed>-?\d+)/"
    r"(?P<submitted>-?\d+)" + _SNAP_TAIL
)
YAW_CTRL_11_RE = re.compile(
    r"^YAW_CTRL 11/15 feedback_sequence_valid=(?P<sequence_valid>[01]) "
    r"last_consumed_feedback_sequence=(?P<sequence>\d+) "
    r"reject stale/skew=(?P<stale>\d+)/(?P<skew>\d+)" + _SNAP_TAIL
)
YAW_CTRL_12_RE = re.compile(
    r"^YAW_CTRL 12/15 track=(?P<track>\S+) reason=(?P<reason>\S+) "
    r"last=(?P<last>\S+) latch=(?P<latch>[01]) "
    r"cnt=(?P<c1>\d+)/(?P<c2>\d+)/(?P<c3>\d+)" + _SNAP_TAIL
)
YAW_CTRL_13_RE = re.compile(
    r"^YAW_CTRL 13/15 obs err/corr/head/hard/rate/busy/sign/div="
    r"(?P<err>\d+)/(?P<corr>\d+)/(?P<head>\d+)/(?P<hard>\d+)/"
    r"(?P<rate>\d+)/(?P<busy>\d+)/(?P<sign>\d+)/(?P<div>\d+)"
    + _SNAP_TAIL
)

KNOWN_PREFIXES = (
    "FW ", "IMU ", "ODOM ", "WHEEL_FEEDBACK_HEALTH ",
    "WHEEL_FEEDBACK_HEALTH_NODE ", "LIFT ", "BAT ", "UART_TX_DEBUG ",
    "UART_RX_DEBUG ",
    "DRIVE_FAULT_", "DRIVE_BASE_SAFE", "DRIVE_FAULT_RESET_SAFE ",
    "DRIVE_CAN_TRACE_", "DRIVE_CAN_TRACE ",
    "DRIVE_TPDO_", "DRIVE_TPDO ",
    "YAW_CTRL ", "YAW_PI ", "CMD ", "CMD_ACK ", "CAN ", "SDO ",
)

# Commands that never change drive/lift state. A session that has sent only
# these may exit without emitting any safety command.
READ_ONLY_COMMANDS = frozenset((
    "FW_VERSION", "VERSION", "UART_TX_DEBUG", "UART_RX_DEBUG",
    "DRIVE_FAULT_DEBUG",
    "DRIVE_CAN_TRACE_STATUS", "DRIVE_CAN_TRACE_DUMP",
    "DRIVE_BASE_SAFE", "DRIVE_TPDO_STATUS", "DRIVE_TPDO_ACTIVE_STATUS",
    "BAT_DEBUG", "IMU_DEBUG",
    "DRIVE_YAW_DEBUG",
    "DRIVE_RPDO_DEBUG",
    "LIFT_STATUS", "LIFT_RAW", "LIFT_RAW_STATUS", "LIFT_SCALE_INFO",
))

SAFE_COMMANDS = ("CMD_VEL 0 0", "STOP", "DISABLE", "LIFT_STOP")


def make_nonce() -> str:
    """Fresh host request nonce: 8 uppercase hex characters, never zero.

    req=00000000 is the reserved unbound value; regenerate if the RNG ever
    produces it.
    """
    while True:
        value = secrets.token_hex(4).upper()
        if value != "00000000":
            return value


# --------------------------------------------------------------------------
# Dataclasses
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class DriveBaseSafeSummary:
    line: str
    safe: int
    state: str
    active: int
    enable: int
    disable: int
    drive_enabled: int
    target_zero: int
    pending: int
    zero: int
    snap: int
    request: str


@dataclass(frozen=True)
class DriveBaseSafeStatus:
    left_valid: int
    right_valid: int
    left_fresh: int
    right_fresh: int
    left_status: int
    right_status: int
    left_age_ms: int
    right_age_ms: int
    left_sequence: int
    right_sequence: int
    left_generation: int
    right_generation: int
    snap: int
    request: str


@dataclass(frozen=True)
class DriveBaseSafeZero:
    valid: int
    age_ms: int
    generation: int
    safety_generation: int
    snap: int
    request: str


@dataclass(frozen=True)
class DriveBaseSafeReport:
    summary: DriveBaseSafeSummary
    status: DriveBaseSafeStatus
    zero: DriveBaseSafeZero


@dataclass(frozen=True)
class DriveFaultResetEvent:
    line: str
    state: str
    phase: str | None = None
    node: int | None = None
    reason: str | None = None
    left_cleared: int | None = None
    right_cleared: int | None = None
    abort: int | None = None
    timeouts: int | None = None
    generation: int | None = None
    request: str | None = None


@dataclass(frozen=True)
class DriveBaseSafeRefreshEvent:
    line: str
    state: str
    phase: str | None = None
    safe: int | None = None
    generation: int | None = None
    txn: int | None = None
    reason: str | None = None
    node: int | None = None
    request: str | None = None


@dataclass(frozen=True)
class LiftStatus:
    line: str
    state: str
    position_mm: int
    target_mm: int
    position_pulses: int
    homed: int
    fault: int


@dataclass(frozen=True)
class CommandAck:
    line: str
    command: str
    sequence: int
    accepted: int
    reason: str
    request: str


@dataclass(frozen=True)
class WheelTpdoConfigEvent:
    line: str
    state: str
    node: int
    request: str
    reason: str | None = None
    left_configured: int | None = None
    right_configured: int | None = None
    mode: str | None = None


@dataclass(frozen=True)
class WheelTpdoStatusSnapshot:
    lines: tuple[str, str, str, str, str]
    request: str
    mode: str
    left_configured: int
    right_configured: int
    left_state: str
    right_state: str
    left_sequence: int
    right_sequence: int
    left_frame_count: int
    right_frame_count: int
    left_ignored_unconfigured: int
    right_ignored_unconfigured: int
    left_ignored_mode: int
    right_ignored_mode: int


@dataclass(frozen=True)
class WheelTpdoActiveEvent:
    line: str
    command: str
    state: str
    request: str
    reason: str | None = None
    mode: str | None = None
    authority: str | None = None


@dataclass(frozen=True)
class WheelTpdoActiveStatusSnapshot:
    lines: tuple[str, str, str, str, str, str, str, str]
    request: str
    mode: str
    authority: str
    odom_authority: str
    left_configured: int
    right_configured: int
    drive_state: str
    software_enabled: int
    hardware_enable_confirmed: int
    hardware_disable_confirmed: int
    target_zero: int
    lift_stopped: int
    transition_active: int
    fault_active: int
    estop_active: int
    left_state: str
    left_raw: int
    left_normalized_mm_s: int
    left_sequence: int
    left_age_ms: int
    left_max_gap_ms: int
    right_state: str
    right_raw: int
    right_normalized_mm_s: int
    right_sequence: int
    right_age_ms: int
    right_max_gap_ms: int
    pair_valid: int
    pair_state: str
    pair_sequence: int
    pair_age_ms: int
    pair_skew_ms: int
    runtime_sdo_606c_poll_enabled: int
    runtime_sdo_606c_poll_count: int
    last_fault: str
    stop_reject: str
    stop_pair_not_new_count: int
    stop_pair_invalid_count: int
    odom_stale_suppressed_count: int


@dataclass(frozen=True)
class WheelFeedbackHealth:
    line: str
    mode: str
    authority: str
    pair_valid: int
    pair_state: str
    pair_sequence: int
    pair_age_ms: int
    pair_skew_ms: int
    runtime_sdo_606c_poll_enabled: int


@dataclass(frozen=True)
class WheelFeedbackNodeHealth:
    line: str
    left_sequence: int
    left_age_ms: int
    right_sequence: int
    right_age_ms: int


@dataclass(frozen=True)
class LiftTransactionEvent:
    line: str
    command: str
    state: str
    txn: int
    request: str
    phase: str | None = None
    reason: str | None = None
    active_txn: int | None = None
    homed: int | None = None


@dataclass(frozen=True)
class OdomStatus:
    line: str
    x_mm: int
    y_mm: int
    theta_mrad: int
    left_mrpm: int
    right_mrpm: int
    state: str
    fault: int

    @property
    def left_rpm(self) -> float:
        return self.left_mrpm / 1000.0

    @property
    def right_rpm(self) -> float:
        return self.right_mrpm / 1000.0


@dataclass(frozen=True)
class FirmwareIdentity:
    line: str
    variant: str
    build: str
    profile: str
    optimization: str
    protocol: int
    txmgr: int
    pi: int
    raw_test: int


@dataclass(frozen=True)
class RxDebugStats:
    """One frame, or one assembled five-line UART_RX_DEBUG snapshot.

    ``part`` is 1..5 for a frame returned by :func:`parse_rx_debug` and 0
    for a complete nonce-bound snapshot returned by
    :func:`parse_rx_debug_snapshot`.
    """

    line: str
    part: int
    snap: int
    request: str
    armed: int | None = None
    rearm_pending: int | None = None
    arm_attempt_sequence: int | None = None
    completion_sequence: int | None = None
    link_state: str | None = None
    outage_age: int | None = None
    rearm_failures: int | None = None
    rearm_recoveries: int | None = None
    rearm_fault_events: int | None = None
    consecutive_failures: int | None = None
    last_hal_status: int | None = None
    immediate_stop_pending: int | None = None
    immediate_stop_events: int | None = None
    immediate_stop_dispatches: int | None = None
    ring_overflow: int | None = None
    line_overflow: int | None = None
    uart_error: int | None = None
    invalid_char: int | None = None
    assembly_timeout: int | None = None
    stale_line: int | None = None
    damage_generation: int | None = None
    desync: int | None = None
    byte_budget_hits: int | None = None
    command_budget_hits: int | None = None
    sync: int | None = None


@dataclass(frozen=True)
class BatterySample:
    line: str
    t_ms: int
    health: int
    fault: int
    age_ms: int
    voltage_mv: int
    current_ma: int
    remaining_mah: int
    rsoc_percent: int
    temp1_dc: int
    temp2_dc: int
    min_cell_mv: int
    max_cell_mv: int


@dataclass(frozen=True)
class BatteryDebug:
    requests: int
    responses: int
    timeout: int
    crc: int
    exception: int
    uart: int


@dataclass(frozen=True)
class TxDebugStats:
    depth: tuple[int, int, int, int]
    high_water: tuple[int, int, int, int]
    sent: tuple[int, int, int, int]
    dropped: tuple[int, int, int, int]
    hal_busy: int
    hal_error: int
    uart_error: int
    timeout: int
    recovery: int
    format_overflow: int
    completion: int
    resync: int


@dataclass(frozen=True)
class YawControlSnapshot:
    mode: str
    pi_enabled: int
    corrected_mrad_s: int
    committed_mrad_s: int
    measured_mrad_s: int
    error_mrad_s: int
    proposed_correction_mrad_s: int
    submitted_correction_mrad_s: int
    feedback_sequence_valid: int
    feedback_sequence: int
    track_state: str
    track_reason: str
    track_latch: int
    large_error_consecutive: int
    wrong_sign_consecutive: int
    divergence_consecutive: int
    obs_large_error: int
    obs_correction_limit: int
    obs_wrong_sign: int
    obs_divergence: int
    snap: int


@dataclass(frozen=True)
class IntervalStats:
    count: int
    median_ms: float
    p95_ms: float
    p99_ms: float
    max_ms: float
    backward_count: int
    gap_count: int


# --------------------------------------------------------------------------
# Line parsers
# --------------------------------------------------------------------------

def parse_summary(line: str) -> DriveBaseSafeSummary | None:
    match = SUMMARY_RE.fullmatch(line.strip())
    if match is None:
        return None
    values = match.groupdict()
    return DriveBaseSafeSummary(
        line=line.strip(),
        state=values.pop("state"),
        pending=int(values.pop("pending"), 16),
        request=values.pop("request"),
        **{key: int(value) for key, value in values.items()}
    )


def parse_status(line: str) -> DriveBaseSafeStatus | None:
    match = STATUS_RE.fullmatch(line.strip())
    if match is None:
        return None
    values = match.groupdict()
    return DriveBaseSafeStatus(
        left_status=int(values.pop("left_status"), 16),
        right_status=int(values.pop("right_status"), 16),
        request=values.pop("request"),
        **{key: int(value) for key, value in values.items()}
    )


def parse_zero(line: str) -> DriveBaseSafeZero | None:
    match = ZERO_RE.fullmatch(line.strip())
    if match is None:
        return None
    values = match.groupdict()
    return DriveBaseSafeZero(
        request=values.pop("request"),
        **{key: int(value) for key, value in values.items()}
    )


def parse_refresh_event(line: str) -> DriveBaseSafeRefreshEvent | None:
    clean = line.strip()
    match = REFRESH_STARTED_RE.fullmatch(clean)
    if match is not None:
        return DriveBaseSafeRefreshEvent(
            line=clean, state="STARTED",
            generation=int(match.group("generation")),
            txn=int(match.group("txn")),
            request=match.group("request"))
    match = REFRESH_PENDING_RE.fullmatch(clean)
    if match is not None:
        return DriveBaseSafeRefreshEvent(
            line=clean, state="PENDING", phase=match.group("phase"),
            txn=int(match.group("txn")),
            request=match.group("request"))
    match = REFRESH_COMPLETE_RE.fullmatch(clean)
    if match is not None:
        return DriveBaseSafeRefreshEvent(
            line=clean, state="COMPLETE",
            safe=int(match.group("safe")),
            generation=int(match.group("generation")),
            txn=int(match.group("txn")),
            request=match.group("request"))
    match = REFRESH_FAILED_RE.fullmatch(clean)
    if match is not None:
        return DriveBaseSafeRefreshEvent(
            line=clean, state="FAILED", reason=match.group("reason"),
            node=int(match.group("node")),
            txn=int(match.group("txn")),
            request=match.group("request"))
    match = REFRESH_BUSY_RE.fullmatch(clean)
    if match is not None:
        return DriveBaseSafeRefreshEvent(
            line=clean, state="FAILED", reason="BUSY",
            txn=int(match.group("txn")),
            request=match.group("request"))
    return None


def parse_fault_reset_event(line: str) -> DriveFaultResetEvent | None:
    clean = line.strip()
    match = FAULT_RESET_STARTED_RE.fullmatch(clean)
    if match is not None:
        return DriveFaultResetEvent(
            line=clean, state="STARTED",
            generation=int(match.group("generation")),
            request=match.group("request"))
    match = FAULT_RESET_PENDING_RE.fullmatch(clean)
    if match is not None:
        return DriveFaultResetEvent(
            line=clean, state="PENDING", phase=match.group("phase"),
            node=int(match.group("node")),
            generation=int(match.group("generation")),
            request=match.group("request"))
    match = FAULT_RESET_COMPLETE_RE.fullmatch(clean)
    if match is not None:
        return DriveFaultResetEvent(
            line=clean, state="COMPLETE",
            left_cleared=int(match.group("left_cleared")),
            right_cleared=int(match.group("right_cleared")),
            generation=int(match.group("generation")),
            request=match.group("request"))
    match = FAULT_RESET_FAILED_RE.fullmatch(clean)
    if match is not None:
        return DriveFaultResetEvent(
            line=clean, state="FAILED", phase=match.group("phase"),
            node=int(match.group("node")), reason=match.group("reason"),
            abort=int(match.group("abort"), 16),
            timeouts=int(match.group("timeouts")),
            generation=int(match.group("generation")),
            request=match.group("request"))
    match = FAULT_RESET_BUSY_RE.fullmatch(clean)
    if match is not None:
        return DriveFaultResetEvent(
            line=clean, state="FAILED", reason="BUSY",
            generation=int(match.group("generation")),
            request=match.group("request"))
    return None


def parse_lift_status(line: str) -> LiftStatus | None:
    clean = line.strip()
    match = LIFT_STATUS_RE.fullmatch(clean)
    if match is None:
        return None
    values = match.groupdict()
    return LiftStatus(
        line=clean,
        state=values.pop("state"),
        fault=int(values.pop("fault"), 0),
        **{key: int(value) for key, value in values.items()},
    )


def parse_command_ack(line: str) -> CommandAck | None:
    match = CMD_ACK_RE.fullmatch(line.strip())
    if match is None:
        return None
    return CommandAck(
        line=line.strip(),
        command=match.group("command"),
        sequence=int(match.group("sequence")),
        accepted=int(match.group("accepted")),
        reason=match.group("reason"),
        request=match.group("request"),
    )


def parse_tpdo_config_event(line: str) -> WheelTpdoConfigEvent | None:
    clean = line.strip()
    match = TPDO_CONFIG_STARTED_RE.fullmatch(clean)
    if match is not None:
        node = match.group("node")
        return WheelTpdoConfigEvent(
            line=clean, state="STARTED",
            node=0 if node == "all" else int(node),
            request=match.group("request"))
    match = TPDO_CONFIG_COMPLETE_RE.fullmatch(clean)
    if match is not None:
        node = match.group("node")
        return WheelTpdoConfigEvent(
            line=clean, state="COMPLETE",
            node=0 if node == "all" else int(node),
            request=match.group("request"),
            left_configured=int(match.group("left_configured")),
            right_configured=int(match.group("right_configured")),
            mode=match.group("mode"))
    match = TPDO_CONFIG_FAILED_RE.fullmatch(clean)
    if match is not None:
        node = match.group("node")
        return WheelTpdoConfigEvent(
            line=clean, state="FAILED",
            node=0 if node == "all" else int(node),
            request=match.group("request"), reason=match.group("reason"))
    return None


def parse_tpdo_status(lines: list[str],
                      nonce: str) -> WheelTpdoStatusSnapshot | None:
    """Parse one exact five-line TPDO snapshot bound to ``nonce``.

    Different-request records and unrelated telemetry are ignored. A
    malformed TPDO status record or duplicate line number poisons this
    transaction so incomplete/mixed evidence can never be accepted.
    """
    matches: dict[int, re.Match[str]] = {}
    raw_lines: dict[int, str] = {}
    regexes = (TPDO_STATUS_1_RE, *TPDO_STATUS_NODE_RES,
               *TPDO_STATUS_COMPARE_RES)
    poisoned = False
    for raw in lines:
        clean = raw.strip()
        if not clean.startswith("DRIVE_TPDO_STATUS "):
            continue
        parsed = None
        part = 0
        for number, regex in enumerate(regexes, start=1):
            candidate = regex.fullmatch(clean)
            if candidate is not None:
                parsed = candidate
                part = number
                break
        if parsed is None:
            poisoned = True
            continue
        if parsed.group("request") != nonce:
            continue
        if part in matches:
            poisoned = True
            continue
        matches[part] = parsed
        raw_lines[part] = clean
    if poisoned or set(matches) != {1, 2, 3, 4, 5}:
        return None
    first, left, right = matches[1], matches[2], matches[3]
    return WheelTpdoStatusSnapshot(
        lines=tuple(raw_lines[number] for number in range(1, 6)),
        request=nonce, mode=first.group("mode"),
        left_configured=int(first.group("left_configured")),
        right_configured=int(first.group("right_configured")),
        left_state=left.group("state"), right_state=right.group("state"),
        left_sequence=int(left.group("sequence")),
        right_sequence=int(right.group("sequence")),
        left_frame_count=int(left.group("frames")),
        right_frame_count=int(right.group("frames")),
        left_ignored_unconfigured=int(left.group("ignored_unconfigured")),
        right_ignored_unconfigured=int(right.group("ignored_unconfigured")),
        left_ignored_mode=int(left.group("ignored_mode")),
        right_ignored_mode=int(right.group("ignored_mode")))


def parse_tpdo_active_event(line: str) -> WheelTpdoActiveEvent | None:
    clean = line.strip()
    command = ("DRIVE_TPDO_ACTIVE_DISABLE" if clean.startswith(
        "DRIVE_TPDO_ACTIVE_DISABLE ") else "DRIVE_TPDO_ACTIVE")
    regexes = (TPDO_ACTIVE_DISABLE_EVENT_RES if command.endswith("DISABLE")
               else TPDO_ACTIVE_EVENT_RES)
    for state, regex in zip(("STARTED", "COMPLETE", "FAILED"), regexes):
        match = regex.fullmatch(clean)
        if match is None:
            continue
        groups = match.groupdict()
        return WheelTpdoActiveEvent(
            line=clean, command=command, state=state,
            request=groups["request"], reason=groups.get("reason"),
            mode=groups.get("mode"), authority=groups.get("authority"))
    return None


def parse_tpdo_active_status(
        lines: list[str], nonce: str) -> WheelTpdoActiveStatusSnapshot | None:
    matches: dict[int, re.Match[str]] = {}
    raw_lines: dict[int, str] = {}
    poisoned = False
    for raw in lines:
        clean = raw.strip()
        if not clean.startswith("DRIVE_TPDO_ACTIVE_STATUS "):
            continue
        parsed = None
        part = 0
        for number, regex in enumerate(TPDO_ACTIVE_STATUS_RES, start=1):
            candidate = regex.fullmatch(clean)
            if candidate is not None:
                parsed, part = candidate, number
                break
        if parsed is None:
            poisoned = True
            continue
        if parsed.group("request") != nonce:
            continue
        if part in matches:
            poisoned = True
            continue
        matches[part] = parsed
        raw_lines[part] = clean
    if poisoned or set(matches) != set(range(1, 9)):
        return None
    first, gates, safety, left, right, pair, runtime, diagnostics = (
        matches[index] for index in range(1, 9))
    return WheelTpdoActiveStatusSnapshot(
        lines=tuple(raw_lines[index] for index in range(1, 9)),
        request=nonce, mode=first.group("mode"),
        authority=first.group("authority"),
        odom_authority=first.group("odom_authority"),
        left_configured=int(first.group("left_configured")),
        right_configured=int(first.group("right_configured")),
        drive_state=gates.group("drive_state"),
        software_enabled=int(gates.group("software_enabled")),
        hardware_enable_confirmed=int(gates.group("hardware_enable")),
        hardware_disable_confirmed=int(gates.group("hardware_disable")),
        target_zero=int(safety.group("target_zero")),
        lift_stopped=int(safety.group("lift_stopped")),
        transition_active=int(safety.group("transition_active")),
        fault_active=int(safety.group("fault_active")),
        estop_active=int(safety.group("estop_active")),
        left_state=left.group("state"), left_raw=int(left.group("raw")),
        left_normalized_mm_s=int(left.group("normalized")),
        left_sequence=int(left.group("sequence")),
        left_age_ms=int(left.group("age")),
        left_max_gap_ms=int(left.group("max_gap")),
        right_state=right.group("state"), right_raw=int(right.group("raw")),
        right_normalized_mm_s=int(right.group("normalized")),
        right_sequence=int(right.group("sequence")),
        right_age_ms=int(right.group("age")),
        right_max_gap_ms=int(right.group("max_gap")),
        pair_valid=int(pair.group("pair_valid")),
        pair_state=pair.group("pair_state"),
        pair_sequence=int(pair.group("pair_sequence")),
        pair_age_ms=int(pair.group("pair_age")),
        pair_skew_ms=int(pair.group("pair_skew")),
        runtime_sdo_606c_poll_enabled=int(runtime.group("poll_enabled")),
        runtime_sdo_606c_poll_count=int(runtime.group("poll_count")),
        last_fault=runtime.group("last_fault"),
        stop_reject=diagnostics.group("stop_reject"),
        stop_pair_not_new_count=int(diagnostics.group("reject_new")),
        stop_pair_invalid_count=int(diagnostics.group("reject_invalid")),
        odom_stale_suppressed_count=int(
            diagnostics.group("odom_suppressed")))


def parse_wheel_feedback_health(line: str) -> WheelFeedbackHealth | None:
    clean = line.strip()
    match = WHEEL_FEEDBACK_HEALTH_RE.fullmatch(clean)
    if match is None:
        return None
    return WheelFeedbackHealth(
        line=clean, mode=match.group("mode"),
        authority=match.group("authority"),
        pair_valid=int(match.group("pair_valid")),
        pair_state=match.group("pair_state"),
        pair_sequence=int(match.group("pair_sequence")),
        pair_age_ms=int(match.group("pair_age")),
        pair_skew_ms=int(match.group("pair_skew")),
        runtime_sdo_606c_poll_enabled=int(match.group("poll_enabled")))


def parse_wheel_feedback_node_health(
        line: str) -> WheelFeedbackNodeHealth | None:
    clean = line.strip()
    match = WHEEL_FEEDBACK_HEALTH_NODE_RE.fullmatch(clean)
    if match is None:
        return None
    return WheelFeedbackNodeHealth(
        line=clean,
        left_sequence=int(match.group("left_sequence")),
        left_age_ms=int(match.group("left_age")),
        right_sequence=int(match.group("right_sequence")),
        right_age_ms=int(match.group("right_age")))


def _parse_lift_event(line: str, command: str, started, pending, complete,
                      failed) -> LiftTransactionEvent | None:
    clean = line.strip()
    match = started.fullmatch(clean)
    if match is not None:
        return LiftTransactionEvent(
            line=clean, command=command, state="STARTED",
            txn=int(match.group("txn")), request=match.group("request"))
    match = pending.fullmatch(clean)
    if match is not None:
        return LiftTransactionEvent(
            line=clean, command=command, state="PENDING",
            phase=match.group("phase"),
            txn=int(match.group("txn")), request=match.group("request"))
    match = complete.fullmatch(clean)
    if match is not None:
        homed = None
        if "homed" in match.groupdict():
            homed = int(match.group("homed"))
        return LiftTransactionEvent(
            line=clean, command=command, state="COMPLETE",
            txn=int(match.group("txn")), request=match.group("request"),
            homed=homed)
    match = failed.fullmatch(clean)
    if match is not None:
        active = match.group("active_txn")
        return LiftTransactionEvent(
            line=clean, command=command, state="FAILED",
            reason=match.group("reason"),
            active_txn=int(active) if active is not None else None,
            txn=int(match.group("txn")), request=match.group("request"))
    return None


def parse_lift_reset_event(line: str) -> LiftTransactionEvent | None:
    return _parse_lift_event(
        line, "LIFT_FAULT_RESET", LIFT_RESET_STARTED_RE,
        LIFT_RESET_PENDING_RE, LIFT_RESET_COMPLETE_RE, LIFT_RESET_FAILED_RE)


def parse_lift_home_event(line: str) -> LiftTransactionEvent | None:
    return _parse_lift_event(
        line, "LIFT_HOME", LIFT_HOME_STARTED_RE, LIFT_HOME_PENDING_RE,
        LIFT_HOME_COMPLETE_RE, LIFT_HOME_FAILED_RE)


def parse_fault_debug_snapshot(lines: list[str],
                               nonce: str) -> dict | None:
    """One complete nonce-bound DRIVE_FAULT_DEBUG snapshot (all 8 tags).

    Returns {tag: line} only when every tag arrived exactly once under one
    snap= with the caller's req=; duplicates poison the snapshot and lines
    from different snapshots are never mixed.
    """
    groups: dict[int, dict[str, str]] = {}
    poisoned: set[int] = set()
    order: list[int] = []
    for raw in lines:
        line = raw.strip()
        tag = None
        for candidate in FAULT_DEBUG_TAGS:
            if line.startswith(candidate + " "):
                tag = candidate
                break
        if tag is None:
            continue
        tail = SNAP_TAIL_RE.search(line)
        if tail is None or tail.group("request") != nonce:
            continue
        snap = int(tail.group("snap"))
        if snap not in groups:
            groups[snap] = {}
            order.append(snap)
        if tag in groups[snap]:
            poisoned.add(snap)
            continue
        groups[snap][tag] = line
    for snap in reversed(order):
        if snap in poisoned:
            continue
        group = groups[snap]
        if all(tag in group for tag in FAULT_DEBUG_TAGS):
            return dict(group)
    return None


def parse_odom(line: str) -> OdomStatus | None:
    match = ODOM_RE.fullmatch(line.strip())
    if match is None:
        return None
    values = match.groupdict()
    return OdomStatus(
        line=line.strip(),
        state=values.pop("state"),
        **{key: int(value) for key, value in values.items()},
    )


def parse_fw_identity(line: str) -> FirmwareIdentity | None:
    match = FW_IDENTITY_RE.fullmatch(line.strip())
    if match is None:
        return None
    return FirmwareIdentity(
        line=line.strip(),
        variant=match.group("variant"),
        build=match.group("build"),
        profile=match.group("profile"),
        optimization=match.group("optimization"),
        protocol=int(match.group("protocol")),
        txmgr=int(match.group("txmgr")),
        pi=int(match.group("pi")),
        raw_test=int(match.group("raw_test")),
    )


def parse_rx_debug(line: str) -> RxDebugStats | None:
    stripped = line.strip()
    for part, regex in enumerate(
            (RX_DEBUG_1_RE, RX_DEBUG_2_RE, RX_DEBUG_3_RE,
             RX_DEBUG_4_RE, RX_DEBUG_5_RE), start=1):
        match = regex.fullmatch(stripped)
        if match is None:
            continue
        values = {
            key: int(value)
            for key, value in match.groupdict().items()
            if key not in ("snap", "request", "link_state")
            and value is not None
        }
        return RxDebugStats(
            line=stripped,
            part=part,
            snap=int(match.group("snap")),
            request=match.group("request"),
            link_state=match.groupdict().get("link_state"),
            **values,
        )
    return None


def parse_battery_sample(line: str) -> BatterySample | None:
    match = BATTERY_RE.fullmatch(line.strip())
    if match is None:
        return None
    return BatterySample(
        line=line.strip(),
        **{key: int(value) for key, value in match.groupdict().items()},
    )


# --------------------------------------------------------------------------
# Snapshot collection: one snap=, one req=, every line exactly once
# --------------------------------------------------------------------------

def collect_numbered_snapshot(lines: list[str], prefix: str, total: int,
                              nonce: str) -> list[str] | None:
    """Return the lines of one complete numbered snapshot bound to nonce.

    Only lines "<prefix> <n>/<total> ... snap=S req=<nonce>" participate.
    A snapshot qualifies only when every line number 1..total arrived
    exactly once under one snap= id; duplicated line numbers poison that
    snapshot. The latest complete snapshot wins. Anything else -> None.
    """
    header = re.compile(
        r"^" + re.escape(prefix) + r" (\d+)/" + str(total) + r" ")
    groups: dict[int, dict[int, str]] = {}
    poisoned: set[int] = set()
    order: list[int] = []
    for raw in lines:
        line = raw.strip()
        head = header.match(line)
        if head is None:
            continue
        tail = SNAP_TAIL_RE.search(line)
        if tail is None or tail.group("request") != nonce:
            continue
        number = int(head.group(1))
        snap = int(tail.group("snap"))
        if snap not in groups:
            groups[snap] = {}
            order.append(snap)
        if number in groups[snap]:
            poisoned.add(snap)
            continue
        groups[snap][number] = line
    for snap in reversed(order):
        if snap in poisoned:
            continue
        group = groups[snap]
        if all(number in group for number in range(1, total + 1)):
            return [group[number] for number in range(1, total + 1)]
    return None


def parse_rx_debug_snapshot(
        lines: list[str], nonce: str) -> RxDebugStats | None:
    """Assemble one complete five-line, snap/nonce-bound RX snapshot."""
    snapshot = collect_numbered_snapshot(
        lines, "UART_RX_DEBUG", 5, nonce)
    if snapshot is None:
        return None
    frames = [parse_rx_debug(line) for line in snapshot]
    if any(frame is None for frame in frames):
        return None
    # collect_numbered_snapshot already pins line number, snap and nonce.
    # Keep the explicit per-part mapping here so a firmware format change
    # cannot silently create a partially populated health snapshot.
    one, two, three, four, five = frames
    assert one is not None and two is not None
    assert three is not None and four is not None
    assert five is not None
    return RxDebugStats(
        line="\n".join(snapshot),
        part=0,
        snap=one.snap,
        request=nonce,
        armed=one.armed,
        rearm_pending=one.rearm_pending,
        arm_attempt_sequence=one.arm_attempt_sequence,
        completion_sequence=one.completion_sequence,
        link_state=one.link_state,
        outage_age=one.outage_age,
        rearm_failures=two.rearm_failures,
        rearm_recoveries=two.rearm_recoveries,
        rearm_fault_events=two.rearm_fault_events,
        consecutive_failures=two.consecutive_failures,
        last_hal_status=two.last_hal_status,
        immediate_stop_pending=three.immediate_stop_pending,
        immediate_stop_events=three.immediate_stop_events,
        immediate_stop_dispatches=three.immediate_stop_dispatches,
        ring_overflow=three.ring_overflow,
        line_overflow=three.line_overflow,
        uart_error=four.uart_error,
        invalid_char=four.invalid_char,
        assembly_timeout=four.assembly_timeout,
        stale_line=four.stale_line,
        damage_generation=four.damage_generation,
        desync=four.desync,
        byte_budget_hits=five.byte_budget_hits,
        command_budget_hits=five.command_budget_hits,
        sync=five.sync,
    )


def parse_battery_debug(lines: list[str], nonce: str) -> BatteryDebug | None:
    snapshot = collect_numbered_snapshot(lines, "BAT_DEBUG", 3, nonce)
    if snapshot is None:
        return None
    first = BAT_DEBUG_1_RE.fullmatch(snapshot[0])
    second = BAT_DEBUG_2_RE.fullmatch(snapshot[1])
    if first is None or second is None:
        return None
    return BatteryDebug(
        requests=int(first.group("requests")),
        responses=int(first.group("responses")),
        timeout=int(second.group("timeout")),
        crc=int(second.group("crc")),
        exception=int(second.group("exception")),
        uart=int(second.group("uart")),
    )


def parse_tx_debug(lines: list[str], nonce: str) -> TxDebugStats | None:
    """Parse one complete, nonce-bound UART_TX_DEBUG snapshot."""
    snapshot = collect_numbered_snapshot(lines, "UART_TX_DEBUG", 6, nonce)
    if snapshot is None:
        return None
    match2 = TX_DEBUG_2_RE.fullmatch(snapshot[1])
    match4 = TX_DEBUG_4_RE.fullmatch(snapshot[3])
    match5 = TX_DEBUG_5_RE.fullmatch(snapshot[4])
    match6 = TX_DEBUG_6_RE.fullmatch(snapshot[5])
    if match2 is None or match4 is None or match5 is None or match6 is None:
        return None
    values2 = [int(value) for value in match2.groups()]
    values4 = [int(value) for value in match4.groups()]
    values5 = [int(value) for value in match5.groups()]
    values6 = [int(value) for value in match6.groups()]
    return TxDebugStats(
        depth=tuple(values2[0:4]), high_water=tuple(values2[4:8]),
        sent=tuple(values4[0:4]), dropped=tuple(values4[4:8]),
        hal_busy=values5[0], hal_error=values5[1], uart_error=values5[2],
        timeout=values5[3], recovery=values5[4],
        format_overflow=values6[0], completion=values6[1],
        resync=values6[2],
    )


def parse_yaw_control(lines: list[str], nonce: str) -> YawControlSnapshot | None:
    """Assemble one complete, nonce-bound YAW_CTRL snapshot (all 15 lines)."""
    snapshot = collect_numbered_snapshot(lines, "YAW_CTRL", 15, nonce)
    if snapshot is None:
        return None
    line3 = YAW_CTRL_3_RE.fullmatch(snapshot[2])
    line4 = YAW_CTRL_4_RE.fullmatch(snapshot[3])
    line11 = YAW_CTRL_11_RE.fullmatch(snapshot[10])
    line12 = YAW_CTRL_12_RE.fullmatch(snapshot[11])
    line13 = YAW_CTRL_13_RE.fullmatch(snapshot[12])
    if (line3 is None or line4 is None or line11 is None or
            line12 is None or line13 is None):
        return None
    return YawControlSnapshot(
        mode=line3.group("mode"),
        pi_enabled=int(line3.group("pi_enabled")),
        corrected_mrad_s=int(line3.group("corrected")),
        committed_mrad_s=int(line3.group("committed")),
        measured_mrad_s=int(line3.group("measured")),
        error_mrad_s=int(line3.group("error")),
        proposed_correction_mrad_s=int(line4.group("proposed")),
        submitted_correction_mrad_s=int(line4.group("submitted")),
        feedback_sequence_valid=int(line11.group("sequence_valid")),
        feedback_sequence=int(line11.group("sequence")),
        track_state=line12.group("track"),
        track_reason=line12.group("reason"),
        track_latch=int(line12.group("latch")),
        large_error_consecutive=int(line12.group("c1")),
        wrong_sign_consecutive=int(line12.group("c2")),
        divergence_consecutive=int(line12.group("c3")),
        obs_large_error=int(line13.group("err")),
        obs_correction_limit=int(line13.group("corr")),
        obs_wrong_sign=int(line13.group("sign")),
        obs_divergence=int(line13.group("div")),
        snap=int(line3.group("snap")),
    )


def final_base_safe_report(lines: list[str],
                           nonce: str) -> DriveBaseSafeReport | None:
    """Latest complete three-frame base-safe report bound to nonce.

    The three frames must share one snap= and echo the caller's req=; a
    duplicated frame under one snap poisons that snapshot. Frames from
    different snapshots are never mixed.
    """
    groups: dict[int, dict[str, object]] = {}
    poisoned: set[int] = set()
    order: list[int] = []

    def record(snap: int, key: str, value: object) -> None:
        if snap not in groups:
            groups[snap] = {}
            order.append(snap)
        if key in groups[snap]:
            poisoned.add(snap)
            return
        groups[snap][key] = value

    for raw in lines:
        line = raw.strip()
        summary = parse_summary(line)
        if summary is not None:
            if summary.request == nonce:
                record(summary.snap, "summary", summary)
            continue
        status = parse_status(line)
        if status is not None:
            if status.request == nonce:
                record(status.snap, "status", status)
            continue
        zero = parse_zero(line)
        if zero is not None:
            if zero.request == nonce:
                record(zero.snap, "zero", zero)
            continue

    for snap in reversed(order):
        if snap in poisoned:
            continue
        group = groups[snap]
        if "summary" in group and "status" in group and "zero" in group:
            return DriveBaseSafeReport(
                summary=group["summary"],
                status=group["status"],
                zero=group["zero"])
    return None


def final_refresh_event(
    lines: list[str],
    nonce: str,
) -> DriveBaseSafeRefreshEvent | None:
    event: DriveBaseSafeRefreshEvent | None = None

    for raw_line in lines:
        line = raw_line.strip()
        if line.startswith("DRIVE_BASE_SAFE_REFRESH "):
            parsed = parse_refresh_event(line)
            if parsed is not None and parsed.request == nonce:
                event = parsed
    return event


def base_safe(report: DriveBaseSafeReport | None) -> bool:
    if report is None:
        return False
    summary = report.summary
    status = report.status
    zero = report.zero
    unsafe_status_mask = (1 << 2) | (1 << 3)
    return all((
        summary.safe == 1,
        summary.active == 0,
        summary.enable == 0,
        summary.disable == 1,
        summary.drive_enabled == 0,
        summary.target_zero == 1,
        summary.pending == 0,
        summary.zero == 1,
        status.left_valid == 1,
        status.right_valid == 1,
        status.left_fresh == 1,
        status.right_fresh == 1,
        status.left_sequence != 0,
        status.right_sequence != 0,
        (status.left_status & unsafe_status_mask) == 0,
        (status.right_status & unsafe_status_mask) == 0,
        zero.valid == 1,
        status.left_generation == zero.safety_generation,
        status.right_generation == zero.safety_generation,
        zero.generation == zero.safety_generation,
    ))


def recovery_gate_allows_reset(
    refresh: DriveBaseSafeRefreshEvent | None,
) -> bool:
    return bool(
        refresh is not None and
        refresh.state == "COMPLETE" and
        refresh.safe == 1
    )


def lift_reset_status_is_safe(status: LiftStatus | None) -> bool:
    return bool(
        status is not None and
        status.state == "RECOVERY_REQUIRED" and
        status.homed == 0 and
        status.fault == 0
    )


def lift_reset_line_is_explicit_failure(line: str) -> bool:
    clean = line.strip()
    return bool(
        clean.startswith("LIFT reason=") or
        clean.startswith("LIFT pending action failed") or
        clean.startswith("LIFT command rejected:") or
        "BASE_STOP_FAILED" in clean or
        "NODE3_COMM_TIMEOUT" in clean or
        "ACTION_START_FAILED" in clean
    )


def embedded_prefix(line: str) -> bool:
    """True when another known frame prefix is embedded mid-line."""
    for prefix in KNOWN_PREFIXES:
        if line.find(prefix, 1) >= 0:
            return True
    return False


# --------------------------------------------------------------------------
# Interval statistics (uart-soak quality gates)
# --------------------------------------------------------------------------

def percentile(values: list[float], fraction: float) -> float:
    """Nearest-rank percentile; values need not be pre-sorted."""
    if not values:
        raise ValueError("percentile of empty sequence")
    ordered = sorted(values)
    rank = max(1, math.ceil(fraction * len(ordered)))
    return float(ordered[rank - 1])


def interval_stats(timestamps_ms: list[int],
                   gap_threshold_ms: float) -> IntervalStats:
    """Statistics over consecutive firmware-timestamp intervals.

    A median inside the nominal band is not sufficient: a stream can hold a
    10 ms median while hiding multi-hundred-ms stalls. p95/p99/max and the
    explicit gap count make those stalls visible.
    """
    backward = 0
    intervals: list[float] = []
    for previous, current in zip(timestamps_ms, timestamps_ms[1:]):
        if current < previous:
            backward += 1
            continue
        intervals.append(float(current - previous))
    if not intervals:
        raise ValueError("no forward timestamp intervals")
    ordered = sorted(intervals)
    midpoint = len(ordered) // 2
    if len(ordered) % 2:
        median = ordered[midpoint]
    else:
        median = (ordered[midpoint - 1] + ordered[midpoint]) / 2.0
    return IntervalStats(
        count=len(intervals),
        median_ms=median,
        p95_ms=percentile(intervals, 0.95),
        p99_ms=percentile(intervals, 0.99),
        max_ms=max(intervals),
        backward_count=backward,
        gap_count=sum(1 for value in intervals if value > gap_threshold_ms),
    )


# --------------------------------------------------------------------------
# Transactions (host side)
# --------------------------------------------------------------------------

class TransactionError(RuntimeError):
    """A firmware transaction failed or returned unusable evidence."""


def run_tpdo_config_transaction(
    io,
    node_id: int,
    timeout_s: float,
    nonce: str,
    *,
    clock=time.monotonic,
) -> WheelTpdoConfigEvent:
    """Configure one wheel TPDO with a strictly nonce-bound transaction."""
    if node_id not in (1, 2):
        raise ValueError("TPDO runner configures exactly node 1 or node 2")
    deadline = clock() + timeout_s
    io.send(f"DRIVE_TPDO_CONFIG node={node_id} req={nonce}")
    started = False
    while clock() < deadline:
        line = io.read_line()
        if line is None:
            continue
        if not line.startswith("DRIVE_TPDO_CONFIG "):
            continue
        event = parse_tpdo_config_event(line)
        if event is None:
            raise TransactionError(
                "malformed TPDO configuration response: " + line.strip())
        if event.request != nonce:
            continue
        if event.node != node_id:
            raise TransactionError(
                "TPDO configuration response used wrong node: " + event.line)
        if event.state == "STARTED":
            if started:
                raise TransactionError(
                    "duplicate TPDO configuration STARTED: " + event.line)
            started = True
            continue
        if event.state == "FAILED":
            raise TransactionError(
                "TPDO configuration failed: " + event.line)
        if event.state == "COMPLETE":
            if not started:
                raise TransactionError(
                    "TPDO configuration COMPLETE before STARTED: "
                    + event.line)
            return event
    raise TransactionError("TPDO configuration timed out")


def run_tpdo_active_transaction(
        io, command: str, timeout_s: float, nonce: str, *,
        clock=time.monotonic) -> WheelTpdoActiveEvent:
    if command not in ("DRIVE_TPDO_ACTIVE_ENABLE",
                       "DRIVE_TPDO_ACTIVE_DISABLE"):
        raise ValueError("unsupported TPDO Active transaction")
    response_command = ("DRIVE_TPDO_ACTIVE" if command.endswith("ENABLE")
                        else "DRIVE_TPDO_ACTIVE_DISABLE")
    deadline = clock() + timeout_s
    io.send(f"{command} req={nonce}")
    started = False
    while clock() < deadline:
        line = io.read_line()
        if line is None or not line.startswith(response_command + " "):
            continue
        event = parse_tpdo_active_event(line)
        if event is None:
            raise TransactionError(
                "malformed TPDO Active response: " + line.strip())
        if event.request != nonce:
            continue
        if event.command != response_command:
            raise TransactionError("wrong TPDO Active response command")
        if event.state == "STARTED":
            if started:
                raise TransactionError("duplicate TPDO Active STARTED")
            started = True
            continue
        if event.state == "FAILED":
            raise TransactionError("TPDO Active command failed: " + event.line)
        if not started:
            raise TransactionError("TPDO Active COMPLETE before STARTED")
        return event
    raise TransactionError("TPDO Active transaction timed out")


def run_drive_fault_reset_transaction(
    io,
    timeout_s: float,
    nonce: str,
    *,
    clock=time.monotonic,
) -> DriveFaultResetEvent:
    """Send DRIVE_FAULT_RESET_SAFE bound to a host nonce.

    ``io`` needs ``send(command)`` and ``read_line() -> str | None``. Only
    response lines echoing ``req=<nonce>`` participate; among those, the
    terminal event must additionally match this command's STARTED gen=.
    A frame still queued inside the firmware TX manager from an earlier
    command carries a different req= and is ignored.
    """
    deadline = clock() + timeout_s
    io.send(f"DRIVE_FAULT_RESET_SAFE req={nonce}")
    generation: int | None = None
    while clock() < deadline:
        line = io.read_line()
        if line is None:
            continue
        event = parse_fault_reset_event(line)
        if event is None or event.request != nonce:
            continue
        if event.state == "STARTED":
            generation = event.generation
            continue
        if event.state == "FAILED":
            raise TransactionError(
                "drive fault reset rejected/failed: " + event.line)
        if generation is None or event.generation != generation:
            continue
        if event.state == "PENDING":
            continue
        if event.state == "COMPLETE":
            if event.left_cleared == 1 and event.right_cleared == 1:
                return event
            raise TransactionError(
                "drive fault reset returned incomplete clear evidence: "
                + event.line)
    raise TransactionError("drive fault reset timed out")


def _run_lift_transaction(
    io,
    timeout_s: float,
    nonce: str,
    command: str,
    parser,
    *,
    clock=time.monotonic,
) -> LiftTransactionEvent:
    """Shared LIFT_FAULT_RESET / LIFT_HOME transaction runner.

    Only lines echoing req=<nonce> participate; the terminal event must
    additionally match this command's STARTED txn=. Legacy unbound lines
    ("LIFT_FAULT_RESET complete", "LIFT sensor homing complete") never
    parse as transaction events and can never satisfy the transaction.
    """
    deadline = clock() + timeout_s
    io.send(f"{command} req={nonce}")
    txn: int | None = None
    while clock() < deadline:
        line = io.read_line()
        if line is None:
            continue
        event = parser(line)
        if event is None or event.request != nonce:
            continue
        if event.state == "STARTED":
            txn = event.txn
            continue
        if event.state == "FAILED":
            raise TransactionError(
                f"{command} rejected/failed: " + event.line)
        if txn is None or event.txn != txn:
            continue
        if event.state == "PENDING":
            continue
        if event.state == "COMPLETE":
            if (command == "LIFT_HOME") and (event.homed != 1):
                raise TransactionError(
                    "LIFT_HOME completed without homed=1: " + event.line)
            return event
    raise TransactionError(f"{command} transaction timed out")


def run_lift_fault_reset_transaction(
    io, timeout_s: float, nonce: str, *, clock=time.monotonic,
) -> LiftTransactionEvent:
    return _run_lift_transaction(
        io, timeout_s, nonce, "LIFT_FAULT_RESET", parse_lift_reset_event,
        clock=clock)


def run_lift_home_transaction(
    io, timeout_s: float, nonce: str, *, clock=time.monotonic,
) -> LiftTransactionEvent:
    return _run_lift_transaction(
        io, timeout_s, nonce, "LIFT_HOME", parse_lift_home_event,
        clock=clock)


def run_base_safe_refresh_transaction(
    io,
    timeout_s: float,
    nonce: str,
    *,
    clock=time.monotonic,
) -> DriveBaseSafeRefreshEvent:
    """Send DRIVE_BASE_SAFE_REFRESH bound to a host nonce.

    Every path (immediate COMPLETE, BUSY, PRECONDITION, asynchronous
    refresh) responds with lines echoing req=<nonce> and this command's
    txn=. COMPLETE is accepted only after this command's STARTED and only
    with the same txn.
    """
    deadline = clock() + timeout_s
    io.send(f"DRIVE_BASE_SAFE_REFRESH req={nonce}")
    txn: int | None = None
    while clock() < deadline:
        line = io.read_line()
        if line is None:
            continue
        event = parse_refresh_event(line)
        if event is None or event.request != nonce:
            continue
        if event.state == "STARTED":
            txn = event.txn
            continue
        if event.state == "FAILED":
            raise TransactionError(
                "base-safe refresh failed: " + event.line)
        if txn is None or event.txn != txn:
            continue
        if event.state == "PENDING":
            continue
        if event.state == "COMPLETE":
            return event
    raise TransactionError("base-safe refresh timed out")


# --------------------------------------------------------------------------
# Serial session with leveled fail-closed cleanup
# --------------------------------------------------------------------------

# Commands that arm motion. Sending any of these obligates the session to a
# full emergency cleanup on every exit path.
_ARMING_COMMANDS = frozenset((
    "ENABLE", "LIFT_HOME", "LIFT_RECOVER", "LIFT_RECOVER_TOP",
    "LIFT_UP", "LIFT_DOWN",
))


def command_is_arming(command: str) -> bool:
    tokens = command.split()
    if not tokens:
        return False
    name = tokens[0]
    if name in _ARMING_COMMANDS:
        return True
    if name in ("CMD_VEL", "RAW"):
        for token in tokens[1:]:
            try:
                if abs(float(token)) > 1e-3:
                    return True
            except ValueError:
                return True
    return False


def command_is_read_only(command: str) -> bool:
    tokens = command.split()
    if not tokens:
        return True
    if tokens[0] == "YAW_PI":
        return len(tokens) >= 2 and tokens[1] == "STATUS"
    return tokens[0] in READ_ONLY_COMMANDS


class SerialSession:
    """Logging serial session with nonce transactions and cleanup discipline.

    Cleanup levels:
      - pure read-only session, any exit          -> close the port, nothing else
      - state-changing session, normal exit       -> close the port only if no
        motion was armed; otherwise full safe-command cleanup
      - state-changing session, signal/exception  -> full safe-command cleanup
    """

    def __init__(self, port: str, log_path, *, serial_factory=None) -> None:
        if serial_factory is None:
            import serial as pyserial

            def serial_factory(device):
                return pyserial.Serial(
                    port=device,
                    baudrate=460800,
                    bytesize=pyserial.EIGHTBITS,
                    parity=pyserial.PARITY_NONE,
                    stopbits=pyserial.STOPBITS_ONE,
                    timeout=0.05,
                    write_timeout=0.5,
                    # Exclusive open replaces the old "fuser -k everything
                    # on the port" behavior: a busy port is an error, not a
                    # license to kill the other process.
                    exclusive=True,
                )

        self.serial = serial_factory(port)
        self.log = log_path.open("wb") if log_path is not None else None
        self.cleaned = False
        self.sent_state_changing = False
        self.sent_arming = False
        self.last_ack_sequence: int | None = None
        self.console_suppressed_prefixes: tuple[str, ...] = ()

    # -- basic IO ----------------------------------------------------------

    def send(self, command: str) -> None:
        if command_is_arming(command):
            self.sent_arming = True
            self.sent_state_changing = True
        elif not command_is_read_only(command):
            self.sent_state_changing = True
        wire = (command + "\r\n").encode("ascii")
        self.serial.write(wire)
        self.serial.flush()
        if self.log is not None:
            self.log.write(b">>> " + wire)
            self.log.flush()

    def read_line(self) -> str | None:
        raw = self.serial.readline()
        if not raw:
            return None
        if self.log is not None:
            self.log.write(raw)
            self.log.flush()
        line = raw.decode("ascii", errors="replace").rstrip("\r\n")
        if not line.startswith(self.console_suppressed_prefixes):
            print(line)
        return line

    def read_for(self, seconds: float) -> list[str]:
        deadline = time.monotonic() + seconds
        lines: list[str] = []
        while time.monotonic() < deadline:
            line = self.read_line()
            if line is not None:
                lines.append(line)
        return lines

    def wait_for(self, pattern: re.Pattern[str],
                 timeout: float) -> tuple[str, list[str]]:
        deadline = time.monotonic() + timeout
        lines: list[str] = []
        while time.monotonic() < deadline:
            line = self.read_line()
            if line is None:
                continue
            lines.append(line)
            if pattern.search(line):
                return line, lines
        raise TransactionError(f"timeout waiting for: {pattern.pattern}")

    # -- transaction primitives -------------------------------------------

    def input_boundary(self, drain_s: float = 0.3) -> None:
        """Discard buffered stale RX data before starting a transaction.

        This cannot flush frames still queued inside the firmware TX
        manager; the req= nonce binding is what protects against those.
        """
        self.serial.reset_input_buffer()
        deadline = time.monotonic() + drain_s
        while time.monotonic() < deadline:
            if self.read_line() is None:
                break

    def query(self, command: str, seconds: float = 2.0) -> list[str]:
        self.send(command)
        return self.read_for(seconds)

    def transact_ack(self, command: str,
                     timeout: float = 5.0) -> tuple[CommandAck, list[str]]:
        """Send one acknowledged command; return its nonce-bound CMD_ACK.

        The returned ack is provably for this command: it must echo the
        req= nonce generated here. A stale CMD_ACK from the firmware TX
        queue carries a different nonce and can never satisfy this call.
        """
        nonce = make_nonce()
        self.input_boundary()
        self.send(f"{command} req={nonce}")
        expected = command.split()[0]
        deadline = time.monotonic() + timeout
        lines: list[str] = []
        while time.monotonic() < deadline:
            line = self.read_line()
            if line is None:
                continue
            lines.append(line)
            ack = parse_command_ack(line)
            if ack is None or ack.command != expected:
                continue
            if ack.reason in ("BAD_REQ", "BAD_COMMAND"):
                # The firmware refused the request grammar; the command did
                # not run. This is a host-side protocol bug — fail fast.
                raise TransactionError(
                    f"firmware rejected the request nonce for {expected}: "
                    + ack.line)
            if ack.request != nonce:
                continue
            self.last_ack_sequence = ack.sequence
            return ack, lines
        raise TransactionError(f"no nonce-bound CMD_ACK for {expected}")

    def wait_for_odom_state(self, state: str, timeout: float) -> OdomStatus:
        """Wait for an ODOM line proving the requested drive state.

        Must only be called after transact_ack() for the state-changing
        command, so every line read here postdates the acknowledgement.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self.read_line()
            if line is None:
                continue
            odom = parse_odom(line)
            if odom is not None and odom.state == state:
                return odom
        raise TransactionError(f"drive did not report ODOM state {state}")

    def drive_fault_reset(self, timeout: float = 30.0) -> DriveFaultResetEvent:
        nonce = make_nonce()
        self.input_boundary()
        return run_drive_fault_reset_transaction(self, timeout, nonce)

    def base_safe_refresh(self, timeout: float = 20.0) -> DriveBaseSafeRefreshEvent:
        nonce = make_nonce()
        self.input_boundary()
        return run_base_safe_refresh_transaction(self, timeout, nonce)

    def lift_fault_reset(self, timeout: float = 15.0) -> LiftTransactionEvent:
        nonce = make_nonce()
        self.input_boundary()
        return run_lift_fault_reset_transaction(self, timeout, nonce)

    def lift_home(self, timeout: float = 120.0) -> LiftTransactionEvent:
        nonce = make_nonce()
        self.input_boundary()
        return run_lift_home_transaction(self, timeout, nonce)

    def wheel_tpdo_config(
            self, node_id: int,
            timeout: float = 12.0) -> WheelTpdoConfigEvent:
        nonce = make_nonce()
        self.input_boundary()
        return run_tpdo_config_transaction(
            self, node_id, timeout, nonce)

    def wheel_tpdo_status(
            self, timeout: float = 12.0, *,
            exclude_nonce: str | None = None) -> WheelTpdoStatusSnapshot:
        nonce = make_nonce()
        while nonce == exclude_nonce:
            nonce = make_nonce()
        self.input_boundary()
        self.send(f"DRIVE_TPDO_STATUS req={nonce}")
        deadline = time.monotonic() + timeout
        lines: list[str] = []
        while time.monotonic() < deadline:
            line = self.read_line()
            if line is None:
                continue
            lines.append(line)
            snapshot = parse_tpdo_status(lines, nonce)
            if snapshot is not None:
                return snapshot
        raise TransactionError("TPDO status transaction timed out")

    def wheel_tpdo_active_command(
            self, enable: bool, timeout: float = 3.0) -> WheelTpdoActiveEvent:
        nonce = make_nonce()
        self.input_boundary()
        command = ("DRIVE_TPDO_ACTIVE_ENABLE" if enable
                   else "DRIVE_TPDO_ACTIVE_DISABLE")
        return run_tpdo_active_transaction(
            self, command, timeout, nonce)

    def wheel_tpdo_active_status(
            self, timeout: float = 3.0) -> WheelTpdoActiveStatusSnapshot:
        nonce = make_nonce()
        self.input_boundary()
        self.send(f"DRIVE_TPDO_ACTIVE_STATUS req={nonce}")
        deadline = time.monotonic() + timeout
        lines: list[str] = []
        while time.monotonic() < deadline:
            line = self.read_line()
            if line is None:
                continue
            lines.append(line)
            snapshot = parse_tpdo_active_status(lines, nonce)
            if snapshot is not None:
                return snapshot
        raise TransactionError("TPDO Active status transaction timed out")

    def query_bound(self, command: str, assemble,
                    timeout: float = 3.0):
        """Send a nonce-bound query; return assemble(lines, nonce) result.

        ``assemble`` must return None until a complete nonce-bound snapshot
        has arrived; its non-None result is returned.
        """
        nonce = make_nonce()
        self.input_boundary()
        self.send(f"{command} req={nonce}")
        deadline = time.monotonic() + timeout
        lines: list[str] = []
        while time.monotonic() < deadline:
            line = self.read_line()
            if line is None:
                continue
            lines.append(line)
            result = assemble(lines, nonce)
            if result is not None:
                return result
        return None

    def query_base_safe(self, timeout: float = 3.0) -> DriveBaseSafeReport | None:
        return self.query_bound(
            "DRIVE_BASE_SAFE", final_base_safe_report, timeout)

    # -- leveled cleanup ---------------------------------------------------

    def cleanup(self, *, emergency: bool = False) -> None:
        if self.cleaned:
            return
        self.cleaned = True
        needs_safe_commands = self.sent_arming or (
            emergency and self.sent_state_changing)
        if needs_safe_commands:
            for command in SAFE_COMMANDS:
                try:
                    wire = (command + "\r\n").encode("ascii")
                    self.serial.write(wire)
                    self.serial.flush()
                    if self.log is not None:
                        self.log.write(b">>> " + wire)
                        self.log.flush()
                    time.sleep(0.05)
                except Exception as exc:  # noqa: BLE001 - report and continue
                    print(f"WARNING: cleanup {command!r} failed: {exc}")
            try:
                self.read_for(0.4)
            except Exception:
                pass
        elif self.sent_state_changing:
            print("NOTE: state-changing session closed normally; "
                  "no additional safety commands were sent")
        else:
            print("NOTE: read-only session; no state commands were sent "
                  "at exit")
        if self.log is not None:
            self.log.close()
        self.serial.close()
