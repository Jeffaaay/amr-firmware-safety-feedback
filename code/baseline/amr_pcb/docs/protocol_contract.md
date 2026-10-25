# Protocol Contract

Current protocol version: **5**. Protocol v5 keeps the protocol-v4
`req=`/`snap=`/`gen=`/`txn=` transaction-binding model unchanged and makes the
single `ODOM` grammar's signed wheel fields unambiguously milli-RPM. Because
interpreting those integers as whole RPM is unsafe, v5 hosts must refuse
protocol-4 firmware; there is no silent compatibility mode.

## Jetson UART

Additive telemetry: `ESTOP v=1` reports the physical PD15 input independently
of ODOM or recovery status. The overall firmware protocol remains 5; `v=1`
versions only this new line. Its complete field/timing/order contract is in
[PD15 emergency-stop telemetry](PD15_ESTOP_TELEMETRY.md). Existing host parsers
may ignore the new prefix; a Jetson base-driver update is still required to
expose it to ROS/UI. It is not an acknowledgment of a stop command.

- Peripheral: USART3.
- Serial configuration: 460800 8N1.
- USART3 FIFO mode is disabled.
- Commands and status lines are CRLF-terminated ASCII.
- RX uses `HAL_UARTEx_ReceiveToIdle_DMA` on DMA1 Stream0 in circular,
  peripheral-to-memory, byte-aligned, memory-increment, high-priority mode.
  The USART3 and DMA interrupts remain enabled; DMA HT/TC and UART IDLE events
  publish producer progress, while parsing and DMA restart stay in main context.
- USART3 never uses the one-byte `HAL_UART_Receive_IT` path.
  `HAL_UART_RxCpltCallback` remains the global USART2 battery completion route.
- TX is owned by the nonblocking Jetson TX manager. Producers enqueue one
  complete copied frame; only the manager starts HAL_UART_Transmit_IT.
- TX priority is CRITICAL, REALTIME (IMU), STATUS (ODOM/LIFT/BAT), then DEBUG.
  During overload, queued DEBUG may be dropped; a full REALTIME or STATUS
  queue discards its oldest queued frame to retain the newest sample or state.
  An in-flight frame is never overwritten. A UART error or bounded in-flight
  timeout aborts and drops the possibly partial frame, then transmits a CRLF
  delimiter before queued traffic resumes.
- These queue policies must not change the text or parseability of any
  transmitted structured frame.

Key commands:

```text
CMD_VEL <linear_int> <angular_int>
RAW <left_rpm> <right_rpm> <duration_ms>   (disabled unless raw_test=1)
STOP
ENABLE
DISABLE
YAW_PI ON
YAW_PI OFF
YAW_PI STATUS
LIGHT RGB <R> <G> <B> [req=<8-hex nonce>]
FW_VERSION
VERSION
LIFT_HOME
LIFT_RECOVER          (alias of LIFT_HOME - starts sensor homing)
LIFT_RECOVER_TOP      (alias of LIFT_HOME - starts sensor homing)
LIFT_FAULT_RESET
LIFT_RST              (alias of LIFT_FAULT_RESET - fault reset only)
FAULT_RST             (alias of LIFT_FAULT_RESET - fault reset only)
LIFT_UP
LIFT_DOWN
LIFT_STOP
LIFT_STATUS
LIFT_RAW
LIFT_RAW_STATUS
LIFT_SCALE_INFO
BAT_DEBUG
DRIVE_YAW_DEBUG
UART_TX_DEBUG
UART_RX_DEBUG
DRIVE_CAN_TRACE_STATUS req=<nonce>
DRIVE_CAN_TRACE_DUMP req=<nonce>
DRIVE_FAULT_DEBUG
DRIVE_FAULT_RESET_SAFE
DRIVE_BASE_SAFE_REFRESH
DRIVE_BASE_SAFE
DRIVE_TPDO_CONFIG node=<1|2> req=<nonce>
DRIVE_TPDO_STATUS req=<nonce>
DRIVE_TPDO_ACTIVE_ENABLE req=<nonce>
DRIVE_TPDO_ACTIVE_DISABLE req=<nonce>
DRIVE_TPDO_ACTIVE_STATUS req=<nonce>
```

## RGB status light

Send `LIGHT RGB <R> <G> <B>` followed by CRLF. Each channel must be exactly
`0` (OFF) or `1` (ON); this is binary GPIO control, with no PWM or brightness
values. Command tokens are uppercase and separated by one or more plain
spaces. No extra arguments or trailing whitespace are accepted.

| R G B | Color |
| --- | --- |
| 0 0 0 | OFF |
| 1 0 0 | RED |
| 0 1 0 | GREEN |
| 0 0 1 | BLUE |
| 1 1 0 | YELLOW |
| 1 0 1 | MAGENTA |
| 0 1 1 | CYAN |
| 1 1 1 | WHITE |

The existing `CMD_ACK` convention applies:

```text
LIGHT RGB 0 0 1
CMD_ACK LIGHT seq=<n> accepted=1 reason=OK req=00000000
LIGHT RGB 0 0 1 req=12345678
CMD_ACK LIGHT seq=<n> accepted=1 reason=OK req=12345678
LIGHT RGB 2 0 0
CMD_ACK LIGHT seq=<n> accepted=0 reason=BAD_COMMAND req=00000000
```

The optional final `req=` token uses the existing nonzero eight-hex-digit
nonce grammar. An invalid nonce returns `accepted=0 reason=BAD_REQ` with
`req=00000000`. Invalid RGB syntax returns `BAD_COMMAND` and echoes a valid
nonce if supplied. Any rejection leaves the stored request unchanged.
An accepted ACK means the request was stored; an active fault can still
override the displayed color.

At each main-loop status update, `Drive_IsFaulted() || Lift_IsFaulted()`
forces solid RED with green and blue OFF. Otherwise the last valid Jetson
request is displayed, including RED or OFF. Before the first valid request,
the no-fault default is GREEN. Faults never erase the request, and valid
commands received during a fault update the saved color for restoration
when both faults clear. The request is held in RAM until reboot; there is no
lighting timeout. Existing motor, lift, watchdog and communication fault
behavior is unchanged.

`StatusLight` owns this decision and the parser only stores requests.
`SafetyIo_SetRgb` converts ON to GPIO RESET (low) and OFF to GPIO SET (high)
on PF0 (red), PE6 (green), and PE5 (blue). Unwanted channels are disabled
before requested channels are enabled, without pulsing active channels off
on each iteration.

```text
Drive/Lift fault?
    +-- YES -> RED
    +-- NO
          +-- Jetson request exists -> latest Jetson RGB
          +-- No Jetson request -> GREEN
```

The production-C lighting tests run as part of
`python tools/run_wheel_rpdo_tests.py` and therefore `BUILD_FIRMWARE.bat`.
They check all eight colors, each fault source and both together, restoration,
commands received during faults, rejected commands, ACKs, GPIO polarity/write
ordering, and UART line/DMA dispatch. Physical color wiring remains a hardware
verification item.

## Status and diagnostics

Key status/diagnostic lines:

```text
ODOM <x_mm> <y_mm> <theta_mrad> <left_mrpm> <right_mrpm> <drive_state> <fault>
LIFT <state> <pos_mm_rounded> <target_mm_rounded> <pos_pulses> <homed> <fault>
IMU <...>
BAT <t_ms> <health> <fault> <age> <voltage_mV> <current_mA> <remaining_mAh> <rsoc> <temp1_dC> <temp2_dC> <min_cell_mV> <max_cell_mV>
FW variant=<variant> build=<build-id> profile=<BENCH_DEBUG|RELEASE_TEST> optimization=<O0|Og> date="<compile-date>" time=<compile-time> protocol=<version> txmgr=1 pi=<0|1> raw_test=<0|1>
WHEEL_FEEDBACK_HEALTH v=1 mode=<mode> authority=TPDO pair_valid=<0|1> pair_state=<NO_DATA|FRESH|STALE|FAULT> pair_seq=<n> pair_age_ms=<n> pair_skew_ms=<n> sdo606c=<0|1>
WHEEL_FEEDBACK_HEALTH_NODE v=1 node1_seq=<n> node1_age_ms=<n> node2_seq=<n> node2_age_ms=<n>
UART_RX_DEBUG 1/5 armed=<0|1> rearm_pending=<0|1> arm_seq=<n> completion_seq=<n> link_state=<HEALTHY|OUTAGE_PENDING|OUTAGE_FAULTED> outage_age=<ms> snap=<snapshot> req=<nonce>
UART_RX_DEBUG 2/5 rearm_fail=<n> rearm_recover=<n> rearm_fault=<n> consecutive=<n> last_hal=<status> snap=<snapshot> req=<nonce>
UART_RX_DEBUG 3/5 stop_pending=<0|1> stop_events=<n> stop_dispatch=<n> ring_ovf=<n> line_ovf=<n> snap=<snapshot> req=<nonce>
UART_RX_DEBUG 4/5 uart_err=<n> bad_ch=<n> asm_to=<n> stale=<n> damage_gen=<n> desync=<n> snap=<snapshot> req=<nonce>
UART_RX_DEBUG 5/5 byte_budget=<n> cmd_budget=<n> sync=<0|1> snap=<snapshot> req=<nonce>
CMD_ACK <command> seq=<monotonic-count> accepted=<0|1> reason=<OK|BAD_REQ|BAD_COMMAND|RAW_DISABLED|TRANSITION_BUSY|LIFT_ACTIVE|NOT_CONFIRMED_DISABLED|FAULT_LATCHED|NOT_ENABLED|BASE_NOT_SAFE|OPERATION_ENABLED|INTERNAL_ERROR> req=<8-hex host nonce>
The following legacy fault snapshot lines are available only in the
`BENCH_DEBUG` profile. `RELEASE_TEST` accepts the read-only command for
compatibility but emits none of these verbose lines:

DRIVE_FAULT_CURRENT state=<state> active=<0|1> type=<id> phase=<id> en=<0|1> dis=<0|1> drive=<0|1> tracking=<0|1> inhibit=<0|1> zero=<0|1> age=<ms> gen=<evidence>/<safety>
DRIVE_FAULT_EVENT event=<0|1> cur=<state> evt=<event-state> type=<transition> phase=<phase> reason=<reason> node=<id> age=<ms>
DRIVE_FAULT_STATUS L=<statusword>/<valid>/<age> R=<statusword>/<valid>/<age> abort=<code> sdo_to=<count> trans_to=<count>
DRIVE_FAULT_EMCY L=<code>/<sequence>/<age> R=<code>/<sequence>/<age>
DRIVE_FAULT_HB L=<state>/<sequence>/<age> R=<state>/<sequence>/<age> en=<0|1> dis=<0|1> inhibit=<0|1> tracking=<0|1>
DRIVE_FAULT_RESET_SAFE state=STARTED gen=<transaction> req=<nonce>
DRIVE_FAULT_RESET_SAFE state=PENDING phase=<phase> node=<1|2> gen=<transaction> req=<nonce>
DRIVE_FAULT_RESET_SAFE state=COMPLETE cleared=<L>/<R> gen=<transaction> req=<nonce>
DRIVE_FAULT_RESET_SAFE state=FAILED reason=<reason> node=<0|1|2> phase=<phase> abort=<code> timeouts=<count> error=<0x603F>/<valid> reg=<0x1001>/<valid> gen=<transaction> req=<nonce>
DRIVE_FAULT_RESET_DIAG active=<0|1> result=<result> phase=<phase> node=<0|1|2> failure=<reason> fail_node=<0|1|2> abort=<code> timeouts=<count> cleared=<L>/<R>
DRIVE_FAULT_RESET_LEFT node=1 sw=<value>/<valid>/<age>/<sequence> error=<value>/<valid>/<age>/<sequence> reg=<value>/<valid>/<age>/<sequence> reset=<0|1> cleared=<0|1>
DRIVE_FAULT_RESET_RIGHT node=2 sw=<value>/<valid>/<age>/<sequence> error=<value>/<valid>/<age>/<sequence> reg=<value>/<valid>/<age>/<sequence> reset=<0|1> cleared=<0|1>
DRIVE_BASE_SAFE safe=<0|1> state=<state> active=<0|1> en=<0|1> dis=<0|1> drive=<0|1> target_zero=<0|1> pending=<mask> zero=<0|1> snap=<snapshot> req=<nonce>
DRIVE_BASE_SAFE_SW valid=<L>/<R> fresh=<L>/<R> sw=<L>/<R> age=<L>/<R> seq=<L>/<R> gen=<L>/<R> snap=<snapshot> req=<nonce>
DRIVE_BASE_SAFE_ZERO valid=<0|1> age=<ms> gen=<evidence>/<safety> snap=<snapshot> req=<nonce>
DRIVE_BASE_SAFE_REFRESH state=STARTED gen=<safety-generation> txn=<per-command> req=<nonce>
DRIVE_BASE_SAFE_REFRESH state=PENDING phase=<SETTLING|CONFIRMING|LEFT_STATUS|RIGHT_STATUS> txn=<per-command> req=<nonce>
DRIVE_BASE_SAFE_REFRESH state=COMPLETE safe=<0|1> gen=<safety-generation> txn=<per-command> req=<nonce>
DRIVE_BASE_SAFE_REFRESH state=FAILED reason=<reason> node=<0|1|2> txn=<per-command> req=<nonce>
DRIVE_BASE_SAFE_REFRESH state=FAILED reason=BUSY active_txn=<in-flight txn> txn=<this command> req=<this nonce>
DRIVE_FAULT_RESET_SAFE state=FAILED reason=BUSY active_gen=<in-flight gen> gen=<this command> req=<this nonce>
LIFT_FAULT_RESET state=STARTED txn=<per-command> req=<nonce>
LIFT_FAULT_RESET state=PENDING phase=P<phase> txn=<per-command> req=<nonce>
LIFT_FAULT_RESET state=COMPLETE txn=<per-command> req=<nonce>
LIFT_FAULT_RESET state=FAILED reason=<reason> txn=<per-command> req=<nonce>
LIFT_FAULT_RESET state=FAILED reason=BUSY active_txn=<in-flight txn> txn=<this command> req=<this nonce>
LIFT_HOME state=STARTED txn=<per-command> req=<nonce>
LIFT_HOME state=PENDING phase=<P<phase>|H<phase>> txn=<per-command> req=<nonce>
LIFT_HOME state=COMPLETE homed=1 txn=<per-command> req=<nonce>
LIFT_HOME state=FAILED reason=<reason> txn=<per-command> req=<nonce>
LIFT_HOME state=FAILED reason=BUSY active_txn=<in-flight txn> txn=<this command> req=<this nonce>
LIFT_RAW 1/2 fw=<variant> state=<state> phase=<phase> homed=<0|1> home_method=SENSOR
LIFT_RAW 2/2 statusword=<0x6041|-> error_code=<0x603F|-> requested_mode=<0x6060|-> displayed_mode=<0x6061|-> homing_method=<0x6098|-> reason=<internal_reason>
YAW_PI enabled=<0|1> default=<0|1> accepted=1
YAW_PI enabled=<0|1> default=<0|1> status=OK mode=<mode> active=<0|1> primed=<0|1> feedback_sequence=<sequence>
YAW_PI tracking feedback_sequence_valid=<0|1> last_consumed_feedback_sequence=<sequence>
YAW_PI rejected=moving
```

The five-line `UART_RX_DEBUG` grammar is intentionally unchanged. Its transport
fields now describe the persistent DMA receiver: `armed=1` means the circular
DMA session is active, `rearm_pending=1` means a cooperative DMA restart is
pending, `arm_seq` counts start/restart attempts, and `completion_seq` is the
USART3 DMA RxEvent sequence rather than a one-byte completion count.
`rearm_fail`, `rearm_recover`, `consecutive`, `last_hal`, and `rearm_fault`
describe DMA start/restart outage episodes. `ring_ovf` aggregates software-ring
overflow and DMA overwrite; an impossible DMA phase is surfaced through
`uart_err` because adding a sixth line would break the five-line snapshot
grammar inherited from protocol v4. `byte_budget`
and `cmd_budget` remain software-ring parser budgets, and `sync=1` means the
current damage generation is synchronized and not discarding through EOL.

Every USART3 stream-damage event advances the damage generation, flushes queued
commands and published-but-undrained DMA backlog, invalidates a partial line,
and discards wire bytes through the next CR/LF. Damage includes the first failed
DMA start/restart, UART error, DMA overwrite or impossible HT/TC/IDLE phase,
software-ring or line overflow, invalid character, and assembly timeout.
Because that flush can also remove a complete STOP/DISABLE/LIFT_STOP, every
damage event schedules a main-context Drive stop plus Lift communication stop.
Multiple events before main service remain visible in `stop_events` but may
coalesce behind one `stop_pending`; `stop_dispatch` counts each consumed
pending request after the Drive-plus-Lift stop dispatch. The pending stop is
always dispatched before aborting or restarting DMA. A UART-error callback and
the failed restart belonging to the same outage do not advance damage twice.

USART3 DMA restart failures form explicit outage episodes. The first failure
changes `HEALTHY` to `OUTAGE_PENDING` and records the outage-start tick.
`JetsonProtocol_ServiceRx()` dispatches immediate Drive/Lift stop first,
refreshes `now`, evaluates the episode age before each restart, and only then
makes one bounded abort/start attempt. Once age is greater than 100 ms it
records one `rearm_fault` diagnostic event and invokes the recoverable Drive
zero/IDLE stop plus Lift communication stop, even if that iteration's restart
succeeds. A stable DMA start returns the RX link to `HEALTHY` and permits a
future independent outage episode. It never revives an earlier command, and
the UART outage does not create a persistent Drive fault or inhibit latch.

The no-motion hardware RX smoke command is:

```text
UART_RX_DEBUG req=13579BDF
```

Receiving its five nonce-bound response lines proves the command traversed the
USART3 DMA path without issuing ENABLE, a nonzero velocity, or a Lift action.
Require `armed=1`, `rearm_pending=0`, `link_state=HEALTHY`, and `sync=1` before
continuing with other read-only checks.

UART_TX_DEBUG is read-only and emits six bounded complete lines containing
active age/length/wire time/generation, queue depth/high-water,
enqueue/transmit/drop counters, HAL busy/error counts, UART error/code,
timeout/recovery counts, format-overflow, completion, and resynchronization
counts. It does not reset or continuously publish the counters. All six
lines of one response share a `snap=` id and echo the query's `req=`;
`DRIVE_YAW_DEBUG` (15 lines) and `BAT_DEBUG` (3 lines, whose first line
reports the Modbus poll/response counters as `poll=`/`resp=`) follow the
same snapshot rule. Hosts assemble a snapshot only from lines with their
own `req=` and one consistent `snap=`, each line number exactly once.

`DRIVE_CAN_TRACE_STATUS req=<nonce>` and
`DRIVE_CAN_TRACE_DUMP req=<nonce>` are read-only flight-recorder queries.
The recorder keeps 128 pre-trigger events in a rolling RAM history, captures
for 500 ms after the first qualifying CAN/SDO fault, then freezes that first
snapshot. Sixteen post-trigger slots are reserved for critical events; if
necessary a later critical event evicts a post-trigger normal event rather
than being lost behind normal traffic. `dn` and `dc` report dropped normal and
critical events separately. A frozen snapshot is automatically exported once
at DEBUG priority and may be exported again with `DRIVE_CAN_TRACE_DUMP`.
Export is incremental:
at most one trace line is queued per cooperative-loop iteration. Every line
carries a snapshot id (`snap=` on envelope/status lines and `sn=` on compact
item lines); manual BEGIN/END lines echo the request nonce and the number of
`i=` item lines between them equals `total=`. `DRIVE_CAN_TRACE_STATUS`
emits 13 nonce-bound lines: recorder state plus duration/gap statistics for
MAIN, RX_SERVICE, HEARTBEAT, DRIVE, LIFT, and SDO_WAIT. Compact item fields
are `e` event, `o` owner, `n` node, `id` CAN identifier, `d` DLC, `h` HAL
result, `q` TX FIFO free before/after, `x` HAL error, `c` LEC/TEC/REC/flags,
`fdv` controller-snapshot validity, `s` Drive/Lift state, `z`
index/subindex/write, `r` internal reason or runtime channel, `m` elapsed ms,
and `v` abort/detail code. A successful SDO contributes only `TX` and
`SDO_DONE`; failure-specific events remain explicit. Normal TPDO RX is
aggregated per node in 100 ms buckets, with `t` as the latest timestamp and
`v` as the bucket frame count. Invalid DLC, interarrival gaps over 100 ms,
FRESH-to-STALE transitions, and STALE-to-FRESH recovery are emitted as
separate events with the node identity and gap/age in `v`; these transitions
are never hidden by aggregation. For TPDO aggregate events, `m` is the bucket
start timestamp rather than an SDO elapsed time. Trace lines never change a
control or fault state.

`DRIVE_FAULT_DEBUG` takes `req=<nonce>` and is read-only. It always emits
In `BENCH_DEBUG`, the bounded `DRIVE_FAULT_CURRENT` line is emitted first.
That line comes directly from
current Drive state, transition, hardware-confirmation, latch, and
zero-evidence fields. The following four lines are a separate historical
root-fault snapshot; when no event exists they contain `event=0`. All eight
response lines (`DRIVE_FAULT_CURRENT`, `DRIVE_FAULT_EVENT`,
`DRIVE_FAULT_STATUS`, `DRIVE_FAULT_EMCY`, `DRIVE_FAULT_HB`,
`DRIVE_FAULT_RESET_DIAG`, `DRIVE_FAULT_RESET_LEFT`,
`DRIVE_FAULT_RESET_RIGHT`) carry the same `snap=` id and echo the query's
`req=`; the host collector rejects a wrong nonce, mixed snapshot ids,
duplicated tags, an incomplete tag set, and legacy lines without `snap=`,
so lines from two different print cycles can never be stitched into one
snapshot. Historical `Drive disabled` or fault messages are never
substitutes for current hardware-safe evidence.

Protocol v4 introduced the request-nonce model retained unchanged by protocol
v5. The host cannot flush frames still queued inside the firmware TX manager,
so clearing its own RX buffer never proves freshness. Every stateful command
(`STOP`, `ENABLE`, `DISABLE`,
`DRIVE_FAULT_RESET_SAFE`, `DRIVE_BASE_SAFE_REFRESH`, `LIFT_FAULT_RESET`,
`LIFT_HOME`) and every structured query (`DRIVE_BASE_SAFE`,
`DRIVE_YAW_DEBUG`, `UART_TX_DEBUG`, `BAT_DEBUG`, `DRIVE_FAULT_DEBUG`,
`DRIVE_CAN_TRACE_STATUS`, `DRIVE_CAN_TRACE_DUMP`)
carries a host-generated `req=<8 hex>` nonce; the firmware echoes it on
every response line, and the host accepts only responses echoing its own
nonce. Multi-line snapshot responses additionally share one `snap=` id per
response: a parser must reject mismatched `req=`, mismatched `snap=`,
duplicated line numbers, and cross-snapshot stitching, and only use a
snapshot after every declared line arrived. Protocol-v5 hosts must refuse
protocol-4 firmware because the ODOM wheel-field unit is part of the versioned
contract. A command without `req=` is answered with `req=00000000`, which a
v5 host never binds to.

Strict nonce grammar. A valid request nonce is ` req=` followed by exactly
eight hexadecimal digits at the very end of the command; `00000000` is
reserved for "no nonce" and is invalid as a host nonce. One shared firmware
parser classifies every line as valid / absent / malformed; too few or too
many digits, a zero nonce, trailing characters after the digits, or a
duplicated `req=` token are malformed.

Strict full-line shape. Every no-argument nonce-bound command - the
transaction starters `ENABLE`, `DRIVE_FAULT_RESET_SAFE`,
`DRIVE_BASE_SAFE_REFRESH`, `LIFT_FAULT_RESET` (aliases `LIFT_RST`,
`FAULT_RST`), `LIFT_HOME` (aliases `LIFT_RECOVER`, `LIFT_RECOVER_TOP`) AND
the structured queries `DRIVE_FAULT_DEBUG`, `DRIVE_BASE_SAFE`,
`DRIVE_YAW_DEBUG`, `UART_TX_DEBUG`, `BAT_DEBUG`,
`DRIVE_CAN_TRACE_STATUS`, `DRIVE_CAN_TRACE_DUMP` - accepts EXACTLY

    <COMMAND> req=XXXXXXXX

with one or more plain spaces between the command token and the nonce
tail, no extra tokens, and no prefix or suffix garbage. Rejections happen
BEFORE any state change, SDO request, Lift action, or queue side effect:
an absent or malformed nonce is answered with
`CMD_ACK <command> seq=<n> accepted=0 reason=BAD_REQ req=00000000`; a
valid nonce with any other shape violation (for example
`ENABLE GARBAGE req=12345678`) is answered with
`CMD_ACK <command> seq=<n> accepted=0 reason=BAD_COMMAND req=<nonce>`
echoing the valid nonce so the host can bind the rejection.
Safety exception: ONLY `STOP`, `DISABLE`, and `LIFT_STOP` are still
executed even with a missing or malformed nonce (their ACK then reports
`reason=BAD_REQ` with `req=00000000`), because an emergency stop must never
be refused over grammar; hosts must still send valid nonces to bind their
acknowledgements. `CMD_VEL` is NOT in this set: it is nonce-optional but
argument-strict (see below), and a malformed `CMD_VEL` - even
`CMD_VEL 0 0 <garbage>` - is rejected outright, because stopping is STOP's
job and a garbled line must never command the base.

Strict argument grammar. Every command that takes arguments is parsed with
a single cursor parser that must consume the ENTIRE line - one or more
plain spaces between tokens, no extra tokens, no trailing garbage:

    CMD_VEL <int32> <int32>
    RAW <int32> <int32> <int32>        (also gated by raw_test; see below)
    LIFT_UP | LIFT_DOWN | LIFT_JOG_UP | LIFT_JOG_UP_SLOW |
    LIFT_JOG_DN | LIFT_JOG_DOWN_SLOW   (no arguments; whole line must match)
    YAW_PI ON | OFF | STATUS

Integer fields accept an optional leading `-`, one or more decimal digits,
and NOTHING else (no `+`, no `.`, no exponent, no hex, no NaN/Inf text),
and are range-checked against the full int32 range. Any violation -
`CMD_VEL 100 500 EXTRA`, `CMD_VEL 100 500 req=BAD`, `RAW 1 2 3 EXTRA`,
`LIFT_UP WRONG`, `YAW_PI ON EXTRA`, a float or out-of-range number - is
rejected with `CMD_ACK <command> ... accepted=0 reason=BAD_COMMAND` BEFORE
any Drive command update, Lift pending action, PI enable-state change, SDO
request, or target-queue change. Only `STOP`, `DISABLE`, and `LIFT_STOP`
still act on a garbled line (they only reduce danger).

USART3 RX framing is fail-closed. The receive path degrades damaged input only
to "no command", never into a different valid command. The DMA destination is a
static, 32-byte-aligned, 4096-byte power-of-two buffer in ordinary `.bss`.
The current FLASH linker places `.bss` in DMA1-accessible AXI SRAM `RAM_D1`
at `0x24000000`, and this firmware does not enable D-cache. Enabling D-cache or
moving `.bss` requires an explicit non-cacheable MPU region or correct
cache-line invalidate policy before DMA RX remains valid.

Each HT, TC, or IDLE callback treats HAL `Size` as a circular-buffer position,
not a delta. The ISR performs only O(1) bookkeeping: it combines the TC epoch
and normalized position as
`absolute_producer = TC_epoch * 4096 + position`, checks the expected HT phase,
records a conservative event timestamp, and returns. It does not parse, copy,
format, call Drive/Lift, abort RX, restart RX, or loop. Main context drains no
more than 1024 DMA bytes per service call, in copy/revalidate chunks of at most
256 bytes. A chunk is committed only after session epoch, damage generation,
producer, consumer, and overwrite bounds are rechecked.

When backlog first becomes nonempty, its arrival timestamp is conservatively
the older of the last reliable empty observation and
`event_tick - wire_duration(delta)`. Bytes drained from that backlog retain
this arrival bound; a delayed IDLE/HT/TC callback can therefore only make a
line older, never artificially fresh. Every software-ring entry carries that
arrival tick and the damage generation. The poller checks generation throughout
drain and immediately before dispatch, rejects complete lines older than
`JETSON_RX_ASSEMBLY_TIMEOUT_MS` (300 ms, including stale STOP), and uses
wrap-safe unsigned subtraction.

Any DMA overwrite, impossible producer phase, software-ring overflow,
command-line overflow (`> UART_COMMAND_LINE_MAX`), USART3 ORE/FE/NE/HAL UART
error, illegal control/binary byte (outside `0x20..0x7E`), assembly timeout, or
first failed DMA start/restart enters the shared damage path. Only a CR/LF
received on the wire after that damage may re-synchronize; the delimiter is
consumed rather than queued. RX work remains bounded by the 1024-byte DMA drain
budget and the existing software-ring byte/command budgets. Restart is attempted
once per main-loop service opportunity after the immediate safety stop and
persistent-outage deadline check; callbacks never restart it.
`UART_RX_DEBUG req=<nonce>` reports the stable five-line view sharing one
`snap=` and `req=`.

RAW is a disabled developer backdoor. `RAW` runs a blocking test move that
pauses the Jetson poll, PI, tracking supervisor, wheel-feedback safety,
watchdog, IMU, and battery updates, so it is compiled out of every standard
build: `ENABLE_DANGEROUS_RAW_TEST` defaults to 0 in all profiles and the
RELEASE_TEST profile enforces 0 at compile time. When disabled, a
well-formed `RAW` is answered `CMD_ACK RAW ... accepted=0
reason=RAW_DISABLED` with no motor command and no blocking loop. `FW_VERSION`
reports the compiled state as `raw_test=<0|1>`. `BUILD_FIRMWARE.bat` forces
`raw_test=0` for both supported normal profiles, verifies that the blocking
function/callback/symbol/string are absent, and records `raw_test=0` in
`Debug/BUILD_INFO.txt` and `output/jetson_tools_ready/BUILD_INFO.txt`. Hosts
must refuse to drive hardware with `raw_test=1` firmware.

`LIFT_FAULT_RESET` (aliases `LIFT_RST`, `FAULT_RST`) and `LIFT_HOME`
(aliases `LIFT_RECOVER`, `LIFT_RECOVER_TOP`) are transactions.
`LIFT_FAULT_RESET` and its aliases perform ONLY the node-3 fault reset and
never start motion; `LIFT_RECOVER`/`LIFT_RECOVER_TOP` are `LIFT_HOME`
aliases and START the sensor homing/recovery motion sequence - they must
never be documented or used as reset-only commands. `STARTED` assigns a per-command monotonic
`txn=` bound to the command's `req=`; every subsequent
`PENDING`/`COMPLETE`/`FAILED` line of that attempt carries the same
`txn=`/`req=`. A command received while another lift recovery or homing
transaction is in flight is answered with
`state=FAILED reason=BUSY active_txn=<in-flight> txn=<new> req=<new nonce>`
bound to the NEW command, and does not disturb the in-flight transaction.
`LIFT_STOP` aborts a bound transaction, which then terminates with
`FAILED reason=ABORTED_BY_STOP`. A host must accept only terminal lines
whose `req=` and `txn=` match its own STARTED line; `LIFT_HOME` completion
additionally requires `homed=1` on the bound COMPLETE line. The legacy
free-text lines (`LIFT_FAULT_RESET complete`, `LIFT sensor homing
complete`, ...) remain as human diagnostics only and must never satisfy a
host gate - they carry no `req=`/`txn=` and may be stale TX-queue residue.
The same rule applies to `DRIVE_FAULT_RESET_SAFE` re-issues (bound
`reason=BUSY active_gen=`) and `DRIVE_BASE_SAFE_REFRESH` re-issues (bound
`reason=BUSY active_txn=`): the reply binds to the NEW nonce, never to the
in-flight transaction's lines.

`STOP`, `ENABLE`, and `DISABLE` are acknowledged synchronously with a
`CMD_ACK` line carrying a firmware-monotonic `seq=`, the accept/reject
result, and the echoed `req=`. Every rejection reports its REAL cause in
`reason=` (`TRANSITION_BUSY`, `LIFT_ACTIVE`, `NOT_CONFIRMED_DISABLED`,
`INTERNAL_ERROR`, ...); `accepted=0 reason=OK` is forbidden by this
contract. `ENABLE` and `DISABLE` forward their real dispatch result:
`DISABLE accepted=0 reason=TRANSITION_BUSY` means nothing was dispatched
(for example a conflicting ENABLE or fault-reset transaction is in flight)
and the host must not assume the drive will disable. `STOP` is always
accepted because its failure path escalates internally to a fail-closed
rollback. A
host executes these commands as transactions: send one command with a fresh
nonce, accept only the `CMD_ACK` echoing that nonce, and treat only ODOM
state lines received after that acknowledgement as the command's outcome.
The acknowledgement alone is never proof of the state change; the post-ack
ODOM state is.

With `DRIVE_BASIC_BRINGUP_PROFILE=1`, Drive uses three normal-motion gates:
state is `ENABLED_IDLE` or `RUNNING`, no hard Drive fault is active, and Lift
is not moving. Historical tracking or motion-inhibit latches, Base Safe,
zero-speed evidence, statusword freshness, and diagnostic
transaction/snapshot identifiers are not normal `CMD_VEL` gates. Base Safe is Lift-only.

`ENABLE` is explicit. From `DISABLED`, acceptance publishes `ENABLING`; the
successful commit requires fresh final `0x6041` reads from both wheel nodes
with fault clear and operation-enabled set, after the verified DS402 and Node
Guard sequence, and only then publishes `ENABLED_IDLE`. Repeating `ENABLE`
while already healthy is an accepted idempotent no-op with no CANopen writes,
state change, or latch clearing. It never performs an automatic fault reset.
Normal nonzero `CMD_VEL` is a streaming command without `req=`.

`DRIVE_FAULT_RESET_SAFE` is a reset-only, nonblocking Node 1 then Node 2
CiA-402 recovery transaction. Every response line carries the transaction's
`gen=`; hosts must bind PENDING/COMPLETE/FAILED to their own command's
STARTED `gen=`. Admission requires a Drive actually in `FAULT` or a fresh
cached `0x6041` fault bit, inactive Drive/Lift transitions, zero submitted
targets, and no pending target write. A fresh nonzero feedback pair rejects
reset; stale diagnostic feedback alone does not authorize or block it. The
transaction then reads fresh `0x6041` from each node and may reject with
`OPERATION_ENABLED`. A faulted node is diagnosed through
`0x603F` and `0x1001`, then receives only `0x0086`, a nonblocking wait, and
`0x0006` before `0x6041` is verified fault-clear and operation-disabled. The
command never writes `0x0007`, `0x000F`, or `0x002F`, never submits a wheel
target, never clears software latches, never enables Drive, and never starts
Lift. Completion establishes the reset result only; the host then issues an
explicit `DISABLE` and waits for fresh `DISABLED` evidence. Reset-only failure
diagnostics do not replace the historical `DRIVE_FAULT_EVENT`.

During a fault-reset or safety transition, exactly one new EMCY event whose
error code is `0x0000` is treated as that node's CiA-301 "error reset"
notification and does not fail the transaction; the tracked EMCY sequence
advances instead. Any new EMCY with a nonzero code fails the transaction
with `NODE_FAULT`, and a sequence jump of two or more events fails closed
because the intermediate codes cannot be proven benign.

`DRIVE_BASE_SAFE` is read-only and emits three bounded complete frames.
`safe=` is calculated directly by the same firmware predicate used by
`Drive_RequestConfirmedStopForLift()`; the diagnostic does not clear a fault or
latch, change a generation/target, start a transition, or initiate an SDO. The
following statusword and zero-evidence frames expose the facts behind that
decision. Consumers must reject a missing/malformed group and use only the
newest complete group. This snapshot reports current cached evidence only; it
does not acquire new `0x606C` or `0x6041` hardware evidence. It remains useful
for read-only diagnostics, but it is not an authoritative host gate after a
refresh transaction.

`DRIVE_BASE_SAFE_REFRESH` is the explicit nonblocking evidence refresh.
Every command outcome is bound to that command by a per-command monotonic
`txn=` plus the echoed `req=`; `gen=` remains the safety generation of the
evidence, which two different transactions may legitimately share. Every
successful path emits `STARTED` first — including the immediate path where
the Lift hardware-safe predicate is already true and `COMPLETE` follows
without an SDO. Reject paths (`FAILED reason=BUSY` while another refresh or
lift confirmed-stop transaction is in flight, `FAILED reason=PRECONDITION`)
also carry this command's `txn=`/`req=` so the host can bind them. A
re-issued command while active no longer reports the active transaction's
phase; it is answered with its own bound `BUSY` failure and does not
restart or cancel the in-flight transaction. Otherwise, after all
non-status preconditions pass, the refresh reuses the Lift confirmed-stop
  transaction owner. When zero evidence predates the latest completed DISABLE,
  `SETTLING`/`CONFIRMING` first require three coherent bilateral feedback pairs
  newer than that post-disable baseline and spanning at least 100 ms. The
  refresh then reads `0x6041:00` as U16 from Node 1 and then Node 2.
Each successful sample is sequence/timestamp bound to the unchanged current
safety generation. Communication completion is distinct from admission:
unsafe status bits report `COMPLETE safe=0`, while timeout/abort/protocol
failure reports `FAILED`. This command never enables a wheel, submits a
target, starts Lift, changes Drive/Lift state, or clears a safety latch.
The nonce/txn-bound `COMPLETE safe=1` line is the authoritative host proof for
Lift admission and Lift recovery only. Normal Drive reset, Drive startup,
Drive `ENABLE`, `CMD_VEL`, and the basic post-soak Drive preflight do not call
Base Safe. Lift hosts must fail closed on `COMPLETE safe=0` or any
`FAILED`/timeout outcome and must not perform a second passive
`DRIVE_BASE_SAFE` query as an authorization gate.

IMU remains sampled and published every 10 ms (100 Hz). The TX centralization
is a software transport fix and is not physical validation of yaw PI signs,
gains, tracking thresholds, traction, or actuator response.

In `DRIVE_BASIC_BRINGUP_PROFILE=1`, `YAW_PI ON` is rejected, the STM32 outer
yaw loop remains disabled, and the Kinco internal velocity loop continues to
execute open-loop differential wheel targets. Tracking remains observational
and cannot stop or fault Drive. Outside that profile, `YAW_PI ON` and
`YAW_PI OFF` are accepted only with zero requested and target
body velocity, no active nonzero Jetson command, no drive safety transition,
no lift interlock transition, no nonzero submitted wheel target, and no
`RUNNING` drive state. A real state change resets the PI/tracking session and
waits for a new coherent wheel pair before priming; it does not enable motors
or clear an existing stop/fault latch. Repeating the current state while idle
is idempotent. Invalid syntax returns `Usage: YAW_PI ON|OFF|STATUS`.

`LIFT_RAW` emits two bounded complete lines and is read-only. During
`WAIT_BASE_STOP`, its node-3 fields are reported as `-` instead of issuing
blocking diagnostic SDOs; line format and field order are unchanged.

`DRIVE_YAW_DEBUG` emits fifteen complete bounded lines, `YAW_CTRL 1/15` through
`YAW_CTRL 15/15`. It is read-only and additive. `P_term`, `I_state`, and
`I_term` mean respectively mrad/s, mrad, and mrad/s;
raw/requested/proposed/submitted corrections are mrad/s. The tracking lines
expose their independent consumed feedback sequence and sequence-valid flag.

The existing final `ODOM` field remains numeric and keeps the same position.
In the basic profile, `fault=1` means a hard Drive fault. Stale feedback,
tracking diagnostics, historical motion-inhibit state, and a normal safety
transaction do not masquerade as a hard fault.

In protocol v5, `ODOM` is the single current wheel-odometry telemetry format.
Its signed
`left_mrpm` and `right_mrpm` fields use milli-RPM: one wire unit is 0.001 RPM.
They are computed directly from the authoritative logical motor RPM values as
`logical_motor_rpm * 1000.0f`, with one final signed integer rounding step and
no whole-RPM intermediate. The conversion saturates at the signed `int32`
limits before serialization.

`ODOM` is emitted only for a current valid authoritative feedback pair. Under
TPDO Active authority a stale or invalid pair is reported by the versioned
`WHEEL_FEEDBACK_HEALTH` lines and suppresses `ODOM`; firmware does not republish
cached wheel velocity as fresh or silently substitute zero. RPM zero alone
still never proves that the AMR has stopped.

## CANopen / FDCAN1

- FDCAN1 uses Classic CAN, standard IDs, and the existing 500 kbit/s nominal
  configuration.
- SDO request COB-ID: `0x600 + node_id`.
- SDO response COB-ID: `0x580 + node_id`.

Shared CiA402 objects:

| Object | Direction | Meaning |
| --- | --- | --- |
| `0x6040:00` | write | controlword |
| `0x6041:00` | read | statusword |
| `0x6060:00` | write | requested mode |
| `0x6061:00` | read | displayed mode |
| `0x603F:00` | read | lift error code |

Wheel runtime objects:

| Object | Direction | Meaning |
| --- | --- | --- |
| `0x606C:00` | read | wheel-drive actual velocity in the existing Kinco DEC unit |
| `0x6063:00` | read | wheel-drive actual position used by odometry |
| `0x60FF:00` | write | wheel-drive target velocity in the existing Kinco DEC unit |

Safe boot initialization may use nonblocking expedited SDO actual-velocity
feedback. After the initial hardware-disable transition, firmware automatically
configures the verified feedback-only TPDO1 profile (Node 1 `0x181`, Node 2
`0x182`, map `0x606C0020`, DLC 4, type `0xFE`, inhibit 0, event 20 ms), reads it
back, NMT-starts each wheel node, and waits for fresh streams. Configuration is
bounded and never persisted. Before TPDO configuration, startup reads both
wheel identities and configures/readbacks RPDO2 `0x301/0x302` with one mapped
`0x60FF0020` Integer32, DLC 4 and type 254. Standard `0x1401/0x1601` support
is checked on the responding device. Normal targets use RPDO, with actuator
commit only after both local TX completions. Configuration/safety use SDO;
there is no runtime SDO target fallback. Any config/readback error, node boot-up
or loss of qualification inhibits ENABLE and normal motion.

Normal startup automatically enters `TPDO_ACTIVE_ARMING` only after software
and hardware disable, zero target, stopped Lift, no transition/fault/E-stop,
both configurations, node freshness/age, and skew gates pass. It waits at most
1000 ms for both node sequences to advance after arm start. The first fresh
pair within 30 ms skew becomes pair sequence 1 and atomically selects
`mode=TPDO_ACTIVE authority=TPDO`. Configuration, freshness, or arming failure
latches `TPDO_FAULT`, keeps runtime SDO `0x606C` polling off, and blocks ENABLE;
there is no automatic SDO fallback. `DRIVE_TPDO_ACTIVE_ENABLE` remains as the
same strict nonce-bound debug/manual arming transaction but is not required on
a normal boot.

Active runtime commits a pair only after both node sequences advance, even if
the raw numbers are unchanged. The centralized authoritative pair supplies
ODOM, STOP/Base Safe, PI/tracking, safety, and diagnostics. Every scheduled
runtime SDO `0x606C` request (normal, STOP, and fault monitor) is disabled, but
manual/configuration/safety SDO, `0x60FF`, Heartbeat, and EMCY remain. An SDO
velocity response is diagnostic-only and cannot overwrite the TPDO pair.

Stale Active feedback never resumes SDO authority. It invalidates the pair,
latches `TPDO_ACTIVE_FEEDBACK_STALE`, suppresses ODOM, and requests the existing
one-shot Emergency Zero and bounded safety/fault path. Explicit safely-disabled
`DRIVE_TPDO_ACTIVE_DISABLE` is a debug/manual transition back to
`TPDO_SHADOW authority=SDO`; it emits matching `STARTED` and `COMPLETE`, but
Drive cannot enable until TPDO authority is re-established.

`DRIVE_TPDO_ACTIVE_STATUS` is an exact nonce-bound eight-line snapshot: 1/8
mode/feedback and ODOM authority/configuration, 2/8 Drive state and hardware
confirmation, 3/8 target/Lift/transition/fault/E-stop gates, 4/8 and 5/8 node
state/raw/normalized/sequence/age/max-gap, 6/8 pair validity/state/sequence/
age/skew, 7/8 runtime `0x606C` poll enable/count and last fault, and 8/8 STOP
rejection and ODOM-suppression diagnostics. Hosts reject malformed, mixed,
duplicate, incomplete, or wrong-nonce snapshots.

Normal `CMD_VEL 0 0`, command-watchdog expiry, protocol `STOP`, Lift motion,
and ordinary UART damage immediately submit bilateral zero and publish
`ENABLED_IDLE` when Drive remains enabled. These are recoverable soft stops:
they do not create a tracking latch, generic motion-inhibit latch, or Drive
fault. A failed zero submission/transaction is instead handled by the bounded
hard-fault cleanup. Bilateral target acknowledgement and fresh measured-zero
evidence remain available only for Lift admission and explicit disable/base
safe qualification.

During initialization or explicit debug Shadow mode under SDO authority, the
existing bounded 100 ms hard-`FAULT` monitor
continues requesting only each wheel's `0x606C`; it never requests `0x6063`.
Under TPDO authority the monitor stays off so a feedback fault cannot become a
silent transport fallback. STOP, rollback, ENABLE, or DISABLE confirmation
retains its existing safety ownership and preemption behavior.

After both wheel statuswords have confirmed hardware disable, the actuator
command baseline and pre-disable zero proof are retired. Firmware captures the
current bilateral feedback sequences as a post-disable epoch; unchanged cached
feedback cannot recreate evidence. Lift admission requires three coherent zero
pairs newer than that baseline over at least 100 ms, followed by the bounded
left/right statusword confirmation.

`DISABLE` acknowledgement means the transaction was accepted, not completed.
Acceptance immediately publishes `DISABLING`, clears motion, obtains the
private stop proof, checks each node's
`0x6040=0x0006` transaction separately, and bounded-polls both `0x6041`
statuswords to verify that Operation Enabled is clear. A valid first read that
still shows Operation Enabled and a single read timeout are
in-progress/retryable outcomes within the configured attempt and overall
deadlines. An explicit SDO abort, EMCY/status fault, continuity loss, or
expired overall deadline is a failure. Only after the full chain sets
hardware-disable confirmation true does it publish `DISABLED`; failure reports
`FAULT` and never emits the successful `Drive disabled` event. Hosts must see
a fresh post-ack `DISABLING` followed by a fresh `DISABLED`; an old buffered
`DISABLED` line is not completion evidence.

At boot, `INIT` uses the same asynchronous disable write/read machinery but
does not submit a wheel target: it writes `0x6040=0x0006` to both wheel nodes,
publishes `DISABLING`, verifies both `0x6041` Operation Enabled bits are clear,
sets hardware-disable confirmation true, then publishes `DISABLED`. `STOP` or
`DISABLE` received during INIT starts or continues that verification instead
of relying on nonexistent prior-state evidence.

Lift base-stop admission has one additional recovery-safe case. It may return
`OK` while Drive state and historical latches remain `FAULT` only when there is
no active transition, hardware disable is confirmed for both wheels, hardware
enable and software drive-enable are false, both cached statuswords are valid,
from the current safety generation, no older than 150 ms, and have Operation
Enabled/Fault clear, the last wheel targets are zero, no target
submission is pending, and current-generation measured-zero evidence is fresh.
This path does not change Drive state, clear latches, or enable a wheel.
If either cached statusword is stale or from another generation, Lift admission
remains false. A Lift confirmed-stop request uses the existing nonblocking SDO
arbiter to reread `0x6041` from Node 1 and Node 2 and binds successful reads to
the current generation; timeout/abort fails closed. The 150 ms threshold reuses
`DRIVE_WHEEL_FEEDBACK_STALE_MS`, so disabled status and measured-zero evidence
cannot have different freshness budgets. Unsigned tick subtraction keeps the
test wrap-safe.

`ENABLE` starts only from `DISABLED` and is an all-or-nothing two-node
operation. Admission requires Lift stopped, both nodes reachable, and no fresh
Kinco fault. It does not require Base Safe,
hardware-disable history, zero-speed evidence, status snapshot history,
tracking state, or generic motion-inhibit history. Software publishes
`ENABLING` when the transaction starts and publishes `ENABLED_IDLE` only after
the verified DS402 sequence and final fresh
bilateral `0x6041` values confirm Fault clear and Operation Enabled set.
Any failure requests bilateral zero, attempts bilateral `0x6040=0x0006`
rollback, and publishes `FAULT`; it never publishes a half-enabled software
state.

When already healthy, `ENABLE` is accepted as an idempotent no-op in `RUNNING`
and `ENABLED_IDLE`. It does not alter the command/target, clear any latch, send
SDOs, or start a new safety transition. `ENABLE` while `FAULT` remains
rejected and requires explicit fault reset. Both the public command entry and
internal transition entry reject a new ENABLE transaction while Lift is
moving.

This bring-up profile has no MCU emergency-stop control gate: PD15 is sampled
for independent telemetry, with no Drive callback, control-context field,
admission check or build-readiness gate for the physical input. The legacy
control-policy metadata remains `software_emergency_input=NOT_USED_BY_DESIGN`;
`estop_telemetry=PD15_ACTIVE_HIGH_V1` records the added telemetry. The separately named
`Canopen_EmergencyZeroWheelTargets` mechanism remains the bilateral zero
cleanup used by hard-fault/disable rollback and is not an input feature.
Kinco communication-loss protection uses Heartbeat: producer time `0x1017`
and consumer entry `0x1016:01` are configured and verified, enable completion
requires both nodes qualified, and runtime loss is a hard fault requiring
explicit recovery.

Every motion-capable lift command first enters an internal single-slot
`WAIT_BASE_STOP` phase. No node-3 motion/preparation request is sent until the
drive requester reports current-generation confirmed stop. A second lift motion
command is rejected rather than replacing the pending command; `LIFT_STOP`
cancels it. These are internal semantics only—the command strings and `LIFT`
status-line format are unchanged.

Lift homing sequence:

```text
read 0x603F and 0x6041
clear only a real statusword fault
0x6040 = 0x0006, verify Ready to Switch On
0x6099:01 = normal 0x6081 configured value (Uint32)
0x6099:02 = normal 0x6081 configured value (Uint32)
0x609A:00 = normal 0x6083 configured value (Uint32)
0x6060 = 6
poll 0x6061 until 6
0x6040 = 0x0007, verify Switched On
0x6040 = 0x000F, verify Operation Enabled
0x6040 = 0x000F, confirm bit 4 low
0x6040 = 0x001F, confirm bit 4 high
poll fresh homing evidence
```

Normal profile-position objects:

| Object | Meaning |
| --- | --- |
| `0x6060:00 = 1` | profile position mode |
| `0x607A:00` | target position in pulses |
| `0x6081:00` | profile velocity in pulses/s |
| `0x6083:00` | acceleration in pulses/s^2 |
| `0x6084:00` | deceleration in pulses/s^2 |
| `0x6064:00` | actual position in pulses |

Before every fresh sensor-homing start, node 3 is configured with expedited
four-byte SDO writes. Both `0x6099` speed subindices receive
`LIFT_PROFILE_VELOCITY_PUL_S` (currently `7200000`), and `0x609A:00` receives
`LIFT_PROFILE_ACCEL_PUL_S2` (currently `18000000`). This normal/homing coupling
is intentional for the current experiment; there are no duplicated homing-only
tuning constants. Kinco FD documentation gives `0x6081` and
`0x6099` the same drive speed encoding, and `0x6083` and `0x609A` the same
drive acceleration encoding, so raw-value reuse makes homing match normal lift
motion without assuming an unverified node-3 encoder resolution. Any write
timeout, transport error, or SDO abort prevents the homing mode/start edge.

## Wheel RPDO diagnostic extension

`DRIVE_RPDO_DEBUG req=<nonce>` is read-only and returns `DRIVE_RPDO 1/9`
through `9/9`, sharing the request nonce and one snapshot ID. The matching
Jetson parser requires all nine parts; stale, mixed or incomplete replies do
not satisfy a gate. Parts contain:

1. Control count/period/last and maximum gap, queued/sent pair counts, state,
   current RPDO readiness and MCU timestamp.
2. Observed completion interval/latency, maximum latency, last/maximum actual
   TX event timestamp skew (nominal CAN bit ticks), hardware pending mask.
3. BUSY, failure, cancellation, bus-off, cancellation-timeout counters/reason.
4. Node 1 qualification, Operational state, boot epoch, `identity=NOT_QUERIED`,
   configuration `phase`, `expected` and `actual` values (8-digit hexadecimal).
5. The same fields for Node 2.
6. Node 1 cached startup COB-ID/type/count/map readbacks, last step/object/abort/result.
7. The same fields for Node 2.
8. Existing Drive runtime last/max/gap/over-20-ms count and maximum observed pair interval.
9. Startup state (`WAIT_SAFE`, `WAIT_FRESH`, `ARMING`, `COMPLETE`, `FAILED`),
   current startup blocking reason, tracking/inhibit flags, Drive state,
   hardware-disable confirmation and BASIC profile flag. Present in RELEASE_TEST.

In BASIC, historical tracking/inhibit flags do not block PDO configuration or
TPDO arming after a verified disabled recovery. Both gates still require the
current disabled-safety predicates; neither clears history or enables motors.
Non-BASIC retains both latch guards. `blocked=NONE` means this startup stage
has no blocking reason; it does not authorize wheel motion or Lift. `ARMING`
reports `WAIT_POST_ARM_PAIR`; failed startup reports its retained fault reason.
Use the newly matching parser: older eight-part snapshots are incomplete.

Normal startup/recovery never queries `0x1018:01..04`. Its absence is not a
configuration failure. Numeric vendor/product/revision/serial fields are not
required on the wire; the matching Jetson parser saves missing numbers as
`UNKNOWN`, not zero or verified identities. Startup also emits `WHEEL_IDENTITY
node=<n> state=NOT_QUERIED vendor=UNKNOWN product=UNKNOWN revision=UNKNOWN
serial=UNKNOWN`. The existing firmware Build ID/ELF identity check is separate
and remains mandatory. Configuration failures retain phase, node, step,
index/subindex, SDO abort and expected/actual in `WHEEL_RPDO_CONFIG` and the
bound nine-part snapshot. Both wheel records must be shown when either fails.

These are diagnostic observations, not a per-frame drive acknowledgement.
`HAL_OK` means queued; `sent` requires both local CAN TX completions/events.
TPDO remains the actual-velocity source. STOP acknowledgement accepts the
request; confirmed stopping additionally requires the safety transaction and
fresh bilateral zero feedback. See `RPDO_40HZ_ACCEPTANCE.md` for units and gates.

## Battery Modbus registers

- Peripheral: USART2.
- Serial configuration: 9600 8N1.
- Direction: PG0 `RS485_WR1`, HIGH = TX, LOW = RX.
- Runtime request: FC03 read-data registers.
- Live block: start `0x0012`, count `0x0017`.
- Known request for slave 1: `01 03 00 12 00 17 A5 C1`.
- Expected response length: 51 bytes.
- Runtime polling is read-only and does not build or transmit Modbus write
  requests.

Register summary:

| Register | Meaning |
| ---: | --- |
| 18 | total voltage, 10 mV units |
| 19 | charge/discharge current, 10 mA signed vendor units |
| 20 | remaining capacity, 10 mAh units |
| 21..36 | cell voltages, mV |
| 37..38 | temperatures, 0.1 K offset |
| 39 | nominal capacity, 10 mAh units |
| 40 | RSOC percent in low byte |
