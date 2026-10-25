# P3 baseline code — AMR PCB safety feedback

这是固件可观测安全状态与通信链路的源码基准，已包含当前 PD15 急停状态上报。
统一 Trace 接入、Agent 反馈/无反馈实验及 HIL 尚未完成。

| 研究入口 | 文件（均相对 `baseline/amr_pcb/`） | 用途 |
| --- | --- | --- |
| 急停输入上报 | `Core/Src/estop_status.c`, `Core/Inc/estop_status.h` | GPIO 状态、去抖、变化和周期报文 |
| UART 调度与协议 | `Core/Src/jetson_tx.c`, `jetson_protocol.c` | 优先级、队列满、nonce/诊断边界 |
| 故障与停止证据 | `Core/Src/drive_safety.c`, `drive_diag.c`, `can_trace.c` | 事件来源及请求/结果区别 |
| 可执行回归 | `tests/host/test_estop_status.c`, `test_status_light.c` | 上电已按下、抖动、队列拒绝、计时回绕 |

`ESTOP v=1` 是输入状态遥测，不证明电机断电、STO、整车已停或串口实际延迟。
Jetson ROS2 base driver 解析这条新消息仍待后续接入。

## 快速运行 / Offline quick start

Requirements: Python 3.10+ and a native C11 compiler (Linux GCC/Clang or Windows LLVM-MinGW).
The retained tests use the Python standard library and fake GPIO/CAN; no serial device is opened.
Run from the repository root:

```sh
python code/verify_baseline.py
python code/run_checks.py
```

Set `AMR_HOST_CC` to a compiler executable when it is not on PATH. On Windows,
use LLVM-MinGW rather than MSVC-targeting Clang; the existing runner also accepts
`AMR_HOST_SYSROOT` for a GNU-compatible Windows sysroot. Test outputs go under
`code/baseline/amr_pcb/output/` and are ignored by Git.

## What this snapshot contains

- `baseline/amr_pcb/Core/`: current application C/headers and STM32 startup source.
- `baseline/amr_pcb/tests/host/`: the existing fake-bus regression tests.
- `baseline/amr_pcb/tools/`: the two retained offline test entry points.
- `baseline/amr_pcb/jetson_tools/`: the exact protocol/RPDO parser dependencies used by those tests.
- `baseline/amr_pcb/docs/`: protocol, startup recovery and PD15 contracts copied with the source.
- `baseline_manifest.json`: SHA-256 and byte size for every imported file, plus a whole-snapshot digest.
- `source_selection.json`: explicit paths approved for the next import.
- `VALIDATION.json`: validation status bound to a snapshot digest.

源基准为当前 `AMR_PCB _RGB`，包含 25 ms 轮速控制、开机等待/恢复逻辑和新 PD15 上报。
本地目录没有可用的独立 Git commit，因此用逐文件哈希和快照摘要锁定，不能把它标成
Bayro Fleet 的某个 Git 提交。当前源码快照也不证明它已烧到实车。

This is an application-source and host-test baseline. The STM32 HAL/CMSIS driver
trees, linker/build project and flash helpers are not bundled, so this folder alone
is not a complete target-firmware build kit. Existing vendor headers and available
license notices are preserved; see [NOTICE](NOTICE.md).

The Bayro Fleet ROS2/navigation stack, deployment maps, credentials, logs, generated
binaries and backup copies are outside this initial selection. The copied Jetson
protocol library contains serial-session helpers, but `run_checks.py` calls only
offline parser/test paths. Production firmware, fleet code and real devices are
not changed by these checks.

## Baseline boundaries

This folder is a frozen input to future experiments. Keep agent-generated recovery
or tuning changes in separate experiment/policy files; do not alter this baseline
to improve a score. The existing shared research harness remains owned by P1;
P2/P3's `dependencies.lock.json` pins are unchanged. A baseline regression suite is
not the planned SIL fault benchmark, commissioning experiment or HIL validation.

PD15 telemetry is already implemented in this snapshot: active HIGH, 30 ms release
debounce, change reports and 100 ms snapshots. It reports input state; it does not
add an MCU emergency-stop control gate. Jetson base-driver ESTOP parsing and
physical acceptance remain follow-up work. Source configuration values, including
deceleration constants, are not measured braking performance.

## Later updates

From the repository root, compare the same explicit selection to a new source directory:

```sh
python code/refresh_baseline.py --source "/path/to/AMR_PCB _RGB"
python code/refresh_baseline.py --source "/path/to/AMR_PCB _RGB" --write
python code/run_checks.py
```

The first command only reports differences. `--write` updates the research copy
and invalidates old validation status; it does not push, build production firmware
or access hardware. Review newly needed dependencies before editing the allowlist.
Record the new test/compiler results against the new digest in `VALIDATION.json`,
update this repository's baseline register, and commit the reviewed snapshot.
Keep the same digest across all three papers when they share a baseline. Existing
experiment manifests must retain the digest of the baseline they actually used.
