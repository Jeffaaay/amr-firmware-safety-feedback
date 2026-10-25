# STM32 AMR Boot Recovery V1 — 独立审查修订报告

日期：2026-10-12（America/Los_Angeles）。工程：`AMR_PCB _RGB`。

当前版本遵循标准 SDO 的简化 V1：正常初始化立即执行；启动暂时无响应时非阻塞等待 5000 ms，完整重试一次；成功只进入 DISABLED，第二次失败锁存 FAULT。移除 N+1 应答数量证明。EMCY Fault/Clear 竞争和畸形 SDO 成功判定已修复。**同节点、同对象的迟到或重放应答仍可能误匹配活动事务；该风险没有被软件消除，实机安全验收尚未完成。**

本报告描述当前 R3 源码。R2 外部复审已确认接收时间戳、typed SDO 宽度和 Host HAL 编码三项修复通过，另复现了 Resume 排空期间新到 EMCY 尚未处理就启动第二轮的边界。本轮仅修复该排空完成条件。R2 原件与外部反例见 `output/boot_recovery_v1_review_r3/before/`、`external_r2_evidence/r2_independent_evidence/README_CN.md`；R1 及更早证据分别保留于 `output/boot_recovery_v1_review_r2/`、`output/boot_recovery_v1_review_fix/`。未烧录固件。

## A. 根因与驱动资料

启动路径对 Node 1、Node 2 写 `0x6040:0 = 0x0006`，再分别读取 `0x6041:0` 确认禁能。原初始化通信超时可以导致锁存 Fault；现场究竟是延迟供电、SDO 时延、无 ACK、TX 积压或 BUS_OFF，仍无实测结论。

用户提供的驱动资料：Kinco iWMC05606，历史型号 `iWMC05606-00450-A150-MBDT-R`，手册同时包含 `-L`。Node 1 为左轮，Node 2 为右轮；具体安装的 L/R 铭牌映射未逐个核对。驱动真实 Firmware Version 未取得；STM32–Jetson Protocol v5 不是驱动固件版本。

资料来源为用户提供的《Kinco iWMC05606 & iWMC05710 集成式伺服轮使用手册》，2025-09-15 版，9.3.2，印刷第 97–99 页：SDO 使用 `0x600+Node_ID` / `0x580+Node_ID`，expedited SDO 的 DLC 为 8。未取得厂商关于事务 nonce、最大响应时延或重复应答上限的保证；本实现不假定这些保证存在。

`0x1018` 身份查询仍不作为启动必要条件。没有增加驱动身份白名单或将 UNKNOWN 固件版本当作启动失败。

## B. 改动范围

| 文件 | 本次修订 |
|---|---|
| `Core/Src/drive_safety.c` | 移除应答数量门槛；保持本轮双侧状态及硬故障检查；R2 沿用真实 RX 时间；R3 在 Resume 尚有待处理帧时保持等待，不增加重试数或启动第二轮 |
| `Core/Src/drive_internal.h` | 删除旧计数方案字段；R2 仅增加最近 SDO 完成结果的 RX 时间 |
| `Core/Inc/canopen.h`, `Core/Src/canopen.c` | 逐帧锁存 boot EMCY；成功严格要求 DLC 8，失败 mailbox 无效；RX 缓冲 64 字节；R2 保存 RX 时间并严格匹配 typed 宽度；R3 返回排空是否完成，FIFO 非空时保留 fence |
| `Core/Src/main.c` | CAN 启动后、其他客户端前开始 boot 硬故障观察 |
| `tests/host/` | 保留原场景及重放局限；R2 增加缓存年龄、数据宽度及容量受限 FIFO 用例，Host DLC 编码对齐随包真实 HAL |

既有 Boot 状态机、命令门控、RPDO/TPDO 授权、Lift 零速条件、运行期 Fault 策略和 `0x1018` 非必要设计保留。未修改 FDCAN 自动重传、硬件 TX 队列、全局 SDO 超时或驱动故障复位逻辑。严格 DLC 检查适用于共享 SDO 层，包括 Lift Node 3；有效的 8 字节帧行为保持。

## C. 状态机与参数

- `FIRST_ATTEMPT → READY`：正常响应直接完成双侧禁能，没有固定五秒延迟。
- `FIRST_ATTEMPT → WAIT_RETRY → SECOND_ATTEMPT → READY / FAILED`：仅启动初始化的暂时无响应、且没有已知硬故障时，允许一次重试。写 ACK 超时、写成功但状态读取超时均覆盖。
- 等待起点是第一轮失败事件；无符号 tick 差达到 5000 ms 才开始第二轮，非阻塞且支持回绕。
- 第二轮重新执行双侧 shutdown 写入及状态确认；任何第二轮失败进入锁存 Fault，不存在第三轮自动重试。
- 有效但暂未禁能的状态保留原有有界轮询；状态确有 Fault 时立即失败，不把真实故障当作晚启动。

本次不调大时间参数：SDO 单次 20 ms、控制字局部最多 3 次、普通事务 quarantine 50 ms、初始化上限 1500 ms、禁能状态轮询 20 ms / 500 ms / 最多 25 次均保持。普通运行期 STOP/DISABLE 超时处理保持。若实测表明 Kinco 时延超出，应另行依据测量调整启动参数。

WAIT 期间清除速度命令和零速证据，软件使能、硬件使能确认、硬件禁能确认均不授予运动资格。ENABLE 被拒绝；CMD_VEL 不缓存、不重放；STOP/DISABLE 不提前或推迟重试。恢复只发布 DISABLED，必须之后收到新的显式 ENABLE 并满足既有 TPDO/心跳/反馈条件，才走原使能流程。不自动 Fault Reset，不复位 MCU/FDCAN。

## D. SDO 清理与安全边界

等待入口隔离轮节点 SDO，退役旧软件事务，清除左右状态字和零速证据。Lift、Heartbeat、EMCY、PDO 继续正常分发。到点后在隔离仍有效时有界排空当前 RX FIFO；随后检查 sticky boot 硬故障、Heartbeat pending、runtime pending、反馈连续性 pending 及 BUS_OFF，确认没有已知硬故障才增加重试次数、记录第二轮 START 并发送新请求。

R3 每次 Resume 只处理该次 FIFO 快照数量，之后再读取 FIFO 待处理量。排空期间若有新帧留下，保持 wheel SDO fence 和 WAIT_RETRY，后续调度继续处理；不循环忙等、不重设五秒起点、不消耗自动重试次数。已派发的硬故障仍先于等待返回被检查。只有确认排空且没有已知故障才开始第二轮。最终检查后未来新到 CAN 帧仍属于正常调度边界，本改动不声称原子封闭任意未来帧到达。

持续到帧可以继续推迟第二轮开始；此时仍保持运动禁止。5000 ms 是最早可开始重试的等待时间，并不承诺在总线持续繁忙时精确于该毫秒启动。

非零 wheel EMCY 逐帧锁存原因、节点和 code，后续 `0x0000` reset notification 只能更新常规最后状态，不能清掉 boot 硬故障。未改变运行期 pending 的消费顺序。

SDO 响应必须匹配节点及 index/subindex。所有成功路径严格要求完整 DLC 8；typed upload 的请求宽度还须匹配响应 CS：1 字节 `0x4F`、2 字节 `0x4B`、4 字节 `0x43`。下载 ACK 要求 CS=`0x60`。错误宽度返回协议错误，不把 padding 当作对象数据。短 Abort 作为畸形响应处理，不当作完整 remote Abort。Blocking 失败同步清除 mailbox.valid。未知或扩展 DLC 不再默认为 8；接收缓冲按 HAL 最大 64 字节预留，避免在应用层校验前复制越界。

禁能发布要求两节点分别取得匹配回读，状态有效、属于当前 safety generation，软件接收年龄不超过 150 ms，并处于合法未 Operation Enabled 的 DS402 状态。合法 `0x0040/0x0060`、`0x0021`、`0x0023` 可通过；Not Ready、Operation Enabled、Fault 和非法组合不能直接通过。状态掩码参考 [Kollmorgen 官方状态编码](https://webhelp.kollmorgen.com/kas4.02/Content/3.UnderstandKAS/CANopen_Status_Word.htm)。禁能不等于实测零速，Lift 仍需独立零速证据。

R2 的 RX 时间是在 `HandleSdoResponse` 派发响应时保存的本机 tick，随完成值一起返回；它不是硬件总线时间戳，也不是调用方稍后消费缓存的时间。Drive safety 共用读取路径及 Lift base-safe refresh 沿用该时间，不能因其他 blocking SDO 客户端拖延调度而把旧缓存重新标为新鲜。150 ms 允许、151 ms 需重读，unsigned 差值支持 tick 回绕；已知 Fault 不因过期而忽略。重读仍受原轮询次数、状态确认窗口及总初始化截止时间约束。

**这些检查是普通 SDO 事务匹配和软件缓存有效性检查，不是线上事务认证。** 软件 generation 和接收时间不能辨认恰好落入新 ACTIVE 请求的旧同对象应答。没有保留 N+1，也没有改成 N+2 或其他数量门槛。CAN 的某些错误重传情形可造成重复接收，参见 [CiA 会议论文](https://can-cia.org/fileadmin/cia/documents/proceedings/1999_livani.pdf)；这不证明当前 Kinco 驱动在现场必然产生此类时序。

## E. 回归与对抗验证

最终软件验证由工程原有构建脚本执行：

```powershell
python tools/run_wheel_rpdo_tests.py
python tools/verify_tpdo_default.py
python tools/verify_odom_precision.py
python tools/test_rpdo_tools.py
python tools/test_lift_recovery_tools.py
powershell -NoProfile -ExecutionPolicy Bypass -File tools/build_firmware.ps1 -BuildProfile RELEASE_TEST
```

BASIC、严格配置及原有生产 C 回归保留。新增检查如下：

| 场景 | 当前期望及记录 |
|---|---|
| 双轮立即回复 | 第一轮直接 DISABLED，无五秒等待 |
| 任一或双轮延迟 3 s；第二轮才响应 | 等待满 5000 ms 后完整初始化一次 |
| 永久无响应 | 第二次失败 Fault，继续 60 s 无第三轮 |
| 写 ACK 正常、状态读取超时 | 第一轮等待一次；第二轮有回复则成功，否则 Fault |
| 有效 non-disabled 后在窗口内变 disabled | 保留轮询，可在第一轮或第二轮成功 |
| Abort、DS402 Fault、EMCY、BUS_OFF | 不作为普通超时恢复，不自动清除 |
| EMCY Fault→Clear，噪声前缀 0/7/8 帧 | 覆盖同批和 RX 预算切分；5000 ms 边界不增加 retries、不发送第二轮控制字、不记录第二轮 START |
| 6040 ACK DLC 4/5/7，错误 CS/index/sub | 不进入成功；短 ACK 不触发状态读取或第二轮；runtime async/blocking 同样拒绝 |
| 通用 SDO DLC | Node 1/2/3，异步与阻塞读写；短 U8/U16 upload、非 6040 短 ACK 拒绝，完整 8 字节接受；失败 mailbox 无效；不支持的 DLC 拒绝 |
| 完成结果延迟消费 | 每配置 32 项：左右轮 × 两次启动 × 普通/回绕 tick × 149/150/151/200 ms；保留原 RX 戳，超过 150 ms 必须取得新的回读才 READY |
| 真实 `LIFT_RAW_STATUS` 交错 | 每配置 4 项：Node 3 静默造成 blocking 诊断延迟，左右轮、两次启动均不能把旧完成结果直接用于 READY |
| typed SDO 宽度 | 每配置 108 项：Node 1/2/3，期望 1/2/4 字节与 CS 4F/4B/43 交叉，异步/阻塞，boot observer 开启/结束；仅匹配宽度成功 |
| Host HAL 与 RX 队列 | DLC 使用随包 HAL 的 raw 编码；每配置 4 项容量 8、消费时注入 Fault/Clear 的模型检查；保留 128 槽软件预算压力场景，不宣称硬件 FIFO 溢出验证 |
| R3 Resume 内部新到帧 | 每配置 8 项：直接 BootService 与完整 Drive_ControlLoop_Update，容量 8、左右轮 Fault/Clear 在 Resume 排空期间到达；不得增加重试数、记录第二轮 START 或发第二轮 SDO；普通新到帧排空后可重试，持续到帧保持有界等待 |
| WAIT 中连续运动/停止命令、tick 回绕 | 无运动、无缓存重放、等待不改变，4999/5000 ms 正确 |
| TPDO、Lift、运行期反馈失联 | 原授权条件及锁存保护保持 |
| **旧同对象响应撞上活动请求** | **仍能产生 READY 和软件 hardware_disable_confirmed=true；未自动使能/运动。记录为 KNOWN LIMITATION，不计作“防重放安全通过”** |

R3 修复前反例保存在 `output/boot_recovery_v1_review_r3/baseline_external_r2_probes.log`，BASIC/严格配置均复现“剩余两帧 EMCY、第二轮已启动且完整主循环已发送一条 shutdown”的问题，没有自动使能或运动。baseline 的退出码 0 表示旧漏洞复现成功。当前最终软件结果以 R3 同目录 `final_regression.log`、`final_build.log` 为准。R2 目录中的 RX 年龄和 typed 宽度反例及结果、R1 日志只作为历史证据。软件测试通过不代表重放局限已解决，也不能证明实际电机安全。

## F. ARM 构建与交付

最终构建成功：**55 个编译单元，0 warnings，0 errors**。完整软件回归随最终构建重新执行并通过；重放反例仍标为 KNOWN LIMITATION。

Build ID：`AMR-DEV-26101306443017-MANUAL-RT`。ELF SHA-256：`af2186c6a924cfd67afa7935201c03e24e5336be0144a8a022552ff0a3a950a7`。

采用工程既有 RELEASE_TEST / -Og 构建，工具链为 Arm GNU 14.2 rel1。最终编译结果见 `output/boot_recovery_v1_review_r3/final_build.log`；当前产物及配套身份见 `Debug/AMR_First.elf`、`Debug/BUILD_INFO.txt`、`Debug/AMR_First.elf.sha256` 和 `output/jetson_tools_ready/`。

当前 R3 源码与 R2 备份 SHA、差异及最终 ELF SHA 记录在 `output/boot_recovery_v1_review_r3/manifest.json`、`changes.patch`。更早目录的 manifest 对应各自历史版本，不是当前源码校验表。构建配置中的 `hardware_execution_allowed=1` 是 RELEASE_TEST 的既有配置标志，不代表本次完成实机验收或批准生产部署。未自动烧录。

## G. 实机验证与回滚

台架测试应同步记录 UART、CAN trace、左右轮供电时序、TEC/REC、BUS_OFF、FDCAN TX pending、真实 6041 请求与应答；保留硬件急停。重点如下：

1. 正常冷启动及左右轮分别延迟约 3 s，测量失败→第二轮 START 至少 5000 ms，只有一次重试。
2. 分别制造完全无 ACK、TX 积压与 BUS_OFF，确认硬故障保护，并核对共享总线 Lift 行为；Host fake bus 不模拟真实仲裁、硬件重传或积压。
3. 注入 EMCY Fault→Clear，尤其是 5000 ms 边界，确认没有第二轮 START、自动 Fault Reset 或运动。
4. 测量 Kinco ACK→disabled 状态转换时间、SDO 最大延迟及重传/重复行为；查实驱动固件版本、L/R 铭牌和厂商保证。跨 MCU 复位残留、外部主站或重放网关也需纳入适用范围。
5. 验证真实双轮未 Operation Enabled、TPDO 有效和 Lift 独立零速条件；恢复后必须新显式 ENABLE，等待期间速度命令不得重放。

回滚 R3 可在停机后按相对路径恢复 `output/boot_recovery_v1_review_r3/before/`，其中 R2 仍有本轮修复的排空边界缺陷。R2、R1 的更早备份位于对应修订证据目录；最初 V1 之前的工程备份位于 `output/boot_recovery_v1/before/`。旧版本不是安全推荐版本。应配套恢复源码、测试、ELF/身份文件与 Jetson 包，保存现场日志。无 Git 元数据，不使用 git reset；只恢复源码时应重新构建。本次没有刷写。

## H. 最终判定与剩余风险

EMCY Fault/Clear 锁存、畸形 SDO 成功判定、缓存年龄重置和 typed upload 宽度问题已修复；R3 进一步修复排空期间新到故障尚未处理就开始第二轮的问题。简化 V1 保留启动可恢复性、一次非阻塞五秒等待和真实故障保护。没有用未经证实的厂商行为建立新鲜性证明。外部复审结果另存为对应轮次证据，当前报告自身不代表外部批准。

P1 同对象旧应答误匹配作为用户明确接受的、待实机验证的剩余风险保留。**不能宣称三个问题全部消除，不能宣称原严格“任意旧应答不得授权”的验收已通过。** 若实机/厂商证据无法支持所需风险界限，必须重新评估是否允许该恢复策略。

真实驱动固件版本、SDO 最大响应时延、重复应答行为、无人 ACK 时的硬件 TX 状态及完整主循环长尾延迟仍未确认。当前结论限于源码审查、Host 回归和 ARM 构建；实机安全验收 NOT RUN。
