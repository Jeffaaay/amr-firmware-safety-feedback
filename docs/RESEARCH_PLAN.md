# CODEX START HERE — Execute the Frozen AMR Research Plan

**Audience: Codex (not a human recipient).** Treat this as an executable engineering handoff, not as a message to send. The plan below is frozen: do not brainstorm new papers or change scope.

**Immediate actions:**
1. Inspect the current local workspace and verify the GitHub account, permissions, source locations and repository names. Do not guess source paths or current firmware behavior.
2. Create the three **private** GitHub repositories below if authentication and creation permissions are available. If any cannot be created, record the exact blocker and provide commands, without claiming success. Add non-confidential planning/report files only; keep proprietary assets private and do not publish any research outcome prematurely.
3. Perform the Oct 23–24 **read-only audit** against the actual STM32/Jetson source. No code changes, test motion, firmware flash, or unsafe production action during the audit. Generate an audit report citing concrete source files, paths, SHA and line numbers where possible. Explicitly mark unknowns.
4. Record the three newly confirmed constraints in `docs/RESEARCH_PLAN.md` and issues: (a) measure conservative braking deceleration on the existing AMR now under an approved safe protocol, (b) correlate Jetson command-send/event-receive timestamps with MCU events using run/command sequence and clock-domain metadata, (c) maintain the fixed Oct 23–Nov 1 execution order.
5. Then prepare Trace Schema v1 and implementation issues for the next milestone, but **do not bypass the October 25 schema freeze or perform hardware tests without approval**. Do not present a planned task as completed.
6. Return a structured status report with actual repository URLs, SHA, audit findings, missing parameters, blockers and the single next executable task.

**Security / IP:** Never send proprietary Bayro code or logs to external LLM APIs or public GitHub without authorization. Never expose keys. Keep agent-generated edits confined to isolated development paths. Production firmware and real motion systems are out of scope for automated modification.

---

# AMR Research Program — Frozen Plan and Codex Execution Handoff

**Plan frozen:** October 23, 2026  
**Owner GitHub account:** `Jeffaaay` (verify authentication and permissions before creating repos)  
**Target:** Three distinct research papers, one reusable experiment harness, research logs with traceable evidence.  
**Status:** Plan / implementation instructions — NOT a claim that experiments, repositories, or submissions are complete.

## 1. Research papers (do not change scope without explicit instruction)

### Paper 1 — AMR-FaultBench
**Working title:** *Agentic Synthesis and Verification of Fault-Recovery Policies for Warehouse AMRs*

**Research question:** Given system-fault traces and permitted interfaces, can coding agents generate recovery policies that retain correctness and safety on held-out faults?

**Experimental focus:** Primarily isolated software-in-the-loop (SIL) CANopen/DS402 + ROS 2 fault injection, with controlled bench hardware-in-the-loop (HIL) confirmation as authorized. Baselines: (a) manually authored policy, (b) single coding agent, (c) typed-graph agent, (d) typed-graph agent with simulation feedback. Separate fault classes for development and held-out evaluation; hold out fault types as well as seeds. Measure safe recovery success, detection time, recovery time, forbidden transitions, human engineering time and agent compute budget. Safety invariants must not be weakened to increase completion rates.

**Target:** RSS 2027 Stage 1 Extended Abstract, December 4, 2026.

### Paper 2 — Agentic Commissioning
**Working title:** *Agentic Commissioning of Payload Variants: Can Coding Agents Re-derive Safety Limits from Platform Parameters?*

**Research question:** Can an agent derive physically defensible control / safety configuration ranges for an existing AMR and planned 600 kg and 1000 kg payload-rated variants, using verified specifications and measurements?

**Experimental focus:** Agent-produced candidate parameters and derivations versus independent physics-based admissibility tests (a feasible *region*, NOT a unique “ground-truth number”), manual commissioning, and feedback-informed commissioning. Inputs include unloaded/loaded robot mass, payload and center of gravity, actual braking capability, speed, sensing/communication/actuation delays, test-floor conditions and uncertainties. Candidate metrics: safety-constraint satisfaction, false stops, commissioning engineering effort, correction rounds, and unseen load/speed generalization.

For orientation, a simplified braking feasibility inequality is `v * (tau_detect + tau_comm + tau_act) + v^2 / (2 * a_min) + margin <= d_available`. The lower-bound effective deceleration `a_min` must come from conservative measurements with defined conditions; this approximation is **not** a complete vehicle safety model and does not give a unique valid TPDO timeout. The 600 kg / 1000 kg ratings refer to payload targets and must not be treated as robot curb masses. Never claim actual 600 kg / 1000 kg vehicle validation without those vehicles and measured data.

**Target:** Complete a substantiated preprint draft by late December if evidence supports claims; target IROS 2027, March 1, 2027, subject to submission rules.

### Paper 3 — Firmware Safety Feedback
**Working title:** *Firmware-Guarded Agentic Iteration for Autonomous Mobile Robots*

**Research question:** With the *same* STM32-enforced safety envelope always enabled, does providing structured safety-trigger feedback to a coding agent improve the efficiency and robustness of AMR navigation/docking skill development?

**Experimental focus:** Real STM32 / CANopen HIL and, with safety review, genuine AMR docking tests. Two primary conditions: agent without safety-trigger details vs. agent receiving the same envelope’s structured trigger traces. Both conditions use identical safety limits, budgets, tasks, and initial configurations. Measure task completion, attempted/blocked unsafe commands, trigger counts, iteration-to-success, and engineering time. Agent is NEVER allowed to edit or disable safety limits, emergency-stop paths, or the independent evaluator.

Only claim firmware-enforced quantities that STM32 can actually observe and limit. Physical workspace constraints require independently valid state estimates and a suitable enforcement architecture. Dual-arm manipulation is future applicability, not a committed 2026 experiment; optional 2027 extension only if hardware and RSS Extension Charter justify it.

**Target:** RSS 2027 Stage 1 Extended Abstract, December 4, 2026.

## 2. Exactly three GitHub repositories

Create these under **`Jeffaaay`**, initially **private** (company IP, credentials and RSS anonymity):

1. `Jeffaaay/amr-faultbench` — authoritative shared harness source (`core/`, traces, experiment runner, fault injection, CANopen emulator, evaluators) plus Paper 1 adapter / experiments.
2. `Jeffaaay/amr-agentic-commissioning` — Paper 2 specification extraction, parameter tables, braking physics oracle, parameter sweeps, and commissioning comparison; use a pinned version of the harness, do not copy its core source.
3. `Jeffaaay/amr-firmware-safety-feedback` — Paper 3 STM32 safety-event instrumentation, Docking/HIL adapters and feedback/no-feedback comparisons; use a pinned version of the harness.

Do NOT create a fourth shared-harness repository and do NOT fork the complete harness into every repo. Initially reference the shared harness from the first repository at a locked commit / release or submodule as practical. Pin exact dependency SHAs in experiment manifests. Make each repo independently understandable and reproducible without exposing Bayro proprietary firmware or unapproved logs.

When logged in to the authorized GitHub account (or as an account with approved repository creation access), the intended CLI setup is:

```bash
gh auth status
gh api user --jq .login  # must show Jeffaaay, or ensure equivalent approved access

gh repo create Jeffaaay/amr-faultbench --private --add-readme \
  --description "Reproducible CANopen/ROS2 fault injection and agentic AMR recovery research"
gh repo create Jeffaaay/amr-agentic-commissioning --private --add-readme \
  --description "Physics-grounded agentic commissioning across AMR payload variants"
gh repo create Jeffaaay/amr-firmware-safety-feedback --private --add-readme \
  --description "STM32 safety-trigger feedback for agentic AMR skill development"
```

After creation, add an initial `README.md`, `docs/RESEARCH_PLAN.md`, a `docs/DECISIONS.md`, a `docs/EXPERIMENT_LOG.md`, and a minimal issue backlog in each. Never invent a successful GitHub action; report actual repository links and initial commit SHAs.

## 3. Three NEW fixed requirements — record explicitly

### A. Measure braking deceleration `a_min` **now**, on the existing AMR

Do NOT defer until 600 kg hardware arrives. In an authorized, isolated, safe test area, with an appropriate operator, independent emergency stop, verified payload handling and an approved test procedure: measure empty and representative maximum rated load at three *approved, safe* speeds. Record stop command timestamp, stop-state/actual stop timing, distance traveled, speed trace, motor/wheel response and test-floor/load conditions. Aim for repeated runs per condition, if the site authorizes them. Start from conservative speeds, stop the test if any uncontrolled behavior is observed, and do not perform uncontrolled fault injection on an operational fleet robot.

Estimate conservative achievable deceleration ranges from the data, with uncertainty and conditions. Do not claim a single constant `a_min` is universally valid. Later repeat a corresponding approved protocol on a completed 600 kg variant, if/when available.

**Required outputs:** `platforms/current_amr.yaml`, `braking_tests/protocol.md`, raw timestamped traces, `braking_tests/summary.csv`, and plot(s) with speed, stopping distance and uncertainty. Store proprietary source/raw data privately unless cleared for public release.

### B. Time-correlate STM32, CAN and Jetson/ROS 2 traces by design

Minimum event metadata:

- `run_id`, `mcu_boot_id` (or equivalent), and scoped `command_sequence` (including wrap-around policy).
- Jetson `send_monotonic_ns` for each command and `receive_monotonic_ns` when it receives a safety event.
- STM32 `mcu_timestamp_us` for the event and the responsible command sequence.
- CAN send/receive timestamps where supported, with clock-domain annotation.
- `event_type`, `rule_id`, `decision` (e.g. REJECT / CLAMP / SAFE_STOP), requested/accepted command (units), relevant system state, firmware/software commit IDs, and dropped-event counters.

**Crucial:** independent MCU and Jetson clocks are **not directly comparable**. Use command sequence / run ID for causal association and send/receive bounds or a separately validated clock-offset synchronization method if absolute latency estimates are required. Use bounded, non-blocking STM32 event logging; a logging failure must never affect control and must be observable.

**Required outputs:** `trace_schema_v1.json`, event schema examples, a matching/reconciliation test across ROS 2 ↔ CAN ↔ STM32, and a documented timestamp uncertainty model.

### C. Fixed execution order for this week

- **Oct 23–24:** Codex performs strictly read-only audit of existing STM32 firmware and Jetson ROS 2: exact commit SHA, build steps, CANopen/DS402/PDO interfaces, watchdog values, command path, safety-interlock inventory, and source-code/IP boundaries. No edits to production code, no flashing, no production credentials or external API uploads.
- **Oct 25:** Freeze trace schema, correlation method, required event vocabulary and schema validation tests.
- **Oct 26–Nov 1:** In parallel build (1) virtual DS402 CANopen slave + fault injector on isolated Linux `vcan` / software test setup and (2) minimal STM32 safety-event logging on a separate approved development/test branch and HIL bench. Perform the controlled braking measurement during a suitable authorized test session. No production robot modifications.
- The four P1 documentation items (paper questions / baselines / related work / protocol) are an evening writing task and must not interrupt the infrastructure implementation.

## 4. Harness execution boundaries

Agent may generate or edit candidates only in allowlisted experimental `policies/` paths. Test runner/evaluator, hidden scenarios, safety constraints and baseline fixtures must be isolated from editable paths. Validate with schema checks, builds, regression and safety tests before accepting any candidate. Never grant the agent live production credentials, motor motion permissions or unrestricted firmware flashing. Do not send confidential Bayro source or logs to external LLM APIs without explicit authorization.

First acceptance gate: a single deterministic `F01_TPDO_DROPOUT` SIL scenario executes against an independently checked controller/model, produces correlated event traces, passes schema validation, and gives a reproducible evaluation report. A successful demo is not yet proof of research novelty or robust generalization.

## 5. Paper logistics and public record

- **RSS 2027 Stage 1:** December 4, 2026; six pages: up to five content pages and one page of references. Paper 1 and Paper 3 are the only RSS Stage 1 targets in this plan.
- **RSS decisions:** February 5, 2027 (reject or rebuttal invitation), February 12 rebuttal deadline, February 26 decision (reject or Extension Charter), April 16 Stage 2 final deadline.
- **IROS 2027:** March 1, 2027 target for Paper 2. Prepare alternate formats for P1 / P3 ahead of the February 26 RSS result but never submit any manuscript simultaneously to two conferences.
- **GitHub:** capture commits, issue histories, experimental logs, and sanitized reproducibility assets. Start private pending IP review and RSS anonymity safeguards.
- **OpenReview:** register/activate early for RSS submissions.
- **arXiv (cs.RO):** register early; first-time/category submissions may require endorsement. arXiv hosts scholarly preprints, NOT daily research notes. Decide posting only when a substantial manuscript exists and after considering RSS blind-review exposure.
- **X:** daily short, accurate, authorized engineering/research logs. Do not reveal unpublished/company confidential details or personally identifiable links to blinded P1/P3 submissions before review decisions.

Primary references:
- RSS 2027 CFP: https://roboticsconference.org/information/cfp/
- OpenReview signup: https://openreview.net/signup
- arXiv registration: https://arxiv.org/user/register
- arXiv endorsement: https://info.arxiv.org/help/endorsement.html

## 6. Report back to the owner

Return one written status message containing:
1. Actual URLs and visibility of all three repos, plus their initial commit SHAs (or a clear blocker).
2. Link/path to the updated research report and decisions log in each repo.
3. Read-only audit outputs: exact SHAs, interfaces, config values, safety interlocks, unknowns, and IP boundaries.
4. Proposed Trace Schema v1 with at least one command-to-safety-event correlation example.
5. Braking experiment readiness (approved test plan, instrumentation and required permissions) — do not state measured `a_min` before data exist.
6. A prioritized issue list and one next executable task. No claims of completed experiments without verifiable files/logs.
