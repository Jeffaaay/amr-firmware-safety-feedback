# Research execution report — October 23, 2026

The frozen plan is preserved. Repository setup, documentation, issue backlog and milestone scheduling are complete. The available local STM32 and ROS 2 sources were statically inspected, with exact Git identities, file/artifact hashes, communication paths and interlock references recorded in a local proprietary audit report. Deployed-baseline identity remains unverified; M0 stays open for reconciliation.

The authoritative shared harness repository is amr-faultbench. Trace Schema v1 and a synthetic Jetson/CAN/MCU correlation example are prepared there; 14 synthetic schema/correlation tests passed. The schema is not frozen. No real ROS/CAN/MCU integration or fault scenario has run.

The commissioning repository contains current_amr.yaml with unknown measurements null, an approval-draft braking protocol and a header-only summary.csv. No measured a_min, raw traces or plots exist. Firmware Safety Feedback contains a development/HIL integration plan without proprietary source.

Three mandatory requirements are recorded in RESEARCH_PLAN.md and issues: real braking deceleration on the current AMR under an approved controlled procedure; unified Jetson/CAN/STM32 traces with explicit identities and clocks; fixed October 23–November 1 execution order.

Outstanding: deployed source/build/config provenance; verified physical specifications; approved braking operator/site/instruments/session; isolated Linux/vcan runtime; separately approved firmware development/HIL access; reviewed correlation mappings. Detailed source-derived findings and production hashes remain local pending IP clearance. No firmware edits, flashing or robot operations occurred.

Next task: reconcile an existing approved deployed build/commit/config manifest as M0 closure. Next dated milestone: October 25 trace-schema freeze. October 26–November 1 implementation has not started. No experiments, paper submissions, public posts or future automatic runs are claimed.

See [decisions](DECISIONS.md), [milestones](MILESTONES.md), [prioritized issues](ISSUES.md), and [experiment log](EXPERIMENT_LOG.md).

2026-10-23 addendum: the local source-backed AMR PCB functional map and interface baseline are complete. Generic Chinese roadmap, F01/recovery/metrics readiness documents, payload templates and feedback-comparison protocol are now available in their owning repositories. See README links. These are preparation artifacts; M0 deployment reconciliation, October 25 freeze and later experiment gates remain open.
