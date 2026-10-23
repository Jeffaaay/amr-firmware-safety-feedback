# Implementation milestones

All dates are 2026, America/Los_Angeles. This schedule is fixed.

| Milestone | Window | Acceptance | Current status |
| --- | --- | --- | --- |
| M0 Read-only source audit | Oct 23–24 | Source identities, builds, command path, CANopen/DS402/PDO, communication parameters, interlocks, unknowns and IP boundaries documented | In progress |
| M1 Trace Schema v1 freeze | Oct 25 | Reviewed schema, event vocabulary, causal matching, uncertainty model and validation tests | Draft preparation only; not frozen |
| M2 Isolated implementation and authorized measurements | Oct 26–Nov 1 | Virtual DS402 slave and fault injection; separate approved STM32 development branch/HIL; approved current-AMR braking session | Not started |

First SIL acceptance: deterministic F01_TPDO_DROPOUT against an independently checked controller/model, correlated trace, schema validation, reproducible evaluation report. This is not a hardware or research-validity claim.

P1 questions, baselines, related work and protocol writing is an evening task and must not interrupt infrastructure work. No additional papers.

Three mandatory requirements: measure real braking deceleration on the existing AMR using an approved controlled procedure; unify Jetson/CAN/STM32 traces with explicit clock domains; retain the Oct 23–Nov 1 execution order.
