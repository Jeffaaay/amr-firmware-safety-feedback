# amr-firmware-safety-feedback

Paper 3: Firmware Safety Feedback. Private research workspace initialized 2026-10-23. No experiment or hardware validation is claimed.

- [Frozen research plan](docs/RESEARCH_PLAN.md)
- [Implementation milestones](docs/MILESTONES.md)
- [Decisions and IP boundaries](docs/DECISIONS.md)
- [Audit status](docs/AUDIT_STATUS.md)
- [Experiment log](docs/EXPERIMENT_LOG.md)

## Structure

- `instrumentation/`
- `adapters/hil/`
- `adapters/docking/`
- `experiments/`
- `tests/`

## Execution boundary

Only allowlisted experimental policies may be edited by candidate-generating agents. Evaluators, hidden scenarios, baseline fixtures and safety constraints must be isolated. No live robot credentials, motion or flashing permissions. No safety-limit weakening.

The shared harness lives only in Jeffaaay/amr-faultbench. Exact dependency pins will be recorded before use. Oct 23–24 audit; Oct 25 schema freeze; Oct 26–Nov 1 isolated implementation and separately authorized measurements.

- [Research execution report](docs/RESEARCH_REPORT.md)
- [Prioritized GitHub backlog](docs/ISSUES.md)

Shared harness reference is locked to `a21dd7c26fc8fdcaff21e46182751ad790fd3877` in dependencies.lock.json and experiments/manifest.template.json. It currently contains scaffolding and a draft trace contract, not a runnable experiment harness. Check out exactly that commit; do not copy core. Before running an experiment, fill every provenance/approval field and record this repository's own commit.
