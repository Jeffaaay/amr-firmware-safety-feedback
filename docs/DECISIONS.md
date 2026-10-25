# Decisions

2026-10-23 — Preserve the supplied frozen three-paper plan verbatim in RESEARCH_PLAN.md. Dates and submission targets there are owner-supplied planning constraints, not independently verified conference facts.

2026-10-23 — Exactly three repositories, all private. amr-faultbench alone owns the shared harness. Other repositories consume an exact commit through dependencies.lock.json; no copied core.

2026-10-23 — M0 is read-only against source trees. No production edits, firmware flashing, hardware access or robot operation. New files belong only to isolated research checkouts.

2026-10-23 — Source-derived proprietary audit evidence remains local. Only original planning, generic schemas, synthetic examples, readiness documents and sanitized status are uploaded. No company source, credentials, logs or experiment results.

2026-10-23 — Trace Schema v1 remains a proposal until October 25. Synthetic checks cannot establish timing or safety on hardware.

2026-10-23 — Braking values remain unknown until authorized measured evidence exists. 600 kg and 1000 kg are planned payload ratings, not curb masses or validated vehicles.

2026-10-23 — Available source versions differ. Preserve separate provenance and resolve deployed baseline before binding experiments. Fourteen synthetic trace-contract tests passed; freeze and hardware acceptance remain open.

2026-10-23 — Added the owner-supplied execution sequence, a local source-backed PCB baseline diagram, and paper-specific readiness contracts. Proprietary source-derived details stay local. No change to the October 23–November 1 gates, no added paper, no hardware action, no experimental result.

2026-10-23 — Trace proposal revised to draft.2 after read-only stop/recovery inspection. Explicit stopping-observation scope, CAN metadata and native-counter normalization prevent false evidence equivalence. Offline contract review is available; October 25 freeze, real integration and all hardware approvals remain open.

2026-10-24 — The owner explicitly requested a public baseline code folder in the
three existing paper repositories. This supersedes the earlier private-only and
no-source-publication decisions for the selected AMR PCB snapshot only. Each
repository now carries the same byte-exact source baseline under code/, identified
by a manifest digest, with paper-specific entry points. Production sources are
unchanged. Bayro Fleet deployment code/data and credentials are not part of this
selection. The shared research harness still belongs to P1 and existing dependency
pins remain unchanged. Baseline host regressions are software checks, not paper
experiment results or hardware validation; the frozen research schedule is unchanged.
