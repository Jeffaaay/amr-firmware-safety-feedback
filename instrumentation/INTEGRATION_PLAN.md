# Safety-event instrumentation integration plan

Scheduled October 26–November 1 only after October 25 schema freeze and separate development/HIL approval. The current application-source baseline is included under code/; production firmware is not modified by this research checkout.

1. Bind the approved firmware/software/configuration digests to the harness dependency pin.
2. Define verified source mappings for command acceptance/rejection, clamps, stop requests and independently observed outcomes. Preserve original safety limits and stop paths in both feedback conditions.
3. Implement bounded event capture on an isolated approved branch, with run/session/boot/command identities and explicit clock domains. Keep logging failure independent of safety action.
4. Run overflow, reboot, sequence/timer wrap, queue loss and timing checks on software, then the approved bench. Validate firmware-observable quantities only.
5. Compare agents with and without trigger details under identical tasks, starting configuration, envelope, evaluator and budgets. HIL and docking require explicit safety review and access; neither has run.

The initial milestone is provenance and schema mapping, not a new control envelope. The researcher cannot edit emergency-stop paths, safety limits or independent evaluators.
