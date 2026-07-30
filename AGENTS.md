# Codex Instructions

## Mission

Help develop this repository as a complete but deliberately simple educational autonomous-driving prototype.

The immediate objective is not production parity with an industrial autonomous-driving stack. The objective is to:

1. complete a coherent end-to-end loop;
2. understand the purpose, assumptions, interfaces, and failure modes of every major subsystem;
3. preserve a modular architecture that can later support deeper work in planning, optimization, and control;
4. avoid unnecessary complexity that competes with university, Formula Student, and professional work.

Before performing a repository-wide diagnosis or proposing a roadmap, read:

- `docs/agent/PROJECT_CONTEXT.md`
- `docs/agent/REVIEW_PLAYBOOK.md`
- `docs/agent/REPORT_TEMPLATE.md`
- `README.md`
- `docs/README.md`

## Operating principles

- Judge the repository against its stated educational objective, not against Waymo-scale production systems.
- Prefer the simplest implementation that teaches the correct concept and enables the next end-to-end milestone.
- Distinguish clearly between:
  - blocking conceptual errors;
  - work required now;
  - useful work before prototype completion;
  - valuable later improvements;
  - unnecessary work for this prototype.
- Never recommend production infrastructure merely because it is considered best practice in large companies.
- Every roadmap recommendation must state its expected benefit, dependency, effort, and what lower-value work it displaces.
- Treat coordinate frames, timestamps, units, data ownership, and module interfaces as first-class concerns.
- Prioritize failures at subsystem boundaries over local code polish.
- Do not replace major architecture decisions without explaining the trade-off and obtaining explicit approval.
- Do not modify code during a diagnostic review unless the user explicitly asks for implementation.

## Review workflow

For a full project review:

1. Inspect the repository structure, entry points, dependencies, configuration, ROS2 packages, launch files, messages, topics, and simulator interfaces.
2. Build or update a concise architecture and data-flow map.
3. Classify each relevant subsystem as `implemented`, `partial`, `stubbed`, `absent`, or `unclear`.
4. Record each subsystem's inputs, outputs, coordinate frame, update rate or timing assumptions, consumers, and likely failure modes.
5. Inspect tests, evaluation scripts, logs, demonstrations, and documentation before judging implementation quality.
6. Identify weaknesses from five perspectives:
   - system architecture;
   - autonomous-systems correctness;
   - mathematical and numerical soundness;
   - software engineering;
   - developer learning value.
7. Rank issues using project impact, urgency, learning value, dependency centrality, and effort.
8. Produce a dependency-aware roadmap using `docs/agent/REPORT_TEMPLATE.md`.
9. Preserve completed milestone documentation as historical records. Add a new review rather than rewriting history unless facts are wrong.

## Educational depth policy

Use three levels when assessing understanding:

- **Level 1 — Operational:** understands purpose, inputs, outputs, configuration, and place in the pipeline.
- **Level 2 — Engineering:** can explain assumptions, debug failures, predict limitations, test behavior, and modify major parts.
- **Level 3 — Reimplementation:** can independently build a simplified version and justify the algorithm mathematically.

Target Level 2 across the whole stack. Recommend Level 3 only for one or two high-value areas, likely planning, optimization, or control. Do not demand reimplementation depth for every dependency or adapter.

## Validation

Before claiming that a change works:

- run the most relevant available tests or checks;
- state exactly what was run;
- state what could not be run and why;
- do not treat visual plausibility as sufficient when a quantitative metric is available;
- do not invent test results, runtime behavior, or repository facts.

## Output style

- Be direct and technically specific.
- Cite file paths and symbols for important findings.
- Separate observations from inferences.
- Avoid generic advice that is not grounded in the repository.
- Keep the active roadmap small enough to be realistic alongside the developer's other commitments.
