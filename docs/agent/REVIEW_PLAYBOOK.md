# Review Playbook

Use this playbook when asked to review the repository, diagnose weaknesses, or update the roadmap.

## 1. Establish the review scope

Determine whether the request is:

- a full repository review;
- a review of changes since a commit or previous report;
- a subsystem review;
- a design review before implementation;
- a learning review focused on developer understanding.

Do not broaden a narrow request without stating why.

## 2. Gather evidence

Inspect the smallest set of files needed to support the review, expanding only when necessary.

For a full review, inspect:

- repository tree and package layout;
- `README.md` and `docs/README.md`;
- ROS2 package manifests, launch files, node entry points, topics, and message types;
- simulator adapters and control interfaces;
- configuration and parameter files;
- tests, evaluation scripts, logs, demos, and benchmark outputs;
- recent commits and unresolved issues when available;
- the latest report in `docs/agent/reports/`, if any.

Do not infer runtime behavior from filenames alone.

## 3. Build the system map

For every relevant subsystem, record:

| Field | Required content |
| --- | --- |
| Status | implemented, partial, stubbed, absent, or unclear |
| Responsibility | what the subsystem is supposed to do |
| Inputs | topics, files, messages, arrays, frames, and units |
| Outputs | topics, files, messages, arrays, frames, and units |
| Timing | rate, timestamp source, synchronization assumptions |
| Consumers | downstream modules |
| Assumptions | environmental, model, geometry, and data assumptions |
| Failure modes | likely observable failures |
| Evidence | file paths, symbols, tests, logs, or documentation |

Pay special attention to:

- frame conventions and transforms;
- timestamp propagation and stale data;
- units and sign conventions;
- conversion from image-space perception to metric geometry;
- static versus dynamic obstacle semantics;
- planner/controller contracts;
- simulator command scaling and saturation;
- open-loop versus genuinely closed-loop behavior.

## 4. Review through five lenses

### Architecture

Check subsystem boundaries, coupling, data ownership, replaceability, and whether the pipeline forms a coherent loop.

### Autonomous-systems correctness

Check whether representations and algorithms are suitable for their stated use, including perception uncertainty, mapping assumptions, planning feasibility, control stability, and actuation semantics.

### Mathematics and numerics

Check conditioning, discretization, coordinate transformations, probabilistic assumptions, optimization formulation, numerical stability, and unit consistency.

### Software engineering

Check testability, configuration, reproducibility, logging, failure handling, dependency management, and code complexity. Do not overvalue style issues that do not block learning or integration.

### Learning value

Identify which components should remain library-assisted, which need engineering-level understanding, and which one or two components justify reimplementation depth.

## 5. Classify findings

Every finding must be placed in exactly one category:

1. **Conceptual blocker** — the current design cannot support the intended next milestone or is likely incorrect.
2. **Required now** — needed for the next dependency-aware milestone.
3. **Useful before prototype completion** — valuable, but not on the immediate critical path.
4. **Valuable later** — appropriate after the first closed loop works.
5. **Not justified for this prototype** — likely overengineering or outside scope.

For each actionable finding, include:

- evidence;
- consequence;
- recommended action;
- dependency;
- estimated effort: XS, S, M, L, or XL;
- learning value: low, medium, or high;
- what lower-priority work it displaces.

## 6. Prioritize

Use qualitative judgment, informed by:

```text
priority ~ (project impact + urgency + dependency centrality + learning value) / effort
```

Do not present the formula as precise mathematics. It is a guardrail against ranking low-impact polish above critical integration work.

Limit the active roadmap to a realistic number of tasks. Prefer three well-defined tasks over fifteen vague tasks.

## 7. Verify understanding

When developer understanding matters, ask targeted questions rather than guessing. Typical questions:

1. What are this component's inputs and outputs?
2. In which coordinate frame and units are they expressed?
3. What assumptions make the algorithm valid?
4. What is the most likely failure mode?
5. Which parameter most affects behavior?
6. How would you test whether the output is wrong?
7. Could you replace the component without regenerating the surrounding architecture?

Classify understanding using the levels in `AGENTS.md`.

## 8. Produce and store the report

Use `docs/agent/REPORT_TEMPLATE.md`.

For a substantial review, create:

```text
docs/agent/reports/YYYY-MM-DD-<scope>.md
```

Update `CURRENT_STATE.md`, `ARCHITECTURE.md`, and `ROADMAP.md` only when the evidence supports a change. Add entries to `DECISION_LOG.md` and `LEARNING_LOG.md`; do not erase prior entries.

## 9. Implementation boundary

A diagnostic request does not authorize code changes.

When implementation is explicitly requested:

- propose the smallest coherent change;
- state acceptance criteria before editing;
- preserve interfaces unless the change requires otherwise;
- run relevant checks;
- report what was and was not validated.
