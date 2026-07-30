# Project Roadmap

Last initialized: 2026-07-30

This is an initial roadmap based on the documented repository state. The first full Codex review should validate and revise it using source code and runtime evidence.

## Roadmap rule

Only a small number of tasks should be active at once. A task enters the active roadmap only when its dependency, acceptance criteria, and learning value are clear.

## Milestone 0 — Validate the system contracts

**Purpose:** ensure that the existing perception and mapping stack can support planning and control without hidden frame, timing, or representation errors.

### Active task candidates

1. **Document coordinate frames, units, and timestamps**
   - Deliverable: a verified interface table for every active ROS2 node.
   - Acceptance: every geometric topic has a producer, consumer, frame, units, and timestamp source.
   - Learning value: high.
   - Displaces: visual polish and new perception features.

2. **Define the planner input contract**
   - Deliverable: one stable representation selected from the current local occupancy or mapping outputs, with documented semantics.
   - Acceptance: a small standalone consumer can read and visualize the exact representation the planner will use.
   - Learning value: high.
   - Displaces: implementing multiple planner variants before the input is stable.

3. **Establish a replayable baseline scenario**
   - Deliverable: one repeatable CARLA route or scenario with logs and baseline outputs.
   - Acceptance: the same scenario can be rerun and produces comparable measurements.
   - Learning value: medium-high.
   - Displaces: adding more scenarios before the first is measurable.

## Milestone 1 — Minimal map-based planning

**Purpose:** generate a usable reference path from the selected environment representation.

- Start with the simplest planner compatible with the representation and scenario.
- Separate path/reference generation from low-level control.
- Do not begin with MPC or a complex optimization-based trajectory planner unless a simpler planner cannot satisfy the baseline scenario.

### Exit criteria

- A documented planner input and output contract exists.
- The planner produces a collision-aware or lane-consistent reference for the baseline scenario.
- Planning time is measured.
- Failure cases are observable and logged.

## Milestone 2 — Basic control and actuation

**Purpose:** track the reference path in closed loop.

Recommended first version:

- Pure Pursuit or Stanley for lateral control;
- PID for longitudinal control;
- simulator-specific scaling and saturation isolated in the actuation layer.

### Exit criteria

- The vehicle follows the reference in the baseline scenario.
- Lateral tracking error, speed error, steering behavior, and completion are measured.
- Controller inputs and outputs have explicit units and sign conventions.
- Planner and controller can be tested independently.

## Milestone 3 — Closed-loop evaluation

**Purpose:** turn the prototype into an engineering experiment rather than a visual demo.

Minimum metrics:

- scenario completion;
- collision or boundary violations;
- lateral tracking error;
- longitudinal speed error;
- control smoothness or steering oscillation;
- per-stage and end-to-end latency.

### Exit criteria

- At least three representative scenarios are repeatable.
- Metrics are collected automatically or with a documented procedure.
- A baseline report identifies the dominant failure mode.

## Milestone 4 — One depth experiment

**Purpose:** deepen one area after the full stack works.

Leading candidate:

- replace the basic lateral controller or planner with an MPC or optimization-based approach;
- compare it against the baseline using the same scenarios and metrics;
- explain the formulation, constraints, solver behavior, and trade-offs.

This milestone should target Level 3 understanding.

## Deferred until justified

- full SLAM;
- HD-map infrastructure;
- complex behavior prediction;
- learned end-to-end planning;
- large-scale deployment infrastructure;
- extensive refactors unrelated to a measured problem;
- multiple advanced controllers before one baseline controller is evaluated.

## First Codex review request

Use this prompt after merging the agent framework:

> Perform a full repository review according to `AGENTS.md` and `docs/agent/REVIEW_PLAYBOOK.md`. Do not modify implementation code. Validate the initial project state and architecture, create a dated report, and update the agent state files only where supported by repository evidence. End with no more than three active tasks for the next development cycle.
