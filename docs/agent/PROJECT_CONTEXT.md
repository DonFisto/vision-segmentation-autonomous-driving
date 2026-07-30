# Project Context

## Purpose

This repository is an educational autonomous-driving prototype. Its primary goal is to build a complete, understandable, closed-loop system in CARLA and ROS2 while exposing the developer to the major components of an autonomous-driving stack.

The project should remain deliberately simpler than a production autonomous-driving system. Completeness, coherent interfaces, measurable behavior, and learning value are more important than feature count or industrial-scale infrastructure.

## Developer context

- The developer is completing a double degree in Mathematics and Computer Science.
- Relevant strengths include algorithms, nonlinear optimization, numerical methods, statistics, concurrent programming, and computer architecture.
- The longer-term technical direction is planning, optimization, and control for autonomous systems.
- Near-term professional goals include gaining industrial experience in autonomous or automotive systems.
- Available time is constrained by a demanding degree program, Formula Student work, and potential part-time engineering work.

Do not recommend a workload that assumes this repository is the developer's only major commitment.

## Current learning objective

Target **Level 2 — Engineering understanding** across the complete stack:

- explain each subsystem's purpose and assumptions;
- understand inputs, outputs, coordinate frames, timing, and consumers;
- diagnose common failures;
- modify or replace major components;
- evaluate behavior with appropriate metrics.

Target **Level 3 — Reimplementation depth** only for one or two selected areas. Planning, optimization, or control are the leading candidates after the first closed-loop prototype works.

## Current project scope

The repository currently covers or partially covers:

- CARLA sensor and odometry input;
- semantic segmentation;
- object extraction and tracking;
- monocular depth;
- depth/object fusion;
- semantic-depth free-space estimation;
- reactive navigation prototypes;
- local static and dynamic occupancy layers;
- short-term accumulated local mapping using simulator-provided odometry;
- CARLA control integration.

The accumulated map is intentionally a short-term local representation, not a complete SLAM system.

## Near-term definition of success

The first complete prototype should demonstrate a reproducible closed loop:

```text
sense -> perceive -> represent local environment -> plan -> control -> actuate -> evaluate
```

For this milestone, the simplest technically coherent solution is preferred. A valid first version may use:

- simulator-provided ego pose or odometry;
- a simple local map or reference-path representation;
- a basic path or trajectory planner;
- Pure Pursuit or Stanley lateral control;
- PID longitudinal control;
- a small set of repeatable CARLA scenarios;
- quantitative tracking, completion, safety, and latency metrics.

## Explicit non-goals for the first prototype

Unless an immediate blocker proves otherwise, do not prioritize:

- full production-grade SLAM;
- a complete HD-map stack;
- complex learned prediction;
- end-to-end driving;
- multi-machine deployment;
- Kubernetes or extensive cloud infrastructure;
- safety certification;
- exhaustive production test coverage;
- premature MPC or complex trajectory optimization before a basic closed loop works.

## Decision rule

When two tasks compete for time, prefer the task that most improves one or more of:

1. end-to-end completion;
2. correctness of subsystem interfaces;
3. ability to diagnose failures;
4. measurable learning in planning, optimization, or control;
5. portfolio clarity for an autonomy engineering role.

Any recommendation should explain what lower-value work it displaces.