# Project Roadmap

Updated 2026-09-16 against `7b54639`. [Lane mapping](../milestones/lane_perception_tracking_mapping.md) and [global routing](../milestones/global_route_planning.md) are completed development integration milestones. The branch `feature/route-lane-association` names the NEXT milestone; no association implementation exists yet.

The active milestone is RoutePlan ↔ LaneMap association. Keep the following two tasks active; later stages are dependency gates, not concurrent commitments. Effort estimates are qualitative (S: bounded design/probe work; M: several implementation/validation sessions; L: broader integration). They are not deadlines.

## Required now — 1. Define and validate geometric association semantics

- **Benefit / why now:** connect global intent to local perceived geometry without mixing ownership, IDs, frames, or uncertainty.
- **Dependency:** current direct-world RoutePlan and LaneMap contracts, hero pose, and the completed frame/startup checkpoint.
- **Deliverable:** a small design and diagnostic specification for selecting the local RoutePlan window around ego; route-relative progress `s`, signed lateral offset `d`, heading error, overlap/coverage, and confidence-aware gates. Specify temporal hysteresis only where measured ambiguity warrants it.
- **Acceptance:** define projection/tie handling, sign conventions in direct CARLA world, input-age policy, new-route/reset behavior, and explicit no-valid-association output. Include empty/partial LaneMap and intersection examples. No equality matching of global/local IDs or revisions; no extra y flip; no OpenDRIVE-filled perception.
- **Effort / learning value:** S–M / high; Level 2 geometry, coordinate contracts, and uncertainty reasoning.
- **Displaces:** new perception models, classical lane-detector retuning, legacy occupancy polish, and premature message proliferation.

These are planned concepts, not a finalized ROS message. Source already exposes local-forward-left curvature alongside world-converted LaneMap poses; record which scalar conventions association needs, and validate signed curvature before any later controller consumes it.

## Required now — 2. Implement and runtime-validate the separate association layer

- **Benefit / why now:** produce a reusable route-relative local reference/corridor for the next planner.
- **Dependency:** task 1's accepted semantics and diagnostic cases.
- **Deliverable:** one modular consumer of RoutePlan, LaneMap, and required ego state; preserve topology generation and lane tracking responsibilities.
- **Acceptance:** demonstrate matched geometry on straightforward lanes, explicit rejection for invalid routes/poor evidence, tolerance of disappearing or partial lanes at intersections, and stable identity/progress behavior across updates. Measure lateral/heading discrepancy, overlap, accepted/rejected availability, discontinuities, and latency; choose thresholds from evidence rather than inventing them here. Record runtime evidence before marking the milestone complete.
- **Effort / learning value:** M / high; Level 2 association and integration debugging.
- **Displaces:** implementing multiple planners or advanced optimization before a reliable input/reference boundary exists.

No implementation or message creation is authorized by the documentation refresh itself.

## Dependency-gated work before prototype completion

| Sequence | Benefit / deliverable | Dependency / acceptance | Effort; learning value | Lower-value work displaced |
| --- | --- | --- | --- | --- |
| 3. Simple local reference/trajectory planner | Convert accepted local corridor into a usable reference with explicit speed/timing/validity semantics | Validated association; define behavior/maneuver responsibility and missing-evidence handling; verify continuity and simple feasibility limits | M–L; high | MPC and complex optimization before a baseline works |
| 4. Trajectory tracking/control | Close the feedback loop with one understandable lateral and longitudinal baseline | Defined trajectory/ego-state contract; measure lateral/speed error and command limits in repeatable CARLA scenarios | M; high | Multiple competing controllers, replacement of simulator odometry |
| 5. Systematic closed-loop evaluation | Quantify completion, collisions/boundary violations, tracking error, smoothness, and latency | Working planner/controller; repeatable scenario setup and recorded metrics | M; high | Adding scenarios/features without a comparable baseline |

Pure Pursuit or Stanley plus PID are possible future baselines, not current implementations or a finalized controller choice. Minimal behavior/maneuver policy belongs at the appropriate planning boundary; global legal lane-change edges alone cannot authorize executing a maneuver around dynamic traffic.

## Valuable later / not justified now

After the first measured loop, select one planning, optimization, or control depth experiment with a baseline comparison (M–L, high learning value; depends on stage 5; displaces additional breadth). Level 3 is a selective target, not a claim of existing mastery.

Full SLAM, production HD-map infrastructure, complex learned prediction, large deployment systems, and broad refactors are not justified for the immediate prototype. They would displace the two active integration tasks without completing their dependencies. Older object/depth/spatial capabilities remain available but are outside the association critical path.

## Evidence and completion rule

Use [CURRENT_STATE.md](CURRENT_STATE.md) and [ARCHITECTURE.md](ARCHITECTURE.md) as the baseline. Source verification establishes implementation; runtime diagnostics establish the bounded integration behavior actually demonstrated. A visually plausible route or a message schema does not complete association, trajectory planning, or control.
