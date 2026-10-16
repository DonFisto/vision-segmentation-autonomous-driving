# Autonomous Driving Laboratory — Updated Project Objectives

**Revision date:** 2026-10-16  
**Purpose:** Record the revised goals agreed in discussion with the project supervisor.  
**Status:** Project objectives and intended architecture; this document does not certify that newly described modules have been implemented.  
**Suggested repository destination:** `00_PROJECT_OVERVIEW.md` (or merge these sections into the existing project overview, retaining the source history).

## 1. Main objective

Design, implement, and evaluate a **modular autonomous-driving prototype in CARLA and ROS 2** that can interpret local road geometry, determine a route, plan feasible local motion around simple obstacles, and follow that motion in closed loop.

The project is primarily an educational engineering prototype. Its purpose is to understand the interfaces, assumptions, algorithms, and failure modes of an autonomous-driving stack—not to implement a production-level self-driving system.

The revised objectives add two explicit directions agreed with the supervisor:

1. **Perception-assisted localization:** model GPS as noisy and improve the estimated vehicle pose using odometry and locally perceived road features, with a map as reference where appropriate.
2. **Simple graph/geometric obstacle-aware motion planning:** represent obstacle constraints using *closed graph paths* and construct collision-free local paths using an understandable approach, rather than beginning with advanced trajectory optimization.

The exact meaning of **“closed graph paths” is not yet confirmed**. It could mean closed polygonal cycles around occupied regions or blocked paths/edges in a navigational graph. Keep that design choice open until clarified; neither interpretation is to be recorded as an agreed implementation detail yet.

## 2. Updated technical objectives

### Objective A — Perception and local road representation

Continue using the existing camera/lane-perception pipeline to obtain local lane geometry with temporal tracking, odometry propagation, and explicit confidence or missing-data states. Maintain the route-ready `LaneMap` as the representation of **locally perceived** lane geometry.

**Success evidence:** valid ROS outputs in CARLA, geometric consistency, measured latency, and explicit handling of missing or uncertain markings, especially at junctions.

### Objective B — Global navigation and route/local association

Use CARLA/OpenDRIVE topology to build a directed road/lane graph and determine a destination-oriented `RoutePlan` using graph search. Associate the global route geometrically with the perception-derived `LaneMap` and ego state so downstream planning receives a finite, route-consistent local reference.

Preserve the distinction between the nominal map route and actually observed lane geometry. Do not match global and local lane IDs by assumed equality, or fill missing perception from simulator ground truth without explicitly marking the source.

**Success evidence:** stable route-relative progress, lateral and heading discrepancies, overlap/coverage, confidence-aware validity, and explainable rejection of stale or inconsistent inputs.

### Objective C — Localization from noisy GPS and local perception

Add a **dedicated localization subsystem** whose job is to estimate the vehicle's pose on the map from imperfect measurements.

- Begin with simulated GPS noise and motion/odometry information.
- Use lane/road features perceived relative to the vehicle to constrain the pose with respect to mapped geometry.
- Produce an **estimated pose and uncertainty** (for example, covariance), not a claim of mathematically exact position.
- Keep the estimator separable from global route planning, lane tracking, and trajectory planning.
- Use CARLA's true pose only as a validation reference, not as the estimator's hidden correction input.
- Avoid a circular measurement model: the current world-frame `LaneMap` is already transformed using ego odometry, so it must not automatically be treated as an independent global localization measurement. Prefer ego-relative observations or explicitly model shared odometry/pose uncertainty.

An interpretable baseline may use map matching and a state estimator such as an Extended Kalman Filter, but the algorithm will be selected after examining available sensors, frame conventions, and required accuracy.

**Success evidence:** compare raw noisy GPS, odometry-only, and fused localization against CARLA evaluation ground truth; report position/heading error, uncertainty behavior, drift, and failure cases. A precision improvement must be demonstrated, not assumed.

### Objective D — Simple obstacle-aware local motion planning

Implement a **small, explainable geometric/graph-based local planner** rather than a complex optimization framework.

- Represent obstacles using the supervisor's proposed **closed-graph-path** abstraction once its precise semantics are agreed.
- Construct candidate paths or graph connections that respect obstacle occupancy, road/lane bounds, and the intended route.
- Use a simple shortest-path or graph-search method when appropriate, reusing established graph-search knowledge without coupling the local planner to the global routing graph.
- Account conservatively for vehicle dimensions and clearance; a collision-free point path is not automatically a collision-free vehicle path.
- Convert the chosen geometric path into a continuous reference/trajectory with a basic speed profile and vehicle feasibility checks.
- Start with static or slowly changing test obstacles and explicit stop/fallback behavior. Do not claim full dynamic-agent interaction or prediction.

**Success evidence:** collision-free behavior in repeatable CARLA cases, measured clearance, path continuity, feasible steering/curvature and speed commands, and explicit handling of blocked/no-path scenarios.

**Design decision pending:** verify whether “closed graph paths” means (1) closed polygonal obstacle boundaries used for visibility-graph planning, (2) closures of graph edges/paths by obstacles, or another specific representation. These lead to different algorithms and should not be silently conflated.

### Objective E — Minimal behavior planning and vehicle control

Implement only the behavior decisions required to execute a safe baseline demonstration—for example: follow route, approach an obstacle, select a valid local path or stop, and handle invalid or unavailable planning inputs. Keep behavior decisions separate from geometric path search and vehicle actuation.

Track the resulting `Trajectory` using one understandable lateral control baseline (for example, Pure Pursuit or Stanley) and one longitudinal speed controller (for example, PID). Keep the control interface replaceable for later MPC or advanced vehicle dynamics work.

**Success evidence:** end-to-end CARLA motion with measured tracking error, command limits, stable stopping/fallback, and reproducible scenarios.

### Objective F — Quantitative integration and evaluation

Evaluate the entire perception → navigation → local planning → control chain with repeatable routes and explicit diagnostics. At minimum measure:

- route completion and navigation failures;
- local reference/association availability and geometry errors;
- localization error relative to controlled CARLA ground truth, once localization is added;
- collision or obstacle-clearance outcomes;
- trajectory lateral/heading tracking error, progress and speed error;
- processing latency, input freshness, and invalid-state handling.

Distinguish **built**, **unit-tested**, **runtime-tested**, and **measured in closed loop**. Do not label modules complete solely because messages compile or visualizations look plausible.

## 3. Target architecture

```text
CARLA sensors ──→ Perception / lane tracking ──→ local lane observations
                                                    │
Noisy GPS ────┐                                     │
Odometry ────┼────→ Localization ◄───────────────────┘
Map geometry ┘           │
                         └──→ estimated ego pose + uncertainty
                                      │
OpenDRIVE topology ──→ global route graph + A* ──→ RoutePlan
                                      │                 │
                    local lane geometry / LaneMap ───────┤
                                                        ↓
                                       RoutePlan ↔ LaneMap association
                                                        ↓
                                          Local route/reference corridor
                                                        ↓
                                              Minimal behavior policy
                                                        ↓
Obstacle representation (closed graph paths, TBD) ─→ Local geometric planner
                                                        ↓
                                          Feasible Trajectory + speed
                                                        ↓
                                           Lateral + speed control
                                                        ↓
                                                    CARLA vehicle
```

**Frame and information-flow caveat:** The diagram is conceptual. Its implementation must clarify local versus world-frame lane observations and avoid feedback loops that count an odometry-derived map as independent localization evidence. The route planner should use the best available estimated pose once localization is validated.

## 4. Scope and priority under the compressed schedule

**Target:** a reproducible, measured **baseline closed-loop demonstration around mid-November 2026**, subject to actual integration results. This is a scope target, not evidence that the remaining work is complete.

**Critical path for the baseline:**

1. Finish RoutePlan ↔ LaneMap association, including frame/quality contracts.
2. Add minimal behavior and a simple local geometric planning baseline.
3. Represent and test basic obstacle constraints with the agreed closed-graph-path interpretation.
4. Produce continuous local trajectories and follow them with one controller.
5. Run repeatable end-to-end CARLA tests and document limitations.

**Localization commitment:** Preserve the localization interface and non-privileged measurement boundary now. Implement perception-assisted noisy-GPS localization as an explicit **subsequent milestone or stretch goal**, unless the supervisor requires it within the November demonstration; doing both localization fusion and full closed-loop obstacle planning inside the compressed window would raise integration risk.

**Deliberately deferred:** sophisticated dynamic-object prediction, multi-agent negotiation, full SLAM, HD mapping infrastructure, large behavior planners, MPC/nonlinear trajectory optimization, and production robustness. These can be layered onto the baseline later.

## 5. Completion criteria

The **November baseline** can be considered finished when a defined set of CARLA scenarios demonstrates:

- destination → valid `RoutePlan`;
- route ↔ perceived-lane association with explicit confidence and failure states;
- a local path/trajectory constrained by road geometry and at least the agreed basic obstacle model;
- trajectory-following control and a safe stop/no-valid-path response;
- reproducible setup, diagnostics, numerical evaluation, and documented limitations.

The **extended research version** additionally demonstrates that a noisy GPS estimate is improved through genuine independent local perception/map evidence, with uncertainty estimates and quantitative comparison against CARLA evaluation ground truth.

These are separate acceptance levels: do not describe the extended localization objective as already achieved or mandatory for declaring the tightly scoped November baseline finished unless that scope is explicitly changed.

## 6. Decisions to confirm before implementation

1. What exactly does **“obstacles as closed graph paths”** mean in the supervisor's intended planning method?
2. Which noisy GPS representation and rate will be used, and what odometry/IMU signals are available?
3. Can localization consume ego-relative lane detections before they are transformed into the world-frame `LaneMap`?
4. What positional accuracy/error reduction would count as a successful localization improvement?
5. Is perception-assisted localization required for the mid-November demonstration, or is it a post-baseline research milestone?

## 7. Engineering principles retained

- Modular ROS 2 packages and explicit versioned interfaces.
- CARLA/OpenDRIVE as an initial topology and evaluation resource, never silently presented as perception output.
- Geometry/frame conventions checked in runtime, including curvature sign and handedness.
- Uncertainty, provenance, input age, and invalid results represented explicitly.
- Small changes and incremental experiments; commit reusable tests, not one-off probes, logs, or build artifacts.
- Algorithmic simplicity first, with clean replacement points for advanced planners, localization, and control.
