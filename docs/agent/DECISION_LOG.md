# Decision Log

Use this file for durable project decisions. Append new entries; do not rewrite earlier decisions merely because the design evolves. When a decision is superseded, link the new entry and mark the old one accordingly.

## Entry template

### ADR-XXX — Decision title

- **Date:** YYYY-MM-DD
- **Status:** proposed / accepted / superseded / rejected
- **Context:** What problem or constraint required a decision?
- **Options considered:**
  1. Option and trade-offs.
  2. Option and trade-offs.
- **Decision:** What was chosen?
- **Rationale:** Why is it the best choice for the current educational objective and constraints?
- **Consequences:** What becomes easier, harder, or deferred?
- **Validation:** How will the decision be tested?
- **Supersedes / superseded by:** ADR reference, when applicable.

---

## ADR-001 — Keep the first prototype deliberately simple

- **Date:** 2026-07-30
- **Status:** accepted
- **Context:** The repository is intended to teach the complete autonomous-driving stack while the developer also manages a double degree, Formula Student, and potential professional work.
- **Options considered:**
  1. Attempt production-like breadth and infrastructure.
  2. Build the simplest coherent closed loop, then deepen one subsystem.
- **Decision:** Build a simple, modular, measurable closed-loop prototype before adding advanced planning, control, localization, or infrastructure.
- **Rationale:** This maximizes conceptual coverage and learning per unit of limited time while preserving a path toward deeper work.
- **Consequences:** Simulator odometry, basic planning, and basic controllers are acceptable. Full SLAM, HD maps, complex prediction, and production deployment are deferred.
- **Validation:** The first system milestone must complete repeatable CARLA scenarios and expose measurable perception-to-actuation behavior.

## ADR-002 — Use CARLA odometry for the first planning-and-control milestone

- **Date:** 2026-07-30
- **Status:** accepted provisionally
- **Context:** The current accumulated mapping stage uses hero odometry, while independent ego-motion estimation is a future direction.
- **Options considered:**
  1. Block planning and control until visual odometry or SLAM is implemented.
  2. Continue using simulator-provided odometry for the first closed loop.
- **Decision:** Keep CARLA odometry as an accepted simplification unless the first full review finds an unusable interface or inconsistency.
- **Rationale:** Replacing odometry is not on the critical path to learning planning and control.
- **Consequences:** Localization realism remains limited, but planning, control, interfaces, and evaluation can progress.
- **Validation:** Confirm frame, timestamp, and transform consistency across odometry, occupancy, mapping, planner, and controller.

## ADR-003 — Target engineering understanding across the stack

- **Date:** 2026-07-30
- **Status:** accepted
- **Context:** AI has materially assisted implementation, and independently rewriting every component would be incompatible with the project's breadth and available time.
- **Options considered:**
  1. Require independent reimplementation of all components.
  2. Require operational understanding only.
  3. Require engineering understanding across the stack and reimplementation depth in selected areas.
- **Decision:** Target Level 2 engineering understanding across all major components and Level 3 reimplementation depth only in one or two high-value areas.
- **Rationale:** This supports responsible ownership of the code without turning every dependency into a separate course.
- **Consequences:** Planning, optimization, or control are likely depth targets; adapters and mature external libraries may remain at Level 1 or Level 2.
- **Validation:** Use targeted understanding questions and focused exercises during project reviews.

## ADR-004 — Separate perceived local lanes from privileged global topology

- **Date:** 2026-09-16 (recorded retrospectively from the completed milestones)
- **Status:** accepted
- **Context / alternatives:** A single simulator-derived map would simplify geometry but hide perception uncertainty; separate representations preserve what each subsystem actually knows.
- **Decision:** `TrackedLaneMappingNode` owns local perceived LaneMap; CARLA/OpenDRIVE supplies privileged global routing topology. RoutePlan ↔ LaneMap association will be a separate layer, not a change to perception or topology generation.
- **Rationale / consequences:** The educational prototype can use simulator topology without presenting it as measured local geometry. Empty/partial LaneMap remains meaningful, especially at intersections; no route-driven hallucination of boundaries.
- **Evidence / validation:** [Lane milestone](../milestones/lane_perception_tracking_mapping.md), `_direct_side` and `_build_vector_map`; global planner has no LaneMap subscription. Runtime evidence is scoped in the milestone records.

## ADR-005 — Own an understandable graph and search pipeline

- **Date:** 2026-09-16 (retrospective)
- **Status:** accepted
- **Context / alternatives:** A CARLA black-box planner would reduce implementation work; explicit graph structures expose connectivity, legal transitions, costs, and failure modes.
- **Decision:** Use project-owned coarse topology, sampled directed routing graph, Dijkstra baseline, and A* production search with CARLA-specific adapters.
- **Rationale / consequences:** Algorithms can be inspected independently of ROS/CARLA, at the cost of maintaining sampling and map-association assumptions. Avoid expanding into industrial map infrastructure.
- **Evidence / validation:** `ff23221` through `a32983f`; [routing milestone](../milestones/global_route_planning.md), graph diagnostics compare A*/Dijkstra costs and continuity. Inspection is not a claim that diagnostics were rerun during this refresh.

## ADR-006 — Global route intent is separate from local trajectory feasibility

- **Date:** 2026-09-16 (retrospective)
- **Status:** accepted
- **Context / alternatives:** Feeding a nominal map polyline directly to control would blur route intent, maneuver decisions, and vehicle dynamics.
- **Decision:** RoutePlan carries global topology provenance and a coarse reference path. Local behavior/trajectory planning and tracking remain separate future responsibilities.
- **Rationale / consequences:** A valid route does not establish collision freedom or dynamic feasibility. Static lane-change graph edges do not execute a behavior maneuver. `STATUS_PARTIAL`/`STATUS_BLOCKED` remain schema constants, not implemented dynamic-obstacle semantics.
- **Evidence / validation:** `route_plan_builder.py`, `routing_graph.py`, [shared interfaces](../../ros/ros2_ws/src/autonomy_interfaces/msg); runtime path-to-graph agreement validates provenance only.

## ADR-007 — Publish production world geometry in direct CARLA convention

- **Date:** 2026-09-16 (records the 2026-09-15 correction)
- **Status:** accepted
- **Context / alternatives:** Tracker/mapper internals use forward-left geometry and signed odometry conversions. Leaving mirrored coordinates under the public `carla_world` label required downstream workarounds.
- **Decision:** Convert published LaneMap world positions/headings back to direct `carla_world`, matching RoutePlan and hero pose. Future route-lane association must not apply an additional y flip.
- **Rationale / consequences:** Conversion belongs at the producer boundary; internal `odom_y_sign=-1`, `odom_yaw_sign=-1` remain. Scalar curvature is still computed in local forward-left coordinates; this ADR does not claim its downstream sign was validated.
- **Evidence / validation:** `cbcea54`, `_internal_world_to_carla_world`, `_internal_world_yaw_to_carla_world`; supplied `DIRECT_CONVENTIONS_MATCH` positional probe in the [lane milestone](../milestones/lane_perception_tracking_mapping.md).

## ADR-008 — Keep Foxglove reflection in display-only copies

- **Date:** 2026-09-16
- **Status:** accepted
- **Context / alternatives:** Changing production coordinates for visual convention would couple planner semantics to display preferences.
- **Decision:** Use `carla_world_viz → hero_viz` and display aliases. Reflect global y/orientation basis, but only reattach already forward-left local lane paths to `hero_viz` without mirroring their points.
- **Rationale / consequences:** Foxglove can use conventional display coordinates without mutating RoutePlan/LaneMap/odometry. Display frames are not planner frames or a rigid reflection TF between worlds.
- **Evidence / validation:** `ce97421`, `FoxgloveWorldVisualizerNode`; supplied matching route/path-viz timestamps and `hero_viz` local-path observation. [Frame registry](ARCHITECTURE.md).

## ADR-009 — Keep global and local ID/revision namespaces independent

- **Date:** 2026-09-16 (retrospective)
- **Status:** accepted
- **Context / alternatives:** Both messages use numeric segment IDs and map revisions; equality would incorrectly suggest shared identity.
- **Decision:** Global RoutePlan IDs derive from OpenDRIVE topology-edge provenance and global map fingerprint. LaneMap IDs identify its local representation, and its revision counts observation changes. Never associate them by numeric equality.
- **Rationale / consequences:** Association must use geometry, timing, confidence, and explicit future semantics. Global map revision is not a local-map revision or a routing-parameter version.
- **Evidence / validation:** `b161709`, `global_segment_ids.py`, `stable_map_revision`, mapper `_build_vector_map`; exact hash construction in the [routing milestone](../milestones/global_route_planning.md).

## ADR-010 — Require bounded runtime evidence for integration completion

- **Date:** 2026-09-16
- **Status:** accepted
- **Context / alternatives:** Source presence, generated schemas, or plausible displays alone do not show that a ROS boundary works at runtime.
- **Decision:** Record source verification separately from runtime validation and preserve dated evidence when closing a milestone. Use reusable diagnostics and startup tooling to support the next subsystem.
- **Rationale / consequences:** Observed graph counts, frame errors, timestamps, and clean restarts are scenario evidence, not universal guarantees. Future association and closed-loop completion need their own runtime records.
- **Evidence / validation:** `4541c0c`, `7b54639`, supplied probes/restart; [2026-09-16 refresh report](reports/2026-09-16-routing-documentation-refresh.md). No new runtime validation was performed by the documentation pass.
