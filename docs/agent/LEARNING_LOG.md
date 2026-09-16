# Learning Log

Use this file to record durable understanding gained from the project. This is not a daily diary and should not duplicate the development timeline.

Append entries when a concept becomes clearer, an assumption is disproved, or a subsystem reaches a new understanding level.

## Entry template

### YYYY-MM-DD — Topic

- **Component:**
- **Previous understanding level:** 1 / 2 / 3
- **Current understanding level:** 1 / 2 / 3
- **What became clear:**
- **Evidence:** code change, experiment, explanation, derivation, or debugging result
- **Remaining confusion:**
- **Next exercise:**
- **Connection to mathematics or systems coursework:**

---

## Initial learning targets

These are hypotheses to validate during the first full review, not claims about current understanding.

| Area | Initial target | Why it matters | Suggested evidence of understanding |
| --- | --- | --- | --- |
| Semantic segmentation | Level 2 | Current perception foundation | Explain dataset labels, model choice, preprocessing, metrics, domain shift, and failure cases |
| Depth and fusion | Level 2 | Bridges image output to geometry | Explain relative versus metric depth, calibration assumptions, synchronization, and fusion limitations |
| Tracking | Level 2 | Introduces temporal state and association | Explain state model, data association, occlusion behavior, and stale tracks |
| Occupancy and mapping | Level 2 | Direct input candidate for planning | Explain projection, frames, grid resolution, accumulation, static/dynamic semantics, and drift |
| Planning | Level 3 candidate | Strong connection to algorithms and optimization | Implement and justify a simplified planner; define costs, constraints, and failure modes |
| Control | Level 3 candidate | Strong connection to ODEs, numerics, and optimization | Derive or reimplement a baseline controller and compare it experimentally |
| ROS2 integration | Level 2 | Essential production-style systems skill | Trace topics, timing, parameters, launch behavior, and debugging workflow |
| CARLA actuation | Level 1–2 | Simulator-specific adapter | Explain scaling, saturation, command ownership, and feedback loop |

## Review questions to revisit

1. Can every module's inputs, outputs, frames, units, and consumers be described without reading the implementation?
2. Can the dominant failure mode of every major subsystem be predicted?
3. Can a component be modified without asking AI to regenerate the surrounding architecture?
4. Which part of the system can currently be reimplemented in simplified form?
5. Which mathematical idea became tangible because of a real debugging or design problem?

## 2026-09-16 — Lane tracking, uncertainty, and ego motion

- **Component:** BEV lane geometry, `LaneTrackingNode`, `TrackedLaneMappingNode`.
- **Understanding evidence:** The implemented integration distinguishes accepted/rejected/missing measurements from held/inferred outputs and separately reports motion propagation. Prior curves are sampled, transformed into the new ego frame, and refit; width has independent confidence. The mapper accepts direct observations and prevents memory from being reinserted as fresh evidence.
- **What became clear:** Temporal smoothness alone is not fresh perception. Confidence must account for measurement support, age, and consistency; missing markings at intersections can be correct.
- **Evidence:** `61ca67a`, `326eaff`, `df913d5`; [lane milestone](../milestones/lane_perception_tracking_mapping.md).
- **Level:** These code and diagnostic artifacts demonstrate engineering work relevant to Level 2. Previous/personal current understanding is not independently assessed here; no Level 3 mastery claim follows from authorship.
- **Remaining gap / next exercise:** Explain one propagated-then-held frame and one rejected adjacent-lane candidate from their status outputs; identify why neither should add direct map evidence.
- **Coursework connection:** Rigid transforms, polynomial approximation, numerical differentiation, temporal state, and confidence modeling.

## 2026-09-16 — Graph representations, shortest paths, and map association

- **Component:** `global_route_planner`.
- **Understanding evidence:** Coarse topology and sampled graph have separate responsibilities. Explicit `LANE_FOLLOW`, `LANE_CHANGE_LEFT`, and `LANE_CHANGE_RIGHT` edges expose legal connectivity and penalty costs. Position association first resolves a semantic lane, then a sampled OpenDRIVE-s node.
- **What became clear:** Dijkstra provides a cost baseline; A* uses a lower bound to guide search. Map-legal transitions and minimum graph cost do not imply feasible motion or collision-free behavior. Global topology provenance is not local LaneMap identity.
- **Evidence:** `e261432`, `a651b63`, `de9dfa5`, `b161709`, `a32983f`; checked-in diagnostic comparisons and supplied Town10HD runtime observations in the [routing milestone](../milestones/global_route_planning.md).
- **Level:** Level 2 engineering target supported by implementation/debugging artifacts; independent Level 3 reimplementation remains unassessed.
- **Remaining gap / next exercise:** Justify the Euclidean heuristic using the edge-cost definition, compare a route's summed cost with both searches, and explain a no-directed-path result without conflating it with failed pose association.
- **Coursework connection:** Directed graphs, heap-based search, admissibility, spatial projection, and indexing.

## 2026-09-16 — ROS durability, frame contracts, and incremental validation

- **Component:** RoutePlan publication, lane/world conversion, planning visualization, startup tooling.
- **Understanding evidence:** Reliable transient-local route outputs support compatible late subscribers; INVALID publication clears obsolete geometry after a no-path search. The frame correction restores direct production coordinates, while Foxglove receives reflected copies and reattached local paths. Startup separates local tmux/SSH orchestration from remote process execution.
- **What became clear:** A frame label cannot repair handedness; a display correction must not leak into planning. A topic's existence is not a message, a durable route is not a fresh ego observation, and an unbounded `echo --once` can wait forever before a goal is supplied.
- **Evidence:** `89d8561`, `cbcea54`, `1d6186b`, `1cf9d60`, `ce97421`, `7b54639`; supplied direct/flip probe, source timestamp equality, and clean restart observations. These were not rerun during this refresh.
- **Level:** Level 2 integration/debugging evidence; personal mastery and Level 3 depth are not established by this documentation review.
- **Remaining gap / next exercise:** Trace one source timestamp through route visualization, distinguish stale odometry from an empty LaneMap, and explain scalar curvature's current local sign convention before using it downstream.
- **Coursework connection:** Coordinate basis changes, distributed message state, clocks, and reproducible experiments.
