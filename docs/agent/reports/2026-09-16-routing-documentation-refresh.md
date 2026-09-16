# Routing Documentation Refresh — 2026-09-16

## Scope

- **Review type:** bounded documentation-only synchronization, not a new full implementation-quality audit or an association design/implementation task.
- **Compared against:** current `feature/route-lane-association` source at `7b54639`, recent Git history, checked-in diagnostic code and historical material, then runtime observations explicitly supplied for this refresh.
- **Initial repository checks:** `git branch --show-current` returned `feature/route-lane-association`; `git status --short` was empty; `git log --oneline --decorate -30` showed HEAD at the same routing/startup checkpoint as `feature/global-route-planning` and their tracked remote refs. No extra route-lane association implementation commits were present.
- **Instructions read:** root `AGENTS.md`, `PROJECT_CONTEXT.md`, `REVIEW_PLAYBOOK.md`, `REPORT_TEMPLATE.md`, root README, and documentation index. Protected instructions/templates were not modified.
- **Runtime checks performed:** none. No CARLA launch, inference, remote restart, ROS build, or implementation tests were needed for this documentation scope.
- **Important evidence unavailable:** raw bags/probe scripts and full logs for the supplied September runtime observations; a separately exported current remote `ros2depth` environment; quantitative route-following evaluation for the future pipeline.

## Executive assessment

The initial July agent state and public overview stopped at accumulated local mapping, while source had advanced through route-ready LaneMap, global OpenDRIVE routing, frame correction, visualization, and reproducible startup. Some short commit summaries also hid substantial additions: `326eaff` added the tracked vector mapper/interface groundwork, while `df913d5` completed the remaining message and stamped tracking integration.

Lane perception/tracking/mapping and global route planning are implemented. The supplied runtime evidence supports the bounded development integration checkpoint. RoutePlan ↔ LaneMap association is NEXT and absent; behavior planning, local trajectory planning, and tracking/control for this pipeline remain future work. Confidence is high in these source boundaries; runtime conclusions are limited to the supplied observations and were not independently reproduced here.

## Evidence inspected

| Area | Source/history/material |
| --- | --- |
| Shared interfaces | `ros/ros2_ws/src/autonomy_interfaces/msg/`: LaneSample, LaneBoundary, LaneSegment, LaneMap, RoutePlan, TrajectoryPoint, Trajectory; package manifest |
| Lane stack | `road_marking_seg_node`, all five processing/tracking modules in `lane_geometry_node`, tracked mapper plus earlier projection/mapping/guidance in `lane_reasoning_nodes`; package/setup declarations and exact `05_lanes.sh` chain |
| Global routing | `topology.py`, `carla_topology_adapter.py`, `routing_graph.py`, `carla_routing_graph_adapter.py`, `routing.py`, `carla_route_association.py`, `routing_graph_index.py`, `global_segment_ids.py`, `route_plan_builder.py`, `global_route_planner_node.py`, `setup.py`, diagnostic code and package manifest |
| Planning visualization | Route visualizer, odometry TF broadcaster, global graph visualizer, Foxglove world adapter, package/entry-point files, workspace Foxglove config |
| Operations/setup | Startup README and `00_stack.sh` through `07_foxglove.sh`, especially `05_lanes.sh`, `06_planning.sh`, `lib/common.sh`; `environment.yml`, model loading source, bridge odometry source |
| Historical evidence | Existing milestone interfaces, limitations, validation sections, parameter tuning history, timeline and all current agent documents; test inventory consists largely of package lint scaffolding plus routing executable diagnostics |
| Git history | Recent log through lane development; `df913d5`, `326eaff`, `89d8561`, `cbcea54`, `1d6186b`, `1cf9d60`, `ce97421`, `7b54639` examined via commit metadata/diffs and corresponding current source; intervening topology/search/association commits traced |

Repository searches for RoutePlan/LaneMap/Trajectory producers, consumers, and entry points support the absence of a combined route-lane consumer or trajectory planner/controller. `carla_route_association.py` associates ego/goal positions with global graph nodes, not RoutePlan with LaneMap.

## Architecture and subsystem status

```text
RGB + calibration -> lane geometry -> tracking + odometry -> local LaneMap
OpenDRIVE -> topology -> sampled graph -> ego/goal association -> A* -> RoutePlan
LaneMap + RoutePlan -> route-lane association [NEXT / absent]
                   -> local trajectory planning -> control [FUTURE / absent]
```

[CURRENT_STATE.md](../CURRENT_STATE.md) records implemented versus partial/absent subsystems. [ARCHITECTURE.md](../ARCHITECTURE.md) provides the verified producer/topic/type/frame/consumer registry and timing/units. The old object/depth/free-space/occupancy and reactive-navigation capabilities remain implemented parallel work, not replacements for the new planning/control boundary.

## What is working well / settled contracts

1. Perception quality and temporal state remain explicit; the mapper does not reinsert held/inferred geometry as direct evidence. It can publish no segment and currently emits at most one local segment with unknown topology/speed-limit semantics.
2. Global routing owns an explicit graph/search pipeline, legal edge types, deterministic topology provenance, and a Dijkstra comparison baseline. The live node uses A*, not CARLA's black-box GlobalRoutePlanner.
3. A completed no-path search publishes INVALID with empty geometry; route QoS is reliable/transient-local/depth 1. Rejected goals and failed ego association are distinct behaviors and do not universally invalidate the last route.
4. Public LaneMap positions/headings and RoutePlan use direct `carla_world`; visualization uses copied/reflected `carla_world_viz` data and `hero_viz` local aliases. Global/local IDs and revisions cannot be compared by equality.
5. The startup scripts expose a reusable core workflow with local tmux and remote processes, five planning/visualization nodes, and explicit operator goal input.

## Supplied runtime observations incorporated

These values came from the task's supplied development validation evidence, not new measurements or assumed test passes:

| Observation | Documentation / qualification |
| --- | --- |
| LaneMap schema 2; direct-vs-flipped lane/route errors 0.191/2.949/3.519/3.603 m versus 49.720/51.123/50.699/53.340 m (min/mean/median/max) | [Lane milestone](../../milestones/lane_perception_tracking_mapping.md); one frame diagnostic, not universal accuracy thresholds |
| Hero-to-lane 4.995 m direct versus 50.061 m flipped; `DIRECT_CONVENTIONS_MATCH` | Positional evidence against another downstream y flip; not a completed association layer or signed-curvature test |
| Town10HD/Town10HD_Opt graph: 3066 nodes, 4522 edges, approximately 3098 follow / 712 left-change / 712 right-change, SCC count 1; revision 6632468842588822852 | [Routing milestone](../../milestones/global_route_planning.md); development scenario values, not universal map invariants |
| Development goal `(109.254, 89.835, 0.0)` in `carla_world`; valid routes with e.g. 128 and 323 poses | Explicitly labeled test goal in runbook/milestone; no fixed pose-count assertion |
| Route frame `carla_world`, 323 poses, 6196 graph points; route-to-graph min/mean/median/max 0.0000 m | Supports routing-geometry provenance; raw probe method not supplied, marker z is raised in source, no 3D-equality or dynamics claim |
| RoutePlan status 1; route Path `carla_world`, display Path `carla_world_viz` sharing the source timestamp; tracked centerline alias `hero_viz` | Display frame/timestamp integration evidence; no planner-frame change |
| Clean local `stop` then `full` restored one each of the five planning/visualization nodes; specified planning and six local display topics observed | One restart validation; not a universal SSH/process-cleanup guarantee or a new restart performed here |

## Findings and prioritized next work

**Conceptual boundaries:** no architecture replacement is needed for this synchronization. Preserve independent perception and privileged global topology. Branch naming and trajectory schema presence must not imply implemented association or control.

**Required now:** documentation drift was corrected. Next development work is (1) define/validate geometric association semantics, then (2) implement and runtime-validate a separate association layer. [ROADMAP.md](../ROADMAP.md) records benefits, dependencies, effort, acceptance, learning value, and displaced work. The expected reference/corridor interface, `s`/`d` sign policy, coverage gates, hysteresis, and no-association behavior are planned concepts, not finalized schemas.

**Useful before prototype completion:** after association, add a simple local reference/trajectory planner, baseline tracking/control, then systematic closed-loop evaluation. **Valuable later:** a selective planning/optimization/control depth experiment. **Not justified now:** premature MPC, production map/deployment infrastructure, or unrelated package refactors that displace the active integration tasks.

## Evaluation plan and developer understanding

Current evidence can assess frame alignment, graph/path provenance, route status behavior, and process/topic restoration within the recorded scenarios. Future association needs lateral/heading discrepancy, overlap, confidence/availability, discontinuity, and latency measurements. Completion, collisions, lateral/speed tracking error, and control smoothness require the future closed loop; no targets/results are invented here.

[LEARNING_LOG.md](../LEARNING_LOG.md) appends engineering evidence around temporal state, ego motion, graph/search/map association, QoS, frames, and incremental runtime validation. Target Level 2 across the stack; independent Level 3 understanding is not established by implementation authorship. No user decision was needed to complete this documentation scope. Association interface/design choices remain for its planned milestone.

## Remaining uncertainty

- LaneMap poses/headings are converted to direct CARLA world, but scalar curvature is computed in local forward-left coordinates and assigned unchanged. The supplied positional probe does not validate downstream signed-curvature semantics. This is documented without modifying implementation.
- The graph display has a 0.05 m z offset; 6196 points numerically match lane-follow endpoints. Without the raw probe, its dimension/selection/interpolation method cannot be certified. Likewise, planar LaneMap z=0 does not establish full 3D alignment.
- There is no separately checked-in current `ros2depth` export or lane checkpoint checksum; scripts establish environment roles and artifact selection rules, not a portable exact installation snapshot.
- Goal orientation, dynamic obstacles/prediction, trajectory feasibility, and heuristic confidence limitations remain explicit. No all-scenario accuracy, latency, or new-route closed-loop validation is claimed.
- Topic waits check discovery; local session status is not process/message health. The supplied restart does not prove cleanup for every remote failure mode.

## Validation record

- Initial branch/status/log commands passed the requested clean-start check.
- Documentation validation: `git diff --check` passed; the requested stale-language search returned no matches; the requested key-term search confirmed the current vocabulary across the updated documentation. A local, untracked Python link check resolved all 123 relative Markdown file/directory links in 17 documents. Scope checks confirmed 14 modified existing documents, exactly 3 new documents, append-only decision/learning logs, and byte-for-byte preservation of protected instructions/templates and older milestones. No implementation tests were created or run.
- No runtime checks, expensive inference, CARLA, builds, commits, or pushes were performed by this pass. Historical milestone files and ADR-001..003 were preserved.
- Two untracked screenshot files appeared under `assets/` after the initial clean check. They were not created, edited, or removed by this documentation pass and are excluded from its scope.

## State updates

Updated existing documents: root `README.md`; `docs/README.md`, `docs/setup.md`, `docs/ros/runbook.md`, `docs/timeline.md`; agent `README.md`, `PROJECT_CONTEXT.md`, `CURRENT_STATE.md`, `ARCHITECTURE.md`, `ROADMAP.md`, `DECISION_LOG.md`, `LEARNING_LOG.md`; `autonomous_driving_startup/README.md`; and the reports index.

Created exactly the two requested milestones, `docs/milestones/lane_perception_tracking_mapping.md` and `docs/milestones/global_route_planning.md`, plus this dated report. ADR-004..010 and three learning entries were appended. No implementation, messages, configuration, launch scripts, or package structure changed.
