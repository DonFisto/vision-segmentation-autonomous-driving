# Current Project State

Last synchronized: 2026-09-16. Source checkpoint: `7b54639` on `feature/route-lane-association`, also the completed `feature/global-route-planning` checkpoint. The branch had no additional implementation commits and a clean worktree before this documentation pass.

## Current milestone

Route-ready lane mapping and global route planning are implemented and development-runtime validated. RoutePlan ↔ LaneMap association is NEXT and absent. This is an educational modular prototype, not a completed route-to-actuation loop.

Evidence precedence: current source → Git history → checked-in diagnostics/material → runtime observations supplied for this refresh → older documentation. No CARLA run, inference, build, or implementation tests were performed during this pass. Runtime observations are supplied evidence, not newly reproduced results. See the [dated report](reports/2026-09-16-routing-documentation-refresh.md).

## Subsystem status

Source paths below are relative to `ros/ros2_ws/src/`; exact interfaces are in [ARCHITECTURE.md](ARCHITECTURE.md).

| Subsystem | Status | Evidence / boundary |
| --- | --- | --- |
| CARLA bridge / hero odometry | Implemented | `carla_bridge_node/carla_bridge_node/carla_bridge_node.py`: direct world pose, independent odometry timer |
| Semantic segmentation, object extraction/tracking | Implemented; parallel capability | `semantic_seg_node`, `object_detection_node`, `tracking_node`; earlier demos and milestones |
| Monocular depth, fusion, free space | Implemented; parallel capability | `depth_node`, `fusion_node`, `free_space_node`; relative depth, not calibrated metric depth |
| Local occupancy / accumulated mapping | Implemented; limited parallel capability | `local_occupancy_node`, `local_mapping_node`; historical spatial stack, not full SLAM |
| Lane geometry pipeline | Implemented | `road_marking_seg_node`, `lane_geometry_node`: BEV, component/context filters, quadratic fitting |
| Temporal lane tracking | Implemented | `lane_geometry_node/lane_geometry_node/lane_tracking_node.py`: confirmation, quality/geometric gates, held/inferred/missing states |
| Odometry propagation | Implemented | Tracker transforms/refits prior curves using buffered hero poses |
| Rolling vector LaneMap | Implemented | `lane_reasoning_nodes/lane_reasoning_nodes/tracked_lane_mapping_node.py`: schema 2, zero or one local segment, direct world positions/headings |
| OpenDRIVE coarse topology | Implemented | `global_route_planner/global_route_planner/topology.py`, `carla_topology_adapter.py` |
| Sampled directed routing graph | Implemented | `routing_graph.py`, `carla_routing_graph_adapter.py`: lane-follow and legal lateral transitions |
| Dijkstra baseline / oracle | Implemented | `routing.py`, comparison checks in routing diagnostics |
| A* production search | Implemented | `routing.py`, `GlobalRoutePlannerNode._publish_route` |
| Ego/goal route association | Implemented | `carla_route_association.py`, `routing_graph_index.py`; world position → driving lane → sampled node; not RoutePlan ↔ LaneMap association |
| RoutePlan and invalid-route publication | Implemented | `global_route_planner_node.py`, `route_plan_builder.py`; `STATUS_VALID` and `STATUS_INVALID` |
| Planning/Foxglove visualization | Implemented | Four adapters in `planning_visualization`; distinct production and display representations |
| Reproducible startup tooling | Implemented | [Startup package](../../autonomous_driving_startup/README.md): local tmux, remote processes, core/full/stop/status |
| Earlier lane reasoning/guidance | Partial / prototype | `lane_projection_node.py`, `lane_mapping_node.py`, `lane_guidance_node.py`; not the active tracked mapper or a behavior planner |
| Reactive navigation / manual actuation | Implemented; limited parallel capability | Earlier navigation nodes and `carla_control_node`; not trajectory tracking for RoutePlan |
| RoutePlan ↔ LaneMap association | Absent; NEXT | No combined consumer or association output implementation at this checkpoint |
| Behavior / maneuver planner | Absent | Global lane-change edges do not implement dynamic maneuver decisions |
| Local trajectory planner | Absent | `Trajectory.msg` and `TrajectoryPoint.msg` exist as schemas only |
| Trajectory tracking for this planning stack | Absent | No new-pipeline trajectory consumer/controller |
| Closed-loop quantitative evaluation for this pipeline | Absent | Supplied evidence covers component/integration diagnostics, not route following |
| Localization independent of CARLA / full SLAM | Absent / deferred | Simulator odometry remains the accepted simplification |

## Validated checkpoint and limits

- LaneMap schema 2 and direct `carla_world` alignment were observed. The [lane milestone](../milestones/lane_perception_tracking_mapping.md) records the direct-versus-flipped probe; empty segments are legitimate when support is insufficient.
- Town10HD development routing observed 3066 nodes, 4522 edges, one strongly connected component, and valid routes with varying pose counts. These are scenario observations, not map invariants. The [routing milestone](../milestones/global_route_planning.md) records the evidence and limits.
- A supplied clean `stop` → `full` restart restored one instance of each of the five planning/visualization nodes. Route output requires a goal after restart; no destination is automatically published.
- Routing is static after startup, without dynamic-obstacle costs, prediction, goal-orientation constraints, or trajectory dynamics checks. RoutePlan confidence is a heuristic, not a probability. `STATUS_PARTIAL` and `STATUS_BLOCKED` are schema constants without active planner behavior.
- Global route segment IDs and map revisions are unrelated to local LaneMap IDs and revisions. Both world geometries use direct CARLA coordinates; future association must not add another y flip.

## Remaining evidence gaps

No raw bag/probe files accompany the supplied September observations. They do not establish universal lane accuracy, calibrated confidence, full 3D alignment, curvature-sign agreement, or performance rates. In source, LaneMap curvature is computed in local forward-left coordinates while its poses/headings are converted to direct world coordinates; a downstream scalar-sign contract remains to be validated. The remote `ros2depth` environment has no separate checked-in lock/export. Developer Level 3 mastery is not established by code authorship.

Next: [define geometric association semantics, then implement and runtime-validate a separate layer](ROADMAP.md) before local trajectory planning and control.
