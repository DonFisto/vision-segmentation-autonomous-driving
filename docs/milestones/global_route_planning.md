# Global Route Planning Milestone

Recorded 2026-09-16 against `7b54639`. Global routing is implemented and development-runtime validated. `feature/route-lane-association` starts from this routing/startup checkpoint; the association of RoutePlan with perceived LaneMap is not implemented.

Implementation claims below come from source/history. Runtime observations are supplied development evidence for this documentation refresh, not measurements reproduced during the pass. No universal map counts, latency claims, or trajectory feasibility claims follow from those observations.

## Purpose and decomposition

The planner provides global intent and nominal map geometry using privileged CARLA/OpenDRIVE topology. It deliberately implements understandable project-owned graph structures and search instead of simply wrapping CARLA's `GlobalRoutePlanner` as a black box. It owns neither lane perception nor local motion planning.

```text
CARLA map.get_topology() / OpenDRIVE
  -> coarse directed topology with sampled edge geometry
  -> sampled lane-level routing graph
  -> legal lane-follow / lane-change transitions
  -> ego and goal position association
  -> A* search (Dijkstra baseline/oracle)
  -> autonomy_interfaces/msg/RoutePlan
```

Sources live in [global_route_planner/global_route_planner](../../ros/ros2_ws/src/global_route_planner/global_route_planner). [setup.py](../../ros/ros2_ws/src/global_route_planner/setup.py) registers `global_route_planner_node` and five diagnostics: `topology_diagnostic_node`, `routing_diagnostic_node`, `routing_graph_diagnostic_node`, `route_association_diagnostic_node`, `live_route_diagnostic_node`.

## Topology, routing graph, and costs

[topology.py](../../ros/ros2_ws/src/global_route_planner/global_route_planner/topology.py) defines `DirectedTopologyGraph`, nodes, edges, and sampled `TopologyPoint` geometry without ROS/CARLA dependencies. Endpoint identity is `(road_id, section_id, lane_id, s_cm)` with OpenDRIVE `s` rounded to centimetres. Edges retain directed endpoint identity, sampled geometry, polyline length, chord length for diagnostics, and junction flags.

[carla_topology_adapter.py](../../ros/ros2_ws/src/global_route_planner/global_route_planner/carla_topology_adapter.py) converts `get_topology()` waypoint pairs and walks `waypoint.next()` at the sampling resolution, retaining exact endpoints. Ambiguous internal branching, missing successors before the exit, and excessive sampling steps fail explicitly instead of selecting an arbitrary branch.

[routing_graph.py](../../ros/ros2_ws/src/global_route_planner/global_route_planner/routing_graph.py) defines `DirectedRoutingGraph` with sampled lane/s node identity. [carla_routing_graph_adapter.py](../../ros/ros2_ws/src/global_route_planner/global_route_planner/carla_routing_graph_adapter.py) expands topology geometry into edges:

| `RoutingEdgeType` | Creation / search cost |
| --- | --- |
| `LANE_FOLLOW` | Consecutive longitudinal samples; Euclidean sample displacement in metres |
| `LANE_CHANGE_LEFT` | Permitted left-marking transition to a sampled adjacent driving lane; displacement + lane-change penalty |
| `LANE_CHANGE_RIGHT` | Equivalent right transition; displacement + lane-change penalty |

Lateral transitions use interior, non-junction samples only. The adapter checks the CARLA lane-marking permission, driving-lane type, same road/section, and availability of a sampled target within the configured OpenDRIVE-s tolerance. These are map-legal graph transitions, not dynamically checked lane-change maneuvers. Each edge retains source topology-edge provenance; duplicate transitions are skipped.

Verified planner defaults in [GlobalRoutePlannerNode.__init__](../../ros/ros2_ws/src/global_route_planner/global_route_planner/global_route_planner_node.py):

| Parameter | Default | Role |
| --- | ---: | --- |
| `sampling_resolution_m` | 2.0 | Nominal topology sampling step |
| `lane_change_penalty_m` | 10.0 | Extra search cost per lateral transition |
| `max_lane_change_s_error_m` | 2.5 | Adjacent sampled-node longitudinal tolerance |
| `max_projection_distance_m` | 5.0 | Maximum world-to-driving-lane projection distance |
| `max_association_s_error_m` | 2.5 | Ego/goal waypoint-to-sampled-node tolerance |

The graph is built once at node startup. A changed CARLA map requires restarting the planner and graph visualizer. No current cost update incorporates dynamic obstacles, predicted agents, or behavior decisions.

## Dijkstra and A*

[routing.py](../../ros/ros2_ws/src/global_route_planner/global_route_planner/routing.py) implements `dijkstra_shortest_path` and `a_star_shortest_path` over explicit nonnegative edge costs, with reconstructed node/edge paths and `NoPathError`. Dijkstra remains the baseline/oracle. The live planner uses A* with 3D Euclidean distance to the goal node as its heuristic. With these edge costs (displacement plus nonnegative penalties), Euclidean distance is a lower bound on remaining cost. The implementation permits reopening nodes for a better g-cost.

Checked-in routing/association diagnostic nodes compare A* and Dijkstra costs, verify edge-cost sums and continuity, and inspect connectivity/edge semantics. These executable checks were inspected, not run in this documentation pass; their presence alone is not a new passing test result.

## Ego/goal association and replanning

[carla_route_association.py](../../ros/ros2_ws/src/global_route_planner/global_route_planner/carla_route_association.py), `associate_carla_world_position`, projects a world position onto a CARLA driving lane and rejects excessive 3D projection distance. [RoutingGraphIndex.nearest_on_lane](../../ros/ros2_ws/src/global_route_planner/global_route_planner/routing_graph_index.py) uses the semantic road/section/lane identity and nearest OpenDRIVE-s sample, rejecting excessive s error. Sample spatial distance is retained for diagnostics.

This association is position-to-global-map association. It does not consume a perception-derived LaneMap. Goal orientation is ignored as a routing constraint; published start/goal poses use projected map position and map yaw rather than the requested orientation.

| Interface | Direction | Contract |
| --- | --- | --- |
| `/carla/hero_odom` (`nav_msgs/msg/Odometry`) | Input | Ego position; frame must match `carla_world` |
| `/planning/goal` (`geometry_msgs/msg/PoseStamped`) | Input | Goal position; frame must match `carla_world` |
| `/planning/route_plan` (`autonomy_interfaces/msg/RoutePlan`) | Output | Global nominal route in direct `carla_world` |

The planner accepts a valid goal and searches immediately if an ego association is available, otherwise waits for odometry. Each newly accepted goal triggers a search, even at the same start node. Odometry causes replanning when association changes to another sampled start node. Both successful and no-path queries record that start node to avoid repeated search at raw odometry frequency.

Rejected goal frame/projection leaves the prior accepted goal active. Bad-frame or unassociable ego odometry logs and returns without publishing a new INVALID route. Do not confuse association rejection with a completed search that finds no directed path. The node has no route age timeout; its goal callback uses the latest stored successful ego association.

## RoutePlan semantics, IDs, and QoS

[RoutePlan.msg](../../ros/ros2_ws/src/autonomy_interfaces/msg/RoutePlan.msg) contains `header`, `route_id`, `map_revision`, `start`, `goal`, `lane_segment_ids`, `coarse_reference_path`, `status`, and `confidence`.

| Constant | Value | Current behavior |
| --- | ---: | --- |
| `STATUS_INVALID` | 0 | Published on `NoPathError`; empty path and segment IDs, confidence 0, associated start/goal retained |
| `STATUS_VALID` | 1 | Published when a route exists |
| `STATUS_PARTIAL` | 2 | Defined in schema; no active planner behavior |
| `STATUS_BLOCKED` | 3 | Defined in schema; no current dynamic-obstacle behavior |

[route_plan_builder.py](../../ros/ros2_ws/src/global_route_planner/global_route_planner/route_plan_builder.py) constructs the coarse path directly from sampled graph nodes. It is global nominal map geometry, not a collision-checked, speed-profiled, dynamically feasible control trajectory. Message/path/pose stamps use planner publication time. `route_id` advances for each publication within the node process; it is not a persistent cross-restart identity.

`association_quality` computes `clamp(1 - max(normalized errors), 0, 1)` over ego/goal projection-distance and sample-s errors relative to their rejection thresholds. Confidence is an association-quality heuristic, NOT a calibrated probability or guarantee of route safety.

`stable_map_revision(map_name, opendrive_text)` hashes UTF-8 `map_name + "\0" + opendrive_text` using SHA-256 and interprets the first 8 digest bytes as an unsigned big-endian uint64. It is a global-map fingerprint, not a graph-parameter fingerprint or a local observation counter.

[global_segment_ids.py](../../ros/ros2_ws/src/global_route_planner/global_route_planner/global_segment_ids.py) hashes namespace `global_route_planner/opendrive_topology_segment/v1`, global map revision, and source/target topology endpoint identities into uint64 IDs. The topology ID map is collision-checked at startup. Ordered route provenance collapses consecutive edges from the same topology segment. Lane changes carry source-segment provenance; the next lane-follow edge moves provenance to the adjacent segment.

**`RoutePlan.lane_segment_ids` and `LaneMap.segments[].id` are separate namespaces. `RoutePlan.map_revision` and `LaneMap.map_revision` are also semantically unrelated. Equality-based route-lane association is invalid.**

RoutePlan publication uses reliable, transient-local, keep-last depth 1 QoS; compatible late subscribers receive the latest valid or invalid message while the publisher lives. The goal subscription uses default depth 10 and does not retain a destination across process restart. Odometry uses sensor-data QoS. No goal is automatically published at startup.

## Visualization and production frames

[planning_visualization](../../ros/ros2_ws/src/planning_visualization/planning_visualization) provides four executables:

- `route_plan_visualizer_node`: RoutePlan → `/planning/route_path`; preserves direct world geometry/stamp, publishes an empty Path for any non-VALID status.
- `odom_tf_broadcaster_node`: odometry → production TF `carla_world → hero` with the odometry stamp.
- `global_lane_graph_visualizer_node`: independently builds the graph with matching defaults and publishes transient-local `/planning/global_lane_graph/markers`, colored by edge type. Marker points are raised by 0.05 m to avoid z-fighting.
- `foxglove_world_visualizer_node`: copies/reflects global display paths/markers into `/planning/route_path_viz` and `/planning/global_lane_graph/markers_viz` in `carla_world_viz`, and publishes TF `carla_world_viz → hero_viz`.

The display adapter uses `y_viz=-y_carla` and the matching orientation basis change. Local lane paths already use forward-left ego geometry, so `/perception/viz/lane/*` aliases change frame labels to `hero_viz` without mirroring point geometry. Production RoutePlan, LaneMap world positions/headings, and hero pose stay in direct CARLA convention. No additional y flip belongs in future route-lane association. The [architecture registry](../agent/ARCHITECTURE.md) records scalar/timestamp qualifications and all six local aliases.

Foxglove uses fixed frame `carla_world_viz`, display frame `hero_viz`. [foxglove_bridge.yaml](../../ros/ros2_ws/config/foxglove_bridge.yaml) exposes planning/perception and TF topics; client publishing is disabled by the current whitelist, so publish test goals from a ROS shell.

## Supplied Town10HD development validation

The supplied scenario is Town10HD / Town10HD_Opt with CARLA 0.9.16. Exact map/export identity across every observation is not supplied; the values below describe development runs, not invariants for every map/version/configuration.

| Observed quantity | Value |
| --- | ---: |
| Sampled graph nodes | 3066 |
| Sampled graph edges | 4522 |
| Lane-follow edges | approximately 3098 |
| Lane-change-left / right edges | approximately 712 / 712 |
| Strongly connected components | 1 |
| Global map revision | 6632468842588822852 |

A validated development test goal was `(109.254, 89.835, 0.0)` in `carla_world`; it is not a generic destination. Valid routes varied with ego position (for example 128 and 323 poses), so there is no fixed expected path length. [The runbook](../ros/runbook.md) gives the command and bounded checks.

A supplied route-versus-graph probe after visualization work observed `route_frame=carla_world`, `route_poses=323`, `graph_points=6196`, with `route_to_graph_m` min/mean/median/max all **0.0000**. This supports the path's provenance in the same routing-graph geometry. It does not establish vehicle dynamic feasibility. The raw probe/method is not checked in with this evidence: 6196 points correspond numerically to two endpoints per 3098 lane-follow edges, not all marker points; the marker z offset also prevents inferring exact 3D equality from this result.

The final supplied visualization chain observed `RoutePlan.status=1`, `/planning/route_path` in `carla_world`, and `/planning/route_path_viz` in `carla_world_viz` with the same source timestamp. `/perception/viz/lane/tracked_centerline` was observed in `hero_viz`. The separate [LaneMap frame probe](lane_perception_tracking_mapping.md) favored direct alignment over an additional y flip.

## Reproducible startup checkpoint

[06_planning.sh](../../autonomous_driving_startup/06_planning.sh) starts the planner and those four adapters in local tmux session `carla_planning`: window `planning` contains planner, route visualizer, production TF, and diagnostics; window `visualization` contains graph visualizer, Foxglove adapter, and an interactive goal shell. Each pane runs processes remotely via SSH.

Supplied runtime evidence reports that a clean `./00_stack.sh stop` followed by `./00_stack.sh full` restored exactly one instance of `global_route_planner`, `route_plan_visualizer`, `odom_tf_broadcaster`, `global_lane_graph_visualizer`, and `foxglove_world_visualizer`. It also observed the six `/perception/viz/lane/*` aliases and all of `/planning/goal`, `/planning/route_plan`, `/planning/route_path`, `/planning/route_path_viz`, `/planning/global_lane_graph/markers`, `/planning/global_lane_graph/markers_viz`.

This is one validated restart, not a guarantee of process cleanup under every SSH failure. The script's `stop` kills its named local tmux sessions; `status` checks session existence. The planner waits for a new goal after restart. A topic can exist before it has published a message; bound one-shot diagnostics with `timeout`.

## Limits and next boundary

There is no dynamic-object prediction or dynamic-obstacle cost in global routing, no behavior planner, no trajectory feasibility check, no goal-orientation constraint, and no local trajectory planner or trajectory-tracking controller for this pipeline. Shared trajectory schemas and older reactive controllers do not change those facts.

NEXT: a separate RoutePlan ↔ LaneMap association layer combines global intent/topology with local perceived geometry/confidence. Planned concepts are ego-local route windows, route-relative progress `s`, signed lateral offset `d`, heading error, overlap/coverage, confidence gates, temporal hysteresis where useful, and explicit no-valid-association behavior. Interfaces/sign policies are not finalized. The layer must tolerate empty/partial LaneMap near intersections and must not hallucinate perceived geometry from OpenDRIVE. See [ROADMAP.md](../agent/ROADMAP.md).

## Relevant commits

| Phase | Commits |
| --- | --- |
| Coarse topology / sampling | `ff23221`, `0234a92` (2026-08-20) |
| Dijkstra / connectivity | `e261432`, `bab6cc1` (2026-08-20–21) |
| Sampled routing / ego-goal association / live diagnostics | `a651b63`, `de9dfa5`, `4541c0c` (2026-08-22) |
| RoutePlan / global segment provenance | `89be996`, `b161709` (2026-08-22) |
| Goal interface / A* / no-path invalidation | `0063ae7`, `a32983f`, `89d8561` (2026-08-23) |
| LaneMap production-frame correction | `cbcea54` (2026-09-15) |
| Route/TF, graph, and Foxglove visualization | `1d6186b`, `1cf9d60`, `ce97421` (2026-09-16) |
| Reproducible stack launchers | `7b54639` (2026-09-16) |
