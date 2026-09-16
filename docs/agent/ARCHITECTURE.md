# Architecture Map

Source-verified at `7b54639`, 2026-09-16. This describes the lane/global-routing path; [older milestones](../README.md) retain the parallel object/depth/free-space/occupancy history.

## Data ownership and flow

```text
LOCAL PERCEPTION SIDE                         GLOBAL PRIVILEGED MAP SIDE
CARLA RGB + CameraInfo                        CARLA/OpenDRIVE
  -> road_marking_node                          -> coarse directed topology
  -> lane_geometry_node (metric BEV)             -> sampled lane routing graph
  -> lane_component_filter_node                 -> legal follow/change edges
  -> lane_context_filter_node                   -> ego + goal map association
  -> lane_curve_fit_node                        -> A* (Dijkstra baseline)
  -> lane_tracking_node + hero odometry          -> RoutePlan
  -> tracked_lane_mapping_node                       |
  -> LaneMap                                         |
       |                                             |
       +----> NEXT: route-lane association <----------+
                  route-relative reference/corridor
                         |
                  FUTURE: behavior/maneuver decisions,
                  local motion/trajectory planning,
                  trajectory tracking/control
```

This does not imply a LaneMap subscription in `global_route_planner`: its map source is CARLA. Conversely, the tracked mapper does not import OpenDRIVE route geometry. Association must remain separate from both topology construction and lane perception. Its interface is not finalized.

The earlier parallel path is RGB → semantic segmentation → object extraction/tracking, plus depth → fusion/free space/occupancy → accumulated mapping with odometry. Reactive navigation uses earlier perception cues. None of those nodes implements the future RoutePlan/Trajectory control contract.

## Relevant interface registry

Source roots: [carla_bridge_node](../../ros/ros2_ws/src/carla_bridge_node/carla_bridge_node), [lane_geometry_node](../../ros/ros2_ws/src/lane_geometry_node/lane_geometry_node), [lane_reasoning_nodes](../../ros/ros2_ws/src/lane_reasoning_nodes/lane_reasoning_nodes), [global_route_planner](../../ros/ros2_ws/src/global_route_planner/global_route_planner), [planning_visualization](../../ros/ros2_ws/src/planning_visualization/planning_visualization). Producers below are executable names; visualization ROS node names omit `_node`. The bridge executable registers the ROS node name `carla_rgb_publisher`.

| Producer | Topic | Message type | Frame | Current consumer / semantic role |
| --- | --- | --- | --- | --- |
| `carla_bridge_node` | `/carla/hero_odom` | `nav_msgs/msg/Odometry` | parent `carla_world`, child `hero` | Tracker, mapper, global planner, TF adapters, older mapping; privileged ego pose |
| `lane_tracking_node` | `/perception/lane/tracked_left_boundary` | `nav_msgs/msg/Path` | `hero`, forward-left lane convention | Tracked mapper and Foxglove adapter; local boundary |
| `lane_tracking_node` | `/perception/lane/tracked_right_boundary` | `nav_msgs/msg/Path` | `hero`, forward-left lane convention | Same, right boundary |
| `lane_tracking_node` | `/perception/lane/tracked_centerline` | `nav_msgs/msg/Path` | `hero`, forward-left lane convention | Same, centerline when supported |
| `lane_tracking_node` | `/perception/lane/tracking_status` | `std_msgs/msg/String` (JSON) | `frame_id` + `stamp_ns` in payload | Mapper; measurement/state/confidence/width/motion evidence absent from Path itself |
| `tracked_lane_mapping_node` | `/perception/lane/local_map/vector` | `autonomy_interfaces/msg/LaneMap` | `carla_world` | Diagnostics now; planned association consumer. Perception-derived rolling geometry |
| Operator / goal publisher | `/planning/goal` | `geometry_msgs/msg/PoseStamped` | must match `carla_world` | `global_route_planner_node`; destination position, orientation ignored |
| `global_route_planner_node` | `/planning/route_plan` | `autonomy_interfaces/msg/RoutePlan` | `carla_world` | Route visualizer, diagnostics; future association. Global intent/nominal geometry |
| `route_plan_visualizer_node` | `/planning/route_path` | `nav_msgs/msg/Path` | `carla_world` | Foxglove adapter/diagnostics; coarse route or empty path on non-VALID status |
| `global_lane_graph_visualizer_node` | `/planning/global_lane_graph/markers` | `visualization_msgs/msg/MarkerArray` | each marker `carla_world` | Foxglove adapter; separately rebuilt static graph, z raised 0.05 m for display |
| `foxglove_world_visualizer_node` | `/planning/route_path_viz` | `nav_msgs/msg/Path` | `carla_world_viz` | Foxglove display only |
| `foxglove_world_visualizer_node` | `/planning/global_lane_graph/markers_viz` | `visualization_msgs/msg/MarkerArray` | each marker `carla_world_viz` | Foxglove display only |
| `foxglove_world_visualizer_node` | `/perception/viz/lane/*` | `nav_msgs/msg/Path` | `hero_viz` | Six display aliases: `left_boundary`, `right_boundary`, `centerline`, `tracked_left_boundary`, `tracked_right_boundary`, `tracked_centerline` |

Untracked inputs to the Foxglove adapter are `/perception/lane/left_boundary`, `/perception/lane/right_boundary`, and `/perception/lane/centerline`, produced by `lane_curve_fit_node` in `hero`. The upstream image chain and gating are in the [lane milestone](../milestones/lane_perception_tracking_mapping.md).

## Timing, units, and failure boundaries

- Positions, widths, horizons, and graph distances are metres; curvature is inverse metres. CARLA routing nodes store yaw in degrees internally and publish quaternions. Lane algorithms use radians and forward-left polynomials.
- The bridge's `publish_hero_odom` uses the node clock when sampling CARLA; `odom_rate_hz=20.0` is a source default, not a measured rate. Image/CameraInfo callbacks also use the bridge clock, not a documented common simulator acquisition timestamp. Odometry twist copies CARLA velocity components; this registry does not certify a body-frame twist contract.
- Tracking is callback-driven. Left/right paths must be within the default 50 ms synchronization tolerance; curve status is selected from coefficient-compatible history. The tracker finds nearest buffered odometry within 150 ms and propagates/refits curves at lane processing time. Missing/stale odometry disables propagation; excessive motion resets tracks.
- The mapper waits for left/right/center paths and JSON status with the same `stamp_ns`, then uses buffered/interpolated odometry. It publishes using the latest odometry stamp on a default 10 Hz timer, only while that odometry is sufficiently fresh (default 300 ms). These settings do not guarantee throughput. With fresh odometry but insufficient lane support it can publish an empty LaneMap; stale/no odometry suppresses map publication and exposes status.
- RoutePlan uses planner publication time. Search runs for a newly accepted goal or a changed sampled ego start node, not every raw odometry callback. There is no current route/odometry age acceptance policy for future consumers. Rejected goals retain the prior goal; failed ego association logs/returns. Only a completed no-path search publishes INVALID. Consumers must not assume all failures clear a durable route.
- RoutePlan, route paths, and graph markers use reliable, transient-local, keep-last depth 1 QoS. LaneMap uses default reliable/volatile depth 10. Local lane visualization aliases use default depth 10; TF uses odometry timestamps. Durable output is retained for compatible late subscribers while the publisher lives, not across a planner restart.

## Coordinate frames

| Name | Meaning and ownership |
| --- | --- |
| `carla_world` | Production direct CARLA world positions/yaw: bridge pose, RoutePlan, published LaneMap positions/headings, and unmirrored global graph display source. No downstream y flip for route-lane association. |
| `hero` | Odometry child name and lane-path frame label. Lane local geometry is x-forward/y-left/z-up; tracker/mapper convert direct odometry internally with `odom_y_sign=-1`, `odom_yaw_sign=-1`. The shared label alone does not make lane paths numerically compatible with direct-CARLA production TF. |
| `carla_world_viz` | Visualization-only world basis with `y_viz=-y_carla`. It is not a planning frame. |
| `hero_viz` | Visualization ego frame under `carla_world_viz`; receives the existing forward-left local lane points without another mirror. |

`odom_tf_broadcaster_node` copies odometry to production TF `carla_world → hero`. `foxglove_world_visualizer_node` builds the separate display tree `carla_world_viz → hero_viz`; it reflects global positions and changes orientation basis with `S=diag(1,-1,1)`, `R_viz=S R_carla S`, quaternion `(x,y,z,w) → (-x,y,-z,w)`. The reflection is performed on copied display messages; it is not a rigid TF transform connecting the two world frames.

The mapper's `_internal_world_to_carla_world` and `_internal_world_yaw_to_carla_world` perform the public vector conversion. Scalar `LaneSample.curvature` is still computed from the local forward-left centerline, without a sign conversion in that publication step. The supplied positional probe does not validate a downstream signed-curvature interpretation; settle that contract before using it for control. See the [lane milestone](../milestones/lane_perception_tracking_mapping.md).

For Foxglove use fixed frame `carla_world_viz`, display frame `hero_viz`, and the `_viz` global topics plus `/perception/viz/lane/*`. The adapter preserves source timestamps, including the observed route/path-viz pair. It does not currently create a mirrored LaneMap alias.

## Shared schema versus implemented semantics

[autonomy_interfaces/msg](../../ros/ros2_ws/src/autonomy_interfaces/msg) defines `LaneSample`, `LaneBoundary`, `LaneSegment`, `LaneMap`, `RoutePlan`, `TrajectoryPoint`, and `Trajectory`. The last two provide time offsets, poses, velocity/acceleration, curvature, steering angle, IDs, validity, and confidence fields for future work; their existence does not establish a trajectory producer or controller.

`LaneMap` schema 2 is rolling perception memory (`globally_consistent=false`); current segment ID is 1 with boundary IDs 2/3, unknown topology/turn and unknown speed limit. A segment's presence does not certify collision freedom or intersection classification.

`RoutePlan.lane_segment_ids` are global OpenDRIVE topology-edge hashes, not local `LaneSegment.id` values. `RoutePlan.map_revision` fingerprints map name and OpenDRIVE; `LaneMap.map_revision` is a local observation integration/pruning counter. Neither namespace permits equality-based association. [Global routing semantics](../milestones/global_route_planning.md) define the hash, statuses, confidence heuristic, and coarse-path limitations.
