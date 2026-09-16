# ROS2 and CARLA Runbook

Synchronized 2026-09-16 with `7b54639`. The current core is lane perception/tracking/LaneMap plus independent global OpenDRIVE routing and planning visualization. RoutePlan ↔ LaneMap association is NEXT, not implemented. The earlier full perception/spatial workflow remains below for its parallel capabilities.

## Current Lane/Planning Core and Startup Package

Complete [setup](../setup.md), including remote environments and binary lane model artifacts. On the LOCAL workstation, from the repository root:

```bash
cd autonomous_driving_startup
./00_stack.sh core
./00_stack.sh status
```

`core` starts `01_sim.sh`, `05_lanes.sh`, `06_planning.sh`, and `07_foxglove.sh`. tmux is local; each pane opens SSH to the remote development machine, where CARLA/ROS processes run. [lib/common.sh](../../autonomous_driving_startup/lib/common.sh) defines connection/layout defaults. This is not remote tmux orchestration.

Other operations, from the same local directory:

```bash
./00_stack.sh full
./00_stack.sh attach carla_planning
./00_stack.sh stop
./00_stack.sh status
```

Choose `core` or `full` for startup. `full` adds object perception, depth/fusion, and legacy spatial mapping. For a clean restart, run `stop` before the chosen startup command. Supplied development evidence validates one `stop` → `full` restart restoring exactly one of each planning/visualization node; this was not rerun for the documentation refresh.

`status` checks named local sessions. `stop` kills those sessions and retains the SSH ControlMaster; `./00_stack.sh close-ssh` closes the shared master. Neither session existence nor topic discovery proves message validity. Avoid treating these commands as remote process-health checks.

`05_lanes.sh` starts `road_marking_node` → `lane_geometry_node` → `lane_component_filter_node` → `lane_context_filter_node` → `lane_curve_fit_node` → `lane_tracking_node` → `tracked_lane_mapping_node`. Odometry supports tracking and mapping. See the [lane milestone](../milestones/lane_perception_tracking_mapping.md) for exact intermediate topics, models, geometry, and quality gates.

`06_planning.sh` has windows `planning` and `visualization`, running these five executables:

```text
global_route_planner global_route_planner_node
planning_visualization route_plan_visualizer_node
planning_visualization odom_tf_broadcaster_node
planning_visualization global_lane_graph_visualizer_node
planning_visualization foxglove_world_visualizer_node
```

The simulation launcher defaults to `NAV_MODE=none`; it prepares a manual-control shell rather than starting a route-following controller. Optional legacy navigation modes do not consume RoutePlan. No behavior planner, local trajectory planner, or tracking controller exists for the current route pipeline.

## Manual Planning Validation

Use the remote interactive goal shell in `carla_planning:visualization`, or prepare a remote shell:

```bash
source ~/miniconda3/etc/profile.d/conda.sh
mamba activate ros2depth
cd ~/vision-segmentation-autonomous-driving/ros/ros2_ws
source install/setup.bash
```

If launching manually instead of through the startup package, run each of the five executables above with `ros2 run` in its own prepared remote shell, after CARLA and hero odometry are available. Do not launch duplicates over an existing stack.

The planner intentionally waits for `/planning/goal`; startup only prints an example and sends no destination. Publish this **development test goal for the validated Town10HD/Town10HD_Opt scenario**, not a generic destination:

```bash
timeout 10s ros2 topic pub --once /planning/goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: carla_world}, pose: {position: {x: 109.254, y: 89.835, z: 0.0}, orientation: {w: 1.0}}}'
```

Goal orientation is currently ignored. The planner rejects wrong-frame/unassociable goals and retains the prior accepted goal. With a valid ego/goal association, A* publishes `STATUS_VALID=1` when a route exists or `STATUS_INVALID=0` with empty geometry when no directed path exists. It replans for a newly accepted goal or a changed sampled start node, not every odometry callback. Pose counts vary with ego/goal and graph configuration.

Bound diagnostics so absent messages cannot block the terminal indefinitely:

```bash
timeout 10s ros2 topic echo /carla/hero_odom --once --field header
timeout 10s ros2 topic echo /perception/lane/tracking_status --once
timeout 10s ros2 topic echo /perception/lane/local_map/vector --once --field schema_version
timeout 10s ros2 topic echo /perception/lane/local_map/status --once
timeout 10s ros2 topic echo /planning/route_plan --once --field status --qos-durability transient_local --qos-reliability reliable
timeout 10s ros2 topic echo /planning/route_path --once --field header --qos-durability transient_local --qos-reliability reliable
timeout 10s ros2 topic echo /planning/route_path_viz --once --field header --qos-durability transient_local --qos-reliability reliable
timeout 10s ros2 topic echo /perception/viz/lane/tracked_centerline --once --field header
ros2 topic info /planning/route_plan --verbose
ros2 topic info /planning/global_lane_graph/markers_viz --verbose
```

Transient-local subscriptions are useful for route/graph outputs that may have published before the diagnostic starts. `timeout` expiry (normally exit code 124) means no result arrived within that bound, not necessarily a planner fault. In particular, `ros2 topic echo --once` can wait indefinitely before a goal has caused any route publication.

Source defaults configure hero odometry at 20 Hz and LaneMap publication at 10 Hz; these are not measured-rate guarantees. If needed, measure with bounded `timeout 10s ros2 topic hz /carla/hero_odom`. For global graph inspection the package also provides `routing_graph_diagnostic_node` and `route_association_diagnostic_node`; they connect to live CARLA and are separate from ordinary topic echoes.

## Foxglove Frames and Topics

Use fixed frame **`carla_world_viz`**, display frame **`hero_viz`**, with:

```text
/planning/global_lane_graph/markers_viz
/planning/route_path_viz
/perception/viz/lane/left_boundary
/perception/viz/lane/right_boundary
/perception/viz/lane/centerline
/perception/viz/lane/tracked_left_boundary
/perception/viz/lane/tracked_right_boundary
/perception/viz/lane/tracked_centerline
```

Production `/planning/route_plan`, `/planning/route_path`, `/planning/global_lane_graph/markers`, published LaneMap world positions/headings, and hero pose use direct `carla_world`. Display aliases reflect global y and orientation basis; local forward-left lane points are only reattached to `hero_viz`, never mirrored again. The separate TF trees are `carla_world → hero` and `carla_world_viz → hero_viz`. Future route-lane association must use production coordinates with no extra y flip. See [ARCHITECTURE.md](../agent/ARCHITECTURE.md) for the `hero` convention and scalar/timestamp qualifications.

Supplied runtime evidence observed RoutePlan status 1, original route Path frame `carla_world`, display Path frame `carla_world_viz` at the same source timestamp, and tracked centerline display frame `hero_viz`. These are recorded observations, not a new run or a guarantee of simultaneous receipt from separate CLI echoes.

[foxglove_bridge.yaml](../../ros/ros2_ws/config/foxglove_bridge.yaml) exposes `/planning/.*`, `/perception/.*`, and TF, but disables client publishing. Send goals from the ROS shell. `07_foxglove.sh` starts the bridge using the separate `~/fox_ws` installation plus the main workspace overlay; see the [startup README](../../autonomous_driving_startup/README.md).

## Lane/Planning Troubleshooting

- **No route after restart:** publish a goal, check `accepted_goal`/`waiting_for_goal` logs and ego association. Topic discovery alone is insufficient. INVALID with empty geometry is different from no message.
- **No or empty LaneMap:** inspect tracking/status and odometry. Fresh odometry with insufficient lane support can produce `segments=[]`; stale/no odometry suppresses map publication. Missing markings at intersections can be correct; do not fill them from OpenDRIVE.
- **Route persists despite bad input:** rejected goals retain the prior goal, and failed ego association logs/returns. Durable route output is not proof of fresh ego state. Only no-path search currently publishes INVALID.
- **Mirrored or misplaced display:** select `_viz` global topics and `/perception/viz/lane/*` under the display frames. Do not change planner inputs to compensate for a display choice.
- **Changed CARLA map/configuration:** planner and graph visualizer build their graphs independently at startup; restart both with matching graph settings. Marker z is raised by 0.05 m for display.
- **Duplicate processes:** inspect `ros2 node list` in the remote diagnostic shell. The supplied clean restart restored one of each five nodes; local tmux `status` alone does not establish that result.
- **Missing lane checkpoint:** `05_lanes.sh` requires a `best_mDice*.pth` artifact in its configured work directory. See [setup](../setup.md); depth/fusion and legacy spatial mapping are not required for route-lane development.

## Legacy/Full Perception-Spatial Stack

This earlier parallel pipeline remains implemented and can be launched through `full`, or manually as below. Its historical capabilities do not constitute the current RoutePlan-to-control architecture.

### Pipeline

```text
CARLA bridge
  |-- RGB image --> semantic segmentation --> object extraction --> tracking --+
  |                                                                            +--> fusion
  |-- RGB image --> monocular depth -------------------------------------------+
  |                    |
  |                    +--> free-space estimation
  |                    +--> local occupancy
  |
  |-- hero odometry ---------------------------> accumulated local mapping
                                                  ^
local occupancy grids ----------------------------+
```

## Prerequisites

- Complete [the setup guide](../setup.md).
- Make the segmentation config and checkpoint available at the locations configured by the segmentation node.
- Start a compatible CARLA instance using its standard launcher. For a headless installation, a typical public command is:

```bash
<CARLA_INSTALL>/CarlaUE4.sh -RenderOffScreen
```

The bridge creates the hero vehicle and attached RGB camera. Start it against a clean simulation world so its initial spawn point is available.

### Prepare Each Legacy Shell

Run the following in every shell used for a ROS2 node:

```bash
# For segmentation/object extraction/tracking:
mamba activate ros2seg
# For bridge/depth/fusion/free-space/occupancy/mapping instead:
# mamba activate ros2depth
cd ros/ros2_ws
source install/setup.bash
```

Commands below are ordered by dependency. Run each long-lived node in its own prepared shell.

## Start the Pipeline

### 1. CARLA Bridge and Hero Odometry

```bash
ros2 run carla_bridge_node carla_bridge_node
```

Publishes RGB images, camera calibration, and `/carla/hero_odom`. The simulator-provided pose is used by accumulated mapping and the current lane/planning pipeline.

### 2. Semantic Segmentation

```bash
ros2 run semantic_seg_node seg_node
```

### 3. Object Extraction

```bash
ros2 run object_detection_node detector
```

This node extracts Cityscapes object classes from the semantic mask; it is not a separate learned object detector.

### 4. Tracking

```bash
ros2 run tracking_node tracking_node
```

### 5. Monocular Depth

```bash
ros2 run depth_node depth_node
```

Depth Anything V2 produces relative depth, not calibrated metric distance.

### 6. Object-Depth Fusion

```bash
ros2 run fusion_node fusion_node
```

### 7. Free-Space Estimation

```bash
ros2 run free_space_node free_space_node
```

### 8. Local Occupancy

```bash
ros2 run local_occupancy_node local_occupancy_node
```

### 9. Accumulated Local Mapping

```bash
ros2 run local_mapping_node local_mapping_node
```

The mapping node combines hero odometry with the combined, static, and dynamic local occupancy grids.

## Topic Reference

| Stage | Main output | Type |
| --- | --- | --- |
| Bridge | `/carla/rgb/image_raw` | `sensor_msgs/msg/Image` |
| Bridge | `/carla/rgb/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` |
| Bridge | `/carla/hero_odom` | `nav_msgs/msg/Odometry` |
| Segmentation | `/perception/semantic_mask` | `sensor_msgs/msg/Image` |
| Segmentation | `/perception/semantic_overlay/compressed` | `sensor_msgs/msg/CompressedImage` |
| Object extraction | `/perception/detections` | `vision_msgs/msg/Detection2DArray` |
| Tracking | `/perception/tracks` | `vision_msgs/msg/Detection2DArray` |
| Depth | `/perception/depth/image` | `sensor_msgs/msg/Image` |
| Depth | `/perception/depth/colormap/compressed` | `sensor_msgs/msg/CompressedImage` |
| Fusion | `/perception/fused_objects` | `std_msgs/msg/String` |
| Free space | `/perception/free_space_status` | `std_msgs/msg/String` |
| Local occupancy | `/perception/local_occupancy_grid` | `nav_msgs/msg/OccupancyGrid` |
| Local occupancy | `/perception/local_static_obstacle_grid` | `nav_msgs/msg/OccupancyGrid` |
| Local occupancy | `/perception/local_dynamic_obstacle_grid` | `nav_msgs/msg/OccupancyGrid` |
| Local occupancy | `/perception/local_occupancy_status` | `std_msgs/msg/String` |
| Local mapping | `/perception/accumulated_local_map` | `nav_msgs/msg/OccupancyGrid` |
| Local mapping | `/perception/accumulated_static_map` | `nav_msgs/msg/OccupancyGrid` |
| Local mapping | `/perception/accumulated_dynamic_map` | `nav_msgs/msg/OccupancyGrid` |
| Local mapping | `/perception/local_mapping_status` | `std_msgs/msg/String` |

Compressed free-space, occupancy, and mapping debug images are also published for visualization.

## Verify the Pipeline

List active nodes and topics:

```bash
ros2 node list
ros2 topic list
```

Check each major boundary in order:

```bash
timeout 10s ros2 topic hz /carla/rgb/image_raw
timeout 10s ros2 topic echo /carla/hero_odom --once
timeout 10s ros2 topic hz /perception/semantic_mask
timeout 10s ros2 topic echo /perception/detections --once
timeout 10s ros2 topic echo /perception/tracks --once
timeout 10s ros2 topic hz /perception/depth/image
timeout 10s ros2 topic echo /perception/fused_objects --once
timeout 10s ros2 topic echo /perception/free_space_status --once
timeout 10s ros2 topic echo /perception/local_occupancy_status --once
timeout 10s ros2 topic echo /perception/local_mapping_status --once
```

Inspect publisher/subscriber connections when a topic exists but data does not flow:

```bash
ros2 topic info /perception/semantic_mask --verbose
ros2 topic info /perception/depth/image --verbose
ros2 topic info /perception/local_occupancy_grid --verbose
ros2 topic info /perception/accumulated_local_map --verbose
```

## Record a Representative Run

```bash
ros2 bag record \
  /carla/rgb/image_raw/compressed \
  /carla/hero_odom \
  /perception/semantic_overlay/compressed \
  /perception/tracks \
  /perception/depth/colormap/compressed \
  /perception/fused_objects \
  /perception/free_space_status \
  /perception/local_occupancy_grid \
  /perception/accumulated_local_map \
  /perception/local_mapping_status
```

Stop recording with `Ctrl+C`. Replay with `ros2 bag play <bag_directory>`.

## Troubleshooting

### Package or executable not found

- Activate the expected environment.
- Source `ros/ros2_ws/install/setup.bash` in the current shell.
- Rebuild with `colcon build --symlink-install` after dependency or package changes.

### Bridge does not start

- Confirm CARLA is running and its Python API release matches the simulator.
- Use a clean world with an available vehicle spawn point.
- Check the bridge log before starting downstream nodes.

### Segmentation does not start

- Confirm the configured model file and checkpoint both exist and are compatible.
- Verify `torch.cuda.is_available()` returns `True`.
- Check that the pinned MMCV, MMEngine, MMSegmentation, and PyTorch versions are installed.

### Depth does not start

- Verify `transformers`, `tokenizers`, and `huggingface-hub` are installed.
- Allow the first run to retrieve the default model, or populate the model cache beforehand.
- Check available GPU memory if initialization or inference fails.

### Detections, tracks, or fusion are empty

- Confirm the semantic mask and depth topics are publishing.
- Inspect `/perception/detections` before debugging tracking.
- Remember that fusion requires both tracks and depth.

### Free-space or occupancy outputs are absent

- Confirm both `/perception/semantic_mask` and `/perception/depth/image` are active.
- Echo the corresponding status topic before inspecting large image or grid messages.
- Check topic connection details for name or type mismatches.

### Accumulated maps are absent

- Confirm `/carla/hero_odom` is publishing.
- Confirm all three local occupancy grid topics are active.
- Start `local_mapping_node` only after the bridge and local occupancy node are producing data.

### Control safety

Only one controller should publish vehicle commands at a time. Stop autonomous command publishers before manual testing, and stop the vehicle before shutting down the perception stack.
