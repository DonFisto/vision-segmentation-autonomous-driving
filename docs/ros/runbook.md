# ROS2 and CARLA Runbook

This runbook starts and verifies the public CARLA-to-ROS2 pipeline through accumulated local mapping.

## Pipeline

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

## Prepare Each Shell

Run the following in every shell used for a ROS2 node:

```bash
conda activate ros2seg
cd ros/ros2_ws
source install/setup.bash
```

Commands below are ordered by dependency. Run each long-lived node in its own prepared shell.

## Start the Pipeline

### 1. CARLA Bridge and Hero Odometry

```bash
ros2 run carla_bridge_node carla_bridge_node
```

Publishes RGB images and `/carla/hero_odom`. The odometry is simulator-provided ground truth used by accumulated mapping.

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
ros2 topic hz /carla/rgb/image_raw
ros2 topic echo /carla/hero_odom --once
ros2 topic hz /perception/semantic_mask
ros2 topic echo /perception/detections --once
ros2 topic echo /perception/tracks --once
ros2 topic hz /perception/depth/image
ros2 topic echo /perception/fused_objects --once
ros2 topic echo /perception/free_space_status --once
ros2 topic echo /perception/local_occupancy_status --once
ros2 topic echo /perception/local_mapping_status --once
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
