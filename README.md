# Vision-Based Perception, Lane Mapping, and Global Routing

A modular educational autonomous-driving prototype in CARLA + ROS2. The goal is to understand and implement a coherent autonomy stack with explicit subsystem interfaces and deliberately simple algorithms.

The current development path combines perception-derived local lane geometry with privileged OpenDRIVE global routing. Earlier object, depth, free-space, reactive-navigation, and occupancy-mapping capabilities remain available alongside it.

<p align="center">
  <img src="assets/AD_Planning_Photo.png" alt="OpenDRIVE routing graph, ego pose, and selected RoutePlan in Foxglove" width="1000"/>
</p>

**Global planning:** OpenDRIVE-derived routing graph, ego pose, and selected RoutePlan visualized live in Foxglove.

## Current Stack

```text
CARLA RGB + camera calibration                 CARLA / OpenDRIVE
        |                                            |
road-marking segmentation                     directed topology
        |                                            |
BEV lane geometry + filtering                 sampled routing graph
        |                                            |
temporal tracking + hero odometry             ego + goal association -> A*
        |                                            |
rolling local LaneMap                         global RoutePlan
        |                                            |
        +--------> RoutePlan ↔ LaneMap association <--+
                   [NEXT; NOT IMPLEMENTED]
                              |
                   local trajectory planning [FUTURE]
                              |
                   trajectory tracking/control [FUTURE]
```

LaneMap and RoutePlan publish world geometry in direct `carla_world`, but own separate geometry, identifier, and revision semantics. Global routing does not consume LaneMap yet. Uncertain or missing perception is not replaced with simulator-map geometry.

<p align="center">
  <img src="assets/AD_Lane_Detection_Photo.png" alt="Camera input alongside road-marking candidates, filtered lane curves, and temporal tracking outputs" width="1000"/>
</p>

**Lane perception and tracking:** road-marking candidates, geometric filtering, curve fitting, temporal tracking, and local lane geometry alongside the camera input. This pipeline feeds the perception-derived rolling LaneMap.

## Highlights

- Custom OpenDRIVE topology/routing graph, Dijkstra baseline, A* search, and valid/invalid `RoutePlan` publication.
- Lane-specific segmentation, metric BEV geometry, quality gating, temporal tracking, and odometry propagation.
- Rolling vector `LaneMap` with sampled boundaries, centerline, curvature, and confidence.
- Planning/Foxglove visualization and reproducible local tmux/SSH startup tooling.
- SegFormer semantic segmentation trained and refined with Cityscapes and CARLA data.
- CARLA dataset collection, Cityscapes-19 conversion, filtering, and targeted rare-class sampling.
- ROS2 nodes for segmentation, object extraction, tracking, monocular depth, and object-depth fusion.
- Semantic-depth free-space estimation and two reactive navigation prototypes.
- Local occupancy grids with separate static and dynamic obstacle layers.
- Short-term accumulated local mapping using simulator-provided odometry.

## Architecture

The repository preserves explicit ROS2 interfaces between these implemented areas:

| Layer | Scope |
| --- | --- |
| Model development | MMSegmentation configs, training, evaluation, inference, and qualitative overlays |
| Dataset engineering | CARLA collection, label conversion, filtering, pruning, and dataset balancing |
| ROS2 perception | Image bridging, segmentation, detection, tracking, depth, and fusion |
| Navigation and mapping | Free-space control, local occupancy estimation, and accumulated local maps |
| Lane perception and mapping | Road markings, BEV curves, temporal tracking, and rolling vector LaneMap |
| Global route planning | Privileged map topology, lane-follow/lane-change graph, ego/goal association, and RoutePlan |
| Operations and visualization | Local tmux/remote processes; production and Foxglove frame adapters |

The ROS2 workspace includes these principal packages:

```text
carla_bridge_node           semantic_seg_node
object_detection_node      tracking_node
depth_node                 fusion_node
free_space_node            reactive_navigation_node
free_space_navigation_node local_occupancy_node
local_mapping_node         carla_control_node
road_marking_seg_node       lane_geometry_node
lane_reasoning_nodes        global_route_planner
planning_visualization     autonomy_interfaces (shared messages)
```

## Repository Layout

```text
configs/       MMSegmentation training and fine-tuning configurations
scripts/       Training, evaluation, inference, and visualization utilities
carla_tools/   Simulator data collection and dataset preparation tools
ros/ros2_ws/   ROS2 workspace; src/ includes:
  autonomy_interfaces/    LaneMap, RoutePlan, and future trajectory schemas
  lane_geometry_node/     BEV geometry, filtering, curve fitting, tracking
  lane_reasoning_nodes/   Tracked lane mapping and earlier reasoning prototypes
  global_route_planner/   OpenDRIVE graph construction and route search
  planning_visualization/ Route, graph, and Foxglove frame adapters
autonomous_driving_startup/ Local tmux launchers for remote CARLA/ROS2 processes
docs/          Setup, operation, timeline, and milestone documentation
assets/        Portfolio images and demonstrations
```

## Experiments and tuning

- [Parameter tuning log](docs/experiments/parameter_tuning_log.md): historical runtime iterations for navigation, occupancy, mapping, and classical lane detection.

## Getting Started

The project provides a pinned reference `ros2seg` environment:

```bash
conda env create -f environment.yml
conda activate ros2seg
```

Current remote launchers also use `ros2depth` for the lane/planning core. Follow the [setup notes](docs/setup.md) and [runbook](docs/ros/runbook.md); the [startup package](autonomous_driving_startup/README.md) runs tmux locally and ROS/CARLA remotely.

## Project Status

As of 2026-09-16, [route-ready lane mapping](docs/milestones/lane_perception_tracking_mapping.md) and [global route planning](docs/milestones/global_route_planning.md) are implemented, with development runtime validation recorded in those milestones. `feature/route-lane-association` starts from the completed routing/startup checkpoint; association is the next milestone and has no implementation yet.

The global coarse route is nominal map geometry, not a dynamically feasible control trajectory. Behavior planning, local trajectory planning, and trajectory tracking for this pipeline remain future work. Existing reactive controllers do not complete that architecture. See the [current state](docs/agent/CURRENT_STATE.md) and [small dependency-aware roadmap](docs/agent/ROADMAP.md).

## Technical Stack

Python, PyTorch, MMSegmentation, MMEngine, SegFormer, Depth Anything V2, ROS2, CARLA, OpenCV, NumPy, `cv_bridge`, and `vision_msgs`.

## Earlier Perception and Spatial Milestones

<p align="center">
  <img src="assets/AD_Project_Demo.gif" alt="Earlier CARLA perception and spatial stack demonstration" width="900"/>
</p>

**Earlier perception/spatial stack:** semantic perception, monocular depth, free-space estimation, occupancy, and accumulated local mapping developed before the lane/routing path became the primary integration focus. This accumulated occupancy representation is distinct from the current vector LaneMap.

<p align="center">
  <img src="assets/Segmentatation+Overlay+Tracking.png" alt="Live CARLA semantic segmentation with extracted object detections and tracking outputs" width="900"/>
</p>

**Semantic and object perception:** live CARLA semantic segmentation with extracted object detections and tracking outputs.

## License

Licensed under the MIT License. See [LICENSE](LICENSE).
