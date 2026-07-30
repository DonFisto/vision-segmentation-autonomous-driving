# Architecture Map

Last initialized: 2026-07-30

This document is an initial map based on repository documentation. The first full Codex review must validate every interface against source code, launch files, topics, frames, parameters, and runtime evidence.

## Current documented pipeline

```text
CARLA RGB + hero odometry
        |
        +--> semantic segmentation --> object extraction --> tracking --+
        |                                                               |
        +--> monocular depth --------------------------------------------+--> object-depth fusion
        |
        +--> segmentation + depth --> free-space estimation --> reactive navigation
        |
        +--> segmentation + depth --> local occupancy layers
                                             |
hero odometry + local occupancy layers ------+--> accumulated local mapping
```

## ROS2 nodes documented in the repository overview

```text
carla_bridge_node
semantic_seg_node
object_detection_node
tracking_node
depth_node
fusion_node
free_space_node
reactive_navigation_node
free_space_navigation_node
local_occupancy_node
local_mapping_node
carla_control_node
```

## Architectural layers

| Layer | Responsibility | Initial assessment |
| --- | --- | --- |
| Model development | Training, evaluation, inference, and qualitative segmentation outputs | Documented |
| Dataset engineering | CARLA collection, Cityscapes conversion, filtering, and balancing | Documented |
| ROS2 perception | Bridging, segmentation, extraction, tracking, depth, and fusion | Documented |
| Navigation and mapping | Free-space navigation, occupancy estimation, and accumulated local mapping | Documented but requires interface validation |
| Planning | Route/reference/path/trajectory generation | Not yet clearly represented as a dedicated layer |
| Control | Lateral and longitudinal tracking | Partial or unclear in current documentation |
| Actuation | Conversion to CARLA steering, throttle, and brake commands | Node exists; contract requires validation |
| Evaluation | Repeatable scenarios and quantitative system metrics | Not yet clearly represented as a first-class layer |

## Intended next-stage architecture

The first closed-loop prototype should evolve toward:

```text
Sensors / simulator state
        |
        v
Perception and local geometry
        |
        v
Local environment representation
        |
        v
Reference or path planner
        |
        v
Lateral + longitudinal controller
        |
        v
CARLA actuation
        |
        +-------------------- feedback --------------------+
```

This is an architectural direction, not a claim that these interfaces already exist.

## Interface registry

The first full review should populate this table from code.

| Producer | Output topic/type | Frame | Units | Rate / timestamp | Consumer | Status |
| --- | --- | --- | --- | --- | --- | --- |
| CARLA bridge | TBD | TBD | TBD | TBD | perception nodes | Unverified |
| Semantic segmentation | TBD | camera/image | class IDs or mask semantics TBD | TBD | extraction, free space, occupancy | Unverified |
| Depth | TBD | camera/image | relative or metric depth TBD | TBD | fusion, free space, occupancy | Unverified |
| Tracking | TBD | image or camera frame TBD | TBD | TBD | fusion / mapping | Unverified |
| Local occupancy | TBD | vehicle-relative TBD | cell resolution TBD | TBD | local mapping / future planner | Unverified |
| Local mapping | TBD | world or rolling local frame TBD | cell resolution TBD | TBD | future planner | Unverified |
| Planner | TBD | TBD | path/trajectory semantics TBD | TBD | controller | Absent or unclear |
| Controller | TBD | vehicle/control frame TBD | steering/throttle/brake scaling TBD | TBD | CARLA control | Partial or unclear |

## Architecture invariants to establish

1. Every geometric output has an explicit coordinate frame.
2. Every numeric quantity has documented units and sign convention.
3. Sensor-derived outputs preserve or explicitly transform timestamps.
4. Static and dynamic obstacles remain semantically distinct where required downstream.
5. The planner consumes one stable, documented environment representation.
6. The controller receives a defined path or trajectory plus current vehicle state.
7. The actuation layer owns simulator-specific scaling, clipping, and command semantics.
8. Evaluation can replay or reproduce representative scenarios.

## Known design boundary

CARLA-provided hero odometry is currently an accepted simplification. Replacing it with visual odometry or SLAM should not block the first planning-and-control milestone unless the review finds that the existing pose interface is unusable.
