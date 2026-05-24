# Local Occupancy Mapping Milestone

## Overview

This milestone documents the first local occupancy-grid prototype implemented in the CARLA + ROS2 vision-based autonomous driving project.

The goal of this stage is to move from image-space perception toward a map-like local spatial representation around the hero vehicle.

The implemented node converts semantic segmentation and monocular depth information into a coarse local occupancy grid:

```text
semantic mask + monocular depth
        -> approximate local occupancy grid
        -> local occupancy status
        -> navigation- and mapping-ready representation
```

This is not yet full SLAM. It is a first intermediate mapping layer that transforms single-frame perception into a structured local grid.

---

## Position in the Project Stack

Before this milestone, the system already included:

```text
CARLA RGB camera
    -> semantic segmentation
    -> object detection from segmentation
    -> tracking
    -> monocular depth estimation
    -> depth-object fusion
    -> semantic-depth free-space estimation
    -> refined free-space navigation
```

The local occupancy node adds the next representation:

```text
/perception/semantic_mask + /perception/depth/image
        -> local_occupancy_node
        -> /perception/local_occupancy_grid
        -> /perception/local_occupancy_debug/compressed
        -> /perception/local_occupancy_status
```

This creates the first step toward local mapping.

---

## Implemented Node

## `local_occupancy_node`

### Role

The `local_occupancy_node` projects image-space semantic/depth evidence into a coarse 2D local grid in front of the hero vehicle.

### Inputs

```text
/perception/semantic_mask
/perception/depth/image
```

### Outputs

```text
/perception/local_occupancy_grid
/perception/local_occupancy_debug/compressed
/perception/local_occupancy_status
```

---

## Input: Semantic Mask

The semantic segmentation node publishes:

```text
/perception/semantic_mask
```

as:

```text
type: sensor_msgs/msg/Image
encoding: mono8
height: 600
width: 800
```

The pixel values correspond to Cityscapes-19 class IDs.

Relevant classes:

```text
0  road
1  sidewalk
2  building
3  wall
4  fence
5  pole
6  traffic light
7  traffic sign
8  vegetation
9  terrain
10 sky
11 person
12 rider
13 car
14 truck
15 bus
16 train
17 motorcycle
18 bicycle
```

The current local occupancy prototype treats:

```text
road -> free
sidewalk, building, wall, fence, pole, person, rider, car, truck, bus, train, motorcycle, bicycle -> occupied
other classes -> unknown / ignored
```

---

## Input: Depth Image

The depth node publishes:

```text
/perception/depth/image
```

as:

```text
type: sensor_msgs/msg/Image
encoding: 32FC1
```

The depth signal comes from Depth Anything V2 and is relative, not metric.

The current observed convention is:

```text
larger depth value = closer
```

The depth information is used to reinforce obstacle detection, especially for close non-road regions.

---

## Output: Occupancy Grid

The main output is:

```text
/perception/local_occupancy_grid
```

with type:

```text
nav_msgs/msg/OccupancyGrid
```

Current cell values:

```text
-1   unknown
 0   free
100  occupied
```

The grid represents an approximate region in front of the hero vehicle.

Known-good test configuration:

```text
resolution = 0.25 m/cell
forward_m  = 18.0 m
width_m    = 10.0 m
```

This produces a grid of approximately:

```text
width_cells  = 40
height_cells = 72
```

depending on the selected width and resolution.

---

## Output: Debug Image

The debug output is:

```text
/perception/local_occupancy_debug/compressed
```

Visualization convention:

```text
green = free / projected road
red   = occupied / non-drivable or obstacle region
gray  = unknown / unobserved
```

This debug topic is used in Foxglove to inspect whether the semantic/depth perception is being converted into a meaningful local grid.

---

## Output: Status Message

The node also publishes:

```text
/perception/local_occupancy_status
```

as JSON inside a `std_msgs/String`.

The status includes:

```text
global free / occupied / unknown ratios
near-field left / center / right ratios
mid-field left / center / right ratios
scores for left / center / right
recommended_direction
```

Example fields:

```json
{
  "global_ratios": {
    "free": 0.10,
    "occupied": 0.41,
    "unknown": 0.48
  },
  "near_field": {
    "left": {
      "free": 0.04,
      "occupied": 0.00,
      "unknown": 0.96
    },
    "center": {
      "free": 0.44,
      "occupied": 0.25,
      "unknown": 0.29
    },
    "right": {
      "free": 0.04,
      "occupied": 0.00,
      "unknown": 0.96
    }
  },
  "scores": {
    "left": 0.04,
    "center": 0.13,
    "right": 0.03
  },
  "recommended_direction": "forward"
}
```

This status is intended to support future local-occupancy navigation.

---

## Known-Good Runtime Parameters

The following parameters produced a stable and visually useful occupancy-grid projection:

```bash
ros2 run local_occupancy_node local_occupancy_node --ros-args \
  -p roi_x_min_ratio:=0.10 \
  -p roi_x_max_ratio:=0.90 \
  -p roi_y_min_ratio:=0.20 \
  -p roi_y_max_ratio:=0.95 \
  -p forward_m:=18.0 \
  -p width_m:=10.0 \
  -p resolution:=0.25 \
  -p far_power:=1.3 \
  -p pixel_stride:=3 \
  -p obstacle_dilate_cells:=1 \
  -p free_dilate_cells:=1
```

Important tuning notes:

```text
far_power controls how image rows are mapped to forward distance.
Lower values stretch the projected road farther forward.
Higher values compress the road toward the near field.
```

The known-good value was:

```text
far_power = 1.3
```

---

## Projection Method

The current node uses a heuristic projection, not a calibrated 3D reconstruction.

The projection assumes:

```text
image row    -> approximate forward distance
image column -> approximate lateral position using horizontal field of view
```

The bottom of the selected ROI is mapped to the near field.

The top of the selected ROI is mapped to farther distances.

This produces a wedge-like map shape, which is expected for a first prototype.

Conceptually:

```text
camera image
    -> select ROI
    -> sample semantic/depth pixels
    -> classify each sampled pixel as free / occupied / unknown
    -> project sampled pixels into local grid cells
    -> dilate free/occupied cells for stability and visibility
```

---

## Why the Projection Is Approximate

The current method does not yet use:

```text
true camera intrinsics
true camera extrinsics
ground-plane geometry
metric depth
hero pose or odometry
```

Therefore, it should not be interpreted as a metric SLAM map.

It is a practical intermediate representation for:

```text
visualization
debugging
navigation-oriented status estimation
bridging perception and mapping
```

---

## Important Lessons

### 1. Image-space free space is not enough

The previous free-space node produced useful image-space masks, but navigation decisions based only on image-space left/center/right ratios can be counterintuitive.

For example, a visually open road may have low free-space ratio if the ROI contains many unknown or non-drivable pixels.

The occupancy grid improves this by moving toward a local spatial representation.

### 2. Local occupancy is more interpretable for navigation

The local grid makes it easier to reason about:

```text
near field
mid field
left side
center corridor
right side
unknown regions
```

This is more useful than only observing raw semantic overlays or depth colormaps.

### 3. Dynamic and static obstacles should eventually be separated

The current occupancy grid uses a single occupied class.

However, future mapping should distinguish:

```text
static non-drivable areas:
  sidewalk, building, wall, fence, pole, vegetation, terrain

dynamic obstacles:
  person, rider, car, truck, bus, train, motorcycle, bicycle
```

This matters because static structures can be accumulated into a persistent map, while dynamic obstacles should decay quickly or remain temporary.

---

## Current Limitations

### 1. Not metric SLAM

The map is local and approximate.

It does not yet accumulate observations over time.

### 2. No hero odometry yet

The node does not yet know how the hero vehicle moves.

Without odometry, the grid is recomputed frame by frame and cannot form an accumulated map.

### 3. Relative depth only

Depth Anything V2 provides relative depth rather than metric distance.

This limits geometric accuracy.

### 4. No true camera calibration

The current image-to-grid projection uses FOV and heuristic row mapping.

A better version should use camera intrinsics and extrinsics.

### 5. Dynamic obstacles are not separated yet

Cars, pedestrians and static structures are currently merged into a single occupied layer.

This is acceptable for the first prototype but not ideal for map accumulation.

### 6. Reactive navigation still dominates behavior

The vehicle can still get stuck in dense traffic or blocked configurations because there is no planner yet.

The occupancy grid is a preparation step for better local planning.

---

## Recommended Next Improvements

### 1. Split static and dynamic occupancy layers

Modify `local_occupancy_node` to internally distinguish:

```text
free
static_non_drivable
dynamic_obstacle
unknown
```

Suggested class split:

```text
free:
  0 road

static:
  1 sidewalk
  2 building
  3 wall
  4 fence
  5 pole
  8 vegetation
  9 terrain

dynamic:
  11 person
  12 rider
  13 car
  14 truck
  15 bus
  16 train
  17 motorcycle
  18 bicycle
```

Future debug colors:

```text
green  = free
red    = static obstacle / non-drivable area
orange = dynamic obstacle
gray   = unknown
```

### 2. Add `/carla/hero_odom`

The next major technical step is publishing hero pose and velocity from CARLA:

```text
/carla/hero_odom
```

with type:

```text
nav_msgs/msg/Odometry
```

This will allow the system to relate consecutive local grids over time.

### 3. Build `local_mapping_node`

Once hero odometry exists, implement:

```text
/perception/local_occupancy_grid + /carla/hero_odom
        -> local_mapping_node
        -> /perception/accumulated_local_map
        -> /perception/accumulated_local_map_debug/compressed
```

The first accumulated map should be a rolling local map around the hero vehicle.

### 4. Create local-occupancy navigation mode

The current refined navigation node uses:

```text
/perception/free_space_status
```

A future navigation node should use:

```text
/perception/local_occupancy_status
```

This would create three showcase modes:

```text
primitive reactive navigation
image-space free-space navigation
local-occupancy navigation
```

### 5. Use better camera projection

A later version should replace the heuristic projection with:

```text
camera intrinsics
camera extrinsics
ground-plane projection
metric or scale-corrected depth
```

---

## Relation to the Project Roadmap

This milestone marks the transition from perception and free-space estimation toward mapping.

Previous milestone:

```text
semantic segmentation + depth -> free-space and obstacle masks
```

Current milestone:

```text
semantic segmentation + depth -> local occupancy grid
```

Next milestone:

```text
local occupancy grid + hero odometry -> accumulated local map
```

Longer-term roadmap:

```text
hero odometry / visual odometry
        -> accumulated local map
        -> local planning
        -> SLAM-like prototype
```

---

## Technical Contribution

This milestone adds a map-like intermediate representation to the project.

The contribution is not a new SLAM algorithm, but a practical bridge between deep-learning perception and local spatial reasoning.

It demonstrates how semantic segmentation and monocular depth can be converted into a local occupancy representation that is:

```text
interpretable
visualizable
usable for navigation decisions
extendable toward mapping
```

---

## Recommended Commit Message

```bash
git add ros/ros2_ws/src/local_occupancy_node docs/milestones/local_occupancy_mapping.md
git commit -m "Document local occupancy mapping milestone"
git push
```
