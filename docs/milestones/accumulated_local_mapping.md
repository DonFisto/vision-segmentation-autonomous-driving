# Accumulated Local Mapping Milestone

## Overview

This milestone documents the first accumulated local mapping prototype implemented in the CARLA + ROS2 vision-based autonomous driving project.

The objective of this stage is to move beyond single-frame perception and build a short-term map-like representation of the environment around the hero vehicle.

The accumulated local mapping node combines:

```text
CARLA hero odometry
+ local occupancy grid
+ static obstacle layer
+ dynamic obstacle layer
        -> accumulated local map
```

This is not full SLAM yet. It is a ground-truth-odometry-based local mapping prototype that demonstrates how frame-by-frame perception can be accumulated over time.

---

## Position in the Project Stack

Before this milestone, the project already had:

```text
CARLA RGB camera
    -> semantic segmentation
    -> object extraction from segmentation
    -> tracking
    -> monocular depth estimation
    -> object-depth fusion
    -> free-space estimation
    -> local occupancy grid
```

This milestone adds:

```text
/carla/hero_odom
/perception/local_occupancy_grid
/perception/local_static_obstacle_grid
/perception/local_dynamic_obstacle_grid
        -> local_mapping_node
        -> /perception/accumulated_local_map
        -> /perception/accumulated_static_map
        -> /perception/accumulated_dynamic_map
        -> /perception/accumulated_local_map_debug/compressed
        -> /perception/local_mapping_status
```

The system therefore moves from:

```text
What do I see right now?
```

to:

```text
What have I seen recently, and where is it located relative to the hero vehicle?
```

---

## Implemented Node

## `local_mapping_node`

### Role

The `local_mapping_node` accumulates local occupancy observations over time using the hero vehicle odometry provided by CARLA.

It receives local grid observations from the current camera frame and projects them into a larger accumulated map in CARLA world coordinates.

### Inputs

```text
/carla/hero_odom
/perception/local_occupancy_grid
/perception/local_static_obstacle_grid
/perception/local_dynamic_obstacle_grid
```

### Outputs

```text
/perception/accumulated_local_map
/perception/accumulated_static_map
/perception/accumulated_dynamic_map
/perception/accumulated_local_map_debug/compressed
/perception/local_mapping_status
```

---

## Input: Hero Odometry

The bridge node publishes:

```text
/carla/hero_odom
```

with type:

```text
nav_msgs/msg/Odometry
```

The odometry message contains:

```text
hero position in CARLA world coordinates
hero orientation as quaternion
linear velocity
angular velocity
```

The frame convention used is:

```text
frame_id: carla_world
child_frame_id: hero
```

This odometry is currently ground-truth motion from CARLA, not estimated visual odometry.

Using CARLA odometry is useful because it provides a reliable reference for testing local map accumulation before implementing visual odometry.

---

## Input: Local Occupancy Layers

The local occupancy node publishes three relevant grids:

```text
/perception/local_occupancy_grid
/perception/local_static_obstacle_grid
/perception/local_dynamic_obstacle_grid
```

The combined local occupancy grid uses:

```text
-1   unknown
 0   free
100  occupied
```

The static and dynamic layers separate the meaning of occupied cells:

```text
static obstacle:
  sidewalks, buildings, walls, fences, poles, vegetation, terrain

dynamic obstacle:
  people, riders, cars, trucks, buses, trains, motorcycles, bicycles
```

This separation is important because static obstacles can be accumulated persistently, while dynamic obstacles should decay or remain temporary.

---

## Output: Accumulated Local Map

The main accumulated map is published as:

```text
/perception/accumulated_local_map
```

with type:

```text
nav_msgs/msg/OccupancyGrid
```

Cell convention:

```text
-1   unknown
 0   free
100  occupied
```

This map combines accumulated free, static, and recent dynamic evidence into one map.

---

## Output: Static and Dynamic Accumulated Maps

The node also publishes separated accumulated layers:

```text
/perception/accumulated_static_map
/perception/accumulated_dynamic_map
```

The static map is intended to represent persistent world structure:

```text
road boundaries
sidewalks
walls
buildings
fences
other non-drivable areas
```

The dynamic map is intended to represent recent moving or temporary obstacles:

```text
cars
pedestrians
cyclists
motorcycles
traffic participants
```

Dynamic evidence decays over time so that moving objects do not permanently pollute the map.

---

## Output: Debug Image

The debug visualization is published as:

```text
/perception/accumulated_local_map_debug/compressed
```

Visualization convention:

```text
gray   = unknown
green  = accumulated free space
red    = accumulated static obstacle / non-drivable area
orange = recent dynamic obstacle
blue   = hero vehicle position and heading
```

The debug image is the main tool used to validate whether local occupancy observations are being accumulated consistently as the vehicle moves.

---

## Output: Mapping Status

The status topic is:

```text
/perception/local_mapping_status
```

It is published as JSON inside a `std_msgs/String`.

It reports:

```text
map size
resolution
map origin
free / occupied / static / dynamic / unknown ratios
last update age
coordinate sign parameters
```

This is useful for debugging whether the mapping node is receiving data and updating the map.

---

## Known-Good Runtime Parameters

The following parameters produced the best accumulated local mapping behavior during testing:

```bash
ros2 run local_mapping_node local_mapping_node --ros-args \
  -p map_size_m:=300.0 \
  -p resolution:=0.25 \
  -p local_forward_m:=18.0 \
  -p local_width_m:=10.0 \
  -p yaw_sign:=1.0 \
  -p lateral_sign:=1.0 \
  -p static_hit_inc:=1.5 \
  -p static_occupied_thresh:=6.0 \
  -p free_dec:=2.0 \
  -p free_thresh:=-2.5 \
  -p dynamic_hit_inc:=5.0 \
  -p dynamic_decay:=0.80 \
  -p dynamic_occupied_thresh:=3.0 \
  -p pixel_scale:=2
```

The matching local occupancy parameters were:

```bash
ros2 run local_occupancy_node local_occupancy_node --ros-args \
  -p roi_x_min_ratio:=0.10 \
  -p roi_x_max_ratio:=0.90 \
  -p roi_y_min_ratio:=0.25 \
  -p roi_y_max_ratio:=0.95 \
  -p static_y_min_ratio:=0.55 \
  -p forward_m:=18.0 \
  -p width_m:=10.0 \
  -p resolution:=0.25 \
  -p far_power:=1.3 \
  -p pixel_stride:=3 \
  -p static_dilate_cells:=0 \
  -p dynamic_dilate_cells:=1 \
  -p free_dilate_cells:=1 \
  -p use_depth_obstacles:=False
```

---

## Main Design Decisions

### 1. Use CARLA odometry before visual odometry

The mapping node currently uses ground-truth odometry from CARLA.

This is intentional.

Before estimating motion from images, it is useful to validate whether the local occupancy grids can be accumulated correctly when accurate motion is available.

This separates two problems:

```text
mapping from occupancy grids
motion estimation
```

Once the mapping system is stable, visual odometry can be introduced later and compared against CARLA odometry.

---

### 2. Separate static and dynamic obstacle layers

The project originally used one generic occupied class.

This was not enough for mapping.

Static and dynamic obstacles need different update rules:

```text
static obstacles:
  accumulate slowly and persistently

dynamic obstacles:
  appear strongly but decay quickly

free space:
  clears false obstacle evidence
```

Without this separation, moving cars or pedestrians would leave permanent ghost obstacles in the accumulated map.

---

### 3. Avoid depth-based promotion of unknown static obstacles

An earlier version promoted unknown close pixels to static obstacles using depth.

This caused false red regions in the front part of the local occupancy grid.

The reason was that heuristic image-to-ground projection can incorrectly project vertical or noisy pixels into the road ahead.

The final stable configuration disables this behavior:

```text
use_depth_obstacles = False
```

Semantic class now decides whether a cell is free, static, dynamic, or unknown.

---

### 4. Filter static vertical objects with `static_y_min_ratio`

Static classes such as buildings, walls, vegetation, and poles often occupy the upper image region.

Projecting all of those pixels into the ground plane caused false static obstacles in front of the vehicle.

The solution was to only project static classes from a lower part of the image:

```text
static_y_min_ratio = 0.55
```

This means that upper-image static pixels are treated more conservatively and are not projected into the local ground map.

This was one of the most important practical fixes.

---

### 5. Treat debug display orientation separately from map projection

A major lesson was that coordinate bugs can occur at different levels:

```text
local occupancy projection
local-to-world projection
CARLA world coordinates
debug image display
```

At one point, the accumulated map looked mirrored even though the underlying local projection was close to correct.

The final result required separating:

```text
map data convention
debug image convention
```

The stable mapping configuration uses:

```text
yaw_sign = 1.0
lateral_sign = 1.0
```

and the debug image display is configured so that the visual turn direction matches the simulator.

---

## Current Behavior

The accumulated local mapping node can now:

```text
receive hero odometry from CARLA
receive local occupancy grids
accumulate free road regions over time
accumulate static non-drivable areas
represent recent dynamic obstacles
display the hero position and heading
generate a useful top-down map-like debug visualization
```

The strongest observed result is that, after tuning, the accumulated map follows the road corridor and turn direction correctly.

This makes the system much more informative than the single-frame local occupancy grid alone.

---

## Current Limitations

### 1. Not full SLAM

The system does not perform loop closure, map optimization, keyframe management, or pose graph optimization.

It is a local accumulated mapping prototype.

### 2. Uses CARLA ground-truth odometry

The current map depends on simulator-provided hero odometry.

This is useful for development, but it is not yet a real visual odometry or SLAM pipeline.

### 3. Heuristic local occupancy projection

The local occupancy grid still uses a heuristic projection from image space to local ground space.

It does not yet use full camera intrinsics, extrinsics, or metric depth projection.

### 4. Fixed map origin

The current accumulated map is initialized around the starting position.

For longer drives, a rolling map centered on the hero vehicle would be better.

### 5. Timestamp synchronization is approximate

The mapping node currently uses the latest available occupancy grid and latest odometry.

A more robust implementation should use timestamp-aware synchronization or interpolation.

### 6. Perception errors still propagate

If semantic segmentation misclassifies road, sidewalk, vehicle, or vegetation, the local occupancy map and accumulated map can still be affected.

More CARLA data collection and model refinement can improve this later.

---

## Practical Lessons Learned

### Local mapping depends on representation quality

The mapping node can only accumulate what the local occupancy node provides.

When the local occupancy grid had false red regions, the accumulated map also accumulated those false obstacles.

Therefore, local map quality depends strongly on:

```text
semantic segmentation quality
local occupancy projection
static/dynamic class split
static object filtering
free-space clearing
```

---

### False static obstacles must be cleared aggressively enough

Static obstacle evidence should persist, but false positives must be correctable.

The stable mapping configuration uses:

```text
static_hit_inc = 1.5
static_occupied_thresh = 6.0
free_dec = 2.0
free_thresh = -2.5
```

This makes the map more tolerant to occasional false static detections and allows repeated free-space observations to correct them.

---

### Dynamic obstacles should decay

Dynamic objects should not become permanent map structure.

The current dynamic layer uses:

```text
dynamic_hit_inc = 5.0
dynamic_decay = 0.80
dynamic_occupied_thresh = 3.0
```

This makes dynamic obstacles visible when recently observed, but allows them to fade if they move away.

---

### Visualization is part of debugging

Foxglove visualization was essential for diagnosing:

```text
false static red regions
incorrect local projection
left/right mirroring
debug display inversion
map accumulation behavior
```

The compressed debug topics make the system easier to inspect remotely.

---

## Relation to the Project Roadmap

This milestone completes the first perception-to-mapping chain:

```text
RGB camera
    -> semantic segmentation
    -> depth estimation
    -> local occupancy grid
    -> hero odometry
    -> accumulated local map
```

Previous milestone:

```text
semantic segmentation + depth -> local occupancy grid
```

Current milestone:

```text
local occupancy grid + hero odometry -> accumulated local map
```

Next likely milestone:

```text
rolling local map centered on the hero vehicle
```

Longer-term roadmap:

```text
visual odometry
    -> compare against CARLA odometry
    -> replace ground-truth odometry
    -> keyframe-based local map
    -> SLAM-like prototype
```

---

## Recommended Next Improvements

### 1. Rolling local map

The next mapping improvement should be a rolling map that remains centered around the hero vehicle.

Current behavior:

```text
fixed map initialized around starting position
```

Improved behavior:

```text
rolling local map centered around current hero position
```

Suggested size:

```text
80 m x 80 m
100 m x 100 m
```

This is more suitable for long driving sequences.

### 2. Timestamp-aware odometry integration

The mapping node should eventually use the odometry message closest to the occupancy grid timestamp.

This would reduce mapping artifacts while the vehicle is moving.

### 3. Better image-to-ground projection

The local occupancy projection should eventually use:

```text
camera intrinsics
camera extrinsics
ground-plane assumption
metric or scale-corrected depth
```

This would reduce errors caused by projecting vertical semantic objects into the ground map.

### 4. Visual odometry

A later milestone should implement:

```text
RGB frames -> visual odometry -> estimated hero motion
```

Then compare:

```text
visual odometry trajectory
vs
CARLA ground-truth odometry
```

### 5. Local occupancy navigation

A future navigation node could consume:

```text
/perception/local_occupancy_status
```

or eventually:

```text
/perception/accumulated_local_map
```

This would provide a stronger navigation mode than the current free-space image-space navigation.

---

## Technical Significance

This milestone is important because it turns the project from a perception-only pipeline into an early mapping system.

The contribution is not a new SLAM algorithm, but a clear intermediate autonomy architecture:

```text
deep-learning perception
    -> semantic/depth scene understanding
    -> local occupancy representation
    -> odometry-based map accumulation
```

This is a strong portfolio milestone because it demonstrates not only model inference, but also system integration, spatial reasoning, and practical debugging of coordinate conventions.

---

## Recommended Commit Message

```bash
git add ros/ros2_ws/src/local_mapping_node \
        ros/ros2_ws/src/local_occupancy_node \
        ros/ros2_ws/src/carla_bridge_node \
        docs/milestones/accumulated_local_mapping.md

git commit -m "Document accumulated local mapping milestone"
git push
```
