# Semantic-Depth Free-Space Estimation Milestone

## Overview

This milestone documents the integration of a semantic-depth free-space estimation module into the CARLA + ROS2 autonomous-driving perception stack.

At this stage, the project has moved beyond object-level perception and has started to build a first intermediate representation of the drivable environment:

```text
semantic segmentation + monocular depth
        -> free-space estimation
        -> obstacle estimation
        -> navigation-ready scene abstraction
```

The free-space node is intended to act as a bridge between perception and future local mapping. It produces simple binary visualizations and a structured status message describing which parts of the image appear free, blocked, or preferable for navigation.

---

## Current Perception Stack

The current working stack is:

```text
CARLA RGB camera
    |
    v
semantic_seg_node
    |
    |-- /perception/semantic_mask
    |-- /perception/semantic_overlay/compressed
    |
    v
object_detection_node
    |
    v
tracking_node
    |
    v
/perception/tracks


CARLA RGB camera
    |
    v
depth_node
    |
    |-- /perception/depth/image
    |-- /perception/depth/colormap/compressed


/perception/tracks + /perception/depth/image
    |
    v
fusion_node
    |
    v
/perception/fused_objects


/perception/semantic_mask + /perception/depth/image
    |
    v
free_space_node
    |
    |-- /perception/free_space_mask/compressed
    |-- /perception/obstacle_mask/compressed
    |-- /perception/free_space_status
```

This milestone focuses on the last block:

```text
semantic mask + depth image -> free-space and obstacle estimation
```

---

## Motivation

The previous reactive navigation node used:

```text
/perception/fused_objects
/perception/depth/image
```

This worked for basic obstacle avoidance, but it had limitations:

- walls or large structures were not always detected as tracked objects;
- raw depth alone could treat the road surface as an obstacle;
- object-level fusion is not enough to describe drivable space;
- future mapping requires a more spatial representation than a list of objects.

The free-space node addresses this by explicitly estimating:

```text
Where is the road?
Where are obstacles?
Which side of the image appears safer?
Is the center path clear?
```

This is a necessary intermediate step before local occupancy mapping.

---

## Implemented Node

## `free_space_node`

### Role

The `free_space_node` combines semantic segmentation and monocular depth to estimate free space and obstacle regions.

### Inputs

```text
/perception/semantic_mask
/perception/depth/image
```

### Outputs

```text
/perception/free_space_mask/compressed
/perception/obstacle_mask/compressed
/perception/free_space_status
```

---

## Input: Semantic Mask

The semantic segmentation node publishes:

```text
/perception/semantic_mask
```

as a ROS2 image:

```text
type: sensor_msgs/msg/Image
encoding: mono8
height: 600
width: 800
```

Each pixel stores the Cityscapes-19 class ID.

Relevant class IDs:

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

The free-space node currently treats:

```text
road -> free space
```

and the following classes as non-drivable or obstacle-like:

```text
sidewalk
wall
fence
pole
person
rider
car
truck
bus
train
motorcycle
bicycle
```

The sidewalk was added as a semantic obstacle after observing that walkpaths were only detected as obstacles inside the depth ROI. By adding sidewalk as a semantic obstacle, walkpaths are now treated consistently across the entire image.

---

## Input: Depth Image

The depth node publishes:

```text
/perception/depth/image
```

as a raw relative depth image:

```text
type: sensor_msgs/msg/Image
encoding: 32FC1
```

The depth comes from Depth Anything V2.

Important convention observed in the current system:

```text
larger depth value = closer
```

This is not metric depth in meters. It is a relative depth signal useful for ranking proximity and detecting close regions.

---

## Output: Free-Space Mask

The free-space mask is published as:

```text
/perception/free_space_mask/compressed
```

Visualization convention:

```text
green = free / drivable region
black = not free or unknown
```

At this stage, the free-space mask is mainly based on road segmentation:

```text
free_space = road pixels - obstacle pixels
```

This produces a clear visual indication of where the road is in the current camera frame.

---

## Output: Obstacle Mask

The obstacle mask is published as:

```text
/perception/obstacle_mask/compressed
```

Visualization convention:

```text
red = obstacle / non-drivable / blocked region
black = free or unknown
```

Obstacle evidence comes from two sources:

### 1. Semantic obstacles

Classes such as sidewalk, wall, fence, pole, vehicles, pedestrians, cyclists and motorcyclists are directly marked as obstacles.

### 2. Close-depth obstacles

The depth image is used to detect close unknown objects in a forward-looking region of interest.

This is useful because some relevant structures may not be detected as tracked objects.

---

## Region of Interest

The free-space node uses a forward-looking ROI for depth-based obstacle detection.

The current working ROI is approximately:

```text
roi_x_min_ratio = 0.30
roi_x_max_ratio = 0.70
roi_y_min_ratio = 0.25
roi_y_max_ratio = 0.60
```

This ROI was chosen after observing that using the bottom part of the image caused the road itself to be treated as a close obstacle.

The key lesson was:

```text
The lower image region is dominated by near road pixels.
Raw depth obstacle detection should avoid relying too much on that region.
```

The updated ROI focuses on the mid-image area ahead of the vehicle.

---

## Output: Free-Space Status

The node publishes a structured JSON message on:

```text
/perception/free_space_status
```

The status summarizes the free-space and obstacle ratios in the left, center and right parts of the ROI.

Example structure:

```json
{
  "stamp": {
    "sec": 123,
    "nanosec": 456789
  },
  "frame_id": "carla_camera",
  "roi": {
    "x_min_ratio": 0.3,
    "x_max_ratio": 0.7,
    "y_min_ratio": 0.25,
    "y_max_ratio": 0.6
  },
  "free_ratio": {
    "left": 0.42,
    "center": 0.78,
    "right": 0.51
  },
  "obstacle_ratio": {
    "left": 0.31,
    "center": 0.08,
    "right": 0.24
  },
  "recommended_direction": "forward"
}
```

The recommended direction can be:

```text
forward
left
right
slow
stop
```

This is not yet a full planning output, but it provides a cleaner input for future navigation modules.

---

## Current Behavior

The current free-space node successfully:

- detects the main road as free space;
- marks sidewalks and walkpaths as non-drivable;
- marks vehicles and structural obstacles when detected semantically;
- uses depth to catch close regions in the forward-looking ROI;
- publishes compressed masks suitable for Foxglove;
- publishes a JSON status message useful for future reactive navigation.

The current Foxglove visualization shows:

```text
RGB / detection overlay
semantic overlay
depth colormap
free-space mask
obstacle mask
fused object JSON
```

The result is a much clearer intermediate representation than raw segmentation or raw depth alone.

---

## Important Fixes Made

### 1. Sidewalk / Walkpath Handling

Initial behavior:

```text
walkpath inside depth ROI     -> obstacle
walkpath outside depth ROI    -> not obstacle
```

Cause:

```text
The walkpath was only being marked as an obstacle through depth-based ROI logic.
```

Fix:

```text
Add Cityscapes class 1, sidewalk, to semantic obstacle classes.
```

This made walkpaths non-drivable everywhere, not only inside the rectangular ROI.

---

### 2. Avoiding Road-as-Obstacle False Positives

Initial behavior:

```text
the road was sometimes treated as a close obstacle
```

Cause:

```text
The bottom part of the image contains near road pixels with high relative depth values.
```

Fix:

```text
Move the depth ROI upward and use semantic road masking.
```

Current logic:

```text
depth obstacle = close depth AND inside ROI AND not road
```

This avoids interpreting the road surface itself as an obstacle.

---

### 3. Compressed Visualization

The masks are published as compressed images:

```text
/perception/free_space_mask/compressed
/perception/obstacle_mask/compressed
```

This reduces Foxglove lag and makes the system easier to monitor remotely.

---

## How to Run

Make sure these nodes are already running:

```text
carla_bridge_node
semantic_seg_node
depth_node
```

Then run:

```bash
cd ~/vision-segmentation-autonomous-driving/ros/ros2_ws
mamba activate ros2depth
source install/setup.bash

ros2 run free_space_node free_space_node
```

If using tuned ROI parameters explicitly:

```bash
ros2 run free_space_node free_space_node --ros-args \
  -p roi_x_min_ratio:=0.30 \
  -p roi_x_max_ratio:=0.70 \
  -p roi_y_min_ratio:=0.25 \
  -p roi_y_max_ratio:=0.60 \
  -p close_depth_thresh:=40.0
```

---

## Useful Checks

Check available topics:

```bash
ros2 topic list | grep free_space
ros2 topic list | grep obstacle
```

Check status:

```bash
ros2 topic echo /perception/free_space_status --once
```

Check publication rate:

```bash
ros2 topic hz /perception/free_space_mask/compressed
ros2 topic hz /perception/obstacle_mask/compressed
```

---

## Foxglove Topics

Recommended topics for visualization:

```text
/carla/rgb/image_raw/compressed
/perception/semantic_overlay/compressed
/perception/detections_overlay/compressed
/perception/depth/colormap/compressed
/perception/free_space_mask/compressed
/perception/obstacle_mask/compressed
/perception/fused_objects
/perception/free_space_status
/carla/cmd_vel
```

Example command:

```bash
ros2 run foxglove_bridge foxglove_bridge --ros-args \
  -p topic_whitelist:="[/carla/rgb/image_raw/compressed,/perception/semantic_overlay/compressed,/perception/detections_overlay/compressed,/perception/depth/colormap/compressed,/perception/free_space_mask/compressed,/perception/obstacle_mask/compressed,/perception/fused_objects,/perception/free_space_status,/carla/cmd_vel]"
```

---

## Current Limitations

### 1. Image-Space Representation

The free-space mask is still an image-space representation.

It does not yet provide a metric bird's-eye-view occupancy grid.

### 2. No Temporal Memory

The node only analyzes the current frame.

It does not remember obstacles that leave the camera view.

### 3. No Ego-Motion Compensation

The system does not yet use odometry or visual odometry.

Therefore, it cannot accumulate free-space observations over time.

### 4. Relative Depth Only

Depth Anything provides relative depth, not metric distance.

This limits the precision of obstacle distance estimation.

### 5. No Lane or Route Understanding

The node estimates free space but does not yet understand:

- lanes;
- traffic rules;
- desired route;
- road topology.

### 6. Semantic Errors Propagate

If semantic segmentation misclassifies road, sidewalk, or vehicles, the free-space mask can also be wrong.

---

## Recommended Next Improvements

### 1. Make Reactive Navigation Consume Free-Space Status

The reactive navigation node should eventually use:

```text
/perception/free_space_status
```

instead of directly using raw depth ROI logic.

This would make control cleaner:

```text
if recommended_direction == forward:
    cruise
elif recommended_direction == left:
    steer left
elif recommended_direction == right:
    steer right
elif recommended_direction == stop:
    stop or reverse
```

### 2. Add Close-Pixel-Ratio Logic

Instead of relying only on thresholded depth values, use ratios:

```text
if obstacle_ratio_center > threshold:
    center blocked
```

This is already partially supported by the status output and should be expanded.

### 3. Add Semantic-Debug Masks

Publish separate debug masks:

```text
/perception/semantic_obstacle_mask/compressed
/perception/depth_obstacle_mask/compressed
/perception/obstacle_mask/compressed
```

This would make it easier to distinguish semantic errors from depth-based errors.

### 4. Add Temporal Smoothing

Smooth free-space ratios:

```text
ratio_smooth = alpha * previous_ratio + (1 - alpha) * current_ratio
```

This would reduce jitter in the recommended direction.

### 5. Move Toward Local Occupancy Mapping

The next major milestone should transform image-space masks into a local spatial representation:

```text
image-space free-space mask + depth + camera model
        -> approximate local occupancy grid
```

This will be the bridge toward mapping and SLAM-like functionality.

---

## Relation to the Project Roadmap

This milestone moves the project from object-aware reactive behavior toward spatial scene understanding.

Previous milestone:

```text
tracking + depth -> fused objects -> reactive navigation
```

Current milestone:

```text
segmentation + depth -> free-space / obstacle masks
```

Next roadmap stage:

```text
free-space estimation -> local occupancy grid -> visual odometry -> keyframe mapping -> SLAM-like prototype
```

The free-space node is therefore a necessary bridge between perception and mapping.

---

## Technical Significance

Before this milestone, the system could identify and track objects and estimate their relative depth.

After this milestone, the system can estimate a first approximation of:

```text
where the vehicle can drive
where obstacles are located
which side of the image seems safer
```

This is a major step toward local navigation and mapping.


