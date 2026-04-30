# Reactive Navigation Milestone

## Current Working Behavior

The reactive navigation node consumes:

```text
/perception/fused_objects
/perception/depth/image
```

and publishes:

```text
/carla/cmd_vel
```

The current behavior is:

- cruise forward when the path appears clear;
- slow down when a close obstacle is detected;
- reverse and turn when a very close obstacle appears ahead;
- use fused object depth when available;
- use a mid-image raw depth ROI fallback to detect walls and structures that are not captured as tracked objects.

This milestone is important because it closes the first perception-to-action loop:

```text
CARLA RGB
  -> segmentation
  -> detections
  -> tracking
  -> depth
  -> fusion
  -> reactive navigation
  -> /carla/cmd_vel
```

---

## Known-Good Test Parameters

The following parameters produced decent behavior after steering fixes and ROI tuning:

```bash
ros2 run reactive_navigation_node reactive_navigation_node --ros-args \
  -p cruise_speed:=0.20 \
  -p slow_speed:=0.10 \
  -p reverse_speed:=-0.18 \
  -p roi_x_min_ratio:=0.30 \
  -p roi_x_max_ratio:=0.70 \
  -p roi_y_min_ratio:=0.25 \
  -p roi_y_max_ratio:=0.60 \
  -p roi_close_thresh:=40.0 \
  -p roi_danger_thresh:=60.0 \
  -p fused_close_thresh:=60.0 \
  -p fused_danger_thresh:=80.0
```

These parameters should be considered the current baseline for future testing.

---

## Behavior Description

### Clear Path

When no close obstacle is detected, the node publishes a low forward cruising command:

```text
linear.x = cruise_speed
angular.z = 0
```

With the current baseline:

```text
cruise_speed = 0.20
```

### Close Obstacle

When an obstacle is detected in the depth ROI or fused-object stream but is not yet extremely close, the vehicle slows down and steers away.

Current slow speed:

```text
slow_speed = 0.10
```

### Dangerous Obstacle Ahead

When a very close obstacle appears in front, the node enters a recovery maneuver:

1. reverse while turning;
2. then move forward slowly while continuing the turn;
3. return to cruising mode.

This behavior was added because simply stopping is not enough when the vehicle gets stuck against walls, barriers or map structures.

---

## Important Implementation Notes

### 1. Steering Sign Fix

The steering sign used by the reactive navigation node had to be corrected to match the behavior of the CARLA bridge.

The original steering direction was inverted.

### 2. Reverse Steering Sign Fix

Reverse motion required a separate steering sign correction.

Reason:

```text
the same steering command produces different apparent path behavior when moving backward
```

Therefore, reverse-and-turn behavior needed its own sign adjustment.

### 3. Raw Depth ROI Fallback

The first reactive version only reacted to fused tracked objects.

This failed when the vehicle encountered walls or structures that were not published as fused objects.

To solve this, the node was extended to also subscribe to:

```text
/perception/depth/image
```

and compute a proximity signal from a region of interest in the depth image.

### 4. Road False Positives

The first raw depth ROI used too much of the lower image region.

Problem:

```text
the bottom part of the image contains mostly near road surface
```

This made the node interpret the road itself as an obstacle.

Fix:

```text
move the ROI upward into a forward-looking mid-image band
```

Current ROI:

```text
roi_x_min_ratio = 0.30
roi_x_max_ratio = 0.70
roi_y_min_ratio = 0.25
roi_y_max_ratio = 0.60
```

This focuses on the region ahead of the vehicle rather than the immediate road surface.

### 5. Depth Convention

With the current Depth Anything output and normalization, the observed working convention is:

```text
larger depth value = closer
```

This is important because threshold logic depends on the depth convention.

---

## Current Limitations

### 1. Reactive, Not Planned

This node is purely reactive. It does not perform trajectory planning.

It only reacts to what is currently visible.

### 2. No Lane Understanding

The node does not yet understand lane geometry, road boundaries or route following.

It may avoid obstacles but does not know where it should ideally drive.

### 3. No Semantic Road Masking Yet

The raw depth ROI still does not explicitly remove road or sidewalk pixels.

A better future version should combine:

```text
depth image + semantic mask
```

to ignore:

```text
road
sidewalk
sky
terrain
```

and focus on likely obstacles.

### 4. Empirical Thresholds

The current thresholds are empirical and depend on the output scale of Depth Anything.

They may change if:

- the model changes;
- input resolution changes;
- depth normalization changes;
- the camera position changes.

### 5. No Local Memory

The node reacts frame by frame.

It does not remember obstacles after they leave the field of view.

This will be addressed later with local mapping.

---

## Suggested Next Improvements

### 1. Close-Pixel-Ratio Criterion

Instead of triggering only from a percentile depth value, require a minimum fraction of close pixels in the ROI.

Example:

```text
if at least 20 percent of ROI pixels are close:
    obstacle detected
```

This would reduce false positives from isolated noisy pixels.

### 2. Semantic-Mask-Aware ROI

Use the semantic segmentation mask to ignore pixels belonging to:

```text
road
sidewalk
sky
terrain
```

and keep pixels belonging to likely obstacles:

```text
wall
fence
pole
building
car
truck
bus
person
rider
bicycle
motorcycle
```

This would make obstacle detection more reliable.

### 3. Temporal Smoothing

Smooth the ROI proximity signal across time:

```text
proximity_smooth = alpha * proximity_previous + (1 - alpha) * proximity_current
```

This would reduce jitter and unstable switching between cruise, slow and reverse modes.

### 4. Configurable Steering Signs

Add ROS parameters for:

```text
forward_steering_sign
reverse_steering_sign
```

This would avoid editing code every time the bridge sign convention changes.

### 5. Local Free-Space Map

The next major architectural improvement should be a local free-space or occupancy map.

Instead of directly steering from the raw depth ROI, the stack should estimate:

```text
free space
obstacle space
unknown space
```

Then a controller can select safer commands from this local representation.

---

## Relation to the Project Roadmap

This milestone closes the first basic loop:

```text
perception -> scene understanding -> control
```

It is not yet full autonomous driving, but it is a meaningful step toward it.

The next roadmap stage should be:

```text
reactive navigation -> local occupancy/free-space map -> visual odometry -> keyframe mapping -> SLAM-like prototype
```

---

