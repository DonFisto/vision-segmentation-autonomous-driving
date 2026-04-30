# Free-Space Navigation Milestone

## Overview

This milestone documents the refined navigation node based on the semantic-depth free-space representation.

Unlike the primitive reactive navigation node, which directly consumes fused objects and raw depth, this refined node consumes:

```text
/perception/free_space_status
```

and publishes:

```text
/carla/cmd_vel
```

This creates a cleaner perception-to-control chain:

```text
semantic segmentation + depth
        ↓
free_space_node
        ↓
/perception/free_space_status
        ↓
free_space_navigation_node
        ↓
/carla/cmd_vel
```

## Known-Good Parameters

The following parameters produced good behavior:

```bash
ros2 run free_space_navigation_node free_space_navigation_node --ros-args \
  -p cruise_speed:=0.30 \
  -p slow_speed:=0.20 \
  -p reverse_speed:=-0.80 \
  -p steer_value:=0.45 \
  -p center_obstacle_stop:=0.35 \
  -p center_free_cruise:=0.55
```

## Behavior

The node uses the free-space status output:

```text
free_ratio.left
free_ratio.center
free_ratio.right

obstacle_ratio.left
obstacle_ratio.center
obstacle_ratio.right

recommended_direction
```

Basic behavior:

```text
recommended_direction = forward → cruise
recommended_direction = left    → slow + steer left
recommended_direction = right   → slow + steer right
recommended_direction = slow    → slow forward
recommended_direction = stop    → reverse + recovery turn
```

A safety override triggers recovery when:

```text
center_obstacle_ratio >= center_obstacle_stop
```

With the current known-good configuration:

```text
center_obstacle_stop = 0.35
center_free_cruise  = 0.55
```

## Why This Is Better Than the Primitive Node

The primitive node used:

```text
/perception/fused_objects
/perception/depth/image
```

directly.

The refined node uses an intermediate representation:

```text
/perception/free_space_status
```

This makes the architecture cleaner because navigation no longer duplicates low-level depth ROI logic.

## Showcase Modes

The project now has two navigation modes.

### Primitive navigation

```text
fused objects + depth → reactive_navigation_node → /carla/cmd_vel
```

### Refined navigation

```text
semantic-depth free-space status → free_space_navigation_node → /carla/cmd_vel
```

Do not run both at the same time, because both publish to:

```text
/carla/cmd_vel
```

## Current Limitations

- The navigation is still reactive.
- It does not perform trajectory planning.
- It does not use a metric occupancy grid.
- It depends on the quality of the free-space mask.
- It does not yet use temporal local mapping.
- It does not follow routes or lanes explicitly.

## Next Step

The next major step is:

```text
free-space mask + depth + camera model → local occupancy grid
```

This will move the project from image-space navigation toward local mapping.


