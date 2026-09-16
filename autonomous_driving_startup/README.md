# Autonomous Driving Startup Scripts

Local orchestration for the CARLA + ROS2 development stack.

## Design

These launchers preserve the working structure of the previous scripts:

- `tmux` runs on the **local workstation**.
- Each pane opens SSH to `limoneros.inf.um.es:32122`.
- ROS/CARLA processes run remotely inside that SSH pane.
- No remote `tmux`, `sudo`, `nohup`, systemd, or `/opt/ros/...` sourcing.
- Remote environments use:
  - `mamba activate ros2seg`
  - `mamba activate ros2depth`
- A single local SSH ControlMaster is shared across all launcher sessions.

## Scripts

| Script | Local tmux session | Responsibility |
|---|---|---|
| `00_stack.sh` | — | Start/status/stop multiple launcher sessions |
| `01_sim.sh` | `carla_sim` | CARLA, bridge, traffic, optional legacy navigation |
| `02_perception.sh` | `carla_perception` | Semantic segmentation, object detection, tracking, overlay |
| `03_depth_fusion.sh` | `carla_depth_fusion` | Monocular depth and object/depth fusion |
| `04_spatial.sh` | `carla_spatial` | Free-space, local occupancy, legacy spatial mapping |
| `05_lanes.sh` | `carla_lanes` | Road markings → BEV → filtering → curves → tracking → LaneMap |
| `06_planning.sh` | `carla_planning` | Global planner, route/TF/graph visualization, Foxglove frame adapter, diagnostics/goal shell |
| `07_foxglove.sh` | `carla_foxglove` | Foxglove bridge |
| `05_lanes_foxglove.sh` | two sessions | Compatibility wrapper for lanes + Foxglove |

## Current milestone

As of 2026-09-16, global routing and route-ready LaneMap are implemented; RoutePlan ↔ LaneMap association is NEXT, not implemented. For that work, run from this directory on the local workstation:

```bash
./00_stack.sh core
```

This starts:

1. simulation / CARLA bridge
2. lane perception + temporal LaneMap
3. global route planner and four planning/visualization adapters
4. Foxglove

Object perception, depth/fusion, and legacy spatial mapping are not required for
the current RoutePlan ↔ LaneMap work.

For every subsystem:

```bash
./00_stack.sh full
```

## Individual launch

Each launcher still works independently:

```bash
./01_sim.sh
./05_lanes.sh
./06_planning.sh
./07_foxglove.sh
```

By default it attaches to its tmux session. To start without attaching:

```bash
ATTACH=0 ./05_lanes.sh
```

## Status / attach / stop

```bash
./00_stack.sh status
./00_stack.sh attach carla_lanes
./00_stack.sh stop
./00_stack.sh close-ssh
```

`stop` only kills the local tmux sessions created by these launchers. The shared
SSH master is intentionally retained for quick restart; `close-ssh` closes it.

## tmux

Inside an attached session:

- `Ctrl+B`, then `n`: next window
- `Ctrl+B`, then `p`: previous window
- `Ctrl+B`, then `d`: detach without stopping processes

## Startup behavior

The revised lane/planning scripts use topic-aware dependency waits for known
interfaces rather than relying entirely on large fixed delays. For example:

- lane BEV waits for road-marking mask + camera info
- curve fitting waits for candidate mask
- tracking waits for curve status + odometry
- LaneMap waits for tracking status + odometry
- global planning waits for odometry

This makes restarts less dependent on machine load and model initialization
time.

`wait_topic` in `lib/common.sh` checks topic discovery, not receipt of a valid message. `status` reports local tmux session existence, not node health. Use the bounded message checks in the [runbook](../docs/ros/runbook.md) to validate data flow.

The launcher environment split is explicit: `02_perception.sh` uses `ros2seg` for semantic segmentation, object detection/tracking, and overlay. The bridge, depth/fusion, spatial, lane (including road-marking segmentation), planning/visualization, and Foxglove launchers use `ros2depth`. The remote repository is `~/vision-segmentation-autonomous-driving`, workspace is its `ros/ros2_ws`, CARLA is `~/CARLA_0.9.16`, and Foxglove has an additional `~/fox_ws` overlay. See [setup](../docs/setup.md) for model and environment assumptions.

## Lane frame convention

The lane tracker/mapper continue to receive:

```text
odom_y_sign = -1.0
odom_yaw_sign = -1.0
```

These are part of the existing internal ego/world convention. The current
mapper source fix converts the published vector LaneMap back to direct
`carla_world`; the startup scripts do not add any downstream coordinate
workaround.

## Global planner

`06_planning.sh` creates two windows in local session `carla_planning`:

| Window | Package / executable | ROS node name / role |
| --- | --- | --- |
| `planning` | `global_route_planner global_route_planner_node` | `global_route_planner`: OpenDRIVE graph + A* → RoutePlan |
| `planning` | `planning_visualization route_plan_visualizer_node` | `route_plan_visualizer`: RoutePlan → Path |
| `planning` | `planning_visualization odom_tf_broadcaster_node` | `odom_tf_broadcaster`: production `carla_world → hero` TF |
| `visualization` | `planning_visualization global_lane_graph_visualizer_node` | `global_lane_graph_visualizer`: full static routing graph markers |
| `visualization` | `planning_visualization foxglove_world_visualizer_node` | `foxglove_world_visualizer`: separate display frame/topics |

The `planning` window also has a diagnostic pane; `visualization` has an interactive goal/validation shell.

The planner waits for `/planning/goal`. A previously validated test goal is printed in the interactive goal shell in the `visualization` window, but no goal is automatically sent. A route only appears after `/planning/goal` is published and ego/goal association permits a search; an absent route immediately after restart is expected. Goal orientation is ignored as a routing constraint. A rejected goal leaves the previous accepted goal active.

Supplied development runtime evidence records a clean `./00_stack.sh stop` → `./00_stack.sh full` restart with exactly one instance of each of the five nodes above. This is one observed restart, not a general guarantee that killing local sessions cleans up every possible remote SSH failure. It was not rerun in the documentation pass. The [global-routing milestone](../docs/milestones/global_route_planning.md) records the observed topic/frame chain.

## Foxglove config

Copy the included file to the remote ROS workspace config location if needed:

```text
ros/ros2_ws/config/foxglove_bridge.yaml
```

The whitelist now includes:

```text
^/planning/.*
```

so RoutePlan and later planning interfaces are visible in Foxglove.

The current config allows planning/perception/TF topic inspection but disables client topic publication; send development goals from the ROS shell. `07_foxglove.sh` sources `~/fox_ws/install/setup.bash`, then the main ROS workspace, and uses the main workspace's config file.

Validated Foxglove 3D setup:

```text
Fixed frame:   carla_world_viz
Display frame: hero_viz
Global graph:  /planning/global_lane_graph/markers_viz
Route:         /planning/route_path_viz
Local lanes:   /perception/viz/lane/*
```

Production world topics remain in `carla_world`; original local lane paths use `hero` with the tracker's forward-left convention. The adapter reflects global display y/orientation and builds `carla_world_viz → hero_viz`. Local lane points are already forward-left, so their aliases only change frame labels to `hero_viz`, without mirroring again. These frames are visualization-only, never planner inputs. See the [coordinate contract](../docs/agent/ARCHITECTURE.md).

## Configuration overrides

Connection defaults may be overridden locally:

```bash
SERVER_HOST=... SERVER_USER=... SERVER_PORT=... ./01_sim.sh
```

Common runtime parameters in `05_lanes.sh` can also be overridden through
environment variables, preserving the previous scripts' behavior.

## Temporary diagnostics

One-off scripts such as:

```text
/tmp/route_lane_frame_probe.py
```

remain temporary diagnostics and are intentionally not part of this package.
