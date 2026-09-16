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
| `06_planning.sh` | `carla_planning` | OpenDRIVE global route planner and route diagnostics |
| `07_foxglove.sh` | `carla_foxglove` | Foxglove bridge |
| `05_lanes_foxglove.sh` | two sessions | Compatibility wrapper for lanes + Foxglove |

## Current milestone

For the current route-planning / lane-association work, the main stack is:

```bash
./00_stack.sh core
```

This starts:

1. simulation / CARLA bridge
2. lane perception + temporal LaneMap
3. global route planner
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

`06_planning.sh` starts:

```bash
ros2 run global_route_planner global_route_planner_node
```

The planner waits for `/planning/goal`. A previously validated test goal is
printed in the interactive planning pane, but no goal is automatically sent.

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
