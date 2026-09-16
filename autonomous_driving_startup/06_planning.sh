#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/lib/common.sh"

SESSION="carla_planning"

ensure_ssh_master
recreate_session "$SESSION" planning 220 60

# ============================================================
# Window 1: planning
#
#   pane 0  global route planner
#   pane 1  RoutePlan -> nav_msgs/Path visualizer
#   pane 2  production carla_world -> hero TF
#   pane 3  planning diagnostics
# ============================================================

tmux split-window -h -t "$SESSION:planning.0"
tmux split-window -v -t "$SESSION:planning.0"
tmux split-window -v -t "$SESSION:planning.1"
tmux select-layout -t "$SESSION:planning" tiled


# Pane 0: global OpenDRIVE route planner.
connect_pane "$SESSION:planning.0"
remote_depth_shell "$SESSION:planning.0"

wait_topic \
  "$SESSION:planning.0" \
  "/carla/hero_odom" \
  "hero odometry"

tmux send-keys -t "$SESSION:planning.0" \
  "ros2 run global_route_planner global_route_planner_node" \
  C-m


# Pane 1: RoutePlan -> directly visualizable nav_msgs/Path.
connect_pane "$SESSION:planning.1"
remote_depth_shell "$SESSION:planning.1"

tmux send-keys -t "$SESSION:planning.1" \
  "ros2 run planning_visualization route_plan_visualizer_node" \
  C-m


# Pane 2: production TF:
# carla_world -> hero
connect_pane "$SESSION:planning.2"
remote_depth_shell "$SESSION:planning.2"

wait_topic \
  "$SESSION:planning.2" \
  "/carla/hero_odom" \
  "hero odometry"

tmux send-keys -t "$SESSION:planning.2" \
  "ros2 run planning_visualization odom_tf_broadcaster_node" \
  C-m


# Pane 3: planning diagnostics.
connect_pane "$SESSION:planning.3"
remote_depth_shell "$SESSION:planning.3"

tmux send-keys -t "$SESSION:planning.3" \
  "watch -n 2 'printf \"=== planning nodes ===\\n\"; ros2 node list | sort | grep -E \"global_route_planner|route_plan_visualizer|odom_tf_broadcaster|global_lane_graph_visualizer|foxglove_world_visualizer\" || true; printf \"\\n=== planning topics ===\\n\"; ros2 topic list --no-daemon --spin-time 2 | sort | grep -E \"^/planning/|^/perception/viz/\" || true'" \
  C-m


# ============================================================
# Window 2: visualization
#
#   pane 0  full OpenDRIVE routing graph
#   pane 1  CARLA -> Foxglove coordinate adapter
#   pane 2  interactive validation / goal shell
# ============================================================

tmux new-window \
  -d \
  -t "$SESSION" \
  -n visualization

tmux split-window \
  -h \
  -t "$SESSION:visualization.0"

tmux split-window \
  -v \
  -t "$SESSION:visualization.1"

tmux select-layout \
  -t "$SESSION:visualization" \
  tiled


# Pane 0: global routing graph MarkerArray.
connect_pane "$SESSION:visualization.0"
remote_depth_shell "$SESSION:visualization.0"

wait_topic \
  "$SESSION:visualization.0" \
  "/carla/hero_odom" \
  "hero odometry"

tmux send-keys -t "$SESSION:visualization.0" \
  "ros2 run planning_visualization global_lane_graph_visualizer_node" \
  C-m


# Pane 1: Foxglove-only visualization coordinate tree.
#
# Publishes:
#   carla_world_viz -> hero_viz
#   /planning/route_path_viz
#   /planning/global_lane_graph/markers_viz
#   /perception/viz/lane/*
connect_pane "$SESSION:visualization.1"
remote_depth_shell "$SESSION:visualization.1"

wait_topic \
  "$SESSION:visualization.1" \
  "/carla/hero_odom" \
  "hero odometry"

tmux send-keys -t "$SESSION:visualization.1" \
  "ros2 run planning_visualization foxglove_world_visualizer_node" \
  C-m


# Pane 2: interactive goal / validation shell.
connect_pane "$SESSION:visualization.2"
remote_depth_shell "$SESSION:visualization.2"

tmux send-keys -t "$SESSION:visualization.2" \
  "echo 'Planning visualization shell ready.'" \
  C-m

tmux send-keys -t "$SESSION:visualization.2" \
  "echo" \
  C-m

tmux send-keys -t "$SESSION:visualization.2" \
  "echo 'Validated test goal:'" \
  C-m

tmux send-keys -t "$SESSION:visualization.2" \
  "echo \"ros2 topic pub --once /planning/goal geometry_msgs/msg/PoseStamped '{header: {frame_id: carla_world}, pose: {position: {x: 109.254, y: 89.835, z: 0.0}, orientation: {w: 1.0}}}'\"" \
  C-m

tmux send-keys -t "$SESSION:visualization.2" \
  "echo" \
  C-m

tmux send-keys -t "$SESSION:visualization.2" \
  "echo 'Foxglove 3D:'" \
  C-m

tmux send-keys -t "$SESSION:visualization.2" \
  "echo '  Fixed frame:   carla_world_viz'" \
  C-m

tmux send-keys -t "$SESSION:visualization.2" \
  "echo '  Display frame: hero_viz'" \
  C-m

tmux send-keys -t "$SESSION:visualization.2" \
  "echo '  Enable: /planning/global_lane_graph/markers_viz'" \
  C-m

tmux send-keys -t "$SESSION:visualization.2" \
  "echo '  Enable: /planning/route_path_viz'" \
  C-m

tmux send-keys -t "$SESSION:visualization.2" \
  "echo '  Enable: /perception/viz/lane/* as desired'" \
  C-m


finish_session "$SESSION" planning
