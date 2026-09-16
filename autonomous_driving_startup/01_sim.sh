#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "$SCRIPT_DIR/lib/common.sh"

SESSION="carla_sim"
ensure_ssh_master
recreate_session "$SESSION" sim

tmux split-window -h -t "$SESSION:sim.0"
tmux split-window -v -t "$SESSION:sim.0"
tmux split-window -v -t "$SESSION:sim.1"
tmux split-window -v -t "$SESSION:sim.3"
tmux select-layout -t "$SESSION:sim" tiled

NAV_MODE="${NAV_MODE:-none}"

# Pane 0: CARLA server.
connect_pane "$SESSION:sim.0"
tmux send-keys -t "$SESSION:sim.0" "cd $CARLA_DIR" C-m
tmux send-keys -t "$SESSION:sim.0" \
  "./CarlaUE4.sh -RenderOffScreen -nosound -carla-rpc-port=2000" C-m

# Pane 1: CARLA ROS2 bridge.
connect_pane "$SESSION:sim.1"
remote_depth_shell "$SESSION:sim.1"
wait_carla_rpc "$SESSION:sim.1"
tmux send-keys -t "$SESSION:sim.1" \
  "ros2 run carla_bridge_node carla_bridge_node" C-m

# Pane 2: traffic generator.
connect_pane "$SESSION:sim.2"
tmux send-keys -t "$SESSION:sim.2" "$ACTIVATE_DEPTH" C-m
tmux send-keys -t "$SESSION:sim.2" "cd $CARLA_DIR/PythonAPI/examples" C-m
wait_carla_rpc "$SESSION:sim.2"
tmux send-keys -t "$SESSION:sim.2" \
  "python generate_traffic.py --host localhost -p 2000 --tm-port 8000 -n 60 -w 30" C-m

# Pane 3: optional legacy navigation / manual-control shell.
connect_pane "$SESSION:sim.3"
remote_depth_shell "$SESSION:sim.3"
wait_topic "$SESSION:sim.3" "/carla/hero_odom" "hero odometry"

case "$NAV_MODE" in
  refined)
    tmux send-keys -t "$SESSION:sim.3" \
      "ros2 run free_space_navigation_node free_space_navigation_node --ros-args -p cruise_speed:=0.30 -p slow_speed:=0.20 -p reverse_speed:=-0.80 -p steer_value:=0.45 -p center_obstacle_stop:=0.35 -p center_free_cruise:=0.55" C-m
    ;;
  primitive)
    tmux send-keys -t "$SESSION:sim.3" \
      "ros2 run reactive_navigation_node reactive_navigation_node --ros-args -p cruise_speed:=0.20 -p slow_speed:=0.10 -p reverse_speed:=-0.18 -p roi_x_min_ratio:=0.30 -p roi_x_max_ratio:=0.70 -p roi_y_min_ratio:=0.25 -p roi_y_max_ratio:=0.60 -p roi_close_thresh:=40.0 -p roi_danger_thresh:=60.0 -p fused_close_thresh:=60.0 -p fused_danger_thresh:=80.0" C-m
    ;;
  none)
    tmux send-keys -t "$SESSION:sim.3" \
      "echo 'NAV_MODE=none. Manual control shell ready.'" C-m
    tmux send-keys -t "$SESSION:sim.3" \
      "echo 'Manual control: ros2 run carla_control_node carla_control'" C-m
    ;;
  *)
    echo "ERROR: NAV_MODE must be none, refined, or primitive." >&2
    exit 1
    ;;
esac

# Pane 4: simulation diagnostics.
connect_pane "$SESSION:sim.4"
remote_depth_shell "$SESSION:sim.4"
tmux send-keys -t "$SESSION:sim.4" \
  "watch -n 2 'printf \"=== CARLA topics ===\\n\"; ros2 topic list --no-daemon --spin-time 2 | sort | grep -E \"^/carla/|^/clock$|^/carla_status$\"'" C-m

finish_session "$SESSION" sim
