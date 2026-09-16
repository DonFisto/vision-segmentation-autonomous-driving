#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/lib/common.sh"

SESSION="carla_spatial"
ensure_ssh_master
recreate_session "$SESSION" spatial

tmux split-window -h -t "$SESSION:spatial.0"
tmux split-window -v -t "$SESSION:spatial.0"
tmux split-window -v -t "$SESSION:spatial.1"
tmux select-layout -t "$SESSION:spatial" tiled

connect_pane "$SESSION:spatial.0"
remote_depth_shell "$SESSION:spatial.0"
wait_topic "$SESSION:spatial.0" "/perception/depth/colormap/compressed" "depth output"
tmux send-keys -t "$SESSION:spatial.0" \
  "ros2 run free_space_node free_space_node --ros-args -p roi_x_min_ratio:=0.30 -p roi_x_max_ratio:=0.70 -p roi_y_min_ratio:=0.25 -p roi_y_max_ratio:=0.60 -p close_depth_thresh:=40.0" C-m

connect_pane "$SESSION:spatial.1"
remote_depth_shell "$SESSION:spatial.1"
wait_topic "$SESSION:spatial.1" "/perception/depth/colormap/compressed" "depth output"
tmux send-keys -t "$SESSION:spatial.1" \
  "sleep 2; ros2 run local_occupancy_node local_occupancy_node --ros-args -p roi_x_min_ratio:=0.10 -p roi_x_max_ratio:=0.90 -p roi_y_min_ratio:=0.25 -p roi_y_max_ratio:=0.95 -p static_y_min_ratio:=0.55 -p forward_m:=18.0 -p width_m:=10.0 -p resolution:=0.25 -p far_power:=1.3 -p pixel_stride:=3 -p static_dilate_cells:=0 -p dynamic_dilate_cells:=1 -p free_dilate_cells:=1 -p use_depth_obstacles:=False" C-m

connect_pane "$SESSION:spatial.2"
remote_depth_shell "$SESSION:spatial.2"
wait_topic "$SESSION:spatial.2" "/carla/hero_odom" "hero odometry"
tmux send-keys -t "$SESSION:spatial.2" \
  "sleep 5; ros2 run local_mapping_node local_mapping_node --ros-args -p map_size_m:=300.0 -p resolution:=0.25 -p local_forward_m:=18.0 -p local_width_m:=10.0 -p yaw_sign:=1.0 -p lateral_sign:=1.0 -p static_hit_inc:=1.5 -p static_occupied_thresh:=6.0 -p free_dec:=2.0 -p free_thresh:=-2.5 -p dynamic_hit_inc:=5.0 -p dynamic_decay:=0.80 -p dynamic_occupied_thresh:=3.0 -p pixel_scale:=2" C-m

connect_pane "$SESSION:spatial.3"
remote_depth_shell "$SESSION:spatial.3"
tmux send-keys -t "$SESSION:spatial.3" \
  "watch -n 2 'ros2 topic list --no-daemon --spin-time 2 | sort | grep -E \"free_space|occupancy|local_map|mapping\"'" C-m

finish_session "$SESSION" spatial
