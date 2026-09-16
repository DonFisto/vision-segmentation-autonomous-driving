#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/lib/common.sh"

SESSION="carla_depth_fusion"
ensure_ssh_master
recreate_session "$SESSION" depth_fusion

tmux split-window -h -t "$SESSION:depth_fusion.0"
tmux split-window -v -t "$SESSION:depth_fusion.0"
tmux select-layout -t "$SESSION:depth_fusion" tiled

connect_pane "$SESSION:depth_fusion.0"
remote_depth_shell "$SESSION:depth_fusion.0"
wait_topic "$SESSION:depth_fusion.0" "/carla/rgb/image_raw" "RGB camera"
tmux send-keys -t "$SESSION:depth_fusion.0" \
  "ros2 run depth_node depth_node --ros-args -p jpeg_quality:=50" C-m

connect_pane "$SESSION:depth_fusion.1"
remote_depth_shell "$SESSION:depth_fusion.1"
wait_topic "$SESSION:depth_fusion.1" "/perception/depth/colormap/compressed" "depth output"
tmux send-keys -t "$SESSION:depth_fusion.1" \
  "ros2 run fusion_node fusion_node" C-m

connect_pane "$SESSION:depth_fusion.2"
remote_depth_shell "$SESSION:depth_fusion.2"
tmux send-keys -t "$SESSION:depth_fusion.2" \
  "watch -n 2 'printf \"=== depth/fusion topics ===\\n\"; ros2 topic list --no-daemon --spin-time 2 | sort | grep -E \"depth|fused\"'" C-m

finish_session "$SESSION" depth_fusion
