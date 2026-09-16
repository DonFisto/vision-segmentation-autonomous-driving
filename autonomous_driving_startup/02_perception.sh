#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/lib/common.sh"

SESSION="carla_perception"
ensure_ssh_master
recreate_session "$SESSION" perception

tmux split-window -h -t "$SESSION:perception.0"
tmux split-window -v -t "$SESSION:perception.0"
tmux split-window -v -t "$SESSION:perception.1"
tmux select-layout -t "$SESSION:perception" tiled

# Legacy/object perception stack. Lane perception is launched by 05_lanes.sh.

connect_pane "$SESSION:perception.0"
remote_seg_shell "$SESSION:perception.0"
wait_topic "$SESSION:perception.0" "/carla/rgb/image_raw" "RGB camera"
tmux send-keys -t "$SESSION:perception.0" \
  "ros2 run semantic_seg_node seg_node" C-m

connect_pane "$SESSION:perception.1"
remote_seg_shell "$SESSION:perception.1"
wait_topic "$SESSION:perception.1" "/carla/rgb/image_raw" "RGB camera"
tmux send-keys -t "$SESSION:perception.1" \
  "sleep 2; ros2 run object_detection_node detector" C-m

connect_pane "$SESSION:perception.2"
remote_seg_shell "$SESSION:perception.2"
wait_topic "$SESSION:perception.2" "/carla/rgb/image_raw" "RGB camera"
tmux send-keys -t "$SESSION:perception.2" \
  "sleep 5; ros2 run tracking_node tracking_node" C-m

connect_pane "$SESSION:perception.3"
remote_seg_shell "$SESSION:perception.3"
wait_topic "$SESSION:perception.3" "/carla/rgb/image_raw" "RGB camera"
tmux send-keys -t "$SESSION:perception.3" \
  "sleep 7; ros2 run detections_overlay_node overlay" C-m

finish_session "$SESSION" perception
