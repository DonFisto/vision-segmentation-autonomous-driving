#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/lib/common.sh"

SESSION="carla_foxglove"
ensure_ssh_master
recreate_session "$SESSION" foxglove 180 48

FOXGLOVE_CONFIG="$ROS_WS/config/foxglove_bridge.yaml"

connect_pane "$SESSION:foxglove.0"
tmux send-keys -t "$SESSION:foxglove.0" "$ACTIVATE_DEPTH" C-m
tmux send-keys -t "$SESSION:foxglove.0" "cd $FOX_WS" C-m
tmux send-keys -t "$SESSION:foxglove.0" "source install/setup.bash" C-m
tmux send-keys -t "$SESSION:foxglove.0" "source $ROS_WS/install/setup.bash" C-m
tmux send-keys -t "$SESSION:foxglove.0" \
  "ros2 run foxglove_bridge foxglove_bridge --ros-args --params-file $FOXGLOVE_CONFIG" C-m

finish_session "$SESSION" foxglove
