#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/lib/common.sh"

SESSION="carla_lanes"
ensure_ssh_master
tmux kill-session -t "$SESSION" 2>/dev/null || true

ROAD_MARKING_THRESHOLD="${ROAD_MARKING_THRESHOLD:-0.50}"
LANE_BEV_FORWARD_MAX_M="${LANE_BEV_FORWARD_MAX_M:-40.0}"
LANE_BEV_LATERAL_M="${LANE_BEV_LATERAL_M:-12.0}"
LANE_BEV_RESOLUTION_M="${LANE_BEV_RESOLUTION_M:-0.10}"
LANE_PROCESS_FORWARD_MIN_M="${LANE_PROCESS_FORWARD_MIN_M:-5.0}"
LANE_PROCESS_FORWARD_MAX_M="${LANE_PROCESS_FORWARD_MAX_M:-30.0}"
LANE_PROCESS_LATERAL_M="${LANE_PROCESS_LATERAL_M:-10.0}"

TRACKER_MAX_ODOM_AGE_MS="${TRACKER_MAX_ODOM_AGE_MS:-150.0}"
TRACKER_ODOM_Y_SIGN="${TRACKER_ODOM_Y_SIGN:--1.0}"
TRACKER_ODOM_YAW_SIGN="${TRACKER_ODOM_YAW_SIGN:--1.0}"
TRACKER_SIDE_TOLERANCE_M="${TRACKER_SIDE_TOLERANCE_M:-0.75}"
TRACKER_MAX_SINGLE_BOUNDARY_ABS_LATERAL_M="${TRACKER_MAX_SINGLE_BOUNDARY_ABS_LATERAL_M:-5.50}"
TRACKER_MIN_PAIR_CONFIDENCE_FOR_WIDTH_UPDATE="${TRACKER_MIN_PAIR_CONFIDENCE_FOR_WIDTH_UPDATE:-0.30}"

LANE_MAP_FORWARD_M="${LANE_MAP_FORWARD_M:-50.0}"
LANE_MAP_BACKWARD_M="${LANE_MAP_BACKWARD_M:-10.0}"
LANE_MAP_LATERAL_M="${LANE_MAP_LATERAL_M:-12.0}"
LANE_MAP_RESOLUTION_M="${LANE_MAP_RESOLUTION_M:-0.20}"
LANE_MAP_HISTORY_SECONDS="${LANE_MAP_HISTORY_SECONDS:-8.0}"
LANE_MAP_MIN_SIDE_CONFIDENCE="${LANE_MAP_MIN_SIDE_CONFIDENCE:-0.45}"

ROAD_MARKING_CONFIG='${HOME}/vision-segmentation-autonomous-driving/configs/lane/segformer_b0_lane_binary_80k.py'
ROAD_MARKING_WORK_DIR='${HOME}/vision-segmentation-autonomous-driving/work_dirs/segformer_b0_lane_binary_80k'

tmux new-session -d -s "$SESSION" -n lanes_core -x 220 -y 65
tmux split-window -h -t "$SESSION:lanes_core.0"
tmux split-window -v -t "$SESSION:lanes_core.0"
tmux split-window -v -t "$SESSION:lanes_core.1"
tmux select-layout -t "$SESSION:lanes_core" tiled

tmux new-window -d -t "$SESSION" -n lanes_tracking
tmux split-window -h -t "$SESSION:lanes_tracking.0"
tmux split-window -v -t "$SESSION:lanes_tracking.0"
tmux select-layout -t "$SESSION:lanes_tracking" tiled

tmux new-window -d -t "$SESSION" -n lanes_mapping
tmux split-window -h -t "$SESSION:lanes_mapping.0"
tmux select-layout -t "$SESSION:lanes_mapping" even-horizontal

# Road-marking segmentation.
connect_pane "$SESSION:lanes_core.0"
remote_depth_shell "$SESSION:lanes_core.0"
wait_topic "$SESSION:lanes_core.0" "/carla/rgb/image_raw" "RGB camera"
tmux send-keys -t "$SESSION:lanes_core.0" \
  "CONFIG=$ROAD_MARKING_CONFIG; CHECKPOINT=\$(find $ROAD_MARKING_WORK_DIR -maxdepth 1 -type f -name 'best_mDice*.pth' -printf '%T@ %p\n' | sort -nr | head -n 1 | cut -d' ' -f2-); if [ -z \"\$CHECKPOINT\" ]; then echo 'ERROR: no best_mDice checkpoint found'; exit 1; fi; echo \"Using checkpoint: \$CHECKPOINT\"; ros2 run road_marking_seg_node road_marking_node --ros-args -p \"config_path:=\$CONFIG\" -p \"checkpoint_path:=\$CHECKPOINT\" -p threshold:=$ROAD_MARKING_THRESHOLD -p publish_probability:=false -p publish_overlay:=true" C-m

# Metric BEV.
connect_pane "$SESSION:lanes_core.1"
remote_depth_shell "$SESSION:lanes_core.1"
wait_topic "$SESSION:lanes_core.1" "/perception/road_marking/mask" "road-marking mask"
wait_topic "$SESSION:lanes_core.1" "/carla/rgb/camera_info" "camera info"
tmux send-keys -t "$SESSION:lanes_core.1" \
  "ros2 run lane_geometry_node lane_geometry_node --ros-args -p camera_forward_m:=1.5 -p camera_height_m:=2.4 -p forward_min_m:=0.0 -p forward_max_m:=$LANE_BEV_FORWARD_MAX_M -p left_extent_m:=$LANE_BEV_LATERAL_M -p right_extent_m:=-$LANE_BEV_LATERAL_M -p resolution_m:=$LANE_BEV_RESOLUTION_M -p grid_spacing_m:=5.0" C-m

# Orientation/component filtering.
connect_pane "$SESSION:lanes_core.2"
remote_depth_shell "$SESSION:lanes_core.2"
wait_topic "$SESSION:lanes_core.2" "/perception/lane/bev_mask" "lane BEV mask"
tmux send-keys -t "$SESSION:lanes_core.2" \
  "ros2 run lane_geometry_node lane_component_filter_node --ros-args -p forward_min_m:=0.0 -p forward_max_m:=$LANE_BEV_FORWARD_MAX_M -p left_extent_m:=$LANE_BEV_LATERAL_M -p right_extent_m:=-$LANE_BEV_LATERAL_M -p resolution_m:=$LANE_BEV_RESOLUTION_M -p process_forward_min_m:=$LANE_PROCESS_FORWARD_MIN_M -p process_forward_max_m:=$LANE_PROCESS_FORWARD_MAX_M -p process_left_extent_m:=$LANE_PROCESS_LATERAL_M -p process_right_extent_m:=-$LANE_PROCESS_LATERAL_M" C-m

# Context suppression.
connect_pane "$SESSION:lanes_core.3"
remote_depth_shell "$SESSION:lanes_core.3"
wait_topic "$SESSION:lanes_core.3" "/perception/lane/bev_mask" "lane BEV mask"
tmux send-keys -t "$SESSION:lanes_core.3" \
  "sleep 2; ros2 run lane_geometry_node lane_context_filter_node --ros-args -p forward_min_m:=0.0 -p forward_max_m:=$LANE_BEV_FORWARD_MAX_M -p left_extent_m:=$LANE_BEV_LATERAL_M -p right_extent_m:=-$LANE_BEV_LATERAL_M -p resolution_m:=$LANE_BEV_RESOLUTION_M -p process_forward_min_m:=$LANE_PROCESS_FORWARD_MIN_M -p process_forward_max_m:=$LANE_PROCESS_FORWARD_MAX_M -p process_left_extent_m:=$LANE_PROCESS_LATERAL_M -p process_right_extent_m:=-$LANE_PROCESS_LATERAL_M" C-m

# Curve fitting.
connect_pane "$SESSION:lanes_tracking.0"
remote_depth_shell "$SESSION:lanes_tracking.0"
wait_topic "$SESSION:lanes_tracking.0" "/perception/lane/candidate_mask" "lane candidate mask"
tmux send-keys -t "$SESSION:lanes_tracking.0" \
  "ros2 run lane_geometry_node lane_curve_fit_node --ros-args -p forward_min_m:=0.0 -p forward_max_m:=$LANE_BEV_FORWARD_MAX_M -p left_extent_m:=$LANE_BEV_LATERAL_M -p right_extent_m:=-$LANE_BEV_LATERAL_M -p resolution_m:=$LANE_BEV_RESOLUTION_M -p fit_forward_min_m:=$LANE_PROCESS_FORWARD_MIN_M -p fit_forward_max_m:=$LANE_PROCESS_FORWARD_MAX_M -p fit_left_extent_m:=$LANE_PROCESS_LATERAL_M -p fit_right_extent_m:=-$LANE_PROCESS_LATERAL_M" C-m

# Temporal lane tracking.
connect_pane "$SESSION:lanes_tracking.1"
remote_depth_shell "$SESSION:lanes_tracking.1"
wait_topic "$SESSION:lanes_tracking.1" "/perception/lane/curve_status" "lane curve status"
wait_topic "$SESSION:lanes_tracking.1" "/carla/hero_odom" "hero odometry"
tmux send-keys -t "$SESSION:lanes_tracking.1" \
  "ros2 run lane_geometry_node lane_tracking_node --ros-args -p odom_topic:=/carla/hero_odom -p maximum_odom_age_ms:=$TRACKER_MAX_ODOM_AGE_MS -p odom_y_sign:=$TRACKER_ODOM_Y_SIGN -p odom_yaw_sign:=$TRACKER_ODOM_YAW_SIGN -p single_boundary_side_tolerance_m:=$TRACKER_SIDE_TOLERANCE_M -p maximum_single_boundary_abs_lateral_m:=$TRACKER_MAX_SINGLE_BOUNDARY_ABS_LATERAL_M -p minimum_pair_confidence_for_width_update:=$TRACKER_MIN_PAIR_CONFIDENCE_FOR_WIDTH_UPDATE" C-m

# Tracking diagnostics.
connect_pane "$SESSION:lanes_tracking.2"
remote_depth_shell "$SESSION:lanes_tracking.2"
tmux send-keys -t "$SESSION:lanes_tracking.2" \
  "watch -n 2 'printf \"=== odometry ===\\n\"; timeout 2 ros2 topic hz /carla/hero_odom 2>/dev/null | tail -n 3; printf \"\\n=== lane topics ===\\n\"; ros2 topic list --no-daemon --spin-time 2 | sort | grep -E \"road_marking|/perception/lane/\"'" C-m

# Rolling route-ready LaneMap.
connect_pane "$SESSION:lanes_mapping.0"
remote_depth_shell "$SESSION:lanes_mapping.0"
wait_topic "$SESSION:lanes_mapping.0" "/perception/lane/tracking_status" "lane tracking status"
wait_topic "$SESSION:lanes_mapping.0" "/carla/hero_odom" "hero odometry"
tmux send-keys -t "$SESSION:lanes_mapping.0" \
  "ros2 run lane_reasoning_nodes tracked_lane_mapping_node --ros-args -p odom_topic:=/carla/hero_odom -p forward_horizon_m:=$LANE_MAP_FORWARD_M -p backward_horizon_m:=$LANE_MAP_BACKWARD_M -p lateral_horizon_m:=$LANE_MAP_LATERAL_M -p grid_resolution_m:=$LANE_MAP_RESOLUTION_M -p history_seconds:=$LANE_MAP_HISTORY_SECONDS -p minimum_side_confidence:=$LANE_MAP_MIN_SIDE_CONFIDENCE -p odom_y_sign:=$TRACKER_ODOM_Y_SIGN -p odom_yaw_sign:=$TRACKER_ODOM_YAW_SIGN" C-m

# Lane-map diagnostics.
connect_pane "$SESSION:lanes_mapping.1"
remote_depth_shell "$SESSION:lanes_mapping.1"
tmux send-keys -t "$SESSION:lanes_mapping.1" \
  "watch -n 2 'printf \"=== LaneMap ===\\n\"; ros2 topic info /perception/lane/local_map/vector 2>/dev/null; printf \"\\n=== status ===\\n\"; ros2 topic info /perception/lane/local_map/status 2>/dev/null'" C-m

finish_session "$SESSION" lanes_core
