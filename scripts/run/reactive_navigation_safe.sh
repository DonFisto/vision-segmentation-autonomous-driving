#!/usr/bin/env bash
set -euo pipefail

cd ~/vision-segmentation-autonomous-driving/ros/ros2_ws
source install/setup.bash

ros2 run reactive_navigation_node reactive_navigation_node --ros-args \
  -p cruise_speed:=0.20 \
  -p slow_speed:=0.10 \
  -p reverse_speed:=-0.18 \
  -p roi_x_min_ratio:=0.30 \
  -p roi_x_max_ratio:=0.70 \
  -p roi_y_min_ratio:=0.25 \
  -p roi_y_max_ratio:=0.60 \
  -p roi_close_thresh:=40.0 \
  -p roi_danger_thresh:=60.0 \
  -p fused_close_thresh:=60.0 \
  -p fused_danger_thresh:=80.0
