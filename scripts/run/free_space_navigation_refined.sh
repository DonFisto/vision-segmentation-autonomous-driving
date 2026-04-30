#!/usr/bin/env bash
set -euo pipefail

cd ~/vision-segmentation-autonomous-driving/ros/ros2_ws
source install/setup.bash

ros2 run free_space_navigation_node free_space_navigation_node --ros-args \
  -p cruise_speed:=0.30 \
  -p slow_speed:=0.20 \
  -p reverse_speed:=-0.80 \
  -p steer_value:=0.45 \
  -p center_obstacle_stop:=0.35 \
  -p center_free_cruise:=0.55
