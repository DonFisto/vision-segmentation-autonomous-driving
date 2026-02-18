# ROS2 + CARLA runbook

This document describes the minimal sequence to bring up the CARLA + ROS2 + Segmentation system.

---

# SYSTEM OVERVIEW

CARLA (server, offscreen)
        ↓
CARLA Bridge Node → publishes /carla/rgb/image_raw
        ↓
Semantic Segmentation Node
        ↓
Publishes:
  • /perception/semantic_mask
  • /perception/semantic_overlay
        ↓
Visualization (Foxglove / RViz) or rosbag recording

Optional:
Control Node → publishes /carla/cmd_vel (geometry_msgs/Twist)

---

# TERMINAL SETUP (SERVER)

You typically need 3–4 terminals.

------------------------------------
Terminal 1 — Start CARLA (offscreen)
------------------------------------

cd ~/CARLA_0.9.16
./CarlaUE4.sh -RenderOffScreen

Wait until the server is fully started.

------------------------------------
Terminal 2 — Source ROS
------------------------------------

source /opt/ros/humble/setup.bash

------------------------------------
Terminal 3 — Source Workspace
------------------------------------

cd ~/vision-segmentation-autonomous-driving/ros/ros2_ws

# If using merged install:
source install/setup.bash

# If using isolated install:
# source install/local_setup.bash

------------------------------------
Terminal 4 — Run Nodes
------------------------------------

# CARLA RGB bridge
ros2 run carla_bridge_node carla_bridge

# Semantic segmentation
ros2 run semantic_seg_node seg_node

# Control node (optional)
ros2 run carla_control_node carla_control

---

# VERIFY TOPICS

ros2 topic list

Expected topics:
  /carla/rgb/image_raw
  /perception/semantic_mask
  /perception/semantic_overlay
  /carla/cmd_vel
  /carla_status

---

# RECORD ROS BAG (OPTIONAL)

Record RGB + segmentation:

ros2 bag record \
  /carla/rgb/image_raw \
  /perception/semantic_overlay \
  /perception/semantic_mask

Stop with Ctrl+C.

---

# PLAY ROS BAG (OPTIONAL)

ros2 bag play <bag_directory>

---

# CONTROL INTERFACE

The control node listens on:

/carla/cmd_vel  (geometry_msgs/Twist)

Mapping:
  linear.x > 0   → forward
  linear.x < 0   → reverse
  linear.x = 0   → brake
  angular.z      → steering

Example manual publish:

ros2 topic pub /carla/cmd_vel geometry_msgs/Twist \
"{linear: {x: 0.5}, angular: {z: 0.0}}"

---

# TROUBLESHOOTING

If segmentation node fails:
  • Ensure model checkpoint path is correct
  • Ensure CUDA is available (if using GPU)
  • Confirm /carla/rgb/image_raw is publishing

If control node fails:
  • Ensure bridge node is running
  • Ensure vehicle was spawned with role_name='hero'

If workspace sourcing fails:
  • Check install layout (merged vs isolated)
  • Rebuild with:
      colcon build --symlink-install
