# Project Timeline — Vision Segmentation & Autonomous Driving

## Phase 0 — MMSegmentation Fundamentals

### Session 1 — Oxford Pets Dataset Setup
- Converted Oxford-IIIT Pets to MMSeg format.
- Implemented binary trimap → segmentation mask conversion.
- Built minimal SegFormer-B0 config for binary segmentation.
- First successful training run on pets.

### Session 2 — Training / Evaluation Toolkit
- Created reusable scripts:
  - train_from_cfg.py
  - eval_mmseg.py
  - infer_trained.py
- Learned MMEngine Runner architecture.
- Fixed SegDataPreProcessor size/size_divisor conflict.

### Session 3 — Pretrained vs Custom Training
- Clarified difference between:
  - Training from scratch
  - Fine-tuning
  - Pure evaluation of pretrained models
- Added load_from correctly in config.

---

## Phase 1 — Cityscapes Expansion

### Session 4 — Cityscapes Dataset Integration
- Downloaded leftImg8bit + gtFine.
- Understood labelTrainIds requirement.
- Built Cityscapes SegFormer-B0 config from scratch.

### Session 5 — First Cityscapes Training
- Long training run (80k iterations).
- Achieved solid mIoU.
- Validated with proper evaluation metrics.

Breakthrough: First properly trained large-scale urban segmentation model.

---

## Phase 2 — ROS2 + CARLA Integration

### Session 6 — ROS2 Workspace Creation
- Created ros2_ws.
- Built custom ROS2 packages:
  - semantic_seg_node
  - carla_bridge_node
  - carla_control_node
  - ascii_cam_node

### Session 7 — Real-Time Segmentation Node
- Loaded MMSeg model inside ROS2 node.
- Subscribed to /carla/rgb/image_raw.
- Published:
  - /perception/semantic_mask
  - /perception/semantic_overlay
- Resolved show_result API incompatibility.

Breakthrough: Real-time segmentation running inside ROS.

---

## Phase 3 — CARLA Simulation Control

### Session 8 — CARLA Server Setup
- Ran CARLA in offscreen mode.
- Connected custom Python CARLA client.
- Spawned vehicle + attached RGB camera.

### Session 9 — Manual Control Challenges
- Manual control failed (no GUI).
- Built terminal-based control.
- Migrated to /carla/cmd_vel Twist-based control.

### Session 10 — Vehicle Control Architecture Fix
- Resolved dual-spawn issue.
- Ensured role_name="hero" consistency.
- Control node finds existing vehicle instead of spawning new one.

Breakthrough: Stable real-time controllable simulation pipeline.

---

## Phase 4 — ROS Bags & Visualization

### Session 11 — Bag Recording
- Recorded segmentation topics.
- Implemented max-size bag recording scripts (1–2GB).
- Exported bags for local playback.

### Session 12 — Foxglove Live Streaming
- Set up Foxglove WebSocket bridge.
- Visualized segmentation live on laptop.
- Eliminated need for bag-based workflow.

Breakthrough: Fully live remote perception visualization.

---

## Phase 5 — ASCII Debug Visualization

### Session 13 — Terminal Visualization
- Implemented ASCII camera view node.
- Fixed terminal character aspect ratio.
- Enabled remote debugging without GUI.

---

## Phase 6 — Repository Structuring

### Session 14 — Project Refactor
- Separated perception repo from ROS workspace.
- Added .gitignore hygiene.
- Documented versioning strategy.
- Structured:
  - configs/
  - scripts/
  - ros/
  - docs/

Breakthrough: Clean portfolio-ready architecture.

---

# Current System State

You now have:

- Trained SegFormer on Cityscapes
- Real-time segmentation node
- CARLA offscreen simulation
- Terminal control via Twist
- Foxglove live streaming
- Bag recording fallback
- Structured repository
- Version-controlled pipeline

---

# Next Major Direction

1) Object Detection + Tracking
- YOLO or lightweight detector
- Tracking-by-detection (Kalman + IoU)

2) Monocular Visual Odometry
- ORB-SLAM2 or lightweight VO
- Ego-motion estimation

3) Global Map Integration
- Keyframe-based sparse map
- Combine with segmentation layers
