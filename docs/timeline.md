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

## Phase 7 — Depth-Aware Navigation and Local Mapping

### Session 15 — Accumulated Local Mapping
- Extended the perception stack through depth fusion, free-space estimation, and reactive navigation.
- Added vehicle-relative local occupancy grids with static and dynamic obstacle layers.
- Used CARLA hero odometry to accumulate recent occupancy observations in world coordinates.
- Published combined, static, and dynamic accumulated maps with debug and status outputs.

Breakthrough: The system progressed from frame-by-frame perception to a short-term map-like representation of the surrounding environment.

---

# Historical State at the End of Phase 7

You now have:

- Trained SegFormer on Cityscapes
- Real-time segmentation, object extraction and tracking
- Monocular depth estimation and object-depth fusion
- Semantic-depth free-space estimation
- Reactive and free-space navigation prototypes
- Local occupancy grids with static and dynamic layers
- Accumulated local mapping using CARLA odometry
- ROS2 visualization and recording outputs
- Structured, version-controlled pipeline

---

# Directions Proposed at the End of Phase 7

1) Improve Local Mapping
- Rolling map support
- Timestamp-aware odometry integration
- Better image-to-ground projection

2) Add Visual Odometry
- Replace simulator-provided ego motion with estimated motion
- Evaluate drift and temporal alignment

3) Connect Mapping to Navigation
- Plan with accumulated occupancy instead of single-frame cues
- Improve static and dynamic obstacle handling

---

## Phase 8 — Lane-Specific Perception and Geometry (2026-08-01–04)

- `500d989` added binary road-marking conversion and a lane SegFormer configuration; `e6b828e` added `road_marking_seg_node`.
- `7e4ef14` introduced metric BEV lane geometry, oriented component/context filtering, quadratic curve fitting, and temporal lane tracking.
- The lane-specific chain became distinct from older classical lane detection and from object/depth/spatial mapping.

## Phase 9 — Odometry-Aware Tracking and Route-Ready LaneMap (2026-08-04–18)

- `4f0a8e3` and `6b12fd4` added hero odometry support and independent tracking-rate publication; `61ca67a` propagated tracked lanes with ego motion.
- `326eaff` added the tracked rolling vector mapper and quality/curvature handling plus interface groundwork; `df913d5` completed shared message and stamped tracker/mapper integration.
- LaneMap carries local sampled geometry and explicit confidence/unknown semantics; empty segments remain valid when perception lacks support.
- Record: [lane perception, tracking, and mapping](milestones/lane_perception_tracking_mapping.md).

## Phase 10 — Global OpenDRIVE Routing (2026-08-20–23)

- `ff23221` / `0234a92`: CARLA/OpenDRIVE coarse directed topology and sampled edge geometry.
- `e261432` / `bab6cc1`: Dijkstra shortest-path baseline and connectivity diagnostics.
- `a651b63`: sampled lane-level routing graph with legal lane-follow and left/right lane-change edges.
- `de9dfa5` / `4541c0c`: ego/goal world-position association and live routing diagnostics.
- `89be996` / `b161709`: RoutePlan publication and deterministic global topology segment IDs.
- `0063ae7` / `a32983f` / `89d8561`: explicit goal input, A* production search, and INVALID RoutePlan publication when no directed route exists.
- Global routing became an implemented subsystem, separate from perceived LaneMap and future trajectory planning. Record: [global route planning](milestones/global_route_planning.md).

## Phase 11 — Frame Contract, Visualization, and Startup (2026-09-15–16)

- `cbcea54`: corrected LaneMap publication from internal mirrored world geometry to direct `carla_world` positions/headings, matching RoutePlan and hero pose.
- `1d6186b`: RoutePlan-to-Path and odometry-to-TF adapters; `1cf9d60`: global routing graph markers.
- `ce97421`: visualization-only `carla_world_viz → hero_viz`, reflected global aliases and unmirrored local forward-left lane aliases for Foxglove.
- `7b54639`: reusable local tmux/SSH startup package with core/full/stop/status and five planning/visualization processes.
- Supplied development runtime evidence records direct-frame alignment, Town10HD graph/routes, route-to-graph agreement, matching source visualization timestamps, and one clean stop/full restart. Exact values and limitations are in the new milestone records; they were not rerun in this documentation pass.

## Phase 12 — RoutePlan ↔ LaneMap Association (NEXT, as of 2026-09-16)

The development context records creation of `feature/route-lane-association` from the completed global-routing/startup checkpoint. Git inspection confirms it points at `7b54639`, also the `feature/global-route-planning` tip, without any association implementation commits. Branch-creation context is supplied; it is not a distinct implementation commit.

Planned next work combines global route intent and nominal topology with local perceived lane geometry/confidence, producing a route-relative reference/corridor. Association semantics and interfaces are not finalized or implemented. Behavior/maneuver planning, local trajectory planning, and tracking/control remain future layers. The [current state](agent/CURRENT_STATE.md) and [roadmap](agent/ROADMAP.md) supersede the historical directions above without erasing them.
