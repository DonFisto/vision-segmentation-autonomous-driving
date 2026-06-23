# Parameter Tuning Log — ROS2/CARLA Autonomous Driving Stack

This document summarizes the main parameter sets tested during the recent CARLA/ROS2 development iterations. It focuses on runtime behavior observed in Foxglove and terminal diagnostics rather than benchmark metrics.

The goal is not to treat these values as final hyperparameters, but to keep a trace of what was tried, what improved, what failed, and what should be used as the current baseline.

---

## 1. Navigation modules

### 1.1 `reactive_navigation_node` — primitive obstacle-reactive navigation

| Iteration | Main parameters | Observed result | Comment |
|---|---|---|---|
| Primitive baseline | `cruise_speed=0.20`, `slow_speed=0.10`, `reverse_speed=-0.18`, `roi_x_min_ratio=0.30`, `roi_x_max_ratio=0.70`, `roi_y_min_ratio=0.25`, `roi_y_max_ratio=0.60`, `roi_close_thresh=40.0`, `roi_danger_thresh=60.0`, `fused_close_thresh=60.0`, `fused_danger_thresh=80.0` | The robot could react to close obstacles but remained very local and short-sighted. It could get stuck in corners or enclosed sections. | Useful as a simple fallback or teaching baseline, but not strong enough as the main navigation strategy. It reacts to detections/depth but does not reason about free space or map structure. |

### 1.2 `free_space_navigation_node` — refined free-space navigation

| Iteration | Main parameters | Observed result | Comment |
|---|---|---|---|
| Early refined navigation | `cruise_speed=0.30`, `slow_speed=0.20`, `reverse_speed=-0.80`, `steer_value=0.45`, `center_obstacle_stop=0.35`, `center_free_cruise=0.55` | Worked better than primitive reactive navigation but could still be aggressive and unstable around tight spaces. | First usable free-space controller. Good for proving that the free-space mask could directly influence control. |
| More conservative refined navigation | `cruise_speed=0.30`, `slow_speed=0.18`, `reverse_speed=-0.55`, `steer_value=0.55`, `center_obstacle_stop=0.24`, `center_free_recovery=0.12`, `center_free_cruise=0.55`, `reverse_duration_sec=0.8`, `recovery_turn_duration_sec=1.0`, `recovery_cooldown_sec=2.0` | More stable recovery behavior and less abrupt reversing. | Current preferred refined navigation baseline. |
| Script typo issue | Same as refined navigation, but with `-p center_free_cruise` missing `:=0.55` | ROS2 failed to parse the parameter override and the navigation node crashed at launch. | Fixed by replacing it with `-p center_free_cruise:=0.55`. This should remain checked in the private startup script. |

---

## 2. Free-space and occupancy modules

### 2.1 `free_space_node`

| Iteration | Main parameters | Observed result | Comment |
|---|---|---|---|
| Current free-space estimation baseline | `roi_x_min_ratio=0.30`, `roi_x_max_ratio=0.70`, `roi_y_min_ratio=0.25`, `roi_y_max_ratio=0.60`, `close_depth_thresh=40.0` | Produced the free-space and obstacle masks used by navigation. | Reasonable for forward obstacle avoidance. It remains image-space and does not solve road/lane structure. |

### 2.2 `local_occupancy_node`

| Iteration | Main parameters | Observed result | Comment |
|---|---|---|---|
| Initial local occupancy version | Used RGB/semantic/depth projection into local grid with obstacle/free-space layers. | Produced useful local occupancy visualization, but static obstacles were sometimes exaggerated. | Good first projection step, but needed semantic/depth filtering. |
| Depth-obstacle issue | `use_depth_obstacles=True` or depth promotion active | Produced false red/static regions, especially from projected vertical semantic classes and depth artifacts. | Depth promotion was too permissive for this scene setup. |
| Current local occupancy baseline | `roi_x_min_ratio=0.10`, `roi_x_max_ratio=0.90`, `roi_y_min_ratio=0.25`, `roi_y_max_ratio=0.95`, `static_y_min_ratio=0.55`, `forward_m=18.0`, `width_m=10.0`, `resolution=0.25`, `far_power=1.3`, `pixel_stride=3`, `static_dilate_cells=0`, `dynamic_dilate_cells=1`, `free_dilate_cells=1`, `use_depth_obstacles=False` | The local grid became much more representative. Free space and obstacle regions became usable for spatial reasoning. | Current preferred baseline. The key decisions were disabling depth obstacle promotion and using `static_y_min_ratio` to avoid projecting upper-image static classes as ground obstacles. |

---

## 3. Accumulated mapping

### 3.1 `local_mapping_node`

| Iteration | Main parameters | Observed result | Comment |
|---|---|---|---|
| Initial accumulated mapping | Used local occupancy + hero odometry to accumulate static/dynamic/free evidence into a larger grid. | The map accumulated, but sign conventions were initially wrong around turns. | Correct concept, but coordinate signs needed debugging. |
| Sign debugging | Tested `yaw_sign` and `lateral_sign` combinations. | Some combinations inverted left/right or turn direction. | The visual map was used as the main validation tool. |
| Current accumulated mapping baseline | `map_size_m=300.0`, `resolution=0.25`, `local_forward_m=18.0`, `local_width_m=10.0`, `yaw_sign=1.0`, `lateral_sign=1.0`, `static_hit_inc=1.5`, `static_occupied_thresh=6.0`, `free_dec=2.0`, `free_thresh=-2.5`, `dynamic_hit_inc=5.0`, `dynamic_decay=0.80`, `dynamic_occupied_thresh=3.0`, `pixel_scale=2` | Accumulated local map became coherent through turns and corridors. | Current working baseline. It is mapping with CARLA ground-truth odometry, not SLAM. |
| Rolling/local map idea | Not implemented yet. Proposed as a future crop/rolling view around the hero pose. | Not tested yet. | Safer first version would keep the existing accumulated map and publish a rolling cropped view centered on the hero. |

---

## 4. Lane detection and lane reasoning

### 4.1 `lane_detection_node` — classical RGB/semantic lane detection

The lane detector evolved through several stages:

```text
RGB image
+ optional semantic road ROI
+ color thresholding
+ morphology
+ connected component filtering
+ Hough line candidates
+ geometric filtering
→ lane mask
→ lane overlay
→ lane status
```

#### Iteration history

| Iteration | Main parameters / behavior | Observed result | Comment |
|---|---|---|---|
| v1: basic color thresholding | `use_semantic_roi=False`; white/yellow thresholding inside ROI; direct pixel-based left/right fitting | Detected lane markings only when they were bright or sunlit. Also detected many non-lane bright objects and road highlights. Confidence was often overoptimistic. | Useful proof of concept, but too permissive. |
| v1 with semantic ROI | `use_semantic_roi=True`, `road_class_id=0` | Reduced many false positives outside the road. | Important improvement, but did not solve crosswalks or bright road markings. |
| Component-filter attempt | Added component filtering: area, width, height, aspect ratio, fill ratio. Example tested values: `min_component_area=20`, `max_component_area=1800`, `max_component_width=70`, `min_component_height=14`, `min_component_aspect=1.3`, `max_component_fill_ratio=0.50` | Reduced blob-like false positives but could reject valid lane fragments. | Good idea, but overly strict values made the detector insensitive. |
| v2: Hough candidate filtering — strict | Added `HoughLinesP` and rejected nearly horizontal segments. Initial stricter values: `hough_threshold=18`, `hough_min_line_length=25`, `hough_max_line_gap=20`, `min_segment_angle_deg=18.0`, `min_segment_length_px=22.0` | Crosswalk/intersection false positives improved, but detection became too insensitive. Many frames produced `seg=0`, `offset=None`, `conf=0.0`. | Correct direction structurally, but too strict. |
| Softer Hough tuning | `white_l_min=145`, `white_s_max=125`, `yellow_s_min=55`, `roi_y_min_ratio=0.45`, `dilate_iterations=1`, `min_component_area=8`, `max_component_area=5000`, `max_component_width=220`, `min_component_aspect=0.12`, `hough_threshold=8`, `hough_min_line_length=12`, `hough_max_line_gap=35`, `min_segment_angle_deg=15.0`, `min_segment_length_px=10.0`, `max_abs_dxdy=2.4`, `max_segments_per_side=14`, `max_output_heading_deg=45.0` | Detector became too sensitive again. It accepted too many segments, accumulated noisy lane map evidence, and still reacted to bright road features. | Useful for recovering sensitivity, but too permissive for mapping/guidance. |
| User stricter adjustment | `white_l_min=160`, `white_s_max=105`, `min_segment_angle_deg=20.0`, `hough_threshold=12` | Better balance, but still somewhat too sensitive. Many accepted segments remained, especially in curved roads and around road markings. | Good intermediate operating point. |
| Conservative family | `white_l_min=175`, `white_s_max=85`, `yellow_s_min=80`, `roi_y_min_ratio=0.50`, `dilate_iterations=0`, `max_component_area=2600`, `max_component_width=150`, `min_component_aspect=0.18`, `max_component_fill_ratio=0.70`, `hough_threshold=16`, `hough_min_line_length=18`, `hough_max_line_gap=24`, `min_segment_angle_deg=23.0`, `min_segment_length_px=16.0`, `min_y_span_px=35`, `max_abs_dxdy=2.0`, `max_segments_per_side=8`, `max_confident_heading_deg=20.0`, `max_output_heading_deg=38.0` | Much more accurate in many frames. It ignored some bad crosswalk/intersection cases and confidence dropped in uncertain cases. However, it could still miss valid lanes or overreact to some center/curb markings. | Current best qualitative family of parameters. Still needs structural improvements rather than only threshold tuning. |

#### Current qualitative status

| Aspect | Status |
|---|---|
| Lane mask cleanliness | Improved significantly compared with v1. |
| Crosswalk rejection | Improved, but not solved. |
| Curved lanes | Partially works, but heading/side assignment can become unstable. |
| Confidence scoring | Better than v1, but still sometimes too optimistic. |
| Single-side detections | Need confidence cap or stronger validation. |
| Lane map integration | Works, but still accumulates noisy geometry if projection threshold is too low. |

---

### 4.2 `lane_projection_node`

| Iteration | Main parameters | Observed result | Comment |
|---|---|---|---|
| Initial skeleton | `px_to_lateral_m=0.01`, `lateral_sign=1.0`, `min_confidence=0.05` | Published approximate vehicle-frame lane geometry from image-space offset/heading. | Useful for interface development, but too permissive for noisy lane detections. |
| Safer mapping threshold | Suggested `min_confidence=0.65–0.75` | Prevents low-confidence detections from entering lane geometry/mapping. | Recommended until lane detection confidence becomes more reliable. |
| Current recommendation | `min_confidence=0.75` | Only higher-confidence detections should be projected. | This protects the accumulated lane map from false positives. |

### 4.3 `lane_mapping_node`

| Iteration | Main parameters | Observed result | Comment |
|---|---|---|---|
| Initial skeleton | `map_size_m=300.0`, `resolution=0.25`, `lane_hit_inc=3.0`, `lane_decay=0.999`, `occupied_thresh=2.0`, `line_thickness_cells=2`, `pixel_scale=2` | Published `/perception/accumulated_lane_map` and debug image. The map began to show a coherent road/lane curve, but also accumulated noisy detections. | Good interface proof. Needs filtering before integration. |
| Current issue | Same as above | Lane map can accumulate wrong geometry from unstable lane detections. | Should only integrate stable/high-confidence detections. Temporal voting is recommended. |

### 4.4 `lane_guidance_node`

| Iteration | Main parameters | Observed result | Comment |
|---|---|---|---|
| Initial skeleton | `min_lane_confidence=0.20`, `kp_offset=0.003`, `kp_heading=0.015`, `steering_sign=1.0`, `max_abs_steer=0.50`, `lane_follow_speed=0.30`, `fallback_speed=0.18` | Published `/navigation/lane_guidance_status`. It does not control `/carla/cmd_vel`. | Safe skeleton for future lane-aware navigation. |
| Safer recommendation threshold | Suggested `min_lane_confidence=0.70–0.80` | Prevents noisy lane status from being considered usable. | Recommended while lane detector is still experimental. |

---

## 5. Startup script / environment parameters

| Iteration | Main change | Observed result | Comment |
|---|---|---|---|
| Original mixed activation | `ACTIVATE_SEG` used `ros2seg`; `ACTIVATE_DEPTH` used `ros2depth` | Python paths became contaminated: active Python from `ros2depth` could import packages from `ros2seg`. | Caused confusion during CUDA debugging. |
| Clean single-environment activation | Both activation commands switched to `ros2depth`, with ROS/Python path variables unset before sourcing the workspace: `unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH` | Prevented accidental `PYTHONPATH` pollution across conda environments. | Recommended for this stack if `ros2depth` contains all segmentation and depth dependencies. |
| GPU driver issue | `nvidia-smi` failed with `Driver/library version mismatch` | CUDA unavailable until the server-side NVIDIA driver/NVML mismatch was fixed. | Not a ROS/package issue. Avoid rebuilding packages to solve driver-level problems. |

---

## 6. Current recommended baselines

### Navigation

```bash
ros2 run free_space_navigation_node free_space_navigation_node --ros-args \
  -p cruise_speed:=0.30 \
  -p slow_speed:=0.18 \
  -p reverse_speed:=-0.55 \
  -p steer_value:=0.55 \
  -p center_obstacle_stop:=0.24 \
  -p center_free_recovery:=0.12 \
  -p center_free_cruise:=0.55 \
  -p reverse_duration_sec:=0.8 \
  -p recovery_turn_duration_sec:=1.0 \
  -p recovery_cooldown_sec:=2.0
```

### Local occupancy

```bash
ros2 run local_occupancy_node local_occupancy_node --ros-args \
  -p roi_x_min_ratio:=0.10 \
  -p roi_x_max_ratio:=0.90 \
  -p roi_y_min_ratio:=0.25 \
  -p roi_y_max_ratio:=0.95 \
  -p static_y_min_ratio:=0.55 \
  -p forward_m:=18.0 \
  -p width_m:=10.0 \
  -p resolution:=0.25 \
  -p far_power:=1.3 \
  -p pixel_stride:=3 \
  -p static_dilate_cells:=0 \
  -p dynamic_dilate_cells:=1 \
  -p free_dilate_cells:=1 \
  -p use_depth_obstacles:=False
```

### Accumulated local mapping

```bash
ros2 run local_mapping_node local_mapping_node --ros-args \
  -p map_size_m:=300.0 \
  -p resolution:=0.25 \
  -p local_forward_m:=18.0 \
  -p local_width_m:=10.0 \
  -p yaw_sign:=1.0 \
  -p lateral_sign:=1.0 \
  -p static_hit_inc:=1.5 \
  -p static_occupied_thresh:=6.0 \
  -p free_dec:=2.0 \
  -p free_thresh:=-2.5 \
  -p dynamic_hit_inc:=5.0 \
  -p dynamic_decay:=0.80 \
  -p dynamic_occupied_thresh:=3.0 \
  -p pixel_scale:=2
```

### Lane detection

```bash
ros2 run lane_detection_node lane_detection_node --ros-args \
  -p use_semantic_roi:=True \
  -p road_class_id:=0 \
  -p min_road_area_ratio:=0.01 \
  -p road_dilate_iterations:=2 \
  -p roi_x_min_ratio:=0.10 \
  -p roi_x_max_ratio:=0.90 \
  -p roi_y_min_ratio:=0.50 \
  -p roi_y_max_ratio:=0.97 \
  -p white_l_min:=175 \
  -p white_s_max:=85 \
  -p yellow_s_min:=80 \
  -p morph_kernel_size:=3 \
  -p dilate_iterations:=0 \
  -p use_component_filter:=True \
  -p min_component_area:=12 \
  -p max_component_area:=2600 \
  -p max_component_width:=150 \
  -p min_component_height:=5 \
  -p min_component_aspect:=0.18 \
  -p max_component_fill_ratio:=0.70 \
  -p use_hough_fit:=True \
  -p hough_threshold:=16 \
  -p hough_min_line_length:=18 \
  -p hough_max_line_gap:=24 \
  -p min_segment_angle_deg:=23.0 \
  -p max_segment_angle_deg:=88.0 \
  -p min_segment_length_px:=16.0 \
  -p min_y_span_px:=35 \
  -p max_abs_dxdy:=2.0 \
  -p max_segments_per_side:=8 \
  -p max_confident_heading_deg:=20.0 \
  -p max_output_heading_deg:=38.0
```

### Lane projection / mapping / guidance

```bash
ros2 run lane_reasoning_nodes lane_projection_node --ros-args \
  -p px_to_lateral_m:=0.01 \
  -p lateral_sign:=1.0 \
  -p min_confidence:=0.75
```

```bash
ros2 run lane_reasoning_nodes lane_mapping_node --ros-args \
  -p map_size_m:=300.0 \
  -p resolution:=0.25 \
  -p lane_hit_inc:=3.0 \
  -p lane_decay:=0.999 \
  -p occupied_thresh:=2.0 \
  -p line_thickness_cells:=2 \
  -p pixel_scale:=2
```

```bash
ros2 run lane_reasoning_nodes lane_guidance_node --ros-args \
  -p min_lane_confidence:=0.80 \
  -p kp_offset:=0.003 \
  -p kp_heading:=0.015 \
  -p steering_sign:=1.0 \
  -p max_abs_steer:=0.50 \
  -p lane_follow_speed:=0.30 \
  -p fallback_speed:=0.18
```

---

## 7. Recommended next changes

### Lane detection v3

Prioritize structural improvements over more threshold tuning:

1. Cap confidence for single-side detections.
2. Add left/right perspective consistency.
3. Add a plausible lane-width check when both sides are visible.
4. Reject centerline candidates when they are likely lane dividers rather than lane boundaries.
5. Add temporal voting before publishing high confidence.
6. Publish more debug fields:
   - raw segments
   - accepted segments
   - left/right segment count
   - reason for rejection
   - confidence sub-scores

### Lane mapping v2

1. Integrate only if `confidence >= 0.75`.
2. Require temporal consistency over several frames.
3. Add decay or confidence-weighted accumulation.
4. Consider a separate “candidate lane map” and “confirmed lane map”.

### Lane guidance

Keep it as recommendation-only until lane mapping is stable. Do not connect it to `/carla/cmd_vel` yet.

---

## 8. Key lessons

- Semantic ROI is essential but insufficient: it removes non-road false positives but cannot distinguish lane boundaries from other road markings.
- Pure brightness/color thresholding is too permissive.
- Hough filtering improves robustness, but parameter tuning is delicate.
- Crosswalks, road symbols, sunlit patches, and yellow center lines are the main false-positive sources.
- Confidence scoring is as important as detection itself.
- Mapping should be protected from noisy detections by confidence and temporal stability gates.
- The current lane detector is good enough as a perception experiment, but not yet reliable enough for closed-loop lane control.
