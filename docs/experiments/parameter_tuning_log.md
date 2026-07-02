# Parameter Tuning Log — ROS2/CARLA Autonomous Driving Stack

This document records the main parameter sets tested while developing the CARLA/ROS2 perception, mapping, navigation, and lane-detection stack. It is based on qualitative runtime observations in Foxglove and terminal diagnostics rather than formal benchmark metrics.

The current update preserves the latest working lane-detection baseline and restores the lane-mapping parameters that gave the best behavior for the current mapping node.

---

## 1. Navigation modules

### 1.1 `reactive_navigation_node` — primitive obstacle-reactive navigation

| Iteration | Main parameters | Observed result | Comment |
|---|---|---|---|
| Primitive baseline | `cruise_speed=0.20`, `slow_speed=0.10`, `reverse_speed=-0.18`, `roi_x_min_ratio=0.30`, `roi_x_max_ratio=0.70`, `roi_y_min_ratio=0.25`, `roi_y_max_ratio=0.60`, `roi_close_thresh=40.0`, `roi_danger_thresh=60.0`, `fused_close_thresh=60.0`, `fused_danger_thresh=80.0` | The vehicle could react to close obstacles, but the behavior was very local and short-sighted. It could get stuck in corners or enclosed sections. | Useful as a simple fallback or teaching baseline, but not strong enough as the main navigation strategy. |

### 1.2 `free_space_navigation_node` — refined free-space navigation

| Iteration | Main parameters | Observed result | Comment |
|---|---|---|---|
| Early refined navigation | `cruise_speed=0.30`, `slow_speed=0.20`, `reverse_speed=-0.80`, `steer_value=0.45`, `center_obstacle_stop=0.35`, `center_free_cruise=0.55` | Worked better than primitive reactive navigation but could still be aggressive and unstable around tight spaces. | First usable free-space controller. |
| More conservative refined navigation | `cruise_speed=0.30`, `slow_speed=0.18`, `reverse_speed=-0.55`, `steer_value=0.55`, `center_obstacle_stop=0.24`, `center_free_recovery=0.12`, `center_free_cruise=0.55`, `reverse_duration_sec=0.8`, `recovery_turn_duration_sec=1.0`, `recovery_cooldown_sec=2.0` | More stable recovery behavior and less abrupt reversing. | Current preferred refined navigation baseline. |
| Script typo issue | Same as refined navigation, but with `-p center_free_cruise` missing `:=0.55` | ROS2 failed to parse the parameter override and the navigation node crashed at launch. | Fixed by replacing it with `-p center_free_cruise:=0.55`. |

#### Current refined navigation baseline

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

---

## 2. Free-space and occupancy modules

### 2.1 `local_occupancy_node`

| Iteration | Main parameters | Observed result | Comment |
|---|---|---|---|
| Initial local occupancy version | RGB/semantic/depth projection into local grid with obstacle/free-space layers. | Produced useful local occupancy visualization, but static obstacles were sometimes exaggerated. | Good first projection step, but needed semantic/depth filtering. |
| Depth-obstacle issue | `use_depth_obstacles=True` or depth promotion active | Produced false red/static regions, especially from projected vertical semantic classes and depth artifacts. | Depth promotion was too permissive for this scene setup. |
| Current local occupancy baseline | `roi_x_min_ratio=0.10`, `roi_x_max_ratio=0.90`, `roi_y_min_ratio=0.25`, `roi_y_max_ratio=0.95`, `static_y_min_ratio=0.55`, `forward_m=18.0`, `width_m=10.0`, `resolution=0.25`, `far_power=1.3`, `pixel_stride=3`, `static_dilate_cells=0`, `dynamic_dilate_cells=1`, `free_dilate_cells=1`, `use_depth_obstacles=False` | The local grid became much more representative. Free space and obstacle regions became usable for spatial reasoning. | Current preferred baseline. |

#### Current local occupancy baseline

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

---

## 3. Accumulated local mapping

### 3.1 `local_mapping_node`

| Iteration | Main parameters | Observed result | Comment |
|---|---|---|---|
| Initial accumulated mapping | Used local occupancy + hero odometry to accumulate static/dynamic/free evidence into a larger grid. | The map accumulated, but sign conventions were initially wrong around turns. | Correct concept, but coordinate signs needed debugging. |
| Sign debugging | Tested `yaw_sign` and `lateral_sign` combinations. | Some combinations inverted left/right or turn direction. | The visual map was used as the main validation tool. |
| Current accumulated mapping baseline | `map_size_m=300.0`, `resolution=0.25`, `local_forward_m=18.0`, `local_width_m=10.0`, `yaw_sign=1.0`, `lateral_sign=1.0`, `static_hit_inc=1.5`, `static_occupied_thresh=6.0`, `free_dec=2.0`, `free_thresh=-2.5`, `dynamic_hit_inc=5.0`, `dynamic_decay=0.80`, `dynamic_occupied_thresh=3.0`, `pixel_scale=2` | Accumulated local map became coherent through turns and corridors. | Current working baseline. It uses CARLA ground-truth odometry, so it is mapping, not SLAM. |

#### Current accumulated local mapping baseline

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

---

## 4. Lane detection and lane reasoning

### 4.1 `lane_detection_node` — classical RGB/semantic lane detection

The lane detector evolved through these stages:

```text
RGB image
+ semantic road ROI
+ strict strong white/yellow color thresholding
+ adaptive shadow-white detection
+ shadow gate for adaptive detection
+ morphology
+ connected component filtering
+ Hough line candidates
+ geometric filtering
+ consistency/confidence gates
→ lane mask
→ lane overlay
→ lane status
```

### 4.2 Lane-detection iteration history

| Iteration | Main parameters / behavior | Observed result | Comment |
|---|---|---|---|
| v1: basic color thresholding | White/yellow thresholding inside ROI; direct pixel-based left/right fitting. | Detected lanes mainly when bright or sunlit. Also detected many non-lane bright objects and road highlights. | Useful proof of concept, but too permissive. |
| v1 + semantic ROI | `use_semantic_roi=True`, `road_class_id=0` | Reduced false positives outside the road. | Important improvement, but insufficient for crosswalks, shadows, and road markings. |
| Component-filter attempt | Area, width, height, aspect ratio, and fill-ratio filters. | Reduced blob-like false positives but rejected valid fragments when too strict. | Good idea but sensitive to tuning. |
| v2 strict Hough filtering | Higher Hough threshold, longer line length, stricter angles. | Improved crosswalk/intersection rejection, but became too insensitive. | Correct structural direction but too strict. |
| Softer Hough tuning | Lower Hough threshold, shorter lines, relaxed component sizes. | Recovered sensitivity but became noisy again. | Useful for recovering detections but too permissive for mapping. |
| Conservative Hough family | `white_l_min=175`, `white_s_max=85`, `yellow_s_min=80`, `hough_threshold=16`, `max_segments_per_side=8` | Cleaner in sunlit conditions, but still poor in shadows. | Good sunlit baseline, insufficient under bridges/tree shadows. |
| LDv3 consistency gates | Perspective consistency, lane-width checks, single-side confidence cap, temporal confidence cap. | More stable confidence behavior. | Structural improvement, but still dependent on mask quality. |
| LDv3.2 adaptive white | CLAHE + adaptive local contrast white detection. | Much better in shadows, but too noisy in bright areas. | Correct direction, but adaptive branch needed illumination gating. |
| LDv3.3 shadow-gated adaptive white | Adaptive branch restricted to darker regions; strong white/yellow path made stricter. | Best result so far. Shadowed lanes became usable and bright-road speckle was much lower. `seg` usually stayed around `6–8`. | Current recommended lane-detection baseline. |

### 4.3 Current recommended LDv3.3 lane-detection baseline

This is the best qualitative parameter set tested so far:

```bash
ros2 run lane_detection_node lane_detection_node --ros-args \
  -p use_semantic_roi:=True \
  -p road_class_id:=0 \
  -p min_road_area_ratio:=0.01 \
  -p road_dilate_iterations:=2 \
  -p roi_x_min_ratio:=0.10 \
  -p roi_x_max_ratio:=0.90 \
  -p roi_y_min_ratio:=0.48 \
  -p roi_y_max_ratio:=0.97 \
  -p white_l_min:=190 \
  -p white_s_max:=70 \
  -p yellow_s_min:=95 \
  -p use_adaptive_white:=True \
  -p use_clahe:=True \
  -p clahe_clip_limit:=1.5 \
  -p clahe_tile_grid_size:=8 \
  -p adaptive_white_block_size:=61 \
  -p adaptive_white_c:=-10 \
  -p adaptive_white_min_l:=50 \
  -p adaptive_white_s_max:=120 \
  -p adaptive_white_use_shadow_gate:=True \
  -p adaptive_white_shadow_l_max:=125 \
  -p morph_kernel_size:=3 \
  -p dilate_iterations:=0 \
  -p use_canny_edges:=False \
  -p use_component_filter:=True \
  -p min_component_area:=10 \
  -p max_component_area:=2600 \
  -p max_component_width:=150 \
  -p min_component_height:=5 \
  -p min_component_aspect:=0.16 \
  -p max_component_fill_ratio:=0.70 \
  -p use_hough_fit:=True \
  -p hough_threshold:=16 \
  -p hough_min_line_length:=18 \
  -p hough_max_line_gap:=24 \
  -p min_segment_angle_deg:=22.0 \
  -p max_segment_angle_deg:=88.0 \
  -p min_segment_length_px:=16.0 \
  -p min_y_span_px:=32 \
  -p max_abs_dxdy:=2.0 \
  -p max_segments_per_side:=4 \
  -p max_confident_heading_deg:=20.0 \
  -p max_output_heading_deg:=38.0 \
  -p enforce_perspective_consistency:=True \
  -p perspective_slope_tolerance:=0.18 \
  -p min_lane_width_px:=200.0 \
  -p max_lane_width_px:=600.0 \
  -p single_side_confidence_cap:=0.50 \
  -p invalid_width_confidence_cap:=0.35 \
  -p min_stable_frames:=3 \
  -p temporal_confidence_cap:=0.60
```

### 4.4 Current lane-detection qualitative status

| Aspect | Status |
|---|---|
| Shadow performance | Much better after adaptive white + shadow gating. |
| Bright-road speckles | Reduced significantly by stricter strong white/yellow thresholds and a lower adaptive shadow gate. |
| General detection quality | Best classical-CV baseline so far. Works reasonably well in both sunlit and shaded areas. |
| Crosswalks and road symbols | Improved but not solved. Some fragments still enter candidate sets. |
| Confidence scoring | More stable than earlier versions, but still sometimes optimistic. |
| Main remaining weakness | Ego-lane pair selection. The detector often finds decent candidates, but it still needs to select the correct left/right pair more intelligently. |

---

## 5. Lane projection, mapping, and guidance

### 5.1 `lane_projection_node`

| Iteration | Main parameters | Observed result | Comment |
|---|---|---|---|
| Initial skeleton | `px_to_lateral_m=0.01`, `lateral_sign=1.0`, `min_confidence=0.05` | Published approximate vehicle-frame lane geometry from image-space offset/heading. | Useful for interface development, but too permissive for noisy detections. |
| Safer mapping threshold | `min_confidence=0.65–0.75` | Reduced low-confidence lane projections. | Useful while LD was still noisy. |
| Strict LDv3 tuning threshold | `min_confidence=0.85–0.90` | Prevented medium-confidence false positives from contaminating lane mapping. | Recommended while mapping remains approximate. |

#### Current lane projection recommendation

```bash
ros2 run lane_reasoning_nodes lane_projection_node --ros-args \
  -p px_to_lateral_m:=0.01 \
  -p lateral_sign:=1.0 \
  -p min_confidence:=0.90
```

### 5.2 `lane_mapping_node`

| Iteration | Main parameters | Observed result | Comment |
|---|---|---|---|
| Initial skeleton | `map_size_m=300.0`, `resolution=0.25`, `lane_hit_inc=3.0`, `lane_decay=0.999`, `occupied_thresh=2.0`, `line_thickness_cells=2`, `pixel_scale=2` | Published `/perception/accumulated_lane_map` and `/perception/accumulated_lane_map_debug/compressed`. | This remains the best parameter set for the current lane mapping implementation. |
| Smaller-map visualization attempt | Reduced map size and increased scale, for example `map_size_m=80.0`, `resolution=0.20`, `pixel_scale=5` | Did not fix interpretability. | The limitation is not only debug scale; the mapping representation should be redesigned. |
| Current baseline | `map_size_m=300.0`, `resolution=0.25`, `lane_hit_inc=3.0`, `lane_decay=0.999`, `occupied_thresh=2.0`, `line_thickness_cells=2`, `pixel_scale=2` | Best available configuration for the current node. | Keep this baseline until lane mapping is reworked. |

#### Current lane mapping baseline

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

#### Lane mapping design note

The current lane mapping parameters above are the best tested values for the existing node, but the mapping approach itself is limited. It does not provide an intuitive or sufficiently informative lane-memory visualization.

The next mapping version should be a rolling ego-centric BEV lane memory that uses richer lane geometry rather than only a compact lane status estimate.

Recommended future debug layers:

```text
blue    accumulated left lane boundary
red     accumulated right lane boundary
green   accumulated centerline
cyan    current-frame projected lane evidence
yellow  hero pose / heading
```

### 5.3 `lane_guidance_node`

| Iteration | Main parameters | Observed result | Comment |
|---|---|---|---|
| Initial skeleton | `min_lane_confidence=0.20`, `kp_offset=0.003`, `kp_heading=0.015`, `steering_sign=1.0`, `max_abs_steer=0.50`, `lane_follow_speed=0.30`, `fallback_speed=0.18` | Published `/navigation/lane_guidance_status`. It does not control `/carla/cmd_vel`. | Safe skeleton for future lane-aware navigation. |
| Strict LDv3 tuning threshold | `min_lane_confidence=0.85` | Makes guidance ignore anything except very confident detections. | Keep lane guidance recommendation-only until mapping and pair selection improve. |

#### Current lane guidance recommendation

```bash
ros2 run lane_reasoning_nodes lane_guidance_node --ros-args \
  -p min_lane_confidence:=0.85 \
  -p kp_offset:=0.003 \
  -p kp_heading:=0.015 \
  -p steering_sign:=1.0 \
  -p max_abs_steer:=0.50 \
  -p lane_follow_speed:=0.30 \
  -p fallback_speed:=0.18
```

---

## 6. Startup script / environment notes

| Iteration | Main change | Observed result | Comment |
|---|---|---|---|
| Original mixed activation | `ACTIVATE_SEG` used `ros2seg`; `ACTIVATE_DEPTH` used `ros2depth` | Python paths became contaminated: active Python from `ros2depth` could import packages from `ros2seg`. | Caused confusion during CUDA debugging. |
| Clean single-environment activation | Both activation commands switched to `ros2depth`, with ROS/Python path variables unset before sourcing the workspace. | Prevented accidental `PYTHONPATH` pollution across conda environments. | Recommended if `ros2depth` contains all segmentation and depth dependencies. |
| GPU driver issue | `nvidia-smi` failed with `Driver/library version mismatch` | CUDA unavailable until the server-side NVIDIA driver/NVML mismatch was fixed. | Not a ROS/package issue. Avoid rebuilding packages to solve driver-level problems. |

Recommended clean environment setup before building/running:

```bash
source ~/miniconda3/etc/profile.d/conda.sh
mamba activate ros2depth
unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH
cd ~/vision-segmentation-autonomous-driving/ros/ros2_ws
source install/setup.bash
```

---

## 7. Recommended next changes

### 7.1 Lane detection LDv3.4 — ego-lane pair selection

The current detector now produces usable candidate segments. The next improvement should select the correct ego-lane pair rather than fitting all accepted candidates together.

Recommended pair score terms:

1. Lane center close to image center at the bottom.
2. Plausible lane width.
3. Lane width narrows toward the horizon.
4. Left and right lines do not cross.
5. Similar perspective direction.
6. Temporal consistency with the previously selected pair.

### 7.2 Lane mapping v2 — rolling ego-centric BEV lane memory

The current lane map parameters are acceptable for the existing node, but the representation should be redesigned.

Recommended redesign:

1. Use selected left/right lane geometry, not only offset and heading.
2. Project selected lane segments into a local BEV frame.
3. Maintain a rolling ego-centric map instead of a large global map.
4. Render separate left/right/center/current-frame layers.
5. Add aggressive decay so old wrong detections do not persist.
6. Keep global lane accumulation as a later feature, not the first mapping target.

### 7.3 Debug improvements

Add separate debug masks:

```text
/perception/lane_mask_strong/compressed
/perception/lane_mask_adaptive/compressed
/perception/lane_mask_final/compressed
```

This would make it possible to tell whether false positives come from the strong white/yellow path, the adaptive shadow path, or the Hough/component selection stage.

---

## 8. Key lessons

- Semantic ROI is essential but insufficient: it removes non-road false positives but cannot distinguish lane boundaries from other road markings.
- Pure brightness/color thresholding is too permissive and fails in shadows.
- Adaptive local-contrast detection improves shadows, but must be gated to darker regions.
- Hough filtering improves robustness, but candidate pair selection is now the limiting factor.
- Crosswalks, road symbols, sunlit patches, yellow center lines, and asphalt cracks remain the main false-positive sources.
- Confidence scoring is as important as detection itself.
- Lane mapping should be protected from noisy detections by confidence and temporal stability gates.
- The current lane detector is now a useful classical-CV baseline, but lane mapping should be redesigned around selected lane geometry and rolling BEV memory.
