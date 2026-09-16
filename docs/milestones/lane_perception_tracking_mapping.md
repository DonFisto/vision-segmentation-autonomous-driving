# Lane Perception, Tracking, and Mapping Milestone

Recorded 2026-09-16 against `7b54639`. Route-ready lane mapping was integrated at `df913d5` (2026-08-18); `cbcea54` (2026-09-15) corrected the published world convention. This record covers that completed subsystem and its subsequent frame/startup integration, not RoutePlan ↔ LaneMap association.

Implementation statements below are source-verified. Runtime numbers are observations supplied for the September documentation refresh; they were not rerun during this pass. Earlier [accumulated occupancy mapping](accumulated_local_mapping.md) remains a separate historical capability.

## Purpose and actual startup chain

The lane stack turns visual road-marking evidence into a short-term, quality-aware local lane representation. It does not construct global road connectivity and does not consume OpenDRIVE geometry to fill perception gaps.

[05_lanes.sh](../../autonomous_driving_startup/05_lanes.sh) launches this exact dependency chain, all in the remote `ros2depth` environment:

| Package / executable | Input | Output / responsibility |
| --- | --- | --- |
| `road_marking_seg_node road_marking_node` | `/carla/rgb/image_raw` | `/perception/road_marking/mask`; binary learned road-marking segmentation |
| `lane_geometry_node lane_geometry_node` | Mask + `/carla/rgb/camera_info` | `/perception/lane/bev_mask`; metric ground-plane BEV |
| `lane_geometry_node lane_component_filter_node` | BEV mask | `/perception/lane/longitudinal_mask`, `/perception/lane/transverse_mask`; oriented morphology and component filtering |
| `lane_geometry_node lane_context_filter_node` | Longitudinal + transverse masks | `/perception/lane/candidate_mask`, `/perception/lane/exclusion_mask`; transverse/dense-junction suppression |
| `lane_geometry_node lane_curve_fit_node` | Candidate mask | `/perception/lane/left_boundary`, `/perception/lane/right_boundary`, `/perception/lane/centerline` (`nav_msgs/msg/Path`) + `/perception/lane/curve_status` (JSON String) |
| `lane_geometry_node lane_tracking_node` | Boundary paths, curve status, `/carla/hero_odom` | Tracked boundary/centerline paths + `/perception/lane/tracking_status` |
| `lane_reasoning_nodes tracked_lane_mapping_node` | Three tracked paths, tracking status, odometry | `/perception/lane/local_map/vector` (`autonomy_interfaces/msg/LaneMap`), rolling raster/debug/status outputs |

The launcher has `lanes_core`, `lanes_tracking`, and `lanes_mapping` tmux windows. Its topic waits check discovery, not successful message processing. In particular, context filtering consumes the component filter's two masks even though the launcher first waits for the upstream BEV topic.

The older `lane_detection_node`, `lane_projection_node`, `lane_mapping_node`, and `lane_guidance_node` are not this startup chain. Their presence does not establish a route-association or behavior-planning layer.

## Geometry and measurement quality

[lane_geometry_node.py](../../ros/ros2_ws/src/lane_geometry_node/lane_geometry_node/lane_geometry_node.py) assumes locally planar ground and a zero-pitch/roll/yaw camera, mounted 1.5 m forward and 2.4 m high. The configured BEV spans 0–40 m forward, ±12 m lateral at 0.10 m resolution; the launcher's filtering/fitting region is 5–30 m forward and ±10 m lateral. These are configured extents, not observed valid coverage.

Lane points use x-forward, y-left, z-up and default frame label `hero`. [lane_curve_fit_node.py](../../ros/ros2_ws/src/lane_geometry_node/lane_geometry_node/lane_curve_fit_node.py) represents lateral displacement as `y(x)=a*x²+b*x+c`, fits RANSAC candidates then refines them, checks coefficient plausibility, and selects an ego-lane pair. Quality includes support count, forward span, residual RMSE, coverage, individual confidence, and pair width/parallelism/centering confidence. A geometrically plausible candidate can still belong to an irrelevant adjacent lane.

Quality is carried in status JSON alongside the Paths; `nav_msgs/Path` alone has no quality or measured/inferred field. Consumers must not infer certainty from a nonempty Path.

## Temporal tracking and odometry

[LaneTrackingNode](../../ros/ros2_ws/src/lane_geometry_node/lane_geometry_node/lane_tracking_node.py) confirms tracks across frames, gates measurement changes by lateral position, heading, and curvature, and smooths coefficients with measurement confidence. `_measurement_from_path` rejects nonfinite/short geometry, distant candidates, wrong-side candidates, and low-confidence measurements. Accepted gates distinguish `accepted_curve_status` from `accepted_path_fallback`.

The output diagnostics distinguish direct updates (`accepted`, `confirmed`, `replaced`), `held`, `rejected`, and `missing`; confirmation/pending state also exists. `inferred` is a separate side flag. Propagation is reported in the `motion` object, not a replacement for the measurement state: a propagated track can then receive a direct update or be held without one.

`_prepare_motion_for_frame` selects nearby buffered odometry (default maximum age 150 ms). `_propagate_polynomial` samples the previous curve, transforms points from the previous ego pose into the new ego pose, clips support, and refits a quadratic. It uses internal `odom_y_sign=-1.0` and `odom_yaw_sign=-1.0` to match forward-left lane geometry. Missing odometry resets the pose anchor; large translation/yaw jumps reset tracks. This is not an independently estimated ego-motion system.

Left/right input stamps must be within the default 50 ms tolerance. Curve status is matched from recent history by compatible coefficients; it is not an exact timestamp join. Tracking status includes the output `stamp_ns`, frame, raw pair confidence, measurement gates, motion diagnostics, and side state/quality. This allows the mapper to join the three output paths and their status by stamp.

## Width, missing boundaries, and intersections

Width has its own estimate and confidence. `_validate_and_update_lane_pair` checks plausible tracked-pair width/variation and requires adequate raw pair confidence to update width. This is a raw-quality gate, not a separate check that both track updates were accepted in the current frame. Limited one-sided inference requires sufficient width confidence, reduces output confidence, and is capped in duration. The mapper applies its own stricter direct-observation gate before adding evidence.

Missing markings at intersections can be correct perception. The tracker can hold/propagate prior geometry briefly or infer one side under its explicit gates; it does not have permission to invent a lane because a global route expects one. If support disappears, boundary/centerline paths and eventually the vector map may be empty. Conversely, the published segment's `is_intersection=false` is a current constant, not a validated intersection detector.

## Rolling vector map and schema 2

[TrackedLaneMappingNode](../../ros/ros2_ws/src/lane_reasoning_nodes/lane_reasoning_nodes/tracked_lane_mapping_node.py) waits for left, right, center, and status at the same stamp, associates buffered/interpolated odometry, and stores accepted direct observations. `_direct_side` requires an `accepted`, `confirmed`, or `replaced` state, a direct measurement, no inference flag, sufficient output confidence, and usable points. Held/inferred paths are not reinserted as fresh observations; memory cannot reinforce itself this way.

Defaults include an 8 s / 160-observation history, 5 s evidence decay time constant, horizons of 50 m forward / 10 m backward / 12 m lateral, 0.20 m raster resolution, and 0.50 m vector bin spacing. `_accumulate_side_bins` counts one contribution per observation per bin, averages geometry using confidence/freshness, and discounts confidence by freshness and geometric inconsistency. Repeated support strengthens geometry without pushing confidence artificially toward one.

`_build_vector_map` selects the longest contiguous run with supported left/right boundaries and plausible width (default 2.5–4.8 m); a run shorter than 6 m does not produce a segment. It smooths boundaries, derives the centerline, and computes curvature with local arc-length quadratic fits, interior median filtering, and endpoint guards. Sample confidence is the geometric mean of side confidence; segment confidence uses the default 20th percentile of sample confidence.

The [shared message definitions](../../ros/ros2_ws/src/autonomy_interfaces/msg) support richer future topology, but the current producer publishes:

| Field / structure | Current meaning |
| --- | --- |
| `LaneMap.header`, `schema_version` | Latest odometry stamp, `carla_world`, schema **2** (also observed at runtime) |
| `map_revision`, `globally_consistent` | Local counter advanced by integration/pruning; `false`, not a global-map fingerprint |
| `segments` | Zero or one rolling local segment, currently ID 1 |
| `LaneSample` | Arc length `s` from start of this local segment, world pose, curvature, left/right half-widths, confidence |
| `LaneBoundary` | IDs 2/3, sampled world points and per-point/mean confidence; current source `SOURCE_MEMORY`, `complete=true` for selected run |
| Topology / neighbors / transitions | Empty predecessor/successor arrays, neighbor IDs 0, `TOPOLOGY_UNKNOWN`, `TURN_UNKNOWN`, lane-change flags false |
| Speed limit | `speed_limit_known=false`, value 0; zero must not be interpreted as a known limit |
| `drivable`, `is_intersection` | Current constants true/false on an emitted segment; not dynamic collision or intersection checks |

The map is planar (published z=0). `complete=true` refers to the retained boundary run, not complete visibility of an entire road. LaneMap is LOCAL perception memory, not the global connectivity map. Its local IDs and revision cannot be matched by equality to RoutePlan's global identifiers/revision.

The configured publication timer is 10 Hz, not a measured throughput claim. Publication requires fresh current odometry (default maximum age 300 ms); otherwise only status is published. With usable odometry but insufficient perception, an empty `segments` array is valid behavior.

## Production frame correction and observed validation

`cbcea54` adds `_internal_world_to_carla_world` and `_internal_world_yaw_to_carla_world`. Before this fix, internal mirrored world geometry was labeled `carla_world`. Current boundary/centerline positions and headings are converted back to direct CARLA world before LaneMap publication. RoutePlan already uses direct CARLA world. Downstream RoutePlan ↔ LaneMap association must NOT apply another y flip.

One supplied runtime frame probe observed:

| Diagnostic (metres) | Min | Mean | Median | Max |
| --- | ---: | ---: | ---: | ---: |
| `lane_to_local_route_direct_m` | 0.191 | 2.949 | 3.519 | 3.603 |
| `lane_to_local_route_flip_y_m` | 49.720 | 51.123 | 50.699 | 53.340 |

`hero_to_lane_direct_m=4.995`, `hero_to_lane_flip_y_m=50.061`; verdict: `DIRECT_CONVENTIONS_MATCH`. These are one observed diagnostic, not universal expected errors, acceptance thresholds, or proof that geometric association is implemented.

Source qualification: `LaneSample.curvature` is still calculated in local forward-left coordinates and assigned unchanged while poses/headings are reflected. The positional probe does not validate its downstream signed-curvature convention. No implementation correction is claimed here. Similarly, planar LaneMap geometry is not evidence of full 3D map alignment.

Local Foxglove paths are separately reattached to `hero_viz` without mirroring their already forward-left coordinates. Supplied evidence observed `/perception/viz/lane/tracked_centerline` in `hero_viz`. See [the frame registry](../agent/ARCHITECTURE.md) for the separate production and display trees.

## Limitations and handoff

The representation depends on visual marking quality, flat-road projection, limited quadratic support, timestamp/odometry availability, and short-term memory. It neither builds junction connectivity nor calibrates confidence probabilistically. No quantitative all-scenario lane accuracy benchmark was supplied.

Global routing is now [implemented independently](global_route_planning.md). The next layer will select an ego-local RoutePlan window and assess perceived LaneMap geometry using progress, lateral/heading discrepancy, coverage, confidence gates, and explicit no-association behavior. Those concepts are planned; no output interface is finalized. Empty/partial perception near intersections must remain observable rather than be filled with OpenDRIVE geometry.

## Relevant history

- `500d989`, `e6b828e`: binary road-marking dataset/config and ROS segmentation node.
- `7e4ef14`: BEV geometry, filtering, curve fitting, temporal tracking.
- `4f0a8e3`, `6b12fd4`, `61ca67a`: hero odometry publication/timing and odometry-aware tracking.
- `326eaff`: tracked vector mapper, quality/curvature handling, and LaneSegment/interface build groundwork; its short commit title understates the added source.
- `df913d5`: remaining shared messages, tracked-mapper registration, and stamped tracking-status integration.
- `cbcea54`: direct-CARLA publication correction.
- `ce97421`, `7b54639`: display-only frame adapter and reproducible startup integration.
