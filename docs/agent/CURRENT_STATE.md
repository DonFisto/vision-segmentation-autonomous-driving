# Current Project State

Last initialized: 2026-07-30

This is an initial state summary derived from the repository's public documentation. It must be replaced or refined by the first full evidence-based Codex review.

## Current milestone

**Perception-to-local-mapping prototype implemented; map-based planning, structured control evaluation, and a clearly demonstrated final closed loop remain the next major phase.**

## Documented subsystem status

| Subsystem | Initial status | Evidence or qualification |
| --- | --- | --- |
| CARLA RGB and hero odometry input | Implemented | Documented in the current stack |
| Semantic segmentation | Implemented | SegFormer-based segmentation and CARLA/Cityscapes workflow documented |
| Object extraction | Implemented | ROS2 node documented |
| Object tracking | Implemented | ROS2 tracking node and examples documented |
| Monocular depth | Implemented | Depth node and milestone documentation present |
| Object-depth fusion | Implemented | Fusion node and milestone documentation present |
| Semantic-depth free-space estimation | Implemented | Free-space milestone documented |
| Reactive navigation prototypes | Implemented, limited | Two prototypes documented; not assumed to be a general planner |
| Local static/dynamic occupancy layers | Implemented | Local occupancy milestone documented |
| Short-term accumulated local mapping | Implemented, limited | Uses simulator-provided odometry; explicitly not full SLAM |
| Route or reference-path planning | Unclear / absent | Not documented as a completed general planning stage |
| Local trajectory planning | Unclear / absent | Requires repository inspection |
| Lateral control | Partial / unclear | CARLA control and reactive navigation exist, but controller design and validation require inspection |
| Longitudinal control | Partial / unclear | Requires repository inspection |
| Actuation interface | Implemented or partial | `carla_control_node` is documented; command semantics and integration require inspection |
| Closed-loop quantitative evaluation | Unclear | No repository-wide evaluation baseline is documented in the overview |
| Ego-motion estimation independent of CARLA | Absent by design | Listed as a future direction |
| Full SLAM | Out of current scope | The existing map is intentionally local and short-term |

## Current strengths

- A modular ROS2 pipeline already spans perception, fusion, navigation, and mapping.
- The repository contains milestone documentation rather than only source code.
- Static and dynamic occupancy are represented separately.
- The project clearly states the limitations of simulator odometry and the local map.
- The current architecture is suitable as a base for planning and control work, subject to interface review.

## Highest-priority unknowns for the first Codex review

1. Exact coordinate frames, transforms, and unit conventions across every node.
2. Timestamp propagation and synchronization between segmentation, depth, tracking, odometry, and mapping.
3. The precise map representation exposed to future planning components.
4. Whether reactive navigation is image-space, map-space, or mixed, and how it relates to the intended planner.
5. The contract between planning, control, and `carla_control_node`.
6. Existing evaluation metrics, replayable scenarios, tests, and logs.
7. Which components are currently understood at Level 1, Level 2, or Level 3 by the developer.

## Immediate project direction

The next phase should aim for the smallest coherent map-based closed loop:

```text
local environment representation
    -> simple path/reference generation
    -> basic lateral and longitudinal control
    -> CARLA actuation
    -> repeatable scenario evaluation
```

The first full review may alter this sequence if repository evidence exposes a more fundamental blocker.
