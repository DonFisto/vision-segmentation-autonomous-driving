# Documentation

This index covers setup, operation, project history, lane mapping, global routing, and the Codex project-review framework. Current state was synchronized against source and supplied development runtime evidence on 2026-09-16.

## Start Here

- [Project overview](../README.md) - portfolio summary, current architecture, and repository layout.
- [Setup notes](setup.md) - environment dependencies and ROS2 workspace preparation.
- [ROS2 and CARLA runbook](ros/runbook.md) - node startup, topic checks, recording, and troubleshooting.
- [Startup package](../autonomous_driving_startup/README.md) - local tmux/SSH orchestration of the remote core or full stack.
- [Project timeline](timeline.md) - chronological development history.
- [Codex review framework](agent/README.md) - persistent project context, architecture state, roadmap, decision and learning logs, and structured review prompts.

## Technical Milestones

The milestone documents are historical implementation records. Read them in this order to follow the progression from depth-aware perception through lane mapping and global routing:

1. [Depth and fusion](milestones/depth_fusion_stack.md) - monocular relative depth and tracked-object fusion.
2. [Reactive navigation](milestones/reactive_navigation.md) - the first depth-driven control prototype.
3. [Semantic-depth free-space estimation](milestones/free_space_estimation.md) - image-space free and occupied regions.
4. [Free-space navigation](milestones/free_space_navigation.md) - navigation driven by the refined free-space representation.
5. [Local occupancy mapping](milestones/local_occupancy_mapping.md) - projection into vehicle-relative occupancy layers.
6. [Accumulated local mapping](milestones/accumulated_local_mapping.md) - short-term world-coordinate mapping using hero odometry.
7. [Lane perception, tracking, and mapping](milestones/lane_perception_tracking_mapping.md) - road markings, BEV geometry, odometry-aware tracking, and route-ready local LaneMap.
8. [Global route planning](milestones/global_route_planning.md) - OpenDRIVE topology, sampled routing graph, A*, RoutePlan, frames, and runtime evidence.

RoutePlan ↔ LaneMap association is next, not implemented. Use [CURRENT_STATE](agent/CURRENT_STATE.md), [ARCHITECTURE](agent/ARCHITECTURE.md), and [ROADMAP](agent/ROADMAP.md) for the current integration boundary. The [dated refresh report](agent/reports/2026-09-16-routing-documentation-refresh.md) distinguishes source inspection from supplied runtime observations. Earlier milestone next steps and the [parameter tuning log](experiments/parameter_tuning_log.md) remain historical, including superseded startup/environment advice.

## Documentation Scope

| Document | Purpose |
| --- | --- |
| `README.md` | Concise project entry point and current capabilities |
| `setup.md` | Development environment and dependency setup |
| `ros/runbook.md` | Operational ROS2 workflow |
| `timeline.md` | Historical progression of the project |
| `milestones/` | Detailed design decisions, validation notes, limitations, and follow-up work |
| `agent/` | Codex reviewer instructions, project state, architecture, roadmap, decisions, learning record, and dated reports |

The milestone pages capture the state of the project when each stage was completed. Later documents may supersede limitations or next steps described in earlier milestones. Agent state files describe the latest reviewed state and should remain grounded in repository evidence.
