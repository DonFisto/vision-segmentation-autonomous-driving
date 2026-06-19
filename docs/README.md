# Documentation

This index covers setup, operation, project history, and the technical milestones behind the current perception and local-mapping stack.

## Start Here

- [Project overview](../README.md) - portfolio summary, current architecture, and repository layout.
- [Setup notes](setup.md) - environment dependencies and ROS2 workspace preparation.
- [ROS2 and CARLA runbook](ros/runbook.md) - node startup, topic checks, recording, and troubleshooting.
- [Project timeline](timeline.md) - chronological development history.

## Technical Milestones

The milestone documents are implementation records. Read them in this order to follow the system from depth-aware perception to accumulated mapping:

1. [Depth and fusion](milestones/depth_fusion_stack.md) - monocular relative depth and tracked-object fusion.
2. [Reactive navigation](milestones/reactive_navigation.md) - the first depth-driven control prototype.
3. [Semantic-depth free-space estimation](milestones/free_space_estimation.md) - image-space free and occupied regions.
4. [Free-space navigation](milestones/free_space_navigation.md) - navigation driven by the refined free-space representation.
5. [Local occupancy mapping](milestones/local_occupancy_mapping.md) - projection into vehicle-relative occupancy layers.
6. [Accumulated local mapping](milestones/accumulated_local_mapping.md) - short-term world-coordinate mapping using hero odometry.

## Documentation Scope

| Document | Purpose |
| --- | --- |
| `README.md` | Concise project entry point and current capabilities |
| `setup.md` | Development environment and dependency setup |
| `ros/runbook.md` | Operational ROS2 workflow |
| `timeline.md` | Historical progression of the project |
| `milestones/` | Detailed design decisions, validation notes, limitations, and follow-up work |

The milestone pages capture the state of the project when each stage was completed. Later documents may supersede limitations or next steps described in earlier milestones.
