# Public Setup Guide

This guide covers the public reference environment and the current lane/global-planning development setup, synchronized against source on 2026-09-16. Earlier object/depth/spatial capabilities remain available. The remote setup below is the configuration encoded by the checked-in launchers, not a claim that every dependency was freshly installed or verified in this documentation pass.

## Environment Assumptions

- Linux on x86-64.
- Conda or Mamba.
- A CUDA-capable GPU with a driver compatible with the pinned PyTorch CUDA build. The current segmentation node initializes its model on CUDA.
- CARLA 0.9.16 installed separately from this repository.
- A binary road-marking checkpoint for the current lane core; a Cityscapes-19-compatible checkpoint for the earlier full semantic stack. Model checkpoints are not stored in Git.
- Internet access during initial environment creation and the first Depth Anything model load.

The tracked [environment.yml](../environment.yml) is the pinned `ros2seg` reference/export. It includes Python 3.11.14, ROS2 Humble through RoboStack, PyTorch 2.1.2+cu121, MMSegmentation 1.2.2, MMCV 2.1.0, CARLA Python API 0.9.16, OpenCV, and ROS messages. It also records a developer-specific `prefix`; environment creation may need a machine-appropriate prefix and access to the matching CUDA wheels. It is not a separate reproducibility lock for the live `ros2depth` environment.

## Current Remote Development Layout

[lib/common.sh](../autonomous_driving_startup/lib/common.sh) defines the current launcher defaults:

| Item | Default / responsibility |
| --- | --- |
| SSH target | `danielmartinez@limoneros.inf.um.es`, port `32122`; connection fields have local environment overrides |
| Remote repository | `~/vision-segmentation-autonomous-driving` |
| Remote ROS workspace | `~/vision-segmentation-autonomous-driving/ros/ros2_ws` |
| CARLA installation | `~/CARLA_0.9.16` |
| Foxglove bridge workspace | `~/fox_ws`, sourced before the main workspace by `07_foxglove.sh` |
| `ros2seg` | `02_perception.sh`: semantic segmentation, object extraction, object tracking, overlay |
| `ros2depth` | Bridge/traffic, depth/fusion, spatial nodes, the entire lane stack (including road-marking segmentation), global planning/visualization, and Foxglove |

Activation sources `~/miniconda3/etc/profile.d/conda.sh` and uses `mamba activate`. tmux and SSH run on the local workstation; CARLA/ROS processes run remotely. These launchers do not source `/opt/ros`. The path literals above are remote layout assumptions, not all configurable environment overrides. See the [startup README](../autonomous_driving_startup/README.md).

## Create the Environment

From the repository root:

```bash
conda env create --name ros2seg -f environment.yml
conda activate ros2seg
```

Mamba can be used in place of Conda:

```bash
mamba env create --name ros2seg -f environment.yml
mamba activate ros2seg
```

The earlier [depth milestone](milestones/depth_fusion_stack.md) records creating `ros2depth` as a clone to isolate the Hugging Face dependency set. This remains a reference starting point, not a newly verified export of the current remote environment:

```bash
conda create --name ros2depth --clone ros2seg
mamba activate ros2depth
```

The depth node requires the Hugging Face inference packages used by Depth Anything V2 (not present in the checked-in `environment.yml` pip list):

```bash
python -m pip install \
  "transformers==4.46.3" \
  "tokenizers==0.20.3" \
  "huggingface-hub==0.26.5"
```

`requirements.lock.txt` records a working environment snapshot, but it contains platform-specific build references. Prefer `environment.yml` for a fresh public installation.

## Prepare Model Artifacts

The semantic segmentation node needs:

1. A model configuration under `configs/cityscapes/`.
2. A compatible checkpoint stored outside Git, for example under `work_dirs/`.

Before building, ensure the segmentation node's configured model and checkpoint locations refer to files available on your machine. The depth node uses `depth-anything/Depth-Anything-V2-Small-hf` by default and downloads it on first use if it is not already cached.

The semantic node currently uses developer-specific `CONFIG_PATH` and `CHECKPOINT_PATH` constants in [seg_node.py](../ros/ros2_ws/src/semantic_seg_node/semantic_seg_node/seg_node.py); the public checkout does not bundle its weights.

The current lane core instead needs the binary road-marking model. [05_lanes.sh](../autonomous_driving_startup/05_lanes.sh) passes `configs/lane/segformer_b0_lane_binary_80k.py` and chooses the newest-by-modification-time `best_mDice*.pth` file in `work_dirs/segformer_b0_lane_binary_80k/` on the remote machine. It fails if none is found. The node accepts `config_path`/`checkpoint_path` parameters and defaults to `cuda:0`; the launcher uses threshold 0.50, overlay enabled, and probability publication disabled. This selection rule does not identify a universally reproducible checkpoint checksum. Global routing itself needs no learned checkpoint; it reads the live CARLA map.

## Build the ROS2 Workspace

Import the external ROS message dependency on a fresh checkout:

```bash
cd ros/ros2_ws
vcs import src < deps.repos
```

Install ROS package dependencies, then build:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Source `install/setup.bash` in every shell used to run repository nodes. If ROS2 is provided by a system installation instead of the Conda environment, source that ROS2 installation before building and before sourcing the workspace.

Current packages include `road_marking_seg_node`, `lane_geometry_node`, `lane_reasoning_nodes`, `autonomy_interfaces`, `global_route_planner`, and `planning_visualization`, in addition to the earlier perception/spatial packages. `autonomy_interfaces` generates LaneMap/RoutePlan and future trajectory schemas; build/source it before importing those messages. The graph visualizer reuses `global_route_planner`. The display adapters also import `tf2_ros`; do not assume the package manifests alone capture the entire working remote environment.

## Verify the Installation

Check the main Python dependencies in the environment that will run them (`transformers` is required for depth, not the standalone planner):

```bash
python -c "import torch, cv2, mmcv, mmengine, mmseg, transformers, carla, rclpy; print('imports: ok')"
python -c "import torch; print('CUDA available:', torch.cuda.is_available())"
```

Check that ROS2 can discover the principal executables:

```bash
ros2 pkg executables carla_bridge_node
ros2 pkg executables semantic_seg_node
ros2 pkg executables depth_node
ros2 pkg executables local_mapping_node
ros2 pkg executables road_marking_seg_node
ros2 pkg executables lane_geometry_node
ros2 pkg executables lane_reasoning_nodes
ros2 pkg executables global_route_planner
ros2 pkg executables planning_visualization
ros2 interface show autonomy_interfaces/msg/LaneMap
ros2 interface show autonomy_interfaces/msg/RoutePlan
```

Run workspace tests and display failures:

```bash
colcon test
colcon test-result --verbose
```

These checks establish dependency/executable discovery, not successful inference or planner runtime behavior. The package test directories include standard lint scaffolding; global routing also provides executable diagnostic nodes. No test results are asserted by listing these commands. Continue with the [ROS2 and CARLA runbook](ros/runbook.md), supply a goal, and inspect the relevant outputs.

## Dependency Notes

- Keep PyTorch, CUDA, MMCV, MMEngine, and MMSegmentation versions aligned with `environment.yml`; upgrading one independently can break binary compatibility.
- The CARLA simulator and Python API should use matching releases.
- `vision_msgs` is declared in `deps.repos` and is imported into the workspace before the build.
- Raw images and occupancy grids can use significant memory and bandwidth. Compressed debug topics are provided for visualization.
