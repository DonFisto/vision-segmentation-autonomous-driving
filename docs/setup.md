# Public Setup Guide

This guide prepares the reference environment and builds the ROS2 workspace for the complete perception and local-mapping stack.

## Environment Assumptions

- Linux on x86-64.
- Conda or Mamba.
- A CUDA-capable GPU with a driver compatible with the pinned PyTorch CUDA build. The current segmentation node initializes its model on CUDA.
- CARLA 0.9.16 installed separately from this repository.
- A Cityscapes-19-compatible segmentation checkpoint. Model checkpoints are not stored in Git.
- Internet access during initial environment creation and the first Depth Anything model load.

The tracked `environment.yml` is the reference environment. It includes Python 3.11, ROS2 Humble through RoboStack, PyTorch, MMSegmentation, MMCV, the CARLA Python API, OpenCV, and ROS message packages.

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

The depth node also requires the Hugging Face inference packages used by Depth Anything V2:

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

## Verify the Installation

Check the main Python dependencies:

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
```

Run workspace tests and display failures:

```bash
colcon test
colcon test-result --verbose
```

The environment is ready when imports succeed, CUDA is available for segmentation, and the ROS2 executables are listed. Continue with the [ROS2 and CARLA runbook](ros/runbook.md).

## Dependency Notes

- Keep PyTorch, CUDA, MMCV, MMEngine, and MMSegmentation versions aligned with `environment.yml`; upgrading one independently can break binary compatibility.
- The CARLA simulator and Python API should use matching releases.
- `vision_msgs` is declared in `deps.repos` and is imported into the workspace before the build.
- Raw images and occupancy grids can use significant memory and bandwidth. Compressed debug topics are provided for visualization.
