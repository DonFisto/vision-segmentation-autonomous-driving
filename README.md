# 🚗 Vision-Based Perception for Autonomous Driving  
Semantic Segmentation + ROS2 + CARLA Integration

This project implements a modular perception pipeline for autonomous driving simulation.  
It combines deep learning-based semantic segmentation (MMSegmentation) with real-time ROS2 integration and CARLA simulator streaming.

> Objective: Bridge ML experimentation and robotics deployment in a structured, reproducible architecture.

---

## Project Overview

This repository is divided into two complementary layers:

### 1. Machine Learning Layer
- Train and evaluate semantic segmentation models
- Manage MMSeg configs (Cityscapes, Oxford-IIIT Pets)
- Run inference and generate overlay visualizations

### 2. Robotics / Deployment Layer
- ROS2 nodes for perception
- CARLA RGB stream bridge
- Real-time segmentation publishing
- Modular topic-based architecture

This separation mirrors real-world AV software stacks (research ↔ integration).

---

## System Architecture

    CARLA Simulator
          │
          ▼
    CARLA Bridge Node (ROS2)
          │
          ▼
    Semantic Segmentation Node (MMSeg)
          │
          ├── Segmentation Mask Topic
          └── Overlay Image Topic
          │
          ▼
    Visualization / Control Nodes

---

## Example Output

### Segmentation Overlay

![Segmentation Example](assets/demo_overlay.png)

---

## Repository Structure

    configs/              MMSeg model configurations
    scripts/              Training, inference, evaluation utilities
    ros/ros2_ws/          ROS2 workspace (CARLA + segmentation nodes)
    docs/                 Setup guides and technical notes
    assets/               Demo images and media

---

## Installation

### Recommended (Conda)

    conda env create -f environment.yml
    conda activate ros2seg

### Alternative (pip)

    pip install -r requirements.lock.txt

---

## Quickstart — Standalone Inference

    python scripts/infer_trained.py \
      --config configs/cityscapes/segformer_b0_cityscapes.py \
      --checkpoint <path_to_checkpoint.pth> \
      --img assets/street.jpg \
      --out-dir out/infer

---

## ROS2 + CARLA Integration

Workspace location:

    ros/ros2_ws/

Import external dependencies:

    vcs import src < deps.repos
    colcon build

See:

    docs/ros/runbook.md

for execution instructions.

---

## Adding a Demo GIF

Place your recorded demo here:

    assets/segmentation_demo.gif

Embed it in this README using:

    ## ROS2 + CARLA Demo
    ![ROS Demo](assets/segmentation_demo.gif)

To resize:

    <img src="assets/segmentation_demo.gif" width="800">

---

## Technical Stack

- Python
- PyTorch
- MMSegmentation / MMEngine
- ROS2
- CARLA
- OpenCV

---

## Development Focus

- Real-time segmentation streaming
- ROS2 topic optimization
- Model latency profiling
- Perception integration for AV pipelines

---

## License

This project is licensed under the MIT License — see the LICENSE file for details.
