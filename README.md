# 🚗 Vision-Based Perception Stack for Autonomous Driving

Semantic Segmentation + Tracking + Depth Estimation + ROS2 + CARLA

This project implements a modular perception pipeline for autonomous driving simulation.  
It combines deep learning-based semantic segmentation, object extraction, tracking, monocular depth estimation, and ROS2 integration with CARLA simulator streaming.

> Objective: bridge ML experimentation and robotics deployment in a structured, reproducible autonomous-driving perception architecture.

---

## Demo



<p align="center">
  <img src="assets/AD_Project_Demo.gif" alt="CARLA perception stack demo" width="800"/>
</p>


---

## Project Overview

This repository is divided into complementary layers.

### 1. Machine Learning Layer

This layer focuses on model training, evaluation, and dataset preparation.

- Train and evaluate semantic segmentation models.
- Manage MMSegmentation and MMEngine configs.
- Fine-tune SegFormer models on CARLA-generated datasets.
- Convert CARLA semantic labels to Cityscapes-19 format.
- Filter and prune synthetic datasets.
- Generate validation overlays and qualitative results.
- Experiment with targeted datasets such as:
  - sign-heavy data,
  - traffic-light-heavy data,
  - pedestrian and VRU-heavy data.

### 2. Simulation and Dataset Layer

This layer handles CARLA data generation and synthetic dataset engineering.

- CARLA RGB and semantic ground-truth collection.
- Multi-map and multi-weather data gathering.
- Traffic and pedestrian generation.
- Map-based train and validation splits.
- Specialized recollection scripts for rare classes.
- Dataset curation through filtering and static-frame pruning.

### 3. Robotics and Deployment Layer

This layer integrates the perception stack into ROS2.

- CARLA RGB stream bridge.
- Real-time semantic segmentation node.
- Object extraction from segmentation masks.
- Object tracking with persistent IDs.
- Monocular depth estimation using Depth Anything.
- Fusion of tracked objects with relative depth.
- Compressed visualization topics for Foxglove.
- Modular topic-based architecture.

This separation mirrors real-world autonomous-driving software stacks, where model development, simulation, deployment, and evaluation are handled as connected but independent layers.

---

## Current System Architecture

```text
CARLA Simulator
      |
      v
CARLA Bridge Node
      |
      |-- /carla/rgb/image_raw
      |
      v
Semantic Segmentation Node
      |
      |-- /perception/semantic_mask
      |-- /perception/semantic_overlay/compressed
      |
      v
Object Detection Node
      |
      |-- /perception/detections
      |
      v
Tracking Node
      |
      |-- /perception/tracks
      |
      v
Fusion Node  <---------------- Depth Node
      |                         ^
      |                         |
      |                 /carla/rgb/image_raw
      |                         |
      |                 /perception/depth/image
      |                 /perception/depth/colormap/compressed
      |
      v
/perception/fused_objects
```

The current pipeline converts raw CARLA RGB images into structured scene understanding:

```text
RGB image
-> semantic segmentation
-> object-level detections
-> temporally stable tracks
-> relative monocular depth
-> object-depth fusion
```

---

## Example Output

### Segmentation Overlay

![Segmentation Example](assets/demo_overlay.png)

---

### CARLA Segmentation and Object Tracking

![Object detection and tracking example in CARLA](assets/Segmentatation+Overlay+Tracking.png)

---

## Main ROS2 Topics

Typical active topics include:

```text
/carla/rgb/image_raw
/carla/rgb/image_raw/compressed

/perception/semantic_mask
/perception/semantic_overlay/compressed

/perception/detections
/perception/detections_overlay/compressed

/perception/tracks

/perception/depth/image
/perception/depth/colormap/compressed

/perception/fused_objects
```

---

## Repository Structure

```text
configs/
  MMSegmentation model configurations and fine-tuning configs

scripts/
  Training, inference, evaluation and overlay-generation utilities

carla_tools/
  CARLA dataset collection, conversion, filtering and pruning scripts

ros/ros2_ws/
  ROS2 workspace containing CARLA bridge and perception nodes

docs/
  Setup guides, technical notes and development documentation

assets/
  Demo images, overlays, GIFs and media used in the README
```

---

## Implemented ROS2 Nodes

### carla_bridge_node

Connects CARLA to ROS2.

Responsibilities:

- Spawn or connect to the CARLA ego camera.
- Publish RGB images.
- Optionally receive control commands.

---

### semantic_seg_node

Runs semantic segmentation inference on CARLA RGB images.

Responsibilities:

- Load MMSegmentation config and checkpoint.
- Run SegFormer inference.
- Publish semantic masks.
- Publish compressed semantic overlays.

---

### object_detection_node

Extracts object-level detections from semantic segmentation masks.

Responsibilities:

- Identify relevant classes.
- Extract connected components and bounding boxes.
- Publish detections as vision_msgs/Detection2DArray.

---

### tracking_node

Performs simple tracking-by-detection.

Responsibilities:

- Associate detections across frames using IoU.
- Assign persistent track IDs.
- Smooth bounding boxes using EMA.
- Tolerate short missed detections.
- Publish /perception/tracks.

Example track labels:

```text
car#7
traffic light#3
pedestrian#2
```

---

### depth_node

Runs Depth Anything V2 for monocular relative depth estimation.

Responsibilities:

- Subscribe to RGB images.
- Infer relative depth.
- Publish raw float depth image.
- Publish compressed depth colormap for Foxglove.

Outputs:

```text
/perception/depth/image
/perception/depth/colormap/compressed
```

---

### fusion_node

Fuses tracked objects with relative depth.

Responsibilities:

- Subscribe to /perception/tracks.
- Subscribe to /perception/depth/image.
- Compute depth statistics inside each tracked bounding box.
- Publish structured object-level fused perception.

Example fused output:

```json
{
  "track_id": 7,
  "class": "car",
  "score": 0.91,
  "depth_median": 0.63,
  "depth_min": 0.41
}
```

---

## Dataset Pipeline

The project includes scripts for generating and curating CARLA datasets.

### Raw CARLA Collection Format

```text
datasets/carla_sign_heavy/
  Town03/
    ClearNoon/
      rgb/
      sem_raw/
    CloudyNoon/
      rgb/
      sem_raw/
```

Each sample contains:

- RGB frame.
- Raw CARLA semantic segmentation label map.

---

### Conversion to Cityscapes-19

CARLA semantic labels are converted into Cityscapes-19 format for compatibility with pretrained segmentation models.

Typical output:

```text
dataset_cityscapes19/
  images/
    train/
    val/
  labels/
    train/
    val/
```

---

### Filtering and Pruning

Raw simulator data can contain many low-value frames. The repository includes filtering and pruning scripts to:

- Keep frames with relevant target-class pixels.
- Retain a bounded amount of context frames.
- Remove near-duplicates.
- Reduce static sequences.
- Improve class balance.

Targeted datasets include:

- sign-heavy datasets,
- traffic-light-heavy datasets,
- vehicle-heavy datasets,
- pedestrian, rider, bicycle and motorcycle-heavy datasets.

---

## Model Training

Training is based on MMSegmentation and SegFormer.

Typical fine-tuning stages explored:

1. Cityscapes-pretrained baseline.
2. CARLA-only fine-tuning.
3. CARLA and Cityscapes mixed fine-tuning.
4. Sign-heavy refinement.
5. VRU-heavy refinement.

Important lesson:

> Global mIoU is not always the best model-selection criterion for autonomous driving.  
> Per-class IoU and downstream usefulness for tracking and fusion may matter more.

---

## Installation

### Recommended Conda or Mamba Environment

```bash
conda env create -f environment.yml
conda activate ros2seg
```

or, if using Mamba:

```bash
mamba env create -f environment.yml
mamba activate ros2seg
```

### Alternative Pip Installation

```bash
pip install -r requirements.lock.txt
```

---

## Depth Environment

Depth Anything dependencies may require a separate environment to avoid breaking the segmentation stack.

Recommended approach:

```bash
conda create --name ros2depth --clone ros2seg
mamba activate ros2depth
```

Working dependency set used for Depth Anything integration:

```text
torch==2.1.2+cu121
transformers==4.46.3
tokenizers==0.20.3
huggingface-hub==0.26.5
```

Important:

> Do not casually upgrade Torch, since it may break MMSegmentation, CUDA, or MMCV compatibility.

---

## Quickstart: Standalone Segmentation Inference

```bash
python scripts/infer_trained.py \
  --config configs/cityscapes/segformer_b0_cityscapes.py \
  --checkpoint <path_to_checkpoint.pth> \
  --img assets/street.jpg \
  --out-dir out/infer
```

---

## Quickstart: ROS2 and CARLA Integration

Workspace location:

```text
ros/ros2_ws/
```

Build the ROS2 workspace:

```bash
cd ros/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Run the CARLA bridge:

```bash
ros2 run carla_bridge_node bridge_node
```

Run semantic segmentation:

```bash
ros2 run semantic_seg_node seg_node
```

Run object detection:

```bash
ros2 run object_detection_node detector
```

Run tracking:

```bash
ros2 run tracking_node tracking_node
```

Run depth estimation:

```bash
ros2 run depth_node depth_node
```

Run fusion:

```bash
ros2 run fusion_node fusion_node
```

See:

```text
docs/ros/runbook.md
```

for execution instructions and troubleshooting notes.

---

## Foxglove Visualization

Recommended compressed topics for lower-lag visualization:

```text
/carla/rgb/image_raw/compressed
/perception/semantic_overlay/compressed
/perception/detections_overlay/compressed
/perception/depth/colormap/compressed
/perception/detections
/perception/tracks
/perception/fused_objects
```

Example Foxglove bridge command:

```bash
ros2 run foxglove_bridge foxglove_bridge --ros-args \
  -p topic_whitelist:="[/carla/rgb/image_raw/compressed,/perception/semantic_overlay/compressed,/perception/detections_overlay/compressed,/perception/depth/colormap/compressed,/perception/detections,/perception/tracks,/perception/fused_objects]"
```

---

## Technical Stack

- Python
- PyTorch
- MMSegmentation
- MMEngine
- SegFormer
- Depth Anything V2
- Transformers and Hugging Face
- ROS2
- CARLA
- OpenCV
- Foxglove
- NumPy
- cv_bridge
- vision_msgs

---

## Development Focus

Current development focuses on:

- real-time segmentation streaming,
- robust CARLA dataset generation,
- rare-class dataset recollection,
- ROS2 topic optimization,
- object tracking stability,
- monocular depth integration,
- object-depth fusion,
- compressed visualization,
- future reactive navigation,
- future local mapping and SLAM-like functionality.

---

## Project Roadmap

### Current Stage

```text
segmentation -> detection -> tracking -> depth -> fusion
```

### Next Stage

```text
fusion -> reactive navigation
```

### Later Stage

```text
segmentation + depth -> local free-space map
visual odometry -> keyframe mapping
mapping -> SLAM-like prototype
```

Long-term, this project may serve as the technical foundation for a future TFG related to autonomous-driving perception, depth-aware tracking, local mapping, or modular automotive perception systems.

---

## License

This project is licensed under the MIT License. See the LICENSE file for details.
