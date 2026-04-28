# Depth and Fusion Milestone

## Overview

This milestone documents the integration of monocular depth estimation and object-depth fusion into the CARLA + ROS2 autonomous-driving perception stack.

At this point, the project has a working perception chain:

```text
CARLA RGB camera
    ↓
semantic_seg_node
    ↓
object_detection_node
    ↓
tracking_node
    ↓
/perception/tracks

CARLA RGB camera
    ↓
depth_node
    ↓
/perception/depth/image
/perception/depth/colormap/compressed

/perception/tracks + /perception/depth/image
    ↓
fusion_node
    ↓
/perception/fused_objects
```

The system is now able to move from frame-level perception to object-level scene understanding by associating tracked objects with relative monocular depth estimates.

---

## Purpose of This Milestone

Before this milestone, the project could:

- stream RGB images from CARLA;
- run semantic segmentation;
- extract object detections;
- track detections over time with persistent IDs.

However, the system did not yet estimate how close or far objects were.

The objective of this milestone was to add:

1. **Monocular depth estimation** using Depth Anything V2.
2. **Compressed depth visualization** for Foxglove.
3. **Object-depth fusion** using tracked bounding boxes and depth maps.
4. **Structured fused-object output** suitable for later reactive navigation and mapping.

---

## Implemented Components

## 1. `depth_node`

### Role

The `depth_node` estimates relative monocular depth from the CARLA RGB stream.

### Input

```text
/carla/rgb/image_raw
```

### Outputs

```text
/perception/depth/image
/perception/depth/colormap/compressed
```

### Output Meaning

- `/perception/depth/image`  
  Raw relative depth image published as `32FC1`.  
  This is intended for computational use by other nodes.

- `/perception/depth/colormap/compressed`  
  JPEG-compressed visualization of the depth map.  
  This is intended for Foxglove visualization with reduced bandwidth.

### Model

The node uses:

```text
Depth Anything V2 Small
```

via Hugging Face Transformers:

```text
depth-anything/Depth-Anything-V2-Small-hf
```

The chosen model is lightweight enough for integration testing and suitable for real-time or near-real-time perception experiments.

---

## 2. `fusion_node`

### Role

The `fusion_node` combines tracked detections with the latest depth image.

### Inputs

```text
/perception/tracks
/perception/depth/image
```

### Output

```text
/perception/fused_objects
```

### Fusion Method

For each tracked object:

1. Read the bounding box from `/perception/tracks`.
2. Crop the corresponding region from `/perception/depth/image`.
3. Filter invalid depth values.
4. Compute depth statistics inside the box.
5. Publish a structured fused object.

Current computed statistics include:

```text
depth_median
depth_mean
depth_min
depth_p10
num_pixels
```

### Example Output

```json
{
  "stamp": {
    "sec": 123,
    "nanosec": 456789
  },
  "frame_id": "camera",
  "objects": [
    {
      "track_id": 7,
      "class": "car",
      "score": 0.91,
      "bbox": {
        "cx": 420.0,
        "cy": 260.0,
        "w": 80.0,
        "h": 50.0
      },
      "depth_median": 0.63,
      "depth_mean": 0.61,
      "depth_min": 0.41,
      "depth_p10": 0.48,
      "num_pixels": 3200
    }
  ]
}
```

---

## Environment Notes

Depth Anything introduced dependency constraints, so a separate environment was created:

```bash
conda create --name ros2depth --clone ros2seg
mamba activate ros2depth
```

The working dependency set was:

```text
torch==2.1.2+cu121
transformers==4.46.3
tokenizers==0.20.3
huggingface-hub==0.26.5
```

Important lesson:

> Do not upgrade Torch casually. The segmentation stack depends on the existing Torch/CUDA/MMCV compatibility.

When Transformers required a newer Torch version, the correct solution was to downgrade Transformers-related packages instead of upgrading Torch.

---

## Running the Depth Node

From the ROS2 workspace:

```bash
cd ~/vision-segmentation-autonomous-driving/ros/ros2_ws
mamba activate ros2depth
source install/setup.bash

ros2 run depth_node depth_node
```

With lower JPEG quality for reduced Foxglove bandwidth:

```bash
ros2 run depth_node depth_node --ros-args -p jpeg_quality:=50
```

Useful checks:

```bash
ros2 topic hz /perception/depth/image
ros2 topic hz /perception/depth/colormap/compressed
```

---

## Running the Fusion Node

Make sure the following nodes are already running:

```text
carla_bridge_node
semantic_seg_node
object_detection_node
tracking_node
depth_node
```

Then run:

```bash
cd ~/vision-segmentation-autonomous-driving/ros/ros2_ws
mamba activate ros2depth
source install/setup.bash

ros2 run fusion_node fusion_node
```

Check output:

```bash
ros2 topic echo /perception/fused_objects
```

---

## Foxglove Visualization

Recommended topic for depth visualization:

```text
/perception/depth/colormap/compressed
```

Recommended Foxglove bridge whitelist:

```bash
ros2 run foxglove_bridge foxglove_bridge --ros-args \
  -p topic_whitelist:="[/carla/rgb/image_raw/compressed,/perception/semantic_overlay/compressed,/perception/detections_overlay/compressed,/perception/depth/colormap/compressed,/perception/detections,/perception/tracks,/perception/fused_objects]"
```

Raw depth should usually not be visualized unless needed:

```text
/perception/depth/image
```

It is useful for fusion, but heavier than the compressed colormap.

---

## Validation Performed

The depth model was first tested offline on recollected CARLA images.

Observed behavior:

- road near the ego vehicle appeared close;
- distant buildings appeared far;
- cars appeared as foreground objects;
- vertical structures such as trees and poles were visible in the depth map.

This confirmed that Depth Anything was producing plausible relative depth on CARLA scenes.

After ROS2 integration, both:

```text
depth_node
fusion_node
```

were successfully launched and confirmed to work in the running perception stack.

---

## Current Capabilities

The perception stack can now produce:

```text
tracked object + relative depth estimate
```

Example interpretation:

```text
car#7            -> medium-close
traffic light#3 -> far
pedestrian#2    -> close
truck#1         -> medium
```

This is an important step because the system no longer only recognizes objects. It starts to estimate their relevance for driving based on proximity.

---

## Limitations

### 1. Depth is relative, not metric

Depth Anything does not directly output meters.

Current values should be interpreted as relative depth, useful for ranking objects by proximity but not yet for precise distance estimation.

Future work may calibrate relative depth using CARLA ground-truth depth.

---

### 2. Bounding-box depth can include background

The current fusion node uses the full bounding box region. This can include:

- road pixels;
- sky pixels;
- nearby background;
- occluding objects;
- empty space around thin objects.

This may make depth statistics noisy.

---

### 3. Small objects remain difficult

For classes such as:

```text
traffic light
traffic sign
pedestrian
bicycle
motorcycle
```

small bounding boxes may contain few valid depth pixels, and model errors may be more visible.

---

### 4. No temporal smoothing yet

The fusion node currently computes depth per frame. Depth estimates can flicker across frames.

Future versions should smooth depth per track ID.

---

### 5. No semantic masking inside the box yet

The fusion node does not yet use the semantic mask to keep only pixels belonging to the object class. This would improve object-depth estimation.

---

## Recommended Next Improvements

### 1. Central-region depth

Instead of using the full bounding box, use only the central region:

```text
center 50 percent of bbox width and height
```

This reduces background contamination.

---

### 2. Per-track depth smoothing

Maintain a dictionary:

```text
track_id -> smoothed_depth
```

with exponential smoothing:

```text
depth_smooth = alpha * previous_depth + (1 - alpha) * current_depth
```

This will make fused outputs more stable.

---

### 3. Semantic-mask-aware fusion

Use the segmentation mask to select only pixels inside the bounding box that belong to the tracked object class.

This is especially useful for cars, pedestrians and bikes.

---

### 4. Relative-to-metric calibration

Use CARLA ground-truth depth to learn a simple mapping:

```text
relative_depth -> approximate meters
```

Possible approaches:

- linear scaling;
- inverse-depth fit;
- per-scene normalization;
- regression using sampled objects.

---

### 5. Fused-object risk score

Compute a simple risk score:

```text
risk = object_class_weight / depth
```

or:

```text
risk = proximity_score + centerline_score + class_priority
```

This would prepare the system for reactive navigation.

---

## Next Milestone

The next major milestone should be:

```text
fusion_node -> reactive_navigation_node
```

The first reactive navigation node should use fused objects to:

- slow down if a close object appears in front;
- stop if an object is very close;
- steer away from close obstacles;
- keep moving if the center path is clear.

This will close the loop from perception to action.

---

## Technical Significance

This milestone is important because it transforms the project from a perception-only visualization system into a scene-understanding pipeline.

Before this milestone:

```text
The system knew what objects were present.
```

After this milestone:

```text
The system can estimate which tracked objects are closer or farther away.
```

This is a necessary step toward:

- obstacle-aware navigation;
- local mapping;
- free-space estimation;
- SLAM-like functionality;
- future autonomous behavior.
