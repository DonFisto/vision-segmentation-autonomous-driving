#!/usr/bin/env bash

# -------- CONFIG --------
BAG_DIR=~/bags
MAX_SIZE_BYTES=$((1024 * 1024 * 1024))   # 1 GB
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
OUTPUT=${BAG_DIR}/carla_run_${TIMESTAMP}

TOPICS=(
  /carla/rgb/image_raw
  /perception/semantic_mask
  /perception/semantic_overlay
  /carla/hero/odometry
)

# -------- SETUP --------
mkdir -p "${BAG_DIR}"

echo "Recording bag to:"
echo "  ${OUTPUT}"
echo "Max file size: 1 GB (auto-split enabled)"
echo "Press CTRL+C to stop."
echo "----------------------------------------"

# -------- RECORD --------
ros2 bag record \
  -o "${OUTPUT}" \
  --max-bag-size "${MAX_SIZE_BYTES}" \
  "${TOPICS[@]}"
