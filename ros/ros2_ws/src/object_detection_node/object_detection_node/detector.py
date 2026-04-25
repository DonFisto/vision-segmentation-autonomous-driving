#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D
from vision_msgs.msg import ObjectHypothesisWithPose

import numpy as np
import cv2
from cv_bridge import CvBridge
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional


# Cityscapes trainIds (your seg node outputs 0..18)
CITYSCAPES_ID2NAME: Dict[int, str] = {
    0: "road",
    1: "sidewalk",
    2: "building",
    3: "wall",
    4: "fence",
    5: "pole",
    6: "traffic light",
    7: "traffic sign",
    8: "vegetation",
    9: "terrain",
    10: "sky",
    11: "person",
    12: "rider",
    13: "car",
    14: "truck",
    15: "bus",
    16: "train",
    17: "motorcycle",
    18: "bicycle",
}


def iou_xywh(a: Tuple[float, float, float, float], b: Tuple[float, float, float, float]) -> float:
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    ax1, ay1, ax2, ay2 = ax, ay, ax + aw, ay + ah
    bx1, by1, bx2, by2 = bx, by, bx + bw, by + bh

    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    union = (aw * ah) + (bw * bh) - inter
    return float(inter / union) if union > 1e-9 else 0.0


@dataclass
class Track:
    tid: int
    label: str
    box: Tuple[float, float, float, float]  # x,y,w,h
    hits: int = 1
    missed: int = 0


class ObjectDetectionNode(Node):
    """
    Turns a Cityscapes class-id mask (/perception/semantic_mask, mono8 0..18)
    into stable 2D detections (/perception/detections, vision_msgs/Detection2DArray)
    with:
      - per-class connected components
      - morphology cleanup
      - area threshold per class
      - simple IoU association (tracking)
      - EMA smoothing for reduced jitter
      - min_hits / max_age gating
    """

    def __init__(self):
        super().__init__("object_detection_node")

        self.bridge = CvBridge()

        # --- Params ---
        self.declare_parameter("mask_topic", "/perception/semantic_mask")
        self.declare_parameter("detections_topic", "/perception/detections")

        # Which classes to detect (names from CITYSCAPES_ID2NAME values)
        self.declare_parameter(
            "target_labels",
            ["car", "truck", "bus", "train", "motorcycle", "bicycle", "person", "rider", "traffic light", "traffic sign"],
        )

        # Morphology
        self.declare_parameter("kernel_size", 5)        # 3,5,7...
        self.declare_parameter("open_iter", 1)
        self.declare_parameter("close_iter", 2)

        # Tracking / smoothing
        self.declare_parameter("iou_match", 0.25)       # match threshold
        self.declare_parameter("ema_alpha", 0.80)       # 0.7..0.9 (higher = smoother, more lag)
        self.declare_parameter("min_hits", 3)           # publish only after seen N frames
        self.declare_parameter("max_age", 5)            # keep track alive for N missed frames

        # Area thresholds (pixels). You can tune these.
        self.declare_parameter("min_area_default", 250)
        self.declare_parameter("min_area_car", 1200)
        self.declare_parameter("min_area_truck", 1400)
        self.declare_parameter("min_area_bus", 1600)
        self.declare_parameter("min_area_train", 1600)
        self.declare_parameter("min_area_person", 350)
        self.declare_parameter("min_area_rider", 350)
        self.declare_parameter("min_area_motorcycle", 250)
        self.declare_parameter("min_area_bicycle", 250)
        self.declare_parameter("min_area_traffic_light", 80)
        self.declare_parameter("min_area_traffic_sign", 120)

        self.mask_topic = str(self.get_parameter("mask_topic").value)
        self.det_topic = str(self.get_parameter("detections_topic").value)

        self.target_labels: List[str] = list(self.get_parameter("target_labels").value)
        # Convert target labels -> target ids
        name2id = {v: k for k, v in CITYSCAPES_ID2NAME.items()}
        self.target_ids: List[int] = [name2id[n] for n in self.target_labels if n in name2id]

        k = int(self.get_parameter("kernel_size").value)
        k = k if k % 2 == 1 else k + 1
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        self.open_iter = int(self.get_parameter("open_iter").value)
        self.close_iter = int(self.get_parameter("close_iter").value)

        self.iou_match = float(self.get_parameter("iou_match").value)
        self.ema_alpha = float(self.get_parameter("ema_alpha").value)
        self.min_hits = int(self.get_parameter("min_hits").value)
        self.max_age = int(self.get_parameter("max_age").value)

        # per-label min area
        self.min_area_default = int(self.get_parameter("min_area_default").value)
        self.min_area_by_label: Dict[str, int] = {
            "car": int(self.get_parameter("min_area_car").value),
            "truck": int(self.get_parameter("min_area_truck").value),
            "bus": int(self.get_parameter("min_area_bus").value),
            "train": int(self.get_parameter("min_area_train").value),
            "person": int(self.get_parameter("min_area_person").value),
            "rider": int(self.get_parameter("min_area_rider").value),
            "motorcycle": int(self.get_parameter("min_area_motorcycle").value),
            "bicycle": int(self.get_parameter("min_area_bicycle").value),
            "traffic light": int(self.get_parameter("min_area_traffic_light").value),
            "traffic sign": int(self.get_parameter("min_area_traffic_sign").value),
        }

        # --- ROS I/O ---
        self.sub = self.create_subscription(Image, self.mask_topic, self.mask_callback, 10)
        self.pub = self.create_publisher(Detection2DArray, self.det_topic, 10)

        # --- Tracker state ---
        self.next_tid = 1
        # Track per label to avoid class flipping
        self.tracks: Dict[str, List[Track]] = {lbl: [] for lbl in self.target_labels}

        self.get_logger().info(f"Subscribing: {self.mask_topic} (mono8 class-id mask 0..18)")
        self.get_logger().info(f"Publishing : {self.det_topic} (vision_msgs/Detection2DArray)")
        self.get_logger().info(f"Targets    : {self.target_labels}")

    def _label_min_area(self, label: str) -> int:
        return int(self.min_area_by_label.get(label, self.min_area_default))

    def _associate_and_update(self, label: str, det_boxes: List[Tuple[float, float, float, float]]) -> List[Track]:
        """
        Greedy IoU matching + EMA smoothing, per label.
        """
        tracks = self.tracks.get(label, [])
        used_tracks = set()
        used_dets = set()

        # build all pair IoUs
        pairs: List[Tuple[float, int, int]] = []
        for ti, tr in enumerate(tracks):
            for di, db in enumerate(det_boxes):
                pairs.append((iou_xywh(tr.box, db), ti, di))
        pairs.sort(reverse=True, key=lambda x: x[0])

        # match
        for iou, ti, di in pairs:
            if iou < self.iou_match:
                break
            if ti in used_tracks or di in used_dets:
                continue
            used_tracks.add(ti)
            used_dets.add(di)

            # EMA smoothing on box
            alpha = self.ema_alpha
            ox, oy, ow, oh = tracks[ti].box
            nx, ny, nw, nh = det_boxes[di]
            sm = (
                alpha * ox + (1.0 - alpha) * nx,
                alpha * oy + (1.0 - alpha) * ny,
                alpha * ow + (1.0 - alpha) * nw,
                alpha * oh + (1.0 - alpha) * nh,
            )
            tracks[ti].box = sm
            tracks[ti].hits += 1
            tracks[ti].missed = 0

        # unmatched tracks age
        for ti, tr in enumerate(tracks):
            if ti not in used_tracks:
                tr.missed += 1

        # create new tracks for unmatched detections
        for di, db in enumerate(det_boxes):
            if di in used_dets:
                continue
            tracks.append(Track(tid=self.next_tid, label=label, box=db, hits=1, missed=0))
            self.next_tid += 1

        # prune old
        tracks = [t for t in tracks if t.missed <= self.max_age]
        self.tracks[label] = tracks
        return tracks

    def _mask_to_boxes_for_class(self, seg: np.ndarray, class_id: int, min_area: int) -> List[Tuple[float, float, float, float]]:
        """
        seg: uint8 class-id map
        returns list of boxes (x,y,w,h) for this class_id after cleanup.
        """
        bin_mask = (seg == class_id).astype(np.uint8) * 255

        if self.open_iter > 0:
            bin_mask = cv2.morphologyEx(bin_mask, cv2.MORPH_OPEN, self.kernel, iterations=self.open_iter)
        if self.close_iter > 0:
            bin_mask = cv2.morphologyEx(bin_mask, cv2.MORPH_CLOSE, self.kernel, iterations=self.close_iter)

        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(bin_mask, connectivity=8)

        boxes: List[Tuple[float, float, float, float]] = []
        for lab in range(1, num_labels):
            x, y, w, h, area = stats[lab]
            if area < min_area:
                continue
            boxes.append((float(x), float(y), float(w), float(h)))
        return boxes

    def mask_callback(self, msg: Image):
        # This is a class-id image (0..18), not a binary mask
        seg = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        if seg is None:
            return

        detections_msg = Detection2DArray()
        detections_msg.header = msg.header

        # For each target class, extract boxes, track, and publish stable tracks
        for label in self.target_labels:
            # map label->id (skip if invalid)
            class_id = None
            for k, v in CITYSCAPES_ID2NAME.items():
                if v == label:
                    class_id = k
                    break
            if class_id is None:
                continue

            min_area = self._label_min_area(label)
            det_boxes = self._mask_to_boxes_for_class(seg, class_id, min_area)
            tracks = self._associate_and_update(label, det_boxes)

            # publish only stable tracks
            for tr in tracks:
                if tr.hits < self.min_hits:
                    continue
                if tr.missed > 0:
                    continue

                x, y, w, h = tr.box
                bbox = BoundingBox2D()
                bbox.center.position.x = float(x + w / 2.0)
                bbox.center.position.y = float(y + h / 2.0)
                bbox.size_x = float(w)
                bbox.size_y = float(h)

                det = Detection2D()
                det.header = msg.header
                det.bbox = bbox

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(label)
                # A simple confidence proxy (0..1-ish): grows with hits but saturates
                hyp.hypothesis.score = float(min(1.0, 0.15 * tr.hits))
                det.results.append(hyp)

                detections_msg.detections.append(det)

        self.pub.publish(detections_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
