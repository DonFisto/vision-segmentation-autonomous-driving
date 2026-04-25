#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import List, Tuple

import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D


@dataclass
class Track:
    track_id: int
    class_id: str
    score: float
    cx: float
    cy: float
    w: float
    h: float
    hits: int
    misses: int


class TrackingNode(Node):
    def __init__(self):
        super().__init__("tracking_node")

        # Parameters
        self.declare_parameter("input_topic", "/perception/detections")
        self.declare_parameter("output_topic", "/perception/tracks")
        self.declare_parameter("iou_threshold", 0.3)
        self.declare_parameter("max_misses", 5)
        self.declare_parameter("min_hits", 3)
        self.declare_parameter("ema_alpha", 0.8)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.max_misses = int(self.get_parameter("max_misses").value)
        self.min_hits = int(self.get_parameter("min_hits").value)
        self.ema_alpha = float(self.get_parameter("ema_alpha").value)

        self.tracks: List[Track] = []
        self.next_track_id = 1

        self.sub = self.create_subscription(
            Detection2DArray,
            self.input_topic,
            self.detections_callback,
            10,
        )

        self.pub = self.create_publisher(
            Detection2DArray,
            self.output_topic,
            10,
        )

        self.get_logger().info(f"Tracking node listening on {self.input_topic}")
        self.get_logger().info(f"Publishing tracks on {self.output_topic}")

    @staticmethod
    def bbox_to_xyxy(cx: float, cy: float, w: float, h: float) -> Tuple[float, float, float, float]:
        x1 = cx - w / 2.0
        y1 = cy - h / 2.0
        x2 = cx + w / 2.0
        y2 = cy + h / 2.0
        return x1, y1, x2, y2

    @staticmethod
    def iou(a: Tuple[float, float, float, float], b: Tuple[float, float, float, float]) -> float:
        ax1, ay1, ax2, ay2 = a
        bx1, by1, bx2, by2 = b

        inter_x1 = max(ax1, bx1)
        inter_y1 = max(ay1, by1)
        inter_x2 = min(ax2, bx2)
        inter_y2 = min(ay2, by2)

        inter_w = max(0.0, inter_x2 - inter_x1)
        inter_h = max(0.0, inter_y2 - inter_y1)
        inter_area = inter_w * inter_h

        area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
        area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
        union = area_a + area_b - inter_area

        if union <= 1e-6:
            return 0.0
        return inter_area / union

    @staticmethod
    def extract_det_info(det: Detection2D) -> Tuple[str, float, float, float, float, float]:
        bbox = det.bbox
        cx = float(bbox.center.position.x)
        cy = float(bbox.center.position.y)
        w = float(bbox.size_x)
        h = float(bbox.size_y)

        class_id = "obj"
        score = 0.0

        if len(det.results) > 0:
            best = max(det.results, key=lambda r: r.hypothesis.score)
            class_id = str(best.hypothesis.class_id) if best.hypothesis.class_id else "obj"
            score = float(best.hypothesis.score)

        return class_id, score, cx, cy, w, h

    def greedy_match(self, detections: List[Detection2D]) -> Tuple[List[Tuple[int, int]], List[int], List[int]]:
        """
        Returns:
          matches: list of (track_idx, det_idx)
          unmatched_tracks: list of track indices
          unmatched_dets: list of detection indices
        """
        if len(self.tracks) == 0 or len(detections) == 0:
            return [], list(range(len(self.tracks))), list(range(len(detections)))

        candidates = []

        for t_idx, track in enumerate(self.tracks):
            track_box = self.bbox_to_xyxy(track.cx, track.cy, track.w, track.h)

            for d_idx, det in enumerate(detections):
                class_id, _, cx, cy, w, h = self.extract_det_info(det)

                # Optional class consistency: only match same class
                if class_id != track.class_id:
                    continue

                det_box = self.bbox_to_xyxy(cx, cy, w, h)
                ov = self.iou(track_box, det_box)

                if ov >= self.iou_threshold:
                    candidates.append((ov, t_idx, d_idx))

        # Highest IoU first
        candidates.sort(key=lambda x: x[0], reverse=True)

        matched_tracks = set()
        matched_dets = set()
        matches = []

        for ov, t_idx, d_idx in candidates:
            if t_idx in matched_tracks or d_idx in matched_dets:
                continue
            matched_tracks.add(t_idx)
            matched_dets.add(d_idx)
            matches.append((t_idx, d_idx))

        unmatched_tracks = [i for i in range(len(self.tracks)) if i not in matched_tracks]
        unmatched_dets = [i for i in range(len(detections)) if i not in matched_dets]

        return matches, unmatched_tracks, unmatched_dets

    def update_track_with_detection(self, track: Track, det: Detection2D):
        class_id, score, cx, cy, w, h = self.extract_det_info(det)

        a = self.ema_alpha
        track.cx = a * track.cx + (1.0 - a) * cx
        track.cy = a * track.cy + (1.0 - a) * cy
        track.w = a * track.w + (1.0 - a) * w
        track.h = a * track.h + (1.0 - a) * h

        track.score = score
        track.class_id = class_id
        track.hits += 1
        track.misses = 0

    def create_track_from_detection(self, det: Detection2D):
        class_id, score, cx, cy, w, h = self.extract_det_info(det)

        track = Track(
            track_id=self.next_track_id,
            class_id=class_id,
            score=score,
            cx=cx,
            cy=cy,
            w=w,
            h=h,
            hits=1,
            misses=0,
        )
        self.next_track_id += 1
        self.tracks.append(track)

    def prune_tracks(self):
        self.tracks = [t for t in self.tracks if t.misses <= self.max_misses]

    def tracks_to_msg(self, header) -> Detection2DArray:
        msg = Detection2DArray()
        msg.header = header

        for track in self.tracks:
            # Only publish confirmed tracks
            if track.hits < self.min_hits:
                continue

            det = Detection2D()
            det.header = header

            bbox = BoundingBox2D()
            bbox.center.position.x = float(track.cx)
            bbox.center.position.y = float(track.cy)
            bbox.size_x = float(track.w)
            bbox.size_y = float(track.h)
            det.bbox = bbox

            # Reuse results field to carry class + score
            # Track ID can be encoded into class string for now
            if len(det.results) == 0:
                from vision_msgs.msg import ObjectHypothesisWithPose
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = f"{track.class_id}#{track.track_id}"
                hyp.hypothesis.score = float(track.score)
                det.results.append(hyp)

            msg.detections.append(det)

        return msg

    def detections_callback(self, msg: Detection2DArray):
        detections = list(msg.detections)

        matches, unmatched_tracks, unmatched_dets = self.greedy_match(detections)

        # Update matched tracks
        for t_idx, d_idx in matches:
            self.update_track_with_detection(self.tracks[t_idx], detections[d_idx])

        # Age unmatched tracks
        for t_idx in unmatched_tracks:
            self.tracks[t_idx].misses += 1

        # Create new tracks for unmatched detections
        for d_idx in unmatched_dets:
            self.create_track_from_detection(detections[d_idx])

        # Remove dead tracks
        self.prune_tracks()

        # Publish current confirmed tracks
        out_msg = self.tracks_to_msg(msg.header)
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
