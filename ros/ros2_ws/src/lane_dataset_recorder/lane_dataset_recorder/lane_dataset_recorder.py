#!/usr/bin/env python3

from __future__ import annotations

import json
import math
import os
import queue
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

import carla
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class LaneDatasetRecorder(Node):
    """
    Records synchronized RGB and semantic-segmentation images directly from
    CARLA.

    The node attaches two cameras with identical parameters and transforms to
    an existing hero vehicle. Frames are paired using CARLA's frame number.

    Dataset outputs:
      rgb/
      semantic_labels/
      roadline_masks/
      metadata.jsonl
      manifest.json

    The semantic_labels images retain every CARLA semantic class ID.
    The roadline_masks images contain a binary RoadLine mask.
    """

    def __init__(self) -> None:
        super().__init__("lane_dataset_recorder")

        # --------------------------------------------------------------
        # CARLA connection and hero selection
        # --------------------------------------------------------------
        self.declare_parameter("host", "localhost")
        self.declare_parameter("port", 2000)
        self.declare_parameter("client_timeout_sec", 10.0)

        self.declare_parameter("hero_role_name", "hero")
        self.declare_parameter("hero_wait_timeout_sec", 20.0)

        # Explicit CARLA actor ID. Negative disables explicit selection.
        self.declare_parameter("vehicle_id", -1)

        # Prefer the vehicle carrying the already-running bridge RGB camera.
        # This is more reliable than role_name when all actors use "autopilot".
        self.declare_parameter("prefer_rgb_sensor_parent", True)

        # Final fallback, safe only when exactly one vehicle exists.
        self.declare_parameter("allow_single_vehicle_fallback", True)

        # --------------------------------------------------------------
        # Aligned camera configuration
        # Match the existing carla_bridge_node camera.
        # --------------------------------------------------------------
        self.declare_parameter("width", 800)
        self.declare_parameter("height", 600)
        self.declare_parameter("fov", 90.0)
        self.declare_parameter("fps", 15.0)

        self.declare_parameter("camera_x", 1.5)
        self.declare_parameter("camera_y", 0.0)
        self.declare_parameter("camera_z", 2.4)
        self.declare_parameter("camera_roll", 0.0)
        self.declare_parameter("camera_pitch", 0.0)
        self.declare_parameter("camera_yaw", 0.0)

        # --------------------------------------------------------------
        # Dataset output
        # --------------------------------------------------------------
        self.declare_parameter(
            "output_dir",
            "/home/danielmartinez/datasets/carla_lane_dataset",
        )
        self.declare_parameter("run_name", "")

        # Record one pair every N synchronized camera frames.
        # At 15 FPS, sample_every_n=3 records approximately 5 samples/s.
        self.declare_parameter("sample_every_n", 3)

        # 0 means unlimited.
        self.declare_parameter("max_samples", 5000)

        # Print collection progress after this many successfully saved samples.
        self.declare_parameter("progress_every_samples", 100)

        self.declare_parameter("rgb_jpeg_quality", 95)
        self.declare_parameter("save_full_semantic_labels", True)

        # CARLA 0.9.16 RoadLine semantic class.
        self.declare_parameter("roadline_label_id", 24)

        # --------------------------------------------------------------
        # Synchronization and buffering
        # --------------------------------------------------------------
        self.declare_parameter("save_queue_size", 64)
        self.declare_parameter("max_pending_frames", 64)
        self.declare_parameter("max_timestamp_difference_sec", 0.001)

        # --------------------------------------------------------------
        # ROS diagnostics
        # --------------------------------------------------------------
        self.declare_parameter("publish_preview", True)
        self.declare_parameter(
            "preview_rgb_topic",
            "/dataset/lane_recorder/rgb/compressed",
        )
        self.declare_parameter(
            "preview_mask_topic",
            "/dataset/lane_recorder/roadline_mask/compressed",
        )
        self.declare_parameter(
            "status_topic",
            "/dataset/lane_recorder/status",
        )

        self.width = int(self.p("width"))
        self.height = int(self.p("height"))
        self.fov = float(self.p("fov"))
        self.fps = float(self.p("fps"))

        self.sample_every_n = max(
            1,
            int(self.p("sample_every_n")),
        )
        self.max_samples = max(
            0,
            int(self.p("max_samples")),
        )
        self.roadline_label_id = int(
            self.p("roadline_label_id")
        )

        self.lock = threading.Lock()
        self.stop_event = threading.Event()

        self.rgb_by_frame: dict[int, dict[str, Any]] = {}
        self.semantic_by_frame: dict[int, dict[str, Any]] = {}

        self.save_queue: queue.Queue[dict[str, Any]] = queue.Queue(
            maxsize=max(1, int(self.p("save_queue_size")))
        )

        self.received_rgb_frames = 0
        self.received_semantic_frames = 0
        self.synchronized_pairs = 0
        self.enqueued_samples = 0
        self.saved_samples = 0
        self.dropped_samples = 0
        self.timestamp_mismatches = 0
        self.write_errors = 0

        self.collection_started_monotonic = time.monotonic()
        self.last_progress_reported_sample = 0

        self.rgb_sensor: Optional[carla.Sensor] = None
        self.semantic_sensor: Optional[carla.Sensor] = None
        self.hero_vehicle: Optional[carla.Vehicle] = None

        self.publish_preview = bool(
            self.p("publish_preview")
        )

        self.preview_rgb_pub = self.create_publisher(
            CompressedImage,
            str(self.p("preview_rgb_topic")),
            10,
        )
        self.preview_mask_pub = self.create_publisher(
            CompressedImage,
            str(self.p("preview_mask_topic")),
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.p("status_topic")),
            10,
        )

        self.client = carla.Client(
            str(self.p("host")),
            int(self.p("port")),
        )
        self.client.set_timeout(
            float(self.p("client_timeout_sec"))
        )

        self.world = self.client.get_world()
        self.map_name = self.world.get_map().name.split("/")[-1]

        self.camera_relative_transform = carla.Transform(
            carla.Location(
                x=float(self.p("camera_x")),
                y=float(self.p("camera_y")),
                z=float(self.p("camera_z")),
            ),
            carla.Rotation(
                roll=float(self.p("camera_roll")),
                pitch=float(self.p("camera_pitch")),
                yaw=float(self.p("camera_yaw")),
            ),
        )

        self.hero_vehicle = self.find_hero_vehicle()

        self.run_dir = self.create_run_directory()
        self.rgb_dir = self.run_dir / "rgb"
        self.semantic_dir = self.run_dir / "semantic_labels"
        self.roadline_dir = self.run_dir / "roadline_masks"

        self.rgb_dir.mkdir(parents=True, exist_ok=True)
        self.semantic_dir.mkdir(parents=True, exist_ok=True)
        self.roadline_dir.mkdir(parents=True, exist_ok=True)

        self.metadata_path = self.run_dir / "metadata.jsonl"
        self.manifest_path = self.run_dir / "manifest.json"

        self.write_manifest()

        self.save_thread = threading.Thread(
            target=self.save_worker,
            name="lane_dataset_save_worker",
            daemon=True,
        )
        self.save_thread.start()

        self.spawn_sensors()

        self.status_timer = self.create_timer(
            1.0,
            self.publish_status,
        )

        self.get_logger().info(
            f"Recording lane dataset to: {self.run_dir}"
        )
        self.get_logger().info(
            f"Attached cameras to hero actor {self.hero_vehicle.id}"
        )
        self.get_logger().info(
            f"RoadLine semantic label ID: {self.roadline_label_id}"
        )

    def p(self, name: str) -> Any:
        return self.get_parameter(name).value

    # ==============================================================
    # CARLA setup
    # ==============================================================

    def find_hero_vehicle(self) -> carla.Vehicle:
        role_name = str(self.p("hero_role_name"))
        requested_vehicle_id = int(self.p("vehicle_id"))
        prefer_rgb_parent = bool(
            self.p("prefer_rgb_sensor_parent")
        )
        allow_single_fallback = bool(
            self.p("allow_single_vehicle_fallback")
        )

        deadline = time.monotonic() + float(
            self.p("hero_wait_timeout_sec")
        )
        last_summary = "no vehicles"

        while time.monotonic() < deadline:
            vehicles = list(
                self.world.get_actors().filter("vehicle.*")
            )

            last_summary = ", ".join(
                (
                    f"id={vehicle.id}, "
                    f"type={vehicle.type_id}, "
                    f"role={vehicle.attributes.get('role_name')!r}"
                )
                for vehicle in vehicles
            ) or "no vehicles"

            # 1. Explicit actor selection.
            if requested_vehicle_id >= 0:
                for vehicle in vehicles:
                    if vehicle.id == requested_vehicle_id:
                        self.get_logger().info(
                            "Using explicitly selected vehicle: "
                            f"id={vehicle.id}, type={vehicle.type_id}"
                        )
                        return vehicle

            # 2. Conventional role_name selection.
            for vehicle in vehicles:
                if vehicle.attributes.get("role_name") == role_name:
                    self.get_logger().info(
                        "Found vehicle by role_name: "
                        f"id={vehicle.id}, type={vehicle.type_id}"
                    )
                    return vehicle

            # 3. Find the parent of the bridge's existing RGB camera.
            if prefer_rgb_parent:
                matching_parents = {}

                for sensor in self.world.get_actors().filter(
                    "sensor.camera.rgb"
                ):
                    parent = sensor.parent

                    if parent is None:
                        continue

                    if not parent.type_id.startswith("vehicle."):
                        continue

                    # Prefer a camera matching the configured bridge camera.
                    try:
                        width_matches = (
                            int(sensor.attributes.get("image_size_x", -1))
                            == self.width
                        )
                        height_matches = (
                            int(sensor.attributes.get("image_size_y", -1))
                            == self.height
                        )
                        fov_matches = abs(
                            float(sensor.attributes.get("fov", -1.0))
                            - self.fov
                        ) < 0.01
                    except (TypeError, ValueError):
                        width_matches = False
                        height_matches = False
                        fov_matches = False

                    if width_matches and height_matches and fov_matches:
                        matching_parents[parent.id] = parent

                if len(matching_parents) == 1:
                    vehicle = next(iter(matching_parents.values()))
                    self.get_logger().warning(
                        "No hero role found; selected parent of the "
                        "existing matching RGB camera: "
                        f"id={vehicle.id}, type={vehicle.type_id}, "
                        f"role={vehicle.attributes.get('role_name')!r}"
                    )
                    return vehicle

                if len(matching_parents) > 1:
                    self.get_logger().warning(
                        "Multiple vehicles carry matching RGB cameras: "
                        + ", ".join(
                            str(actor_id)
                            for actor_id in matching_parents
                        )
                    )

            # 4. Safe fallback when only one vehicle exists.
            if allow_single_fallback and len(vehicles) == 1:
                vehicle = vehicles[0]
                self.get_logger().warning(
                    "No hero role or matching camera parent found; "
                    "using the only CARLA vehicle: "
                    f"id={vehicle.id}, type={vehicle.type_id}, "
                    f"role={vehicle.attributes.get('role_name')!r}"
                )
                return vehicle

            self.get_logger().info(
                f"Waiting for ego vehicle. Available: {last_summary}"
            )
            time.sleep(0.5)

        raise RuntimeError(
            "Could not identify the ego vehicle. "
            f"Last available vehicles: {last_summary}. "
            "Use -p vehicle_id:=<actor_id> to select it explicitly."
        )


    def configure_camera_blueprint(
        self,
        blueprint: carla.ActorBlueprint,
    ) -> None:
        blueprint.set_attribute(
            "image_size_x",
            str(self.width),
        )
        blueprint.set_attribute(
            "image_size_y",
            str(self.height),
        )
        blueprint.set_attribute(
            "fov",
            str(self.fov),
        )
        blueprint.set_attribute(
            "sensor_tick",
            str(1.0 / max(self.fps, 1.0)),
        )

    def spawn_sensors(self) -> None:
        blueprint_library = self.world.get_blueprint_library()

        rgb_blueprint = blueprint_library.find(
            "sensor.camera.rgb"
        )
        semantic_blueprint = blueprint_library.find(
            "sensor.camera.semantic_segmentation"
        )

        self.configure_camera_blueprint(rgb_blueprint)
        self.configure_camera_blueprint(semantic_blueprint)

        try:
            self.rgb_sensor = self.world.spawn_actor(
                rgb_blueprint,
                self.camera_relative_transform,
                attach_to=self.hero_vehicle,
                attachment_type=carla.AttachmentType.Rigid,
            )

            self.semantic_sensor = self.world.spawn_actor(
                semantic_blueprint,
                self.camera_relative_transform,
                attach_to=self.hero_vehicle,
                attachment_type=carla.AttachmentType.Rigid,
            )

            self.rgb_sensor.listen(self.on_rgb_image)
            self.semantic_sensor.listen(
                self.on_semantic_image
            )

        except Exception:
            self.destroy_sensors()
            raise

    # ==============================================================
    # Dataset setup and metadata
    # ==============================================================

    def create_run_directory(self) -> Path:
        root = Path(
            os.path.expanduser(
                str(self.p("output_dir"))
            )
        )

        requested_name = str(self.p("run_name")).strip()

        if requested_name:
            run_name = requested_name
        else:
            timestamp = datetime.now().strftime(
                "%Y%m%d_%H%M%S"
            )
            run_name = f"{timestamp}_{self.map_name}"

        run_dir = root / run_name
        run_dir.mkdir(parents=True, exist_ok=False)
        return run_dir

    def camera_intrinsics(self) -> dict[str, float]:
        focal_length = self.width / (
            2.0
            * math.tan(
                math.radians(self.fov) / 2.0
            )
        )

        return {
            "fx": float(focal_length),
            "fy": float(focal_length),
            "cx": float(self.width / 2.0),
            "cy": float(self.height / 2.0),
        }

    @staticmethod
    def transform_to_dict(
        transform: carla.Transform,
    ) -> dict[str, Any]:
        return {
            "location": {
                "x": float(transform.location.x),
                "y": float(transform.location.y),
                "z": float(transform.location.z),
            },
            "rotation": {
                "roll": float(transform.rotation.roll),
                "pitch": float(transform.rotation.pitch),
                "yaw": float(transform.rotation.yaw),
            },
        }

    def weather_to_dict(self) -> dict[str, float]:
        weather = self.world.get_weather()

        fields = [
            "cloudiness",
            "precipitation",
            "precipitation_deposits",
            "wind_intensity",
            "sun_azimuth_angle",
            "sun_altitude_angle",
            "fog_density",
            "fog_distance",
            "fog_falloff",
            "wetness",
            "scattering_intensity",
            "mie_scattering_scale",
            "rayleigh_scattering_scale",
            "dust_storm",
        ]

        return {
            field: float(getattr(weather, field))
            for field in fields
            if hasattr(weather, field)
        }

    def write_manifest(self) -> None:
        manifest = {
            "dataset_version": 1,
            "created_at_utc": datetime.now(
                timezone.utc
            ).isoformat(),
            "map_name": self.map_name,
            "hero_role_name": str(
                self.p("hero_role_name")
            ),
            "camera": {
                "width": self.width,
                "height": self.height,
                "fov_deg": self.fov,
                "fps": self.fps,
                "relative_transform": self.transform_to_dict(
                    self.camera_relative_transform
                ),
                "intrinsics": self.camera_intrinsics(),
            },
            "semantic_labels": {
                "roadline_label_id": self.roadline_label_id,
                "full_semantic_labels_saved": bool(
                    self.p("save_full_semantic_labels")
                ),
            },
            "sampling": {
                "sample_every_n": self.sample_every_n,
                "max_samples": self.max_samples,
            },
            "weather_at_start": self.weather_to_dict(),
        }

        self.manifest_path.write_text(
            json.dumps(
                manifest,
                indent=2,
                sort_keys=True,
            )
            + "\n"
        )

    # ==============================================================
    # Synchronized sensor callbacks
    # ==============================================================

    def on_rgb_image(self, image: carla.Image) -> None:
        bgra = np.frombuffer(
            image.raw_data,
            dtype=np.uint8,
        ).reshape(
            (image.height, image.width, 4)
        )

        bgr = bgra[:, :, :3].copy()

        packet = {
            "frame": int(image.frame),
            "timestamp": float(image.timestamp),
            "camera_transform": self.transform_to_dict(
                image.transform
            ),
            "bgr": bgr,
        }

        with self.lock:
            self.received_rgb_frames += 1
            self.rgb_by_frame[int(image.frame)] = packet
            pair = self.take_pair_locked(int(image.frame))
            self.prune_pending_locked()

        if pair is not None:
            self.handle_pair(pair)

    def on_semantic_image(
        self,
        image: carla.Image,
    ) -> None:
        bgra = np.frombuffer(
            image.raw_data,
            dtype=np.uint8,
        ).reshape(
            (image.height, image.width, 4)
        )

        # CARLA raw semantic tag is stored in the red channel.
        labels = bgra[:, :, 2].copy()

        packet = {
            "frame": int(image.frame),
            "timestamp": float(image.timestamp),
            "camera_transform": self.transform_to_dict(
                image.transform
            ),
            "labels": labels,
        }

        with self.lock:
            self.received_semantic_frames += 1
            self.semantic_by_frame[int(image.frame)] = packet
            pair = self.take_pair_locked(int(image.frame))
            self.prune_pending_locked()

        if pair is not None:
            self.handle_pair(pair)

    def take_pair_locked(
        self,
        frame: int,
    ) -> Optional[dict[str, Any]]:
        rgb = self.rgb_by_frame.get(frame)
        semantic = self.semantic_by_frame.get(frame)

        if rgb is None or semantic is None:
            return None

        self.rgb_by_frame.pop(frame, None)
        self.semantic_by_frame.pop(frame, None)

        return {
            "frame": frame,
            "rgb": rgb,
            "semantic": semantic,
        }

    def prune_pending_locked(self) -> None:
        maximum = max(
            1,
            int(self.p("max_pending_frames")),
        )

        for storage in (
            self.rgb_by_frame,
            self.semantic_by_frame,
        ):
            if len(storage) <= maximum:
                continue

            keys = sorted(storage)
            remove_count = len(storage) - maximum

            for key in keys[:remove_count]:
                storage.pop(key, None)

    def handle_pair(
        self,
        pair: dict[str, Any],
    ) -> None:
        self.synchronized_pairs += 1

        rgb_timestamp = float(
            pair["rgb"]["timestamp"]
        )
        semantic_timestamp = float(
            pair["semantic"]["timestamp"]
        )

        timestamp_difference = abs(
            rgb_timestamp - semantic_timestamp
        )

        if timestamp_difference > float(
            self.p("max_timestamp_difference_sec")
        ):
            self.timestamp_mismatches += 1

        pair_index = self.synchronized_pairs - 1

        if pair_index % self.sample_every_n != 0:
            return

        if (
            self.max_samples > 0
            and self.enqueued_samples >= self.max_samples
        ):
            return

        labels = pair["semantic"]["labels"]

        roadline_mask = (
            labels == self.roadline_label_id
        ).astype(np.uint8) * 255

        save_packet = {
            "frame": int(pair["frame"]),
            "rgb_timestamp": rgb_timestamp,
            "semantic_timestamp": semantic_timestamp,
            "timestamp_difference_sec": timestamp_difference,
            "camera_transform": pair["rgb"][
                "camera_transform"
            ],
            "bgr": pair["rgb"]["bgr"],
            "labels": labels,
            "roadline_mask": roadline_mask,
        }

        if self.publish_preview:
            self.publish_preview_images(save_packet)

        try:
            self.save_queue.put_nowait(save_packet)
            self.enqueued_samples += 1
        except queue.Full:
            self.dropped_samples += 1

    # ==============================================================
    # Asynchronous disk writer
    # ==============================================================

    def save_worker(self) -> None:
        while (
            not self.stop_event.is_set()
            or not self.save_queue.empty()
        ):
            try:
                packet = self.save_queue.get(
                    timeout=0.2
                )
            except queue.Empty:
                continue

            try:
                self.save_sample(packet)
            except Exception as exc:
                self.write_errors += 1
                self.get_logger().error(
                    f"Failed to save dataset sample: {exc}"
                )
            finally:
                self.save_queue.task_done()

    def save_sample(
        self,
        packet: dict[str, Any],
    ) -> None:
        frame = int(packet["frame"])
        basename = f"frame_{frame:08d}"

        rgb_path = self.rgb_dir / f"{basename}.jpg"
        semantic_path = (
            self.semantic_dir / f"{basename}.png"
        )
        roadline_path = (
            self.roadline_dir / f"{basename}.png"
        )

        rgb_ok = cv2.imwrite(
            str(rgb_path),
            packet["bgr"],
            [
                int(cv2.IMWRITE_JPEG_QUALITY),
                int(self.p("rgb_jpeg_quality")),
            ],
        )

        if not rgb_ok:
            raise RuntimeError(
                f"Could not write {rgb_path}"
            )

        if bool(
            self.p("save_full_semantic_labels")
        ):
            semantic_ok = cv2.imwrite(
                str(semantic_path),
                packet["labels"],
            )

            if not semantic_ok:
                raise RuntimeError(
                    f"Could not write {semantic_path}"
                )

        roadline_ok = cv2.imwrite(
            str(roadline_path),
            packet["roadline_mask"],
        )

        if not roadline_ok:
            raise RuntimeError(
                f"Could not write {roadline_path}"
            )

        mask = packet["roadline_mask"]
        roadline_pixels = int(
            np.count_nonzero(mask)
        )

        sample_index = self.saved_samples

        record = {
            "sample_index": sample_index,
            "carla_frame": frame,
            "rgb_timestamp": float(
                packet["rgb_timestamp"]
            ),
            "semantic_timestamp": float(
                packet["semantic_timestamp"]
            ),
            "timestamp_difference_sec": float(
                packet["timestamp_difference_sec"]
            ),
            "map_name": self.map_name,
            "image_width": self.width,
            "image_height": self.height,
            "fov_deg": self.fov,
            "camera_intrinsics": self.camera_intrinsics(),
            "camera_world_transform": packet[
                "camera_transform"
            ],
            "roadline_label_id": self.roadline_label_id,
            "roadline_pixel_count": roadline_pixels,
            "roadline_pixel_fraction": float(
                roadline_pixels / mask.size
            ),
            "rgb_path": str(
                rgb_path.relative_to(self.run_dir)
            ),
            "semantic_labels_path": (
                str(
                    semantic_path.relative_to(
                        self.run_dir
                    )
                )
                if bool(
                    self.p(
                        "save_full_semantic_labels"
                    )
                )
                else None
            ),
            "roadline_mask_path": str(
                roadline_path.relative_to(
                    self.run_dir
                )
            ),
        }

        with self.metadata_path.open(
            "a",
            encoding="utf-8",
        ) as metadata_file:
            metadata_file.write(
                json.dumps(
                    record,
                    separators=(",", ":"),
                )
                + "\n"
            )

        self.saved_samples += 1
        self.log_collection_progress()

    # ==============================================================
    # Collection progress
    # ==============================================================

    @staticmethod
    def format_duration(seconds: float) -> str:
        seconds = max(0, int(round(seconds)))
        hours, remainder = divmod(seconds, 3600)
        minutes, seconds = divmod(remainder, 60)

        if hours > 0:
            return f"{hours:d}h {minutes:02d}m {seconds:02d}s"

        if minutes > 0:
            return f"{minutes:d}m {seconds:02d}s"

        return f"{seconds:d}s"

    def log_collection_progress(self, force: bool = False) -> None:
        saved = int(self.saved_samples)
        maximum = int(self.max_samples)
        report_every = max(
            1,
            int(self.p("progress_every_samples")),
        )

        complete = maximum > 0 and saved >= maximum

        if (
            not force
            and not complete
            and saved - self.last_progress_reported_sample
            < report_every
        ):
            return

        elapsed = max(
            1e-6,
            time.monotonic()
            - self.collection_started_monotonic,
        )
        rate = saved / elapsed

        fields = []

        if maximum > 0:
            percentage = 100.0 * saved / maximum
            remaining = max(0, maximum - saved)
            eta_seconds = (
                remaining / rate
                if rate > 1e-6
                else 0.0
            )

            fields.append(
                f"{saved}/{maximum} samples "
                f"({percentage:.1f}%)"
            )
            fields.append(
                f"ETA {self.format_duration(eta_seconds)}"
            )
        else:
            fields.append(f"{saved} samples")

        fields.extend(
            [
                f"{rate:.2f} samples/s",
                f"elapsed {self.format_duration(elapsed)}",
                f"queue={self.save_queue.qsize()}",
                f"dropped={self.dropped_samples}",
                f"errors={self.write_errors}",
            ]
        )

        prefix = (
            "Collection complete"
            if complete
            else "Collection progress"
        )

        self.get_logger().info(
            prefix + ": " + " | ".join(fields)
        )

        self.last_progress_reported_sample = saved

    # ==============================================================
    # ROS preview and status
    # ==============================================================

    def publish_preview_images(
        self,
        packet: dict[str, Any],
    ) -> None:
        stamp = self.get_clock().now().to_msg()

        rgb_ok, rgb_encoded = cv2.imencode(
            ".jpg",
            packet["bgr"],
            [
                int(cv2.IMWRITE_JPEG_QUALITY),
                85,
            ],
        )

        if rgb_ok:
            rgb_msg = CompressedImage()
            rgb_msg.header.stamp = stamp
            rgb_msg.header.frame_id = (
                "carla_lane_dataset_camera"
            )
            rgb_msg.format = "jpeg"
            rgb_msg.data = rgb_encoded.tobytes()
            self.preview_rgb_pub.publish(rgb_msg)

        mask_ok, mask_encoded = cv2.imencode(
            ".png",
            packet["roadline_mask"],
        )

        if mask_ok:
            mask_msg = CompressedImage()
            mask_msg.header.stamp = stamp
            mask_msg.header.frame_id = (
                "carla_lane_dataset_camera"
            )
            mask_msg.format = "png"
            mask_msg.data = mask_encoded.tobytes()
            self.preview_mask_pub.publish(mask_msg)

    def publish_status(self) -> None:
        with self.lock:
            pending_rgb = len(self.rgb_by_frame)
            pending_semantic = len(
                self.semantic_by_frame
            )

        complete = bool(
            self.max_samples > 0
            and self.saved_samples >= self.max_samples
        )

        status = {
            "version": "lane_dataset_recorder_v1",
            "map_name": self.map_name,
            "output_directory": str(self.run_dir),
            "received_rgb_frames": int(
                self.received_rgb_frames
            ),
            "received_semantic_frames": int(
                self.received_semantic_frames
            ),
            "synchronized_pairs": int(
                self.synchronized_pairs
            ),
            "enqueued_samples": int(
                self.enqueued_samples
            ),
            "saved_samples": int(
                self.saved_samples
            ),
            "queued_samples": int(
                self.save_queue.qsize()
            ),
            "dropped_samples": int(
                self.dropped_samples
            ),
            "timestamp_mismatches": int(
                self.timestamp_mismatches
            ),
            "write_errors": int(
                self.write_errors
            ),
            "pending_rgb_frames": int(
                pending_rgb
            ),
            "pending_semantic_frames": int(
                pending_semantic
            ),
            "sample_every_n": int(
                self.sample_every_n
            ),
            "max_samples": int(
                self.max_samples
            ),
            "complete": complete,
        }

        self.status_pub.publish(
            String(
                data=json.dumps(
                    status,
                    separators=(",", ":"),
                )
            )
        )

    # ==============================================================
    # Shutdown
    # ==============================================================

    def destroy_sensors(self) -> None:
        for sensor_name in (
            "rgb_sensor",
            "semantic_sensor",
        ):
            sensor = getattr(
                self,
                sensor_name,
                None,
            )

            if sensor is None:
                continue

            try:
                sensor.stop()
            except Exception:
                pass

            try:
                sensor.destroy()
            except Exception:
                pass

            setattr(self, sensor_name, None)

    def destroy_node(self) -> None:
        self.destroy_sensors()

        self.stop_event.set()

        if hasattr(self, "save_thread"):
            self.save_thread.join(timeout=5.0)

        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[LaneDatasetRecorder] = None

    try:
        node = LaneDatasetRecorder()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
