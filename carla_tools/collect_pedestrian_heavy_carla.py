#!/usr/bin/env python3
import os
import sys
import time
import json
import queue
import argparse
from pathlib import Path

import cv2
import numpy as np

try:
    sys.path.append(
        next(
            p for p in [
                os.path.join(
                    os.environ.get("CARLA_ROOT", ""),
                    "PythonAPI",
                    "carla",
                    "dist"
                )
            ] if p
        )
    )
except StopIteration:
    pass

import carla


# =========================
# USER CONFIG
# =========================
HOST = "127.0.0.1"
PORT = 2000
TIMEOUT = 10.0

OUTPUT_ROOT = Path("vru_recollection")

IMAGE_W = 1280
IMAGE_H = 720
FOV = 90

SAVE_EVERY_N_FRAMES_MIN = 8
MIN_TOTAL_TARGET_PIXELS = 800
MIN_PIXELS_PER_CLASS = {
    "pedestrian": 250,
    "rider": 150,
    "motorcycle": 120,
    "bicycle": 120,
}

MIN_MASK_CHANGE_RATIO = 0.01
MAX_SAVED = None

# Common CARLA semantic IDs
TARGET_CLASSES = {
    "pedestrian": 12,
    "rider": 13,
    "motorcycle": 18,
    "bicycle": 19,
}

SAVE_COLOR_PREVIEW = True

WEATHER_PRESETS = {
    "clear_noon": carla.WeatherParameters.ClearNoon,
    "clear_sunset": carla.WeatherParameters.ClearSunset,
    "cloudy_noon": carla.WeatherParameters.CloudyNoon,
    "cloudy_sunset": carla.WeatherParameters.CloudySunset,
    "wet_noon": carla.WeatherParameters.WetNoon,
    "wet_sunset": carla.WeatherParameters.WetSunset,
    "mid_rainy_noon": carla.WeatherParameters.MidRainyNoon,
    "mid_rain_sunset": carla.WeatherParameters.MidRainSunset,
    "wet_cloudy_noon": carla.WeatherParameters.WetCloudyNoon,
    "wet_cloudy_sunset": carla.WeatherParameters.WetCloudySunset,
    "hard_rain_noon": carla.WeatherParameters.HardRainNoon,
    "hard_rain_sunset": carla.WeatherParameters.HardRainSunset,
    "soft_rain_noon": carla.WeatherParameters.SoftRainNoon,
    "soft_rain_sunset": carla.WeatherParameters.SoftRainSunset,
}


# =========================
# ARGUMENTS
# =========================
def parse_args():
    parser = argparse.ArgumentParser(
        description="Collect pedestrian/rider/bike/motorbike-heavy CARLA data."
    )
    parser.add_argument(
        "--host",
        type=str,
        default=HOST,
        help=f"CARLA host (default: {HOST})",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=PORT,
        help=f"CARLA port (default: {PORT})",
    )
    parser.add_argument(
        "--weather",
        type=str,
        default=None,
        help=(
            "Weather preset name. Example: clear_noon, cloudy_noon, "
            "wet_cloudy_noon, hard_rain_noon, soft_rain_sunset"
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=str(OUTPUT_ROOT),
        help=f"Output root directory (default: {OUTPUT_ROOT})",
    )
    parser.add_argument(
        "--max-saved",
        type=int,
        default=MAX_SAVED,
        help="Optional maximum number of saved samples.",
    )
    return parser.parse_args()


# =========================
# HELPERS
# =========================
def make_output_dirs(output_root: Path):
    rgb_dir = output_root / "rgb"
    sem_dir = output_root / "semantic_raw"
    meta_dir = output_root / "meta"
    sem_color_dir = output_root / "semantic_color"

    rgb_dir.mkdir(parents=True, exist_ok=True)
    sem_dir.mkdir(parents=True, exist_ok=True)
    meta_dir.mkdir(parents=True, exist_ok=True)
    if SAVE_COLOR_PREVIEW:
        sem_color_dir.mkdir(parents=True, exist_ok=True)

    return rgb_dir, sem_dir, meta_dir, sem_color_dir


def print_available_weathers():
    print("Available weather presets:")
    for key in WEATHER_PRESETS:
        print(f"  - {key}")


def image_to_bgra_array(image: carla.Image) -> np.ndarray:
    arr = np.frombuffer(image.raw_data, dtype=np.uint8)
    arr = arr.reshape((image.height, image.width, 4))
    return arr


def semantic_label_map_from_image(image: carla.Image) -> np.ndarray:
    # CARLA semantic label is stored in the R channel of the BGRA image
    bgra = image_to_bgra_array(image)
    labels = bgra[:, :, 2]
    return labels


def rgb_array_from_image(image: carla.Image) -> np.ndarray:
    bgra = image_to_bgra_array(image)
    bgr = bgra[:, :, :3]
    rgb = bgr[:, :, ::-1]
    return rgb


def save_semantic_color_preview(image: carla.Image, out_path: Path):
    image.convert(carla.ColorConverter.CityScapesPalette)
    arr = image_to_bgra_array(image)[:, :, :3]
    cv2.imwrite(str(out_path), arr)


def compute_class_pixel_counts(labels: np.ndarray, class_map: dict) -> dict:
    counts = {}
    for name, class_id in class_map.items():
        counts[name] = int(np.sum(labels == class_id))
    return counts


def build_target_binary_mask(labels: np.ndarray, class_ids) -> np.ndarray:
    return np.isin(labels, list(class_ids)).astype(np.uint8)


def mask_change_ratio(mask_a: np.ndarray, mask_b: np.ndarray) -> float:
    if mask_a is None or mask_b is None:
        return 1.0
    diff = np.count_nonzero(mask_a != mask_b)
    return diff / mask_a.size


def frame_is_informative(class_counts: dict):
    total = sum(class_counts.values())
    per_class_ok = {
        name: class_counts[name] >= MIN_PIXELS_PER_CLASS.get(name, 0)
        for name in class_counts
    }
    at_least_one_class_ok = any(per_class_ok.values())
    total_ok = total >= MIN_TOTAL_TARGET_PIXELS
    accepted = total_ok and at_least_one_class_ok

    diagnostics = {
        "total_target_pixels": total,
        "per_class_ok": per_class_ok,
        "total_ok": total_ok,
        "at_least_one_class_ok": at_least_one_class_ok,
    }
    return accepted, diagnostics


def save_sample(
    base_name: str,
    rgb_np: np.ndarray,
    sem_labels: np.ndarray,
    sem_image: carla.Image,
    class_counts: dict,
    diagnostics: dict,
    world_frame: int,
    timestamp: float,
    rgb_dir: Path,
    sem_dir: Path,
    meta_dir: Path,
    sem_color_dir: Path | None,
):
    rgb_path = rgb_dir / f"{base_name}.png"
    sem_raw_path = sem_dir / f"{base_name}.png"
    meta_path = meta_dir / f"{base_name}.json"

    cv2.imwrite(str(rgb_path), cv2.cvtColor(rgb_np, cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(sem_raw_path), sem_labels)

    if SAVE_COLOR_PREVIEW and sem_color_dir is not None:
        color_path = sem_color_dir / f"{base_name}.png"
        # Copy behavior by recreating from raw bytes would be ideal, but this is fine here
        save_semantic_color_preview(sem_image, color_path)

    meta = {
        "base_name": base_name,
        "world_frame": world_frame,
        "timestamp": timestamp,
        "class_counts": class_counts,
        "diagnostics": diagnostics,
        "image_size": {
            "width": int(rgb_np.shape[1]),
            "height": int(rgb_np.shape[0]),
        },
    }

    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)


def spawn_camera(world, bp_lib, vehicle, blueprint_id, transform, image_w, image_h, fov):
    cam_bp = bp_lib.find(blueprint_id)
    cam_bp.set_attribute("image_size_x", str(image_w))
    cam_bp.set_attribute("image_size_y", str(image_h))
    cam_bp.set_attribute("fov", str(fov))
    return world.spawn_actor(cam_bp, transform, attach_to=vehicle)


def set_synchronous_mode(world, traffic_manager=None, fixed_delta=0.05, enabled=True):
    settings = world.get_settings()
    settings.synchronous_mode = enabled
    settings.fixed_delta_seconds = fixed_delta if enabled else None
    world.apply_settings(settings)

    if traffic_manager is not None:
        traffic_manager.set_synchronous_mode(enabled)


# =========================
# MAIN
# =========================
def main():
    args = parse_args()

    output_root = Path(args.output_dir)
    rgb_dir, sem_dir, meta_dir, sem_color_dir = make_output_dirs(output_root)

    client = carla.Client(args.host, args.port)
    client.set_timeout(TIMEOUT)

    world = client.get_world()

    if args.weather is not None:
        weather_key = args.weather.lower()
        if weather_key not in WEATHER_PRESETS:
            print(f"[ERROR] Unknown weather preset: {args.weather}")
            print_available_weathers()
            return

        world.set_weather(WEATHER_PRESETS[weather_key])
        print(f"[INFO] Weather set to: {weather_key}")
    else:
        print("[INFO] No weather preset provided; keeping current world weather.")

    bp_lib = world.get_blueprint_library()
    traffic_manager = client.get_trafficmanager()
    original_settings = world.get_settings()

    actor_list = []
    saved_count = 0
    frame_count = 0
    last_saved_frame_idx = -10_000
    last_saved_target_mask = None

    rgb_queue = queue.Queue()
    sem_queue = queue.Queue()

    try:
        set_synchronous_mode(world, traffic_manager=traffic_manager, fixed_delta=0.05, enabled=True)

        spawn_points = world.get_map().get_spawn_points()
        if not spawn_points:
            raise RuntimeError("No spawn points found in current CARLA map.")

        vehicle_candidates = bp_lib.filter("vehicle.*")
        if not vehicle_candidates:
            raise RuntimeError("No vehicle blueprints found.")

        vehicle_bp = vehicle_candidates[0]
        vehicle = world.try_spawn_actor(vehicle_bp, spawn_points[0])
        if vehicle is None:
            raise RuntimeError("Could not spawn ego vehicle.")

        actor_list.append(vehicle)
        vehicle.set_autopilot(True, traffic_manager.get_port())

        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.2))

        rgb_cam = spawn_camera(
            world,
            bp_lib,
            vehicle,
            "sensor.camera.rgb",
            camera_transform,
            IMAGE_W,
            IMAGE_H,
            FOV,
        )
        sem_cam = spawn_camera(
            world,
            bp_lib,
            vehicle,
            "sensor.camera.semantic_segmentation",
            camera_transform,
            IMAGE_W,
            IMAGE_H,
            FOV,
        )

        actor_list.extend([rgb_cam, sem_cam])

        rgb_cam.listen(rgb_queue.put)
        sem_cam.listen(sem_queue.put)

        print("[INFO] Collecting targeted VRU frames...")
        print(f"[INFO] Saving to: {output_root.resolve()}")
        print(f"[INFO] Target classes: {TARGET_CLASSES}")

        while True:
            world.tick()
            frame_count += 1

            rgb_img = rgb_queue.get(timeout=2.0)
            sem_img = sem_queue.get(timeout=2.0)

            if rgb_img.frame != sem_img.frame:
                print(
                    f"[WARN] Frame mismatch RGB={rgb_img.frame}, "
                    f"SEM={sem_img.frame}, skipping."
                )
                continue

            rgb_np = rgb_array_from_image(rgb_img)
            sem_labels = semantic_label_map_from_image(sem_img)

            class_counts = compute_class_pixel_counts(sem_labels, TARGET_CLASSES)
            accepted_by_content, diagnostics = frame_is_informative(class_counts)

            if not accepted_by_content:
                continue

            current_target_mask = build_target_binary_mask(
                sem_labels,
                TARGET_CLASSES.values()
            )

            frames_since_last_save = frame_count - last_saved_frame_idx
            enough_cooldown = frames_since_last_save >= SAVE_EVERY_N_FRAMES_MIN

            changed_ratio = mask_change_ratio(current_target_mask, last_saved_target_mask)
            changed_enough = changed_ratio >= MIN_MASK_CHANGE_RATIO

            if not enough_cooldown:
                continue

            if not changed_enough:
                continue

            base_name = f"frame_{rgb_img.frame:08d}"
            save_sample(
                base_name=base_name,
                rgb_np=rgb_np,
                sem_labels=sem_labels,
                sem_image=sem_img,
                class_counts=class_counts,
                diagnostics={
                    **diagnostics,
                    "frames_since_last_save": frames_since_last_save,
                    "mask_change_ratio": changed_ratio,
                },
                world_frame=rgb_img.frame,
                timestamp=rgb_img.timestamp,
                rgb_dir=rgb_dir,
                sem_dir=sem_dir,
                meta_dir=meta_dir,
                sem_color_dir=sem_color_dir if SAVE_COLOR_PREVIEW else None,
            )

            saved_count += 1
            last_saved_frame_idx = frame_count
            last_saved_target_mask = current_target_mask.copy()

            print(
                f"[SAVED {saved_count}] {base_name} | "
                f"counts={class_counts} | change={changed_ratio:.4f}"
            )

            if args.max_saved is not None and saved_count >= args.max_saved:
                print("[INFO] Reached --max-saved. Stopping.")
                break

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user.")
    finally:
        print("[INFO] Cleaning up actors and restoring world settings...")
        for actor in actor_list:
            try:
                actor.destroy()
            except Exception:
                pass

        world.apply_settings(original_settings)
        try:
            traffic_manager.set_synchronous_mode(False)
        except Exception:
            pass


if __name__ == "__main__":
    main()
