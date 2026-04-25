#!/usr/bin/env python3
import argparse
import queue
import signal
from pathlib import Path

import cv2
import numpy as np
import carla

# CARLA 0.9.16 semantic IDs (your mapping convention)
PERSON_ID = 12
RIDER_ID = 13
CAR_ID = 14
TRUCK_ID = 15
BUS_ID = 16
TRAIN_ID = 17
MOTORCYCLE_ID = 18
BICYCLE_ID = 19

VRU_IDS = [PERSON_ID, RIDER_ID, MOTORCYCLE_ID, BICYCLE_ID]
VEHICLE_CONTEXT_IDS = [CAR_ID, TRUCK_ID, BUS_ID]


def current_town_name(world):
    return world.get_map().name.split("/")[-1]


def find_hero_vehicle(world):
    for actor in world.get_actors().filter("vehicle.*"):
        try:
            if actor.attributes.get("role_name", "") == "hero":
                return actor
        except Exception:
            pass
    return None


def spawn_hero_vehicle(world):
    bp_lib = world.get_blueprint_library()
    bps = bp_lib.filter("vehicle.*model3*")
    if not bps:
        bps = bp_lib.filter("vehicle.*")

    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        raise RuntimeError("No spawn points found in this CARLA map.")

    bp = bps[0]
    bp.set_attribute("role_name", "hero")
    vehicle = world.try_spawn_actor(bp, spawn_points[0])
    if vehicle is None:
        raise RuntimeError("Failed to spawn hero vehicle.")
    return vehicle


def respawn_hero_vehicle(world):
    bp_lib = world.get_blueprint_library()
    bps = bp_lib.filter("vehicle.*model3*")
    if not bps:
        bps = bp_lib.filter("vehicle.*")

    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        raise RuntimeError("No spawn points found in this CARLA map.")

    rng = np.random.default_rng()
    for idx in rng.permutation(len(spawn_points)):
        bp = bps[0]
        bp.set_attribute("role_name", "hero")
        vehicle = world.try_spawn_actor(bp, spawn_points[idx])
        if vehicle is not None:
            return vehicle

    raise RuntimeError("Failed to respawn hero vehicle.")


def carla_image_to_bgr(image: carla.Image) -> np.ndarray:
    arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))
    return arr[:, :, :3].copy()


def carla_semantic_to_ids(image: carla.Image) -> np.ndarray:
    image.convert(carla.ColorConverter.Raw)
    arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))
    return arr[:, :, 2].copy()


def build_weather(name: str):
    if hasattr(carla.WeatherParameters, name):
        return getattr(carla.WeatherParameters, name)
    raise ValueError(f"Unknown weather preset: {name}")


def count_pixels(mask: np.ndarray, class_id: int) -> int:
    return int(np.sum(mask == class_id))


def frame_is_interesting(mask: np.ndarray,
                         min_person: int,
                         min_rider: int,
                         min_bicycle: int,
                         min_motorcycle: int,
                         min_total_vru: int,
                         min_vehicle_context: int):
    person_px = count_pixels(mask, PERSON_ID)
    rider_px = count_pixels(mask, RIDER_ID)
    bicycle_px = count_pixels(mask, BICYCLE_ID)
    motorcycle_px = count_pixels(mask, MOTORCYCLE_ID)

    total_vru = person_px + rider_px + bicycle_px + motorcycle_px

    vehicle_context_px = 0
    for cid in VEHICLE_CONTEXT_IDS:
        vehicle_context_px += count_pixels(mask, cid)

    interesting = (
        person_px >= min_person or
        rider_px >= min_rider or
        bicycle_px >= min_bicycle or
        motorcycle_px >= min_motorcycle or
        total_vru >= min_total_vru
    )

    # Optional context boost: keep some scenes with VRUs plus nearby vehicles
    if not interesting and total_vru > 0 and vehicle_context_px >= min_vehicle_context:
        interesting = True

    stats = {
        "person_px": person_px,
        "rider_px": rider_px,
        "bicycle_px": bicycle_px,
        "motorcycle_px": motorcycle_px,
        "total_vru_px": total_vru,
        "vehicle_context_px": vehicle_context_px,
    }
    return interesting, stats


def attach_cameras(world, vehicle, width, height, fov):
    bp_lib = world.get_blueprint_library()

    rgb_bp = bp_lib.find("sensor.camera.rgb")
    rgb_bp.set_attribute("image_size_x", str(width))
    rgb_bp.set_attribute("image_size_y", str(height))
    rgb_bp.set_attribute("fov", str(fov))
    rgb_bp.set_attribute("sensor_tick", "0.0")

    sem_bp = bp_lib.find("sensor.camera.semantic_segmentation")
    sem_bp.set_attribute("image_size_x", str(width))
    sem_bp.set_attribute("image_size_y", str(height))
    sem_bp.set_attribute("fov", str(fov))
    sem_bp.set_attribute("sensor_tick", "0.0")

    cam_tf = carla.Transform(carla.Location(x=1.5, z=2.4))
    rgb_cam = world.spawn_actor(rgb_bp, cam_tf, attach_to=vehicle)
    sem_cam = world.spawn_actor(sem_bp, cam_tf, attach_to=vehicle)

    rgb_q = queue.Queue()
    sem_q = queue.Queue()

    rgb_cam.listen(lambda img: rgb_q.put(img))
    sem_cam.listen(lambda img: sem_q.put(img))

    return rgb_cam, sem_cam, rgb_q, sem_q


def main():
    ap = argparse.ArgumentParser(description="Collect pedestrian/rider/bike/motorbike-heavy CARLA frames from the current map.")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--tm-port", type=int, default=8000)
    ap.add_argument("--weather", default="ClearNoon")
    ap.add_argument("--out", default=str(Path.home() / "datasets" / "carla_vru_heavy"))
    ap.add_argument("--width", type=int, default=800)
    ap.add_argument("--height", type=int, default=600)
    ap.add_argument("--fov", type=float, default=90.0)
    ap.add_argument("--fps", type=int, default=15)
    ap.add_argument("--save-every", type=int, default=2)
    ap.add_argument("--max-frames", type=int, default=4000)

    # Per-class thresholds
    ap.add_argument("--min-person-pixels", type=int, default=80)
    ap.add_argument("--min-rider-pixels", type=int, default=60)
    ap.add_argument("--min-bicycle-pixels", type=int, default=60)
    ap.add_argument("--min-motorcycle-pixels", type=int, default=60)
    ap.add_argument("--min-total-vru-pixels", type=int, default=120)
    ap.add_argument("--min-vehicle-context-pixels", type=int, default=400)

    ap.add_argument("--keep-context-ratio", type=float, default=0.05,
                    help="Keep this fraction of non-interesting frames for context")
    ap.add_argument("--autopilot", action="store_true")
    ap.add_argument("--respawn-every-saved", type=int, default=250,
                    help="Respawn ego vehicle after this many saved frames")
    ap.add_argument("--sync", action="store_true", default=True)
    ap.add_argument("--no-sync", action="store_true", default=False)
    args = ap.parse_args()

    if args.no_sync:
        args.sync = False

    client = carla.Client(args.host, args.port)
    client.set_timeout(120.0)
    world = client.get_world()
    map_name = current_town_name(world)

    try:
        world.set_weather(build_weather(args.weather))
        print(f"[collector] Weather set to: {args.weather}")
    except Exception as e:
        print(f"[collector] WARNING: could not set weather '{args.weather}': {e}")

    out_root = Path(args.out) / map_name / args.weather
    rgb_dir = out_root / "rgb"
    sem_dir = out_root / "sem_raw"
    rgb_dir.mkdir(parents=True, exist_ok=True)
    sem_dir.mkdir(parents=True, exist_ok=True)

    orig_settings = world.get_settings()
    tm = client.get_trafficmanager(args.tm_port)
    tm_port = args.tm_port

    if args.sync:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1.0 / max(args.fps, 1)
        settings.no_rendering_mode = False
        world.apply_settings(settings)
        tm.set_synchronous_mode(True)

    hero = find_hero_vehicle(world)
    spawned_vehicle = False
    if hero is None:
        hero = spawn_hero_vehicle(world)
        spawned_vehicle = True
        print(f"[collector] Spawned hero vehicle id={hero.id}")
    else:
        print(f"[collector] Found existing hero vehicle id={hero.id}")

    if args.autopilot:
        hero.set_autopilot(True, tm_port)
        print("[collector] Autopilot enabled")

    rgb_cam, sem_cam, rgb_q, sem_q = attach_cameras(world, hero, args.width, args.height, args.fov)

    rng = np.random.default_rng()
    stop = {"flag": False}

    def on_sigint(sig, frame):
        stop["flag"] = True

    signal.signal(signal.SIGINT, on_sigint)

    print("[collector] VRU-heavy collection started")
    print(f"[collector] Map                : {map_name}")
    print(f"[collector] Weather            : {args.weather}")
    print(f"[collector] Output root        : {out_root}")
    print(f"[collector] Max saved frames   : {args.max_frames}")
    print(f"[collector] save_every         : {args.save_every}")
    print(f"[collector] min_person_px      : {args.min_person_pixels}")
    print(f"[collector] min_rider_px       : {args.min_rider_pixels}")
    print(f"[collector] min_bicycle_px     : {args.min_bicycle_pixels}")
    print(f"[collector] min_motorcycle_px  : {args.min_motorcycle_pixels}")
    print(f"[collector] min_total_vru_px   : {args.min_total_vru_pixels}")
    print(f"[collector] context_keep_ratio : {args.keep_context_ratio}")

    saved = 0
    saved_interesting = 0
    saved_context = 0
    frame_idx = 0
    continous_frames_discarded = 0
    max_continous_frames_discarded = args.max_frames/4

    try:
        while not stop["flag"] and saved < args.max_frames:
            if args.sync:
                world.tick()
            else:
                world.wait_for_tick()

            try:
                rgb_img = rgb_q.get(timeout=2.0)
                sem_img = sem_q.get(timeout=2.0)
            except queue.Empty:
                print("[collector] WARNING: sensor timeout")
                continue

            if rgb_img.frame != sem_img.frame:
                target = max(rgb_img.frame, sem_img.frame)
                while rgb_img.frame < target:
                    try:
                        rgb_img = rgb_q.get(timeout=0.5)
                    except queue.Empty:
                        break
                while sem_img.frame < target:
                    try:
                        sem_img = sem_q.get(timeout=0.5)
                    except queue.Empty:
                        break

            if frame_idx % args.save_every != 0:
                frame_idx += 1
                continue

            bgr = carla_image_to_bgr(rgb_img)
            ids = carla_semantic_to_ids(sem_img)

            interesting, stats = frame_is_interesting(
                ids,
                min_person=args.min_person_pixels,
                min_rider=args.min_rider_pixels,
                min_bicycle=args.min_bicycle_pixels,
                min_motorcycle=args.min_motorcycle_pixels,
                min_total_vru=args.min_total_vru_pixels,
                min_vehicle_context=args.min_vehicle_context_pixels,
            )

            keep = interesting or (saved_context < args.keep_context_ratio*args.max_frames*2 and rng.random() < args.keep_context_ratio)

            if keep:
                continous_frames_discarded = 0
                name = f"{saved:06d}.png"
                cv2.imwrite(str(rgb_dir / name), bgr)
                cv2.imwrite(str(sem_dir / name), ids)

                saved += 1
                if interesting:
                    saved_interesting += 1
                else:
                    saved_context += 1

                if saved % 50 == 0:
                    print(
                        f"[collector] saved={saved} "
                        f"(interesting={saved_interesting}, context={saved_context}) "
                        f"last stats={stats}"
                    )

                if args.autopilot and args.respawn_every_saved > 0 and saved % args.respawn_every_saved == 0:
                    print("[collector] Respawning hero for diversity...")

                    try:
                        rgb_cam.stop()
                        sem_cam.stop()
                    except Exception:
                        pass
                    try:
                        rgb_cam.destroy()
                        sem_cam.destroy()
                    except Exception:
                        pass

                    try:
                        hero.set_autopilot(False, tm_port)
                    except Exception:
                        pass
                    try:
                        hero.destroy()
                    except Exception:
                        pass

                    hero = respawn_hero_vehicle(world)
                    if args.autopilot:
                        hero.set_autopilot(True, tm_port)

                    rgb_cam, sem_cam, rgb_q, sem_q = attach_cameras(
                        world, hero, args.width, args.height, args.fov
                    )

                    for _ in range(10):
                        if args.sync:
                            world.tick()
                        else:
                            world.wait_for_tick()
            else:
                continous_frames_discarded += 1
                if args.autopilot and continous_frames_discarded > max_continous_frames_discarded:
                    continous_frames_discarded = 0
                    print("[collector] Respawning hero after timeout...")

                    try:
                        rgb_cam.stop()
                        sem_cam.stop()
                    except Exception:
                        pass
                    try:
                        rgb_cam.destroy()
                        sem_cam.destroy()
                    except Exception:
                        pass

                    try:
                        hero.set_autopilot(False, tm_port)
                    except Exception:
                        pass
                    try:
                        hero.destroy()
                    except Exception:
                        pass

                    hero = respawn_hero_vehicle(world)
                    if args.autopilot:
                        hero.set_autopilot(True, tm_port)

                    rgb_cam, sem_cam, rgb_q, sem_q = attach_cameras(
                        world, hero, args.width, args.height, args.fov
                    )

                    for _ in range(10):
                        if args.sync:
                            world.tick()
                        else:
                            world.wait_for_tick()
            frame_idx += 1
    finally:
        print("[collector] Cleaning up...")
        try:
            rgb_cam.stop()
            sem_cam.stop()
        except Exception:
            pass
        try:
            rgb_cam.destroy()
            sem_cam.destroy()
        except Exception:
            pass

        if args.autopilot:
            try:
                hero.set_autopilot(False, tm_port)
            except Exception:
                pass

        try:
            hero.destroy()
        except Exception:
            pass

        if args.sync:
            try:
                world.apply_settings(orig_settings)
                tm.set_synchronous_mode(False)
            except Exception:
                pass

        print(f"[collector] Done. saved={saved}, interesting={saved_interesting}, context={saved_context}")


if __name__ == "__main__":
    main()
