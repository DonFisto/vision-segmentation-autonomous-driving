#!/usr/bin/env python3
import argparse
import queue
import signal
import time
from pathlib import Path

import cv2
import numpy as np
import carla


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
        raise RuntimeError("Failed to spawn hero vehicle (spawn point occupied).")
    return vehicle


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


def current_town_name(world):
    return world.get_map().name.split("/")[-1]


def main():
    ap = argparse.ArgumentParser(description="Collect CARLA RGB + semantic frames.")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)

    ap.add_argument("--town", default="", help="CARLA town name, e.g. Town01, Town05, Town10HD_Opt")
    ap.add_argument("--weather", default="ClearNoon", help="CARLA weather preset name")
    ap.add_argument("--out", default=str(Path.home() / "datasets" / "carla_raw"))

    ap.add_argument("--width", type=int, default=800)
    ap.add_argument("--height", type=int, default=600)
    ap.add_argument("--fov", type=float, default=90.0)

    ap.add_argument("--fps", type=int, default=15)
    ap.add_argument("--save-every", type=int, default=2)
    ap.add_argument("--max-frames", type=int, default=5000)

    ap.add_argument("--sync", action="store_true", default=True)
    ap.add_argument("--no-sync", action="store_true", default=False)
    ap.add_argument("--autopilot", action="store_true")

    args = ap.parse_args()
    if args.no_sync:
        args.sync = False

    client = carla.Client(args.host, args.port)
    client.set_timeout(120.0)

    world = client.get_world()
    current_town = current_town_name(world)

    if args.town and args.town != current_town:
        print(f"[collector] Loading town: {args.town} (current: {current_town})")
        world = client.load_world(args.town)
        time.sleep(3.0)
    else:
        print(f"[collector] Reusing current town: {current_town}")

    map_name = current_town_name(world)

    try:
        weather = build_weather(args.weather)
        world.set_weather(weather)
        print(f"[collector] Weather set to: {args.weather}")
    except Exception as e:
        print(f"[collector] WARNING: could not set weather '{args.weather}': {e}")

    out_root = Path(args.out) / map_name / args.weather
    rgb_dir = out_root / "rgb"
    sem_dir = out_root / "sem_raw"
    rgb_dir.mkdir(parents=True, exist_ok=True)
    sem_dir.mkdir(parents=True, exist_ok=True)

    orig_settings = world.get_settings()
    tm = client.get_trafficmanager()

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
        hero.set_autopilot(True, tm.get_port())
        print("[collector] Autopilot enabled")

    bp_lib = world.get_blueprint_library()

    rgb_bp = bp_lib.find("sensor.camera.rgb")
    rgb_bp.set_attribute("image_size_x", str(args.width))
    rgb_bp.set_attribute("image_size_y", str(args.height))
    rgb_bp.set_attribute("fov", str(args.fov))
    rgb_bp.set_attribute("sensor_tick", "0.0")

    sem_bp = bp_lib.find("sensor.camera.semantic_segmentation")
    sem_bp.set_attribute("image_size_x", str(args.width))
    sem_bp.set_attribute("image_size_y", str(args.height))
    sem_bp.set_attribute("fov", str(args.fov))
    sem_bp.set_attribute("sensor_tick", "0.0")

    cam_tf = carla.Transform(carla.Location(x=1.5, z=2.4))
    rgb_cam = world.spawn_actor(rgb_bp, cam_tf, attach_to=hero)
    sem_cam = world.spawn_actor(sem_bp, cam_tf, attach_to=hero)

    rgb_q = queue.Queue()
    sem_q = queue.Queue()

    rgb_cam.listen(lambda img: rgb_q.put(img))
    sem_cam.listen(lambda img: sem_q.put(img))

    stop = {"flag": False}

    def on_sigint(sig, frame):
        stop["flag"] = True

    signal.signal(signal.SIGINT, on_sigint)

    print("[collector] Recording started")
    print(f"[collector] Output root : {out_root}")
    print(f"[collector] Town        : {map_name}")
    print(f"[collector] Weather     : {args.weather}")
    print(f"[collector] Max frames  : {args.max_frames}")
    print(f"[collector] Save every  : {args.save_every}")
    print(f"[collector] FPS         : {args.fps}")
    print(f"[collector] Sync        : {args.sync}")
    print(f"[collector] Autopilot   : {args.autopilot}")

    saved = 0
    frame_idx = 0

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

            if frame_idx % args.save_every == 0:
                bgr = carla_image_to_bgr(rgb_img)
                ids = carla_semantic_to_ids(sem_img)

                name = f"{saved:06d}.png"
                cv2.imwrite(str(rgb_dir / name), bgr)
                cv2.imwrite(str(sem_dir / name), ids)

                saved += 1
                if saved % 100 == 0:
                    print(f"[collector] saved {saved} frames")

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
                hero.set_autopilot(False, tm.get_port())
            except Exception:
                pass

        if spawned_vehicle:
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

        print("[collector] Done.")


if __name__ == "__main__":
    main()
