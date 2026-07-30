#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import queue
import random
import signal
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

import carla
import cv2
import numpy as np

ROADLINE_ID = 24


def current_town_name(world: carla.World) -> str:
    return world.get_map().name.split("/")[-1]


def normalize_map_name(name: str) -> str:
    value = name.strip()
    if value.lower() in {"", "current", "keep"}:
        return "current"
    return value.split("/")[-1]


def weather_presets() -> dict[str, carla.WeatherParameters]:
    presets: dict[str, carla.WeatherParameters] = {}
    for name in dir(carla.WeatherParameters):
        if name.startswith("_"):
            continue
        value = getattr(carla.WeatherParameters, name)
        if (
            hasattr(value, "cloudiness")
            and hasattr(value, "precipitation")
            and hasattr(value, "sun_altitude_angle")
        ):
            presets[name] = value
    return dict(sorted(presets.items()))


def parse_weather_list(value: str) -> list[str]:
    names = [item.strip() for item in value.split(",") if item.strip()]
    if not names:
        raise ValueError("At least one weather preset is required.")

    presets = weather_presets()
    unknown = [name for name in names if name not in presets]
    if unknown:
        raise ValueError(
            "Unknown weather preset(s): "
            + ", ".join(unknown)
            + ". Available: "
            + ", ".join(presets)
        )
    return names


def weather_to_dict(weather: carla.WeatherParameters) -> dict[str, float]:
    fields = (
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
    )
    return {
        field: float(getattr(weather, field))
        for field in fields
        if hasattr(weather, field)
    }


def transform_to_dict(transform: carla.Transform) -> dict[str, Any]:
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


def speed_mps(actor: carla.Actor) -> float:
    velocity = actor.get_velocity()
    return math.sqrt(
        velocity.x * velocity.x
        + velocity.y * velocity.y
        + velocity.z * velocity.z
    )


def carla_image_to_bgr(image: carla.Image) -> np.ndarray:
    array = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(
        (image.height, image.width, 4)
    )
    return array[:, :, :3].copy()


def carla_semantic_to_ids(image: carla.Image) -> np.ndarray:
    image.convert(carla.ColorConverter.Raw)
    array = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(
        (image.height, image.width, 4)
    )
    return array[:, :, 2].copy()


def destroy_actor(actor: Optional[carla.Actor]) -> None:
    if actor is None:
        return
    try:
        if isinstance(actor, carla.Sensor):
            actor.stop()
    except Exception:
        pass
    try:
        actor.destroy()
    except Exception:
        pass


def drain_queue(sensor_queue: queue.Queue) -> None:
    while True:
        try:
            sensor_queue.get_nowait()
        except queue.Empty:
            return


def get_at_least_frame(
    sensor_queue: queue.Queue,
    target_frame: int,
    timeout: float,
) -> carla.Image:
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise queue.Empty
        image = sensor_queue.get(timeout=remaining)
        if image.frame >= target_frame:
            return image


def get_aligned_pair(
    rgb_queue: queue.Queue,
    semantic_queue: queue.Queue,
    target_frame: int,
    timeout: float,
) -> tuple[carla.Image, carla.Image]:
    rgb = get_at_least_frame(rgb_queue, target_frame, timeout)
    semantic = get_at_least_frame(semantic_queue, target_frame, timeout)

    deadline = time.monotonic() + timeout
    while rgb.frame != semantic.frame:
        target = max(rgb.frame, semantic.frame)
        remaining = max(0.1, deadline - time.monotonic())
        if rgb.frame < target:
            rgb = get_at_least_frame(rgb_queue, target, remaining)
        if semantic.frame < target:
            semantic = get_at_least_frame(semantic_queue, target, remaining)
        if time.monotonic() >= deadline and rgb.frame != semantic.frame:
            raise queue.Empty

    return rgb, semantic


def choose_vehicle_blueprint(
    world: carla.World,
    pattern: str,
) -> carla.ActorBlueprint:
    blueprints = list(world.get_blueprint_library().filter(pattern))
    if not blueprints:
        blueprints = list(world.get_blueprint_library().filter("vehicle.*"))
    if not blueprints:
        raise RuntimeError("No vehicle blueprints are available.")
    blueprint = blueprints[0]
    if blueprint.has_attribute("role_name"):
        blueprint.set_attribute("role_name", "hero")
    return blueprint


def spawn_hero_vehicle(
    world: carla.World,
    blueprint_pattern: str,
    rng: np.random.Generator,
) -> carla.Vehicle:
    blueprint = choose_vehicle_blueprint(world, blueprint_pattern)
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        raise RuntimeError("No spawn points found in this CARLA map.")

    for index in rng.permutation(len(spawn_points)):
        vehicle = world.try_spawn_actor(blueprint, spawn_points[int(index)])
        if vehicle is not None:
            return vehicle

    raise RuntimeError("Failed to spawn the collector hero vehicle.")


def attach_cameras(
    world: carla.World,
    vehicle: carla.Vehicle,
    width: int,
    height: int,
    fov: float,
) -> tuple[carla.Sensor, carla.Sensor, queue.Queue, queue.Queue]:
    blueprint_library = world.get_blueprint_library()

    rgb_blueprint = blueprint_library.find("sensor.camera.rgb")
    rgb_blueprint.set_attribute("image_size_x", str(width))
    rgb_blueprint.set_attribute("image_size_y", str(height))
    rgb_blueprint.set_attribute("fov", str(fov))
    rgb_blueprint.set_attribute("sensor_tick", "0.0")

    semantic_blueprint = blueprint_library.find(
        "sensor.camera.semantic_segmentation"
    )
    semantic_blueprint.set_attribute("image_size_x", str(width))
    semantic_blueprint.set_attribute("image_size_y", str(height))
    semantic_blueprint.set_attribute("fov", str(fov))
    semantic_blueprint.set_attribute("sensor_tick", "0.0")

    camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
    rgb_camera = world.spawn_actor(
        rgb_blueprint,
        camera_transform,
        attach_to=vehicle,
        attachment_type=carla.AttachmentType.Rigid,
    )
    semantic_camera = world.spawn_actor(
        semantic_blueprint,
        camera_transform,
        attach_to=vehicle,
        attachment_type=carla.AttachmentType.Rigid,
    )

    rgb_queue: queue.Queue = queue.Queue()
    semantic_queue: queue.Queue = queue.Queue()
    rgb_camera.listen(rgb_queue.put)
    semantic_camera.listen(semantic_queue.put)
    return rgb_camera, semantic_camera, rgb_queue, semantic_queue


def filter_traffic_blueprints(
    world: carla.World,
    safe: bool,
) -> list[carla.ActorBlueprint]:
    blueprints = list(world.get_blueprint_library().filter("vehicle.*"))
    if not safe:
        return blueprints

    excluded = (
        "vehicle.carlamotors.carlacola",
        "vehicle.carlamotors.firetruck",
        "vehicle.tesla.cybertruck",
        "vehicle.mitsubishi.fusorosa",
    )
    filtered: list[carla.ActorBlueprint] = []
    for blueprint in blueprints:
        if blueprint.id in excluded:
            continue
        if blueprint.has_attribute("number_of_wheels"):
            wheels = int(blueprint.get_attribute("number_of_wheels"))
            if wheels != 4:
                continue
        filtered.append(blueprint)
    return filtered or blueprints


def spawn_traffic_vehicles(
    client: carla.Client,
    world: carla.World,
    tm_port: int,
    count: int,
    seed: int,
    safe: bool,
) -> list[int]:
    if count <= 0:
        return []

    rng = random.Random(seed)
    blueprints = filter_traffic_blueprints(world, safe)
    spawn_points = list(world.get_map().get_spawn_points())
    rng.shuffle(spawn_points)
    count = min(count, len(spawn_points))

    batch = []
    for transform in spawn_points[:count]:
        blueprint = rng.choice(blueprints)
        if blueprint.has_attribute("color"):
            blueprint.set_attribute(
                "color",
                rng.choice(blueprint.get_attribute("color").recommended_values),
            )
        if blueprint.has_attribute("driver_id"):
            blueprint.set_attribute(
                "driver_id",
                rng.choice(
                    blueprint.get_attribute("driver_id").recommended_values
                ),
            )
        if blueprint.has_attribute("role_name"):
            blueprint.set_attribute("role_name", "autopilot")

        batch.append(
            carla.command.SpawnActor(blueprint, transform).then(
                carla.command.SetAutopilot(
                    carla.command.FutureActor,
                    True,
                    tm_port,
                )
            )
        )

    actor_ids: list[int] = []
    for response in client.apply_batch_sync(batch, True):
        if response.error:
            print(f"[collector] Traffic spawn warning: {response.error}")
        else:
            actor_ids.append(response.actor_id)

    print(f"[collector] Spawned {len(actor_ids)} traffic vehicles")
    return actor_ids


def spawn_walkers(
    client: carla.Client,
    world: carla.World,
    count: int,
    seed: int,
) -> tuple[list[int], list[int]]:
    if count <= 0:
        return [], []

    rng = random.Random(seed)
    walker_blueprints = list(
        world.get_blueprint_library().filter("walker.pedestrian.*")
    )
    spawn_points: list[carla.Transform] = []
    for _ in range(count):
        location = world.get_random_location_from_navigation()
        if location is not None:
            spawn_points.append(carla.Transform(location))

    walker_batch = []
    walker_speeds: list[float] = []
    for transform in spawn_points:
        blueprint = rng.choice(walker_blueprints)
        if blueprint.has_attribute("is_invincible"):
            blueprint.set_attribute("is_invincible", "false")

        speed = 1.4
        if blueprint.has_attribute("speed"):
            values = blueprint.get_attribute("speed").recommended_values
            if len(values) > 1:
                speed = float(values[1])

        walker_speeds.append(speed)
        walker_batch.append(carla.command.SpawnActor(blueprint, transform))

    walker_ids: list[int] = []
    valid_speeds: list[float] = []
    for response, speed in zip(
        client.apply_batch_sync(walker_batch, True), walker_speeds
    ):
        if response.error:
            print(f"[collector] Walker spawn warning: {response.error}")
        else:
            walker_ids.append(response.actor_id)
            valid_speeds.append(speed)

    controller_blueprint = world.get_blueprint_library().find(
        "controller.ai.walker"
    )
    controller_batch = [
        carla.command.SpawnActor(
            controller_blueprint,
            carla.Transform(),
            walker_id,
        )
        for walker_id in walker_ids
    ]

    controller_ids: list[int] = []
    for response in client.apply_batch_sync(controller_batch, True):
        if response.error:
            print(f"[collector] Walker controller warning: {response.error}")
        else:
            controller_ids.append(response.actor_id)

    world.tick()

    active_count = min(len(controller_ids), len(walker_ids), len(valid_speeds))
    for controller_id, speed in zip(
        controller_ids[:active_count], valid_speeds[:active_count]
    ):
        controller = world.get_actor(controller_id)
        if controller is None:
            continue
        controller.start()
        destination = world.get_random_location_from_navigation()
        if destination is not None:
            controller.go_to_location(destination)
        controller.set_max_speed(float(speed))

    print(f"[collector] Spawned {active_count} walkers")
    return walker_ids, controller_ids


def destroy_actor_ids(client: carla.Client, actor_ids: list[int]) -> None:
    if not actor_ids:
        return
    client.apply_batch([carla.command.DestroyActor(actor_id) for actor_id in actor_ids])


def apply_weather(
    world: carla.World,
    name: str,
    settle_ticks: int,
    sync: bool,
) -> None:
    preset = weather_presets()[name]
    world.set_weather(preset)
    weather = world.get_weather()
    print(
        f"[collector] Weather changed to {name}: "
        f"cloud={weather.cloudiness:.1f}, "
        f"rain={weather.precipitation:.1f}, "
        f"wetness={weather.wetness:.1f}, "
        f"sun_altitude={weather.sun_altitude_angle:.1f}"
    )

    for _ in range(max(0, settle_ticks)):
        if sync:
            world.tick()
        else:
            world.wait_for_tick()


def camera_intrinsics(width: int, height: int, fov: float) -> dict[str, float]:
    focal = width / (2.0 * math.tan(math.radians(fov) / 2.0))
    return {
        "fx": float(focal),
        "fy": float(focal),
        "cx": float(width / 2.0),
        "cy": float(height / 2.0),
    }


def format_duration(seconds: float) -> str:
    seconds = max(0, int(round(seconds)))
    hours, remainder = divmod(seconds, 3600)
    minutes, seconds = divmod(remainder, 60)
    if hours:
        return f"{hours}h {minutes:02d}m {seconds:02d}s"
    if minutes:
        return f"{minutes}m {seconds:02d}s"
    return f"{seconds}s"


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Standalone CARLA lane-dataset collector with automatic map loading, "
            "weather cycling, traffic spawning, and stuck-vehicle recovery."
        )
    )
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--tm-port", type=int, default=8000)
    parser.add_argument(
        "--map",
        default="current",
        help="Map to load, for example Town03. Use 'current' to keep it.",
    )
    parser.add_argument(
        "--weathers",
        default="ClearNoon",
        help="Comma-separated CARLA weather presets.",
    )
    parser.add_argument(
        "--frames-per-weather",
        type=int,
        default=500,
        help="Saved samples before switching to the next weather.",
    )
    parser.add_argument("--weather-settle-ticks", type=int, default=15)

    parser.add_argument(
        "--out",
        default=str(Path.home() / "datasets" / "carla_lane_dataset"),
    )
    parser.add_argument("--run-name", default="")
    parser.add_argument("--width", type=int, default=800)
    parser.add_argument("--height", type=int, default=600)
    parser.add_argument("--fov", type=float, default=90.0)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--save-every", type=int, default=1)
    parser.add_argument("--max-frames", type=int, default=2000)
    parser.add_argument("--progress-every", type=int, default=25)
    parser.add_argument("--jpeg-quality", type=int, default=95)
    parser.add_argument("--save-semantic-raw", action=argparse.BooleanOptionalAction, default=True)

    parser.add_argument("--autopilot", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--vehicle-blueprint", default="vehicle.*model3*")
    parser.add_argument("--respawn-every-saved", type=int, default=250)
    parser.add_argument("--respawn-on-weather-change", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--stuck-seconds", type=float, default=20.0)
    parser.add_argument("--stuck-speed-mps", type=float, default=0.25)
    parser.add_argument("--warmup-ticks", type=int, default=10)

    parser.add_argument("--traffic-vehicles", type=int, default=15)
    parser.add_argument("--walkers", type=int, default=5)
    parser.add_argument("--safe-traffic", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--traffic-seed", type=int, default=42)
    parser.add_argument("--traffic-distance", type=float, default=2.5)

    parser.add_argument("--sync", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--sensor-timeout", type=float, default=3.0)
    parser.add_argument("--client-timeout", type=float, default=120.0)
    parser.add_argument("--list-weather", action="store_true")
    args = parser.parse_args()

    if args.list_weather:
        print("\n".join(weather_presets()))
        return 0

    if args.max_frames <= 0:
        raise ValueError("--max-frames must be positive")
    if args.save_every <= 0:
        raise ValueError("--save-every must be positive")
    if args.frames_per_weather <= 0:
        raise ValueError("--frames-per-weather must be positive")

    selected_weathers = parse_weather_list(args.weathers)
    requested_map = normalize_map_name(args.map)

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.client_timeout)
    world = client.get_world()

    if requested_map != "current" and current_town_name(world) != requested_map:
        print(
            f"[collector] Loading map {requested_map} on CARLA port {args.port}..."
        )
        world = client.load_world(requested_map)
        time.sleep(2.0)
        print(f"[collector] Map loaded: {current_town_name(world)}")

    map_name = current_town_name(world)
    run_name = args.run_name.strip() or (
        f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_{map_name}"
    )
    run_root = Path(args.out).expanduser() / map_name / run_name
    if run_root.exists():
        raise FileExistsError(
            f"Run directory already exists: {run_root}. Use a new --run-name."
        )

    rgb_dir = run_root / "rgb"
    semantic_dir = run_root / "semantic_labels"
    roadline_dir = run_root / "roadline_masks"
    rgb_dir.mkdir(parents=True, exist_ok=False)
    semantic_dir.mkdir(parents=True, exist_ok=False)
    roadline_dir.mkdir(parents=True, exist_ok=False)
    metadata_path = run_root / "metadata.jsonl"
    manifest_path = run_root / "manifest.json"

    original_settings = world.get_settings()
    traffic_manager = client.get_trafficmanager(args.tm_port)
    traffic_manager.set_random_device_seed(args.traffic_seed)
    traffic_manager.set_global_distance_to_leading_vehicle(args.traffic_distance)

    if args.sync:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1.0 / max(args.fps, 1)
        settings.no_rendering_mode = False
        world.apply_settings(settings)
        traffic_manager.set_synchronous_mode(True)
    else:
        traffic_manager.set_synchronous_mode(False)

    rng = np.random.default_rng(args.traffic_seed)
    hero: Optional[carla.Vehicle] = None
    rgb_camera: Optional[carla.Sensor] = None
    semantic_camera: Optional[carla.Sensor] = None
    rgb_queue: Optional[queue.Queue] = None
    semantic_queue: Optional[queue.Queue] = None
    traffic_vehicle_ids: list[int] = []
    walker_ids: list[int] = []
    walker_controller_ids: list[int] = []

    stop = {"flag": False}

    def on_signal(_signum, _frame) -> None:
        stop["flag"] = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    current_weather_index = 0
    current_weather = selected_weathers[current_weather_index]
    saved_for_weather = 0
    saved = 0
    processed = 0
    sensor_timeouts = 0
    respawn_count = 0
    low_speed_ticks = 0
    started_at = time.monotonic()

    def tick_world() -> int:
        if args.sync:
            return int(world.tick())
        snapshot = world.wait_for_tick()
        return int(snapshot.frame)

    def warmup() -> None:
        assert rgb_queue is not None
        assert semantic_queue is not None
        drain_queue(rgb_queue)
        drain_queue(semantic_queue)
        for _ in range(max(0, args.warmup_ticks)):
            tick_world()
        drain_queue(rgb_queue)
        drain_queue(semantic_queue)

    def spawn_collector_vehicle() -> None:
        nonlocal hero, rgb_camera, semantic_camera, rgb_queue, semantic_queue
        nonlocal low_speed_ticks, respawn_count

        destroy_actor(rgb_camera)
        destroy_actor(semantic_camera)
        if hero is not None:
            try:
                hero.set_autopilot(False, args.tm_port)
            except Exception:
                pass
        destroy_actor(hero)

        hero = spawn_hero_vehicle(world, args.vehicle_blueprint, rng)
        if args.autopilot:
            hero.set_autopilot(True, args.tm_port)

        rgb_camera, semantic_camera, rgb_queue, semantic_queue = attach_cameras(
            world,
            hero,
            args.width,
            args.height,
            args.fov,
        )
        low_speed_ticks = 0
        respawn_count += 1
        print(
            f"[collector] Hero ready: id={hero.id}, "
            f"spawn/respawn count={respawn_count}"
        )
        warmup()

    try:
        apply_weather(
            world,
            current_weather,
            args.weather_settle_ticks,
            args.sync,
        )

        spawn_collector_vehicle()
        traffic_vehicle_ids = spawn_traffic_vehicles(
            client,
            world,
            args.tm_port,
            args.traffic_vehicles,
            args.traffic_seed,
            args.safe_traffic,
        )
        walker_ids, walker_controller_ids = spawn_walkers(
            client,
            world,
            args.walkers,
            args.traffic_seed + 1,
        )

        manifest = {
            "dataset_version": 2,
            "created_at_utc": datetime.now(timezone.utc).isoformat(),
            "host": args.host,
            "carla_port": args.port,
            "traffic_manager_port": args.tm_port,
            "map_name": map_name,
            "weather_cycle": selected_weathers,
            "frames_per_weather": args.frames_per_weather,
            "camera": {
                "width": args.width,
                "height": args.height,
                "fov_deg": args.fov,
                "intrinsics": camera_intrinsics(args.width, args.height, args.fov),
                "relative_transform": transform_to_dict(
                    carla.Transform(carla.Location(x=1.5, z=2.4))
                ),
            },
            "roadline_label_id": ROADLINE_ID,
            "sampling": {
                "fps": args.fps,
                "save_every": args.save_every,
                "max_frames": args.max_frames,
            },
            "traffic": {
                "requested_vehicles": args.traffic_vehicles,
                "spawned_vehicles": len(traffic_vehicle_ids),
                "requested_walkers": args.walkers,
                "spawned_walkers": len(walker_ids),
                "seed": args.traffic_seed,
            },
        }
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")

        print("[collector] Lane collection started")
        print(f"[collector] CARLA port          : {args.port}")
        print(f"[collector] Traffic Manager    : {args.tm_port}")
        print(f"[collector] Map                 : {map_name}")
        print(f"[collector] Weather cycle       : {', '.join(selected_weathers)}")
        print(f"[collector] Output              : {run_root}")
        print(f"[collector] Max saved frames    : {args.max_frames}")
        print(f"[collector] Respawn every saved : {args.respawn_every_saved}")
        print(f"[collector] Stuck timeout       : {args.stuck_seconds:.1f}s")

        while not stop["flag"] and saved < args.max_frames:
            assert hero is not None
            assert rgb_queue is not None
            assert semantic_queue is not None

            target_frame = tick_world()

            try:
                rgb_image, semantic_image = get_aligned_pair(
                    rgb_queue,
                    semantic_queue,
                    target_frame,
                    args.sensor_timeout,
                )
            except queue.Empty:
                sensor_timeouts += 1
                print(
                    f"[collector] WARNING: sensor timeout "
                    f"(count={sensor_timeouts})"
                )
                continue

            vehicle_speed = speed_mps(hero)
            if args.autopilot and vehicle_speed < args.stuck_speed_mps:
                low_speed_ticks += 1
            else:
                low_speed_ticks = 0

            stuck_limit = max(1, int(args.stuck_seconds * args.fps))
            if args.autopilot and low_speed_ticks >= stuck_limit:
                print(
                    f"[collector] Hero stuck for about {args.stuck_seconds:.1f}s; "
                    "respawning at another spawn point..."
                )
                spawn_collector_vehicle()
                continue

            if processed % args.save_every != 0:
                processed += 1
                continue

            bgr = carla_image_to_bgr(rgb_image)
            semantic_ids = carla_semantic_to_ids(semantic_image)
            roadline_mask = (semantic_ids == ROADLINE_ID).astype(np.uint8) * 255

            name = f"{saved:06d}"
            rgb_path = rgb_dir / f"{name}.jpg"
            semantic_path = semantic_dir / f"{name}.png"
            roadline_path = roadline_dir / f"{name}.png"

            if not cv2.imwrite(
                str(rgb_path),
                bgr,
                [int(cv2.IMWRITE_JPEG_QUALITY), args.jpeg_quality],
            ):
                raise RuntimeError(f"Failed to write {rgb_path}")

            if args.save_semantic_raw:
                if not cv2.imwrite(str(semantic_path), semantic_ids):
                    raise RuntimeError(f"Failed to write {semantic_path}")

            if not cv2.imwrite(str(roadline_path), roadline_mask):
                raise RuntimeError(f"Failed to write {roadline_path}")

            roadline_pixels = int(np.count_nonzero(roadline_mask))
            record = {
                "sample_index": saved,
                "carla_frame": int(rgb_image.frame),
                "simulation_timestamp": float(rgb_image.timestamp),
                "map_name": map_name,
                "weather": current_weather,
                "weather_parameters": weather_to_dict(world.get_weather()),
                "hero_actor_id": int(hero.id),
                "hero_speed_mps": float(vehicle_speed),
                "hero_transform": transform_to_dict(hero.get_transform()),
                "camera_transform": transform_to_dict(rgb_image.transform),
                "roadline_pixel_count": roadline_pixels,
                "roadline_pixel_fraction": float(
                    roadline_pixels / roadline_mask.size
                ),
                "rgb_path": str(rgb_path.relative_to(run_root)),
                "semantic_labels_path": (
                    str(semantic_path.relative_to(run_root))
                    if args.save_semantic_raw
                    else None
                ),
                "roadline_mask_path": str(roadline_path.relative_to(run_root)),
            }
            with metadata_path.open("a", encoding="utf-8") as metadata_file:
                metadata_file.write(json.dumps(record, separators=(",", ":")) + "\n")

            saved += 1
            saved_for_weather += 1
            processed += 1

            if saved % max(1, args.progress_every) == 0 or saved == args.max_frames:
                elapsed = max(1e-6, time.monotonic() - started_at)
                rate = saved / elapsed
                remaining = max(0, args.max_frames - saved)
                eta = remaining / rate if rate > 1e-9 else 0.0
                print(
                    f"[collector] progress={saved}/{args.max_frames} "
                    f"({100.0 * saved / args.max_frames:.1f}%) "
                    f"rate={rate:.2f} samples/s "
                    f"ETA={format_duration(eta)} "
                    f"weather={current_weather} "
                    f"speed={vehicle_speed:.2f}m/s "
                    f"roadline_px={roadline_pixels}"
                )

            weather_change_due = (
                len(selected_weathers) > 1
                and saved_for_weather >= args.frames_per_weather
                and saved < args.max_frames
            )
            periodic_respawn_due = (
                args.respawn_every_saved > 0
                and saved % args.respawn_every_saved == 0
                and saved < args.max_frames
            )

            if weather_change_due:
                current_weather_index = (
                    current_weather_index + 1
                ) % len(selected_weathers)
                current_weather = selected_weathers[current_weather_index]
                saved_for_weather = 0
                apply_weather(
                    world,
                    current_weather,
                    args.weather_settle_ticks,
                    args.sync,
                )
                if args.respawn_on_weather_change:
                    print("[collector] Respawning hero after weather change...")
                    spawn_collector_vehicle()
            elif periodic_respawn_due:
                print("[collector] Periodic hero respawn for route diversity...")
                spawn_collector_vehicle()

    finally:
        print("[collector] Cleaning up...")

        destroy_actor(rgb_camera)
        destroy_actor(semantic_camera)
        if hero is not None:
            try:
                hero.set_autopilot(False, args.tm_port)
            except Exception:
                pass
        destroy_actor(hero)

        for controller_id in walker_controller_ids:
            controller = world.get_actor(controller_id)
            if controller is not None:
                try:
                    controller.stop()
                except Exception:
                    pass

        destroy_actor_ids(client, walker_controller_ids)
        destroy_actor_ids(client, walker_ids)
        destroy_actor_ids(client, traffic_vehicle_ids)

        try:
            traffic_manager.set_synchronous_mode(False)
        except Exception:
            pass

        try:
            world.apply_settings(original_settings)
        except Exception:
            pass

        elapsed = time.monotonic() - started_at
        print(
            f"[collector] Done. saved={saved}, "
            f"sensor_timeouts={sensor_timeouts}, "
            f"respawns={respawn_count}, "
            f"elapsed={format_duration(elapsed)}"
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
