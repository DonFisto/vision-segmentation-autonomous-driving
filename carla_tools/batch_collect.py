#!/usr/bin/env python3
import subprocess
import sys
from pathlib import Path

PYTHON = sys.executable
REPO_ROOT = Path.home() / "vision-segmentation-autonomous-driving"
COLLECTOR = REPO_ROOT / "carla_tools" / "collect_carla_cityscapes.py"

WEATHERS = [
    "ClearNoon",
    "CloudyNoon",
    "WetNoon",
    "ClearSunset",
]

HOST = "localhost"
PORT = 2000
OUT = str(Path.home() / "datasets" / "carla_raw")

WIDTH = 800
HEIGHT = 600
FOV = 90.0
FPS = 15
SAVE_EVERY = 2
MAX_FRAMES = 3000


def run_one(weather: str):
    cmd = [
        PYTHON, str(COLLECTOR),
        "--host", HOST,
        "--port", str(PORT),
        "--weather", weather,
        "--out", OUT,
        "--width", str(WIDTH),
        "--height", str(HEIGHT),
        "--fov", str(FOV),
        "--fps", str(FPS),
        "--save-every", str(SAVE_EVERY),
        "--max-frames", str(MAX_FRAMES),
        "--autopilot",
    ]

    print("=" * 80)
    print(f"[batch] Collecting current map with weather={weather}")
    print(" ".join(cmd))
    print("=" * 80)

    result = subprocess.run(cmd)
    return result.returncode == 0


def main():
    failures = []

    for weather in WEATHERS:
        ok = run_one(weather)
        if not ok:
            failures.append(weather)
            print(f"[batch] ERROR: failed for weather={weather}")
            break

    print("\n[batch] Finished.")
    if failures:
        print("[batch] Failed weather presets:")
        for weather in failures:
            print(f"  - {weather}")
    else:
        print("[batch] All weather presets succeeded.")


if __name__ == "__main__":
    main()
