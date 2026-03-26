#!/usr/bin/env python3
import argparse
from pathlib import Path
import cv2
import numpy as np

IGNORE = 255

# CARLA 0.9.16 semantic IDs -> Cityscapes-19 train IDs
CARLA_0916_TO_CS19 = {
    1: 0,    # road
    2: 1,    # sidewalk
    3: 2,    # building
    4: 3,    # wall
    5: 4,    # fence
    6: 5,    # pole
    7: 6,    # traffic light
    8: 7,    # traffic sign
    9: 8,    # vegetation
    10: 9,   # terrain
    11: 10,  # sky
    12: 11,  # person
    13: 12,  # rider
    14: 13,  # car
    15: 14,  # truck
    16: 15,  # bus
    17: 16,  # train
    18: 17,  # motorcycle
    19: 18,  # bicycle

    24: 0,       # road line -> road
    25: 9,       # ground -> terrain
    26: 2,       # bridge -> building
    27: IGNORE,  # rail track
    28: 4,       # guard rail -> fence

    0: IGNORE,
    20: IGNORE,  # static
    21: IGNORE,  # dynamic
    22: IGNORE,  # other
    23: IGNORE,  # water
}

VALID_EXTS = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}


def convert_one(ids: np.ndarray) -> np.ndarray:
    out = np.full(ids.shape, IGNORE, dtype=np.uint8)
    for carla_id, cs_id in CARLA_0916_TO_CS19.items():
        out[ids == carla_id] = cs_id
    return out


def list_pairs(in_root: Path):
    """
    Expected nested structure:
      in_root/
        TownXX/
          WeatherYY/
            rgb/
            sem_raw/
    Yields: (town, weather, rgb_path, sem_path)
    """
    pairs = []
    for town_dir in sorted(in_root.iterdir()):
        if not town_dir.is_dir():
            continue
        # skip legacy flat folders if still present
        if town_dir.name in {"rgb", "sem_raw"}:
            continue

        for weather_dir in sorted(town_dir.iterdir()):
            if not weather_dir.is_dir():
                continue

            rgb_dir = weather_dir / "rgb"
            sem_dir = weather_dir / "sem_raw"
            if not rgb_dir.is_dir() or not sem_dir.is_dir():
                continue

            rgb_files = sorted(
                [p for p in rgb_dir.iterdir() if p.is_file() and p.suffix.lower() in VALID_EXTS]
            )

            for rgb_path in rgb_files:
                sem_path = sem_dir / rgb_path.name
                if sem_path.is_file():
                    pairs.append((town_dir.name, weather_dir.name, rgb_path, sem_path))
    return pairs


def main():
    ap = argparse.ArgumentParser(description="Convert nested CARLA semantic masks to Cityscapes-19.")
    ap.add_argument("--in-root", required=True, help="Root with nested Town/Weather/rgb + sem_raw folders")
    ap.add_argument("--out-root", required=True, help="Output root with images/{train,val}, labels/{train,val}")
    ap.add_argument(
        "--val-maps",
        nargs="+",
        required=True,
        help="Map folder names to reserve fully for validation, e.g. Town10HD Town05",
    )
    ap.add_argument(
        "--copy-mode",
        choices=["copy", "overwrite"],
        default="overwrite",
        help="'overwrite' rewrites files with same numeric names per split; 'copy' keeps unique names with town_weather prefixes",
    )
    args = ap.parse_args()

    in_root = Path(args.in_root)
    out_root = Path(args.out_root)
    val_maps = set(args.val_maps)

    (out_root / "images" / "train").mkdir(parents=True, exist_ok=True)
    (out_root / "images" / "val").mkdir(parents=True, exist_ok=True)
    (out_root / "labels" / "train").mkdir(parents=True, exist_ok=True)
    (out_root / "labels" / "val").mkdir(parents=True, exist_ok=True)

    pairs = list_pairs(in_root)
    if not pairs:
        raise SystemExit(f"No valid nested rgb/sem_raw pairs found under {in_root}")

    print(f"[convert] Found {len(pairs)} paired frames total")
    print(f"[convert] Validation maps: {sorted(val_maps)}")

    train_count = 0
    val_count = 0

    for idx, (town, weather, rgb_path, sem_path) in enumerate(pairs, 1):
        split = "val" if town in val_maps else "train"

        bgr = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
        ids = cv2.imread(str(sem_path), cv2.IMREAD_GRAYSCALE)

        if bgr is None or ids is None:
            print(f"[convert] WARNING unreadable pair: {rgb_path} / {sem_path}")
            continue

        cs = convert_one(ids)

        if args.copy_mode == "copy":
            out_name = f"{town}__{weather}__{rgb_path.stem}.png"
        else:
            # keep original name; safe as long as file names are globally unique or you run per-town
            out_name = rgb_path.name

        img_out = out_root / "images" / split / out_name
        lbl_out = out_root / "labels" / split / out_name

        cv2.imwrite(str(img_out), bgr)
        cv2.imwrite(str(lbl_out), cs)

        if split == "train":
            train_count += 1
        else:
            val_count += 1

        if idx % 500 == 0:
            print(f"[convert] processed {idx}/{len(pairs)}")

    print("[convert] done")
    print(f"[convert] train: {train_count}")
    print(f"[convert] val:   {val_count}")


if __name__ == "__main__":
    main()
