#!/usr/bin/env python3
import argparse
import random
import shutil
from pathlib import Path

import cv2
import numpy as np

# Cityscapes-19 IDs
ID2NAME = {
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

DEFAULT_TARGET_IDS = [6, 7, 11, 12, 13, 14, 15, 16, 17, 18]


def copy_pair(img_src: Path, lbl_src: Path, img_dst: Path, lbl_dst: Path):
    img_dst.parent.mkdir(parents=True, exist_ok=True)
    lbl_dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(img_src, img_dst)
    shutil.copy2(lbl_src, lbl_dst)


def score_mask(mask: np.ndarray, target_ids):
    total_target_pixels = 0
    per_class = {}
    for cid in target_ids:
        cnt = int(np.sum(mask == cid))
        per_class[cid] = cnt
        total_target_pixels += cnt
    return total_target_pixels, per_class


def process_split(split: str, in_root: Path, out_root: Path, target_ids, min_target_pixels: int,
                  keep_boring_ratio: float, seed: int):
    img_dir = in_root / "images" / split
    lbl_dir = in_root / "labels" / split

    out_img_dir = out_root / "images" / split
    out_lbl_dir = out_root / "labels" / split

    files = sorted([p.name for p in img_dir.glob("*.png") if (lbl_dir / p.name).exists()])
    if not files:
        print(f"[filter] No files found in split '{split}'")
        return

    rng = random.Random(seed if split == "train" else seed + 1)

    kept_informative = 0
    kept_boring = 0
    skipped_boring = 0

    for name in files:
        img_path = img_dir / name
        lbl_path = lbl_dir / name

        mask = cv2.imread(str(lbl_path), cv2.IMREAD_UNCHANGED)
        if mask is None:
            print(f"[filter] WARNING unreadable label: {lbl_path}")
            continue

        total_target_pixels, per_class = score_mask(mask, target_ids)

        informative = total_target_pixels >= min_target_pixels

        keep = False
        if informative:
            keep = True
            kept_informative += 1
        else:
            if rng.random() < keep_boring_ratio:
                keep = True
                kept_boring += 1
            else:
                skipped_boring += 1

        if keep:
            copy_pair(
                img_path,
                lbl_path,
                out_img_dir / name,
                out_lbl_dir / name
            )

    print(f"[filter] Split: {split}")
    print(f"[filter]   informative kept: {kept_informative}")
    print(f"[filter]   boring kept     : {kept_boring}")
    print(f"[filter]   boring skipped  : {skipped_boring}")
    print(f"[filter]   total kept      : {kept_informative + kept_boring}")


def main():
    ap = argparse.ArgumentParser(description="Filter a Cityscapes-19 style dataset by informative target-class content.")
    ap.add_argument("--in-root", required=True, help="Input dataset root (with images/train, labels/train, ...)")
    ap.add_argument("--out-root", required=True, help="Output filtered dataset root")
    ap.add_argument(
        "--target-ids",
        nargs="+",
        type=int,
        default=DEFAULT_TARGET_IDS,
        help="Class IDs considered informative"
    )
    ap.add_argument(
        "--min-target-pixels",
        type=int,
        default=400,
        help="Minimum total target pixels to classify a frame as informative"
    )
    ap.add_argument(
        "--keep-boring-ratio",
        type=float,
        default=0.15,
        help="Random fraction of boring frames to keep"
    )
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()

    in_root = Path(args.in_root)
    out_root = Path(args.out_root)

    print("[filter] Input root :", in_root)
    print("[filter] Output root:", out_root)
    print("[filter] Target IDs :", args.target_ids)
    print("[filter] Target classes:", [ID2NAME.get(i, str(i)) for i in args.target_ids])
    print("[filter] min_target_pixels:", args.min_target_pixels)
    print("[filter] keep_boring_ratio:", args.keep_boring_ratio)

    if out_root.exists():
        print(f"[filter] WARNING: output root exists and files may be overwritten: {out_root}")

    process_split(
        "train",
        in_root,
        out_root,
        args.target_ids,
        args.min_target_pixels,
        args.keep_boring_ratio,
        args.seed,
    )

    # For validation, keep everything by default so evaluation stays honest
    process_split(
        "val",
        in_root,
        out_root,
        args.target_ids,
        min_target_pixels=-1,   # every val image becomes "informative"
        keep_boring_ratio=1.0,  # keep all val frames
        seed=args.seed,
    )

    print("[filter] Done.")


if __name__ == "__main__":
    main()
