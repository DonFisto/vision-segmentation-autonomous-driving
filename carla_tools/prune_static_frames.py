#!/usr/bin/env python3
import argparse
import shutil
from pathlib import Path

import cv2
import numpy as np


def copy_pair(img_src: Path, lbl_src: Path, img_dst: Path, lbl_dst: Path):
    img_dst.parent.mkdir(parents=True, exist_ok=True)
    lbl_dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(img_src, img_dst)
    shutil.copy2(lbl_src, lbl_dst)


def frame_diff_score(img_a: np.ndarray, img_b: np.ndarray, size=(160, 90)) -> float:
    a = cv2.resize(img_a, size, interpolation=cv2.INTER_AREA)
    b = cv2.resize(img_b, size, interpolation=cv2.INTER_AREA)
    a = cv2.cvtColor(a, cv2.COLOR_BGR2GRAY)
    b = cv2.cvtColor(b, cv2.COLOR_BGR2GRAY)
    return float(np.mean(np.abs(a.astype(np.float32) - b.astype(np.float32))))


def semantic_diff_score(mask_a: np.ndarray, mask_b: np.ndarray, size=(160, 90)) -> float:
    a = cv2.resize(mask_a, size, interpolation=cv2.INTER_NEAREST)
    b = cv2.resize(mask_b, size, interpolation=cv2.INTER_NEAREST)
    return float(np.mean((a != b).astype(np.float32)))


def process_train_split(in_root: Path, out_root: Path, img_diff_thresh: float, sem_diff_thresh: float):
    img_dir = in_root / "images" / "train"
    lbl_dir = in_root / "labels" / "train"

    out_img_dir = out_root / "images" / "train"
    out_lbl_dir = out_root / "labels" / "train"

    files = sorted([p.name for p in img_dir.glob("*.png") if (lbl_dir / p.name).exists()])
    if not files:
        print("[prune] No train files found.")
        return

    kept = 0
    skipped = 0

    prev_img = None
    prev_lbl = None

    for name in files:
        img_path = img_dir / name
        lbl_path = lbl_dir / name

        img = cv2.imread(str(img_path), cv2.IMREAD_COLOR)
        lbl = cv2.imread(str(lbl_path), cv2.IMREAD_UNCHANGED)

        if img is None or lbl is None:
            print(f"[prune] WARNING unreadable pair: {name}")
            continue

        keep = False

        if prev_img is None:
            keep = True
        else:
            img_diff = frame_diff_score(prev_img, img)
            sem_diff = semantic_diff_score(prev_lbl, lbl)

            if img_diff >= img_diff_thresh or sem_diff >= sem_diff_thresh:
                keep = True
            else:
                keep = False

        if keep:
            copy_pair(img_path, lbl_path, out_img_dir / name, out_lbl_dir / name)
            prev_img = img
            prev_lbl = lbl
            kept += 1
        else:
            skipped += 1

    print(f"[prune] Train kept   : {kept}")
    print(f"[prune] Train skipped: {skipped}")


def copy_val_split(in_root: Path, out_root: Path):
    img_dir = in_root / "images" / "val"
    lbl_dir = in_root / "labels" / "val"

    out_img_dir = out_root / "images" / "val"
    out_lbl_dir = out_root / "labels" / "val"

    files = sorted([p.name for p in img_dir.glob("*.png") if (lbl_dir / p.name).exists()])
    copied = 0

    for name in files:
        copy_pair(img_dir / name, lbl_dir / name, out_img_dir / name, out_lbl_dir / name)
        copied += 1

    print(f"[prune] Val copied: {copied}")


def main():
    ap = argparse.ArgumentParser(description="Prune near-duplicate static frames from filtered dataset.")
    ap.add_argument("--in-root", required=True, help="Input filtered dataset root")
    ap.add_argument("--out-root", required=True, help="Output pruned dataset root")
    ap.add_argument("--img-diff-thresh", type=float, default=3.0, help="Mean grayscale diff threshold")
    ap.add_argument("--sem-diff-thresh", type=float, default=0.01, help="Fraction of changed semantic pixels")
    args = ap.parse_args()

    in_root = Path(args.in_root)
    out_root = Path(args.out_root)

    print("[prune] Input root :", in_root)
    print("[prune] Output root:", out_root)
    print("[prune] img_diff_thresh:", args.img_diff_thresh)
    print("[prune] sem_diff_thresh:", args.sem_diff_thresh)

    process_train_split(in_root, out_root, args.img_diff_thresh, args.sem_diff_thresh)
    copy_val_split(in_root, out_root)

    print("[prune] Done.")


if __name__ == "__main__":
    main()
