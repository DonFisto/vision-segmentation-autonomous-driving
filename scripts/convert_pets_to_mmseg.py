import argparse
import random
import shutil
from pathlib import Path

import numpy as np
from PIL import Image

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Convert Oxford-IIIT Pet dataset into MMSeg format")
    p.add_argument("--src", type=Path, default=Path("datasets/oxford-iiit-pet"),
                   help="Path to the Oxford-IIIT Pet Dataset (contains images/ and annotations/trimaps/)")
    p.add_argument("--dst", type=Path, default=Path("datasets/oxford-iiit-pet-mmseg"),
                   help="Destination directory (will contain img_dir/ and ann_dir/)")
    p.add_argument("--val-ratio", type=float, default=0.2,
                   help="Validation split ratio in (0,1). Example: 0.2")
    p.add_argument("--seed", type=int, default=42, help="Random seed for split reproducibility")

    p.add_argument("--force", action="store_true",
                   help="If set, deletes existing dst/img_dir and dst/ann_dir before writing")
    p.add_argument("--exts", nargs="+", default=[".jpg", ".jpeg", ".png", ".bmp"],
                   help="Allowed image extensions when resolving image files (fallback mode)")
    p.add_argument("--dry-run", action="store_true",
                   help="If set, only prints what would happen (no copying/writing)")
    args = p.parse_args()

    if not (0.0 < args.val_ratio < 1.0):
        raise ValueError("--val-ratio must be between 0 and 1 (exclusive). Example: 0.2")

    # Normalize extensions
    args.exts = [e.lower() if e.startswith(".") else f".{e.lower()}" for e in args.exts]
    return args


def convert_trimap_to_binary(trimap_path:Path) -> Image.Image:
    trimap = np.array(Image.open(trimap_path), dtype=np.uint8)
    out = np.zeros_like(trimap, dtype=np.uint8)
    out[(trimap == 1) | (trimap == 3)] = 1  # Pet

    return Image.fromarray(out, mode="L")
def main() -> None:
    args = parse_args()

    # Optional cleanup
    if args.force and args.dst.exists():
        shutil.rmtree(args.dst / "img_dir", ignore_errors=True)
        shutil.rmtree(args.dst / "ann_dir", ignore_errors=True)

    src_img_dir = args.src / "images"
    src_trimap_dir = args.src / "annotations" / "trimaps"

    if not src_img_dir.exists():
        raise FileNotFoundError(f"Source image directory not found: {src_img_dir}")
    if not src_trimap_dir.exists():
        raise FileNotFoundError(f"Source trimap directory not found: {src_trimap_dir}")
    
    for split in ("train", "val"):
        (args.dst / "img_dir" / split).mkdir(parents=True, exist_ok=True)
        (args.dst / "ann_dir" / split).mkdir(parents=True, exist_ok=True)

    # Collect all trimap png files
    all_trimap_paths = list(src_trimap_dir.glob("*.png"))

    # Sanitize stems: some files may be macOS resource files like ._NAME.png
    # or have an accidental leading dot. We remove a leading '._' or '.' from stems
    # and deduplicate the resulting IDs while preserving order.
    ids_list = []
    seen = set()
    adjusted = 0
    for p in all_trimap_paths:
        raw = p.stem
        cleaned = raw
        if raw.startswith('._'):
            cleaned = raw[2:]
            adjusted += 1
        elif raw.startswith('.'):
            cleaned = raw[1:]
            adjusted += 1

        if cleaned in seen:
            # duplicate after cleaning (e.g. both '._X' and 'X' present) -> skip
            continue
        seen.add(cleaned)
        ids_list.append(cleaned)

    if adjusted:
        print(f"Warning: adjusted {adjusted} trimap IDs by stripping leading metadata chars in {src_trimap_dir}")

    ids = sorted(ids_list)

    random.seed(args.seed)
    random.shuffle(ids)
    n_val = int(args.val_ratio * len(ids))
    val_ids = set(ids[:n_val])
    train_ids = set(ids[n_val:])

    def find_image_path(_id: str) -> Path:
        jpg = src_img_dir / f"{_id}.jpg"
        if jpg.exists():
            return jpg

        for ext in args.exts:
            cand = src_img_dir / f"{_id}{ext}"
            if cand.exists():
                return cand

        # last resort: restricted glob
        matches = []
        for ext in args.exts:
            matches.extend(src_img_dir.glob(f"{_id}{ext}"))

        if not matches:
            raise FileNotFoundError(f"No image file found for ID: {_id} (allowed: {args.exts})")
        return matches[0]

    
    skipped = 0
    written = 0


    def copy_pair(_id: str, split: str) -> None:
        nonlocal skipped, written

        trimap_path = src_trimap_dir / f"{_id}.png"
        if not trimap_path.exists():
            skipped += 1
            return

        try:
            img_path = find_image_path(_id)
        except FileNotFoundError:
            skipped += 1
            return

        dst_img = args.dst / "img_dir" / split / f"{_id}.jpg"
        dst_msk = args.dst / "ann_dir" / split / f"{_id}.png"

        written += 1
        if args.dry_run:
            return

        shutil.copy2(img_path, dst_img)
        convert_trimap_to_binary(trimap_path).save(dst_msk)



    for _id in train_ids:
        copy_pair(_id, "train")
    for _id in val_ids:
        copy_pair(_id, "val")

    n_train = len(list((args.dst / "img_dir" / "train").glob("*.jpg")))
    n_val2 = len(list((args.dst / "img_dir" / "val").glob("*.jpg")))
    print(f"Conversion completed.")
    print(f"Output directory: {args.dst}")
    print(f"Training samples: {n_train}")
    print(f"Validation samples: {n_val2}")

    print(f"Written pairs: {written}")
    print(f"Skipped (missing trimap/image): {skipped}")

    
if __name__ == "__main__":
    main()
