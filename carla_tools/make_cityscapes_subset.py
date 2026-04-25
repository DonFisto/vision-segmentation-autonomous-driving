#!/usr/bin/env python3
import argparse
import random
import shutil
from pathlib import Path


def copy_file(src: Path, dst: Path):
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)


def collect_train_pairs(root: Path):
    img_root = root / "leftImg8bit" / "train"
    lbl_root = root / "gtFine" / "train"

    pairs = []
    for city_dir in sorted(img_root.iterdir()):
        if not city_dir.is_dir():
            continue
        city = city_dir.name
        for img_path in sorted(city_dir.glob("*_leftImg8bit.png")):
            stem = img_path.name.replace("_leftImg8bit.png", "")
            lbl_path = lbl_root / city / f"{stem}_gtFine_labelTrainIds.png"
            if lbl_path.exists():
                pairs.append((city, img_path, lbl_path))
    return pairs


def copy_val_split(root: Path, out_root: Path):
    for split in ["val"]:
        for sub in ["leftImg8bit", "gtFine"]:
            src = root / sub / split
            dst = out_root / sub / split
            if not src.exists():
                continue
            for path in src.rglob("*"):
                if path.is_file():
                    rel = path.relative_to(src)
                    copy_file(path, dst / rel)


def main():
    ap = argparse.ArgumentParser(description="Create a smaller Cityscapes subset.")
    ap.add_argument("--in-root", required=True, help="Official Cityscapes root")
    ap.add_argument("--out-root", required=True, help="Subset output root")
    ap.add_argument("--num-train", type=int, default=1000, help="Number of train images to keep")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument(
        "--mode",
        choices=["random", "balanced_by_city"],
        default="balanced_by_city",
        help="Sampling mode"
    )
    args = ap.parse_args()

    in_root = Path(args.in_root)
    out_root = Path(args.out_root)

    pairs = collect_train_pairs(in_root)
    if not pairs:
        raise SystemExit("No valid Cityscapes train pairs found.")

    rng = random.Random(args.seed)

    if args.mode == "random":
        rng.shuffle(pairs)
        chosen = pairs[:min(args.num_train, len(pairs))]
    else:
        # balanced_by_city
        by_city = {}
        for city, img_path, lbl_path in pairs:
            by_city.setdefault(city, []).append((city, img_path, lbl_path))

        cities = sorted(by_city.keys())
        for city in cities:
            rng.shuffle(by_city[city])

        chosen = []
        i = 0
        while len(chosen) < min(args.num_train, len(pairs)):
            progressed = False
            for city in cities:
                if i < len(by_city[city]) and len(chosen) < args.num_train:
                    chosen.append(by_city[city][i])
                    progressed = True
            if not progressed:
                break
            i += 1

    print(f"[subset] total available train pairs: {len(pairs)}")
    print(f"[subset] selected train pairs: {len(chosen)}")

    # copy selected train
    for city, img_path, lbl_path in chosen:
        img_rel = img_path.relative_to(in_root / "leftImg8bit" / "train")
        lbl_rel = lbl_path.relative_to(in_root / "gtFine" / "train")

        copy_file(img_path, out_root / "leftImg8bit" / "train" / img_rel)
        copy_file(lbl_path, out_root / "gtFine" / "train" / lbl_rel)

    # copy full val
    copy_val_split(in_root, out_root)

    print("[subset] done")
    print(f"[subset] output: {out_root}")


if __name__ == "__main__":
    main()
