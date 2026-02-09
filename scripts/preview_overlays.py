import argparse
import random
from pathlib import Path

import numpy as np
from PIL import Image
import matplotlib.pyplot as plt


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--root", type=Path, default=Path("datasets/oxford-iiit-pet-mmseg"))
    p.add_argument("--split", type=str, default="train", choices=["train", "val"])
    p.add_argument("--n", type=int, default=6, help="Number of samples to preview.")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--out", type=Path, default=Path("out/pets_preview"))
    return p.parse_args()


def main():
    args = parse_args()

    img_dir = args.root / "img_dir" / args.split
    ann_dir = args.root / "ann_dir" / args.split
    args.out.mkdir(parents=True, exist_ok=True)

    imgs = sorted(img_dir.glob("*.jpg"))
    if not imgs:
        raise FileNotFoundError(f"No images found in {img_dir}")

    random.seed(args.seed)
    sample = random.sample(imgs, min(args.n, len(imgs)))

    for img_path in sample:
        mask_path = ann_dir / f"{img_path.stem}.png"
        if not mask_path.exists():
            continue

        img = np.array(Image.open(img_path).convert("RGB"), dtype=np.uint8)
        mask = np.array(Image.open(mask_path), dtype=np.uint8)

        # Create an overlay: pet pixels tinted red
        overlay = img.copy()
        pet = (mask == 1)
        overlay[pet] = (overlay[pet] * 0.5 + np.array([255, 0, 0]) * 0.5).astype(np.uint8)

        # Save figure without axes
        plt.figure()
        plt.imshow(overlay)
        plt.axis("off")
        plt.title(img_path.stem)
        plt.savefig(args.out / f"{img_path.stem}.png", bbox_inches="tight", pad_inches=0)
        plt.close()

    print(f"Saved previews to: {args.out}")


if __name__ == "__main__":
    main()
