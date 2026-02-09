#!/usr/bin/env python3
import os
import argparse
from pathlib import Path

from mmseg.apis import init_model, inference_model, show_result_pyplot


def main():
    parser = argparse.ArgumentParser(description="Inference script for MMSegmentation")
    parser.add_argument("--config", required=True, help="Path to MMSeg config .py")
    parser.add_argument("--checkpoint", required=True, help="Path to checkpoint .pth")
    parser.add_argument("--device", default="cuda:0", help="Device (e.g. cuda:0 or cpu)")
    parser.add_argument("--img", required=True, help="Path to image or directory of images")
    parser.add_argument("--out-dir", required=True, help="Directory to save outputs")
    parser.add_argument("--opacity", type=float, default=0.5, help="Overlay opacity")
    parser.add_argument("--suffix", default="_pred.png", help="Output filename suffix")
    args = parser.parse_args()

    img_path = Path(args.img)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # 1. Load model
    model = init_model(args.config, args.checkpoint, device=args.device)

    # 2. Collect images
    if img_path.is_file():
        img_list = [img_path]
    elif img_path.is_dir():
        img_list = sorted(
            p for p in img_path.iterdir()
            if p.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp"}
        )
    else:
        raise ValueError(f"Invalid --img path: {img_path}")

    if not img_list:
        raise RuntimeError("No images found for inference")

    print(f"Running inference on {len(img_list)} images")

    # 3. Inference loop
    for img in img_list:
        result = inference_model(model, str(img))

        out_file = out_dir / f"{img.stem}{args.suffix}"

        show_result_pyplot(
            model,
            str(img),
            result,
            opacity=args.opacity,
            out_file=str(out_file),
            show=False,
        )

        print(f"Saved: {out_file}")

    print("Inference complete.")


if __name__ == "__main__":
    main()
