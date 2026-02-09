#!/usr/bin/env python3
"""
MMSegmentation inference script (robust, no manual logits decoding).

What it does:
- Loads a pretrained (or your trained) MMSeg model from config + checkpoint
- Runs inference on one image or a folder of images
- Saves:
  1) overlay visualization (image + colored mask)
  2) raw predicted mask as PNG (class indices, 8-bit if possible, 16-bit if needed)

Requirements:
- mmsegmentation installed and importable
- opencv-python (for writing PNG masks)

Usage examples:
  python infer_mmseg.py \
    --config configs/pets/segformer_b0_pets.py \
    --checkpoint work_dirs/segformer_b0_pets/iter_2000.pth \
    --input datasets/oxford-iiit-pet-mmseg/img_dir/val/keeshond_91.jpg \
    --out-dir out/infer

  python infer_mmseg.py \
    --config configs/pets/segformer_b0_pets.py \
    --checkpoint work_dirs/segformer_b0_pets/iter_2000.pth \
    --input datasets/oxford-iiit-pet-mmseg/img_dir/val \
    --out-dir out/infer --recursive
"""

import argparse
import os
from pathlib import Path

import numpy as np
import cv2

from mmseg.apis import init_model, inference_model, show_result_pyplot


IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", ".webp"}


def list_images(input_path: Path, recursive: bool) -> list[Path]:
    if input_path.is_file():
        return [input_path]

    if not input_path.is_dir():
        raise FileNotFoundError(f"Input not found: {input_path}")

    if recursive:
        files = [p for p in input_path.rglob("*") if p.suffix.lower() in IMG_EXTS]
    else:
        files = [p for p in input_path.iterdir() if p.is_file() and p.suffix.lower() in IMG_EXTS]

    files.sort()
    return files


def save_index_mask(mask: np.ndarray, out_path: Path) -> None:
    """
    Saves the predicted mask as class indices.
    - If max class id <= 255 => save as 8-bit PNG
    - Else => save as 16-bit PNG
    """
    mask = np.asarray(mask)

    if mask.ndim != 2:
        raise ValueError(f"Expected 2D mask [H,W], got shape {mask.shape}")

    max_id = int(mask.max()) if mask.size else 0
    if max_id <= 255:
        mask_u8 = mask.astype(np.uint8, copy=False)
        ok = cv2.imwrite(str(out_path), mask_u8)
    else:
        mask_u16 = mask.astype(np.uint16, copy=False)
        ok = cv2.imwrite(str(out_path), mask_u16)

    if not ok:
        raise RuntimeError(f"Failed to write mask to {out_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, help="Path to MMSeg config .py")
    parser.add_argument("--checkpoint", required=True, help="Path to checkpoint .pth")
    parser.add_argument("--device", default="cuda:0", help="e.g. cuda:0 or cpu")
    parser.add_argument("--input", required=True, help="Image file or folder of images")
    parser.add_argument("--out-dir", default="out/infer", help="Output directory")
    parser.add_argument("--opacity", type=float, default=0.5, help="Overlay opacity (0..1)")
    parser.add_argument("--recursive", action="store_true", help="Recurse into subfolders")
    parser.add_argument("--no-overlay", action="store_true", help="Do not save overlay visualization")
    parser.add_argument("--no-mask", action="store_true", help="Do not save raw index mask")
    args = parser.parse_args()

    cfg = args.config
    ckpt = args.checkpoint
    device = args.device

    in_path = Path(args.input)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    overlay_dir = out_dir / "overlay"
    mask_dir = out_dir / "mask"
    if not args.no_overlay:
        overlay_dir.mkdir(parents=True, exist_ok=True)
    if not args.no_mask:
        mask_dir.mkdir(parents=True, exist_ok=True)

    # Init model once
    model = init_model(cfg, ckpt, device=device)

    images = list_images(in_path, recursive=args.recursive)
    if not images:
        raise RuntimeError(f"No images found under: {in_path}")

    for img_path in images:
        # Inference
        result = inference_model(model, str(img_path))

        # MMSeg returns a SegDataSample. The predicted semantic segmentation is here:
        # result.pred_sem_seg.data is a tensor with shape [1,H,W] or [H,W] depending on version.
        pred = result.pred_sem_seg.data
        if hasattr(pred, "detach"):
            pred = pred.detach().cpu().numpy()

        # Normalize to [H,W]
        if pred.ndim == 3 and pred.shape[0] == 1:
            pred_hw = pred[0]
        elif pred.ndim == 2:
            pred_hw = pred
        else:
            raise ValueError(f"Unexpected pred shape for {img_path.name}: {pred.shape}")

        stem = img_path.stem

        # Save raw mask (class indices)
        if not args.no_mask:
            mask_out = mask_dir / f"{stem}_mask.png"
            save_index_mask(pred_hw, mask_out)

        # Save overlay visualization using MMSeg's built-in renderer
        if not args.no_overlay:
            overlay_out = overlay_dir / f"{stem}_overlay.png"
            show_result_pyplot(
                model,
                str(img_path),
                result,
                opacity=args.opacity,
                out_file=str(overlay_out),
                show=False,
            )

        print(f"[OK] {img_path.name}")

    print(f"Done. Outputs saved under: {out_dir}")


if __name__ == "__main__":
    main()
