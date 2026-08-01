#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import errno
import os
import shutil
from pathlib import Path

import cv2
import numpy as np


VALID_IMAGE_SUFFIXES = {
    ".jpg",
    ".jpeg",
    ".png",
    ".bmp",
    ".tif",
    ".tiff",
}

VALID_MASK_VALUES = {0, 1, 255}


def link_or_copy(source: Path, destination: Path) -> str:
    """
    Hard-link when source and destination are on the same filesystem.
    Fall back to copying across filesystems.
    """
    try:
        os.link(source, destination)
        return "hardlink"
    except OSError as exc:
        if exc.errno != errno.EXDEV:
            raise

        shutil.copy2(source, destination)
        return "copy"


def find_pairs(
    input_root: Path,
) -> list[tuple[str, str, Path, Path]]:
    """
    Expected structure:

      input_root/
        Town01/
          Town01_all_weather/
            rgb/
              000000.jpg
            roadline_masks/
              000000.png

    Images and masks are paired by filename stem.
    """
    pairs: list[tuple[str, str, Path, Path]] = []

    for town_dir in sorted(input_root.iterdir()):
        if not town_dir.is_dir():
            continue

        for run_dir in sorted(town_dir.iterdir()):
            if not run_dir.is_dir():
                continue

            rgb_dir = run_dir / "rgb"
            mask_dir = run_dir / "roadline_masks"

            if not rgb_dir.is_dir() or not mask_dir.is_dir():
                continue

            masks_by_stem = {
                path.stem: path
                for path in mask_dir.iterdir()
                if path.is_file()
                and path.suffix.lower() in VALID_IMAGE_SUFFIXES
            }

            for image_path in sorted(rgb_dir.iterdir()):
                if (
                    not image_path.is_file()
                    or image_path.suffix.lower()
                    not in VALID_IMAGE_SUFFIXES
                ):
                    continue

                mask_path = masks_by_stem.get(image_path.stem)

                if mask_path is None:
                    raise RuntimeError(
                        "Missing road-line mask for image:\n"
                        f"  image: {image_path}\n"
                        f"  expected stem: {image_path.stem}"
                    )

                pairs.append(
                    (
                        town_dir.name,
                        run_dir.name,
                        image_path,
                        mask_path,
                    )
                )

    return pairs


def write_jpeg(
    source_path: Path,
    source_image: np.ndarray,
    destination_path: Path,
    jpeg_quality: int,
) -> str:
    """
    Preserve existing JPEG files through a hard link where possible.
    Convert any other image format to JPEG.
    """
    if source_path.suffix.lower() in {".jpg", ".jpeg"}:
        return link_or_copy(source_path, destination_path)

    success = cv2.imwrite(
        str(destination_path),
        source_image,
        [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality],
    )

    if not success:
        raise RuntimeError(
            f"Could not write image: {destination_path}"
        )

    return "encoded"


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Convert CARLA RGB + roadline_masks runs into a "
            "binary MMSegmentation lane-marking dataset."
        )
    )

    parser.add_argument(
        "--in-root",
        required=True,
        help=(
            "Root containing Town/Run/rgb and "
            "Town/Run/roadline_masks directories"
        ),
    )

    parser.add_argument(
        "--out-root",
        required=True,
        help=(
            "Fresh output directory containing "
            "images/{train,val} and labels/{train,val}"
        ),
    )

    parser.add_argument(
        "--val-maps",
        nargs="+",
        required=True,
        help=(
            "Complete map folders reserved for validation, "
            "for example Town10HD_Opt"
        ),
    )

    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=95,
        help=(
            "JPEG quality used only when a source image is not "
            "already JPEG"
        ),
    )

    parser.add_argument(
        "--progress-every",
        type=int,
        default=1000,
    )

    args = parser.parse_args()

    input_root = Path(args.in_root).expanduser().resolve()
    output_root = Path(args.out_root).expanduser().resolve()
    validation_maps = set(args.val_maps)

    if not input_root.is_dir():
        raise SystemExit(
            f"Input directory does not exist: {input_root}"
        )

    if output_root.exists():
        raise SystemExit(
            f"Output already exists: {output_root}\n"
            "Remove it explicitly before rebuilding."
        )

    if not 1 <= args.jpeg_quality <= 100:
        raise SystemExit(
            "--jpeg-quality must be between 1 and 100"
        )

    pairs = find_pairs(input_root)

    if not pairs:
        raise SystemExit(
            "No RGB/roadline-mask pairs were found under:\n"
            f"  {input_root}"
        )

    discovered_maps = {
        town
        for town, _, _, _ in pairs
    }

    unknown_validation_maps = (
        validation_maps - discovered_maps
    )

    if unknown_validation_maps:
        raise SystemExit(
            "Validation maps were not found in the input dataset: "
            f"{sorted(unknown_validation_maps)}\n"
            f"Available maps: {sorted(discovered_maps)}"
        )

    print(f"[convert] Input root:       {input_root}")
    print(f"[convert] Output root:      {output_root}")
    print(f"[convert] Pairs found:      {len(pairs)}")
    print(
        f"[convert] Validation maps:  "
        f"{sorted(validation_maps)}"
    )
    print(
        f"[convert] Training maps:    "
        f"{sorted(discovered_maps - validation_maps)}"
    )

    for split in ("train", "val"):
        (output_root / "images" / split).mkdir(
            parents=True
        )
        (output_root / "labels" / split).mkdir(
            parents=True
        )

    manifest_path = output_root / "manifest.csv"

    fieldnames = [
        "split",
        "town",
        "run",
        "frame",
        "image",
        "label",
        "source_image",
        "source_mask",
        "width",
        "height",
        "lane_pixels",
        "background_pixels",
        "lane_fraction",
        "image_storage",
    ]

    split_counts = {
        "train": 0,
        "val": 0,
    }

    split_lane_pixels = {
        "train": 0,
        "val": 0,
    }

    split_background_pixels = {
        "train": 0,
        "val": 0,
    }

    empty_masks = {
        "train": 0,
        "val": 0,
    }

    with manifest_path.open(
        "w",
        newline="",
        encoding="utf-8",
    ) as manifest_file:
        writer = csv.DictWriter(
            manifest_file,
            fieldnames=fieldnames,
        )
        writer.writeheader()

        for index, (
            town,
            run_name,
            image_path,
            mask_path,
        ) in enumerate(pairs, start=1):
            split = (
                "val"
                if town in validation_maps
                else "train"
            )

            image = cv2.imread(
                str(image_path),
                cv2.IMREAD_COLOR,
            )

            mask = cv2.imread(
                str(mask_path),
                cv2.IMREAD_UNCHANGED,
            )

            if image is None:
                raise RuntimeError(
                    f"Unreadable RGB image: {image_path}"
                )

            if mask is None:
                raise RuntimeError(
                    f"Unreadable mask: {mask_path}"
                )

            if mask.ndim == 3:
                mask = mask[:, :, 0]

            source_values = set(
                int(value)
                for value in np.unique(mask)
            )

            if not source_values.issubset(
                VALID_MASK_VALUES
            ):
                raise RuntimeError(
                    "Unexpected values in road-line mask:\n"
                    f"  mask: {mask_path}\n"
                    f"  values: {sorted(source_values)}\n"
                    "Expected only 0, 1, and/or 255.\n"
                    "Do not pass semantic_labels to this script."
                )

            if image.shape[:2] != mask.shape[:2]:
                raise RuntimeError(
                    "Image-mask dimension mismatch:\n"
                    f"  image: {image_path} "
                    f"{image.shape[:2]}\n"
                    f"  mask:  {mask_path} "
                    f"{mask.shape[:2]}"
                )

            # Correct MMSegmentation class IDs:
            #
            #   0 = background
            #   1 = lane marking
            #
            # The collector normally stores masks as 0/255.
            binary_mask = (
                mask > 0
            ).astype(np.uint8)

            output_stem = (
                f"{town}__{run_name}__"
                f"{image_path.stem}"
            )

            output_image_name = (
                f"{output_stem}.jpg"
            )
            output_label_name = (
                f"{output_stem}.png"
            )

            output_image_path = (
                output_root
                / "images"
                / split
                / output_image_name
            )

            output_label_path = (
                output_root
                / "labels"
                / split
                / output_label_name
            )

            image_storage = write_jpeg(
                source_path=image_path,
                source_image=image,
                destination_path=output_image_path,
                jpeg_quality=args.jpeg_quality,
            )

            if not cv2.imwrite(
                str(output_label_path),
                binary_mask,
            ):
                raise RuntimeError(
                    "Could not write binary label: "
                    f"{output_label_path}"
                )

            lane_pixels = int(
                np.count_nonzero(binary_mask)
            )
            total_pixels = int(binary_mask.size)
            background_pixels = (
                total_pixels - lane_pixels
            )
            lane_fraction = (
                lane_pixels / total_pixels
            )

            split_counts[split] += 1
            split_lane_pixels[split] += lane_pixels
            split_background_pixels[
                split
            ] += background_pixels

            if lane_pixels == 0:
                empty_masks[split] += 1

            writer.writerow(
                {
                    "split": split,
                    "town": town,
                    "run": run_name,
                    "frame": image_path.stem,
                    "image": (
                        f"images/{split}/"
                        f"{output_image_name}"
                    ),
                    "label": (
                        f"labels/{split}/"
                        f"{output_label_name}"
                    ),
                    "source_image": str(image_path),
                    "source_mask": str(mask_path),
                    "width": image.shape[1],
                    "height": image.shape[0],
                    "lane_pixels": lane_pixels,
                    "background_pixels": (
                        background_pixels
                    ),
                    "lane_fraction": (
                        f"{lane_fraction:.8f}"
                    ),
                    "image_storage": image_storage,
                }
            )

            if (
                args.progress_every > 0
                and index % args.progress_every == 0
            ):
                print(
                    f"[convert] {index}/{len(pairs)} "
                    f"train={split_counts['train']} "
                    f"val={split_counts['val']}"
                )

    print()
    print("[convert] Completed successfully")

    for split in ("train", "val"):
        total_pixels = (
            split_lane_pixels[split]
            + split_background_pixels[split]
        )

        lane_fraction = (
            split_lane_pixels[split]
            / total_pixels
            if total_pixels
            else 0.0
        )

        print()
        print(f"[{split}]")
        print(
            f"  pairs:              "
            f"{split_counts[split]}"
        )
        print(
            f"  empty lane masks:   "
            f"{empty_masks[split]}"
        )
        print(
            f"  lane pixels:        "
            f"{split_lane_pixels[split]}"
        )
        print(
            f"  background pixels:  "
            f"{split_background_pixels[split]}"
        )
        print(
            f"  lane fraction:      "
            f"{100.0 * lane_fraction:.4f}%"
        )

        if split_lane_pixels[split] > 0:
            raw_ratio = (
                split_background_pixels[split]
                / split_lane_pixels[split]
            )
            print(
                f"  background/lane:    "
                f"{raw_ratio:.2f}:1"
            )

    print()
    print(f"Manifest: {manifest_path}")
    print()
    print("Label definition:")
    print("  0 = background")
    print("  1 = lane_marking")


if __name__ == "__main__":
    main()
