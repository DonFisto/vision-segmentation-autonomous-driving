#!/usr/bin/env python3
"""
Evaluate an MMSegmentation model from config + checkpoint.

Works with MMSegmentation (mmseg 1.x) + MMEngine Runner.

Examples:
  # Evaluate on validation split (uses val_dataloader/val_evaluator from config)
  python eval_mmseg.py \
    --config configs/pets/segformer_b0_pets.py \
    --checkpoint work_dirs/segformer_b0_pets/iter_2000.pth \
    --mode val

  # Evaluate on test split (uses test_dataloader/test_evaluator from config, if present)
  python eval_mmseg.py \
    --config configs/pets/segformer_b0_pets.py \
    --checkpoint work_dirs/segformer_b0_pets/iter_2000.pth \
    --mode test

  # Force CPU and save metrics to a JSON file
  python eval_mmseg.py \
    --config configs/pets/segformer_b0_pets.py \
    --checkpoint work_dirs/segformer_b0_pets/iter_2000.pth \
    --device cpu \
    --out-json out/eval_metrics.json
"""

import argparse
import json
from pathlib import Path

from mmengine.config import Config
from mmengine.runner import Runner
from mmseg.utils import register_all_modules

def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, help="Path to MMSeg config .py")
    parser.add_argument("--checkpoint", required=True, help="Path to checkpoint .pth")
    parser.add_argument("--mode", choices=["val", "test"], default="val", help="Which loop to run")
    parser.add_argument("--device", default=None, help="e.g. cuda:0 or cpu (optional override)")
    parser.add_argument("--work-dir", default=None, help="Optional work_dir override")
    parser.add_argument("--out-json", default=None, help="Optional path to write metrics JSON")
    args = parser.parse_args()

    register_all_modules(init_default_scope=True)
    cfg = Config.fromfile(args.config)

    # Load checkpoint
    cfg.load_from = args.checkpoint

    # Avoid distributed launch unless your config explicitly sets it
    # (for typical single-GPU local evaluation)
    cfg.launcher = "none"

    # Optional overrides
    if args.work_dir:
        cfg.work_dir = args.work_dir
    else:
        # keep outputs away from training folder unless you want them there
        cfg.work_dir = str(Path("out") / "eval" / Path(args.config).stem)

    if args.device is not None:
        # MMEngine Runner reads cfg.device in many OpenMMLab projects
        cfg.device = args.device

    # Build and run
    runner = Runner.from_cfg(cfg)

    if args.mode == "test":
        if not hasattr(cfg, "test_dataloader") or not hasattr(cfg, "test_evaluator"):
            raise RuntimeError(
                "Config has no test_dataloader/test_evaluator. "
                "Use --mode val or add test_* sections to the config."
            )
        metrics = runner.test()
    else:
        if not hasattr(cfg, "val_dataloader") or not hasattr(cfg, "val_evaluator"):
            raise RuntimeError(
                "Config has no val_dataloader/val_evaluator. "
                "Add them to the config or use --mode test if you have test_*."
            )
        metrics = runner.val()

    # Print metrics nicely
    print("\n=== Metrics ===")
    if isinstance(metrics, dict):
        for k, v in metrics.items():
            print(f"{k}: {v}")
    else:
        print(metrics)

    # Optionally save
    if args.out_json:
        out_path = Path(args.out_json)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        with out_path.open("w", encoding="utf-8") as f:
            json.dump(metrics, f, indent=2, ensure_ascii=False)
        print(f"\nSaved metrics to: {out_path}")


if __name__ == "__main__":
    main()
