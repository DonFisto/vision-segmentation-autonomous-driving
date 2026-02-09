import argparse
from mmengine.config import Config
from mmengine.runner import Runner
from mmseg.utils import register_all_modules

def main():
    parser = argparse.ArgumentParser(description="Train a model from a config file")
    parser.add_argument("--config", required=True, help="Path to MMSeg config .py")
    parser.add_argument("--work-dir", default=None, help="Optional work_dir override")
    parser.add_argument("--seed", type=int, default=42, help="Random seed for reproducibility")
    args = parser.parse_args()
    # 1. Register MMSeg modules (CRITICAL)
    register_all_modules(init_default_scope=True)

    # 2. Load config
    cfg = Config.fromfile(args.config)

    # Optional overrides
    if args.work_dir:
        cfg.work_dir = args.work_dir
    if args.seed is not None:
        cfg.randomness = dict(seed=args.seed)

    # 3. Build runner
    runner = Runner.from_cfg(cfg)

    # 4. Start training
    runner.train()

if __name__ == '__main__':
    main()
