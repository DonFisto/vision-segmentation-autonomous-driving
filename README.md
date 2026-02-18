# vision-segmentation-autonomous-driving

Semantic segmentation toolkit (MMSegmentation) + CARLA/ROS2 integration.

## Repository layout
- `configs/` MMSeg configs (pets, cityscapes)
- `scripts/` training/inference/eval utilities
- `ros/ros2_ws/` ROS2 workspace (nodes: CARLA bridge, control, segmentation, ASCII cam)
- `docs/` setup + architecture + usage notes

## Quickstart (MMSeg)
Create env (example):
- `environment.yml` / `requirements*.txt` are provided; pick one workflow and stick to it.

Example inference:
```bash
python scripts/infer_trained.py --config <cfg.py> --checkpoint <ckpt.pth> --img <img_or_dir> --out-dir out/infer
```

Example evaluation:
```bash
python scripts/eval_mmseg.py --config <cfg.py> --checkpoint <ckpt.pth> --mode val
```


### `docs/project/repo_map.md`

```bash
cat > docs/project/repo_map.md <<'EOF'
```
# Repo map

## configs/
- `pets/` Oxford-IIIT Pet binary segmentation
- `cityscapes/` Cityscapes semantic segmentation

## scripts/
- `convert_pets_to_mmseg.py` dataset conversion (Pets -> MMSeg format)
- `train_from_cfg.py` training entrypoint (MMEngine Runner)
- `eval_mmseg.py` eval entrypoint (val/test)
- `infer_trained.py` inference + visualization (single image or directory)

## ros/ros2_ws/
ROS2 workspace that contains:
- CARLA RGB publisher/bridge
- Control node (terminal-driven)
- Semantic segmentation node (publishes mask + overlay)
- ASCII camera visualization node
