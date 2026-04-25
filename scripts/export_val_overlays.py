import os
import cv2
import numpy as np
from mmengine.config import Config
from mmseg.apis import init_model, inference_model

CONFIG = "/home/danielmartinez/vision-segmentation-autonomous-driving/configs/cityscapes/segformer_b0_cityscapes_carla_ft.py"
CHECKPOINT = "/home/danielmartinez/vision-segmentation-autonomous-driving/work_dirs/segformer_b0_cityscapes_carla_ft/iter_38000.pth"

VAL_IMAGES = "/home/danielmartinez/datasets/carla_cityscapes19/images/val"
OUT_DIR = "/home/danielmartinez/val_overlays"

os.makedirs(OUT_DIR, exist_ok=True)

print("Loading model...")
model = init_model(CONFIG, CHECKPOINT, device="cuda:0")

files = sorted(os.listdir(VAL_IMAGES))[:100]   # export first 100

for f in files:
    path = os.path.join(VAL_IMAGES, f)

    result = inference_model(model, path)

    pred = result.pred_sem_seg.data[0].cpu().numpy()

    img = cv2.imread(path)

    color = model.dataset_meta["palette"]

    overlay = np.zeros_like(img)

    for cls_id, c in enumerate(color):
        overlay[pred == cls_id] = c

    vis = cv2.addWeighted(img, 0.5, overlay, 0.5, 0)

    cv2.imwrite(os.path.join(OUT_DIR, f), vis)

print("DONE → overlays saved in", OUT_DIR)
