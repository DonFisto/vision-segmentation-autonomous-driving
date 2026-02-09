from pathlib import Path
import numpy as np
from PIL import Image
import torch

from mmengine.config import Config
from mmengine.dataset import Compose
from mmseg.utils import register_all_modules
register_all_modules(init_default_scope=True)
from mmseg.registry import MODELS
from mmcv.transforms import Compose

def main():
    cfg_path = 'configs/pets/segformer_b0_pets.py'
    img_dir = Path('datasets/oxford-iiit-pet-mmseg/img_dir/val')  # ajusta si tu ruta es distinta
    out_dir = Path('out/untrained_manual')
    out_dir.mkdir(parents=True, exist_ok=True)

    cfg = Config.fromfile(cfg_path)

    # 1) Construir el modelo desde el config (pesos aleatorios)
    model = MODELS.build(cfg.model)
    model.init_weights()  # deja explícito que es "untrained"
    model.cuda()
    model.eval()

    # 2) Pipeline de test/val (carga imagen, resize, pack)
    # En tu config lo llamamos test_pipeline; si no existe, usa el de val_dataloader.
    if 'test_pipeline' in cfg:
        pipeline_cfg = cfg.test_pipeline
    else:
        pipeline_cfg = cfg.val_dataloader.dataset.pipeline
    # Remove transforms that require ground-truth segmentation
    pipeline_cfg = [t for t in pipeline_cfg if t.get('type') not in ['LoadAnnotations','LoadSegAnnotations']]
    pipeline = Compose(pipeline_cfg)

    # 3) Inferencia imagen por imagen y guardado de máscara
    img_paths = sorted([p for p in img_dir.iterdir() if p.suffix.lower() in ['.jpg', '.png', '.jpeg']])
    if not img_paths:
        raise RuntimeError(f'No images found in {img_dir}')

    print(f'Found {len(img_paths)} images. Saving masks to {out_dir}/')

    with torch.no_grad():
        for i, p in enumerate(img_paths):
            data = dict(img_path=str(p))
            data = pipeline(data)  # crea dict con 'inputs' y 'data_samples'

            # El modelo espera batch: inputs [B,C,H,W] y data_samples como lista
            batch = dict(
                inputs=data['inputs'].unsqueeze(0).cuda(),
                data_samples=[data['data_samples']]
            )

            # test_step devuelve una lista de SegDataSample
            outputs = model.test_step(batch)
            ds = outputs[0]

            mask = ds.pred_sem_seg.data.cpu().numpy()

            # Make sure it's 2D (H, W)
            mask = np.squeeze(mask)

            # If it still isn’t 2D, take the last 2 dims as (H, W)
            if mask.ndim != 2:
                mask = mask.reshape(mask.shape[-2], mask.shape[-1])

            mask = mask.astype(np.uint8)
            Image.fromarray(mask, mode='L').save(out_dir / f'pred_{i:04d}.png')


            if i % 25 == 0:
                print(f'  [{i}/{len(img_paths)}] {p.name}')

    print('Done.')
    print(f'Check: {out_dir}/')


if __name__ == '__main__':
    main()
