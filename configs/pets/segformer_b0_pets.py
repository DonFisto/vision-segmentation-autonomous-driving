# configs/pets/segformer_b0_pets.py
# Minimal Oxford Pets binary semantic segmentation config for MMSeg 1.2.2

# ----- 1) Dataset basics -----
dataset_type = 'BaseSegDataset'
data_root = 'datasets/oxford-iiit-pet-mmseg'  # relative to project root

# Classes for binary segmentation
metainfo = dict(
    classes=('background', 'pet'),
    palette=[[0, 0, 0], [255, 0, 0]]
)

# Image crop size (SegFormer default style)
crop_size = (512, 512)

# ----- 2) Data processing pipelines -----
train_pipeline = [
    dict(type='LoadImageFromFile'),
    dict(type='LoadAnnotations'),
    dict(type='RandomResize', scale=(512, 512), ratio_range=(0.5, 2.0), keep_ratio=True),
    dict(type='RandomCrop', crop_size=crop_size, cat_max_ratio=0.95),
    dict(type='RandomFlip', prob=0.5),
    dict(type='PackSegInputs')
]

test_pipeline = [
    dict(type='LoadImageFromFile'),
    dict(type='Resize', scale=(512, 512), keep_ratio=True),
    dict(type='LoadAnnotations'),  # optional, used for validation metrics
    dict(type='PackSegInputs')
]

# ----- 3) Dataloaders -----
train_dataloader = dict(
    batch_size=8,
    num_workers=4,
    persistent_workers=True,
    sampler=dict(type='InfiniteSampler', shuffle=True),
    dataset=dict(
        type='BaseSegDataset',
        data_root=data_root,
        metainfo=metainfo,
        data_prefix=dict(
            img_path='img_dir/train',
            seg_map_path='ann_dir/train'
        ),
        img_suffix='.jpg',        # change to .png if needed
        seg_map_suffix='.png',
        pipeline=train_pipeline
    )
)


val_dataloader = dict(
    batch_size=1,
    num_workers=2,
    persistent_workers=True,
    sampler=dict(type='DefaultSampler', shuffle=False),
    dataset=dict(
        type='BaseSegDataset',
        data_root=data_root,
        metainfo=metainfo,
        data_prefix=dict(
            img_path='img_dir/val',
            seg_map_path='ann_dir/val'
        ),
        img_suffix='.jpg',        # change to .png if needed
        seg_map_suffix='.png',
        pipeline=test_pipeline
    )
)

test_dataloader = val_dataloader

# ----- 4) Evaluator -----
val_evaluator = dict(type='IoUMetric', iou_metrics=['mIoU'])
test_evaluator = val_evaluator

# ----- 5) Model -----
# SegFormer-B0 backbone + SegFormerHead decode head
# num_classes=2 for binary segmentation
model = dict(
    type='EncoderDecoder',
    data_preprocessor=dict(
        type='SegDataPreProcessor',
        mean=[123.675, 116.28, 103.53],
        std=[58.395, 57.12, 57.375],
        bgr_to_rgb=True,
        pad_val=0,
        seg_pad_val=255,
        size=crop_size,
        size_divisor=None
    ),
    backbone=dict(
        type='MixVisionTransformer',
        in_channels=3,
        embed_dims=32,
        num_stages=4,
        num_layers=[2, 2, 2, 2],
        num_heads=[1, 2, 5, 8],
        patch_sizes=[7, 3, 3, 3],
        sr_ratios=[8, 4, 2, 1],
        out_indices=(0, 1, 2, 3),
        mlp_ratio=4,
        qkv_bias=True,
        drop_rate=0.0,
        attn_drop_rate=0.0,
        drop_path_rate=0.1
    ),
    decode_head=dict(
        type='SegformerHead',
        in_channels=[32, 64, 160, 256],
        in_index=[0, 1, 2, 3],
        channels=256,
        dropout_ratio=0.1,
        num_classes=2,
        norm_cfg=dict(type='BN', requires_grad=True),
        align_corners=False,
        loss_decode=dict(type='CrossEntropyLoss', use_sigmoid=False, loss_weight=1.0)
    ),
    # Auxiliary head optional; keep off for minimal setup
    train_cfg=dict(),
    test_cfg=dict(mode='whole')
)

# ----- 6) Optimization -----
optim_wrapper = dict(
    type='OptimWrapper',
    optimizer=dict(type='AdamW', lr=6e-4, weight_decay=0.01)
)

# Learning rate schedule (simple)
param_scheduler = [
    dict(type='PolyLR', eta_min=1e-5, power=1.0, begin=0, end=2000, by_epoch=False)
]

# ----- 7) Training loop -----
train_cfg = dict(type='IterBasedTrainLoop', max_iters=2000, val_interval=200)
val_cfg = dict(type='ValLoop')
test_cfg = dict(type='TestLoop')

# ----- 8) Runtime / logging / checkpoints -----
default_scope = 'mmseg'

default_hooks = dict(
    timer=dict(type='IterTimerHook'),
    logger=dict(type='LoggerHook', interval=50),
    param_scheduler=dict(type='ParamSchedulerHook'),
    checkpoint=dict(type='CheckpointHook', interval=200, by_epoch=False, max_keep_ckpts=3),
    sampler_seed=dict(type='DistSamplerSeedHook')
)

# Where outputs (logs/checkpoints) go
work_dir = './work_dirs/segformer_b0_pets'

# Mixed precision (optional); disable if you want maximum stability
# fp16 = dict(loss_scale='dynamic')

# Randomness
randomness = dict(seed=0)
