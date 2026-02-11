# SegFormer-B0 on Cityscapes (evaluation / inference ready)
# MMSegmentation 1.x compatible

# -------------------------------------------------
# 1. Dataset
# -------------------------------------------------
dataset_type = 'CityscapesDataset'
data_root = 'datasets/cityscapes'

crop_size = (512, 1024)

# Cityscapes has 19 semantic classes
num_classes = 19

# -------------------------------------------------
# 2. Pipelines
# -------------------------------------------------
train_pipeline = [
    dict(type='LoadImageFromFile'),
    dict(type='LoadAnnotations'),
    dict(
        type='RandomResize',
        scale=(2048, 1024),
        ratio_range=(0.5, 2.0),
        keep_ratio=True
    ),
    dict(type='RandomCrop', crop_size=crop_size, cat_max_ratio=0.75),
    dict(type='RandomFlip', prob=0.5),
    dict(type='PackSegInputs')
]

test_pipeline = [
    dict(type='LoadImageFromFile'),
    dict(type='Resize', scale=(2048, 1024), keep_ratio=True),
    dict(type='LoadAnnotations'),  # needed for evaluation
    dict(type='PackSegInputs')
]

# -------------------------------------------------
# 3. Dataloaders
# -------------------------------------------------
train_dataloader = dict(
    batch_size=2,
    num_workers=4,
    sampler=dict(type='InfiniteSampler', shuffle=True),
    dataset=dict(
        type=dataset_type,
        data_root=data_root,
        data_prefix=dict(
            img_path='leftImg8bit/train',
            seg_map_path='gtFine/train'
        ),
        pipeline=train_pipeline
    )
)

val_dataloader = dict(
    batch_size=1,
    num_workers=2,
    sampler=dict(type='DefaultSampler', shuffle=False),
    dataset=dict(
        type=dataset_type,
        data_root=data_root,
        data_prefix=dict(
            img_path='leftImg8bit/val',
            seg_map_path='gtFine/val'
        ),
        pipeline=test_pipeline
    )
)

test_dataloader = val_dataloader

# -------------------------------------------------
# 4. Evaluation
# -------------------------------------------------
val_evaluator = dict(
    type='IoUMetric',
    iou_metrics=['mIoU']
)
test_evaluator = val_evaluator

# -------------------------------------------------
# 5. Model
# -------------------------------------------------
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
        num_classes=num_classes,
        norm_cfg=dict(type='BN', requires_grad=True),
        align_corners=False,
        loss_decode=dict(
            type='CrossEntropyLoss',
            use_sigmoid=False,
            loss_weight=1.0
        )
    ),
    train_cfg=dict(),
    test_cfg=dict(mode='whole')
)

# -------------------------------------------------
# 6. Optimization (inactive for eval-only)
# -------------------------------------------------
optim_wrapper = dict(
    type='OptimWrapper',
    optimizer=dict(type='AdamW', lr=6e-4, weight_decay=0.01)
)

param_scheduler = [
    dict(
        type='PolyLR',
        eta_min=1e-5,
        power=1.0,
        begin=0,
        end=80000,
        by_epoch=False
    )
]

# -------------------------------------------------
# 7. Runtime
# -------------------------------------------------
train_cfg = dict(type='IterBasedTrainLoop', max_iters=80000, val_interval=4000)
val_cfg = dict(type='ValLoop')
test_cfg = dict(type='TestLoop')

default_scope = 'mmseg'

default_hooks = dict(
    timer=dict(type='IterTimerHook'),
    logger=dict(type='LoggerHook', interval=50),
    param_scheduler=dict(type='ParamSchedulerHook'),
    checkpoint=dict(type='CheckpointHook', interval=4000, by_epoch=False),
    sampler_seed=dict(type='DistSamplerSeedHook')
)

work_dir = './work_dirs/segformer_b0_cityscapes'
randomness = dict(seed=0)


# -------------------------------------------------
# 8. Pretrained initialization
# -------------------------------------------------
load_from = 'work_dirs/segformer_b0_cityscapes/segformer_mit-b0_8x1_1024x1024_160k_cityscapes_20211208_101857-e7f88502.pth'