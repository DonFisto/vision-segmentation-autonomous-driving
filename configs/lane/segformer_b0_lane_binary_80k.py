_base_ = ["../cityscapes/segformer_b0_cityscapes.py"]

load_from = (
    "/home/danielmartinez/"
    "vision-segmentation-autonomous-driving/"
    "work_dirs/pretrained/"
    "segformer_b0_vruheavy_without_classifier.pth"
)

work_dir = (
    "/home/danielmartinez/"
    "vision-segmentation-autonomous-driving/"
    "work_dirs/segformer_b0_lane_binary_80k"
)

data_root = "/home/danielmartinez/datasets/carla_lane_binary"

# CARLA source images are 800x600. This retains their 4:3 geometry.
crop_size = (576, 768)

metainfo = dict(
    classes=("background", "lane_marking"),
    palette=[
        [0, 0, 0],
        [255, 255, 255],
    ],
)

model = dict(
    data_preprocessor=dict(
        size=crop_size,
        size_divisor=None,
        seg_pad_val=255,
    ),
    decode_head=dict(
        num_classes=2,
        out_channels=2,
        loss_decode=dict(
            type="CrossEntropyLoss",
            use_sigmoid=False,
            class_weight=[1.0, 10.0],
            loss_weight=1.0,
        ),
    ),
)

train_pipeline = [
    dict(type="LoadImageFromFile"),
    dict(type="LoadAnnotations"),
    dict(
        type="Resize",
        scale=(768, 576),
        keep_ratio=False,
    ),
    dict(
        type="RandomFlip",
        prob=0.5,
        direction="horizontal",
    ),
    dict(type="PhotoMetricDistortion"),
    dict(type="PackSegInputs"),
]

test_pipeline = [
    dict(type="LoadImageFromFile"),
    dict(
        type="Resize",
        scale=(768, 576),
        keep_ratio=False,
    ),
    dict(type="LoadAnnotations"),
    dict(type="PackSegInputs"),
]

train_dataloader = dict(
    _delete_=True,
    batch_size=2,
    num_workers=4,
    persistent_workers=True,
    sampler=dict(
        type="InfiniteSampler",
        shuffle=True,
    ),
    dataset=dict(
        type="BaseSegDataset",
        data_root=data_root,
        data_prefix=dict(
            img_path="images/train",
            seg_map_path="labels/train",
        ),
        img_suffix=".jpg",
        seg_map_suffix=".png",
        reduce_zero_label=False,
        ignore_index=255,
        metainfo=metainfo,
        pipeline=train_pipeline,
    ),
)

val_dataloader = dict(
    _delete_=True,
    batch_size=1,
    num_workers=2,
    persistent_workers=True,
    sampler=dict(
        type="DefaultSampler",
        shuffle=False,
    ),
    dataset=dict(
        type="BaseSegDataset",
        data_root=data_root,
        data_prefix=dict(
            img_path="images/val",
            seg_map_path="labels/val",
        ),
        img_suffix=".jpg",
        seg_map_suffix=".png",
        reduce_zero_label=False,
        ignore_index=255,
        metainfo=metainfo,
        pipeline=test_pipeline,
    ),
)

test_dataloader = val_dataloader

val_evaluator = dict(
    type="IoUMetric",
    iou_metrics=[
        "mIoU",
        "mDice",
        "mFscore",
    ],
    beta=1,
)

test_evaluator = val_evaluator

optim_wrapper = dict(
    _delete_=True,
    type="OptimWrapper",
    accumulative_counts=2,
    optimizer=dict(
        type="AdamW",
        lr=6e-5,
        weight_decay=0.01,
    ),
    paramwise_cfg=dict(
        custom_keys={
            "backbone": dict(lr_mult=0.1),
            "decode_head": dict(lr_mult=1.0),
        }
    ),
)

train_cfg = dict(
    type="IterBasedTrainLoop",
    max_iters=80000,
    val_interval=10000,
)

val_cfg = dict(type="ValLoop")
test_cfg = dict(type="TestLoop")

default_hooks = dict(
    checkpoint=dict(
        type="CheckpointHook",
        by_epoch=False,
        interval=10000,
        max_keep_ckpts=5,
        save_best="mDice",
        rule="greater",
    ),
    logger=dict(
        type="LoggerHook",
        interval=50,
    ),
)

param_scheduler = [
    dict(
        type="LinearLR",
        start_factor=0.1,
        begin=0,
        end=1000,
        by_epoch=False,
    ),
    dict(
        type="PolyLR",
        begin=1000,
        end=80000,
        eta_min=1e-6,
        power=1.0,
        by_epoch=False,
    ),
]

randomness = dict(seed=42)
