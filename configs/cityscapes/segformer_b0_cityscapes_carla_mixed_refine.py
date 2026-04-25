_base_ = ['./segformer_b0_cityscapes.py']

load_from = '/home/danielmartinez/vision-segmentation-autonomous-driving/work_dirs/segformer_b0_cityscapes_carla_ft/iter_40000.pth'
work_dir = '/home/danielmartinez/vision-segmentation-autonomous-driving/work_dirs/segformer_b0_cityscapes_carla_mixed_refine'

carla_root = '/home/danielmartinez/datasets/carla_cityscapes19_filtered_pruned'
city_root = '/home/danielmartinez/datasets/cityscapes_subset_1200'

crop_size = (512, 1024)

metainfo = dict(
    classes=(
        'road', 'sidewalk', 'building', 'wall', 'fence', 'pole',
        'traffic light', 'traffic sign', 'vegetation', 'terrain',
        'sky', 'person', 'rider', 'car', 'truck', 'bus', 'train',
        'motorcycle', 'bicycle'
    ),
    palette=[
        [128, 64,128],
        [244, 35,232],
        [ 70, 70, 70],
        [102,102,156],
        [190,153,153],
        [153,153,153],
        [250,170, 30],
        [220,220,  0],
        [107,142, 35],
        [152,251,152],
        [ 70,130,180],
        [220, 20, 60],
        [255,  0,  0],
        [  0,  0,142],
        [  0,  0, 70],
        [  0, 60,100],
        [  0, 80,100],
        [  0,  0,230],
        [119, 11, 32]
    ]
)

train_pipeline = [
    dict(type='LoadImageFromFile'),
    dict(type='LoadAnnotations'),
    dict(type='RandomResize', scale=(2048, 1024), ratio_range=(0.5, 2.0), keep_ratio=True),
    dict(type='RandomCrop', crop_size=crop_size, cat_max_ratio=0.75),
    dict(type='RandomFlip', prob=0.5),
    dict(type='PackSegInputs'),
]

test_pipeline = [
    dict(type='LoadImageFromFile'),
    dict(type='Resize', scale=(2048, 1024), keep_ratio=True),
    dict(type='LoadAnnotations'),
    dict(type='PackSegInputs'),
]

# CARLA train
carla_train = dict(
    type='BaseSegDataset',
    data_root=carla_root,
    data_prefix=dict(img_path='images/train', seg_map_path='labels/train'),
    img_suffix='.png',
    seg_map_suffix='.png',
    metainfo=metainfo,
    pipeline=train_pipeline,
)

# Cityscapes train
city_train = dict(
    type='CityscapesDataset',
    data_root=city_root,
    data_prefix=dict(img_path='leftImg8bit/train', seg_map_path='gtFine/train'),
    metainfo=metainfo,
    pipeline=train_pipeline,
)

# Approximate 60/40 in favor of CARLA by repeating CARLA once
train_dataloader = dict(
    _delete_=True,
    batch_size=2,
    num_workers=4,
    sampler=dict(type='InfiniteSampler', shuffle=True),
    dataset=dict(
        type='ConcatDataset',
        datasets=[
            dict(type='RepeatDataset', times=2, dataset=carla_train),
            city_train,
        ]
    )
)

# Validate only on held-out CARLA map
val_dataloader = dict(
    _delete_=True,
    batch_size=1,
    num_workers=2,
    sampler=dict(type='DefaultSampler', shuffle=False),
    dataset=dict(
        type='BaseSegDataset',
        data_root=carla_root,
        data_prefix=dict(img_path='images/val', seg_map_path='labels/val'),
        img_suffix='.png',
        seg_map_suffix='.png',
        metainfo=metainfo,
        pipeline=test_pipeline,
    )
)

test_dataloader = dict(
    _delete_=True,
    batch_size=1,
    num_workers=2,
    sampler=dict(type='DefaultSampler', shuffle=False),
    dataset=dict(
        type='BaseSegDataset',
        data_root=carla_root,
        data_prefix=dict(img_path='images/val', seg_map_path='labels/val'),
        img_suffix='.png',
        seg_map_suffix='.png',
        metainfo=metainfo,
        pipeline=test_pipeline,
    )
)

val_evaluator = dict(type='IoUMetric', iou_metrics=['mIoU'])
test_evaluator = val_evaluator

optim_wrapper = dict(
    optimizer=dict(type='AdamW', lr=1e-5, weight_decay=0.01)
)

# Short refinement run
train_cfg = dict(type='IterBasedTrainLoop', max_iters=3000, val_interval=500)
val_cfg = dict(type='ValLoop')
test_cfg = dict(type='TestLoop')

default_hooks = dict(
    checkpoint=dict(type='CheckpointHook', by_epoch=False, interval=500),
    logger=dict(type='LoggerHook', interval=50),
)

param_scheduler = [
    dict(
        type='PolyLR',
        begin=0,
        end=3000,
        eta_min=1e-6,
        power=1.0,
        by_epoch=False,
    )
]
