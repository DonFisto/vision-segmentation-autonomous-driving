_base_ = ['./segformer_b0_cityscapes.py']

load_from = '/home/danielmartinez/vision-segmentation-autonomous-driving/work_dirs/segformer_b0_cityscapes/iter_80000.pth'
work_dir = '/home/danielmartinez/vision-segmentation-autonomous-driving/work_dirs/segformer_b0_cityscapes_carla_ft'

dataset_type = 'BaseSegDataset'
data_root = '/home/danielmartinez/datasets/carla_cityscapes19'

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

train_dataloader = dict(
    batch_size=2,
    num_workers=4,
    sampler=dict(type='InfiniteSampler', shuffle=True),
    dataset=dict(
        type=dataset_type,
        data_root=data_root,
        data_prefix=dict(img_path='images/train', seg_map_path='labels/train'),
        img_suffix='.png',
        seg_map_suffix='.png',
        metainfo=metainfo,
        pipeline=[
            dict(type='LoadImageFromFile'),
            dict(type='LoadAnnotations'),
            dict(type='RandomResize', scale=(2048, 1024), ratio_range=(0.5, 2.0), keep_ratio=True),
            dict(type='RandomCrop', crop_size=(512, 1024), cat_max_ratio=0.75),
            dict(type='RandomFlip', prob=0.5),
            dict(type='PackSegInputs'),
        ]
    )
)

val_dataloader = dict(
    batch_size=1,
    num_workers=2,
    sampler=dict(type='DefaultSampler', shuffle=False),
    dataset=dict(
        type=dataset_type,
        data_root=data_root,
        data_prefix=dict(img_path='images/val', seg_map_path='labels/val'),
        img_suffix='.png',
        seg_map_suffix='.png',
        metainfo=metainfo,
        pipeline=[
            dict(type='LoadImageFromFile'),
            dict(type='Resize', scale=(2048, 1024), keep_ratio=True),
            dict(type='LoadAnnotations'),
            dict(type='PackSegInputs'),
        ]
    )
)

test_dataloader = val_dataloader

optim_wrapper = dict(
    optimizer=dict(type='AdamW', lr=6e-5, weight_decay=0.01)
)

train_cfg = dict(type='IterBasedTrainLoop', max_iters=40000, val_interval=2000)
val_cfg = dict(type='ValLoop')
test_cfg = dict(type='TestLoop')

default_hooks = dict(
    checkpoint=dict(type='CheckpointHook', by_epoch=False, interval=2000)
)
