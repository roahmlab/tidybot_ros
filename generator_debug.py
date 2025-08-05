from typing import Iterator, Tuple, Any

import os, glob
import json
import numpy as np
import tensorflow as tf
import tensorflow_datasets as tfds
import tensorflow_hub as hub
import pyarrow.parquet as pq
import av
import cv2

DATA_ROOT = 'kinova'
PARQUET_DIR = os.path.join(DATA_ROOT, 'data/chunk-000')
META_DIR = os.path.join(DATA_ROOT, 'meta')
VIDEOS_DIR = os.path.join(DATA_ROOT, 'videos/chunk-000')
EXTERN_CAMERA_DIR = os.path.join(VIDEOS_DIR, 'observation.images.cam_exterior')
WRIST_CAMERA_DIR = os.path.join(VIDEOS_DIR, 'observation.images.cam_wrist')


def _parse_example(pq_file):
    # Load parquet table and convert to pandas
    table = pq.read_table(pq_file)
    data = table.to_pandas()
    
    # Extract episode id from filename
    episode_id = os.path.splitext(os.path.basename(pq_file))[0]
    
    # Open video files once per episode
    exterior_container = av.open(os.path.join(EXTERN_CAMERA_DIR, f"{episode_id}.mp4"))
    wrist_container = av.open(os.path.join(WRIST_CAMERA_DIR, f"{episode_id}.mp4"))
            
    # Build frame lists
    exterior_frames = []
    for i, frame in enumerate(exterior_container.decode(video=0)):
        print(i)
        frame = frame.to_ndarray(format='bgr24')
        ext_frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        exterior_frames.append(ext_frame_rgb)
    
    wrist_frames = []
    for i, frame in enumerate(wrist_container.decode(video=0)):
        frame = frame.to_ndarray(format='bgr24')
        wrist_frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        wrist_frames.append(wrist_frame_rgb)

    return pq_file

parquet_files = glob.glob(os.path.join(PARQUET_DIR, 'episode_000099.parquet'))
_parse_example(parquet_files[0])