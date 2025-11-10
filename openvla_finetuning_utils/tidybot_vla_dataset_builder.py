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
import pandas as pd

import csv

ROSBAG_DIR = '../../../../episode_bag'
EPISODES_DIR = 'data.parquet'
episode_dirs = sorted(glob.glob(os.path.join(EPISODES_DIR, "episode_*")))
num_episodes = len(episode_dirs)

class TidybotVLA(tfds.core.GeneratorBasedBuilder):
    """DatasetBuilder for tidybot recorded episodes."""

    VERSION = tfds.core.Version('1.0.0')
    RELEASE_NOTES = {
      '1.0.0': 'Initial release.',
    }

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._embed = hub.load("https://tfhub.dev/google/universal-sentence-encoder-large/5")

    def _info(self) -> tfds.core.DatasetInfo:
        """Dataset metadata (homepage, citation,...)."""
        return self.dataset_info_from_configs(
            features=tfds.features.FeaturesDict({
                'steps': tfds.features.Dataset({
                    'observation': tfds.features.FeaturesDict({
                        'image': tfds.features.Image(
                            shape=(224, 224, 3),
                            dtype=np.uint8,
                            encoding_format='png',
                            doc='Main camera RGB observation.',
                        ),
                        'wrist_image': tfds.features.Image(
                            shape=(224, 224, 3),
                            dtype=np.uint8,
                            encoding_format='png',
                            doc='Wrist camera RGB observation.',
                        ),
                        'state': tfds.features.Tensor(
                            shape=(8,),
                            dtype=np.float32,
                            doc='Robot state, consists of [7x robot joint angles, '
                                '1x gripper position].',
                        ),
                    }),
                    'action': tfds.features.Tensor(
                        shape=(7,),
                        dtype=np.float32,
                        doc='Robot action, consists of 6x EE delta, '
                            '1x gripper position].',
                    ),
                    'discount': tfds.features.Scalar(
                        dtype=np.float32,
                        doc='Discount if provided, default to 1.'
                    ),
                    'reward': tfds.features.Scalar(
                        dtype=np.float32,
                        doc='Reward if provided, 1 on final step for demos.'
                    ),
                    'is_first': tfds.features.Scalar(
                        dtype=np.bool_,
                        doc='True on first step of the episode.'
                    ),
                    'is_last': tfds.features.Scalar(
                        dtype=np.bool_,
                        doc='True on last step of the episode.'
                    ),
                    'is_terminal': tfds.features.Scalar(
                        dtype=np.bool_,
                        doc='True on last step of the episode if it is a terminal step, True for demos.'
                    ),
                    'language_instruction': tfds.features.Text(
                        doc='Language Instruction.'
                    ),
                    'language_embedding': tfds.features.Tensor(
                        shape=(512,),
                        dtype=np.float32,
                        doc='Kona language embedding. '
                            'See https://tfhub.dev/google/universal-sentence-encoder-large/5'
                    ),
                    'timestamp': tfds.features.Scalar(
                        dtype=np.float32,
                        doc='Timestamp in seconds since epoch.',
                    ),
                    'frame_index': tfds.features.Scalar(
                        dtype=np.int64,
                        doc='Frame index within the episode.',
                    ),
                    'episode_index': tfds.features.Scalar(
                        dtype=np.int64,
                        doc='Episode index in dataset.',
                    ),
                    'index': tfds.features.Scalar(
                        dtype=np.int64,
                        doc='Global step index.',
                    ),
                    'task_index': tfds.features.Scalar(
                        dtype=np.int64,
                        doc='Task index (useful if dataset includes multiple tasks).',
                    ),
                }),
                'episode_metadata': tfds.features.FeaturesDict({
                    'file_path': tfds.features.Text(
                        doc='Path to the original data file.'
                    ),
                }),
            }))
    
    def _split_generators(self, dl_manager: tfds.download.DownloadManager):
        parquet_files = sorted(glob.glob(os.path.join(EPISODES_DIR, 'episode_*')))
        return {'train': self._generate_examples(parquet_files)}
        
    @staticmethod
    def crop_and_resize_single_image(image: np.ndarray, crop_scale: float, output_size=(224, 224)) -> np.ndarray:
        """
        Center-crops a single image to have area `crop_scale` * (original image area),
        then resizes to `output_size`.

        Args:
            image: np.ndarray of shape (H, W, C) with dtype=np.uint8 or np.float32, range [0,255] or [0,1]
            crop_scale: float in (0,1] indicating what fraction of the original image area to crop.
            output_size: tuple (H, W) to resize cropped image to.

        Returns:
            Cropped and resized image as np.ndarray of shape (output_size[0], output_size[1], C)
        """
        assert 0 < crop_scale <= 1.0, "crop_scale must be in (0, 1]"
        assert image.ndim == 3 and image.shape[2] == 3, "Input must be (H, W, 3)"

        H, W, _ = image.shape
        orig_area = H * W
        target_area = crop_scale * orig_area
        crop_side = int(np.sqrt(target_area * W / H))  # preserve aspect ratio

        crop_height = min(crop_side, H)
        crop_width = min(int(crop_side * H / W), W)

        y1 = (H - crop_height) // 2
        x1 = W - crop_width  # crop out left side
        y2 = y1 + crop_height
        x2 = W  # keep right edge

        cropped = image[y1:y2, x1:x2]
        resized = cv2.resize(cropped, output_size, interpolation=cv2.INTER_AREA)

        return resized

    def _generate_examples(self, parquet_files: list) -> Iterator[Tuple[str, Any]]:
        # Load language instructions once (assuming one task here)
        with open(os.path.join(ROSBAG_DIR, 'tasks.jsonl')) as f:
            tasks = [json.loads(line) for line in f]
        language_instruction = tasks[0]['task']

        def _parse_example(pq_file):
            try:
                # Load parquet table and convert to pandas
                table = pq.read_table(pq_file)
                data = table.to_pandas()

                # Extract episode id from filename
                episode_id = os.path.splitext(os.path.basename(pq_file))[0]

                # Open video files once per episode
                exterior_container = av.open(os.path.join(ROSBAG_DIR, episode_id, "ext_camera.mp4"))
                wrist_container = av.open(os.path.join(ROSBAG_DIR, episode_id, "arm_camera.mp4"))

                # Build frame lists
                exterior_frames = []
                for i, frame in enumerate(exterior_container.decode(video=0)): 
                    frame = frame.to_ndarray(format='bgr24') 
                    frame = self.crop_and_resize_single_image(image=frame, crop_scale=1)
                    ext_frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) 
                    exterior_frames.append(ext_frame_rgb) 
                 
                wrist_frames = [] 
                for i, frame in enumerate(wrist_container.decode(video=0)): 
                    frame = frame.to_ndarray(format='bgr24') 
                    frame = self.crop_and_resize_single_image(image=frame, crop_scale=0.75)
                    wrist_frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) 
                    wrist_frames.append(wrist_frame_rgb) 
              
                language_embedding = self._embed([language_instruction])[0].numpy() 

                episode = [] 
                for i, row in data.iterrows(): 
                    # Capture frame from frame_idx 
                    frame_idx = int(row['frame_index']) 
                    ext_frame_rgb = exterior_frames[frame_idx] 
                    wrist_frame_rgb = wrist_frames[frame_idx] 
                    obs_state = row['observation.state'].astype(np.float32)
                    action = row['action'].astype(np.float32)

                    step = { 
                        'observation': { 
                            'image': ext_frame_rgb, 
                            'wrist_image': wrist_frame_rgb, 
                            'state': obs_state, # EEF XYZ (3) + Quaternion (4) + Gripper Open/Close (1) 
                        }, 
                        'action': action, # EEF Delta XYZ (3) + Roll-Pitch-Yaw (3) + Gripper Open/Close (1)
                        'discount': float(row.get('discount', 1.0)), 
                        'reward': float(row.get('reward', 0.0)), 
                        'is_first': i == 0, 
                        'is_last': i == len(data) - 1, 
                        'is_terminal': i == len(data) - 1, 
                        'language_instruction': language_instruction, 
                        'language_embedding': language_embedding, # embedding not necessary for OpenVLA
                        'timestamp': float(row['timestamp']), 
                        'frame_index': int(row['frame_index']), 
                        'episode_index': int(row['index']), 
                        'index': int(row['index']), 
                        'task_index': int(row['task_index']), 
                    } 
                    episode.append(step) 
                
                # Save actions for debugging
                with open("eef_actions.csv", "w", newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(["dx", "dy", "dz", "droll", "dpitch", "dyaw", "gripper"])
                    for step in episode:
                        writer.writerow(step["action"])

                sample = {
                    "steps": episode,
                    "episode_metadata": {
                        "file_path": pq_file,
                    },
                }

                return pq_file, sample 
             
            except Exception as e:     
                print(f"[DEBUG] Exception type: {type(e)}, message: {e}")
                import traceback; traceback.print_exc()
                return None
  
        for pq_file in parquet_files: 
            result = _parse_example(pq_file) 
            if result is not None: 
                yield result 
 
        # beam = tfds.core.lazy_imports.apache_beam 
        # return ( 
        #         beam.Create(parquet_files) 
        #         | beam.Map(_parse_example) 
        #         | beam.Filter(lambda x: x is not None) 
        # ) 