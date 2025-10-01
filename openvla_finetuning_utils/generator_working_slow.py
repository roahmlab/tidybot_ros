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

class Kinova(tfds.core.GeneratorBasedBuilder):
    """DatasetBuilder for example dataset."""

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
                            shape=(480, 640, 3),
                            dtype=np.uint8,
                            encoding_format='png',
                            doc='Main camera RGB observation.',
                        ),
                        'wrist_image': tfds.features.Image(
                            shape=(480, 640, 3),
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
                        'state_ros': tfds.features.Tensor(
                                shape=(8,),
                                dtype=np.float32,
                                doc='Robot joint state from ROS: [7x joint positions, 1x gripper width].',
                        ),
                        # 'images': tfds.features.FeaturesDict({
                        #     'cam_exterior': tfds.features.Video(
                        #         shape=(None, 480, 640, 3),
                        #         dtype=np.uint8,
                        #         encoding_format='png',
                        #         doc='Exterior RGB camera video, 15 FPS, 480x640, no audio.',
                        #     ),
                        #     'cam_wrist': tfds.features.Video(
                        #         shape=(None, 480, 640, 3),
                        #         dtype=np.uint8,
                        #         encoding_format='png',
                        #         doc='Wrist RGB camera video, 15 FPS, 480x640, no audio.',
                        #     ),
                        # }),
                    }),
                    'action': tfds.features.Tensor(
                        shape=(8,),
                        dtype=np.float32,
                        doc='Robot action, consists of [7x joint velocities, '
                            '1x gripper velocities].',
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
        parquet_files = glob.glob(os.path.join(PARQUET_DIR, 'episode_*.parquet'))
        return {
            'train': self._generate_examples(parquet_files)
        }

    def _generate_examples(self, parquet_files: list) -> Iterator[Tuple[str, Any]]:
        # Load language instructions once (assuming one task here)
        with open('kinova/meta/tasks.jsonl', 'r') as f:
            tasks = [json.loads(line) for line in f]
        language_instruction = tasks[0]['task']

        for pq_file in parquet_files:
            
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
                frame = frame.to_ndarray(format='bgr24')
                ext_frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                exterior_frames.append(ext_frame_rgb)

            wrist_frames = []
            for i, frame in enumerate(wrist_container.decode(video=0)):
                frame = frame.to_ndarray(format='bgr24')
                wrist_frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                wrist_frames.append(wrist_frame_rgb)

            language_embedding = self._embed([language_instruction])[0].numpy()
        
            episode = []
            for i, row in data.iterrows():
                # Capture frame from frame_idx
                frame_idx = int(row['frame_index'])
                ext_frame_rgb = exterior_frames[frame_idx]
                wrist_frame_rgb = wrist_frames[frame_idx]

                step = {
                    'observation': {
                        'image': ext_frame_rgb,
                        'wrist_image': wrist_frame_rgb,
                        'state': row['observation.state'].astype(np.float32),
                        'state_ros': row['observation.state_ros'].astype(np.float32),
                    },
                    'action': row['action'].astype(np.float32),
                    'discount': float(row.get('discount', 1.0)),
                    'reward': float(row.get('reward', 0.0)),
                    'is_first': i == 0,
                    'is_last': i == len(data) - 1,
                    'is_terminal': i == len(data) - 1,
                    'language_instruction': language_instruction,
                    'language_embedding': language_embedding,
                    'timestamp': float(row['timestamp']),
                    'frame_index': int(row['frame_index']),
                    'episode_index': int(row['episode_index']),
                    'index': int(row['index']),
                    'task_index': int(row['task_index']),
                }
                episode.append(step)

            sample = {
                'steps': episode,
                'episode_metadata': {
                    'file_path': pq_file,
                }
            }

            yield pq_file, sample