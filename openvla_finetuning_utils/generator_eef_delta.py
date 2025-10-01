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
import pinocchio as pin

import csv

DATA_ROOT = 'kinova_small'
PARQUET_DIR = os.path.join(DATA_ROOT, 'data/chunk-000')
META_DIR = os.path.join(DATA_ROOT, 'meta')
VIDEOS_DIR = os.path.join(DATA_ROOT, 'videos/chunk-000')
EXTERN_CAMERA_DIR = os.path.join(VIDEOS_DIR, 'observation.images.cam_exterior')
WRIST_CAMERA_DIR = os.path.join(VIDEOS_DIR, 'observation.images.cam_wrist')

class KinovaSmall(tfds.core.GeneratorBasedBuilder):
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
                        'state_ros': tfds.features.Tensor(
                                shape=(8,),
                                dtype=np.float32,
                                doc='Robot joint state from ROS: [7x joint positions, 1x gripper width].',
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
        parquet_files = glob.glob(os.path.join(PARQUET_DIR, 'episode_*.parquet'))
        return {
            'train': self._generate_examples(parquet_files)
        }
    
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
    
    _model = None
    _data = None
    _fk_frame_id = None

    @classmethod
    def compute_eef_deltas_from_joint_positions(cls, actions: np.ndarray) -> np.ndarray:
        if cls._model is None:
            ee_frame_name = "end_effector_link"
            urdf_path = os.path.expanduser("~/gen3_robotiq_2f_85.urdf")
            cls._model = pin.buildModelFromUrdf(urdf_path)
            cls._data = cls._model.createData()
            cls._fk_frame_id = cls._model.getFrameId(ee_frame_name)
            if cls._fk_frame_id == len(cls._model.frames):
                raise ValueError(f"End-effector frame '{ee_frame_name}' not found in URDF.")

        def compute_eef_pose(joint_pos: np.ndarray) -> np.ndarray:
            pin.forwardKinematics(cls._model, cls._data, joint_pos)
            pin.updateFramePlacements(cls._model, cls._data)
            pose = cls._data.oMf[cls._fk_frame_id]
            return np.concatenate([pose.translation, pin.rpy.matrixToRpy(pose.rotation)])
        
        T = actions.shape[0]
        eef_poses = np.stack([compute_eef_pose(j[:7]) for j in actions], axis=0)
        eef_deltas = np.zeros((T, 7), dtype=np.float32)
        eef_deltas[1:, :6] = eef_poses[1:] - eef_poses[:-1]
        eef_deltas[:, 6] = actions[:, 7]
        return eef_deltas
    
    @classmethod
    def compute_eef_poses_from_joint_positions(cls, actions: np.ndarray) -> np.ndarray:
        if cls._model is None:
            ee_frame_name = "end_effector_link"
            urdf_path = os.path.expanduser("~/gen3_robotiq_2f_85.urdf")
            cls._model = pin.buildModelFromUrdf(urdf_path)
            cls._data = cls._model.createData()
            cls._fk_frame_id = cls._model.getFrameId(ee_frame_name)
            if cls._fk_frame_id == len(cls._model.frames):
                raise ValueError(f"End-effector frame '{ee_frame_name}' not found in URDF.")

        def compute_eef_pose(joint_pos: np.ndarray) -> np.ndarray:
            pin.forwardKinematics(cls._model, cls._data, joint_pos)
            pin.updateFramePlacements(cls._model, cls._data)
            pose = cls._data.oMf[cls._fk_frame_id]
            return np.concatenate([pose.translation, pin.rpy.matrixToRpy(pose.rotation)])
        
        T = actions.shape[0]
        eef_poses = np.stack([compute_eef_pose(j[:7]) for j in actions], axis=0)
        output = np.zeros((T, 7), dtype=np.float32)
        output[:, :6] = eef_poses
        output[:, 6] = actions[:, 7]
        return output
    
    def _generate_examples(self, parquet_files: list) -> Iterator[Tuple[str, Any]]:
        # Load language instructions once (assuming one task here)
        with open(os.path.join(META_DIR, 'tasks.jsonl')) as f:
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
                exterior_container = av.open(os.path.join(EXTERN_CAMERA_DIR, f"{episode_id}.mp4"))
                wrist_container = av.open(os.path.join(WRIST_CAMERA_DIR, f"{episode_id}.mp4"))

                # Build frame lists
                exterior_frames = []
                for i, frame in enumerate(exterior_container.decode(video=0)): 
                    frame = frame.to_ndarray(format='bgr24') 
                    frame = self.crop_and_resize_single_image(image=frame, crop_scale=0.75)
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

                    step = { 
                        'observation': { 
                            'image': ext_frame_rgb, 
                            'wrist_image': wrist_frame_rgb, 
                            'state': np.concatenate([ 
                                np.deg2rad(row['observation.state'][:7].astype(np.float32)), 
                                row['observation.state'][7:].astype(np.float32) 
                            ]), 
                            'state_ros': np.concatenate([ 
                                np.deg2rad(row['observation.state_ros'][:7].astype(np.float32)), 
                                row['observation.state_ros'][7:].astype(np.float32) 
                            ]), 
                        }, 
                        'action': np.concatenate([ 
                            np.deg2rad(row['action'][:7].astype(np.float32)), 
                            row['action'][7:].astype(np.float32) 
                        ]), 
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
                
                all_joint_actions = np.stack([step['action'] for step in episode], axis=0)  # (T, 8)
                eef_deltas = self.compute_eef_poses_from_joint_positions(all_joint_actions)  # (T, 7)

                with open("eef_actions.csv", "w", newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(["dx", "dy", "dz", "droll", "dpitch", "dyaw", "gripper"])
                    for step, eef_action in zip(episode, eef_deltas):
                        step["action"] = eef_action.astype(np.float32)
                        writer.writerow(step["action"])

                sample = { 
                    'steps': episode, 
                    'episode_metadata': { 
                        'file_path': pq_file, 
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