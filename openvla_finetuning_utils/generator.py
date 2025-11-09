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
from rosbags.highlevel import AnyReader
import pandas as pd

import csv

EPISODES_DIR = 'episode_bag'
episode_dirs = sorted(glob.glob(os.path.join(EPISODES_DIR, "episode_*")))
num_episodes = len(episode_dirs)

PARQUET_DIR = f"tidybot_vla_{num_episodes}"
os.makedirs(PARQUET_DIR, exist_ok=True)

ACTIONS_DIR = 'actions'
OBSERVATIONS_DIR = 'observations'

class TidybotVLA(tfds.core.GeneratorBasedBuilder):
    """DatasetBuilder for tidybot recorded episodes."""

    VERSION = tfds.core.Version('1.0.0')
    RELEASE_NOTES = {
      '1.0.0': 'Initial release.',
    }

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

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

    def _convert_rosbag_to_parquet(self, episode_dir: str) -> str:
        """Convert actions + observations rosbags into one Parquet file."""
        obs_bag = os.path.join(episode_dir, "observations")
        act_bag = os.path.join(episode_dir, "actions")

        episode_id = os.path.basename(episode_dir)
        output_parquet = os.path.join(PARQUET_DIR, f"{episode_id}.parquet")

        obs_states = []
        timestamps = []

        # ---- Read observations (arm_pose + gripper_state) ----
        with AnyReader([obs_bag]) as reader:
            arm_states, gripper_states = [], []

            for conn, timestamp, rawdata in reader.messages():
                if '/arm_pose' in conn.topic:
                    msg = reader.deserialize(rawdata, conn.msgtype)
                    obs_pose = msg.pose
                    joint_state = self.inverse_kinematics([
                        obs_pose.position.x,
                        obs_pose.position.y,
                        obs_pose.position.z,
                        obs_pose.orientation.x,
                        obs_pose.orientation.y,
                        obs_pose.orientation.z,
                        obs_pose.orientation.w,
                    ])
                    arm_states.append(joint_state)
                    timestamps.append(timestamp * 1e-9)
                elif '/gripper_state' in conn.topic:
                    msg = reader.deserialize(rawdata, conn.msgtype)
                    gripper_states.append(msg.data)

            if len(arm_states) != len(gripper_states):
                raise ValueError(f"Observed message count mismatch: {episode_id}")

            obs_states = [np.concatenate([np.array(joints, dtype=np.float32), [float(g)]]) 
                        for joints, g in zip(arm_states, gripper_states)]

        # ---- Read actions (target_pose + gripper_command) ----
        with AnyReader([act_bag]) as reader:
            arm_target_poses, gripper_cmds = [], []

            for conn, timestamp, rawdata in reader.messages():
                if '/tidybot/arm/target_pose' in conn.topic:
                    msg = reader.deserialize(rawdata, conn.msgtype)
                    pose = msg  # geometry_msgs/msg/Pose
                    arm_target_poses.append([
                        pose.position.x, pose.position.y, pose.position.z,
                        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
                    ])
                elif '/tidybot/gripper/commands' in conn.topic:
                    msg = reader.deserialize(rawdata, conn.msgtype)
                    gripper_cmds.append(float(msg.data))

            if len(arm_target_poses) != len(gripper_cmds):
                raise ValueError(f"Action message count mismatch: {episode_id}")

        arm_target_poses = np.array(arm_target_poses, dtype=np.float32)  # shape (T, 7)
        gripper_cmds = np.array(gripper_cmds, dtype=np.float32)

        # Convert quaternions → RPY for delta computation
        def quat_to_rpy(qx, qy, qz, qw):
            R = pin.Quaternion(qw, qx, qy, qz).toRotationMatrix()
            return pin.rpy.matrixToRpy(R)

        eef_rpy = np.array([quat_to_rpy(*q[3:]) for q in arm_target_poses])
        eef_xyz = arm_target_poses[:, :3]

        # Compute deltas
        dpos = np.diff(eef_xyz, axis=0, prepend=eef_xyz[[0]])          # (T, 3)
        drpy = np.diff(eef_rpy, axis=0, prepend=eef_rpy[[0]])          # (T, 3)

        eef_deltas = np.concatenate([dpos, drpy, gripper_cmds[:, None]], axis=1).astype(np.float32)  # (T, 7)

        n = len(obs_states)
        rows = []
        for i in range(n):
            rows.append({
                "timestamp": timestamps[i],
                "frame_index": i,
                "episode_index": i,
                "task_index": 0,
                "observation.state": obs_states[i],
                "observation.state_ros": obs_states[i],
                "action": eef_deltas[i],
                "reward": 0.0,
                "discount": 1.0,
                "index": i,
            })

        df = pd.DataFrame(rows)
        df.to_parquet(output_parquet, index=False)
        print(f"[INFO] Wrote {output_parquet} ({len(df)} steps)")
        return output_parquet
    
    def _split_generators(self, dl_manager: tfds.download.DownloadManager):
        episode_dirs = sorted(glob.glob(os.path.join(EPISODES_DIR, 'episode_*')))
        parquet_files = []

        for episode_dir in episode_dirs:
            pq_path = self._convert_rosbag_to_parquet(episode_dir)
            if pq_path is not None:
                parquet_files.append(pq_path)

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
    
    _model = None
    _data = None
    _fk_frame_id = None

    @classmethod
    def inverse_kinematics(cls, pose: np.ndarray) -> np.ndarray:
        if cls._model is None:
            ee_frame_name = "end_effector_link"
            urdf_path = os.path.expanduser("~/gen3_robotiq_2f_85.urdf")
            cls._model = pin.buildModelFromUrdf(urdf_path)
            cls._data = cls._model.createData()
            cls._fk_frame_id = cls._model.getFrameId(ee_frame_name)
            if cls._fk_frame_id == len(cls._model.frames):
                raise ValueError(f"End-effector frame '{ee_frame_name}' not found in URDF.")
            cls._ik_solver = pin.ik.HybridSolver(cls._model, ee_frame_name)

        x, y, z, qx, qy, qz, qw = pose[:7]
        target = pin.SE3(pin.Quaternion(qw, qx, qy, qz).toRotationMatrix(), np.array([x, y, z]))
        q_init = np.zeros(cls._model.nq)
        q_sol, success = cls._ik_solver.solve(q_init, target)
        if not success:
            raise RuntimeError("IK solver failed to converge for EEF pose")
        
        return q_sol
        
    def _generate_examples(self, parquet_files: list) -> Iterator[Tuple[str, Any]]:
        # Load language instructions once (assuming one task here)
        with open(os.path.join(EPISODES_DIR, 'tasks.jsonl')) as f:
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
                exterior_container = av.open(os.path.join(EPISODES_DIR, episode_id, "external_camera.mp4"))
                wrist_container = av.open(os.path.join(EPISODES_DIR, episode_id, "arm_camera.mp4"))

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
                    obs_state = row['observation.state'].astype(np.float32)
                    obs_state_ros = row['observation.state_ros'].astype(np.float32)
                    action = row['action'].astype(np.float32)

                    step = { 
                        'observation': { 
                            'image': ext_frame_rgb, 
                            'wrist_image': wrist_frame_rgb, 
                            'state': obs_state, 
                            'state_ros': obs_state_ros, 
                        }, 
                        'action': action, 
                        'discount': float(row.get('discount', 1.0)), 
                        'reward': float(row.get('reward', 0.0)), 
                        'is_first': i == 0, 
                        'is_last': i == len(data) - 1, 
                        'is_terminal': i == len(data) - 1, 
                        'language_instruction': language_instruction, 
                        'language_embedding': language_instruction, # no language embedding
                        'timestamp': float(row['timestamp']), 
                        'frame_index': int(row['frame_index']), 
                        'episode_index': int(row['episode_index']), 
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