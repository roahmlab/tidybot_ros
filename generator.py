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

SHA256 fail:
pip cache purge
pip install --upgrade pip-tools

torchrun --standalone --nnodes 1 --nproc-per-node 2 vla-scripts/finetune.py \
  --vla_path "openvla/openvla-7b" \
  --data_root_dir ~/tensorflow_datasets/ \
  --dataset_name kinova_99 \
  --run_root_dir ~/openvla_finetune_logs/ \
  --adapter_tmp_dir ~/openvla_finetune_adapter_tmp/ \
  --lora_rank 32 \
  --batch_size 4 \
  --grad_accumulation_steps 4 \
  --learning_rate 5e-4 \
  --image_aug False \
  --wandb_project tidybot-openvla \
  --wandb_entity yuandi-huang-university-of-michigan \
  --save_steps 2000 

/home/yuandi/tensorflow_datasets/dataset_statistics_aef531d2c5a8775d45398cc8cd6fc62f042c82e55b7ade3b35a4c010aa2a24a7.json

conda install pytorch==2.2.0 torchvision==0.17.0 torchaudio==2.2.0 pytorch-cuda=12.1 -c pytorch -c nvidia

conda install pytorch=2.3.1 torchvision=0.18.1 torchaudio=2.3.1 pytorch-cuda=12.1 \
  -c pytorch -c nvidia # USE THIS

pip install torch==2.2.0+cu121 torchvision==0.17.0+cu121 torchaudio==2.2.0+cu121 -f https://download.pytorch.org/whl/torch_stable.html

# change nvidia driver:
sudo ln -sf /usr/lib/x86_64-linux-gnu/libnvidia-ml.so.535.183.01 /usr/lib/x86_64-linux-gnu/libnvidia-ml.so.1

  liblapack-3.9.0-24_linux64_openblas
  libopenblas-0.3.27-pthreads_hac2b453_1
  libosqp-0.6.3-h59595ed_0
  libqdldl-0.1.5-h27087fc_1
  libscotch-7.0.4-h2fe6a88_5
  libspral-2024.05.08-h1b93dcb_1
  libsqlite-3.42.0-h2797004_0
  libstdcxx-ng-13.1.0-hfd8a6a1_0
  libxml2-2.12.7-hc051c1a_1
  libzlib-1.2.13-hd590300_5
  metis-5.1.0-h59595ed_1007
  mumps-include-5.7.2-ha770c72_0
  mumps-seq-5.7.2-h6e8dedb_0
  ncurses-6.4-hcb278e6_0
  numpy-1.26.4-py39h474f0d3_0
  octomap-1.9.8-h924138e_0
  openssl-1.1.1w-hd590300_0
  pinocchio-3.1.0-py39h69fb9f6_1
  pip-23.2.1-pyhd8ed1ab_0
  proxsuite-0.6.7-py39h74842e3_0
  python-3.9.0-hffdb5ce_5_cpython
  python_abi-3.9-8_cp39
  qhull-2020.2-h434a139_5
  qhull-static-2020.2-h434a139_5
  readline-8.2-h8228510_1
  scipy-1.13.1-py39haf93ffa_0
  setuptools-68.0.0-pyhd8ed1ab_0
  simde-0.8.2-h84d6215_0
  sqlite-3.42.0-h2c6b66d_0
  tinyxml2-10.0.0-h59595ed_0
  tk-8.6.12-h27826a3_0
  tzdata-2023c-h71feb2d_0
  unixodbc-2.3.12-h661eb56_0
  urdfdom-4.0.0-hee28ff1_1
  urdfdom_headers-1.1.1-h00ab1b0_0
  wheel-0.41.0-pyhd8ed1ab_0
  xz-5.2.6-h166bdaf_0
  zlib-1.2.13-hd590300_5
  zstd-1.5.6-ha6fb4c9_0

*** CORRUPTED RECORD CAUSED BY TF-DATASETS MISMATCH: pip install --upgrade "tensorflow-datasets==4.9.2"

*** torch has no attribute uint64: pip install safetensors==0.3.3
*** numpy has no attribute _core: pip install accelerate==0.25.0

"kinova_99": { 
   "image_obs_keys": {"primary": "image", "secondary": None, "wrist": "wrist_image"}, 
   "depth_obs_keys": {"primary": None, "secondary": None, "wrist": None}, 
   "state_obs_keys": ["state", "state_ros"], 
   "state_encoding": StateEncoding.JOINT, 
   "action_encoding": ActionEncoding.EEF_POS, 
},

Traceback (most recent call last):                                                                                                                                                                                 
  File "/home/yuandi/tidybot_platform/src/tidybot_openvla/openvla/vla-scripts/finetune.py", line 373, in <module>                                                                                                  
    finetune()
  File "/home/yuandi/miniforge3/envs/openvla-3.10/lib/python3.10/site-packages/draccus/argparsing.py", line 203, in wrapper_inner
    response = fn(cfg, *args, **kwargs)
  File "/home/yuandi/tidybot_platform/src/tidybot_openvla/openvla/vla-scripts/finetune.py", line 253, in finetune
    for batch_idx, batch in enumerate(dataloader):
  File "/home/yuandi/miniforge3/envs/openvla-3.10/lib/python3.10/site-packages/torch/utils/data/dataloader.py", line 631, in __next__
    data = self._next_data()
  File "/home/yuandi/miniforge3/envs/openvla-3.10/lib/python3.10/site-packages/torch/utils/data/dataloader.py", line 675, in _next_data
    data = self._dataset_fetcher.fetch(index)  # may raise StopIteration
  File "/home/yuandi/miniforge3/envs/openvla-3.10/lib/python3.10/site-packages/torch/utils/data/_utils/fetch.py", line 32, in fetch
    data.append(next(self.dataset_iter))
  File "/home/yuandi/tidybot_platform/src/tidybot_openvla/openvla/prismatic/vla/datasets/datasets.py", line 146, in __iter__
    for rlds_batch in self.dataset.as_numpy_iterator():
  File "/home/yuandi/miniforge3/envs/openvla-3.10/lib/python3.10/site-packages/tensorflow/python/data/ops/dataset_ops.py", line 4733, in __next__
    return nest.map_structure(to_numpy, next(self._iterator))
  File "/home/yuandi/miniforge3/envs/openvla-3.10/lib/python3.10/site-packages/tensorflow/python/data/ops/iterator_ops.py", line 810, in __next__
    return self._next_internal()
  File "/home/yuandi/miniforge3/envs/openvla-3.10/lib/python3.10/site-packages/tensorflow/python/data/ops/iterator_ops.py", line 773, in _next_internal
    ret = gen_dataset_ops.iterator_get_next(
  File "/home/yuandi/miniforge3/envs/openvla-3.10/lib/python3.10/site-packages/tensorflow/python/ops/gen_dataset_ops.py", line 3029, in iterator_get_next
    _ops.raise_from_not_ok_status(e, name)
  File "/home/yuandi/miniforge3/envs/openvla-3.10/lib/python3.10/site-packages/tensorflow/python/framework/ops.py", line 5883, in raise_from_not_ok_status
    raise core._status_to_exception(e) from None  # pylint: disable=protected-access
tensorflow.python.framework.errors_impl.InvalidArgumentError: {{function_node __wrapped__IteratorGetNext_output_types_14_device_/job:localhost/replica:0/task:0/device:CPU:0}} Invalid PNG data, size 44449
         [[{{function_node cond_1_false_1481}}{{node cond_1/decode_image/DecodeImage}}]] [Op:IteratorGetNext] name: 
[2025-08-06 16:56:36,592] torch.distributed.elastic.multiprocessing.api: [WARNING] Sending process 8941 closing signal SIGTERM
[2025-08-06 16:56:36,592] torch.distributed.elastic.multiprocessing.api: [WARNING] Sending process 8943 closing signal SIGTERM
