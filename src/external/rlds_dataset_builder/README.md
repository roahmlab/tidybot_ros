# Reinforcement Learning Dataset Generation with Open-X Embodiment Standard
To generate a tensorflow dataset with recorded episodes, first run
``` bash
ros2 run tidybot_episode rosbag_to_parquet
```
This reads the recorded messages from each rosbag and creates a parquet
file for each episode.

Then, refer to https://github.com/kpertsch/rlds_dataset_builder for converting
into the X-embodiment RLDS format. We have provided a generator.py script which
parses the parquet files and original video recordings. Make sure to copy the
outputted parquet files and generator script into a new dataset directory within
rlds_dataset_builder before running tfds build.

We use the generated tensorflow dataset to fine-tune the OpenVLA model in our
experiments. To register the dataset, we add the following configuration to 
configs.py:
``` python
"tidybot_vla": {
    "image_obs_keys": {"primary": "image", "secondary": None, "wrist": "wrist_image"},
    "depth_obs_keys": {"primary": None, "secondary": None, "wrist": None},
    "state_obs_keys": ["state"],
    "state_encoding": StateEncoding.POS_QUAT,
    "action_encoding": ActionEncoding.EEF_POS,
},
```

We also add the following transform which follows the Open-X Embodiment standard in transforms.py:
``` python
def tidybot_vla_dataset_transform(trajectory: Dict[str, Any]) -> Dict[str, Any]:
    trajectory["observation"]["EEF_state"] = trajectory["observation"]["state"][:, :7]
    trajectory["observation"]["gripper_state"] = trajectory["observation"]["state"][:, -1:]
    return trajectory
```