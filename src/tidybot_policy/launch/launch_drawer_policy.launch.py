"""
Launch file for the drawer task system.

This launches:
1. multi_stage_planner - Generic motion executor (action server)
2. drawer_policy - Task-specific policy server (service + action client)

Usage:
  ros2 launch tidybot_policy launch_drawer_policy.launch.py

Then call the service:
  ros2 service call /open_drawer_task tidybot_utils/srv/OpenDrawerTask \
    "{joint_type: 'prismatic', handle_pose: {...}, joint_axis: {...}, pull_amount: 0.2}"
"""

import os
import xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # 1. Generate URDF from xacro
    robot_description_path = get_package_share_directory("tidybot_description")
    doc = xacro.process_file(str(robot_description_path + "/urdf/tidybot.xacro"))
    urdf_xml = doc.toxml()
    outpath = Path(robot_description_path) / "urdf/tidybot.urdf"
    outpath.write_text(urdf_xml, encoding="utf-8")

    # 2. Load MoveIt Configs
    moveit_config = MoveItConfigsBuilder(
        "tidybot", package_name="tidybot_moveit_config"
    ).to_moveit_configs()

    return LaunchDescription([
        # Launch arg declarations
        DeclareLaunchArgument(
            'approach_distance', default_value='0.15',
            description='Distance to approach from (meters)'
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'start_recording_on_action', default_value='false',
            description='If true, start episode recording when an action is received and launch synchronized_recorder'
        ),

        # Multi-Stage Executor (generic motion executor)
        Node(
            package='tidybot_solver',
            executable='multi_stage_planner',
            name='multi_stage_planner',
            output='screen',
            parameters=[
                moveit_config.to_dict(),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'arm_group': 'gen3_7dof',
                    'tip_link': 'tool_frame',
                    'start_recording_on_action': LaunchConfiguration('start_recording_on_action'),
                }
            ]
        ),

        # Synchronized recorder (only when start_recording_on_action is true)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('tidybot_episode'),
                    'launch',
                    'synchronized_recorder.launch.py',
                )
            ),
            launch_arguments={
                'use_sim': LaunchConfiguration('use_sim_time'),
                'storage_uri': 'episode_bag',
                'fps': '30.0',
                'tactile_enabled': 'True',
                'cameras': '["arm","ext"]',
            }.items(),
            condition=IfCondition(LaunchConfiguration('start_recording_on_action')),
        ),

        # Drawer Policy (task-specific policy server)
        Node(
            package='tidybot_policy',
            executable='drawer_policy',
            name='drawer_policy',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'approach_distance': LaunchConfiguration('approach_distance'),
                'approach_duration': 3.0,
                'grasp_duration': 2.0,
                'pull_duration': 2.5,
                'gripper_duration': 1.0
            }]
        )
    ])
