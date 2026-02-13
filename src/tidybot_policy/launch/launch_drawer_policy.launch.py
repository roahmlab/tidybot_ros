"""
Launch file for the drawer task system.

This launches:
1. multi_stage_planner - Generic motion executor (action server)
2. drawer_policy - Task-specific policy server (service + action client)
3. sensor_data_recorder (optional) - Records contact force data

Usage:
  ros2 launch tidybot_policy launch_drawer_policy.launch.py

  # With sensor recording disabled:
  ros2 launch tidybot_policy launch_drawer_policy.launch.py record_sensor_data:=false

Then call the service:
  ros2 service call /open_drawer_task tidybot_utils/srv/OpenDrawerTask \
    "{joint_type: 'prismatic', handle_pose: {...}, joint_axis: {...}, pull_amount: 0.2}"
"""

import os
import xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # 1. Generate URDF from xacro
    robot_description_path = get_package_share_directory("tidybot_description")
    doc = xacro.process_file(str(robot_description_path + "/urdf/tidybot.xacro"))
    urdf_xml = doc.toxml()
    outpath = Path(robot_description_path) / "urdf/tidybot.urdf"
    outpath.write_text(urdf_xml, encoding="utf-8")

    # 2. Load MoveIt Configs with explicit URDF
    moveit_config = MoveItConfigsBuilder(
        "tidybot", package_name="tidybot_moveit_config"
    ).robot_description(
        file_path=str(outpath)
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
            'record_sensor_data', default_value='true',
            description='Enable sensor data recording during drawer tasks'
        ),
        DeclareLaunchArgument(
            'output_dir', default_value='./sensor_data',
            description='Directory for recorded sensor data CSV files'
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
                    'tip_link': 'tool_frame'
                }
            ]
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
                'approach_duration': 2.0,
                'grasp_duration': 2.0,
                'pull_duration': 5.0,
                'gripper_duration': 2.0,
                'wait_after_grasp_duration': 0.5,
                'record_sensor_data': LaunchConfiguration('record_sensor_data'),
            }]
        ),
        
        # Sensor Data Recorder (optional, controlled by drawer_policy)
        Node(
            condition=IfCondition(LaunchConfiguration('record_sensor_data')),
            package='tidybot_episode',
            executable='sensor_data_recorder',
            name='sensor_data_recorder',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'output_dir': LaunchConfiguration('output_dir'),
                'record_rate': 50.0,
            }]
        ),
    ])
