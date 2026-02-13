"""
Launch file for the drawer task system.

This launches:
1. multi_stage_planner - Generic motion executor (action server)
2. drawer_policy - Task-specific policy server (service + action client)
3. sensor_data_recorder (optional) - Records contact force data

Usage:
  # For Isaac Lab with Hand-E gripper (default):
  ros2 launch tidybot_policy launch_drawer_policy.launch.py use_sim:=isaac

  # For Isaac Lab with 2F-85 gripper:
  ros2 launch tidybot_policy launch_drawer_policy.launch.py use_sim:=isaac gripper_type:=2f85

  # For Gazebo (always 2F-85):
  ros2 launch tidybot_policy launch_drawer_policy.launch.py use_sim:=gazebo

  # For hardware (always 2F-85):
  ros2 launch tidybot_policy launch_drawer_policy.launch.py use_sim:=hardware

Then call the service:
  ros2 service call /open_drawer_task tidybot_utils/srv/OpenDrawerTask \\
    "{joint_type: 'prismatic', handle_pose: {...}, joint_axis: {...}, pull_amount: 0.2}"
"""

import os
import xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from moveit_configs_utils import MoveItConfigsBuilder


def _launch_nodes(context):
    """Build nodes after launch args are resolved."""
    use_sim = context.launch_configurations['use_sim']
    robot_description_path = get_package_share_directory("tidybot_description")

    if use_sim == 'isaac':
        # Isaac Lab: gripper_type selectable via argument
        gripper_type = context.launch_configurations.get('gripper_type', 'hande')
        if gripper_type == 'hande':
            urdf_filename = "tidybot_hande_isaac.urdf"
            srdf_file = "config/tidybot_hande.srdf"
            joint_limits_file = "config/joint_limits_hande.yaml"
        else:
            urdf_filename = "tidybot_2f_85_isaac.urdf"
            srdf_file = "config/tidybot.srdf"
            joint_limits_file = "config/joint_limits.yaml"
        outpath = str(Path(robot_description_path) / f"urdf/{urdf_filename}")
        moveit_config = MoveItConfigsBuilder(
            "tidybot", package_name="tidybot_moveit_config"
        ).robot_description(
            file_path=outpath
        ).robot_description_semantic(
            file_path=srdf_file
        ).joint_limits(
            file_path=joint_limits_file
        ).to_moveit_configs()
    else:
        # Gazebo / hardware: 2F-85 gripper, URDF from xacro
        gripper_type = '2f85'
        doc = xacro.process_file(str(robot_description_path + "/urdf/tidybot.xacro"))
        urdf_xml = doc.toxml()
        outpath = Path(robot_description_path) / "urdf/tidybot.urdf"
        outpath.write_text(urdf_xml, encoding="utf-8")
        moveit_config = MoveItConfigsBuilder(
            "tidybot", package_name="tidybot_moveit_config"
        ).robot_description(
            file_path=str(outpath)
        ).to_moveit_configs()

    use_sim_time = context.launch_configurations.get('use_sim_time', 'true')

    return [
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
                    'gripper_type': gripper_type,
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
                'approach_duration': 4.0,
                'grasp_duration': 4.0,
                'pull_duration': 5.0,
                'gripper_duration': 3.0,
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
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim', default_value='isaac',
            description='Simulation backend: isaac, gazebo, or hardware'
        ),
        DeclareLaunchArgument(
            'gripper_type', default_value='hande',
            description="Gripper type: 'hande' or '2f85' (only used when use_sim=isaac)"
        ),
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

        OpaqueFunction(function=_launch_nodes),
    ])
