from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_dir = get_package_share_directory('tidybot_control')
    solver_pkg_dir = get_package_share_directory('tidybot_solver')
    joy_params = os.path.join(package_dir, 'config', 'joystick.yaml')
    moveit_config = MoveItConfigsBuilder("tidybot", package_name="tidybot_moveit_config").to_moveit_configs()
    moveit_config.robot_description["use_sim_time"] = True

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[joy_params],
    )
    
    joystick_controller_node = Node(
        package='tidybot_control',
        executable='joystick_controller',
        name='joystick_controller',
        parameters=[{'use_sim_time': True}],
    )

    joystick_to_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([solver_pkg_dir, "launch", "moveit_servo_twist.launch.py"])
        )
    )

    move_group_launch = generate_move_group_launch(moveit_config)

    return LaunchDescription([
        joy_node,
        joystick_controller_node,
        joystick_to_moveit,
        move_group_launch
    ])