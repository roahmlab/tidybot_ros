from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
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

    moveit_servo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                solver_pkg_dir,
                'launch',
                'joystick_twist.launch.py',
            )
        ),
    )
    return LaunchDescription([
        joy_node,
        joystick_controller_node,
        moveit_servo,
    ])