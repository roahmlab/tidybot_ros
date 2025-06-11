from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_dir = get_package_share_directory('tidybot_control')
    joystick_node_path = os.path.join(package_dir, 'tidybot_control', 'joystick_node.py')
    joy_params = os.path.join(package_dir, 'config', 'joystick.yaml')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[joy_params],
    )
    
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_params],
    )

    twist_to_multiarray_node = Node(
        package='tidybot_control',
        executable='twist_to_multiarray',
        name='twist_to_multiarray',
    )

    return LaunchDescription([
        joy_node,
        teleop_node,
        twist_to_multiarray_node,
    ])