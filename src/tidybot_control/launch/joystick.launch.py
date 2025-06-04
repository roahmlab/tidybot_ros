from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    package_dir = get_package_share_directory('tidybot_control')
    joystick_node_path = os.path.join(package_dir, 'tidybot_control', 'joystick_node.py')

    joystick_node = Node(
        package='tidybot_control',
        executable='joystick_node',
        name='joystick_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_name': 'tidybot'
        }],
        remappings=[
            ('/cmd_vel', '/tidybot/cmd_vel')
        ]
    )

    ld.add_action(joystick_node)

    return ld