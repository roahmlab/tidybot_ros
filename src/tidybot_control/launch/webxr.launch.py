from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('tidybot_control')

    ws_publisher = Node(
        package='tidybot_control',
        executable='web_server_publisher',
        name='ws_publisher',
    )

    ws_relay = Node(
        package='tidybot_control',
        executable='ws_relay',
        name='ws_relay',
    )

    state_controller = Node(
        package='tidybot_control',
        executable='state_controller',
        name='state_controller',
    )

    return LaunchDescription([
        ws_publisher,
        ws_relay,
        state_controller
    ])