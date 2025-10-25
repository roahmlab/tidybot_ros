#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'storage_uri',
            default_value='episode_bag',
            description='Base directory for storing episode data'
        ),
        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Whether to use simulation topics (true) or real robot topics (false)'
        ),
        DeclareLaunchArgument(
            'fps',
            default_value='10.0',
            description='Recording frequency in Hz'
        ),
        Node(
            package='tidybot_episode',
            executable='synchronized_recorder',
            name='synchronized_recorder',
            parameters=[{
                'storage_uri': LaunchConfiguration('storage_uri'),
                'use_sim': LaunchConfiguration('use_sim'),
                'use_sim_time': LaunchConfiguration('use_sim'),
                'fps': LaunchConfiguration('fps')
            }],
            output='screen',
            remappings=[
                ("/tf", "/tf_relay"),
                ("/tf_static", "/tf_static_relay"),
            ],
        )
    ])
