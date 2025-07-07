from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node

import xacro
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    package_dir = get_package_share_directory('tidybot_description')

    robot_description_content = DeclareLaunchArgument(
        'robot_description',
        default_value=Command([
                    'xacro ',
                    PathJoinSubstitution([package_dir, 'urdf', 'tidybot.xacro']), ' '
                ]),
        description='Path to the robot description file (Xacro format)'
    )
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    ignore_timestamp = DeclareLaunchArgument(
        'ignore_timestamp',
        default_value='false',
        description='Ignore timestamp in robot state publisher if true'
    )

    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': LaunchConfiguration('robot_description'),
                                          'use_sim_time': LaunchConfiguration('use_sim_time'),
                                          'ignore_timestamp': LaunchConfiguration('ignore_timestamp')
                                      }],)

    ld.add_action(use_sim_time)
    ld.add_action(robot_description_content)
    ld.add_action(robot_state_publisher_node)
    return ld

