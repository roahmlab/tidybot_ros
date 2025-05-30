from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

import xacro
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    package_dir = get_package_share_directory('tidybot_description')
    urdf_path = os.path.join(package_dir, 'urdf', 'tidybot.xacro')

    robot_description_content = xacro.process_file(urdf_path).toxml()

    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': robot_description_content,
                                      }])

    ld.add_action(robot_state_publisher_node)
    return ld
