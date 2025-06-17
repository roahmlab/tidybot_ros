import os
import yaml

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    kinematics_yaml_path = os.path.join(
        get_package_share_directory('tidybot_moveit_config'),
        'config',
        'kinematics.yaml'
    )

    with open(kinematics_yaml_path, 'r') as f:
        kinematics_config = yaml.safe_load(f)

    web_server_subscriber = Node(
        package="tidybot_moveit_config",
        executable="web_server_subscriber",
        name="web_server_subscriber",
        output="screen",
        parameters=[{"robot_description_kinematics": kinematics_config}],
    )

    return LaunchDescription([
        web_server_subscriber
    ])
