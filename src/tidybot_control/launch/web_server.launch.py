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
    tidybot_moveit_pkg = FindPackageShare("tidybot_moveit_config")

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([tidybot_moveit_pkg, "launch", "demo.launch.py"])
        )
    )

    kinematics_yaml_path = os.path.join(
        get_package_share_directory('tidybot_moveit_config'),
        'config',
        'kinematics.yaml'
    )

    with open(kinematics_yaml_path, 'r') as f:
        kinematics_config = yaml.safe_load(f)

    web_server_publisher = Node(
        package="tidybot_control",
        executable="web_server_publisher",
        name="web_server_publisher",
        output="screen",
    )

    web_server_relay = Node(
        package="tidybot_control",
        executable="ws_relay",
        name="ws_relay",
        output="screen",
    )

    state_controller = Node(
        package="tidybot_control",
        executable="state_controller",
        name="state_controller",
        output="screen",
    )

    web_server_moveit = Node(
        package="tidybot_moveit_config",
        executable="web_server_moveit",
        name="web_server_moveit",
        output="screen",
        parameters=[{"robot_description_kinematics": kinematics_config}],
    )

    reset_env_server = Node(
        package="tidybot_control",
        executable="reset_env",
        name="reset_env_server",
        output="screen",
    )

    return LaunchDescription([
        moveit_launch,
        web_server_publisher,
        web_server_relay,
        state_controller,
        web_server_moveit,
        reset_env_server,
    ])
