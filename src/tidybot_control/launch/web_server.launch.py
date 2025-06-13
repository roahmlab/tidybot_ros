from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    tidybot_moveit_pkg = FindPackageShare("tidybot_moveit_config")

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([tidybot_moveit_pkg, "launch", "demo.launch.py"])
        )
    )

    web_server_relay = Node(
        package="tidybot_control",
        executable="web_server_relay",
        name="web_server_relay",
        output="screen",
    )

    web_server_node = Node(
        package="tidybot_control",
        executable="web_server_subscriber",
        name="web_server_subscriber",
        output="screen",
    )

    return LaunchDescription([
        moveit_launch,
        web_server_node,
        web_server_relay,
    ])
