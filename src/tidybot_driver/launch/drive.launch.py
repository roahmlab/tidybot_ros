from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition, AndCondition, OrCondition

def generate_launch_description():

    mode = DeclareLaunchArgument(
        "mode", 
        default_value="full", 
        description="Control mode: full, arm_only, base_only"
    )

    arm_server = Node(
        package="tidybot_driver",
        executable="arm_server",
        name="arm_server",
        condition=IfCondition(OrCondition([
            IfCondition(LaunchConfiguration("mode") == "full"),
            IfCondition(LaunchConfiguration("mode") == "arm_only"),
        ]))
    )

    base_server = Node(
        package="tidybot_driver",
        executable="base_server",
        name="base_server",
        condition=IfCondition(OrCondition([
            IfCondition(LaunchConfiguration("mode") == "full"),
            IfCondition(LaunchConfiguration("mode") == "base_only"),
        ])),
        prefix=['sudo', 'chrt', '-f', '80']
    )

    return LaunchDescription([
        mode,
        arm_server,
        base_server
    ])