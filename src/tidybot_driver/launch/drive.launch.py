from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition

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
        condition=IfCondition(PythonExpression(
            LaunchConfiguration("mode"), " == 'arm_only' or ", LaunchConfiguration("mode"), " == 'full'"
        ))
    )

    base_server = Node(
        package="tidybot_driver",
        executable="base_server",
        name="base_server",
        condition=IfCondition(PythonExpression(
            LaunchConfiguration("mode"), " == 'base_only' or ", LaunchConfiguration("mode"), " == 'full'"
        )),
        # prefix=['sudo', 'chrt', '-f', '80']
    )

    return LaunchDescription([
        mode,
        arm_server,
        base_server
    ])