from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
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
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("mode"), "' == 'arm_only' or '", LaunchConfiguration("mode"), "' == 'full'"
        ]))
    )
    
    camera_wrist_streamer = Node(
        package="tidybot_driver",
        executable="camera_wrist",
        name="camera_wrist",
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("mode"), "' == 'arm_only' or '", LaunchConfiguration("mode"), "' == 'full'"
        ]))
    )
    
    camera_ext_streamer = Node(
        package="tidybot_driver",
        executable="camera_ext",
        name="camera_ext",
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("mode"), "' == 'arm_only' or '", LaunchConfiguration("mode"), "' == 'full'"
        ]))
    )

    base_server = Node(
        package="tidybot_driver",
        executable="base_server",
        name="base_server",
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("mode"), "' == 'base_only' or '", LaunchConfiguration("mode"), "' == 'full'"
        ]))
    )
    
    tidybot_jsp = Node(
        package="tidybot_driver",
        executable="tidybot_joint_state_publisher",
        name="tidybot_joint_state_publisher",
        parameters=[{"mode": LaunchConfiguration("mode")}]
    )

    return LaunchDescription([
        mode,
        arm_server,
        camera_wrist_streamer,
        camera_ext_streamer,
        base_server,
        tidybot_jsp
    ])
