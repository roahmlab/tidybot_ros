from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    tidybot_description_pkg = FindPackageShare("tidybot_description")

    mode = DeclareLaunchArgument(
        "mode", 
        default_value="full", 
        description="Control mode: full, arm_only, base_only"
    )

    base_mode = DeclareLaunchArgument(
        "base_mode", 
        default_value="position", 
        description="Base control mode: position or velocity"
    )

    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([tidybot_description_pkg, "launch", "description.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": "false",
            "ignore_timestamp": "false"
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", PathJoinSubstitution([tidybot_description_pkg, "config", "tidybot.rviz"])],
        parameters=[{"use_sim_time": False}],
    )

    tf_relay = Node(
        package="tidybot_description",
        executable="tf_relay",
        name="tf_relay",
        output="screen",
        parameters=[{"use_sim_time": False}],
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
        parameters=[{"mode": LaunchConfiguration("base_mode")}],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("mode"), "' == 'base_only' or '", LaunchConfiguration("mode"), "' == 'full'"
        ]))
    )

    jsp_node = Node(
        package="tidybot_driver",
        executable="joint_state_publisher",
        name="tidybot_joint_state_publisher",
        parameters=[{"mode": LaunchConfiguration("mode")}]
    )

    return LaunchDescription([
        mode,
        base_mode,
        rsp_launch,
        rviz_node,
        tf_relay,
        arm_server,
        camera_wrist_streamer,
        camera_ext_streamer,
        base_server,
        jsp_node,
    ])
