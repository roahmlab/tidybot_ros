from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    pkg_path = FindPackageShare(
        "tidybot_description"
    )

    ld.add_action(
        DeclareLaunchArgument(
            name="jsp_gui",
            default_value="true",
            choices=["true", "false"],
            description="Flag to enable joint_state_publisher_gui",
        )
    )

    default_rviz_config_path = PathJoinSubstitution(
        [pkg_path, "config", "tidybot.rviz"]
    )

    ld.add_action(
        DeclareLaunchArgument(
            name="rviz_config",
            default_value=default_rviz_config_path,
            description="Absolute path to rviz config file",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            name="robot_description_content",
            default_value=Command(
                ["xacro ", PathJoinSubstitution([pkg_path, "urdf", "tidybot.xacro"]), " "]
            ),
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            name="use_sim_time",
            default_value="true",
            description="Use simulation time if true"
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            name="use_rviz",
            default_value="true",
            description="Flag to enable RViz"
        )
    )

    # need to manually pass configuration in because of https://github.com/ros2/launch/issues/313
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_path, "launch", "description.launch.py"])
            ),
            launch_arguments={
                "robot_description": LaunchConfiguration("robot_description_content"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
        )
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    ld.add_action(
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            condition=UnlessCondition(LaunchConfiguration("jsp_gui")),
        )
    )

    ld.add_action(
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(LaunchConfiguration("jsp_gui")),
        )
    )

    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            condition=IfCondition(LaunchConfiguration("use_rviz"))
        )
    )
    return ld
