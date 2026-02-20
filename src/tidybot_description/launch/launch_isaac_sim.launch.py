"""
Launch TidyBot with Isaac Sim

This launch file starts all necessary nodes to run TidyBot with Isaac Sim
instead of Gazebo. Isaac Sim must be running separately with the ROS 2
Action Graph configured.

Nodes launched:
    - robot_state_publisher: Publishes /tf from /joint_states
    - tf_relay: Relays TF to separate topics for RViz compatibility
    - isaac_sim_bridge: Bridges ros2_control topics to Isaac Sim's /joint_command

Usage:
    # Hand-E gripper (default):
    ros2 launch tidybot_description launch_isaac_sim.launch.py

    # 2F-85 gripper:
    ros2 launch tidybot_description launch_isaac_sim.launch.py gripper_type:=2f85

Arguments:
    gripper_type: Which gripper to use ('hande' or '2f85')
    use_rviz: Launch RViz for visualization (default: true)
    publish_rate: Isaac Sim bridge publishing rate in Hz (default: 50.0)
    use_velocity_control: Use velocity control for base (default: false)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
)
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def _launch_nodes(context):
    """Build nodes after launch args are resolved."""
    gripper_type = context.launch_configurations['gripper_type']
    tidybot_pkg = FindPackageShare("tidybot_description")

    if gripper_type == 'hande':
        urdf_filename = "tidybot_hande_isaac.urdf"
        rviz_config = PathJoinSubstitution([tidybot_pkg, "config", "tidybot_hande.rviz"])
    else:
        urdf_filename = "tidybot_2f_85_isaac.urdf"
        rviz_config = PathJoinSubstitution([tidybot_pkg, "config", "tidybot.rviz"])

    robot_description_content = Command([
        "cat ",
        PathJoinSubstitution([tidybot_pkg, "urdf", urdf_filename]),
    ])

    return [
        # Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": ParameterValue(robot_description_content, value_type=str),
                "use_sim_time": True,
                "ignore_timestamp": True,
            }],
        ),

        # TF Relay
        Node(
            package="tidybot_description",
            executable="tf_relay",
            name="tf_relay",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),

        # Isaac Sim Bridge
        Node(
            package="tidybot_description",
            executable="isaac_sim_bridge",
            name="isaac_sim_bridge",
            output="screen",
            parameters=[{
                "publish_rate": LaunchConfiguration("publish_rate"),
                "use_velocity_control": LaunchConfiguration("use_velocity_control"),
            }],
        ),

        # RViz (optional)
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": True}],
            remappings=[
                ("/tf", "/tf_relay"),
                ("/tf_static", "/tf_static_relay"),
            ],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "gripper_type",
            default_value="hande",
            description="Gripper type: 'hande' (Robotiq Hand-E) or '2f85' (Robotiq 2F-85)"
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz for visualization"
        ),
        DeclareLaunchArgument(
            "publish_rate",
            default_value="50.0",
            description="Isaac Sim bridge publishing rate in Hz"
        ),
        DeclareLaunchArgument(
            "use_velocity_control",
            default_value="false",
            description="Use velocity control for base instead of position"
        ),

        OpaqueFunction(function=_launch_nodes),
    ])
