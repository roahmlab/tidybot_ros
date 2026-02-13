"""
Launch file for TidyBot ROS2 stack with Isaac Lab simulation.

Starts:
  1. robot_state_publisher – publishes /tf from /joint_states
  2. tf_relay – relays TF for RViz compatibility
  3. isaac_lab_bridge – monitors Isaac Lab connection
  4. rviz2 (optional) – visualization
  5. Drawer policy stack (optional):
     - multi_stage_planner (MoveIt)
     - drawer_policy (task service)
     - sensor_data_recorder

Arguments:
    gripper_type: Which gripper to use ('hande' or '2f85')
    use_rviz: Launch RViz (default: true)
    enable_drawer_policy: Launch drawer policy stack (default: true)
    record_sensor_data: Enable sensor recording (default: true)
    approach_distance: Distance to approach drawer handle from (meters)

Usage:
  # Hand-E gripper (default):
  ros2 launch tidybot_description launch_isaac_lab.launch.py

  # 2F-85 gripper:
  ros2 launch tidybot_description launch_isaac_lab.launch.py gripper_type:=2f85
"""


from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import (
    LaunchConfiguration,
)
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import Command, PathJoinSubstitution


def _launch_nodes(context):
    """Build nodes after launch args are resolved."""
    gripper_type = context.launch_configurations['gripper_type']
    robot_description_path = get_package_share_directory("tidybot_description")
    tidybot_pkg = FindPackageShare("tidybot_description")

    if gripper_type == 'hande':
        urdf_filename = "tidybot_hande_isaac.urdf"
        srdf_file = "config/tidybot_hande.srdf"
        joint_limits_file = "config/joint_limits_hande.yaml"
        rviz_config = PathJoinSubstitution([tidybot_pkg, "config", "tidybot_hande.rviz"])
    else:
        urdf_filename = "tidybot_2f_85_isaac.urdf"
        srdf_file = "config/tidybot.srdf"
        joint_limits_file = "config/joint_limits.yaml"
        rviz_config = PathJoinSubstitution([tidybot_pkg, "config", "tidybot.rviz"])

    # Robot description from pre-built URDF
    isaac_urdf_path = PathJoinSubstitution([tidybot_pkg, "urdf", urdf_filename])
    robot_description_content = Command(["cat ", isaac_urdf_path])

    # URDF path for MoveIt
    outpath = str(Path(robot_description_path) / f"urdf/{urdf_filename}")

    # Load MoveIt Configs
    moveit_config = MoveItConfigsBuilder(
        "tidybot", package_name="tidybot_moveit_config"
    ).robot_description(
        file_path=outpath
    ).robot_description_semantic(
        file_path=srdf_file
    ).joint_limits(
        file_path=joint_limits_file
    ).to_moveit_configs()

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

        # Isaac Lab Bridge
        Node(
            package="tidybot_description",
            executable="isaac_lab_bridge",
            name="isaac_lab_bridge",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "timeout_sec": 10.0,
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

        # === Drawer Policy Stack (optional) ===

        # Multi-Stage Planner (MoveIt-based motion executor)
        Node(
            condition=IfCondition(LaunchConfiguration("enable_drawer_policy")),
            package="tidybot_solver",
            executable="multi_stage_planner",
            name="multi_stage_planner",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {
                    "use_sim_time": True,
                    "arm_group": "gen3_7dof",
                    "tip_link": "tool_frame",
                    "gripper_type": gripper_type,
                }
            ],
        ),

        # Drawer Policy (task-specific policy server)
        Node(
            condition=IfCondition(LaunchConfiguration("enable_drawer_policy")),
            package="tidybot_policy",
            executable="drawer_policy",
            name="drawer_policy",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "approach_distance": LaunchConfiguration("approach_distance"),
                "approach_duration": 3.0,
                "grasp_duration": 2.0,
                "pull_duration": 2.5,
                "gripper_duration": 1.0,
                "wait_after_grasp_duration": 2.0,
                "record_sensor_data": LaunchConfiguration("record_sensor_data"),
            }],
        ),

        # Sensor Data Recorder
        Node(
            condition=IfCondition(LaunchConfiguration("record_sensor_data")),
            package="tidybot_episode",
            executable="sensor_data_recorder",
            name="sensor_data_recorder",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "output_dir": LaunchConfiguration("output_dir"),
                "record_rate": 20.0,
            }],
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
            "enable_drawer_policy",
            default_value="true",
            description="Launch drawer policy stack (planner + policy + recorder)"
        ),
        DeclareLaunchArgument(
            "record_sensor_data",
            default_value="true",
            description="Enable sensor data recording during drawer tasks"
        ),
        DeclareLaunchArgument(
            "output_dir",
            default_value="./sensor_data",
            description="Directory for recorded sensor data CSV files"
        ),
        DeclareLaunchArgument(
            "approach_distance",
            default_value="0.15",
            description="Distance to approach drawer handle from (meters)"
        ),

        OpaqueFunction(function=_launch_nodes),
    ])
