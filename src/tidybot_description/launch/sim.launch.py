from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    TimerAction,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution,
    Command,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    ros_gz_sim_pkg = FindPackageShare("ros_gz_sim")
    tidybot_pkg = FindPackageShare("tidybot_description")

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH", PathJoinSubstitution([tidybot_pkg, "urdf"])
            ),
            SetEnvironmentVariable(
                "GZ_SIM_PLUGIN_PATH", PathJoinSubstitution([tidybot_pkg, "plugins"])
            ),
            DeclareLaunchArgument(
                "use_rviz", default_value="true", description="Flag to enable RViz"
            ),
            DeclareLaunchArgument(
                "base_mode",
                default_value="position",
                description='Base control mode: "position" for position control, "velocity" for velocity control',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([ros_gz_sim_pkg, "launch", "gz_sim.launch.py"])
                ),
                launch_arguments={"gz_args": "-r empty.sdf"}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([tidybot_pkg, "launch", "display.launch.py"])
                ),
                launch_arguments={
                    "robot_description_content": Command(
                        [
                            "xacro ",
                            PathJoinSubstitution(
                                [tidybot_pkg, "urdf", "tidybot.xacro"]
                            ),
                            " hardware_plugin:=gz_ros2_control/GazeboSimSystem",
                        ]
                    ),
                    "use_sim_time": "true",
                    "use_rviz": LaunchConfiguration("use_rviz"),
                    "jsp_gui": "false",
                }.items(),
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=["-name", "tidybot", "-topic", "robot_description"],
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                parameters=[{"use_sim_time": True}],
                arguments=[
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                    "/world/empty/model/tidybot/joint_state@sensor_msgs/msg/JointState[gz.msgs.JointState",
                ],
                remappings=[
                    ("/world/empty/model/tidybot/joint_state", "/joint_states"),
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["tidybot_base_pos_controller"],
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        ['"', LaunchConfiguration("base_mode"), '" == "position"']
                    )
                ),
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["tidybot_base_vel_controller"],
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        ['"', LaunchConfiguration("base_mode"), '" == "velocity"']
                    )
                ),
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["gen3_lite_controller"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["gen3_lite_2f_controller"],
                output="screen",
            ),
        ]
    )
