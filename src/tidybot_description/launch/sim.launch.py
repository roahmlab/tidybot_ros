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
    ld = LaunchDescription()
    ros_gz_sim_pkg = FindPackageShare("ros_gz_sim")
    tidybot_pkg = FindPackageShare("tidybot_description")
    ld.add_action(
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH", PathJoinSubstitution([tidybot_pkg, "urdf"])
        )
    )
    ld.add_action(
        SetEnvironmentVariable(
            "GZ_SIM_SYSTEM_PLUGIN_PATH", "/opt/ros/jazzy/lib/"
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_rviz", default_value="true", description="Flag to enable RViz"
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "base_mode",
            default_value="position",
            description='Base control mode: "position" for position control, "velocity" for velocity control',
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([ros_gz_sim_pkg, "launch", "gz_sim.launch.py"])
            ),
            launch_arguments={"gz_args": "-r empty.sdf"}.items(),
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([tidybot_pkg, "launch", "display.launch.py"])
            ),
            launch_arguments={
                "robot_description_content": Command(
                    [
                        "xacro ",
                        PathJoinSubstitution([tidybot_pkg, "urdf", "tidybot.xacro"]),
                        " hardware_plugin:=gz_ros2_control/GazeboSimSystem",
                    ]
                ),
                "use_sim_time": "true",
                "use_rviz": LaunchConfiguration("use_rviz"),
                "jsp": "false",
                "jsp_gui": "false",
            }.items(),
        )
    )
    ld.add_action(
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=["-world", "empty", 
                       "-entity", "tidybot", 
                       "-topic", "robot_description",
                       "-x", "0.0",
                       "-y", "0.0",
                       "-z", "0.0",
                       "-Y", "0.0"],
            output="screen",
        )
    )
    gz_topic = '/model/tidybot'
    joint_state_gz_topic = '/world/empty' + gz_topic + '/joint_state'
    link_pose_gz_topic = gz_topic + '/pose'
    ld.add_action(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            parameters=[{"use_sim_time": True}],
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                joint_state_gz_topic + "@sensor_msgs/msg/JointState[gz.msgs.Model",
                link_pose_gz_topic + "@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                "/world/empty/control@ros_gz_interfaces/srv/ControlWorld",
                "/world/empty/create@ros_gz_interfaces/srv/SpawnEntity",
            ],
            output="screen",
        )
    )
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen",
        )
    )
    ld.add_action(
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
        )
    )
    ld.add_action(
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
        )
    )
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["gen3_7dof_controller"],
            output="screen",
        )
    )
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["gen3_lite_2f_controller"],
            output="screen",
        )
    )
    return ld
