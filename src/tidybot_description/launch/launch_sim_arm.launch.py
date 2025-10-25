from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution,
    Command,
    LaunchConfiguration,
)
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

ros_gz_sim_pkg = FindPackageShare("ros_gz_sim")
tidybot_pkg = FindPackageShare("tidybot_description")
default_rviz_config_path = PathJoinSubstitution(
    [tidybot_pkg, "config", "arm.rviz"]
)

def launch_world(context, *args, **kwargs):
    world = LaunchConfiguration("world").perform(context)
    world_path = get_package_share_directory("tidybot_description") + f"/urdf/world/{world}.sdf"
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([ros_gz_sim_pkg, "launch", "gz_sim.launch.py"])
            ),
            launch_arguments={"gz_args": f"-r {world_path}"}.items(),
        )
    ]

def launch_robot_description(context, *args, **kwargs):
    robot_description_content = Command(
        [
            "xacro ",
            PathJoinSubstitution([tidybot_pkg, "urdf", "arm.xacro"]),
        ],
    ).perform(context)
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([tidybot_pkg, "launch", "description.launch.py"])
            ),
            launch_arguments={
                "robot_description": robot_description_content,
                "use_sim_time": "true",
                "ignore_timestamp": "true",
            }.items(),
        ),
    ]

def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=[
                PathJoinSubstitution([tidybot_pkg, "urdf"]),
            ]
        )
    )
    ld.add_action(
        SetEnvironmentVariable(
            "GZ_SIM_SYSTEM_PLUGIN_PATH", "/opt/ros/jazzy/lib/"
        )
    )
    # if use rviz for visualization
    ld.add_action(
        DeclareLaunchArgument(
            "use_rviz", default_value="true", description="Flag to enable RViz"
        )
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
            "world",
            default_value="empty",
            choices=["empty", "office", "warehouse", "fetch_coke", "fetch_cube"],
            description="Path to the world file to load in Gazebo",
        )
    )
    ld.add_action(
        OpaqueFunction(function=launch_world)
    )
    ld.add_action(
        OpaqueFunction(function=launch_robot_description)
    )
    ld.add_action(
        Node(
            package="tidybot_description",
            executable="tf_relay",
            name="tf_relay",
            output="screen",
            parameters=[{"use_sim_time": True}],
        )
    )
    # launch rviz if enabled
    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[
                {"use_sim_time": True},
            ],
            remappings=[
                ("/tf", "/tf_relay"),
                ("/tf_static", "/tf_static_relay"),
            ],
            condition=IfCondition(LaunchConfiguration("use_rviz"))
        )
    )

    ld.add_action(
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=["-world", "empty", 
                       "-entity", "arm", 
                       "-topic", "robot_description",
                       "-x", "0.0",
                       "-y", "0.0",
                       "-z", "0.0",
                       "-Y", "0.0"],
            output="screen",
        )
    )
    gz_topic = '/model/arm'
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
                "/tidybot/camera_wrist/color/raw@sensor_msgs/msg/Image[gz.msgs.Image@/arm_camera/image",
                "/tidybot/camera_wrist/depth/raw@sensor_msgs/msg/Image[gz.msgs.Image@/arm_camera/depth_image",
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
            arguments=["gen3_7dof_controller"],
            output="screen",
        )
    )
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["robotiq_2f_85_controller"],
            output="screen",
        )
    )
    return ld
