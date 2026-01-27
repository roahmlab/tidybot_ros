from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch_ros.actions import Node
from launch_ros.actions import SetRemap
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    tidybot_description_pkg = FindPackageShare("tidybot_description")
    tidybot_driver_pkg = FindPackageShare("tidybot_driver")
    kinova_vision_pkg = FindPackageShare("kinova_vision")
    orbbec_launch_pkg = FindPackageShare("orbbec_camera")

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

    ext_camera = DeclareLaunchArgument(
        "ext_camera", 
        default_value="false", 
        description="Use external camera"
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
        arguments=["-d", PathJoinSubstitution([tidybot_driver_pkg, "config", "hardware.rviz"])],
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
    
    kinova_vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([kinova_vision_pkg, "launch", "kinova_vision.launch.py"])
        ),
        launch_arguments={
            "device": "192.168.1.10",  # Standard Gen3 IP
            "color_fps": "30",
            "depth_fps": "30",
        }.items(),
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("mode"), "' == 'arm_only' or '", LaunchConfiguration("mode"), "' == 'full'"
        ]))
    )

    camera_wrist_republisher = Node(
        package="tidybot_driver",
        executable="camera_wrist",
        name="camera_wrist",
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("mode"), "' == 'arm_only' or '", LaunchConfiguration("mode"), "' == 'full'"
        ])),
        parameters=[
            {"target_size": 224},
            {"crop_scale": 1.0}
        ],
    )
    
    camera_base_streamer = GroupAction(
        actions=[
            SetRemap(src='/camera/color/image_raw', dst='/tidybot/camera_base/color/raw'),
            SetRemap(src='/camera/color/image_raw/compressed', dst='/tidybot/camera_base/color/compressed'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([orbbec_launch_pkg, "launch", "femto_bolt.launch.py"])
                ),
                launch_arguments={
                    "enable_ldp": "false",
                    "enable_depth": "false",
                    "enable_depth_scale": "false",
                    "enable_ir": "false",
                    "enable_color": "true",
                    "enable_sync_output_accel_gyro": "false",
                    "publish_tf": "false",
                    "color_fps": "30",
                    # "color_width": "640",
                    # "color_height": "360",
                }.items(),
            ),
        ],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("mode"), "' == 'base_only' or '", LaunchConfiguration("mode"), "' == 'full'"
        ]))
    )
    
    camera_ext_streamer = Node(
        package="tidybot_driver",
        executable="camera_ext",
        name="camera_ext",
        condition=IfCondition(LaunchConfiguration("ext_camera"))
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
        ext_camera,
        rsp_launch,
        rviz_node,
        tf_relay,
        arm_server,
        kinova_vision_launch,
        camera_wrist_republisher,
        camera_base_streamer,
        camera_ext_streamer,
        base_server,
        jsp_node,
    ])