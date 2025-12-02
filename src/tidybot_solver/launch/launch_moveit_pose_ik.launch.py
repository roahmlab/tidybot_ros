from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Declare arguments
    use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="true",
        description="Use simulation (Gazebo) if true, hardware if false",
    )

    # Create the node
    pose_ik_node = Node(
        package="tidybot_solver",
        executable="moveit_ee_pose_ik",
        name="moveit_ee_pose_ik",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim")},
            {"planning_frame": "arm_base_link"},
        ],
    )

    return LaunchDescription([
        use_sim_arg,
        pose_ik_node,
    ])
