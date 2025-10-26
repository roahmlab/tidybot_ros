from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch_param_builder import ParameterBuilder
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="true",
        description="Use simulation (Gazebo) if true, hardware if false",
    )

    moveit_config = (
        MoveItConfigsBuilder("tidybot", package_name="tidybot_moveit_config")
        .robot_description(file_path="config/tidybot.urdf.xacro")
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_semantic()
        .to_moveit_configs()
    )

    def launch_solver_node(context):
        use_sim_value = LaunchConfiguration("use_sim").perform(context).lower()
        servo_yaml = (
            "config/tidybot_servo_sim_config.yaml"
            if use_sim_value in ("true")
            else "config/tidybot_servo_hardware_config.yaml"
        )
        servo_params = {
            "moveit_servo": ParameterBuilder("tidybot_solver").yaml(servo_yaml).to_dict()
        }
        node = Node(
            package="tidybot_solver",
            executable="moveit_ee_vel_ik",
            name="moveit_ee_vel_ik",
            parameters=[
                servo_params,
                {"update_period": 0.001},  # Update period in seconds
                {"planning_group_name": "gen3_7dof"},
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
                {"use_sim_time": LaunchConfiguration("use_sim")},
                {"base_frame": "world"},
            ],
            output="screen",
        )
        return [node]

    return LaunchDescription(
        [
            use_sim_arg,
            OpaqueFunction(function=launch_solver_node),
        ]
    )
