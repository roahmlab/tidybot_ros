from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ros_gz_sim_pkg = FindPackageShare('ros_gz_sim')
    tidybot_pkg = FindPackageShare('tidybot_description')

    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            PathJoinSubstitution([tidybot_pkg, 'urdf'])
        ),
        SetEnvironmentVariable(
            'GZ_SIM_PLUGIN_PATH',
            PathJoinSubstitution([tidybot_pkg, 'plugins'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py'])
            ),
            launch_arguments={'gz_args': '-r empty.sdf'}.items()
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': Command([
                    'xacro ',
                    PathJoinSubstitution([tidybot_pkg, 'urdf', 'tidybot.xacro'])
                ]),
                'use_sim_time': True
            }],
            output='screen',
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'tidybot', '-topic', 'robot_description'],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/world/empty/model/tidybot/joint_state@sensor_msgs/msg/JointState[gz.msgs.JointState',
            ],
            remappings=[
                ('/world/empty/model/tidybot/joint_state', '/joint_states'),
            ],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller'],
            output='screen',
        ),
    ])
