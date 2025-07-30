import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='tidybot_episode',
            executable='obs_recorder',
            name='episode_recorder',
            parameters=[{'use_sim_time': True}],
            remappings=[
                ('/tf', '/tf_relay'),
                ('/tf_static', '/tf_static_relay'),
            ],
            output='screen',
        )
    ])