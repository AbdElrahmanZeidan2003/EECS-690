from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'slam_params_file': '...', #find out what is the path for this
                'use_sim_time': False
            }],
            remappings=[('/scan', '/scan_raw')]
        ),
        Node(
            package='hiwonder_nav',
            executable='main_node',
            output='screen'
        )
    ])
