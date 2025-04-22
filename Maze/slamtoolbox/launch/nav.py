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
                'slam_params_file': '/.../slam_config.yaml', #find path to this file 
                'use_sim_time': False
            }],
            remappings=[('/scan', '/scan_raw')]
        ),
        Node(
            package='class_pkg',
            executable='maze_explorer',
            name='maze_explorer',
            output='screen'
        )
    ])
