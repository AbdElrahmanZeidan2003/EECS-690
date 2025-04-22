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
                'slam_params_file': '/home/ubuntu/ros2_ws/src/class_pkg/config/slam_config.yaml',
                'use_sim_time': False
            }],
            remappings=[('/scan', '/scan_raw')]
        ),
        Node(
            package='class_pkg',
            executable='explore_and_save',
            name='explore_and_save',
            output='screen'
        )
    ])
