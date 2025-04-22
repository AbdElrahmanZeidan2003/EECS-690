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
            package= 'nav2_bringup', 
            executable = 'bringup_launch.py',
            name = 'nav2_bringup',
            output = 'screen',
            arguments = ['--ros-args', '--log-level', 'info'],
            parameters = [{'use_sim_time' : False}]
        ),
        Node(
            package='class_pkg',
            executable='maze_explorer',
            name='maze_explorer',
            output='screen'
        )
    ])
