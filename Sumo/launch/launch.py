from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='class_pkg',
            executable='orange',
            name='orange',
            output='screen'
        ),
        Node(
            package='class_pkg',
            executable='safety_node',
            name='safety_node',
            output='screen'
        )
    ])
