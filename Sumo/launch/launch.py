from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sumo',
            executable='orange',
            name='orange',
            output='screen'
        ),
        Node(
            package='sumo',
            executable='safety_node',
            name='safety_node',
            output='screen'
        )
    ])