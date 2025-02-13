from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='attack_dog',
            executable='detection',
            name='cup_detection',
            output='screen'
        ),
        Node(
            package='attack_dog',
            executable='navigation',
            name='nav_planner',
            output='screen'
        ),
        Node(
            package='attack_dog',
            executable='attack',
            name='attack_node',
            output='screen'
        )
    ])
