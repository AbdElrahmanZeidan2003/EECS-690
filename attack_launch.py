from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Cup Detection Node (Stage 1)
        Node(
            package='attack_dog',
            executable='detection',
            name='cup_detection',
            output='screen'
        ),
        # Navigation Planner (Stage 2)
        Node(
            package='attack_dog',
            executable='navigation',
            name='nav_planner',
            output='screen'
        ),
        # Attack Execution Node (Stage 3)
        Node(
            package='attack_dog',
            executable='attack',
            name='attack_node',
            output='screen'
        ),
        # Attack Confirmation Node (Stage 3)
        Node(
            package='attack_dog',
            executable='attack_confirm',
            name='attack_confirm',
            output='screen'
        ),
        # Reset Node (Stage 4)
        Node(
            package='attack_dog',
            executable='reset_node',
            name='reset_node',
            output='screen'
        )
    ])
