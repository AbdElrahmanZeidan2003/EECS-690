from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        # SLAM Toolbox
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
        # Nav2 
        Node(package='nav2_controller', executable='controller_server', name='controller_server', output='screen'),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen'),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen'),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager_navigation',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{
                 'use_sim_time': False,
                 'autostart': True,
                 'node_names': [
                     'controller_server',
                     'planner_server',
                     'bt_navigator',
                     'map_server'
                 ]
             }]
        ),
        # Phase 1: Maze Explorer 
        Node(
            package='class_pkg',
            executable='maze_explorer',
            name='maze_explorer',
            output='screen'
        ),
        # Phase 2: Maze Solver (runs after delay)
        TimerAction(
            period= 2,  
            actions=[
                Node(
                    package='class_pkg',
                    executable='maze_solver',
                    name='maze_solver',
                    output='screen'
                )
            ]
        )
    ])
