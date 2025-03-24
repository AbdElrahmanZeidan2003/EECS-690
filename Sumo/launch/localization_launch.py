from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    slam_config = LaunchConfiguration('slam_params_file')

    declare_slam_config_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            FindPackageShare('sumo_bot').find('sth - this is supposed to be the main folder ... '),
            'config',
            'localization.yaml'
        ),
        description='Full path to the SLAM toolbox parameters file'
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config]
    )

    wall_proximity_node = Node(
        package='sth - main package name ',
        executable='localization',
        name='sumo_localization_node',
        output='screen',
        parameters=[{'min_safe_distance': 0.3}]
    )

    return LaunchDescription([
        declare_slam_config_arg,
        slam_toolbox_node,
        localization_node
    ])
