import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_waypoints_file = os.path.join(
        get_package_share_directory('robo_ai_nav'), 'config', 'waypoints.yaml')

    waypoints_file = LaunchConfiguration('waypoints_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='waypoints_file',
            default_value=default_waypoints_file,
            description='Path to the waypoints YAML file.',
        ),
        Node(
            package='robo_ai_nav',
            executable='waypoint_navigator',
            name='waypoint_navigator',
            output='screen',
            parameters=[{
                'waypoints_file': waypoints_file,
                'use_sim_time': True,
                'scan_dwell_sec': 2.0,
            }],
        ),
    ])
