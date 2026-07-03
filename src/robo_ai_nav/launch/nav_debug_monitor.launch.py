"""Launch the nav debug lifecycle monitor."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robo_ai_nav')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='robo_ai_nav',
            executable='nav_debug_monitor',
            name='nav_debug_monitor',
            output='screen',
            parameters=[
                os.path.join(pkg_share, 'config', 'nav_debug_monitor.yaml'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
