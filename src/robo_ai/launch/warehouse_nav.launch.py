import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('robo_ai')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    map_file = os.path.join(pkg, 'maps', 'warehouse_map.yaml')
    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': nav2_params,
            'use_sim_time': 'true'
        }.items()
    )

    return LaunchDescription([nav2])
