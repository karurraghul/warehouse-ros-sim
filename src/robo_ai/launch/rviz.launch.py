from ament_index_python import get_package_share_directory
import launch
import os
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package='robo_ai'
    ).find('robo_ai')
    default_rviz_config_path = os.path.join(pkg_share, 'config/rviz.rviz')
    rvizconfig = LaunchConfiguration('rvizconfig')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='rvizconfig',
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rvizconfig],
            parameters=[{'use_sim_time': True}],
        )
    ])