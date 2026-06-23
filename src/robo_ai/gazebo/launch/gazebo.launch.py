from ament_index_python import get_package_share_directory
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
from launch_ros.actions import Node
import os
import sys
from os import environ, pathsep

sys.path.insert(0, '/opt/ros/humble/lib/gazebo_ros')
from gazebo_ros_paths import GazeboRosPaths


def generate_launch_description():
    model_path, plugin_path, media_path = GazeboRosPaths.get_paths()

    if 'GAZEBO_MODEL_PATH' in environ:
        model_path += pathsep + environ['GAZEBO_MODEL_PATH']
    if 'GAZEBO_PLUGIN_PATH' in environ:
        plugin_path += pathsep + environ['GAZEBO_PLUGIN_PATH']
    if 'GAZEBO_RESOURCE_PATH' in environ:
        media_path += pathsep + environ['GAZEBO_RESOURCE_PATH']

    gazebo_env = {
        'GAZEBO_MODEL_PATH': model_path,
        'GAZEBO_PLUGIN_PATH': plugin_path,
        'GAZEBO_RESOURCE_PATH': media_path,
    }

    world_path = os.path.join(
        get_package_share_directory('robo_ai'), 'worlds/room2.sdf')
    rob_description_launch_file = os.path.join(
        get_package_share_directory('robo_ai'), 'launch', 'state_publisher.launch.py')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return launch.LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', plugin_path),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', media_path),

        launch.actions.ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so', world_path],
            additional_env=gazebo_env,
            output='screen'),

        launch.actions.DeclareLaunchArgument(
            name='use_sim_time', default_value='True',
            description='Flag to enable use_sim_time'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rob_description_launch_file)),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'rob_description', '-topic', 'robot_description'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])