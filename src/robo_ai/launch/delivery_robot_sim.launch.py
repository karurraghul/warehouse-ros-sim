from ament_index_python import get_package_share_directory
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    pkg_share = get_package_share_directory('robo_ai')
    pkg_models = os.path.join(pkg_share, 'models')
    if pkg_models not in model_path:
        model_path = model_path + pathsep + pkg_models

    gazebo_env = {
        'GAZEBO_MODEL_PATH': model_path,
        'GAZEBO_PLUGIN_PATH': plugin_path,
        'GAZEBO_RESOURCE_PATH': media_path,
    }

    world_path = os.path.join(pkg_share, 'worlds', 'my_world.sdf')
    delivery_robot_xacro = os.path.join(
        pkg_share, 'models', 'urdf', 'delivery_robot_description.xacro')
    state_publisher_launch_file = os.path.join(
        pkg_share, 'launch', 'state_publisher.launch.py')

    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x')
    y_pose = LaunchConfiguration('y')
    z_pose = LaunchConfiguration('z')
    yaw_pose = LaunchConfiguration('yaw')

    return launch.LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', plugin_path),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', media_path),

        DeclareLaunchArgument(
            name='use_sim_time', default_value='True',
            description='Flag to enable use_sim_time'),
        DeclareLaunchArgument(
            name='x', default_value='0.0',
            description='Spawn x position'),
        DeclareLaunchArgument(
            name='y', default_value='0.0',
            description='Spawn y position'),
        DeclareLaunchArgument(
            name='z', default_value='0.0',
            description='Spawn z position'),
        DeclareLaunchArgument(
            name='yaw', default_value='0.0',
            description='Spawn yaw (radians)'),

        launch.actions.ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so', world_path],
            additional_env=gazebo_env,
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(state_publisher_launch_file),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'model': delivery_robot_xacro,
            }.items()),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'delivery_robot',
                '-topic', 'robot_description',
                '-x', x_pose,
                '-y', y_pose,
                '-z', z_pose,
                '-Y', yaw_pose,
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
