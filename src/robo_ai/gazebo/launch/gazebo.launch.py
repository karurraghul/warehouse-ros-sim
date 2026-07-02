from ament_index_python import get_package_share_directory
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
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
    resource_path = media_path
    for resource_dir in [
            '/usr/share/gazebo-11',
            os.path.join(pkg_share, 'gazebo_models')]:
        if os.path.isdir(resource_dir) and resource_dir not in resource_path:
            resource_path = (
                resource_path + pathsep + resource_dir
                if resource_path else resource_dir
            )

    ros_distro = os.environ.get('ROS_DISTRO', 'humble')
    default_ros_lib = f'/opt/ros/{ros_distro}/lib'
    if os.path.isdir(default_ros_lib) and default_ros_lib not in plugin_path:
        plugin_path = (
            plugin_path + pathsep + default_ros_lib
            if plugin_path else default_ros_lib
        )

    robo_ai_gazebo_models = os.path.join(pkg_share, 'gazebo_models', 'models')
    if os.path.isdir(robo_ai_gazebo_models) and robo_ai_gazebo_models not in model_path:
        model_path = (
            model_path + pathsep + robo_ai_gazebo_models
            if model_path else robo_ai_gazebo_models
        )

    robo_ai_share_root = os.path.dirname(pkg_share)
    if robo_ai_share_root not in model_path:
        model_path = (
            model_path + pathsep + robo_ai_share_root
            if model_path else robo_ai_share_root
        )

    gazebo_env = {
        'GAZEBO_MODEL_PATH': model_path,
        'GAZEBO_PLUGIN_PATH': plugin_path,
        'GAZEBO_RESOURCE_PATH': resource_path,
    }

    world_path = os.path.join(pkg_share, 'worlds', 'warehouse.world')
    state_publisher_launch = os.path.join(
        pkg_share, 'launch', 'state_publisher_delivery.launch.py')

    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x')
    y_pose = LaunchConfiguration('y')
    z_pose = LaunchConfiguration('z')
    yaw_pose = LaunchConfiguration('yaw')

    return launch.LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', plugin_path),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', resource_path),

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
            PythonLaunchDescriptionSource(state_publisher_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items()),

        TimerAction(
            period=5.0,
            actions=[
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
            ],
        ),
    ])
