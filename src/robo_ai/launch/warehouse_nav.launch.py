"""Nav2 navigation stack with selectable localization (AMCL or slam_toolbox)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def _launch_setup(context, *args, **kwargs):
    robo_ai_share = get_package_share_directory('robo_ai')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(nav2_bringup, 'launch')

    localization_mode = LaunchConfiguration('localization_mode').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    namespace = LaunchConfiguration('namespace')

    map_file = os.path.join(robo_ai_share, 'maps', 'warehouse_map.yaml')
    nav2_params = os.path.join(robo_ai_share, 'config', 'nav2_params.yaml')
    bt_pose_xml = os.path.join(
        robo_ai_share, 'config', 'navigate_to_pose_w_replanning_light_recovery.xml')
    bt_through_poses_xml = os.path.join(
        robo_ai_share, 'config', 'navigate_through_poses_w_replanning_light_recovery.xml')

    param_rewrites = {
        'default_nav_to_pose_bt_xml': bt_pose_xml,
        'default_nav_through_poses_bt_xml': bt_through_poses_xml,
        'yaml_filename': map_file,
    }
    if localization_mode in ('slam_online', 'slam_localization'):
        param_rewrites[
            'global_costmap.global_costmap.ros__parameters.obstacle_layer.enabled'
        ] = 'false'

    configured_params = RewrittenYaml(
        source_file=nav2_params,
        param_rewrites=param_rewrites,
        convert_types=True,
    )

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robo_ai_share, 'launch', 'warehouse_localization.launch.py')),
        launch_arguments={
            'localization_mode': LaunchConfiguration('localization_mode'),
            'namespace': namespace,
            'map': map_file,
            'params_file': configured_params,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
            'log_level': log_level,
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': configured_params,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
            'container_name': 'nav2_container',
            'log_level': log_level,
        }.items(),
    )

    nav2_container = Node(
        name='nav2_container',
        package='rclcpp_components',
        executable='component_container_isolated',
        parameters=[configured_params, {'autostart': autostart}],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
        output='screen',
    )

    return [
        GroupAction([
            nav2_container,
            localization_launch,
            navigation_launch,
        ]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'localization_mode',
            default_value='amcl',
            description='Localization: amcl, slam_online, or slam_localization'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_composition', default_value='True'),
        DeclareLaunchArgument('use_respawn', default_value='False'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('namespace', default_value=''),
        OpaqueFunction(function=_launch_setup),
    ])
