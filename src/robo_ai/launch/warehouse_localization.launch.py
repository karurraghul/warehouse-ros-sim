"""Launch exactly one localization backend: AMCL, slam_toolbox online, or localization."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    robo_ai_share = get_package_share_directory('robo_ai')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    localization_mode = LaunchConfiguration('localization_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    namespace = LaunchConfiguration('namespace')

    is_amcl = PythonExpression(["'", localization_mode, "' == 'amcl'"])
    is_slam_online = PythonExpression(["'", localization_mode, "' == 'slam_online'"])
    is_slam_loc = PythonExpression(["'", localization_mode, "' == 'slam_localization'"])

    maps_dir = os.path.join(robo_ai_share, 'maps')
    warehouse_map_base = os.path.join(maps_dir, 'warehouse')
    slam_online_params = os.path.join(robo_ai_share, 'config', 'slam_toolbox_online.yaml')
    slam_loc_params_src = os.path.join(robo_ai_share, 'config', 'slam_toolbox_localization.yaml')

    posegraph_path = warehouse_map_base + '.posegraph'
    posegraph_data = warehouse_map_base + '.data'
    slam_map_actions = []
    if not os.path.isfile(posegraph_path) or not os.path.isfile(posegraph_data):
        missing_msg = (
            f'slam_localization requires {posegraph_path} and {posegraph_data}. '
            'Run mapping with localization_mode:=slam_online, then '
            'python3 src/robo_ai/scripts/bootstrap_slam_map.py '
            '--map-name warehouse --output-dir src/robo_ai/maps'
        )
        slam_map_actions.append(
            GroupAction(
                condition=IfCondition(is_slam_loc),
                actions=[
                    LogInfo(msg=f'ERROR: {missing_msg}'),
                    Shutdown(reason='Missing warehouse posegraph for slam_localization'),
                ],
            ),
        )

    amcl_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'localization_launch.py')),
        condition=IfCondition(is_amcl),
        launch_arguments={
            'namespace': namespace,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
            'container_name': 'nav2_container',
            'log_level': log_level,
        }.items(),
    )

    # Mapping-only mode: builds posegraph for bootstrap; not for production missions.
    slam_online_node = Node(
        condition=IfCondition(is_slam_online),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_online_params,
            {'use_sim_time': use_sim_time},
        ],
    )

    slam_map_saver = GroupAction(
        condition=IfCondition(is_slam_online),
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_saver_server',
                name='map_saver',
                output='screen',
                parameters=[params_file],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_slam',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'node_names': ['map_saver'],
                }],
            ),
        ],
    )

    slam_localization_node = Node(
        condition=IfCondition(is_slam_loc),
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_loc_params_src,
            {'use_sim_time': use_sim_time, 'map_file_name': warehouse_map_base},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'localization_mode',
            default_value='slam_localization',
            description='Localization backend: slam_localization, amcl, or slam_online'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('map'),
        DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_composition', default_value='True'),
        DeclareLaunchArgument('use_respawn', default_value='False'),
        DeclareLaunchArgument('log_level', default_value='info'),
        *slam_map_actions,
        amcl_localization,
        slam_online_node,
        slam_map_saver,
        slam_localization_node,
    ])
