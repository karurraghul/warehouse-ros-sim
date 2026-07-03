"""Unified launch: Gazebo warehouse sim + robot spawn, Nav2 bringup, the
ArUco/QR marker detector, and (optionally) the waypoint navigator.

Usage:
    ros2 launch robo_ai warehouse_full.launch.py
    ros2 launch robo_ai warehouse_full.launch.py run_waypoint_navigator:=true

    ros2 launch robo_ai warehouse_full.launch.py localization_mode:=slam_online
    ros2 launch robo_ai warehouse_full.launch.py localization_mode:=slam_localization

Camera topic naming depends on the gazebo_ros_camera plugin config in
delivery_robot_plugins.gazebo - after launch, run `ros2 topic list` to
confirm the actual image topic and override `camera_topic` if it differs
from the default guess below.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robo_ai_share = get_package_share_directory('robo_ai')
    robo_ai_vision_share = get_package_share_directory('robo_ai_vision')
    robo_ai_nav_share = get_package_share_directory('robo_ai_nav')

    use_sim_time = LaunchConfiguration('use_sim_time')
    camera_topic = LaunchConfiguration('camera_topic')
    run_waypoint_navigator = LaunchConfiguration('run_waypoint_navigator')
    waypoints_file = LaunchConfiguration('waypoints_file')
    use_rviz = LaunchConfiguration('use_rviz')
    localization_mode = LaunchConfiguration('localization_mode')
    run_debug_monitor = LaunchConfiguration('run_debug_monitor')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robo_ai_share, 'launch', 'warehouse_delivery.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robo_ai_share, 'launch', 'warehouse_nav.launch.py')),
        launch_arguments={
            'localization_mode': localization_mode,
        }.items(),
    )

    # Nav2 needs map->odom from AMCL; robot and /scan must exist first.
    nav2_launch_delayed = TimerAction(period=6.0, actions=[nav2_launch])

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(robo_ai_share, 'config', 'ekf.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )
    ekf_node_delayed = TimerAction(period=6.0, actions=[ekf_node])

    initial_pose_node = Node(
        package='robo_ai_nav',
        executable='initial_pose_publisher',
        name='initial_pose_publisher',
        output='screen',
        parameters=[{
            'waypoints_file': waypoints_file,
            'use_sim_time': use_sim_time,
            'localization_mode': localization_mode,
        }],
    )
    initial_pose_delayed = TimerAction(period=7.0, actions=[initial_pose_node])

    debug_monitor_node = Node(
        package='robo_ai_nav',
        executable='nav_debug_monitor',
        name='nav_debug_monitor',
        output='screen',
        condition=IfCondition(run_debug_monitor),
        parameters=[
            os.path.join(robo_ai_nav_share, 'config', 'nav_debug_monitor.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )
    debug_monitor_delayed = TimerAction(period=8.0, actions=[debug_monitor_node])

    marker_detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robo_ai_vision_share, 'launch', 'marker_detector.launch.py')),
        launch_arguments={
            'camera_topic': camera_topic,
            'no_marker_log_interval_sec': '30.0',
        }.items(),
    )

    waypoint_navigator_node = Node(
        package='robo_ai_nav',
        executable='waypoint_navigator',
        name='waypoint_navigator',
        output='screen',
        condition=IfCondition(run_waypoint_navigator),
        parameters=[{
            'waypoints_file': waypoints_file,
            'use_sim_time': use_sim_time,
            'scan_dwell_sec': 2.0,
            'localization_mode': localization_mode,
        }],
    )
    # After Nav2 activation (~6s) + initial_pose (~7s) + AMCL settle.
    waypoint_navigator_delayed = TimerAction(
        period=12.0,
        actions=[waypoint_navigator_node],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(robo_ai_share, 'config', 'warehouse_nav.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )
    # After initial_pose (7s) + AMCL map->odom; avoids LaserScan TF cache drops.
    rviz_delayed = TimerAction(period=10.0, actions=[rviz_node])

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time', default_value='True',
            description='Flag to enable use_sim_time'),
        DeclareLaunchArgument(
            name='camera_topic', default_value='/delivery_camera/image_raw',
            description=(
                'Camera image topic for the marker detector. Confirm with '
                '`ros2 topic list` after launch and override if needed.')),
        DeclareLaunchArgument(
            name='run_waypoint_navigator', default_value='true',
            description='Also start robo_ai_nav waypoint_navigator on launch.'),
        DeclareLaunchArgument(
            name='waypoints_file',
            default_value=os.path.join(robo_ai_nav_share, 'config', 'waypoints.yaml'),
            description='Path to the waypoints YAML file.'),
        DeclareLaunchArgument(
            name='use_rviz', default_value='true',
            description='Launch RViz2 with warehouse Nav2 visualization.'),
        DeclareLaunchArgument(
            name='localization_mode',
            default_value='slam_localization',
            description=(
                'Localization backend: slam_localization (default), amcl, or '
                'slam_online (mapping/bootstrap only).')),
        DeclareLaunchArgument(
            name='run_debug_monitor', default_value='true',
            description='Launch nav_debug_monitor (stuck/collision/direction checks).'),

        sim_launch,
        ekf_node_delayed,
        nav2_launch_delayed,
        initial_pose_delayed,
        debug_monitor_delayed,
        marker_detector_launch,
        waypoint_navigator_delayed,
        rviz_delayed,
    ])
