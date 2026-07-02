"""Drive the delivery robot through a list of warehouse waypoints using
Nav2's `nav2_simple_commander` (`BasicNavigator.followWaypoints`).

Waypoints and the assumed initial pose are loaded from a YAML file (default:
`config/waypoints.yaml` in this package). While parked at each waypoint it
also listens to `/detected_markers` (published by robo_ai_vision's
marker_detector_node) and logs any marker seen so far, so scanning and
navigation can be exercised together without tight coupling between the two
packages.
"""
import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import String


def yaw_to_quaternion_zw(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def make_pose_stamped(navigator, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    qz, qw = yaw_to_quaternion_zw(float(yaw))
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def load_waypoints_file(path):
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    return data['initial_pose'], data['waypoints']


def main(args=None):
    rclpy.init(args=args)

    navigator = BasicNavigator()
    navigator.declare_parameter(
        'waypoints_file',
        os.path.join(get_package_share_directory('robo_ai_nav'),
                      'config', 'waypoints.yaml'))
    waypoints_file = navigator.get_parameter('waypoints_file').value

    initial_pose_cfg, waypoints_cfg = load_waypoints_file(waypoints_file)

    latest_markers = {}

    def on_detected_markers(msg: String):
        latest_markers['data'] = msg.data

    navigator.create_subscription(String, '/detected_markers', on_detected_markers, 10)

    navigator.get_logger().info(f'Loaded {len(waypoints_cfg)} waypoints from {waypoints_file}')

    initial_pose = make_pose_stamped(
        navigator, initial_pose_cfg['x'], initial_pose_cfg['y'], initial_pose_cfg['yaw'])
    navigator.setInitialPose(initial_pose)

    navigator.get_logger().info('Waiting for Nav2 to become active...')
    navigator.waitUntilNav2Active()

    goal_poses = [
        make_pose_stamped(navigator, wp['x'], wp['y'], wp['yaw'])
        for wp in waypoints_cfg
    ]
    names = [wp['name'] for wp in waypoints_cfg]

    navigator.get_logger().info(f'Following {len(goal_poses)} waypoints: {names}')
    navigator.followWaypoints(goal_poses)

    last_index = -1
    while not navigator.isTaskComplete():
        rclpy.spin_once(navigator, timeout_sec=0.5)
        feedback = navigator.getFeedback()
        if feedback and feedback.current_waypoint != last_index:
            last_index = feedback.current_waypoint
            if last_index < len(names):
                navigator.get_logger().info(
                    f'Heading to waypoint {last_index + 1}/{len(names)}: {names[last_index]}')
        if 'data' in latest_markers:
            navigator.get_logger().info(f'Marker(s) seen: {latest_markers.pop("data")}')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('All waypoints completed successfully.')
    elif result == TaskResult.CANCELED:
        navigator.get_logger().warn('Waypoint following was canceled.')
    elif result == TaskResult.FAILED:
        navigator.get_logger().error('Waypoint following failed.')

    navigator.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
