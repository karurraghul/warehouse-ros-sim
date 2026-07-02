"""Drive the delivery robot through a list of warehouse waypoints using
Nav2's `nav2_simple_commander` (`BasicNavigator.goToPose` per leg).

Waypoints and the assumed initial pose are loaded from a YAML file (default:
`config/waypoints.yaml` in this package). Optional per-waypoint `nav_profile`
switches costmap inflation before each leg. While parked at each waypoint it
listens to `/detected_markers` and logs scan results.

Shelf-row stops may define `retreat_after_scan` to back the robot out of a
tight aisle before continuing to the next waypoint.
"""
import json
import math
import os
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rcl_interfaces.msg import Parameter as ParamMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String


def yaw_to_quaternion_zw(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))


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


def load_yaml_file(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def load_waypoints_file(path):
    data = load_yaml_file(path)
    return data['initial_pose'], data['waypoints']


def parse_aruco_ids(marker_json):
    if not marker_json:
        return []
    try:
        payload = json.loads(marker_json)
    except json.JSONDecodeError:
        return []
    return [
        d['id'] for d in payload.get('detections', [])
        if d.get('type') == 'aruco'
    ]


def make_double_param(name, value):
    msg = ParamMsg()
    msg.name = name
    msg.value = ParameterValue(
        type=ParameterType.PARAMETER_DOUBLE,
        double_value=float(value),
    )
    return msg


class NavProfileApplier:
    """Apply Nav2 costmap/controller params for per-waypoint navigation profiles."""

    COSTMAP_NODES = (
        '/local_costmap/local_costmap',
        '/global_costmap/global_costmap',
    )

    def __init__(self, node, profiles):
        self._node = node
        self._profiles = profiles
        self._clients = {}

    def _get_client(self, target_node):
        if target_node not in self._clients:
            self._clients[target_node] = self._node.create_client(
                SetParameters, f'{target_node}/set_parameters')
        return self._clients[target_node]

    def _set_params(self, target_node, param_dict):
        client = self._get_client(target_node)
        if not client.wait_for_service(timeout_sec=2.0):
            self._node.get_logger().warn(
                f'Parameter service unavailable: {target_node}/set_parameters')
            return False

        request = SetParameters.Request()
        request.parameters = [
            make_double_param(name, value) for name, value in param_dict.items()
        ]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            self._node.get_logger().warn(
                f'Failed to set parameters on {target_node}')
            return False

        for result in future.result().results:
            if not result.successful:
                self._node.get_logger().warn(
                    f'Parameter set failed on {target_node}: {result.reason}')
                return False
        return True

    def apply(self, profile_name, waypoint_name):
        profile = self._profiles.get(profile_name)
        if profile is None:
            self._node.get_logger().warn(
                f'Unknown nav profile "{profile_name}" for {waypoint_name}, '
                f'using "default"')
            profile_name = 'default'
            profile = self._profiles['default']

        inflation = profile['inflation_radius']
        xy_tol = profile['xy_goal_tolerance']
        self._node.get_logger().info(
            f"Applying nav profile '{profile_name}' for {waypoint_name} "
            f'(inflation={inflation}, xy_goal_tolerance={xy_tol})')

        costmap_params = {'inflation_layer.inflation_radius': inflation}
        ok = True
        for node_name in self.COSTMAP_NODES:
            ok = self._set_params(node_name, costmap_params) and ok
        ok = self._set_params(
            '/controller_server',
            {'general_goal_checker.xy_goal_tolerance': xy_tol},
        ) and ok
        return ok

    def restore_default(self):
        return self.apply('default', 'restore')


def validate_goal_clearance(navigator, wp, leg_label):
    if not wp.get('shelf_row_stop'):
        return True
    min_x = float(wp.get('min_clear_x', 2.5))
    goal_x = float(wp['x'])
    if goal_x > min_x:
        navigator.get_logger().error(
            f'Goal ({goal_x}, {wp["y"]}) rejected for {leg_label}: '
            f'x > min_clear_x {min_x} for shelf_row_stop')
        return False
    return True


def log_waypoint_scan(navigator, wp_cfg, marker_json, amcl_pose):
    if wp_cfg.get('skip_scan'):
        return

    name = wp_cfg.get('name', 'unknown')
    expected = wp_cfg.get('expected_aruco_id')
    seen = parse_aruco_ids(marker_json)

    if amcl_pose is not None:
        p = amcl_pose.pose.pose.position
        yaw = yaw_from_quaternion(amcl_pose.pose.pose.orientation)
        navigator.get_logger().info(
            f'AMCL pose at {name}: x={p.x:.2f}, y={p.y:.2f}, yaw={yaw:.2f} rad')

    if expected is not None:
        if expected in seen:
            navigator.get_logger().info(
                f'SCAN OK at {name}: expected ArUco id={expected}, '
                f'saw ids={seen}')
        else:
            navigator.get_logger().warn(
                f'SCAN MISS at {name}: expected ArUco id={expected}, '
                f'saw ids={seen or "none"}. Move closer or fix marker yaw.')
    elif seen:
        navigator.get_logger().info(f'At {name}: saw ArUco ids={seen}')
    else:
        navigator.get_logger().warn(f'At {name}: no ArUco markers detected.')


def dwell_for_scan(navigator, dwell_sec):
    """Hold at goal briefly so the camera can publish detections."""
    deadline = time.monotonic() + dwell_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(navigator, timeout_sec=0.1)


def navigate_to_pose(navigator, wp, leg_label, profile_applier, latest_markers):
    if not validate_goal_clearance(navigator, wp, leg_label):
        return TaskResult.FAILED

    profile_name = wp.get('nav_profile', 'default')
    profile_applier.apply(profile_name, leg_label)

    navigator.get_logger().info(
        f'Navigating: {leg_label} at ({wp["x"]}, {wp["y"]}, yaw={wp["yaw"]})')

    goal = make_pose_stamped(navigator, wp['x'], wp['y'], wp['yaw'])
    navigator.goToPose(goal)

    while not navigator.isTaskComplete():
        rclpy.spin_once(navigator, timeout_sec=0.5)
        if 'data' in latest_markers:
            seen = parse_aruco_ids(latest_markers['data'])
            if seen:
                navigator.get_logger().info(
                    f'Live detection while navigating to {leg_label}: '
                    f'ArUco ids={seen}')

    leg_result = navigator.getResult()
    if leg_result != TaskResult.SUCCEEDED:
        navigator.get_logger().error(
            f'Failed {leg_label} (result={leg_result})')
    else:
        navigator.get_logger().info(f'Reached {leg_label}')
    return leg_result


def main(args=None):
    rclpy.init(args=args)

    navigator = BasicNavigator()
    pkg_share = get_package_share_directory('robo_ai_nav')
    navigator.declare_parameter(
        'waypoints_file',
        os.path.join(pkg_share, 'config', 'waypoints.yaml'))
    navigator.declare_parameter(
        'nav_profiles_file',
        os.path.join(pkg_share, 'config', 'nav_profiles.yaml'))
    navigator.declare_parameter('shutdown_nav2_on_exit', True)
    navigator.declare_parameter('shutdown_nav2_on_failure', False)
    navigator.declare_parameter('scan_dwell_sec', 4.0)

    waypoints_file = navigator.get_parameter('waypoints_file').value
    nav_profiles_file = navigator.get_parameter('nav_profiles_file').value
    shutdown_on_exit = navigator.get_parameter('shutdown_nav2_on_exit').value
    shutdown_on_failure = navigator.get_parameter('shutdown_nav2_on_failure').value
    scan_dwell_sec = navigator.get_parameter('scan_dwell_sec').value

    initial_pose_cfg, waypoints_cfg = load_waypoints_file(waypoints_file)
    nav_profiles = load_yaml_file(nav_profiles_file)
    profile_applier = NavProfileApplier(navigator, nav_profiles)

    latest_markers = {}
    latest_amcl = {'pose': None}

    def on_detected_markers(msg: String):
        latest_markers['data'] = msg.data

    def on_amcl_pose(msg: PoseWithCovarianceStamped):
        latest_amcl['pose'] = msg

    navigator.create_subscription(String, '/detected_markers', on_detected_markers, 10)
    navigator.create_subscription(
        PoseWithCovarianceStamped, '/amcl_pose', on_amcl_pose, 10)

    navigator.get_logger().info(f'Loaded {len(waypoints_cfg)} waypoints from {waypoints_file}')
    for i, wp in enumerate(waypoints_cfg):
        expected = wp.get('expected_aruco_id', '?')
        profile = wp.get('nav_profile', 'default')
        retreat = 'yes' if wp.get('retreat_after_scan') else 'no'
        navigator.get_logger().info(
            f'  [{i}] {wp["name"]}: ({wp["x"]}, {wp["y"]}, yaw={wp["yaw"]}) '
            f'-> expect ArUco id={expected}, nav_profile={profile}, '
            f'retreat_after_scan={retreat}')

    initial_pose = make_pose_stamped(
        navigator, initial_pose_cfg['x'], initial_pose_cfg['y'], initial_pose_cfg['yaw'])
    navigator.setInitialPose(initial_pose)

    navigator.get_logger().info('Waiting for Nav2 to become active...')
    navigator.waitUntilNav2Active()

    overall_result = TaskResult.SUCCEEDED

    for index, wp in enumerate(waypoints_cfg):
        leg_label = f'waypoint {index + 1}/{len(waypoints_cfg)}: {wp["name"]}'
        leg_result = navigate_to_pose(
            navigator, wp, leg_label, profile_applier, latest_markers)
        if leg_result != TaskResult.SUCCEEDED:
            overall_result = leg_result
            break

        dwell_for_scan(navigator, scan_dwell_sec)
        log_waypoint_scan(navigator, wp, latest_markers.get('data'), latest_amcl['pose'])

        retreat = wp.get('retreat_after_scan')
        if retreat:
            retreat_wp = {
                'x': retreat['x'],
                'y': retreat['y'],
                'yaw': retreat['yaw'],
                'nav_profile': wp.get('retreat_nav_profile', 'default'),
            }
            retreat_label = f'retreat after {wp["name"]}'
            navigator.get_logger().info(
                f'Backing out of aisle before next waypoint -> '
                f'({retreat_wp["x"]}, {retreat_wp["y"]}, yaw={retreat_wp["yaw"]})')
            retreat_result = navigate_to_pose(
                navigator, retreat_wp, retreat_label, profile_applier, latest_markers)
            if retreat_result != TaskResult.SUCCEEDED:
                overall_result = retreat_result
                navigator.get_logger().error(
                    'Retreat from shelf aisle failed; cannot safely reach next waypoint.')
                break

    profile_applier.restore_default()

    if overall_result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('All waypoints completed successfully.')
        if shutdown_on_exit:
            navigator.get_logger().info('Shutting down Nav2 lifecycle nodes.')
            navigator.lifecycleShutdown()
    elif overall_result == TaskResult.CANCELED:
        navigator.get_logger().warn('Waypoint following was canceled.')
        if shutdown_on_failure:
            navigator.lifecycleShutdown()
        else:
            navigator.get_logger().info(
                'Nav2 left running (shutdown_nav2_on_failure:=false).')
    elif overall_result == TaskResult.FAILED:
        navigator.get_logger().error(
            'Waypoint following failed. Check Nav2 logs for planner/controller errors.')
        if shutdown_on_failure:
            navigator.lifecycleShutdown()
        else:
            navigator.get_logger().info(
                'Nav2 left running (shutdown_nav2_on_failure:=false).')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
