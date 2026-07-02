"""Drive the delivery robot through a list of warehouse waypoints using
Nav2's `nav2_simple_commander` (`BasicNavigator.goToPose` per leg).

Waypoints and the assumed initial pose are loaded from a YAML file (default:
`config/waypoints.yaml` in this package). Optional per-waypoint `nav_profile`
switches costmap inflation before each leg. While parked at each waypoint it listens to `/detected_markers`, exits the
dwell early when the expected marker is seen, and logs one SCAN OK/MISS line.

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


def make_param(name, value):
    msg = ParamMsg()
    msg.name = name
    if isinstance(value, bool):
        msg.value = ParameterValue(
            type=ParameterType.PARAMETER_BOOL,
            bool_value=value,
        )
    else:
        msg.value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE,
            double_value=float(value),
        )
    return msg


def make_double_param(name, value):
    return make_param(name, value)


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
            make_param(name, value) for name, value in param_dict.items()
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
        cost_scale = profile.get('cost_scaling_factor', 3.0)
        obstacle_scale = profile.get('base_obstacle_scale', 0.08)
        static_map_only = profile.get('static_map_only', False)
        local_static_only = profile.get('local_static_only', False)
        self._node.get_logger().info(
            f"Applying nav profile '{profile_name}' for {waypoint_name} "
            f'(inflation={inflation}, cost_scale={cost_scale}, '
            f'obstacle_scale={obstacle_scale}, static_map_only={static_map_only}, '
            f'local_static_only={local_static_only}, xy_tol={xy_tol})')

        costmap_params = {
            'inflation_layer.inflation_radius': inflation,
            'inflation_layer.cost_scaling_factor': cost_scale,
        }
        if static_map_only:
            costmap_params['obstacle_layer.enabled'] = False
        ok = True
        for node_name in self.COSTMAP_NODES:
            node_params = dict(costmap_params)
            if static_map_only and node_name.endswith('local_costmap'):
                node_params.pop('obstacle_layer.enabled', None)
            if local_static_only and node_name.endswith('local_costmap'):
                node_params['voxel_layer.enabled'] = False
            ok = self._set_params(node_name, node_params) and ok
        ok = self._set_params(
            '/controller_server',
            {
                'general_goal_checker.xy_goal_tolerance': xy_tol,
                'FollowPath.BaseObstacle.scale': obstacle_scale,
            },
        ) and ok
        return ok

    def restore_default(self):
        ok = self.apply('default', 'restore')
        return self._set_params(
            '/local_costmap/local_costmap',
            {'voxel_layer.enabled': True},
        ) and ok


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

    if expected is not None:
        if expected in seen:
            navigator.get_logger().info(
                f'SCAN OK at {name}: expected ArUco id={expected}, '
                f'saw ids={seen}')
        else:
            pose_hint = ''
            if amcl_pose is not None:
                p = amcl_pose.pose.pose.position
                yaw = yaw_from_quaternion(amcl_pose.pose.pose.orientation)
                pose_hint = (
                    f' AMCL pose: x={p.x:.2f}, y={p.y:.2f}, yaw={yaw:.2f} rad.')
            navigator.get_logger().warn(
                f'SCAN MISS at {name}: expected ArUco id={expected}, '
                f'saw ids={seen or "none"}.{pose_hint} '
                f'Move closer or fix marker yaw.')
    elif seen:
        navigator.get_logger().info(f'SCAN OK at {name}: saw ArUco ids={seen}')
    else:
        pose_hint = ''
        if amcl_pose is not None:
            p = amcl_pose.pose.pose.position
            yaw = yaw_from_quaternion(amcl_pose.pose.pose.orientation)
            pose_hint = (
                f' AMCL pose: x={p.x:.2f}, y={p.y:.2f}, yaw={yaw:.2f} rad.')
        navigator.get_logger().warn(
            f'SCAN MISS at {name}: no ArUco markers detected.{pose_hint}')


def dwell_for_scan(navigator, dwell_sec, expected_id, latest_markers, early_exit=True):
    deadline = time.monotonic() + dwell_sec
    confirmed = False
    while time.monotonic() < deadline:
        rclpy.spin_once(navigator, timeout_sec=0.1)
        if early_exit and expected_id is not None:
            seen = parse_aruco_ids(latest_markers.get('data', ''))
            if expected_id in seen:
                confirmed = True
                break
    return confirmed


def build_pose_list(navigator, wp):
    """Build Nav2 pose list: optional through_poses then final x/y/yaw."""
    poses = []
    for point in wp.get('through_poses', []):
        poses.append(make_pose_stamped(
            navigator, point['x'], point['y'], point['yaw']))
    poses.append(make_pose_stamped(navigator, wp['x'], wp['y'], wp['yaw']))
    return poses


def relocalize_at_pose(navigator, x, y, yaw, label):
    """Reseed AMCL at a known map pose (reduces sim drift between short legs)."""
    navigator.get_logger().info(
        f'Relocalizing AMCL after {label} at ({x:.2f}, {y:.2f}, yaw={yaw:.2f})')
    navigator.setInitialPose(make_pose_stamped(navigator, x, y, yaw))
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        rclpy.spin_once(navigator, timeout_sec=0.1)
    navigator.clearLocalCostmap()
    time.sleep(0.5)


def force_relocalize_via_spin(navigator, label, spin_dist=1.57, time_allowance=10):
    """Spin in place so AMCL gets fresh lidar before the next leg."""
    navigator.get_logger().info(
        f'Forcing relocalization via spin after {label} '
        f'(target_yaw={spin_dist:.2f} rad)...')
    if not navigator.spin(spin_dist=spin_dist, time_allowance=time_allowance):
        navigator.get_logger().warn(f'Spin rejected after {label}; continuing')
        return
    while not navigator.isTaskComplete():
        rclpy.spin_once(navigator, timeout_sec=0.5)
    result = navigator.getResult()
    if result != TaskResult.SUCCEEDED:
        navigator.get_logger().warn(
            f'Spin after {label} finished with {result}; continuing')
    time.sleep(1.0)


def yaw_from_pose_stamped(pose):
    return yaw_from_quaternion(pose.pose.orientation)


# Offsets tried when a sequential leg fails (dynamic local rerouting).
REROUTE_OFFSETS = (
    (0.0, -0.12),
    (-0.35, 0.0),
    (-0.25, -0.15),
    (0.0, -0.25),
)


def navigate_single_pose(navigator, pose, step_label):
    navigator.goToPose(pose)
    while not navigator.isTaskComplete():
        rclpy.spin_once(navigator, timeout_sec=0.5)
    return navigator.getResult()


def navigate_single_pose_with_reroute(navigator, pose, step_label):
    """Try primary goal, then offset alternatives with costmap clears."""
    base_x = pose.pose.position.x
    base_y = pose.pose.position.y
    yaw = yaw_from_pose_stamped(pose)

    for attempt, (dx, dy) in enumerate([(0.0, 0.0), *REROUTE_OFFSETS]):
        if attempt > 0:
            navigator.get_logger().warn(
                f'Rerouting {step_label}: retry {attempt} at offset '
                f'({dx:+.2f}, {dy:+.2f}) from ({base_x:.2f}, {base_y:.2f})')
            navigator.clearAllCostmaps()
            time.sleep(0.5)

        goal = make_pose_stamped(navigator, base_x + dx, base_y + dy, yaw)
        result = navigate_single_pose(navigator, goal, step_label)
        if result == TaskResult.SUCCEEDED:
            if attempt > 0:
                navigator.get_logger().info(
                    f'Reroute succeeded for {step_label} on attempt {attempt + 1}')
            return TaskResult.SUCCEEDED

    navigator.get_logger().error(
        f'All reroute attempts failed for {step_label}')
    return TaskResult.FAILED


def navigate_sequential_poses(
        navigator, poses, leg_label, retry_on_fail=False, relocalize_steps=True):
    """Force each pose in order — optional offset rerouting on failure."""
    for step, pose in enumerate(poses, start=1):
        step_label = f'{leg_label} [{step}/{len(poses)}]'
        navigator.get_logger().info(
            f'Sequential leg: {step_label} -> '
            f'({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})')

        if retry_on_fail:
            step_result = navigate_single_pose_with_reroute(
                navigator, pose, step_label)
        else:
            step_result = navigate_single_pose(navigator, pose, step_label)

        if step_result != TaskResult.SUCCEEDED:
            navigator.get_logger().error(
                f'Failed {step_label} (result={step_result})')
            return step_result
        navigator.get_logger().info(f'Reached {step_label}')
        if relocalize_steps and step < len(poses):
            relocalize_at_pose(
                navigator,
                pose.pose.position.x,
                pose.pose.position.y,
                yaw_from_pose_stamped(pose),
                step_label,
            )
    return TaskResult.SUCCEEDED


def navigate_to_pose(navigator, wp, leg_label, profile_applier):
    if not validate_goal_clearance(navigator, wp, leg_label):
        return TaskResult.FAILED

    pause_sec = float(wp.get('pause_before_sec', 0.0))
    if pause_sec > 0.0:
        navigator.get_logger().info(
            f'Pausing {pause_sec:.1f}s before {leg_label} (AMCL/costmap settle)')
        time.sleep(pause_sec)

    profile_name = wp.get('nav_profile', 'default')
    profile_applier.apply(profile_name, leg_label)

    through_poses = wp.get('through_poses')
    if through_poses:
        poses = build_pose_list(navigator, wp)
        if wp.get('force_sequential', False):
            navigator.get_logger().info(
                f'Forced sequential navigation through {len(poses)} poses: {leg_label}')
            return navigate_sequential_poses(
                navigator, poses, leg_label,
                retry_on_fail=wp.get('retry_on_fail', False),
                relocalize_steps=wp.get('relocalize_steps', True))

        navigator.get_logger().info(
            f'Navigating through {len(poses)} poses: {leg_label} '
            f'-> final ({wp["x"]}, {wp["y"]}, yaw={wp["yaw"]})')
        navigator.goThroughPoses(poses)
    else:
        goal = make_pose_stamped(navigator, wp['x'], wp['y'], wp['yaw'])
        if wp.get('retry_on_fail', False):
            navigator.get_logger().info(
                f'Navigating with reroute: {leg_label} at '
                f'({wp["x"]}, {wp["y"]}, yaw={wp["yaw"]})')
            return navigate_single_pose_with_reroute(navigator, goal, leg_label)

        navigator.get_logger().info(
            f'Navigating: {leg_label} at ({wp["x"]}, {wp["y"]}, yaw={wp["yaw"]})')
        navigator.goToPose(goal)

    while not navigator.isTaskComplete():
        rclpy.spin_once(navigator, timeout_sec=0.5)

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
    navigator.declare_parameter('scan_dwell_sec', 2.0)

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
        through = len(wp.get('through_poses', []))
        navigator.get_logger().info(
            f'  [{i}] {wp["name"]}: ({wp["x"]}, {wp["y"]}, yaw={wp["yaw"]}) '
            f'-> expect ArUco id={expected}, nav_profile={profile}, '
            f'through_poses={through}, retreat_after_scan={retreat}')

    initial_pose = make_pose_stamped(
        navigator, initial_pose_cfg['x'], initial_pose_cfg['y'], initial_pose_cfg['yaw'])
    navigator.setInitialPose(initial_pose)

    navigator.get_logger().info('Waiting for Nav2 to become active...')
    navigator.waitUntilNav2Active()

    overall_result = TaskResult.SUCCEEDED

    for index, wp in enumerate(waypoints_cfg):
        latest_markers.pop('data', None)
        leg_label = f'waypoint {index + 1}/{len(waypoints_cfg)}: {wp["name"]}'
        leg_result = navigate_to_pose(navigator, wp, leg_label, profile_applier)
        if leg_result != TaskResult.SUCCEEDED:
            overall_result = leg_result
            break

        if wp.get('skip_scan'):
            navigator.get_logger().info(
                'Clearing local costmap after transit leg (drop stale marks)')
            navigator.clearLocalCostmap()
            time.sleep(0.5)

        if not wp.get('skip_scan'):
            expected = wp.get('expected_aruco_id')
            dwell_for_scan(
                navigator, scan_dwell_sec, expected, latest_markers)
            log_waypoint_scan(
                navigator, wp, latest_markers.get('data'), latest_amcl['pose'])

            spin_relocalize = wp.get(
                'spin_relocalize_after_scan',
                expected is not None,
            )
            if spin_relocalize:
                force_relocalize_via_spin(navigator, wp['name'])

        if wp.get('relocalize_after_scan'):
            relocalize_at_pose(
                navigator, wp['x'], wp['y'], wp['yaw'], wp['name'])

        retreat = wp.get('retreat_after_scan')
        if retreat:
            if wp.get('clear_costmap_before_retreat'):
                navigator.get_logger().info(
                    f'Clearing costmaps before retreat from {wp["name"]}')
                navigator.clearAllCostmaps()
                time.sleep(0.5)
            latest_markers.pop('data', None)
            retreat_wp = {
                'x': retreat['x'],
                'y': retreat['y'],
                'yaw': retreat['yaw'],
                'nav_profile': wp.get('retreat_nav_profile', 'default'),
                'retry_on_fail': wp.get('retreat_retry_on_fail', False),
            }
            retreat_label = f'retreat after {wp["name"]}'
            navigator.get_logger().info(
                f'Backing out of aisle before next waypoint -> '
                f'({retreat_wp["x"]}, {retreat_wp["y"]}, yaw={retreat_wp["yaw"]})')
            retreat_result = navigate_to_pose(
                navigator, retreat_wp, retreat_label, profile_applier)
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
