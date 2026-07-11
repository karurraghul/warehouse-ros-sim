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

_DEBUG_LOG_PATH = '/home/raghul/warehouse-ros-sim/.cursor/debug-bc9dc2.log'
_DEBUG_SESSION_ID = 'bc9dc2'


def _debug_log(hypothesis_id, location, message, data=None, run_id='pre-fix'):
    # #region agent log
    payload = {
        'sessionId': _DEBUG_SESSION_ID,
        'timestamp': int(time.time() * 1000),
        'hypothesisId': hypothesis_id,
        'location': location,
        'message': message,
        'data': data or {},
        'runId': run_id,
    }
    try:
        with open(_DEBUG_LOG_PATH, 'a', encoding='utf-8') as log_file:
            log_file.write(json.dumps(payload) + '\n')
    except OSError:
        pass
    # #endregion

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rcl_interfaces.msg import Parameter as ParamMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from robo_ai_nav.localized_pose import (
    localization_ready,
    pose_from_tf,
    uses_tf_localization,
)
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


AMCL_POSE_QOS = QoSProfile(
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


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


def load_map_bounds(map_yaml_path, margin=0.35):
    """Return map-frame x/y limits that keep the robot footprint inside the costmap."""
    data = load_yaml_file(map_yaml_path)
    origin_x, origin_y = float(data['origin'][0]), float(data['origin'][1])
    resolution = float(data['resolution'])
    image_path = os.path.join(os.path.dirname(map_yaml_path), data['image'])
    with open(image_path, 'rb') as image_file:
        magic = image_file.readline().strip()
        if magic != b'P5':
            raise ValueError(f'Expected P5 PGM map at {image_path}, got {magic!r}')

        line = image_file.readline()
        while line.startswith(b'#'):
            line = image_file.readline()
        width, height = (int(value) for value in line.split())

        maxval_line = image_file.readline()
        while maxval_line.startswith(b'#'):
            maxval_line = image_file.readline()
        int(maxval_line.strip())

    return {
        'x_min': origin_x + margin,
        'x_max': origin_x + width * resolution - margin,
        'y_min': origin_y + margin,
        'y_max': origin_y + height * resolution - margin,
    }


def clamp_to_map(x, y, bounds):
    return (
        max(bounds['x_min'], min(bounds['x_max'], float(x))),
        max(bounds['y_min'], min(bounds['y_max'], float(y))),
    )


def pose_inside_map(x, y, bounds):
    return (
        bounds['x_min'] <= float(x) <= bounds['x_max']
        and bounds['y_min'] <= float(y) <= bounds['y_max']
    )


def validate_mission_poses(initial_pose, waypoints, bounds, logger):
    """Fail fast if any configured pose is outside the static map / global costmap."""
    ok = True

    def check(label, x, y):
        nonlocal ok
        if not pose_inside_map(x, y, bounds):
            logger.error(
                f'{label} ({x}, {y}) is outside map bounds '
                f'x=[{bounds["x_min"]:.2f}, {bounds["x_max"]:.2f}] '
                f'y=[{bounds["y_min"]:.2f}, {bounds["y_max"]:.2f}]')
            ok = False

    check('initial_pose', initial_pose['x'], initial_pose['y'])
    for wp in waypoints:
        check(wp['name'], wp['x'], wp['y'])
        for point in wp.get('through_poses', []):
            check(f'{wp["name"]} through_pose', point['x'], point['y'])
        retreat = wp.get('retreat_after_scan')
        if retreat:
            check(f'{wp["name"]} retreat', retreat['x'], retreat['y'])
    return ok


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


def find_robot_pose_map_from_markers(marker_json, expected_id):
    """Return (x, y, yaw) from ArUco PnP in the latest detection message, if present."""
    if not marker_json or expected_id is None:
        return None
    try:
        payload = json.loads(marker_json)
    except json.JSONDecodeError:
        return None
    for detection in payload.get('detections', []):
        if detection.get('type') != 'aruco':
            continue
        if int(detection.get('id', -1)) != int(expected_id):
            continue
        pose = detection.get('robot_pose_map')
        if not pose:
            return None
        return float(pose['x']), float(pose['y']), float(pose['yaw'])
    return None


def pose_from_amcl(amcl_pose):
    if amcl_pose is None:
        return None
    position = amcl_pose.pose.pose.position
    yaw = yaw_from_quaternion(amcl_pose.pose.pose.orientation)
    return position.x, position.y, yaw


def validate_pnp_against_amcl(pnp_xyyaw, amcl_pose, max_xy_m, max_yaw_rad):
    """Return True when PnP is close enough to AMCL to trust as a relocalize seed."""
    if amcl_pose is None or pnp_xyyaw is None:
        return True
    dist, dyaw = pose_delta_xy_yaw(
        pnp_xyyaw[0], pnp_xyyaw[1], pnp_xyyaw[2], amcl_pose)
    if dist is None:
        return True
    return dist <= max_xy_m and dyaw <= max_yaw_rad


def resolve_relocalize_pose(
        navigator, x_yaml, y_yaml, yaw_yaml, label,
        latest_markers=None, latest_amcl=None, expected_aruco_id=None,
        prefer_marker=False, prefer_amcl=False, amcl_max_age_sec=1.0,
        pnp_max_xy_m=0.4, pnp_max_yaw_rad=0.25, trust_pnp=False):
    """Choose relocalize seed: validated ArUco PnP, optional AMCL, then YAML standoff."""
    amcl_pose = None if latest_amcl is None else latest_amcl.get('pose')
    amcl_fresh = amcl_pose_is_fresh(navigator, amcl_pose, amcl_max_age_sec)
    marker_json = None if latest_markers is None else latest_markers.get('data')

    if prefer_marker and expected_aruco_id is not None:
        pnp_pose = find_robot_pose_map_from_markers(marker_json, expected_aruco_id)
        if pnp_pose is not None:
            pnp_ok = trust_pnp or not amcl_fresh or validate_pnp_against_amcl(
                pnp_pose, amcl_pose, pnp_max_xy_m, pnp_max_yaw_rad)
            if not pnp_ok:
                dist, dyaw = pose_delta_xy_yaw(
                    pnp_pose[0], pnp_pose[1], pnp_pose[2], amcl_pose)
                navigator.get_logger().warn(
                    f'{label}: PnP rejected (delta {dist:.2f}m, {dyaw:.2f}rad vs AMCL); '
                    f'PnP=({pnp_pose[0]:.2f}, {pnp_pose[1]:.2f}, yaw={pnp_pose[2]:.2f})')
            else:
                navigator.get_logger().info(
                    f'{label}: relocalize from ArUco PnP '
                    f'({pnp_pose[0]:.2f}, {pnp_pose[1]:.2f}, yaw={pnp_pose[2]:.2f})')
                return pnp_pose[0], pnp_pose[1], pnp_pose[2], 'pnp'

    if prefer_amcl:
        if amcl_fresh:
            amcl_xyyaw = pose_from_amcl(amcl_pose)
            if amcl_xyyaw is not None:
                navigator.get_logger().info(
                    f'{label}: relocalize from AMCL '
                    f'({amcl_xyyaw[0]:.2f}, {amcl_xyyaw[1]:.2f}, yaw={amcl_xyyaw[2]:.2f})')
                return amcl_xyyaw[0], amcl_xyyaw[1], amcl_xyyaw[2], 'amcl'
        navigator.get_logger().warn(
            f'{label}: no fresh AMCL pose for relocalize '
            f'(max age {amcl_max_age_sec:.1f}s); falling back to YAML')
    elif prefer_marker and not amcl_fresh:
        navigator.get_logger().warn(
            f'{label}: no fresh pose for PnP validation '
            f'(max age {amcl_max_age_sec:.1f}s); falling back to YAML')

    navigator.get_logger().info(
        f'{label}: relocalize fallback to YAML '
        f'({x_yaml:.2f}, {y_yaml:.2f}, yaw={yaw_yaml:.2f})')
    return float(x_yaml), float(y_yaml), float(yaw_yaml), 'yaml'


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


def make_double_array_param(name, values):
    msg = ParamMsg()
    msg.name = name
    msg.value = ParameterValue(
        type=ParameterType.PARAMETER_DOUBLE_ARRAY,
        double_array_value=[float(v) for v in values],
    )
    return msg


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def pose_delta_xy_yaw(target_x, target_y, target_yaw, amcl_pose):
    if amcl_pose is None:
        return None, None
    position = amcl_pose.pose.pose.position
    orientation = amcl_pose.pose.pose.orientation
    dist = math.hypot(
        float(target_x) - position.x,
        float(target_y) - position.y,
    )
    dyaw = abs(normalize_angle(
        float(target_yaw) - yaw_from_quaternion(orientation)))
    return dist, dyaw


def should_skip_sequential_step(amcl_pose, goal_pose, xy_tol, yaw_tol, xy_only=False):
    """Skip redundant sequential legs when the robot is already at the goal."""
    goal_x = goal_pose.pose.position.x
    goal_y = goal_pose.pose.position.y
    goal_yaw = yaw_from_pose_stamped(goal_pose)
    dist, dyaw = pose_delta_xy_yaw(goal_x, goal_y, goal_yaw, amcl_pose)
    if dist is None:
        return False, None
    metrics = {'dist_m': round(dist, 3), 'dyaw_rad': round(dyaw, 3), 'xy_only': xy_only}
    if dist > xy_tol:
        return False, metrics
    if xy_only or dyaw <= yaw_tol:
        return True, metrics
    return False, metrics


def amcl_pose_is_fresh(navigator, amcl_pose, max_age_sec=1.0):
    if amcl_pose is None:
        return False
    stamp = rclpy.time.Time.from_msg(amcl_pose.header.stamp)
    age_sec = (navigator.get_clock().now() - stamp).nanoseconds / 1e9
    return age_sec <= max_age_sec


class NavProfileApplier:
    """Apply Nav2 costmap/controller params for per-waypoint navigation profiles."""

    COSTMAP_NODES = (
        '/local_costmap/local_costmap',
        '/global_costmap/global_costmap',
    )

    def __init__(self, node, profiles, map_bounds=None, localization_mode='amcl'):
        self._node = node
        self._profiles = profiles
        self._clients = {}
        self.map_bounds = map_bounds
        self._localization_mode = localization_mode

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
        params = []
        for name, value in param_dict.items():
            if isinstance(value, (list, tuple)):
                params.append(make_double_array_param(name, value))
            else:
                params.append(make_param(name, value))
        request.parameters = params
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
        max_vel_x = float(profile.get('max_vel_x', 0.26))
        # A profile explicitly asking for live obstacles (e.g. "narrow" for
        # the dock corridor, where the static SLAM map has unmapped shadow
        # gaps behind real props) always wins, even in SLAM localization
        # mode. Otherwise SLAM modes default to static-only global costmap
        # to avoid drift-smeared live scans poisoning the global plan.
        live_obstacles = profile.get('use_live_global_obstacles', False)
        static_map_only = profile.get('static_map_only', False)
        if live_obstacles:
            static_map_only = False
        elif self._localization_mode in ('slam_online', 'slam_localization'):
            static_map_only = True
        local_static_only = profile.get('local_static_only', False)
        if live_obstacles:
            local_static_only = False
        elif (self._localization_mode in ('slam_online', 'slam_localization')
                and profile_name == 'narrow'):
            local_static_only = True
        self._node.get_logger().info(
            f"Applying nav profile '{profile_name}' for {waypoint_name} "
            f'(inflation={inflation}, cost_scale={cost_scale}, '
            f'obstacle_scale={obstacle_scale}, max_vel_x={max_vel_x}, '
            f'static_map_only={static_map_only}, '
            f'local_static_only={local_static_only}, xy_tol={xy_tol})')

        costmap_params = {
            'inflation_layer.inflation_radius': inflation,
            'inflation_layer.cost_scaling_factor': cost_scale,
        }
        global_extra = {}
        if static_map_only:
            global_extra['obstacle_layer.enabled'] = False
        else:
            global_extra['obstacle_layer.enabled'] = True
        ok = True
        for node_name in self.COSTMAP_NODES:
            node_params = dict(costmap_params)
            if node_name.endswith('global_costmap'):
                node_params.update(global_extra)
            if local_static_only and node_name.endswith('local_costmap'):
                node_params['voxel_layer.enabled'] = False
            ok = self._set_params(node_name, node_params) and ok
        ok = self._set_params(
            '/controller_server',
            {
                'general_goal_checker.xy_goal_tolerance': xy_tol,
                # RPP (Regulated Pure Pursuit) speed knob. The old DWB keys
                # (FollowPath.max_vel_x / max_speed_xy / BaseObstacle.scale)
                # don't exist on RPP; obstacle_scale from the profile is now
                # unused since RPP handles proximity via cost-regulated
                # velocity scaling instead of a critic weight.
                'FollowPath.desired_linear_vel': max_vel_x,
            },
        ) and ok
        ok = self._set_params(
            '/velocity_smoother',
            {'max_velocity': [max_vel_x, 0.0, 1.0]},
        ) and ok
        return ok

    def apply_slam_global_costmap(self):
        """Use live SLAM /map via static_layer only; disable global scan marks."""
        if self._localization_mode not in ('slam_online', 'slam_localization'):
            return True
        return self.apply('default', 'slam_startup')

    def restore_default(self):
        ok = self.apply('default', 'restore')
        if self._localization_mode in ('slam_online', 'slam_localization'):
            return ok
        return self._set_params(
            '/global_costmap/global_costmap',
            {'obstacle_layer.enabled': True},
        ) and self._set_params(
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


def amcl_settle_dwell(navigator, seconds, label):
    """Hold still and process lidar so AMCL can scan-match before the next leg."""
    navigator.get_logger().info(
        f'AMCL settle dwell {seconds:.1f}s after {label}')
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(navigator, timeout_sec=0.1)


def probe_path_to_pose(navigator, goal_pose, timeout_sec=5.0):
    """Return True if planner_server can reach goal_pose."""
    client = ActionClient(navigator, ComputePathToPose, 'compute_path_to_pose')
    if not client.wait_for_server(timeout_sec=timeout_sec):
        navigator.get_logger().warn('compute_path_to_pose action server unavailable')
        return False

    request = ComputePathToPose.Goal()
    request.goal = goal_pose
    request.planner_id = 'GridBased'
    request.use_start = False

    send_future = client.send_goal_async(request)
    rclpy.spin_until_future_complete(navigator, send_future, timeout_sec=timeout_sec)
    if not send_future.done() or send_future.result() is None:
        return False

    goal_handle = send_future.result()
    if not goal_handle.accepted:
        return False

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(navigator, result_future, timeout_sec=timeout_sec)
    if not result_future.done() or result_future.result() is None:
        return False

    status = result_future.result().status
    return status == GoalStatus.STATUS_SUCCEEDED


def mapping_spin(navigator, radians, angular_speed=0.35):
    """Rotate in place so SLAM can expand mapped free space."""
    if abs(radians) < 1e-3:
        return

    pub = navigator.create_publisher(Twist, '/cmd_vel', 10)
    twist = Twist()
    twist.angular.z = angular_speed if radians >= 0.0 else -angular_speed
    duration = abs(radians) / abs(angular_speed)
    navigator.get_logger().info(
        f'SLAM mapping spin {math.degrees(radians):.0f} deg '
        f'at {abs(angular_speed):.2f} rad/s')
    deadline = time.monotonic() + duration + 0.5
    while time.monotonic() < deadline:
        if time.monotonic() < deadline - 0.5:
            pub.publish(twist)
        else:
            stop = Twist()
            pub.publish(stop)
        rclpy.spin_once(navigator, timeout_sec=0.05)
    time.sleep(0.3)


WEDGE_STALL_SEC = 18.0
WEDGE_MIN_DISPLACEMENT_M = 0.06
WEDGE_MAX_BACKUPS = 2
WEDGE_CANCEL_SETTLE_SEC = 0.5


def amcl_xy_from_latest(latest_amcl):
    if latest_amcl is None:
        return None
    pose = latest_amcl.get('pose')
    if pose is None:
        return None
    position = pose.pose.pose.position
    return position.x, position.y


def displacement_from_start(latest_amcl, start_xy):
    current = amcl_xy_from_latest(latest_amcl)
    if current is None or start_xy is None:
        return None
    return math.hypot(current[0] - start_xy[0], current[1] - start_xy[1])


def odom_xy_from_msg(odom_msg):
    if odom_msg is None:
        return None
    position = odom_msg.pose.pose.position
    return position.x, position.y


def odom_displacement_m(start_xy, current_xy):
    if start_xy is None or current_xy is None:
        return None
    return math.hypot(current_xy[0] - start_xy[0], current_xy[1] - start_xy[1])


def drive_wedge_backup(navigator, distance_m=0.25, speed_mps=0.20):
    """Reverse briefly to break stiction when wedged; matches manual teleop fix."""
    pub_nav = navigator.create_publisher(Twist, 'cmd_vel_nav', 10)
    pub_vel = navigator.create_publisher(Twist, '/cmd_vel', 10)
    twist = Twist()
    twist.linear.x = -abs(speed_mps)
    duration = distance_m / abs(speed_mps)
    navigator.get_logger().warn(
        f'Wedge escape: backing up {distance_m:.2f}m at {speed_mps:.2f} m/s')
    # #region agent log
    _debug_log(
        'H11',
        'waypoint_navigator.py:drive_wedge_backup',
        'manual_wedge_backup',
        {'distance_m': distance_m, 'speed_mps': speed_mps, 'topic': 'cmd_vel_nav'},
        run_id='post-fix',
    )
    # #endregion
    deadline = time.monotonic() + duration + 0.5
    while time.monotonic() < deadline:
        if time.monotonic() < deadline - 0.5:
            pub_nav.publish(twist)
            pub_vel.publish(twist)
        else:
            stop = Twist()
            pub_nav.publish(stop)
            pub_vel.publish(stop)
        rclpy.spin_once(navigator, timeout_sec=0.05)
    stop = Twist()
    pub_nav.publish(stop)
    pub_vel.publish(stop)
    time.sleep(0.3)


def wait_for_slam_planner_ready(navigator, first_wp, profile_applier,
                                timeout_sec=60.0, spin_rad=1.57,
                                min_warmup_sec=0.0):
    """Wait until Navfn can plan to the first waypoint; spin to map if needed."""
    profile_applier.apply_slam_global_costmap()
    if min_warmup_sec > 0.0:
        amcl_settle_dwell(navigator, min_warmup_sec, 'slam_online map warmup')

    goal = make_pose_stamped(navigator, first_wp['x'], first_wp['y'], first_wp['yaw'])
    navigator.get_logger().info(
        f'Waiting for planner path to first waypoint '
        f'({first_wp["x"]}, {first_wp["y"]}) (timeout={timeout_sec:.0f}s)...')

    deadline = time.monotonic() + timeout_sec
    attempt = 0
    while time.monotonic() < deadline:
        attempt += 1
        if probe_path_to_pose(navigator, goal, timeout_sec=5.0):
            navigator.get_logger().info(
                f'SLAM planner ready for {first_wp["name"]} after {attempt} attempt(s)')
            return True

        remaining = deadline - time.monotonic()
        navigator.get_logger().info(
            f'SLAM planner not ready (attempt {attempt}); '
            f'mapping spin then retry ({remaining:.0f}s left)')
        if remaining <= spin_rad / 0.35 + 2.0:
            break
        mapping_spin(navigator, spin_rad)
        amcl_settle_dwell(navigator, 1.0, 'post mapping spin')

    navigator.get_logger().error(
        f'SLAM planner could not reach first waypoint {first_wp["name"]} '
        f'within {timeout_sec:.0f}s')
    return False


def wait_for_localization(navigator, ready_state, localization_mode, tf_buffer,
                          timeout_sec=60.0):
    """Wait for AMCL pose or map->base_footprint TF before starting the mission."""
    if ready_state['value']:
        navigator.initial_pose_received = True
        return True

    label = localization_mode
    navigator.get_logger().info(
        f'Waiting for localization ({label}) from initial_pose_publisher...')
    deadline = time.monotonic() + timeout_sec
    while not ready_state['value'] and time.monotonic() < deadline:
        if uses_tf_localization(localization_mode):
            pose = pose_from_tf(tf_buffer)
            if pose is not None:
                ready_state['value'] = True
                navigator.initial_pose_received = True
                break
        rclpy.spin_once(navigator, timeout_sec=0.5)

    if not ready_state['value']:
        navigator.get_logger().error(
            f'Localization ({label}) not ready within {timeout_sec:.0f}s; '
            'check initial_pose_publisher, /scan, and Nav2 lifecycle.')
        return False

    navigator.initial_pose_received = True
    navigator.get_logger().info(f'Localization confirmed ({label}).')
    return True


def wait_for_amcl_localization(navigator, amcl_ready, timeout_sec=60.0):
    """Backward-compatible wrapper for AMCL-only wait."""
    return wait_for_localization(
        navigator, amcl_ready, 'amcl', None, timeout_sec=timeout_sec)


def wait_for_nav2_active(navigator, timeout_sec=120.0, localization_mode='amcl'):
    """Wait for Nav2 action servers without re-publishing /initialpose."""
    navigator.get_logger().info('Waiting for Nav2 lifecycle nodes to activate...')
    deadline = time.monotonic() + timeout_sec
    lifecycle_nodes = ['bt_navigator']
    if localization_mode == 'amcl':
        lifecycle_nodes.insert(0, 'amcl')
    for node_name in lifecycle_nodes:
        service = f'{node_name}/get_state'
        client = navigator.create_client(GetState, service)
        while not client.wait_for_service(timeout_sec=1.0):
            if time.monotonic() > deadline:
                navigator.get_logger().error(f'Timeout waiting for {service}')
                return False
            rclpy.spin_once(navigator, timeout_sec=0.5)

        req = GetState.Request()
        state = 'unknown'
        while state != 'active':
            if time.monotonic() > deadline:
                navigator.get_logger().error(
                    f'Timeout waiting for {node_name} to become active (last={state})')
                return False
            future = client.call_async(req)
            rclpy.spin_until_future_complete(navigator, future, timeout_sec=2.0)
            if future.result() is not None:
                state = future.result().current_state.label
            rclpy.spin_once(navigator, timeout_sec=0.1)

    navigator.get_logger().info('Nav2 is ready for use!')
    return True


def build_pose_list(navigator, wp):
    """Build Nav2 pose list: optional through_poses then final x/y/yaw."""
    poses = []
    for point in wp.get('through_poses', []):
        poses.append(make_pose_stamped(
            navigator, point['x'], point['y'], point['yaw']))
    poses.append(make_pose_stamped(navigator, wp['x'], wp['y'], wp['yaw']))
    return poses


def build_sequential_steps(navigator, wp, default_profile='default'):
    """Build ordered sequential legs with optional per-step dwell_after_sec."""
    steps = []
    for point in wp.get('through_poses', []):
        steps.append({
            'pose': make_pose_stamped(navigator, point['x'], point['y'], point['yaw']),
            'dwell_after_sec': float(point.get('dwell_after_sec', 0.0)),
            'relocalize_after_dwell': bool(point.get('relocalize_after_dwell', False)),
            'relocalize_force': bool(point.get('relocalize_force', False)),
            'nav_profile': point.get('nav_profile', default_profile),
        })
    steps.append({
        'pose': make_pose_stamped(navigator, wp['x'], wp['y'], wp['yaw']),
        'dwell_after_sec': float(wp.get('dwell_after_sec', 0.0)),
        'relocalize_after_dwell': bool(wp.get('relocalize_after_dwell', False)),
        'relocalize_force': bool(wp.get('relocalize_force', False)),
        'nav_profile': wp.get('nav_profile', default_profile),
    })
    return steps


def relocalize_cfg_kwargs(relocalize_cfg):
    return {
        'latest_amcl': relocalize_cfg.get('latest_amcl'),
        'min_xy_m': relocalize_cfg.get('min_xy_m', 0.15),
        'min_yaw_rad': relocalize_cfg.get('min_yaw_rad', 0.15),
        'max_xy_m': relocalize_cfg.get('max_xy_m', 1.0),
        'max_yaw_rad': relocalize_cfg.get('max_yaw_rad', 0.52),
        'settle_sec': relocalize_cfg.get('settle_sec', 1.5),
        'amcl_max_age_sec': relocalize_cfg.get('amcl_max_age_sec', 1.0),
        'post_settle_sec': relocalize_cfg.get('post_settle_sec', 2.0),
        'localization_mode': relocalize_cfg.get('localization_mode', 'amcl'),
    }


def try_mid_leg_pnp_snap(navigator, step_label, relocalize_cfg, map_bounds):
    """If dock marker is visible, snap localization from ArUco PnP before the next leg."""
    if not relocalize_cfg.get('mid_leg_pnp_snap'):
        return
    expected_id = relocalize_cfg.get('expected_aruco_id')
    if expected_id is None:
        return
    latest_markers = relocalize_cfg.get('latest_markers')
    pnp_pose = find_robot_pose_map_from_markers(
        None if latest_markers is None else latest_markers.get('data'),
        expected_id,
    )
    if pnp_pose is None:
        return
    navigator.get_logger().info(
        f'{step_label}: mid-leg ArUco PnP snap id={expected_id} '
        f'({pnp_pose[0]:.2f}, {pnp_pose[1]:.2f}, yaw={pnp_pose[2]:.2f})')
    relocalize_at_pose(
        navigator,
        pnp_pose[0],
        pnp_pose[1],
        pnp_pose[2],
        f'{step_label} (mid-leg PnP)',
        map_bounds,
        force=True,
        **relocalize_cfg_kwargs(relocalize_cfg),
    )


def finish_relocalize(navigator, label, post_settle_sec=2.0):
    """Let localization absorb /initialpose, refresh costmaps, then hold for scan matching."""
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        rclpy.spin_once(navigator, timeout_sec=0.1)
    navigator.clearAllCostmaps()
    if post_settle_sec > 0.0:
        amcl_settle_dwell(navigator, post_settle_sec, f'{label} (post-relocalize)')


def relocalize_at_pose(
        navigator, x, y, yaw, label, map_bounds=None, latest_amcl=None,
        min_xy_m=0.15, min_yaw_rad=0.15, max_xy_m=1.0, max_yaw_rad=0.52,
        settle_sec=1.5, amcl_max_age_sec=1.0, post_settle_sec=2.0,
        force=False, reseed_only=False, localization_mode='amcl'):
    """Reseed localization at a map pose when drift is moderate."""
    if map_bounds is not None:
        raw_x, raw_y = x, y
        x, y = clamp_to_map(x, y, map_bounds)
        if (x, y) != (raw_x, raw_y):
            navigator.get_logger().warn(
                f'Clamped relocalize pose for {label} from '
                f'({raw_x:.2f}, {raw_y:.2f}) to ({x:.2f}, {y:.2f})')

    amcl_pose = None if latest_amcl is None else latest_amcl.get('pose')
    if not amcl_pose_is_fresh(navigator, amcl_pose, amcl_max_age_sec):
        if force:
            # Give the pose source (AMCL or slam_toolbox TF poll) a short
            # window to catch up before trusting an unchecked publish.
            # Without this, a momentarily stale pose (TF backlog, sim lag)
            # combined with force=True used to fall straight through to
            # setInitialPose() with dist/dyaw = inf, i.e. no plausibility
            # check at all — this is what caused waypoints to jump on scans.
            navigator.get_logger().warn(
                f'Forced relocalize for {label}: /amcl_pose stale '
                f'(max age {amcl_max_age_sec:.1f}s); waiting {settle_sec:.1f}s '
                f'for a fresh pose before publishing')
            amcl_settle_dwell(navigator, settle_sec, label)
            amcl_pose = None if latest_amcl is None else latest_amcl.get('pose')
            if not amcl_pose_is_fresh(navigator, amcl_pose, amcl_max_age_sec):
                navigator.get_logger().warn(
                    f'Skipping forced relocalize for {label}: pose still stale '
                    f'after settle (TF/localization may be backlogged)')
                return
        else:
            navigator.get_logger().warn(
                f'Skipping relocalize for {label}: no fresh /amcl_pose '
                f'(max age {amcl_max_age_sec:.1f}s)')
            return

    dist, dyaw = pose_delta_xy_yaw(x, y, yaw, amcl_pose)
    if dist is None:
        # No reference pose at all (not just stale) — still refuse to
        # publish blind even when force=True; an unconditional inf/inf
        # delta bypasses every downstream sanity check below.
        navigator.get_logger().warn(
            f'Skipping relocalize for {label}: no pose reference available '
            f'(cannot validate target against current estimate)')
        return

    if dist < min_xy_m and dyaw < min_yaw_rad and not reseed_only:
        navigator.get_logger().info(
            f'Skipping relocalize for {label}: already close to target '
            f'(delta {dist:.2f}m, {dyaw:.2f}rad)')
        return

    if not force and not reseed_only and (dist > max_xy_m or dyaw > max_yaw_rad):
        navigator.get_logger().warn(
            f'Large AMCL delta before relocalize for {label}: '
            f'{dist:.2f}m, {dyaw:.2f}rad — waiting {settle_sec:.1f}s for scans')
        amcl_settle_dwell(navigator, settle_sec, label)
        amcl_pose = latest_amcl.get('pose')
        if not amcl_pose_is_fresh(navigator, amcl_pose, amcl_max_age_sec):
            navigator.get_logger().warn(
                f'Skipping relocalize for {label}: /amcl_pose stale after settle')
            return
        dist, dyaw = pose_delta_xy_yaw(x, y, yaw, amcl_pose)
        if dist > max_xy_m or dyaw > max_yaw_rad:
            navigator.get_logger().warn(
                f'Skipping relocalize for {label}: delta still '
                f'{dist:.2f}m, {dyaw:.2f}rad after settle (scan/TF may be backlogged)')
            return
    elif force and (dist > max_xy_m or dyaw > max_yaw_rad):
        # Forced relocalize deliberately overrides the normal max-delta
        # guard (used at planned drift-correction checkpoints), but a
        # single bad PnP read (glare/occlusion) should still be caught —
        # an absolute ceiling well beyond any planned correction, not the
        # tunable max_xy_m/max_yaw_rad which force is meant to bypass.
        hard_xy_ceiling = max(3.0, max_xy_m * 3.0)
        hard_yaw_ceiling = max(1.57, max_yaw_rad * 3.0)
        if dist > hard_xy_ceiling or dyaw > hard_yaw_ceiling:
            navigator.get_logger().error(
                f'Rejecting forced relocalize for {label}: delta '
                f'{dist:.2f}m, {dyaw:.2f}rad exceeds hard ceiling '
                f'{hard_xy_ceiling:.2f}m/{hard_yaw_ceiling:.2f}rad — '
                f'likely a bad PnP read, not real drift')
            return
        navigator.get_logger().warn(
            f'Forced relocalize for {label} despite large delta '
            f'{dist:.2f}m, {dyaw:.2f}rad')

    action = 'Reseeding' if reseed_only else ('Forced relocalizing' if force else 'Relocalizing')
    backend = 'slam_toolbox' if uses_tf_localization(localization_mode) else 'AMCL'
    navigator.get_logger().info(
        f'{action} {backend} after {label} at ({x:.2f}, {y:.2f}, yaw={yaw:.2f}) '
        f'(delta {dist:.2f}m, {dyaw:.2f}rad)')
    navigator.setInitialPose(make_pose_stamped(navigator, x, y, yaw))
    finish_relocalize(navigator, label, post_settle_sec)


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


def navigate_single_pose(navigator, pose, step_label, latest_amcl=None):
    wedge_backup_count = 0
    odom_state = {'msg': None}

    def on_odom(msg):
        odom_state['msg'] = msg

    odom_sub = navigator.create_subscription(Odometry, '/odom', on_odom, 10)

    def start_navigation():
        navigator.goToPose(pose)

    try:
        start_navigation()
        # #region agent log
        started_at = time.monotonic()
        last_progress_log = started_at
        odom_deadline = time.monotonic() + 1.0
        while odom_state['msg'] is None and time.monotonic() < odom_deadline:
            rclpy.spin_once(navigator, timeout_sec=0.1)
        start_odom_xy = odom_xy_from_msg(odom_state['msg'])
        _debug_log(
            'H3',
            'waypoint_navigator.py:navigate_single_pose',
            'sequential_leg_started',
            {
                'step_label': step_label,
                'goal_x': round(pose.pose.position.x, 3),
                'goal_y': round(pose.pose.position.y, 3),
            },
        )
        # #endregion
        while not navigator.isTaskComplete():
            rclpy.spin_once(navigator, timeout_sec=0.5)
            now = time.monotonic()
            elapsed = now - started_at
            displacement = odom_displacement_m(
                start_odom_xy, odom_xy_from_msg(odom_state['msg']))
            if (
                wedge_backup_count < WEDGE_MAX_BACKUPS
                and elapsed >= WEDGE_STALL_SEC
                and displacement is not None
                and displacement < WEDGE_MIN_DISPLACEMENT_M
            ):
                wedge_backup_count += 1
                navigator.get_logger().warn(
                    f'Wedge stall on {step_label}: {elapsed:.0f}s, '
                    f'{displacement:.3f}m odom moved; canceling Nav2 and backing up '
                    f'({wedge_backup_count}/{WEDGE_MAX_BACKUPS})')
                # #region agent log
                _debug_log(
                    'H11',
                    'waypoint_navigator.py:navigate_single_pose',
                    'wedge_stall_detected',
                    {
                        'step_label': step_label,
                        'elapsed_sec': round(elapsed, 1),
                        'displacement_m': round(displacement, 3),
                        'displacement_source': 'odom',
                        'wedge_backup_count': wedge_backup_count,
                    },
                    run_id='post-fix',
                )
                # #endregion
                navigator.cancelTask()
                while not navigator.isTaskComplete():
                    rclpy.spin_once(navigator, timeout_sec=0.5)
                settle_deadline = time.monotonic() + WEDGE_CANCEL_SETTLE_SEC
                while time.monotonic() < settle_deadline:
                    rclpy.spin_once(navigator, timeout_sec=0.05)
                drive_wedge_backup(navigator)
                started_at = time.monotonic()
                last_progress_log = started_at
                start_odom_xy = odom_xy_from_msg(odom_state['msg'])
                start_navigation()
                continue
            # #region agent log
            if now - last_progress_log >= 15.0:
                last_progress_log = now
                _debug_log(
                    'H2',
                    'waypoint_navigator.py:navigate_single_pose',
                    'sequential_leg_still_running',
                    {'step_label': step_label, 'elapsed_sec': round(elapsed, 1)},
                )
            # #endregion
        result = navigator.getResult()
        # #region agent log
        _debug_log(
            'H3',
            'waypoint_navigator.py:navigate_single_pose',
            'sequential_leg_finished',
            {
                'step_label': step_label,
                'elapsed_sec': round(time.monotonic() - started_at, 1),
                'result': str(result),
            },
        )
        # #endregion
        return result
    finally:
        navigator.destroy_subscription(odom_sub)


def navigate_single_pose_with_reroute(navigator, pose, step_label, latest_amcl=None):
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
        result = navigate_single_pose(
            navigator, goal, step_label, latest_amcl=latest_amcl)
        if result == TaskResult.SUCCEEDED:
            if attempt > 0:
                navigator.get_logger().info(
                    f'Reroute succeeded for {step_label} on attempt {attempt + 1}')
            return TaskResult.SUCCEEDED

    navigator.get_logger().error(
        f'All reroute attempts failed for {step_label}')
    return TaskResult.FAILED


def navigate_sequential_poses(
        navigator, steps, leg_label, profile_applier, default_profile='default',
        retry_on_fail=False, relocalize_steps=False,
        map_bounds=None, relocalize_cfg=None, goal_xy_tol=0.25, goal_yaw_tol=0.25):
    """Force each pose in order — optional offset rerouting on failure."""
    relocalize_cfg = relocalize_cfg or {}
    latest_amcl = relocalize_cfg.get('latest_amcl')
    for step, entry in enumerate(steps, start=1):
        pose = entry['pose']
        step_label = f'{leg_label} [{step}/{len(steps)}]'
        step_profile = entry.get('nav_profile', default_profile)
        profile_applier.apply(step_profile, step_label)
        step_xy_tol = float(
            profile_applier._profiles.get(
                step_profile, profile_applier._profiles['default']
            ).get('xy_goal_tolerance', goal_xy_tol))
        try_mid_leg_pnp_snap(navigator, step_label, relocalize_cfg, map_bounds)

        amcl_pose = None if latest_amcl is None else latest_amcl.get('pose')
        is_final = step == len(steps)
        skip, metrics = should_skip_sequential_step(
            amcl_pose,
            pose,
            step_xy_tol,
            goal_yaw_tol,
            xy_only=not is_final,
        )
        if skip:
            navigator.get_logger().info(
                f'Skipping {step_label}: already within tolerance '
                f'(dist={metrics["dist_m"]:.2f}m, dyaw={metrics["dyaw_rad"]:.2f}rad, '
                f'xy_only={metrics["xy_only"]})')
            # #region agent log
            _debug_log(
                'H6',
                'waypoint_navigator.py:navigate_sequential_poses',
                'sequential_step_skipped',
                {
                    'step_label': step_label,
                    'goal_x': round(pose.pose.position.x, 3),
                    'goal_y': round(pose.pose.position.y, 3),
                    **metrics,
                },
            )
            # #endregion
        else:
            navigator.get_logger().info(
                f'Sequential leg: {step_label} -> '
                f'({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})')

            if retry_on_fail:
                step_result = navigate_single_pose_with_reroute(
                    navigator, pose, step_label, latest_amcl=latest_amcl)
            else:
                step_result = navigate_single_pose(
                    navigator, pose, step_label, latest_amcl=latest_amcl)

            if step_result != TaskResult.SUCCEEDED:
                navigator.get_logger().error(
                    f'Failed {step_label} (result={step_result})')
                return step_result
            navigator.get_logger().info(f'Reached {step_label}')

        dwell_after = float(entry.get('dwell_after_sec', 0.0))
        if dwell_after > 0.0:
            amcl_settle_dwell(navigator, dwell_after, step_label)

        if entry.get('relocalize_after_dwell'):
            anchor_x = pose.pose.position.x
            anchor_y = pose.pose.position.y
            anchor_yaw = yaw_from_pose_stamped(pose)
            x, y, yaw, source = resolve_relocalize_pose(
                navigator,
                anchor_x,
                anchor_y,
                anchor_yaw,
                step_label,
                latest_markers=relocalize_cfg.get('latest_markers'),
                latest_amcl=relocalize_cfg.get('latest_amcl'),
                expected_aruco_id=entry.get('expected_aruco_id'),
                prefer_marker=True,
                prefer_amcl=False,
                amcl_max_age_sec=relocalize_cfg.get('amcl_max_age_sec', 1.0),
                trust_pnp=entry.get('relocalize_force', True),
            )
            loc_mode = relocalize_cfg.get('localization_mode', 'amcl')
            if source == 'yaml' and loc_mode in ('slam_online', 'slam_localization'):
                # No validated PnP was available, so the only "correction"
                # on offer is snapping belief to the hardcoded YAML anchor.
                # In SLAM modes the scan matcher is already tracking pose
                # continuously — a forced YAML snap injects up to ~0.25m of
                # error, drags the robot off its own path, and misplaces
                # live scans against the map (phantom inflation). Every
                # observed mid-mission stall followed one of these snaps.
                navigator.get_logger().info(
                    f'{step_label}: skipping YAML relocalize in {loc_mode} '
                    f'(scan matcher already tracking; no validated PnP)')
            else:
                force_dwell = (
                    entry.get('relocalize_force', True) or source == 'yaml')
                relocalize_at_pose(
                    navigator,
                    x,
                    y,
                    yaw,
                    step_label,
                    map_bounds,
                    force=force_dwell,
                    reseed_only=False,
                    **relocalize_cfg_kwargs(relocalize_cfg),
                )

        if relocalize_steps and step < len(steps):
            relocalize_at_pose(
                navigator,
                pose.pose.position.x,
                pose.pose.position.y,
                yaw_from_pose_stamped(pose),
                step_label,
                map_bounds,
                **relocalize_cfg_kwargs(relocalize_cfg),
            )
    return TaskResult.SUCCEEDED


def apply_relocalize_before(navigator, wp, leg_label, map_bounds, relocalize_cfg):
    anchor = wp.get('relocalize_before')
    if not anchor:
        return
    relocalize_at_pose(
        navigator,
        anchor['x'],
        anchor['y'],
        anchor['yaw'],
        f'{leg_label} (pre-leg anchor)',
        map_bounds,
        force=anchor.get('force', True),
        **relocalize_cfg_kwargs(relocalize_cfg),
    )


def navigate_to_pose(navigator, wp, leg_label, profile_applier, relocalize_cfg=None):
    if not validate_goal_clearance(navigator, wp, leg_label):
        return TaskResult.FAILED

    relocalize_cfg = relocalize_cfg or {}

    pause_sec = float(wp.get('pause_before_sec', 0.0))
    if pause_sec > 0.0:
        navigator.get_logger().info(
            f'Pausing {pause_sec:.1f}s before {leg_label} (AMCL/costmap settle)')
        time.sleep(pause_sec)

    apply_relocalize_before(
        navigator, wp, leg_label, profile_applier.map_bounds, relocalize_cfg)

    if wp.get('clear_costmap_before_leg'):
        navigator.get_logger().info(f'Clearing costmaps before {leg_label}')
        navigator.clearAllCostmaps()
        time.sleep(0.5)

    profile_name = wp.get('nav_profile', 'default')
    profile_applier.apply(profile_name, leg_label)

    through_poses = wp.get('through_poses')
    if through_poses:
        poses = build_pose_list(navigator, wp)
        if wp.get('force_sequential', False):
            steps = build_sequential_steps(navigator, wp, profile_name)
            profile = profile_applier._profiles.get(
                profile_name, profile_applier._profiles['default'])
            navigator.get_logger().info(
                f'Forced sequential navigation through {len(steps)} poses: {leg_label}')
            return navigate_sequential_poses(
                navigator, steps, leg_label, profile_applier, profile_name,
                retry_on_fail=wp.get('retry_on_fail', False),
                relocalize_steps=wp.get('relocalize_steps', False),
                map_bounds=profile_applier.map_bounds,
                goal_xy_tol=float(profile['xy_goal_tolerance']),
                goal_yaw_tol=0.25,
                relocalize_cfg={
                    **relocalize_cfg,
                    'mid_leg_pnp_snap': wp.get('mid_leg_pnp_snap', False),
                    'expected_aruco_id': wp.get('expected_aruco_id'),
                    'latest_markers': relocalize_cfg.get('latest_markers'),
                })

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
            return navigate_single_pose_with_reroute(
                navigator, goal, leg_label,
                latest_amcl=relocalize_cfg.get('latest_amcl'))

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
    navigator.get_logger().info('waypoint_navigator starting...')
    robo_ai_share = get_package_share_directory('robo_ai')
    pkg_share = get_package_share_directory('robo_ai_nav')
    map_yaml = os.path.join(robo_ai_share, 'maps', 'warehouse_map.yaml')
    map_bounds = load_map_bounds(map_yaml)
    navigator.declare_parameter(
        'waypoints_file',
        os.path.join(pkg_share, 'config', 'waypoints.yaml'))
    navigator.declare_parameter(
        'nav_profiles_file',
        os.path.join(pkg_share, 'config', 'nav_profiles.yaml'))
    navigator.declare_parameter('shutdown_nav2_on_exit', True)
    navigator.declare_parameter('shutdown_nav2_on_failure', False)
    navigator.declare_parameter('scan_dwell_sec', 2.0)
    navigator.declare_parameter('relocalize_min_xy_m', 0.15)
    navigator.declare_parameter('relocalize_min_yaw_rad', 0.15)
    navigator.declare_parameter('relocalize_max_xy_m', 1.0)
    navigator.declare_parameter('relocalize_max_yaw_rad', 0.52)
    navigator.declare_parameter('relocalize_settle_sec', 1.5)
    navigator.declare_parameter('relocalize_post_settle_sec', 2.0)
    navigator.declare_parameter('amcl_pose_max_age_sec', 3.0)
    navigator.declare_parameter('pnp_max_xy_m', 0.4)
    navigator.declare_parameter('pnp_max_yaw_rad', 0.25)
    navigator.declare_parameter('localization_mode', 'amcl')
    navigator.declare_parameter('slam_map_warmup_sec', 15.0)
    navigator.declare_parameter('slam_planner_ready_timeout_sec', 60.0)
    navigator.declare_parameter('slam_mapping_spin_rad', 1.57)

    waypoints_file = navigator.get_parameter('waypoints_file').value
    nav_profiles_file = navigator.get_parameter('nav_profiles_file').value
    shutdown_on_exit = navigator.get_parameter('shutdown_nav2_on_exit').value
    shutdown_on_failure = navigator.get_parameter('shutdown_nav2_on_failure').value
    scan_dwell_sec = navigator.get_parameter('scan_dwell_sec').value
    localization_mode = navigator.get_parameter('localization_mode').value
    relocalize_params = {
        'min_xy_m': navigator.get_parameter('relocalize_min_xy_m').value,
        'min_yaw_rad': navigator.get_parameter('relocalize_min_yaw_rad').value,
        'max_xy_m': navigator.get_parameter('relocalize_max_xy_m').value,
        'max_yaw_rad': navigator.get_parameter('relocalize_max_yaw_rad').value,
        'settle_sec': navigator.get_parameter('relocalize_settle_sec').value,
        'post_settle_sec': navigator.get_parameter('relocalize_post_settle_sec').value,
        'amcl_max_age_sec': navigator.get_parameter('amcl_pose_max_age_sec').value,
        'pnp_max_xy_m': navigator.get_parameter('pnp_max_xy_m').value,
        'pnp_max_yaw_rad': navigator.get_parameter('pnp_max_yaw_rad').value,
    }

    initial_pose_cfg, waypoints_cfg = load_waypoints_file(waypoints_file)
    nav_profiles = load_yaml_file(nav_profiles_file)
    if not validate_mission_poses(
            initial_pose_cfg, waypoints_cfg, map_bounds, navigator.get_logger()):
        navigator.get_logger().error(
            'Mission aborted: one or more poses are outside the static map.')
        rclpy.shutdown()
        return
    profile_applier = NavProfileApplier(
        navigator, nav_profiles, map_bounds, localization_mode)

    latest_markers = {}
    latest_amcl = {'pose': None}
    amcl_ready = {'value': False}
    relocalize_cfg = dict(
        relocalize_params,
        latest_amcl=latest_amcl,
        latest_markers=latest_markers,
        localization_mode=localization_mode,
    )
    tf_buffer = None
    tf_listener = None
    if uses_tf_localization(localization_mode):
        tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        tf_listener = TransformListener(tf_buffer, navigator, spin_thread=False)

    def refresh_localized_pose():
        if not uses_tf_localization(localization_mode):
            return
        pose = pose_from_tf(tf_buffer)
        if pose is None:
            return
        latest_amcl['pose'] = pose
        amcl_ready['value'] = True
        navigator.initial_pose_received = True

    def on_detected_markers(msg: String):
        latest_markers['data'] = msg.data

    def on_amcl_pose(msg: PoseWithCovarianceStamped):
        latest_amcl['pose'] = msg
        amcl_ready['value'] = True
        navigator.initial_pose_received = True

    navigator.create_subscription(String, '/detected_markers', on_detected_markers, 10)
    if localization_mode == 'amcl':
        navigator.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', on_amcl_pose, AMCL_POSE_QOS)
    elif uses_tf_localization(localization_mode):
        navigator.create_timer(0.2, refresh_localized_pose)

    navigator.get_logger().info(
        f'Loaded {len(waypoints_cfg)} waypoints from {waypoints_file} '
        f'(localization_mode={localization_mode})')
    for i, wp in enumerate(waypoints_cfg):
        expected = wp.get('expected_aruco_id', '?')
        profile = wp.get('nav_profile', 'default')
        retreat = 'yes' if wp.get('retreat_after_scan') else 'no'
        through = len(wp.get('through_poses', []))
        navigator.get_logger().info(
            f'  [{i}] {wp["name"]}: ({wp["x"]}, {wp["y"]}, yaw={wp["yaw"]}) '
            f'-> expect ArUco id={expected}, nav_profile={profile}, '
            f'through_poses={through}, retreat_after_scan={retreat}')

    # Do not call setInitialPose here — initial_pose_publisher already seeded AMCL.
    if not wait_for_localization(
            navigator, amcl_ready, localization_mode, tf_buffer):
        if tf_listener is not None:
            tf_listener = None
        rclpy.shutdown()
        return

    if not wait_for_nav2_active(navigator, localization_mode=localization_mode):
        if tf_listener is not None:
            tf_listener = None
        rclpy.shutdown()
        return

    if localization_mode == 'slam_online':
        ready_timeout = float(
            navigator.get_parameter('slam_planner_ready_timeout_sec').value)
        spin_rad = float(navigator.get_parameter('slam_mapping_spin_rad').value)
        min_warmup = float(navigator.get_parameter('slam_map_warmup_sec').value)
        if not wait_for_slam_planner_ready(
                navigator,
                waypoints_cfg[0],
                profile_applier,
                timeout_sec=ready_timeout,
                spin_rad=spin_rad,
                min_warmup_sec=min_warmup):
            profile_applier.restore_default()
            if tf_listener is not None:
                tf_listener = None
            rclpy.shutdown()
            return
    elif localization_mode == 'slam_localization':
        profile_applier.apply_slam_global_costmap()

    overall_result = TaskResult.SUCCEEDED

    for index, wp in enumerate(waypoints_cfg):
        latest_markers.pop('data', None)
        leg_label = f'waypoint {index + 1}/{len(waypoints_cfg)}: {wp["name"]}'
        leg_result = navigate_to_pose(
            navigator, wp, leg_label, profile_applier, relocalize_cfg)
        if leg_result != TaskResult.SUCCEEDED:
            overall_result = leg_result
            break

        dwell_after = float(wp.get('dwell_after_sec', 0.0))
        if dwell_after > 0.0:
            amcl_settle_dwell(navigator, dwell_after, wp['name'])

        if wp.get('skip_scan'):
            navigator.get_logger().info(
                'Clearing local costmap after transit leg (drop stale marks)')
            navigator.clearLocalCostmap()
            time.sleep(0.5)

        marker_confirmed = False
        if not wp.get('skip_scan'):
            expected = wp.get('expected_aruco_id')
            marker_confirmed = dwell_for_scan(
                navigator, scan_dwell_sec, expected, latest_markers)
            log_waypoint_scan(
                navigator, wp, latest_markers.get('data'), latest_amcl['pose'])

        spin_relocalize = wp.get('spin_relocalize_after_scan', False)
        if wp.get('relocalize_after_scan') and spin_relocalize:
            navigator.get_logger().info(
                f'{wp["name"]}: relocalize-before-spin; '
                f'AMCL will not be reseeded after spin')

        if wp.get('relocalize_after_scan'):
            force = wp.get('relocalize_force', False)
            x, y, yaw, source = resolve_relocalize_pose(
                navigator,
                wp['x'],
                wp['y'],
                wp['yaw'],
                wp['name'],
                latest_markers=latest_markers,
                latest_amcl=latest_amcl,
                expected_aruco_id=wp.get('expected_aruco_id'),
                prefer_marker=bool(wp.get('relocalize_from_marker')),
                prefer_amcl=False,
                amcl_max_age_sec=relocalize_cfg.get('amcl_max_age_sec', 1.0),
                pnp_max_xy_m=relocalize_cfg.get('pnp_max_xy_m', 0.4),
                pnp_max_yaw_rad=relocalize_cfg.get('pnp_max_yaw_rad', 0.25),
                trust_pnp=force,
            )
            loc_mode = relocalize_cfg.get('localization_mode', 'amcl')
            if source == 'yaml' and loc_mode in ('slam_online', 'slam_localization'):
                # Same rationale as the sequential-step guard: without a
                # validated PnP read, force-snapping to the YAML standoff in
                # SLAM mode only corrupts a pose the scan matcher is already
                # tracking. Observed repeatedly to precede controller stalls.
                navigator.get_logger().info(
                    f'{wp["name"]}: skipping YAML relocalize in {loc_mode} '
                    f'(scan matcher already tracking; no validated PnP)')
            else:
                if source == 'yaml':
                    force = True
                relocalize_at_pose(
                    navigator, x, y, yaw, wp['name'], map_bounds,
                    force=force,
                    reseed_only=False,
                    **relocalize_cfg_kwargs(relocalize_cfg))

        if not wp.get('skip_scan') and spin_relocalize:
            force_relocalize_via_spin(navigator, wp['name'])

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
                navigator, retreat_wp, retreat_label, profile_applier, relocalize_cfg)
            if retreat_result != TaskResult.SUCCEEDED:
                overall_result = retreat_result
                navigator.get_logger().error(
                    'Retreat from shelf aisle failed; cannot safely reach next waypoint.')
                break
            if wp.get('relocalize_after_retreat', True):
                x, y, yaw, _ = resolve_relocalize_pose(
                    navigator,
                    retreat['x'],
                    retreat['y'],
                    retreat['yaw'],
                    f'retreat anchor after {wp["name"]}',
                    latest_amcl=latest_amcl,
                    prefer_amcl=True,
                    amcl_max_age_sec=relocalize_cfg.get('amcl_max_age_sec', 1.0),
                )
                relocalize_at_pose(
                    navigator,
                    x,
                    y,
                    yaw,
                    f'retreat anchor after {wp["name"]}',
                    map_bounds,
                    force=retreat.get('force', True),
                    reseed_only=True,
                    **relocalize_cfg_kwargs(relocalize_cfg))

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

    if tf_listener is not None:
        tf_listener = None
    rclpy.shutdown()


if __name__ == '__main__':
    main()