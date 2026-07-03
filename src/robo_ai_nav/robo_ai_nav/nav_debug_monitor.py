"""Nav debug lifecycle monitor: stuck, collision, and wrong-direction detection.

Uses /cmd_vel intent vs /odom motion and front-sector /scan proximity because
Gazebo does not publish contact/collision topics for this robot.
"""
import json
import math
import time
from enum import Enum

import rclpy
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import State as LifecycleState
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class NavHealth(Enum):
    OK = 'ok'
    STUCK = 'stuck'
    COLLISION = 'collision'
    WRONG_DIRECTION = 'wrong_direction'


def yaw_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def min_front_scan_range(scan, half_angle_rad):
    if scan is None or not scan.ranges:
        return float('inf')

    min_range = float('inf')
    angle = scan.angle_min
    for distance in scan.ranges:
        if math.isfinite(distance) and scan.range_min <= distance <= scan.range_max:
            if abs(angle) <= half_angle_rad:
                min_range = min(min_range, distance)
        angle += scan.angle_increment
    return min_range


class NavDebugMonitor(LifecycleNode):
    def __init__(self):
        super().__init__('nav_debug_monitor')
        self.declare_parameter('check_period_sec', 0.5)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('stuck_time_sec', 5.0)
        self.declare_parameter('stuck_distance_m', 0.06)
        self.declare_parameter('cmd_linear_threshold', 0.04)
        self.declare_parameter('cmd_angular_threshold', 0.08)
        self.declare_parameter('collision_range_m', 0.20)
        self.declare_parameter('collision_front_half_angle_deg', 50.0)
        self.declare_parameter('collision_time_sec', 1.5)
        self.declare_parameter('wrong_direction_time_sec', 4.0)
        self.declare_parameter('wrong_direction_min_progress_m', 0.08)
        self.declare_parameter('log_repeat_sec', 8.0)

        self._cmd_vel = Twist()
        self._scan = None
        self._odom = None
        self._motion_window_start = None
        self._motion_window_origin = None
        self._motion_window_yaw = None
        self._motion_intent_since = None
        self._blocked_since = None
        self._health = NavHealth.OK
        self._last_log_at = {}
        self._timer = None
        self._subs = []
        self._pubs = {}

    def _get_param(self, name):
        return self.get_parameter(name).value

    def on_configure(self, _state):
        status_topic = '/nav_debug/status'
        self._pubs = {
            'status': self.create_lifecycle_publisher(String, status_topic, 10),
            'stuck': self.create_lifecycle_publisher(Bool, '/nav_debug/stuck', 10),
            'collision': self.create_lifecycle_publisher(Bool, '/nav_debug/collision', 10),
            'wrong_direction': self.create_lifecycle_publisher(
                Bool, '/nav_debug/wrong_direction', 10),
        }
        self.get_logger().info(
            'Configured nav debug monitor (publishing status on /nav_debug/status)')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, _state):
        cmd_topic = self._get_param('cmd_vel_topic')
        odom_topic = self._get_param('odom_topic')
        scan_topic = self._get_param('scan_topic')

        self._subs = [
            self.create_subscription(Twist, cmd_topic, self._on_cmd_vel, 10),
            self.create_subscription(Odometry, odom_topic, self._on_odom, 10),
            self.create_subscription(LaserScan, scan_topic, self._on_scan, 10),
        ]
        period = self._get_param('check_period_sec')
        self._timer = self.create_timer(period, self._evaluate)
        for pub in self._pubs.values():
            pub.on_activate(self)
        self.get_logger().info(
            f'Active: monitoring {cmd_topic}, {odom_topic}, {scan_topic}')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, _state):
        if self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None
        for sub in self._subs:
            self.destroy_subscription(sub)
        self._subs = []
        for pub in self._pubs.values():
            pub.on_deactivate(self)
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, _state):
        for pub in self._pubs.values():
            self.destroy_publisher(pub)
        self._pubs = {}
        return TransitionCallbackReturn.SUCCESS

    def _on_cmd_vel(self, msg):
        self._cmd_vel = msg

    def _on_odom(self, msg):
        self._odom = msg

    def _on_scan(self, msg):
        self._scan = msg

    def _motion_intent(self):
        linear = abs(self._cmd_vel.linear.x)
        angular = abs(self._cmd_vel.angular.z)
        return (
            linear >= self._get_param('cmd_linear_threshold')
            or angular >= self._get_param('cmd_angular_threshold')
        )

    def _odom_xy_yaw(self):
        if self._odom is None:
            return None
        pose = self._odom.pose.pose
        yaw = yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        return pose.position.x, pose.position.y, yaw

    def _reset_motion_window(self, now, pose):
        self._motion_window_start = now
        self._motion_window_origin = pose
        self._motion_window_yaw = pose[2]

    def _evaluate(self):
        now = time.monotonic()
        pose = self._odom_xy_yaw()
        intent = self._motion_intent()

        if intent:
            if self._motion_intent_since is None:
                self._motion_intent_since = now
            if pose is not None and self._motion_window_start is None:
                self._reset_motion_window(now, pose)
        else:
            self._motion_intent_since = None
            self._motion_window_start = None
            self._motion_window_origin = None
            self._blocked_since = None
            self._set_health(NavHealth.OK, now, 'No motion command')
            return

        health = NavHealth.OK
        detail = 'Moving as expected'

        front_min = min_front_scan_range(
            self._scan,
            math.radians(self._get_param('collision_front_half_angle_deg')),
        )
        if (
            self._cmd_vel.linear.x > self._get_param('cmd_linear_threshold')
            and front_min < self._get_param('collision_range_m')
        ):
            if self._blocked_since is None:
                self._blocked_since = now
            blocked_for = now - self._blocked_since
            if blocked_for >= self._get_param('collision_time_sec'):
                health = NavHealth.COLLISION
                detail = (
                    f'Blocked: front obstacle {front_min:.2f}m while driving forward '
                    f'for {blocked_for:.1f}s (cmd_vel.linear.x={self._cmd_vel.linear.x:.2f})')
        else:
            self._blocked_since = None

        if pose is not None and self._motion_window_origin is not None:
            dx = pose[0] - self._motion_window_origin[0]
            dy = pose[1] - self._motion_window_origin[1]
            displacement = math.hypot(dx, dy)
            heading_x = math.cos(self._motion_window_yaw)
            heading_y = math.sin(self._motion_window_yaw)
            forward_progress = dx * heading_x + dy * heading_y

            intent_duration = now - self._motion_intent_since
            if (
                health == NavHealth.OK
                and intent_duration >= self._get_param('stuck_time_sec')
                and displacement < self._get_param('stuck_distance_m')
            ):
                health = NavHealth.STUCK
                detail = (
                    f'Stuck: commanded motion for {intent_duration:.1f}s but moved '
                    f'only {displacement:.3f}m (cmd_vel.linear.x={self._cmd_vel.linear.x:.2f}, '
                    f'front_range={front_min:.2f}m)')

            if (
                health == NavHealth.OK
                and self._cmd_vel.linear.x > self._get_param('cmd_linear_threshold')
                and intent_duration >= self._get_param('wrong_direction_time_sec')
            ):
                min_progress = self._get_param('wrong_direction_min_progress_m')
                if forward_progress < -0.03:
                    health = NavHealth.WRONG_DIRECTION
                    detail = (
                        f'Wrong direction: driving forward but moved backward '
                        f'{forward_progress:.2f}m over {intent_duration:.1f}s')
                elif forward_progress < min_progress and abs(self._cmd_vel.angular.z) < 0.2:
                    health = NavHealth.WRONG_DIRECTION
                    detail = (
                        f'Wrong direction: forward command but only {forward_progress:.2f}m '
                        f'progress in {intent_duration:.1f}s (expected >={min_progress:.2f}m)')

        self._set_health(health, now, detail, front_min, pose)

    def _set_health(self, health, now, detail, front_min=float('inf'), pose=None):
        self._health = health
        stuck = health == NavHealth.STUCK
        collision = health == NavHealth.COLLISION
        wrong_dir = health == NavHealth.WRONG_DIRECTION

        status = {
            'health': health.value,
            'detail': detail,
            'cmd_vel': {
                'linear_x': round(self._cmd_vel.linear.x, 3),
                'angular_z': round(self._cmd_vel.angular.z, 3),
            },
            'front_scan_min_m': round(front_min, 3) if math.isfinite(front_min) else None,
        }
        if pose is not None:
            status['odom'] = {
                'x': round(pose[0], 3),
                'y': round(pose[1], 3),
                'yaw_deg': round(math.degrees(pose[2]), 1),
            }

        msg = String()
        msg.data = json.dumps(status)
        self._pubs['status'].publish(msg)
        self._pubs['stuck'].publish(Bool(data=stuck))
        self._pubs['collision'].publish(Bool(data=collision))
        self._pubs['wrong_direction'].publish(Bool(data=wrong_dir))

        if health != NavHealth.OK:
            last = self._last_log_at.get(health, 0.0)
            if now - last >= self._get_param('log_repeat_sec'):
                self._last_log_at[health] = now
                log_fn = self.get_logger().error if health == NavHealth.COLLISION else self.get_logger().warn
                log_fn(f'NAV DEBUG [{health.value}]: {detail}')
        elif self._last_log_at:
            self._last_log_at.clear()


def main(args=None):
    rclpy.init(args=args)
    node = NavDebugMonitor()
    node.trigger_configure()
    node.trigger_activate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        state = node.get_current_state()
        if state.id == LifecycleState.PRIMARY_STATE_ACTIVE:
            node.trigger_deactivate()
            state = node.get_current_state()
        if state.id == LifecycleState.PRIMARY_STATE_INACTIVE:
            node.trigger_cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
