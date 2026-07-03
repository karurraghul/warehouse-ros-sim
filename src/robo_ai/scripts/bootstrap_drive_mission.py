#!/usr/bin/env python3
"""Drive the warehouse loop with cmd_vel for slam_online map bootstrap.

Nav2 cannot plan until SLAM has mapped free space; this script follows
map-frame waypoints using TF feedback so async_slam_toolbox can build a
full posegraph before serialization.
"""
import argparse
import math
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def yaw_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def load_targets(waypoints_file):
    with open(waypoints_file, 'r', encoding='utf-8') as handle:
        cfg = yaml.safe_load(handle)

    targets = []
    for wp in cfg.get('waypoints', []):
        for pose in wp.get('through_poses', []):
            targets.append((float(pose['x']), float(pose['y']), float(pose.get('yaw', 0.0))))
        targets.append((float(wp['x']), float(wp['y']), float(wp.get('yaw', 0.0))))
    return targets


class BootstrapDrive(Node):
    def __init__(self):
        super().__init__('bootstrap_drive_mission')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def wait_for_tf(self, timeout_sec=90.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self.get_pose_map() is not None:
                self.get_logger().info('map->base_footprint TF available')
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().error('Timed out waiting for map->base_footprint TF')
        return False

    def get_pose_map(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', Time(), timeout=Duration(seconds=0.5))
        except Exception:
            return None
        yaw = yaw_from_quaternion(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        )
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
            yaw,
        )

    def stop(self):
        self.cmd_pub.publish(Twist())

    def spin_in_place(self, radians, angular_speed=0.35):
        if abs(radians) < 1e-3:
            return
        twist = Twist()
        twist.angular.z = angular_speed if radians >= 0.0 else -angular_speed
        duration = abs(radians) / abs(angular_speed)
        deadline = time.monotonic() + duration + 0.5
        while time.monotonic() < deadline:
            if time.monotonic() < deadline - 0.5:
                self.cmd_pub.publish(twist)
            else:
                self.stop()
            rclpy.spin_once(self, timeout_sec=0.05)
        time.sleep(0.2)

    def drive_to(self, goal_x, goal_y, goal_yaw, timeout_sec=180.0,
                 xy_tol=0.3, yaw_tol=0.25, label=''):
        self.get_logger().info(
            f'Driving to {label or f"({goal_x:.2f}, {goal_y:.2f})"} '
            f'(timeout={timeout_sec:.0f}s)')
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            pose = self.get_pose_map()
            if pose is None:
                rclpy.spin_once(self, timeout_sec=0.05)
                continue

            x, y, yaw = pose
            dx = goal_x - x
            dy = goal_y - y
            dist = math.hypot(dx, dy)
            target_heading = math.atan2(dy, dx)
            heading_error = normalize_angle(target_heading - yaw)
            yaw_error = normalize_angle(goal_yaw - yaw)

            twist = Twist()
            if dist > xy_tol:
                if abs(heading_error) > 0.35:
                    twist.angular.z = max(-0.6, min(0.6, 1.5 * heading_error))
                else:
                    twist.linear.x = max(0.0, min(0.22, 0.5 * dist))
                    twist.angular.z = max(-0.4, min(0.4, 1.0 * heading_error))
            elif abs(yaw_error) > yaw_tol:
                twist.angular.z = max(-0.5, min(0.5, 1.2 * yaw_error))
            else:
                self.stop()
                self.get_logger().info(
                    f'Reached {label or f"({goal_x:.2f}, {goal_y:.2f})"} '
                    f'at ({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.0f} deg)')
                time.sleep(0.5)
                return True

            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.stop()
        self.get_logger().warn(
            f'Timed out driving to {label or f"({goal_x:.2f}, {goal_y:.2f})"}')
        return False


def main():
    parser = argparse.ArgumentParser(description='Drive warehouse loop for SLAM bootstrap')
    parser.add_argument(
        '--waypoints-file',
        default='',
        help='Path to waypoints.yaml (default: robo_ai_nav install share)')
    parser.add_argument('--startup-spin-rad', type=float, default=6.28)
    args = parser.parse_args()

    if args.waypoints_file:
        waypoints_file = args.waypoints_file
    else:
        from ament_index_python.packages import get_package_share_directory
        waypoints_file = get_package_share_directory('robo_ai_nav') + '/config/waypoints.yaml'

    targets = load_targets(waypoints_file)
    if not targets:
        print('No drive targets found in waypoints file', file=sys.stderr)
        return 1

    rclpy.init()
    node = BootstrapDrive()
    try:
        if not node.wait_for_tf():
            return 1
        node.spin_in_place(args.startup_spin_rad)
        failed = 0
        for index, (x, y, yaw) in enumerate(targets):
            label = f'step {index + 1}/{len(targets)}'
            if not node.drive_to(x, y, yaw, label=label):
                failed += 1
        node.stop()
        node.get_logger().info(
            f'Bootstrap drive finished ({len(targets) - failed}/{len(targets)} steps ok)')
        return 0 if failed == 0 else 1
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
