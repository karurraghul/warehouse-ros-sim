"""Publish /initialpose from waypoints.yaml until AMCL reports a pose."""
import math
import os
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def yaw_to_quaternion_zw(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def load_initial_pose(path):
    with open(path, 'r') as handle:
        data = yaml.safe_load(handle)
    return data['initial_pose']


def make_initial_pose_msg(node, x, y, yaw):
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    qz, qw = yaw_to_quaternion_zw(float(yaw))
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw
    msg.pose.covariance[0] = 0.25
    msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.06853891909122467
    return msg


def main(args=None):
    rclpy.init(args=args)
    node = Node('initial_pose_publisher')
    pkg_share = get_package_share_directory('robo_ai_nav')
    node.declare_parameter(
        'waypoints_file',
        os.path.join(pkg_share, 'config', 'waypoints.yaml'))
    node.declare_parameter('publish_period_sec', 1.0)
    node.declare_parameter('max_attempts', 30)
    node.declare_parameter('use_sim_time', True)

    use_sim_time = node.get_parameter('use_sim_time').value
    if use_sim_time:
        from rclpy.parameter import Parameter
        node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

    waypoints_file = node.get_parameter('waypoints_file').value
    period = node.get_parameter('publish_period_sec').value
    max_attempts = node.get_parameter('max_attempts').value
    initial_pose = load_initial_pose(waypoints_file)

    amcl_qos = QoSProfile(
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    amcl_ready = {'value': False}

    def on_amcl_pose(_msg):
        amcl_ready['value'] = True

    node.create_subscription(
        PoseWithCovarianceStamped, '/amcl_pose', on_amcl_pose, amcl_qos)

    node.get_logger().info(
        f'Publishing initial pose from {waypoints_file}: '
        f'({initial_pose["x"]}, {initial_pose["y"]}, yaw={initial_pose["yaw"]})')

    for attempt in range(1, max_attempts + 1):
        if amcl_ready['value']:
            node.get_logger().info('AMCL pose received; initial pose publisher done.')
            break
        msg = make_initial_pose_msg(
            node, initial_pose['x'], initial_pose['y'], initial_pose['yaw'])
        pub.publish(msg)
        node.get_logger().info(f'Published /initialpose (attempt {attempt}/{max_attempts})')
        deadline = time.monotonic() + period
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if amcl_ready['value']:
                break
        if amcl_ready['value']:
            node.get_logger().info('AMCL pose received; initial pose publisher done.')
            break
    else:
        node.get_logger().warn(
            'AMCL did not publish /amcl_pose before max attempts; '
            'check map_server, scan, and Nav2 lifecycle.')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
