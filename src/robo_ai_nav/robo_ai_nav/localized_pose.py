"""Localized pose helpers shared by mission nodes (AMCL or slam_toolbox)."""
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformException


def uses_tf_localization(localization_mode):
    return localization_mode in ('slam_online', 'slam_localization')


def yaw_to_quaternion_zw(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def make_pose_with_covariance(node, x, y, yaw):
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


def publish_initial_pose(publisher, node, x, y, yaw):
    """Publish /initialpose (AMCL and slam_toolbox localization both listen)."""
    publisher.publish(make_pose_with_covariance(node, x, y, yaw))


def pose_from_tf(tf_buffer):
    """Return map->base_footprint as PoseWithCovarianceStamped, or None."""
    try:
        transform = tf_buffer.lookup_transform(
            'map',
            'base_footprint',
            rclpy.time.Time(),
            timeout=Duration(seconds=0.5),
        )
    except TransformException:
        return None

    msg = PoseWithCovarianceStamped()
    msg.header = transform.header
    msg.pose.pose.position.x = transform.transform.translation.x
    msg.pose.pose.position.y = transform.transform.translation.y
    msg.pose.pose.position.z = transform.transform.translation.z
    msg.pose.pose.orientation = transform.transform.rotation
    return msg


def localization_ready(localization_mode, amcl_ready, tf_buffer):
    if localization_mode == 'amcl':
        return amcl_ready
    return pose_from_tf(tf_buffer) is not None
