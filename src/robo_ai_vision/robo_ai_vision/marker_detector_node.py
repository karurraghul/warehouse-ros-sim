"""Detect ArUco and QR markers in the delivery robot's camera feed.

Publishes detections as JSON on `/detected_markers`, including optional
map-frame robot pose estimates from ArUco PnP when marker layout is configured.
"""
import json
import math
import os

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from robo_ai_vision.marker_pose_math import (
    camera_matrix,
    estimate_robot_pose_map,
)

try:
    from pyzbar import pyzbar
    _HAVE_PYZBAR = True
except ImportError:
    _HAVE_PYZBAR = False


def _marker_pixel_size(corners):
    pts = corners.reshape(-1, 2)
    edges = [
        math.hypot(pts[i][0] - pts[(i + 1) % 4][0], pts[i][1] - pts[(i + 1) % 4][1])
        for i in range(4)
    ]
    return sum(edges) / len(edges)


def _marker_center(corners):
    pts = corners.reshape(-1, 2)
    return float(pts[:, 0].mean()), float(pts[:, 1].mean())


def load_marker_layout(path):
    with open(path, 'r') as handle:
        data = yaml.safe_load(handle)
    layout = {}
    for entry in data.get('markers', []):
        layout[int(entry['aruco_id'])] = entry
    return layout


class MarkerDetectorNode(Node):

    def __init__(self):
        super().__init__('marker_detector_node')

        pkg_share = get_package_share_directory('robo_ai_nav')
        default_layout = os.path.join(pkg_share, 'config', 'marker_layout.yaml')

        self.declare_parameter('camera_topic', '/delivery_camera/image_raw')
        self.declare_parameter('detections_topic', '/detected_markers')
        self.declare_parameter('annotated_topic', '/marker_detector/image_annotated')
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('no_marker_log_interval_sec', 30.0)
        self.declare_parameter('marker_layout_file', default_layout)
        self.declare_parameter('marker_size_m', 0.15)
        self.declare_parameter('camera_horizontal_fov', 0.9337511)

        camera_topic = self.get_parameter('camera_topic').value
        detections_topic = self.get_parameter('detections_topic').value
        annotated_topic = self.get_parameter('annotated_topic').value
        dict_name = self.get_parameter('aruco_dictionary').value
        self._publish_annotated = self.get_parameter('publish_annotated').value
        self._no_marker_log_interval = self.get_parameter(
            'no_marker_log_interval_sec').value
        marker_layout_file = self.get_parameter('marker_layout_file').value
        self._marker_size_m = float(self.get_parameter('marker_size_m').value)
        self._camera_hfov = float(self.get_parameter('camera_horizontal_fov').value)

        self._marker_layout = {}
        if marker_layout_file and os.path.isfile(marker_layout_file):
            self._marker_layout = load_marker_layout(marker_layout_file)
            self.get_logger().info(
                f'Loaded {len(self._marker_layout)} marker layout entries from '
                f'{marker_layout_file}')
        else:
            self.get_logger().warn(
                f'Marker layout file not found at {marker_layout_file!r}; '
                f'PnP robot pose will not be published.')

        self._bridge = CvBridge()
        self._aruco_dict = cv2.aruco.Dictionary_get(getattr(cv2.aruco, dict_name))
        self._aruco_params = cv2.aruco.DetectorParameters_create()
        self._qr_detector = cv2.QRCodeDetector()
        self._dist_coeffs = np.zeros((5, 1), dtype=np.float64)

        self._detections_pub = self.create_publisher(String, detections_topic, 10)
        self._annotated_pub = (
            self.create_publisher(Image, annotated_topic, 10)
            if self._publish_annotated else None
        )

        self._image_sub = self.create_subscription(
            Image, camera_topic, self._on_image, 10)

        self._warned_no_pyzbar = False
        self._logged_first_frame = False
        self._frame_count = 0
        self._last_no_marker_log = self.get_clock().now()
        self._last_logged_aruco_ids = None
        self._image_width = 0
        self._image_height = 0

        self.get_logger().info(
            f'Subscribing to camera topic "{camera_topic}". '
            f'If no images arrive, run `ros2 topic list` and set the '
            f'camera_topic parameter to match the actual topic name.')
        if not _HAVE_PYZBAR:
            self.get_logger().warn(
                'pyzbar not found - falling back to cv2.QRCodeDetector, '
                'which may fail to decode QR codes on systems where OpenCV '
                'was built without the QUIRC library. '
                'Install with: sudo apt install python3-pyzbar libzbar0')

    def _log_no_markers_throttled(self):
        now = self.get_clock().now()
        elapsed = (now - self._last_no_marker_log).nanoseconds / 1e9
        if elapsed < self._no_marker_log_interval:
            return
        self._last_no_marker_log = now
        self.get_logger().debug(
            f'No markers in frame ({self._image_width}x{self._image_height}, '
            f'frame #{self._frame_count}). Check robot distance/orientation '
            f'or view /marker_detector/image_annotated in RViz.')

    def _estimate_robot_pose(self, marker_id, corners, cam_mtx):
        layout = self._marker_layout.get(int(marker_id))
        if layout is None:
            return None
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self._marker_size_m, cam_mtx, self._dist_coeffs)
        try:
            x, y, yaw = estimate_robot_pose_map(
                float(layout['x']),
                float(layout['y']),
                float(layout.get('z', 0.3)),
                float(layout['yaw']),
                rvecs[0],
                tvecs[0],
                cam_mtx,
                self._dist_coeffs,
            )
        except cv2.error as exc:
            self.get_logger().debug(
                f'PnP failed for ArUco id={marker_id}: {exc}')
            return None
        return {'x': round(x, 4), 'y': round(y, 4), 'yaw': round(yaw, 4)}

    def _on_image(self, msg: Image):
        self._frame_count += 1
        self._image_width = msg.width
        self._image_height = msg.height

        if not self._logged_first_frame:
            self._logged_first_frame = True
            self.get_logger().info(
                f'First camera frame: {msg.width}x{msg.height}, '
                f'frame_id="{msg.header.frame_id}"')

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cam_mtx = camera_matrix(msg.width, msg.height, self._camera_hfov)

        detections = []
        annotated = frame if self._publish_annotated else None

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self._aruco_dict, parameters=self._aruco_params)
        if ids is not None:
            for marker_corners, marker_id in zip(corners, ids.flatten()):
                pts = marker_corners.reshape(-1, 2).tolist()
                px_size = _marker_pixel_size(marker_corners)
                cx, cy = _marker_center(marker_corners)
                entry = {
                    'type': 'aruco',
                    'id': int(marker_id),
                    'corners': pts,
                    'pixel_size': round(px_size, 1),
                    'center_px': [round(cx, 1), round(cy, 1)],
                }
                robot_pose = self._estimate_robot_pose(
                    int(marker_id), marker_corners, cam_mtx)
                if robot_pose is not None:
                    entry['robot_pose_map'] = robot_pose
                detections.append(entry)
            if annotated is not None:
                cv2.aruco.drawDetectedMarkers(annotated, corners, ids)

        for text, points in self._detect_qr_codes(gray):
            detections.append({'type': 'qr', 'text': text, 'corners': points})
            if annotated is not None and points:
                pts_arr = [(int(x), int(y)) for x, y in points]
                for i in range(len(pts_arr)):
                    cv2.line(annotated, pts_arr[i], pts_arr[(i + 1) % len(pts_arr)],
                              (0, 255, 0), 2)
                cv2.putText(annotated, text, pts_arr[0],
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if detections:
            aruco_ids = sorted(
                d['id'] for d in detections if d['type'] == 'aruco')
            aruco_id_set = frozenset(aruco_ids)
            if aruco_id_set != self._last_logged_aruco_ids:
                self._last_logged_aruco_ids = aruco_id_set
                if aruco_ids:
                    self.get_logger().info(
                        f'ArUco marker(s) visible: ids={aruco_ids}')
            out = String()
            out.data = json.dumps({
                'stamp_sec': msg.header.stamp.sec,
                'frame_id': msg.header.frame_id,
                'detections': detections,
            })
            self._detections_pub.publish(out)
        else:
            if self._last_logged_aruco_ids:
                self._last_logged_aruco_ids = frozenset()
            self._log_no_markers_throttled()

        if annotated is not None:
            out_msg = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out_msg.header = msg.header
            self._annotated_pub.publish(out_msg)

    def _detect_qr_codes(self, gray):
        results = []
        if _HAVE_PYZBAR:
            for symbol in pyzbar.decode(gray):
                text = symbol.data.decode('utf-8', errors='replace')
                points = [[p.x, p.y] for p in symbol.polygon]
                results.append((text, points))
            return results

        found, points = self._qr_detector.detect(gray)
        if not found:
            return results
        try:
            data, _ = self._qr_detector.decode(gray, points)
        except cv2.error:
            if not self._warned_no_pyzbar:
                self._warned_no_pyzbar = True
                self.get_logger().warn(
                    'A QR code was localized but cv2.QRCodeDetector.decode() '
                    'raised an error (likely missing QUIRC support in this '
                    'OpenCV build). Install pyzbar for reliable QR decoding: '
                    'sudo apt install python3-pyzbar libzbar0')
            return results
        if data:
            corners = points.reshape(-1, 2).tolist() if points is not None else []
            results.append((data, corners))
        return results


def main(args=None):
    rclpy.init(args=args)
    node = MarkerDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
