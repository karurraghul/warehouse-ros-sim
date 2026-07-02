"""Detect ArUco and QR markers in the delivery robot's camera feed.

Publishes detections as a JSON-encoded std_msgs/String on `/detected_markers`
(vision_msgs is not used here since it is not installed in this
environment; swap in vision_msgs/Detection2DArray later if desired) and an
annotated debug image on `/marker_detector/image_annotated`.

QR decoding tries `pyzbar` first (install with
`sudo apt install python3-pyzbar libzbar0` if not present), falling back to
OpenCV's built-in `cv2.QRCodeDetector`. Note that the stock Ubuntu 22.04
`python3-opencv` package is built without the QUIRC backend, so
`cv2.QRCodeDetector` alone may never decode anything on some systems -
`pyzbar` is the more reliable option if QR detection is important to you.
"""
import json

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    from pyzbar import pyzbar
    _HAVE_PYZBAR = True
except ImportError:
    _HAVE_PYZBAR = False


class MarkerDetectorNode(Node):

    def __init__(self):
        super().__init__('marker_detector_node')

        self.declare_parameter('camera_topic', '/delivery_camera/image_raw')
        self.declare_parameter('detections_topic', '/detected_markers')
        self.declare_parameter('annotated_topic', '/marker_detector/image_annotated')
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('publish_annotated', True)

        camera_topic = self.get_parameter('camera_topic').value
        detections_topic = self.get_parameter('detections_topic').value
        annotated_topic = self.get_parameter('annotated_topic').value
        dict_name = self.get_parameter('aruco_dictionary').value
        self._publish_annotated = self.get_parameter('publish_annotated').value

        self._bridge = CvBridge()
        self._aruco_dict = cv2.aruco.Dictionary_get(getattr(cv2.aruco, dict_name))
        self._aruco_params = cv2.aruco.DetectorParameters_create()
        self._qr_detector = cv2.QRCodeDetector()

        self._detections_pub = self.create_publisher(String, detections_topic, 10)
        self._annotated_pub = (
            self.create_publisher(Image, annotated_topic, 10)
            if self._publish_annotated else None
        )

        self._image_sub = self.create_subscription(
            Image, camera_topic, self._on_image, 10)

        self._warned_no_pyzbar = False

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

    def _on_image(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detections = []
        annotated = frame if self._publish_annotated else None

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self._aruco_dict, parameters=self._aruco_params)
        if ids is not None:
            for marker_corners, marker_id in zip(corners, ids.flatten()):
                pts = marker_corners.reshape(-1, 2).tolist()
                detections.append({
                    'type': 'aruco',
                    'id': int(marker_id),
                    'corners': pts,
                })
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
            out = String()
            out.data = json.dumps({
                'stamp_sec': msg.header.stamp.sec,
                'frame_id': msg.header.frame_id,
                'detections': detections,
            })
            self._detections_pub.publish(out)

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

        # Localize first (cheap, works without QUIRC); only attempt the
        # decode step - which throws a cv2.error on some OpenCV builds
        # without the QUIRC backend - when a QR-like pattern was actually
        # found, and never let that exception escape (it would otherwise
        # kill the whole node on every frame containing a QR code).
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
