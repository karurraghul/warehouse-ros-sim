"""Regression tests for ArUco PnP map-frame pose estimation."""
import math

import cv2
import numpy as np
import pytest

from robo_ai_vision.marker_pose_math import (
    T_BASEFOOTPRINT_OPTICAL,
    camera_matrix,
    estimate_robot_pose_map,
    invert_transform,
    make_transform,
    marker_rotation_in_map,
)


def _synthetic_rvec_tvec(robot_x, robot_y, robot_yaw,
                         marker_x, marker_y, marker_z, marker_model_yaw):
    """Build OpenCV rvec/tvec from a known robot pose (round-trip ground truth)."""
    transform_map_base = make_transform(robot_x, robot_y, 0.0, 0.0, 0.0, robot_yaw)
    transform_map_optical = transform_map_base @ T_BASEFOOTPRINT_OPTICAL

    rotation_map_marker = marker_rotation_in_map(marker_model_yaw)
    transform_map_marker = np.eye(4)
    transform_map_marker[:3, :3] = rotation_map_marker
    transform_map_marker[:3, 3] = [marker_x, marker_y, marker_z]

    transform_cam_map = invert_transform(transform_map_optical)
    transform_cam_marker = transform_cam_map @ transform_map_marker

    rvec, _ = cv2.Rodrigues(transform_cam_marker[:3, :3])
    tvec = transform_cam_marker[:3, 3].reshape(3, 1)
    return rvec, tvec


def _assert_pose_near(actual, expected_x, expected_y, expected_yaw,
                      xy_tol=0.3, yaw_tol=0.15):
    x, y, yaw = actual
    assert abs(x - expected_x) <= xy_tol, f'x {x:.3f} vs {expected_x:.3f}'
    assert abs(y - expected_y) <= xy_tol, f'y {y:.3f} vs {expected_y:.3f}'
    dyaw = abs(math.atan2(
        math.sin(yaw - expected_yaw), math.cos(yaw - expected_yaw)))
    assert dyaw <= yaw_tol, f'yaw {yaw:.3f} vs {expected_yaw:.3f} (dyaw={dyaw:.3f})'


@pytest.mark.parametrize(
    'robot_x,robot_y,robot_yaw,marker_x,marker_y,marker_z,marker_yaw',
    [
        (2.0, 0.57943, 0.0, 4.05, 0.57943, 0.3, 0.0),
        (0.35, 4.0, 1.571, 0.0, 6.5, 0.3, 1.571),
        (-4.2, -8.51815, 0.0, -3.74722, -8.51815, 0.3, -0.603732),
        (0.5, -9.55, -1.571, 0.5, -9.0, 0.3, 1.571),
    ],
)
def test_estimate_robot_pose_map_round_trip(
        robot_x, robot_y, robot_yaw, marker_x, marker_y, marker_z, marker_yaw):
    cam_mtx = camera_matrix(640, 480, 0.9337511)
    dist = np.zeros((5, 1), dtype=np.float64)
    rvec, tvec = _synthetic_rvec_tvec(
        robot_x, robot_y, robot_yaw,
        marker_x, marker_y, marker_z, marker_yaw)
    result = estimate_robot_pose_map(
        marker_x, marker_y, marker_z, marker_yaw,
        rvec, tvec, cam_mtx, dist)
    _assert_pose_near(result, robot_x, robot_y, robot_yaw)


def test_outward_normal_uses_model_yaw_plus_pi():
    rotation = marker_rotation_in_map(0.0)
    normal = rotation[:, 2]
    np.testing.assert_allclose(normal, [-1.0, 0.0, 0.0], atol=1e-6)
