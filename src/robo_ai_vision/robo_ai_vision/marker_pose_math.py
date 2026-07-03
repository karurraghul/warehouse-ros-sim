"""Estimate map-frame robot pose from an ArUco detection and known marker layout."""
import math

import cv2
import numpy as np


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def make_transform(x, y, z, roll, pitch, yaw):
    transform = np.eye(4)
    transform[:3, :3] = rpy_to_matrix(roll, pitch, yaw)
    transform[:3, 3] = [x, y, z]
    return transform


def invert_transform(transform):
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    inverse = np.eye(4)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation
    return inverse


def yaw_from_rotation(rotation):
    return math.atan2(rotation[1, 0], rotation[0, 0])


# base_footprint -> camera_optical_frame (matches delivery_robot_description.xacro).
T_BASEFOOTPRINT_OPTICAL = (
    make_transform(0.0, 0.0, 0.010, 0.0, 0.0, 0.0)
    @ make_transform(0.073, -0.011, 0.084, 0.0, 0.0, 0.0)
    @ make_transform(0.0, 0.0, 0.0, -math.pi / 2.0, 0.0, -math.pi / 2.0)
)
T_OPTICAL_BASEFOOTPRINT = invert_transform(T_BASEFOOTPRINT_OPTICAL)


def marker_rotation_in_map(model_yaw):
    """Build rotation from ArUco marker frame to map frame.

    ``model_yaw`` is the Gazebo model yaw from marker_layout.yaml.  Plates use a
    thin box with model +X as the plate normal; the aisle-visible face is the
    opposite side, so outward normal (OpenCV +Z) is model_yaw + pi.
    OpenCV marker axes: X right, Y down in image, Z out of the plate.
    """
    outward_yaw = model_yaw + math.pi
    normal = np.array([math.cos(outward_yaw), math.sin(outward_yaw), 0.0])
    y_axis = np.array([0.0, 0.0, -1.0])
    x_axis = np.cross(y_axis, normal)
    norm = np.linalg.norm(x_axis)
    if norm < 1e-9:
        x_axis = np.array([0.0, 1.0, 0.0])
    else:
        x_axis /= norm
    return np.column_stack([x_axis, y_axis, normal])


def camera_matrix(width, height, horizontal_fov):
    fx = 0.5 * width / math.tan(horizontal_fov / 2.0)
    fy = fx
    cx = width / 2.0
    cy = height / 2.0
    return np.array([
        [fx, 0.0, cx],
        [0.0, fy, cy],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)


def estimate_robot_pose_map(marker_x, marker_y, marker_z, marker_yaw,
                            rvec, tvec, camera_matrix, dist_coeffs,
                            camera_frame='camera_optical_frame'):
    """Return (x, y, yaw) for base_footprint in map frame."""
    rotation_cam_marker, _ = cv2.Rodrigues(rvec)
    translation_cam_marker = tvec.reshape(3)

    rotation_marker_cam = rotation_cam_marker.T
    translation_marker_cam = -rotation_marker_cam @ translation_cam_marker

    rotation_map_marker = marker_rotation_in_map(marker_yaw)
    translation_map_marker = np.array([marker_x, marker_y, marker_z])

    rotation_map_cam = rotation_map_marker @ rotation_marker_cam
    translation_map_cam = (
        rotation_map_marker @ translation_marker_cam + translation_map_marker)

    rotation_map_optical = rotation_map_cam
    translation_map_optical = translation_map_cam

    transform_map_optical = np.eye(4)
    transform_map_optical[:3, :3] = rotation_map_optical
    transform_map_optical[:3, 3] = translation_map_optical

    if camera_frame == 'base_footprint':
        transform_map_base = transform_map_optical
    else:
        transform_map_base = transform_map_optical @ T_OPTICAL_BASEFOOTPRINT
    yaw = yaw_from_rotation(transform_map_base[:3, :3])
    return (
        float(transform_map_base[0, 3]),
        float(transform_map_base[1, 3]),
        float(yaw),
    )
