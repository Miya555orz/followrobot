"""Regression tests for the calibrated ROS static-transform convention."""

import importlib.util
from pathlib import Path

import numpy as np


MODULE_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "extrinsics_tf_publisher.py"
)
SPEC = importlib.util.spec_from_file_location("extrinsics_tf_publisher", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def quaternion_to_rotation(quaternion):
    x, y, z, w = quaternion
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def test_rotation_matrix_quaternion_round_trip():
    angle = np.deg2rad(17.0)
    rotation = np.array([
        [np.cos(angle), 0.0, np.sin(angle)],
        [0.0, 1.0, 0.0],
        [-np.sin(angle), 0.0, np.cos(angle)],
    ])
    quaternion = MODULE.rotation_matrix_to_quaternion(rotation)
    quaternion /= np.linalg.norm(quaternion)
    assert np.allclose(quaternion_to_rotation(quaternion), rotation, atol=1e-7)


def test_ros_parent_child_pose_matches_opencv_point_mapping():
    # OpenCV stereoCalibrate maps Sony point coordinates into Gemini coordinates.
    gemini_from_sony = np.eye(4)
    gemini_from_sony[:3, 3] = [0.12, -0.01, 0.02]
    # parent=Gemini color, child=Sony: TF stores the pose of Sony in Gemini.
    ros_parent_from_child = gemini_from_sony
    point_sony = np.array([0.3, 0.1, 2.0, 1.0])
    point_gemini = gemini_from_sony @ point_sony
    assert np.allclose(ros_parent_from_child @ point_sony, point_gemini)
