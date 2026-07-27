#!/usr/bin/env python3
"""Connect Sony below Gemini color using the calibrated rigid transform."""

import argparse
import math

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def rotation_matrix_to_quaternion(matrix):
    """Return ROS quaternion [x, y, z, w] from a proper 3x3 rotation matrix."""
    trace = float(np.trace(matrix))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return np.array([
            (matrix[2, 1] - matrix[1, 2]) / s,
            (matrix[0, 2] - matrix[2, 0]) / s,
            (matrix[1, 0] - matrix[0, 1]) / s,
            0.25 * s,
        ])
    index = int(np.argmax(np.diag(matrix)))
    if index == 0:
        s = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        quaternion = [
            0.25 * s,
            (matrix[0, 1] + matrix[1, 0]) / s,
            (matrix[0, 2] + matrix[2, 0]) / s,
            (matrix[2, 1] - matrix[1, 2]) / s,
        ]
    elif index == 1:
        s = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        quaternion = [
            (matrix[0, 1] + matrix[1, 0]) / s,
            0.25 * s,
            (matrix[1, 2] + matrix[2, 1]) / s,
            (matrix[0, 2] - matrix[2, 0]) / s,
        ]
    else:
        s = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        quaternion = [
            (matrix[0, 2] + matrix[2, 0]) / s,
            (matrix[1, 2] + matrix[2, 1]) / s,
            0.25 * s,
            (matrix[1, 0] - matrix[0, 1]) / s,
        ]
    return np.asarray(quaternion)


class ExtrinsicsTfPublisher(Node):
    def __init__(self, calibration_path):
        super().__init__('sony_gemini_extrinsics_tf_publisher')
        with open(calibration_path, 'r', encoding='utf-8') as stream:
            calibration = yaml.safe_load(stream)
        if calibration.get('schema_version') != 2:
            raise RuntimeError(
                'Unsupported calibration schema. Re-run stereo_calibrate.py '
                'to generate schema_version 2.')

        transform = np.asarray(
            calibration['ros_tf_T_parent_from_child'], dtype=np.float64)
        if transform.shape != (4, 4):
            raise RuntimeError('ros_tf_T_parent_from_child must be a 4x4 matrix')
        rotation = transform[:3, :3]
        if not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-4):
            raise RuntimeError('Calibration rotation matrix is not orthonormal')

        message = TransformStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = calibration['frame_id']
        message.child_frame_id = calibration['child_frame_id']
        message.transform.translation.x = float(transform[0, 3])
        message.transform.translation.y = float(transform[1, 3])
        message.transform.translation.z = float(transform[2, 3])
        qx, qy, qz, qw = rotation_matrix_to_quaternion(rotation)
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        message.transform.rotation.x = float(qx / norm)
        message.transform.rotation.y = float(qy / norm)
        message.transform.rotation.z = float(qz / norm)
        message.transform.rotation.w = float(qw / norm)

        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(message)
        self.get_logger().info(
            f'Published calibrated static TF {message.header.frame_id} -> '
            f'{message.child_frame_id} from {calibration_path}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--calibration', required=True)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = ExtrinsicsTfPublisher(args.calibration)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
