#!/usr/bin/env python3
"""
Sony ZV-E10 II <-> Orbbec Gemini 335 RGB stereo calibration.
Detects chessboard corners on both camera streams simultaneously,
runs cv2.stereoCalibrate, and saves unambiguous OpenCV and ROS TF transforms.
"""
import argparse
from collections import deque
from pathlib import Path
import threading

import yaml
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data


def stamp_to_nanoseconds(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def closest_frame_pair(sony_frames, gemini_frames, max_skew_ns=None):
    """Return the newest eligible pair, or the closest pair for diagnostics."""
    if not sony_frames or not gemini_frames:
        return None
    pairs = [(sony, gemini) for sony in sony_frames for gemini in gemini_frames]
    if max_skew_ns is not None:
        eligible = [
            pair for pair in pairs
            if abs(pair[0][0] - pair[1][0]) <= max_skew_ns
        ]
        if eligible:
            # Prefer the newest synchronized observation. Selecting the global
            # minimum skew can otherwise keep returning an old exact match.
            return max(
                eligible,
                key=lambda pair: (min(pair[0][0], pair[1][0]),
                                  -abs(pair[0][0] - pair[1][0])))
    return min(pairs, key=lambda pair: abs(pair[0][0] - pair[1][0]))


def reprojection_rmse(object_points, image_points, camera_matrix, distortion):
    """Return single-camera PnP reprojection RMSE and board pose."""
    ok, rvec, tvec = cv2.solvePnP(
        object_points, image_points, camera_matrix, distortion,
        flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return float('inf'), None
    projected, _ = cv2.projectPoints(
        object_points, rvec, tvec, camera_matrix, distortion)
    residual = projected.reshape(-1, 2) - image_points.reshape(-1, 2)
    rmse = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
    rotation, _ = cv2.Rodrigues(rvec)
    pose_camera_from_board = np.eye(4, dtype=np.float64)
    pose_camera_from_board[:3, :3] = rotation
    pose_camera_from_board[:3, 3] = tvec.reshape(3)
    return rmse, pose_camera_from_board


def rotation_distance_degrees(first, second):
    relative = first[:3, :3] @ second[:3, :3].T
    cosine = np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


def symmetric_epipolar_rmse(first_points, second_points, fundamental):
    """Return symmetric point-to-epipolar-line RMSE for one image pair."""
    first = first_points.reshape(-1, 2).astype(np.float64)
    second = second_points.reshape(-1, 2).astype(np.float64)
    first_h = np.column_stack([first, np.ones(len(first))])
    second_h = np.column_stack([second, np.ones(len(second))])
    lines_in_second = (fundamental @ first_h.T).T
    lines_in_first = (fundamental.T @ second_h.T).T
    second_denominator = np.maximum(
        np.linalg.norm(lines_in_second[:, :2], axis=1), 1e-12)
    first_denominator = np.maximum(
        np.linalg.norm(lines_in_first[:, :2], axis=1), 1e-12)
    distance_second = np.sum(
        second_h * lines_in_second, axis=1) / second_denominator
    distance_first = np.sum(
        first_h * lines_in_first, axis=1) / first_denominator
    squared = 0.5 * (
        distance_first * distance_first +
        distance_second * distance_second)
    return float(np.sqrt(np.mean(squared)))


def write_ros_camera_info(path, camera_name, image_size, camera_matrix, distortion):
    """Write a camera_info_manager-compatible pinhole calibration file."""
    width, height = image_size
    fx = float(camera_matrix[0, 0])
    fy = float(camera_matrix[1, 1])
    cx = float(camera_matrix[0, 2])
    cy = float(camera_matrix[1, 2])
    document = {
        'image_width': int(width),
        'image_height': int(height),
        'camera_name': camera_name,
        'camera_matrix': {
            'rows': 3, 'cols': 3,
            'data': np.asarray(camera_matrix).reshape(-1).tolist(),
        },
        'distortion_model': 'plumb_bob',
        'distortion_coefficients': {
            'rows': 1,
            'cols': int(np.asarray(distortion).size),
            'data': np.asarray(distortion).reshape(-1).tolist(),
        },
        'rectification_matrix': {
            'rows': 3, 'cols': 3,
            'data': np.eye(3).reshape(-1).tolist(),
        },
        'projection_matrix': {
            'rows': 3, 'cols': 4,
            'data': [
                fx, 0.0, cx, 0.0,
                0.0, fy, cy, 0.0,
                0.0, 0.0, 1.0, 0.0,
            ],
        },
    }
    with open(path, 'w', encoding='utf-8') as stream:
        yaml.safe_dump(document, stream, sort_keys=False)


class StereoCalibrator(Node):
    def __init__(
            self, rows, cols, square_size, output_path, max_skew_ms, max_rmse_px,
            show_preview, samples_dir, frame_buffer_size, min_sharpness,
            max_mono_rmse_px, max_pair_translation_m, max_pair_rotation_deg,
            refine_intrinsics, max_epipolar_rmse_px,
            max_stereo_outlier_fraction):
        super().__init__('stereo_calibrator')
        self.bridge = CvBridge()
        self.rows = rows
        self.cols = cols
        self.square_size = square_size
        self.output_path = output_path
        self.max_skew_ns = int(max_skew_ms * 1_000_000)
        self.max_rmse_px = max_rmse_px
        self.show_preview = show_preview
        self.min_sharpness = min_sharpness
        self.max_mono_rmse_px = max_mono_rmse_px
        self.max_pair_translation_m = max_pair_translation_m
        self.max_pair_rotation_deg = max_pair_rotation_deg
        self.refine_intrinsics = refine_intrinsics
        self.max_epipolar_rmse_px = max_epipolar_rmse_px
        self.max_stereo_outlier_fraction = max_stereo_outlier_fraction
        self.samples_dir = Path(samples_dir)
        self.samples_dir.mkdir(parents=True, exist_ok=True)
        Path(self.output_path).expanduser().parent.mkdir(parents=True, exist_ok=True)

        # The ROS executor updates these queues while the terminal input thread
        # requests captures. Keep image and timestamp in one immutable tuple,
        # then snapshot the selected pair while holding the lock.
        self.frame_lock = threading.Lock()
        self.preview_lock = threading.Lock()
        self.sony_frames = deque(maxlen=frame_buffer_size)
        self.gemini_frames = deque(maxlen=frame_buffer_size)
        self.last_captured_frame_ids = None
        self.preview_image = None

        # Calibration storage.
        self.sony_points = []
        self.gemini_points = []
        self.pair_transforms = []
        self.sample_metadata = []
        self.count = 0

        # cached camera infos
        self.K_sony = None
        self.D_sony = None
        self.K_gemini = None
        self.D_gemini = None
        self.sony_size = None
        self.gemini_size = None

        # prepare chessboard object points
        self.objp = np.zeros((rows * cols, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_size

        qos_image = qos_profile_sensor_data
        self.sub_sony = self.create_subscription(
            Image, '/sony/image_raw', self.sony_cb, qos_image)
        self.sub_gemini = self.create_subscription(
            Image, '/camera/color/image_raw', self.gemini_cb, qos_image)
        self.sub_sony_info = self.create_subscription(
            CameraInfo, '/sony/camera_info', self.sony_info_cb, qos_image)
        self.sub_gemini_info = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.gemini_info_cb, qos_image)

        self.timer = self.create_timer(0.5, self.status_timer)
        self.get_logger().info(
            'StereoCalibrator ready. Press ENTER in terminal to capture an image pair, '
            'type "load" to load samples-dir, "calib" to run calibration, '
            'or "quit" to exit.'
        )

    def sony_cb(self, msg):
        gray = cv2.cvtColor(
            self.bridge.imgmsg_to_cv2(msg, 'bgr8'), cv2.COLOR_BGR2GRAY)
        sample = (stamp_to_nanoseconds(msg.header.stamp), gray)
        with self.frame_lock:
            self.sony_frames.append(sample)

    def gemini_cb(self, msg):
        gray = cv2.cvtColor(
            self.bridge.imgmsg_to_cv2(msg, 'bgr8'), cv2.COLOR_BGR2GRAY)
        sample = (stamp_to_nanoseconds(msg.header.stamp), gray)
        with self.frame_lock:
            self.gemini_frames.append(sample)

    def sony_info_cb(self, msg):
        if self.K_sony is None:
            self.K_sony = np.array(msg.k).reshape(3, 3)
            self.D_sony = np.array(msg.d)
            self.sony_size = (msg.width, msg.height)
            self.get_logger().info(f'Sony intrinsics loaded: {msg.width}x{msg.height}')

    def gemini_info_cb(self, msg):
        if self.K_gemini is None:
            self.K_gemini = np.array(msg.k).reshape(3, 3)
            self.D_gemini = np.array(msg.d)
            self.gemini_size = (msg.width, msg.height)
            self.get_logger().info(f'Gemini color intrinsics loaded: {msg.width}x{msg.height}')

    def status_timer(self):
        if self.show_preview:
            with self.preview_lock:
                preview = self.preview_image
            if preview is not None:
                cv2.imshow(
                    'stereo_calib (left=Sony, right=Gemini)', preview)
            cv2.waitKey(1)

    def capture_pair(self):
        if self.K_sony is None or self.K_gemini is None:
            self.get_logger().warn(
                'Pair rejected: waiting for both CameraInfo messages')
            return

        with self.frame_lock:
            pair = closest_frame_pair(
                self.sony_frames, self.gemini_frames, self.max_skew_ns)
            if pair is None:
                sony_gray = None
            else:
                (sony_ns, sony_buffered), (gemini_ns, gemini_buffered) = pair
                frame_ids = (sony_ns, gemini_ns)
                # Copies prevent later preview drawing or OpenCV refinement from
                # mutating a frame that remains in the callback queue.
                sony_gray = sony_buffered.copy()
                gemini_gray = gemini_buffered.copy()

        if sony_gray is None:
            self.get_logger().warn('No images received yet')
            return

        if frame_ids == self.last_captured_frame_ids:
            self.get_logger().warn(
                'Pair rejected: the closest synchronized frames were already captured; '
                'wait for new frames before pressing ENTER again')
            return

        skew_ns = abs(sony_ns - gemini_ns)
        if skew_ns > self.max_skew_ns:
            self.get_logger().warn(
                f'Pair rejected: closest buffered timestamp skew '
                f'{skew_ns / 1e6:.1f} ms exceeds '
                f'{self.max_skew_ns / 1e6:.1f} ms')
            return

        sony_sharpness = float(cv2.Laplacian(sony_gray, cv2.CV_64F).var())
        gemini_sharpness = float(cv2.Laplacian(gemini_gray, cv2.CV_64F).var())
        if min(sony_sharpness, gemini_sharpness) < self.min_sharpness:
            self.get_logger().warn(
                'Pair rejected: image is too blurry '
                f'(Sony={sony_sharpness:.1f}, Gemini={gemini_sharpness:.1f}, '
                f'minimum={self.min_sharpness:.1f})')
            return

        ret_s, corners_s = cv2.findChessboardCorners(sony_gray, (self.cols, self.rows),
                                                      cv2.CALIB_CB_ADAPTIVE_THRESH +
                                                      cv2.CALIB_CB_NORMALIZE_IMAGE)
        ret_g, corners_g = cv2.findChessboardCorners(gemini_gray, (self.cols, self.rows),
                                                      cv2.CALIB_CB_ADAPTIVE_THRESH +
                                                      cv2.CALIB_CB_NORMALIZE_IMAGE)

        if ret_s and ret_g:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            cv2.cornerSubPix(sony_gray, corners_s, (11, 11), (-1, -1), criteria)
            cv2.cornerSubPix(gemini_gray, corners_g, (11, 11), (-1, -1), criteria)

            sony_rmse, sony_pose = reprojection_rmse(
                self.objp, corners_s, self.K_sony, self.D_sony)
            gemini_rmse, gemini_pose = reprojection_rmse(
                self.objp, corners_g, self.K_gemini, self.D_gemini)
            if max(sony_rmse, gemini_rmse) > self.max_mono_rmse_px:
                self.get_logger().warn(
                    'Pair rejected: monocular reprojection error is too high '
                    f'(Sony={sony_rmse:.3f}px, Gemini={gemini_rmse:.3f}px, '
                    f'maximum={self.max_mono_rmse_px:.3f}px)')
                return

            gemini_from_sony = gemini_pose @ np.linalg.inv(sony_pose)
            self.sony_points.append(corners_s)
            self.gemini_points.append(corners_g)
            self.pair_transforms.append(gemini_from_sony)
            self.count += 1
            self.last_captured_frame_ids = frame_ids
            sample_stem = f'pair_{self.count:03d}'
            sony_path = self.samples_dir / f'{sample_stem}_sony.png'
            gemini_path = self.samples_dir / f'{sample_stem}_gemini.png'
            cv2.imwrite(str(sony_path), sony_gray)
            cv2.imwrite(str(gemini_path), gemini_gray)
            self.sample_metadata.append({
                'index': self.count,
                'sony_timestamp_ns': sony_ns,
                'gemini_timestamp_ns': gemini_ns,
                'skew_ms': skew_ns / 1e6,
                'sony_sharpness': sony_sharpness,
                'gemini_sharpness': gemini_sharpness,
                'sony_mono_rmse_px': sony_rmse,
                'gemini_mono_rmse_px': gemini_rmse,
                'sony_image': str(sony_path),
                'gemini_image': str(gemini_path),
            })
            with open(self.samples_dir / 'samples.yaml', 'w', encoding='utf-8') as stream:
                yaml.safe_dump(
                    {'schema_version': 1, 'samples': self.sample_metadata},
                    stream, sort_keys=False)
            self.get_logger().info(
                f'Pair {self.count} captured '
                f'(skew={skew_ns / 1e6:.1f}ms, '
                f'mono_rmse={sony_rmse:.3f}/{gemini_rmse:.3f}px, '
                f'sharpness={sony_sharpness:.1f}/{gemini_sharpness:.1f})'
            )
        else:
            self.get_logger().warn(
                f'Chessboard not found: Sony={"OK" if ret_s else "FAIL"}, '
                f'Gemini={"OK" if ret_g else "FAIL"}'
            )

        if self.show_preview:
            sony_preview = sony_gray.copy()
            gemini_preview = gemini_gray.copy()
            cv2.drawChessboardCorners(
                sony_preview, (self.cols, self.rows), corners_s, ret_s)
            cv2.drawChessboardCorners(
                gemini_preview, (self.cols, self.rows), corners_g, ret_g)
            if gemini_preview.shape[0] != sony_preview.shape[0]:
                scale = sony_preview.shape[0] / gemini_preview.shape[0]
                new_w = int(gemini_preview.shape[1] * scale)
                gemini_resized = cv2.resize(
                    gemini_preview, (new_w, sony_preview.shape[0]))
            else:
                gemini_resized = gemini_preview
            combined = np.hstack([sony_preview, gemini_resized])
            with self.preview_lock:
                self.preview_image = combined

    def load_saved_samples(self):
        """Reload accepted PNG pairs so calibration can be repeated offline."""
        if self.K_sony is None or self.K_gemini is None:
            self.get_logger().warn(
                'Cannot load samples yet: waiting for both CameraInfo messages')
            return
        if self.count:
            self.get_logger().error(
                'Cannot load into a non-empty session; restart the calibrator first')
            return

        sony_paths = sorted(self.samples_dir.glob('pair_*_sony.png'))
        if not sony_paths:
            self.get_logger().error(
                f'No pair_*_sony.png files found in {self.samples_dir}')
            return

        loaded = 0
        for sony_path in sony_paths:
            gemini_path = Path(
                str(sony_path).replace('_sony.png', '_gemini.png'))
            if not gemini_path.exists():
                self.get_logger().warn(
                    f'Skipping {sony_path.name}: matching Gemini image is missing')
                continue
            sony_gray = cv2.imread(str(sony_path), cv2.IMREAD_GRAYSCALE)
            gemini_gray = cv2.imread(str(gemini_path), cv2.IMREAD_GRAYSCALE)
            if sony_gray is None or gemini_gray is None:
                self.get_logger().warn(
                    f'Skipping {sony_path.name}: image could not be decoded')
                continue

            flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
            ret_s, corners_s = cv2.findChessboardCorners(
                sony_gray, (self.cols, self.rows), flags)
            ret_g, corners_g = cv2.findChessboardCorners(
                gemini_gray, (self.cols, self.rows), flags)
            if not ret_s or not ret_g:
                self.get_logger().warn(
                    f'Skipping {sony_path.name}: chessboard redetection failed '
                    f'(Sony={ret_s}, Gemini={ret_g})')
                continue

            criteria = (
                cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            cv2.cornerSubPix(
                sony_gray, corners_s, (11, 11), (-1, -1), criteria)
            cv2.cornerSubPix(
                gemini_gray, corners_g, (11, 11), (-1, -1), criteria)
            sony_rmse, sony_pose = reprojection_rmse(
                self.objp, corners_s, self.K_sony, self.D_sony)
            gemini_rmse, gemini_pose = reprojection_rmse(
                self.objp, corners_g, self.K_gemini, self.D_gemini)
            if max(sony_rmse, gemini_rmse) > self.max_mono_rmse_px:
                self.get_logger().warn(
                    f'Skipping {sony_path.name}: mono RMSE '
                    f'{sony_rmse:.3f}/{gemini_rmse:.3f}px exceeds '
                    f'{self.max_mono_rmse_px:.3f}px')
                continue

            self.sony_points.append(corners_s)
            self.gemini_points.append(corners_g)
            self.pair_transforms.append(
                gemini_pose @ np.linalg.inv(sony_pose))
            loaded += 1

        self.count = loaded
        self.get_logger().info(
            f'Loaded {loaded}/{len(sony_paths)} saved image pairs from '
            f'{self.samples_dir}')

    def run_calibration(self):
        if self.count < 20:
            self.get_logger().error(f'Need at least 20 pairs, only have {self.count}')
            return
        if self.K_sony is None or self.K_gemini is None:
            self.get_logger().error('Missing camera intrinsics')
            return
        if self.K_sony[0, 0] <= 0 or self.K_sony[1, 1] <= 0:
            self.get_logger().error('Sony CameraInfo has invalid focal lengths')
            return
        if self.K_gemini[0, 0] <= 0 or self.K_gemini[1, 1] <= 0:
            self.get_logger().error('Gemini CameraInfo has invalid focal lengths')
            return

        for name, samples, size in (
                ('Sony', self.sony_points, self.sony_size),
                ('Gemini', self.gemini_points, self.gemini_size)):
            centers = np.asarray(
                [corners.reshape(-1, 2).mean(axis=0) for corners in samples])
            span = (centers.max(axis=0) - centers.min(axis=0)) / np.asarray(size)
            if span[0] < 0.35 or span[1] < 0.25:
                self.get_logger().error(
                    f'{name} calibration-board coverage is too narrow '
                    f'(x={span[0]:.2f}, y={span[1]:.2f}); capture corners and centre')
                return

        # A rigid camera pair must yield nearly the same relative pose from
        # every independently solved board pose. Remove gross corner-order,
        # motion, and pose-estimation outliers before the joint optimization.
        medoid_index = min(
            range(self.count),
            key=lambda candidate: sum(
                rotation_distance_degrees(
                    self.pair_transforms[candidate], transform) +
                100.0 * np.linalg.norm(
                    self.pair_transforms[candidate][:3, 3] - transform[:3, 3])
                for transform in self.pair_transforms))
        medoid = self.pair_transforms[medoid_index]
        translation_errors = np.asarray([
            np.linalg.norm(transform[:3, 3] - medoid[:3, 3])
            for transform in self.pair_transforms])
        rotation_errors = np.asarray([
            rotation_distance_degrees(transform, medoid)
            for transform in self.pair_transforms])
        translation_median = float(np.median(translation_errors))
        rotation_median = float(np.median(rotation_errors))
        translation_mad = float(np.median(np.abs(
            translation_errors - translation_median)))
        rotation_mad = float(np.median(np.abs(
            rotation_errors - rotation_median)))
        # Robust statistics adapt to normal PnP noise, while the configured
        # maxima remain hard caps rather than accidentally widening the gate.
        translation_limit = min(
            self.max_pair_translation_m,
            max(0.01, translation_median + 3.0 * max(translation_mad, 1e-6)))
        rotation_limit = min(
            self.max_pair_rotation_deg,
            max(0.5, rotation_median + 3.0 * max(rotation_mad, 1e-6)))
        kept_indices = [
            index for index in range(self.count)
            if translation_errors[index] <= translation_limit and
            rotation_errors[index] <= rotation_limit
        ]
        rejected_indices = [
            index + 1 for index in range(self.count) if index not in kept_indices]
        if rejected_indices:
            self.get_logger().warn(
                f'Rejected rigid-pose outlier pairs {rejected_indices} '
                f'(translation_limit={translation_limit:.3f}m, '
                f'rotation_limit={rotation_limit:.2f}deg)')
        if len(kept_indices) < 20:
            self.get_logger().error(
                f'Only {len(kept_indices)} pairs remain after rigid-pose filtering; '
                'capture more diverse, stationary pairs')
            return

        flags = (
            cv2.CALIB_USE_INTRINSIC_GUESS
            if self.refine_intrinsics else cv2.CALIB_FIX_INTRINSIC)
        active_indices = list(kept_indices)
        maximum_stereo_rejections = int(np.floor(
            len(active_indices) * self.max_stereo_outlier_fraction))
        stereo_rejected_indices = []

        while True:
            sony_points = [
                self.sony_points[index] for index in active_indices]
            gemini_points = [
                self.gemini_points[index] for index in active_indices]
            obj_points = [self.objp] * len(active_indices)
            self.get_logger().info(
                f'Running stereoCalibrate with {len(active_indices)}/'
                f'{self.count} pairs...')
            ret, K1, D1, K2, D2, R, T, E, F = cv2.stereoCalibrate(
                obj_points,
                sony_points,
                gemini_points,
                self.K_sony.copy(), self.D_sony.copy(),
                self.K_gemini.copy(), self.D_gemini.copy(),
                self.sony_size,
                criteria=(
                    cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS,
                    100, 1e-5),
                flags=flags,
            )

            epipolar_errors = np.asarray([
                symmetric_epipolar_rmse(first, second, F)
                for first, second in zip(sony_points, gemini_points)
            ])
            epipolar_median = float(np.median(epipolar_errors))
            epipolar_mad = float(np.median(np.abs(
                epipolar_errors - epipolar_median)))
            worst_local_index = int(np.argmax(epipolar_errors))
            worst_error = float(epipolar_errors[worst_local_index])
            robust_limit = max(
                self.max_epipolar_rmse_px,
                epipolar_median + 3.0 * max(epipolar_mad, 1e-6))
            self.get_logger().info(
                f'Stereo diagnostics: RMSE={ret:.4f}px, epipolar '
                f'median={epipolar_median:.3f}px, '
                f'max={worst_error:.3f}px')

            can_remove = (
                ret > self.max_rmse_px and
                worst_error > robust_limit and
                len(active_indices) > 20 and
                len(stereo_rejected_indices) < maximum_stereo_rejections)
            if not can_remove:
                break

            rejected_original_index = active_indices.pop(worst_local_index)
            stereo_rejected_indices.append(rejected_original_index + 1)
            self.get_logger().warn(
                f'Rejected stereo epipolar outlier pair '
                f'{rejected_original_index + 1} '
                f'({worst_error:.3f}px > {robust_limit:.3f}px)')

        self.get_logger().info(f'Calibration done. RMSE={ret:.4f} px')
        if ret > self.max_rmse_px:
            self.get_logger().error(
                f'RMSE {ret:.3f}px exceeds {self.max_rmse_px:.3f}px; '
                'calibration file was not written')
            return
        self.get_logger().info(f'R:\n{R}')
        self.get_logger().info(f'T (m): {T.T}')
        if self.refine_intrinsics:
            self.get_logger().info(
                'Joint intrinsic refinement was enabled; writing matched '
                'Sony and Gemini CameraInfo files next to the extrinsics')

        # OpenCV returns p_gemini = R * p_sony + T, i.e. T_gemini<-sony.
        T_gemini_from_sony = np.eye(4)
        T_gemini_from_sony[:3, :3] = R
        T_gemini_from_sony[:3, 3] = T.ravel()
        # The Orbbec driver already owns camera_link -> camera_color_optical_frame.
        # Connect the otherwise-unparented Sony optical frame below Gemini color:
        # parent=Gemini color, child=Sony. Its ROS pose is exactly T_gemini<-sony.

        # Save
        result = {
            'schema_version': 2,
            'opencv_T_gemini_from_sony': T_gemini_from_sony.tolist(),
            'ros_tf_T_parent_from_child': T_gemini_from_sony.tolist(),
            'R': R.tolist(),
            'T': T.ravel().tolist(),
            'rmse_px': float(ret),
            'num_pairs': len(active_indices),
            'num_pairs_captured': self.count,
            'rejected_pair_indices': rejected_indices,
            'stereo_rejected_pair_indices': stereo_rejected_indices,
            'epipolar_rmse_median_px': epipolar_median,
            'epipolar_rmse_max_px': worst_error,
            'intrinsics_refined': self.refine_intrinsics,
            'sony_camera_matrix': K1.tolist(),
            'sony_distortion': np.asarray(D1).reshape(-1).tolist(),
            'gemini_camera_matrix': K2.tolist(),
            'gemini_distortion': np.asarray(D2).reshape(-1).tolist(),
            'board_rows': self.rows,
            'board_cols': self.cols,
            'square_size_meters': self.square_size,
            'sony_image_size': list(self.sony_size),
            'gemini_image_size': list(self.gemini_size),
            'cam0': 'sony_zv_e10_ii',
            'cam1': 'gemini_335_color',
            'frame_id': 'camera_color_optical_frame',
            'child_frame_id': 'sony_camera_optical_frame',
        }
        with open(self.output_path, 'w') as f:
            yaml.dump(result, f, default_flow_style=False)
        if self.refine_intrinsics:
            output_parent = Path(self.output_path).expanduser().parent
            write_ros_camera_info(
                output_parent / 'sony_refined_camera_info.yaml',
                'sony_zv_e10_ii', self.sony_size, K1, D1)
            write_ros_camera_info(
                output_parent / 'gemini_refined_camera_info.yaml',
                'camera', self.gemini_size, K2, D2)
        self.get_logger().info(f'Saved to {self.output_path}')


def main():
    parser = argparse.ArgumentParser(description='Sony-Gemini stereo calibration')
    parser.add_argument('--rows', type=int, default=6, help='Chessboard inner corners: rows')
    parser.add_argument('--cols', type=int, default=8, help='Chessboard inner corners: cols')
    parser.add_argument('--square', type=float, default=0.025, help='Square size in meters')
    parser.add_argument('--output', type=str, default='T_sony_gemini.yaml')
    parser.add_argument(
        '--max-skew-ms', type=float, default=50.0,
        help='Reject a manually captured pair when timestamps differ by more than this')
    parser.add_argument(
        '--max-rmse-px', type=float, default=1.0,
        help='Do not write a calibration whose stereo RMSE exceeds this threshold')
    parser.add_argument(
        '--show-preview', action='store_true',
        help='Open an OpenCV window; omit this option for headless SSH calibration')
    parser.add_argument(
        '--samples-dir', default='',
        help='Directory for accepted source image pairs and samples.yaml')
    parser.add_argument(
        '--frame-buffer-size', type=int, default=30,
        help='Frames retained per camera when selecting the closest timestamp pair')
    parser.add_argument(
        '--min-sharpness', type=float, default=20.0,
        help='Reject a pair when either Laplacian sharpness is below this value')
    parser.add_argument(
        '--max-mono-rmse-px', type=float, default=2.0,
        help='Reject a pair when either fixed-intrinsic PnP RMSE exceeds this value')
    parser.add_argument(
        '--max-pair-translation-m', type=float, default=0.05,
        help='Minimum rigid-pose outlier threshold for relative translation')
    parser.add_argument(
        '--max-pair-rotation-deg', type=float, default=5.0,
        help='Minimum rigid-pose outlier threshold for relative rotation')
    parser.add_argument(
        '--refine-intrinsics', action='store_true',
        help='Jointly refine both intrinsics and write matched CameraInfo YAML files')
    parser.add_argument(
        '--max-epipolar-rmse-px', type=float, default=1.5,
        help='Minimum robust threshold for rejecting a stereo correspondence pair')
    parser.add_argument(
        '--max-stereo-outlier-fraction', type=float, default=0.25,
        help='Maximum fraction removed by iterative epipolar outlier rejection')
    args = parser.parse_args()
    if args.frame_buffer_size < 2:
        parser.error('--frame-buffer-size must be at least 2')
    if args.max_skew_ms <= 0.0:
        parser.error('--max-skew-ms must be positive')
    if not 0.0 <= args.max_stereo_outlier_fraction <= 0.5:
        parser.error('--max-stereo-outlier-fraction must be between 0.0 and 0.5')
    if args.samples_dir:
        samples_dir = args.samples_dir
    else:
        output = Path(args.output).expanduser()
        samples_dir = str(output.parent / f'{output.stem}_samples')

    rclpy.init()
    node = StereoCalibrator(
        args.rows, args.cols, args.square, args.output,
        args.max_skew_ms, args.max_rmse_px, args.show_preview,
        samples_dir, args.frame_buffer_size, args.min_sharpness,
        args.max_mono_rmse_px, args.max_pair_translation_m,
        args.max_pair_rotation_deg, args.refine_intrinsics,
        args.max_epipolar_rmse_px, args.max_stereo_outlier_fraction)

    def input_thread():
        while rclpy.ok():
            try:
                cmd = input()
                if cmd.strip() == 'quit':
                    rclpy.shutdown()
                    return
                elif cmd.strip() == 'load':
                    node.load_saved_samples()
                elif cmd.strip() == 'calib':
                    node.run_calibration()
                else:
                    node.capture_pair()
            except EOFError:
                break

    thread = threading.Thread(target=input_thread, daemon=True)
    thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
