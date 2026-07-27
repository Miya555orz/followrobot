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


class StereoCalibrator(Node):
    def __init__(
            self, rows, cols, square_size, output_path, max_skew_ms, max_rmse_px,
            show_preview, samples_dir, frame_buffer_size, min_sharpness,
            max_mono_rmse_px, max_pair_translation_m, max_pair_rotation_deg):
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
            'or type "calib" to run calibration, "quit" to exit.'
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
        translation_limit = max(
            self.max_pair_translation_m,
            translation_median + 3.0 * max(translation_mad, 1e-6))
        rotation_limit = max(
            self.max_pair_rotation_deg,
            rotation_median + 3.0 * max(rotation_mad, 1e-6))
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

        sony_points = [self.sony_points[index] for index in kept_indices]
        gemini_points = [self.gemini_points[index] for index in kept_indices]
        obj_points = [self.objp] * len(kept_indices)

        self.get_logger().info(
            f'Running stereoCalibrate with {len(kept_indices)}/{self.count} pairs...')
        flags = cv2.CALIB_FIX_INTRINSIC
        ret, K1, D1, K2, D2, R, T, E, F = cv2.stereoCalibrate(
            obj_points,
            sony_points,    # image points cam 0 (Sony)
            gemini_points,  # image points cam 1 (Gemini)
            self.K_sony, self.D_sony,
            self.K_gemini, self.D_gemini,
            self.sony_size,
            criteria=(cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 100, 1e-5),
            flags=flags,
        )

        self.get_logger().info(f'Calibration done. RMSE={ret:.4f} px')
        if ret > self.max_rmse_px:
            self.get_logger().error(
                f'RMSE {ret:.3f}px exceeds {self.max_rmse_px:.3f}px; '
                'calibration file was not written')
            return
        self.get_logger().info(f'R:\n{R}')
        self.get_logger().info(f'T (m): {T.T}')

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
            'num_pairs': len(kept_indices),
            'num_pairs_captured': self.count,
            'rejected_pair_indices': rejected_indices,
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
    args = parser.parse_args()
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
        args.max_pair_rotation_deg)

    def input_thread():
        while rclpy.ok():
            try:
                cmd = input()
                if cmd.strip() == 'quit':
                    rclpy.shutdown()
                    return
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
