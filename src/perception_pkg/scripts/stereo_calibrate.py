#!/usr/bin/env python3
"""
Sony ZV-E10 II <-> Orbbec Gemini 335 RGB stereo calibration.
Detects chessboard corners on both camera streams simultaneously,
runs cv2.stereoCalibrate, and saves unambiguous OpenCV and ROS TF transforms.
"""
import argparse
import yaml
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data


class StereoCalibrator(Node):
    def __init__(
            self, rows, cols, square_size, output_path, max_skew_ms, max_rmse_px,
            show_preview):
        super().__init__('stereo_calibrator')
        self.bridge = CvBridge()
        self.rows = rows
        self.cols = cols
        self.square_size = square_size
        self.output_path = output_path
        self.max_skew_ns = int(max_skew_ms * 1_000_000)
        self.max_rmse_px = max_rmse_px
        self.show_preview = show_preview

        # storage
        self.sony_points = []
        self.gemini_points = []
        self.sony_gray = None
        self.gemini_color = None
        self.sony_ts = None
        self.gemini_ts = None
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
        self.sony_gray = cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg, 'bgr8'), cv2.COLOR_BGR2GRAY)
        self.sony_ts = msg.header.stamp

    def gemini_cb(self, msg):
        self.gemini_color = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.gemini_ts = msg.header.stamp

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
        """print capture count every 0.5s"""
        pass

    def capture_pair(self):
        if self.sony_gray is None or self.gemini_color is None:
            self.get_logger().warn('No images received yet')
            return
        sony_ns = self.sony_ts.sec * 1_000_000_000 + self.sony_ts.nanosec
        gemini_ns = self.gemini_ts.sec * 1_000_000_000 + self.gemini_ts.nanosec
        skew_ns = abs(sony_ns - gemini_ns)
        if skew_ns > self.max_skew_ns:
            self.get_logger().warn(
                f'Pair rejected: timestamp skew {skew_ns / 1e6:.1f} ms exceeds '
                f'{self.max_skew_ns / 1e6:.1f} ms')
            return

        gemini_gray = cv2.cvtColor(self.gemini_color, cv2.COLOR_BGR2GRAY)

        ret_s, corners_s = cv2.findChessboardCorners(self.sony_gray, (self.cols, self.rows),
                                                      cv2.CALIB_CB_ADAPTIVE_THRESH +
                                                      cv2.CALIB_CB_NORMALIZE_IMAGE)
        ret_g, corners_g = cv2.findChessboardCorners(gemini_gray, (self.cols, self.rows),
                                                      cv2.CALIB_CB_ADAPTIVE_THRESH +
                                                      cv2.CALIB_CB_NORMALIZE_IMAGE)

        if ret_s and ret_g:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            cv2.cornerSubPix(self.sony_gray, corners_s, (11, 11), (-1, -1), criteria)
            cv2.cornerSubPix(gemini_gray, corners_g, (11, 11), (-1, -1), criteria)

            self.sony_points.append(corners_s)
            self.gemini_points.append(corners_g)
            self.count += 1
            self.get_logger().info(
                f'Pair {self.count} captured '
                f'(Sony ts={self.sony_ts.sec}.{self.sony_ts.nanosec:09d}, '
                f'Gemini ts={self.gemini_ts.sec}.{self.gemini_ts.nanosec:09d})'
            )
        else:
            self.get_logger().warn(
                f'Chessboard not found: Sony={"OK" if ret_s else "FAIL"}, '
                f'Gemini={"OK" if ret_g else "FAIL"}'
            )

        if self.show_preview:
            cv2.drawChessboardCorners(
                self.sony_gray, (self.cols, self.rows), corners_s, ret_s)
            cv2.drawChessboardCorners(
                gemini_gray, (self.cols, self.rows), corners_g, ret_g)
            if gemini_gray.shape[0] != self.sony_gray.shape[0]:
                scale = self.sony_gray.shape[0] / gemini_gray.shape[0]
                new_w = int(gemini_gray.shape[1] * scale)
                gemini_resized = cv2.resize(
                    gemini_gray, (new_w, self.sony_gray.shape[0]))
            else:
                gemini_resized = gemini_gray
            combined = np.hstack([self.sony_gray, gemini_resized])
            cv2.imshow('stereo_calib (left=Sony, right=Gemini)', combined)
            cv2.waitKey(1)

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

        obj_points = [self.objp] * self.count

        self.get_logger().info(f'Running stereoCalibrate with {self.count} pairs...')
        flags = cv2.CALIB_FIX_INTRINSIC
        ret, K1, D1, K2, D2, R, T, E, F = cv2.stereoCalibrate(
            obj_points,
            self.sony_points,    # image points cam 0 (Sony)
            self.gemini_points,  # image points cam 1 (Gemini)
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
            'num_pairs': self.count,
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
    args = parser.parse_args()

    rclpy.init()
    node = StereoCalibrator(
        args.rows, args.cols, args.square, args.output,
        args.max_skew_ms, args.max_rmse_px, args.show_preview)

    import threading
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
