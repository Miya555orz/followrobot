#!/usr/bin/env python3
"""从视频文件或 V4L2/UVC 相机逐帧发布到 ROS2 图像话题。

默认输出 `/sony/image_raw`，可用于：

- 无相机环境下使用录制视频验证感知管线；
- Jetson 上把 Sony ZV-E10M2 的 UVC `/dev/video*` 设备发布为 ROS2 图像。
"""

import argparse
import os

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image


class VideoPublisher(Node):
    def __init__(
        self,
        source,
        *,
        is_device=False,
        topic="/sony/image_raw",
        camera_info_topic="/sony/camera_info",
        frame_id="sony_camera_optical_frame",
        loop=True,
        width=0,
        height=0,
        fps=0.0,
        camera_info_yaml="",
    ):
        super().__init__("sony_uvc_publisher" if is_device else "video_publisher")
        self._loop = loop
        self._frame_id = frame_id
        self._camera_info = self._load_camera_info(camera_info_yaml)

        if not is_device and not os.path.isfile(source):
            self.get_logger().fatal(f"视频文件不存在: {source}")
            raise FileNotFoundError(f"视频文件不存在: {source}")

        capture_source = self._parse_device(source) if is_device else source
        api = cv2.CAP_V4L2 if is_device else cv2.CAP_ANY
        self._cap = cv2.VideoCapture(capture_source, api)
        if not self._cap.isOpened():
            self.get_logger().fatal(f"无法打开输入源: {source}")
            raise RuntimeError(f"无法打开输入源: {source}")

        if width > 0:
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(width))
        if height > 0:
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(height))
        if fps > 0:
            self._cap.set(cv2.CAP_PROP_FPS, float(fps))

        self._width = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self._height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self._fps = self._cap.get(cv2.CAP_PROP_FPS)
        if self._fps <= 0:
            self._fps = 30.0
        self._frame_count = int(self._cap.get(cv2.CAP_PROP_FRAME_COUNT))

        source_kind = "相机设备" if is_device else "视频"
        self.get_logger().info(
            f"{source_kind}: {source} 分辨率={self._width}x{self._height} "
            f"FPS={self._fps:.1f} 总帧数={self._frame_count}"
        )

        period = 1.0 / self._fps
        self._pub = self.create_publisher(Image, topic, 10)
        self._camera_info_pub = self.create_publisher(CameraInfo, camera_info_topic, 10)
        self._bridge = CvBridge()
        self._timer = self.create_timer(period, self._publish_frame)
        self._frame_index = 0
        self.get_logger().info(f"开始向 {topic} 发布图像，向 {camera_info_topic} 发布 CameraInfo")

    @staticmethod
    def _parse_device(source):
        if source.startswith("/dev/video"):
            return source
        try:
            return int(source)
        except ValueError:
            return source

    def _load_camera_info(self, camera_info_yaml):
        info = CameraInfo()
        info.width = 0
        info.height = 0
        if not camera_info_yaml:
            return info
        try:
            import yaml

            with open(camera_info_yaml, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            info.width = int(data.get("image_width", 0))
            info.height = int(data.get("image_height", 0))
            info.distortion_model = str(data.get("distortion_model", "plumb_bob"))
            info.d = [float(x) for x in data.get("distortion_coefficients", {}).get("data", [])]
            info.k = [float(x) for x in data.get("camera_matrix", {}).get("data", [0.0] * 9)]
            info.r = [float(x) for x in data.get("rectification_matrix", {}).get("data", [0.0] * 9)]
            info.p = [float(x) for x in data.get("projection_matrix", {}).get("data", [0.0] * 12)]
            self.get_logger().info(f"已加载相机内参: {camera_info_yaml}")
        except Exception as exc:  # noqa: BLE001 - keep publisher usable without calibration.
            self.get_logger().warning(f"加载相机内参失败，将发布未标定 CameraInfo: {exc}")
        return info

    def _publish_frame(self):
        ret, frame = self._cap.read()
        if not ret:
            if not self._loop:
                self.get_logger().info("视频播放完毕，退出")
                rclpy.shutdown()
                return
            self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            self._frame_index = 0
            self.get_logger().info("循环播放")
            ret, frame = self._cap.read()
            if not ret:
                self.get_logger().error("重新读取视频失败")
                rclpy.shutdown()
                return

        self._frame_index += 1
        msg = self._bridge.cv2_to_imgmsg(frame, "bgr8")
        msg.header.frame_id = self._frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(msg)

        info = CameraInfo()
        info.header = msg.header
        info.width = self._camera_info.width or msg.width
        info.height = self._camera_info.height or msg.height
        info.distortion_model = self._camera_info.distortion_model
        info.d = list(self._camera_info.d)
        info.k = list(self._camera_info.k)
        info.r = list(self._camera_info.r)
        info.p = list(self._camera_info.p)
        self._camera_info_pub.publish(info)


def main(args=None):
    parser = argparse.ArgumentParser(
        description="将视频文件或 UVC/V4L2 相机发布为 ROS2 图像话题"
    )
    source_group = parser.add_mutually_exclusive_group(required=True)
    source_group.add_argument("--path", help="视频文件路径")
    source_group.add_argument("--device", help="相机设备，例如 /dev/video8 或 8")
    parser.add_argument(
        "--topic",
        default="/sony/image_raw",
        help="输出图像话题（默认: /sony/image_raw）",
    )
    parser.add_argument(
        "--camera-info-topic",
        default="/sony/camera_info",
        help="输出 CameraInfo 话题（默认: /sony/camera_info）",
    )
    parser.add_argument(
        "--frame-id",
        default="sony_camera_optical_frame",
        help="图像 frame_id（默认: sony_camera_optical_frame）",
    )
    parser.add_argument("--width", type=int, default=0, help="相机请求宽度；0 表示不设置")
    parser.add_argument("--height", type=int, default=0, help="相机请求高度；0 表示不设置")
    parser.add_argument("--fps", type=float, default=0.0, help="相机请求 FPS；0 表示不设置")
    parser.add_argument(
        "--camera-info-yaml",
        default="",
        help="可选 CameraInfo 标定 YAML，例如 perception_pkg/config/calibration/sony_zv_e10_ii.yaml",
    )
    parser.add_argument(
        "--no-loop",
        action="store_true",
        help="播放完不循环，直接退出",
    )
    parsed, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    is_device = parsed.device is not None
    node = VideoPublisher(
        source=parsed.device if is_device else parsed.path,
        is_device=is_device,
        topic=parsed.topic,
        camera_info_topic=parsed.camera_info_topic,
        frame_id=parsed.frame_id,
        loop=not parsed.no_loop,
        width=parsed.width,
        height=parsed.height,
        fps=parsed.fps,
        camera_info_yaml=parsed.camera_info_yaml,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
