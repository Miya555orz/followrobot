#!/usr/bin/env python3
"""Tiny ROS 2 image-to-browser viewer for lab bring-up.

It subscribes to two sensor_msgs/Image topics and serves them as MJPEG streams:

  http://JETSON_IP:8088/

This is intentionally lightweight and dependency-minimal for Jetson field tests.
It is not a production web server.
"""

from __future__ import annotations

import argparse
import html
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Dict, Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


class ImageCache:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._frames: Dict[str, bytes] = {}
        self._stamps: Dict[str, float] = {}

    def set(self, name: str, jpeg: bytes) -> None:
        with self._lock:
            self._frames[name] = jpeg
            self._stamps[name] = time.time()

    def get(self, name: str) -> tuple[Optional[bytes], Optional[float]]:
        with self._lock:
            return self._frames.get(name), self._stamps.get(name)


class RosImageSubscriber(Node):
    def __init__(self, topics: Dict[str, str], jpeg_quality: int, cache: ImageCache) -> None:
        super().__init__("ros_image_mjpeg_viewer")
        self._bridge = CvBridge()
        self._jpeg_quality = int(jpeg_quality)
        self._cache = cache
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._subs = []
        for name, topic in topics.items():
            self.get_logger().info(f"Serving {name} from {topic}")
            self._subs.append(
                self.create_subscription(
                    Image,
                    topic,
                    lambda msg, stream_name=name: self._on_image(stream_name, msg),
                    qos,
                )
            )

    def _on_image(self, name: str, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            try:
                frame = self._bridge.imgmsg_to_cv2(msg)
                if frame.ndim == 2:
                    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            except Exception as exc:  # pragma: no cover - lab diagnostic path
                self.get_logger().warn("Failed to convert %s: %s", name, exc)
                return

        ok, encoded = cv2.imencode(
            ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality]
        )
        if ok:
            self._cache.set(name, encoded.tobytes())


def make_handler(cache: ImageCache, stream_names: list[str], page_title: str):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *args) -> None:  # quiet default logs
            return

        def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
            if self.path in ("/", "/index.html"):
                self._send_index()
                return
            if self.path.startswith("/stream/"):
                self._send_stream(self.path.rsplit("/", 1)[-1])
                return
            if self.path.startswith("/snapshot/"):
                name = self.path.rsplit("/", 1)[-1].replace(".jpg", "")
                self._send_snapshot(name)
                return
            self.send_error(404, "not found")

        def _send_index(self) -> None:
            cards = []
            for name in stream_names:
                safe = html.escape(name)
                cards.append(
                    f"""
                    <section class="card">
                      <h2>{safe}</h2>
                      <img src="/stream/{safe}" alt="{safe}">
                      <p><a href="/snapshot/{safe}.jpg">snapshot</a></p>
                    </section>
                    """
                )
            body = f"""<!doctype html>
            <html><head><meta charset="utf-8"><title>{html.escape(page_title)}</title>
            <style>
              body {{ margin: 0; padding: 20px; background: #10131a; color: #e9eef7;
                     font-family: system-ui, -apple-system, BlinkMacSystemFont, sans-serif; }}
              h1 {{ margin: 0 0 14px; font-size: 22px; font-weight: 650; }}
              .grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(420px, 1fr));
                       gap: 18px; align-items: start; }}
              .card {{ background: #171c26; border: 1px solid #2a3345; border-radius: 14px;
                       padding: 14px; box-shadow: 0 10px 30px rgba(0,0,0,.25); }}
              h2 {{ margin: 0 0 10px; color: #8ec5ff; font-size: 16px; }}
              img {{ width: 100%; height: auto; border-radius: 10px; background: #06080c; }}
              a {{ color: #91f0a5; }}
              .hint {{ color: #a7b1c2; margin-bottom: 18px; }}
            </style></head>
            <body><h1>{html.escape(page_title)}</h1>
            <div class="hint">Sony raw image + OpenCV perception debug stream. Refresh if a panel is blank for the first few seconds.</div>
            <main class="grid">{''.join(cards)}</main></body></html>"""
            payload = body.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def _send_snapshot(self, name: str) -> None:
            frame, _ = cache.get(name)
            if frame is None:
                self.send_error(503, f"no frame yet for {name}")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(frame)))
            self.end_headers()
            self.wfile.write(frame)

        def _send_stream(self, name: str) -> None:
            if name not in stream_names:
                self.send_error(404, "unknown stream")
                return
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            last_stamp = 0.0
            while True:
                frame, stamp = cache.get(name)
                if frame is not None and stamp != last_stamp:
                    last_stamp = stamp or 0.0
                    try:
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                    except (BrokenPipeError, ConnectionResetError):
                        return
                time.sleep(0.05)

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8088)
    parser.add_argument("--raw-topic", default="/sony/image_raw")
    parser.add_argument("--debug-topic", default="/perception/debug_image")
    parser.add_argument("--jpeg-quality", type=int, default=75)
    args = parser.parse_args()

    cache = ImageCache()
    topics = {"sony_raw": args.raw_topic, "opencv_debug": args.debug_topic}

    rclpy.init()
    node = RosImageSubscriber(topics, args.jpeg_quality, cache)
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    server = ThreadingHTTPServer(
        (args.host, args.port),
        make_handler(cache, list(topics.keys()), "followrobot live perception viewer"),
    )
    print(f"Open http://{args.host}:{args.port}/ (use Jetson IP from the PC)", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
