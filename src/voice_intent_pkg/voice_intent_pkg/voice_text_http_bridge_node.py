"""HTTP JSON input -> ROS 2 /voice/text bridge."""

import hmac
import json
import queue
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from voice_intent_pkg.voice_text_protocol import (
    PayloadValidationError,
    RequestDeduplicator,
    VoiceTextRequest,
    validate_payload,
)


class VoiceTextHttpBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("voice_text_http_bridge_node")
        self.declare_parameter("bind_address", "0.0.0.0")
        self.declare_parameter("port", 8081)
        self.declare_parameter("text_topic", "/voice/text")
        self.declare_parameter("max_text_length", 500)
        self.declare_parameter("max_body_bytes", 4096)
        self.declare_parameter("queue_capacity", 32)
        self.declare_parameter("dedup_ttl_sec", 60.0)
        self.declare_parameter("dedup_max_entries", 512)
        self.declare_parameter("auth_token", "")

        self._bind_address = str(self.get_parameter("bind_address").value)
        self._port = int(self.get_parameter("port").value)
        self._max_text_length = int(
            self.get_parameter("max_text_length").value
        )
        self._max_body_bytes = int(self.get_parameter("max_body_bytes").value)
        queue_capacity = int(self.get_parameter("queue_capacity").value)
        self._auth_token = str(self.get_parameter("auth_token").value)
        self._validate_parameters(queue_capacity)

        self._requests: "queue.Queue[VoiceTextRequest]" = queue.Queue(
            maxsize=queue_capacity
        )
        self._deduplicator = RequestDeduplicator(
            ttl_sec=float(self.get_parameter("dedup_ttl_sec").value),
            max_entries=int(self.get_parameter("dedup_max_entries").value),
        )
        text_topic = str(self.get_parameter("text_topic").value)
        self._publisher = self.create_publisher(String, text_topic, 10)
        self.create_timer(0.02, self._publish_queued_requests)

        handler = self._make_handler()
        self._http_server = ThreadingHTTPServer(
            (self._bind_address, self._port), handler
        )
        self._http_thread = threading.Thread(
            target=self._http_server.serve_forever,
            name="voice_text_http_server",
            daemon=True,
        )
        self._http_thread.start()
        self.get_logger().info(
            "voice text HTTP bridge ready | http://%s:%d/voice/text -> %s"
            % (self._bind_address, self._port, text_topic)
        )

    def _validate_parameters(self, queue_capacity: int) -> None:
        if not 1 <= self._port <= 65535:
            raise ValueError("port must be between 1 and 65535")
        if self._max_text_length <= 0 or self._max_body_bytes <= 0:
            raise ValueError("text and body limits must be positive")
        if queue_capacity <= 0:
            raise ValueError("queue_capacity must be positive")

    def _make_handler(self) -> type[BaseHTTPRequestHandler]:
        enqueue = self._enqueue_request
        health = lambda: {"status": "ok", "queued": self._requests.qsize()}
        auth_token = self._auth_token
        max_body_bytes = self._max_body_bytes
        max_text_length = self._max_text_length

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:
                if self.path == "/health":
                    self._write_json(200, health())
                    return
                self._write_json(404, {"error": "not found"})

            def do_POST(self) -> None:
                if self.path != "/voice/text":
                    self._write_json(404, {"error": "not found"})
                    return
                if auth_token:
                    expected = f"Bearer {auth_token}"
                    supplied = self.headers.get("Authorization", "")
                    if not hmac.compare_digest(supplied, expected):
                        self._write_json(401, {"error": "unauthorized"})
                        return

                try:
                    length = int(self.headers.get("Content-Length", "0"))
                except ValueError:
                    self._write_json(400, {"error": "invalid Content-Length"})
                    return
                if length <= 0 or length > max_body_bytes:
                    self._write_json(413, {"error": "invalid body size"})
                    return

                try:
                    payload = json.loads(self.rfile.read(length))
                    request = validate_payload(payload, max_text_length)
                except (json.JSONDecodeError, UnicodeDecodeError) as exc:
                    self._write_json(400, {"error": f"invalid JSON: {exc}"})
                    return
                except PayloadValidationError as exc:
                    self._write_json(422, {"error": str(exc)})
                    return

                status, response = enqueue(request)
                self._write_json(status, response)

            def _write_json(self, status: int, payload: dict) -> None:
                body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, format_string: str, *args: object) -> None:
                return

        return Handler

    def _enqueue_request(self, request: VoiceTextRequest) -> tuple[int, dict]:
        if self._deduplicator.check_and_record(request.request_id):
            return 200, {
                "accepted": False,
                "duplicate": True,
                "request_id": request.request_id,
            }
        try:
            self._requests.put_nowait(request)
        except queue.Full:
            self._deduplicator.forget(request.request_id)
            return 503, {
                "accepted": False,
                "duplicate": False,
                "request_id": request.request_id,
                "error": "bridge queue is full",
            }
        return 202, {
            "accepted": True,
            "duplicate": False,
            "request_id": request.request_id,
        }

    def _publish_queued_requests(self) -> None:
        while True:
            try:
                request = self._requests.get_nowait()
            except queue.Empty:
                return
            message = String()
            message.data = request.text
            self._publisher.publish(message)
            self.get_logger().info(
                'remote ASR text published | request_id=%s source=%s text="%s"'
                % (request.request_id, request.source, request.text)
            )

    def destroy_node(self) -> bool:
        self._http_server.shutdown()
        self._http_server.server_close()
        self._http_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VoiceTextHttpBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
