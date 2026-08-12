"""HTTP JSON input -> ROS 2 /voice/text bridge."""

import hmac
import json
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from external_control_pkg.msg import VoiceCommand
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from vision_servo_msgs.msg import SystemState

from voice_intent_pkg.voice_text_protocol import (
    PayloadValidationError,
    RequestDeduplicator,
    VoiceTextRequest,
    VoiceCommandRequest,
    validate_command_payload,
    validate_payload,
    validate_timestamp,
)


class VoiceTextHttpBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("voice_text_http_bridge_node")
        self.declare_parameter("bind_address", "0.0.0.0")
        self.declare_parameter("port", 8081)
        self.declare_parameter("text_topic", "/voice/text")
        self.declare_parameter("command_topic", "/external/voice_command")
        self.declare_parameter("allow_text_endpoint", False)
        self.declare_parameter("max_text_length", 500)
        self.declare_parameter("max_body_bytes", 4096)
        self.declare_parameter("queue_capacity", 32)
        self.declare_parameter("dedup_ttl_sec", 60.0)
        self.declare_parameter("dedup_max_entries", 512)
        self.declare_parameter("auth_token", "")
        self.declare_parameter("require_auth", True)
        self.declare_parameter("max_request_age_sec", 10.0)
        self.declare_parameter("max_future_skew_sec", 5.0)
        self.declare_parameter("state_topic", "/system/state")
        self.declare_parameter("state_timeout_sec", 3.0)

        self._bind_address = str(self.get_parameter("bind_address").value)
        self._port = int(self.get_parameter("port").value)
        self._max_text_length = int(
            self.get_parameter("max_text_length").value
        )
        self._max_body_bytes = int(self.get_parameter("max_body_bytes").value)
        queue_capacity = int(self.get_parameter("queue_capacity").value)
        self._auth_token = str(self.get_parameter("auth_token").value)
        self._require_auth = bool(self.get_parameter("require_auth").value)
        self._allow_text_endpoint = bool(
            self.get_parameter("allow_text_endpoint").value
        )
        self._max_request_age_sec = float(
            self.get_parameter("max_request_age_sec").value
        )
        self._max_future_skew_sec = float(
            self.get_parameter("max_future_skew_sec").value
        )
        self._state_timeout_sec = float(
            self.get_parameter("state_timeout_sec").value
        )
        self._validate_parameters(queue_capacity)
        if self._require_auth and not self._auth_token:
            raise ValueError(
                "auth_token is required; set FCR_VOICE_AUTH_TOKEN or disable voice"
            )

        self._state_lock = threading.Lock()
        self._system_state = None
        self._system_state_at = 0.0
        self._state_subscription = self.create_subscription(
            SystemState,
            str(self.get_parameter("state_topic").value),
            self._system_state_callback,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        self._requests: "queue.Queue[VoiceTextRequest]" = queue.Queue(
            maxsize=queue_capacity
        )
        self._commands: "queue.Queue[VoiceCommandRequest]" = queue.Queue(
            maxsize=queue_capacity
        )
        self._deduplicator = RequestDeduplicator(
            ttl_sec=float(self.get_parameter("dedup_ttl_sec").value),
            max_entries=int(self.get_parameter("dedup_max_entries").value),
        )
        text_topic = str(self.get_parameter("text_topic").value)
        self._publisher = self.create_publisher(String, text_topic, 10)
        command_topic = str(self.get_parameter("command_topic").value)
        self._command_publisher = self.create_publisher(
            VoiceCommand, command_topic, 10
        )
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
        if not self._auth_token:
            self.get_logger().warning(
                "voice HTTP authentication is disabled; set FCR_VOICE_AUTH_TOKEN"
            )
        self.get_logger().info(
            "voice HTTP bridge ready | http://%s:%d/voice/command -> %s"
            % (self._bind_address, self._port, command_topic)
        )

    def _validate_parameters(self, queue_capacity: int) -> None:
        if not 1 <= self._port <= 65535:
            raise ValueError("port must be between 1 and 65535")
        if self._max_text_length <= 0 or self._max_body_bytes <= 0:
            raise ValueError("text and body limits must be positive")
        if queue_capacity <= 0:
            raise ValueError("queue_capacity must be positive")
        if self._state_timeout_sec <= 0.0:
            raise ValueError("state_timeout_sec must be positive")
        if self._max_request_age_sec <= 0.0 or self._max_future_skew_sec <= 0.0:
            raise ValueError("request timestamp limits must be positive")

    def _system_state_callback(self, message: SystemState) -> None:
        with self._state_lock:
            self._system_state = message
            self._system_state_at = time.monotonic()

    def _state_snapshot(self) -> dict:
        with self._state_lock:
            message = self._system_state
            age_sec = time.monotonic() - self._system_state_at
            if message is None or age_sec > self._state_timeout_sec:
                return {
                    "available": False,
                    "age_sec": None if message is None else age_sec,
                    "reason": "system_state_unavailable" if message is None
                    else "system_state_stale",
                }
            return {
                "available": True,
                "age_sec": age_sec,
                "mode": int(message.mode),
                "mode_name": message.mode_name,
                "cinematic_state": int(message.cinematic_state),
                "state_version": int(message.state_version),
                "emergency_stop": bool(message.emergency_stop),
                "detail": message.detail,
            }

    def _make_handler(self) -> type[BaseHTTPRequestHandler]:
        enqueue = self._enqueue_request
        enqueue_command = self._enqueue_command
        health = lambda: {
            "status": "ok",
            "text_queued": self._requests.qsize(),
            "command_queued": self._commands.qsize(),
            "protocol_version": 1,
        }
        state_snapshot = self._state_snapshot
        auth_token = self._auth_token
        max_body_bytes = self._max_body_bytes
        max_text_length = self._max_text_length
        allow_text_endpoint = self._allow_text_endpoint

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:
                if self.path == "/health":
                    self._write_json(200, health())
                    return
                if self.path == "/voice/state":
                    if auth_token:
                        expected = f"Bearer {auth_token}"
                        supplied = self.headers.get("Authorization", "")
                        if not hmac.compare_digest(supplied, expected):
                            self._write_json(401, {"error": "unauthorized"})
                            return
                    snapshot = state_snapshot()
                    self._write_json(200 if snapshot["available"] else 503, snapshot)
                    return
                self._write_json(404, {"error": "not found"})

            def do_POST(self) -> None:
                if self.path not in ("/voice/text", "/voice/command"):
                    self._write_json(404, {"error": "not found"})
                    return
                if self.path == "/voice/text" and not allow_text_endpoint:
                    self._write_json(
                        410,
                        {"error": "raw text endpoint disabled; use /voice/command"},
                    )
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
                    request = (
                        validate_command_payload(payload, max_text_length)
                        if self.path == "/voice/command"
                        else validate_payload(payload, max_text_length)
                    )
                except (json.JSONDecodeError, UnicodeDecodeError) as exc:
                    self._write_json(400, {"error": f"invalid JSON: {exc}"})
                    return
                except PayloadValidationError as exc:
                    self._write_json(422, {"error": str(exc)})
                    return

                status, response = (
                    enqueue_command(request)
                    if self.path == "/voice/command"
                    else enqueue(request)
                )
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
        return self._enqueue_fresh(request, self._requests)

    def _enqueue_command(self, request: VoiceCommandRequest) -> tuple[int, dict]:
        return self._enqueue_fresh(request, self._commands)

    def _enqueue_fresh(self, request, target_queue) -> tuple[int, dict]:
        try:
            validate_timestamp(
                request.timestamp,
                max_request_age_sec=self._max_request_age_sec,
                max_future_skew_sec=self._max_future_skew_sec,
            )
        except PayloadValidationError as exc:
            return 422, {
                "accepted": False,
                "duplicate": False,
                "request_id": request.request_id,
                "error": str(exc),
            }
        if self._deduplicator.check_and_record(request.request_id):
            return 200, {
                "accepted": False,
                "duplicate": True,
                "request_id": request.request_id,
            }
        try:
            target_queue.put_nowait(request)
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
                break
            message = String()
            message.data = request.text
            self._publisher.publish(message)
            self.get_logger().info(
                'remote ASR text published | request_id=%s source=%s text="%s"'
                % (request.request_id, request.source, request.text)
            )
        while True:
            try:
                request = self._commands.get_nowait()
            except queue.Empty:
                return
            message = VoiceCommand()
            sec = int(request.timestamp)
            message.header.stamp.sec = sec
            message.header.stamp.nanosec = int((request.timestamp - sec) * 1e9)
            message.header.frame_id = request.source
            message.intents = list(request.intents)
            message.confidences = list(request.confidences)
            message.raw_text = request.raw_text
            message.distance = request.distance
            message.unit = request.unit
            message.distance_relative = request.distance_relative
            message.angle = request.angle
            message.direction = request.direction
            message.speed = request.speed
            message.target_desc = request.target_desc
            message.follow = request.follow
            self._command_publisher.publish(message)
            self.get_logger().info(
                "remote candidate command published | request_id=%s source=%s intents=%s"
                % (request.request_id, request.source, ",".join(request.intents))
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
