"""Validation and idempotency helpers for the voice text HTTP bridge."""

import time
import threading
import math
from collections import OrderedDict
from dataclasses import dataclass
from typing import Any, Mapping


class PayloadValidationError(ValueError):
    """Raised when an HTTP voice text payload violates the contract."""


@dataclass(frozen=True)
class VoiceTextRequest:
    text: str
    request_id: str
    timestamp: float
    source: str


@dataclass(frozen=True)
class VoiceCommandRequest:
    raw_text: str
    intents: tuple[str, ...]
    confidences: tuple[float, ...]
    request_id: str
    timestamp: float
    source: str
    distance: float = -1.0
    unit: str = ""
    distance_relative: bool = False
    angle: float = -1.0
    direction: int = 0
    speed: str = ""
    target_desc: str = ""
    follow: bool = False


ALLOWED_CANDIDATE_INTENTS = {
    "emergency_stop", "stop_all", "status_query",
    "gimbal_nudge_left", "gimbal_nudge_right", "gimbal_nudge_up",
    "gimbal_nudge_down", "gimbal_stop", "gimbal_home",
    "gimbal_speed_up", "gimbal_speed_down",
    "query_gimbal_status",
    "chassis_move_forward", "chassis_move_backward", "chassis_move_left",
    "chassis_move_right", "chassis_turn_left", "chassis_turn_right",
    "chassis_stop", "chassis_speed_up", "chassis_speed_down",
    "query_chassis_status",
    "camera_take_photo", "camera_start_recording", "camera_stop_recording",
    "query_camera_status",
    "start_following", "stop_following", "distance_adjust",
    "enter_cinematic", "exit_cinematic", "start_orbit", "start_dolly",
    "start_truck", "start_static_track", "stop_cinematic",
    "query_camera_motion_status",
}


def validate_timestamp(
    timestamp: float,
    *,
    now: float | None = None,
    max_request_age_sec: float,
    max_future_skew_sec: float,
) -> None:
    """Reject stale/replayed requests and clocks implausibly far in the future."""
    current = time.time() if now is None else now
    age_sec = current - timestamp
    if age_sec > max_request_age_sec:
        raise PayloadValidationError("request timestamp is stale")
    if age_sec < -max_future_skew_sec:
        raise PayloadValidationError("request timestamp is in the future")


def validate_payload(
    payload: Any,
    max_text_length: int,
    max_request_id_length: int = 128,
) -> VoiceTextRequest:
    if not isinstance(payload, Mapping):
        raise PayloadValidationError("JSON body must be an object")

    text = payload.get("text")
    request_id = payload.get("request_id")
    timestamp = payload.get("timestamp")
    source = payload.get("source", "remote_asr")

    if not isinstance(text, str) or not text.strip():
        raise PayloadValidationError("text must be a non-empty string")
    text = text.strip()
    if len(text) > max_text_length:
        raise PayloadValidationError(
            f"text exceeds maximum length {max_text_length}"
        )
    if not isinstance(request_id, str) or not request_id.strip():
        raise PayloadValidationError("request_id must be a non-empty string")
    request_id = request_id.strip()
    if len(request_id) > max_request_id_length:
        raise PayloadValidationError(
            f"request_id exceeds maximum length {max_request_id_length}"
        )
    if isinstance(timestamp, bool) or not isinstance(timestamp, (int, float)):
        raise PayloadValidationError("timestamp must be a Unix timestamp number")
    if not math.isfinite(float(timestamp)):
        raise PayloadValidationError("timestamp must be finite")
    if not isinstance(source, str):
        raise PayloadValidationError("source must be a string")

    return VoiceTextRequest(
        text=text,
        request_id=request_id,
        timestamp=float(timestamp),
        source=source.strip()[:64],
    )


def validate_command_payload(
    payload: Any,
    max_text_length: int,
    max_intents: int = 3,
) -> VoiceCommandRequest:
    """Validate the versioned Windows candidate-intent HTTP contract."""
    if not isinstance(payload, Mapping):
        raise PayloadValidationError("JSON body must be an object")
    if payload.get("protocol_version") != 1:
        raise PayloadValidationError("protocol_version must be 1")

    common = validate_payload(
        {
            "text": payload.get("raw_text"),
            "request_id": payload.get("request_id"),
            "timestamp": payload.get("timestamp"),
            "source": payload.get("source", "windows_voice_agent"),
        },
        max_text_length=max_text_length,
    )
    intents = payload.get("intents")
    confidences = payload.get("confidences")
    if not isinstance(intents, list) or not 1 <= len(intents) <= max_intents:
        raise PayloadValidationError(f"intents must contain 1..{max_intents} items")
    if not all(isinstance(value, str) and value.strip() for value in intents):
        raise PayloadValidationError("every intent must be a non-empty string")
    if any(value.strip() not in ALLOWED_CANDIDATE_INTENTS for value in intents):
        raise PayloadValidationError("candidate contains an unsupported intent")
    if not isinstance(confidences, list) or len(confidences) != len(intents):
        raise PayloadValidationError("confidences must align with intents")
    if not all(
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
        and 0.0 <= float(value) <= 1.0
        for value in confidences
    ):
        raise PayloadValidationError("confidence values must be within [0, 1]")

    parameters = payload.get("parameters", {})
    if not isinstance(parameters, Mapping):
        raise PayloadValidationError("parameters must be an object")
    distance = parameters.get("distance", -1.0)
    angle = parameters.get("angle", -1.0)
    direction = parameters.get("direction", 0)
    if isinstance(distance, bool) or not isinstance(distance, (int, float)):
        raise PayloadValidationError("parameters.distance must be numeric")
    if isinstance(angle, bool) or not isinstance(angle, (int, float)):
        raise PayloadValidationError("parameters.angle must be numeric")
    if not math.isfinite(float(distance)) or not math.isfinite(float(angle)):
        raise PayloadValidationError("numeric parameters must be finite")
    if isinstance(direction, bool) or not isinstance(direction, int) or direction not in (-1, 0, 1):
        raise PayloadValidationError("parameters.direction must be -1, 0, or 1")
    unit = parameters.get("unit", "")
    speed = parameters.get("speed", "")
    target_desc = parameters.get("target_desc", "")
    distance_relative = parameters.get("distance_relative", False)
    follow = parameters.get("follow", False)
    if not isinstance(unit, str):
        raise PayloadValidationError("parameters.unit must be a string")
    if unit not in ("", "m", "cm", "degree"):
        raise PayloadValidationError("unsupported parameters.unit")
    if not isinstance(speed, str):
        raise PayloadValidationError("parameters.speed must be a string")
    if speed not in ("", "slow", "mid", "fast"):
        raise PayloadValidationError("unsupported parameters.speed")
    if not isinstance(target_desc, str):
        raise PayloadValidationError("parameters.target_desc must be a string")
    if not isinstance(distance_relative, bool):
        raise PayloadValidationError("parameters.distance_relative must be boolean")
    if not isinstance(follow, bool):
        raise PayloadValidationError("parameters.follow must be boolean")

    return VoiceCommandRequest(
        raw_text=common.text,
        intents=tuple(value.strip() for value in intents),
        confidences=tuple(float(value) for value in confidences),
        request_id=common.request_id,
        timestamp=common.timestamp,
        source=common.source,
        distance=float(distance),
        unit=unit,
        distance_relative=distance_relative,
        angle=float(angle),
        direction=direction,
        speed=speed,
        target_desc=target_desc[:128],
        follow=follow,
    )


class RequestDeduplicator:
    """Bounded TTL cache for request IDs."""

    def __init__(self, ttl_sec: float, max_entries: int) -> None:
        self._ttl_sec = ttl_sec
        self._max_entries = max_entries
        self._entries: "OrderedDict[str, float]" = OrderedDict()
        self._lock = threading.Lock()

    def check_and_record(self, request_id: str, now: float | None = None) -> bool:
        current = time.monotonic() if now is None else now
        with self._lock:
            cutoff = current - self._ttl_sec
            while self._entries:
                _, recorded_at = next(iter(self._entries.items()))
                if recorded_at >= cutoff:
                    break
                self._entries.popitem(last=False)

            if request_id in self._entries:
                return True

            self._entries[request_id] = current
            while len(self._entries) > self._max_entries:
                self._entries.popitem(last=False)
            return False

    def forget(self, request_id: str) -> None:
        with self._lock:
            self._entries.pop(request_id, None)
