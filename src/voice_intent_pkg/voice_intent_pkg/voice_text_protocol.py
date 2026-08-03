"""Validation and idempotency helpers for the voice text HTTP bridge."""

import time
import threading
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
    if not isinstance(source, str):
        raise PayloadValidationError("source must be a string")

    return VoiceTextRequest(
        text=text,
        request_id=request_id,
        timestamp=float(timestamp),
        source=source.strip()[:64],
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
