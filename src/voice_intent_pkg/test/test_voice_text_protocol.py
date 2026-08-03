import pytest

from voice_intent_pkg.voice_text_protocol import (
    PayloadValidationError,
    RequestDeduplicator,
    validate_payload,
)


def test_validate_payload_normalizes_text() -> None:
    request = validate_payload(
        {
            "text": "  云台向右一点  ",
            "request_id": "request-1",
            "timestamp": 123.5,
            "source": "windows_qwen3_asr",
        },
        max_text_length=100,
    )

    assert request.text == "云台向右一点"
    assert request.request_id == "request-1"
    assert request.timestamp == 123.5
    assert request.source == "windows_qwen3_asr"


@pytest.mark.parametrize(
    "payload",
    [
        {},
        {"text": "", "request_id": "id", "timestamp": 1},
        {"text": "向右", "request_id": "", "timestamp": 1},
        {"text": "向右", "request_id": "id", "timestamp": "now"},
    ],
)
def test_validate_payload_rejects_invalid_input(payload: dict) -> None:
    with pytest.raises(PayloadValidationError):
        validate_payload(payload, max_text_length=100)


def test_validate_payload_enforces_text_limit() -> None:
    with pytest.raises(PayloadValidationError):
        validate_payload(
            {"text": "12345", "request_id": "id", "timestamp": 1},
            max_text_length=4,
        )


def test_deduplicator_expires_and_can_forget() -> None:
    deduplicator = RequestDeduplicator(ttl_sec=10.0, max_entries=2)

    assert not deduplicator.check_and_record("a", now=0.0)
    assert deduplicator.check_and_record("a", now=1.0)
    deduplicator.forget("a")
    assert not deduplicator.check_and_record("a", now=2.0)
    assert not deduplicator.check_and_record("b", now=3.0)
    assert not deduplicator.check_and_record("c", now=20.0)
    assert not deduplicator.check_and_record("a", now=20.0)
