import pytest

from voice_intent_pkg.voice_text_protocol import (
    PayloadValidationError,
    RequestDeduplicator,
    validate_command_payload,
    validate_payload,
    validate_timestamp,
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


def test_timestamp_accepts_current_request() -> None:
    validate_timestamp(
        99.0,
        now=100.0,
        max_request_age_sec=10.0,
        max_future_skew_sec=5.0,
    )


def test_timestamp_rejects_stale_and_future_request() -> None:
    with pytest.raises(PayloadValidationError, match="stale"):
        validate_timestamp(
            80.0,
            now=100.0,
            max_request_age_sec=10.0,
            max_future_skew_sec=5.0,
        )
    with pytest.raises(PayloadValidationError, match="future"):
        validate_timestamp(
            110.0,
            now=100.0,
            max_request_age_sec=10.0,
            max_future_skew_sec=5.0,
        )


def test_validates_structured_candidate_command() -> None:
    command = validate_command_payload(
        {
            "protocol_version": 1,
            "raw_text": "向前走十厘米",
            "intents": ["chassis_move_forward"],
            "confidences": [0.91],
            "parameters": {
                "distance": 10.0,
                "unit": "cm",
                "distance_relative": False,
                "angle": -1.0,
                "direction": 1,
            },
            "request_id": "request-1",
            "timestamp": 100.0,
            "source": "windows-test",
        },
        max_text_length=500,
    )
    assert command.intents == ("chassis_move_forward",)
    assert command.distance == 10.0
    assert command.unit == "cm"


def test_rejects_misaligned_candidate_confidence() -> None:
    with pytest.raises(PayloadValidationError):
        validate_command_payload(
            {
                "protocol_version": 1,
                "raw_text": "急停",
                "intents": ["emergency_stop"],
                "confidences": [],
                "request_id": "request-2",
                "timestamp": 100.0,
            },
            max_text_length=500,
        )


def test_rejects_string_boolean_parameter() -> None:
    with pytest.raises(PayloadValidationError):
        validate_command_payload(
            {
                "protocol_version": 1,
                "raw_text": "开始跟随",
                "intents": ["start_following"],
                "confidences": [0.9],
                "parameters": {"follow": "false"},
                "request_id": "request-bool",
                "timestamp": 100.0,
            },
            max_text_length=500,
        )


def test_rejects_non_finite_timestamp() -> None:
    with pytest.raises(PayloadValidationError):
        validate_command_payload(
            {
                "protocol_version": 1,
                "raw_text": "查询状态",
                "intents": ["status_query"],
                "confidences": [1.0],
                "request_id": "request-nan",
                "timestamp": float("nan"),
            },
            max_text_length=500,
        )
