import importlib.util
from pathlib import Path
import sys
import numpy as np


MODULE_PATH = Path(__file__).with_name("fcr_voice_agent.py")
SPEC = importlib.util.spec_from_file_location("fcr_voice_agent", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_deep_merge_preserves_nested_defaults():
    merged = MODULE._deep_merge(
        {"audio": {"rate": 16000, "device": None}},
        {"audio": {"device": 3}},
    )
    assert merged == {"audio": {"rate": 16000, "device": 3}}


def test_standby_phrase_is_deterministic_and_not_estop():
    assert MODULE.deterministic_safety_intent("进入待机状态。") == "stop_all"
    assert MODULE.deterministic_safety_intent("回到 待机状态") == "stop_all"


def test_only_explicit_emergency_phrase_bypasses_classifier():
    assert MODULE.deterministic_safety_intent("紧急停止！") == "emergency_stop"
    assert MODULE.deterministic_safety_intent("停止跟随") is None
    assert MODULE.deterministic_safety_intent("进入待机状态") != "emergency_stop"


def test_convert_candidate_does_not_invent_unspecified_distance():
    candidate = MODULE.convert_candidate(
        {
            "text": "向前一点",
            "bert_confidence": 0.91,
            "fine_confusion": 0.03,
        },
        {"action": "move_forward", "distance": 1, "unit": "m"},
    )
    assert candidate["intent"] == "chassis_move_forward"
    assert candidate["parameters"]["distance"] == -1.0


def test_follow_closer_without_quantity_uses_safe_relative_step():
    candidate = MODULE.convert_candidate(
        {
            "text": "跟近一点",
            "bert_confidence": 0.92,
            "fine_confusion": 0.02,
        },
        {
            "action": "set_follow_distance",
            "closer": True,
            "farther": False,
            "distance_delta": 1,
            "unit": "m",
        },
    )
    assert candidate["intent"] == "distance_adjust"
    assert candidate["parameters"]["distance"] == -0.2
    assert candidate["parameters"]["distance_relative"] is True
