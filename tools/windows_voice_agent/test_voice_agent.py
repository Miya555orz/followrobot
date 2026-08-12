import importlib.util
from pathlib import Path
import sys
import tempfile
import wave

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


def test_write_wav_is_mono_pcm16():
    segment = MODULE.AudioSegment(
        samples=np.array([-1.0, 0.0, 1.0], dtype=np.float32),
        sample_rate=16000,
    )
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "sample.wav"
        MODULE.write_wav(segment, path)
        with wave.open(str(path), "rb") as stream:
            assert stream.getnchannels() == 1
            assert stream.getsampwidth() == 2
            assert stream.getframerate() == 16000
            assert stream.getnframes() == 3


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
