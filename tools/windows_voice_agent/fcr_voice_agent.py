"""Long-running Windows microphone -> ASR -> intent gate -> Jetson bridge.

The Windows process is deliberately an edge input device.  It never talks to
ROS DDS and never emits actuator velocities.  Jetson remains the authority for
state validation, command arbitration, and execution.
"""

from __future__ import annotations

import argparse
import json
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import queue
import re
import signal
import sys
import tempfile
import threading
import time
import uuid
import wave
from dataclasses import dataclass
from typing import Any


LOG = logging.getLogger("fcr_voice_agent")
EMERGENCY_PHRASES = ("急停", "紧急停车", "紧急停止", "立即停车", "马上停车")
STATE_TO_CLASSIFIER = {
    "STANDBY": "待机",
    "FOLLOW": "跟随",
    "CINEMATIC": "运镜",
}


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    result = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = value
    return result


DEFAULT_CONFIG: dict[str, Any] = {
    "audio": {
        "device": None,
        "sample_rate": 16000,
        "block_ms": 30,
        "energy_threshold": 0.015,
        "speech_start_ms": 180,
        "silence_timeout_ms": 750,
        "pre_roll_ms": 300,
        "max_utterance_sec": 8.0,
        "min_utterance_sec": 0.35,
    },
    "asr": {
        "module_root": "D:/code/fcr_ros2/voice/server",
        "model": "Qwen/Qwen3-ASR-0.6B",
        "device": "cuda",
        "offline": True,
    },
    "classifier": {
        "enabled": True,
        "project_root": "D:/code/fcr_ros2/classfier/fcr_speech_interpreter",
        "bert_model": "xlx2673/classifier",
        "require_eight_labels": True,
    },
    "extractor": {
        "enabled": False,
        "model": "qwen2.5:1.5b",
        "ollama_url": "http://127.0.0.1:11434/api/generate",
    },
    "jetson": {
        "base_url": "http://192.168.50.35:8081",
        "auth_token_env": "FCR_VOICE_AUTH_TOKEN",
        "require_auth": True,
        "connect_timeout_sec": 1.0,
        "request_timeout_sec": 2.0,
        "retry_delays_sec": [0.2, 0.5, 1.0],
        "command_ttl_sec": 3.0,
        "state_cache_sec": 2.0,
    },
    "runtime": {
        "duplicate_window_sec": 1.5,
        "log_dir": "logs",
        "log_level": "INFO",
    },
}


def load_config(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        loaded = json.load(stream)
    config = _deep_merge(DEFAULT_CONFIG, loaded)
    env_url = os.environ.get("FCR_JETSON_VOICE_URL")
    if env_url:
        config["jetson"]["base_url"] = env_url
    return config


def setup_logging(config: dict[str, Any], root: Path) -> None:
    runtime = config["runtime"]
    log_dir = Path(runtime["log_dir"])
    if not log_dir.is_absolute():
        log_dir = root / log_dir
    log_dir.mkdir(parents=True, exist_ok=True)
    formatter = logging.Formatter(
        "%(asctime)s %(levelname)s %(name)s %(message)s"
    )
    file_handler = RotatingFileHandler(
        log_dir / "voice_agent.log",
        maxBytes=20 * 1024 * 1024,
        backupCount=10,
        encoding="utf-8",
    )
    file_handler.setFormatter(formatter)
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    logging.basicConfig(
        level=getattr(logging, str(runtime["log_level"]).upper(), logging.INFO),
        handlers=[file_handler, console_handler],
    )


class JetsonClient:
    def __init__(self, config: dict[str, Any]) -> None:
        import requests

        self._requests = requests
        self._config = config
        self._base_url = str(config["base_url"]).rstrip("/")
        token = os.environ.get(str(config["auth_token_env"]), "")
        if bool(config.get("require_auth", True)) and not token:
            raise RuntimeError(
                f"required environment variable {config['auth_token_env']} is empty"
            )
        self._headers = {"Content-Type": "application/json"}
        if token:
            self._headers["Authorization"] = f"Bearer {token}"
        self._session = requests.Session()
        self._state: str | None = None
        self._state_at = 0.0

    @property
    def timeout(self) -> tuple[float, float]:
        return (
            float(self._config["connect_timeout_sec"]),
            float(self._config["request_timeout_sec"]),
        )

    def health(self) -> bool:
        try:
            response = self._session.get(
                f"{self._base_url}/health", timeout=self.timeout
            )
            return response.ok
        except self._requests.RequestException:
            return False

    def classifier_state(self) -> str | None:
        now = time.monotonic()
        if (
            self._state is not None
            and now - self._state_at <= float(self._config["state_cache_sec"])
        ):
            return STATE_TO_CLASSIFIER.get(self._state)
        try:
            response = self._session.get(
                f"{self._base_url}/voice/state",
                headers=self._headers,
                timeout=self.timeout,
            )
            response.raise_for_status()
            payload = response.json()
            if not payload.get("available", False):
                raise RuntimeError("Jetson system state is not available")
            self._state = str(payload.get("mode_name", "")).upper()
            if self._state not in STATE_TO_CLASSIFIER:
                raise RuntimeError(f"unsupported Jetson mode: {self._state}")
            self._state_at = now
        except (self._requests.RequestException, ValueError, RuntimeError) as exc:
            LOG.warning("Jetson state unavailable; ordinary commands are blocked: %s", exc)
            self._state = None
            self._state_at = 0.0
            return None
        return STATE_TO_CLASSIFIER.get(self._state)

    def send_command(
        self, text: str, intent: str, confidence: float, parameters: dict[str, Any]
    ) -> dict[str, Any]:
        request_id = str(uuid.uuid4())
        created_at = time.time()
        payload = {
            "protocol_version": 1,
            "raw_text": text,
            "intents": [intent],
            "confidences": [confidence],
            "parameters": parameters,
            "request_id": request_id,
            "timestamp": created_at,
            "source": "windows_fcr_voice_agent",
        }
        delays = [0.0] + [float(x) for x in self._config["retry_delays_sec"]]
        last_error: Exception | None = None
        for delay in delays:
            if delay:
                time.sleep(delay)
            if time.time() - created_at > float(self._config["command_ttl_sec"]):
                raise TimeoutError("voice command expired before delivery")
            try:
                response = self._session.post(
                    f"{self._base_url}/voice/command",
                    json=payload,
                    headers=self._headers,
                    timeout=self.timeout,
                )
                if response.status_code >= 500:
                    last_error = RuntimeError(f"Jetson HTTP {response.status_code}")
                    continue
                response.raise_for_status()
                result = response.json()
                result.setdefault("request_id", request_id)
                if intent in (
                    "start_following", "stop_following",
                    "enter_cinematic", "exit_cinematic", "emergency_stop",
                ):
                    self._state_at = 0.0
                return result
            except (self._requests.ConnectionError, self._requests.Timeout) as exc:
                last_error = exc
                continue
        raise RuntimeError(f"failed to deliver command: {last_error}")


class IntentGate:
    def __init__(
        self, config: dict[str, Any], extractor_config: dict[str, Any]
    ) -> None:
        self._enabled = bool(config["enabled"])
        self._splitter: Any = None
        if not self._enabled:
            LOG.warning("Windows intent prefilter is disabled")
            return
        project_root = Path(config["project_root"]).resolve()
        if not project_root.is_dir():
            raise FileNotFoundError(f"classifier project not found: {project_root}")
        sys.path.insert(0, str(project_root))
        from classifier import DoubleLayerClassifier
        from extractor import Extractor
        from splitter import Splitter

        classifier = DoubleLayerClassifier(bert_dir=str(config["bert_model"]))
        classifier.load_all()
        num_labels = int(classifier.bert_model.config.num_labels)
        if bool(config["require_eight_labels"]) and num_labels != 8:
            raise RuntimeError(
                f"incompatible BERT model: expected 8 labels, got {num_labels}"
            )
        self._splitter = Splitter(classifier)
        self._extractor = None
        if bool(extractor_config["enabled"]):
            self._extractor = Extractor(
                model_name=str(extractor_config["model"]),
                ollama_url=str(extractor_config["ollama_url"]),
            )
            self._extractor.load_schemas()
        LOG.info("Windows intent gate ready with %d-label BERT", num_labels)

    def candidates(
        self, text: str, state: str | None
    ) -> tuple[list[dict[str, Any]], str]:
        if any(phrase in text for phrase in EMERGENCY_PHRASES):
            return [{
                "intent": "emergency_stop",
                "confidence": 1.0,
                "parameters": {},
            }], "emergency_phrase_bypass"
        if state is None:
            return [], "jetson_state_unavailable"
        if not self._enabled:
            return [], "prefilter_disabled_has_no_structured_candidate"
        result = self._splitter.split(text, current_state=state)
        if result.rejected or not result.filtered:
            return [], result.reject_reason or "all_intents_filtered"
        candidates = []
        for item in result.filtered:
            extracted = (
                self._extractor.extract(
                    item["text"], item["coarse_id"], item["fine_id"]
                )
                if self._extractor is not None
                else {}
            )
            converted = convert_candidate(item, extracted)
            if converted is not None:
                candidates.append(converted)
        summary = ",".join(
            f"{item['coarse_name']}/{item['fine_name']}" for item in result.filtered
        )
        return candidates, summary


ACTION_TO_INTENT = {
    "move_left": "chassis_move_left",
    "move_right": "chassis_move_right",
    "move_forward": "chassis_move_forward",
    "move_backward": "chassis_move_backward",
    "turn_left": "chassis_turn_left",
    "turn_right": "chassis_turn_right",
    "look_left": "gimbal_nudge_left",
    "look_right": "gimbal_nudge_right",
    "look_up": "gimbal_nudge_up",
    "look_down": "gimbal_nudge_down",
    "gimbal_home": "gimbal_home",
    "enter_follow": "start_following",
    "set_follow_distance": "distance_adjust",
    "enter_camera": "enter_cinematic",
    "static_track": "start_static_track",
    "orbit_left": "start_orbit",
    "orbit_right": "start_orbit",
    "pan_left": "start_truck",
    "pan_right": "start_truck",
    "push_forward": "start_dolly",
    "pull_back": "start_dolly",
    "start_recording": "camera_start_recording",
    "take_photo": "camera_take_photo",
    "stop_move": "chassis_stop",
    "stop_vision": "gimbal_stop",
    "exit_follow": "stop_following",
    "exit_camera": "exit_cinematic",
    "stop_recording": "camera_stop_recording",
    "emergency_stop": "emergency_stop",
    "query_chassis": "query_chassis_status",
    "query_gimbal": "query_gimbal_status",
    "query_follow": "status_query",
    "query_camera": "query_camera_motion_status",
    "query_recording": "query_camera_status",
    "query_basic": "status_query",
}

CLASS_TO_ACTION = {
    (0, 0): "move_left", (0, 1): "move_right", (0, 2): "move_forward",
    (0, 3): "move_backward", (0, 4): "turn_left", (0, 5): "turn_right",
    (0, 6): "adjust_speed",
    (1, 0): "look_left", (1, 1): "look_right", (1, 2): "look_up",
    (1, 3): "look_down", (1, 4): "gimbal_home",
    (1, 5): "adjust_angular_speed",
    (2, 0): "enter_follow", (2, 1): "set_follow_distance",
    (3, 0): "enter_camera", (3, 1): "static_track", (3, 2): "orbit_left",
    (3, 3): "orbit_right", (3, 4): "pan_left", (3, 5): "pan_right",
    (3, 6): "push_forward", (3, 7): "pull_back",
    (4, 0): "start_recording", (4, 1): "take_photo", (4, 2): "set_target",
    (5, 0): "stop_move", (5, 1): "stop_vision", (5, 2): "exit_follow",
    (5, 3): "exit_camera", (5, 4): "stop_recording",
    (5, 5): "emergency_stop",
    (6, 0): "query_chassis", (6, 1): "query_gimbal", (6, 2): "query_follow",
    (6, 3): "query_camera", (6, 4): "query_recording", (6, 5): "query_basic",
}

QUANTITY_RE = re.compile(
    r"(?:\d+(?:\.\d+)?|[零〇一二两三四五六七八九十百半]+)\s*"
    r"(?:厘米|公分|米|度|cm|m)"
)


def chinese_number(value: str) -> float | None:
    if not value:
        return None
    if value == "半":
        return 0.5
    try:
        return float(value)
    except ValueError:
        pass
    digits = {"零": 0, "〇": 0, "一": 1, "二": 2, "两": 2, "三": 3,
              "四": 4, "五": 5, "六": 6, "七": 7, "八": 8, "九": 9}
    total = 0
    current = 0
    for char in value:
        if char in digits:
            current = digits[char]
        elif char == "十":
            total += (current or 1) * 10
            current = 0
        elif char == "百":
            total += (current or 1) * 100
            current = 0
        else:
            return None
    return float(total + current)


def explicit_quantity(text: str) -> tuple[float, str] | None:
    match = QUANTITY_RE.search(text)
    if match is None:
        return None
    raw = match.group(0).strip()
    number_match = re.match(r"\d+(?:\.\d+)?|[零〇一二两三四五六七八九十百半]+", raw)
    if number_match is None:
        return None
    value = chinese_number(number_match.group(0))
    if value is None:
        return None
    if "厘米" in raw or "公分" in raw or raw.endswith("cm"):
        return value, "cm"
    if "度" in raw:
        return value, "degree"
    return value, "m"


def convert_candidate(
    classified: dict[str, Any], extracted: dict[str, Any]
) -> dict[str, Any] | None:
    # The trained classifier owns the action class.  An optional LLM may enrich
    # parameters, but must never replace a deterministic classifier decision.
    action = CLASS_TO_ACTION.get(
        (int(classified.get("coarse_id", -1)), int(classified.get("fine_id", -1))),
        "",
    )
    if action == "adjust_speed":
        source_text = str(classified.get("text", ""))
        up = bool(extracted.get("speed_up")) or any(word in source_text for word in ("快", "加速", "提高"))
        down = bool(extracted.get("speed_down")) or any(word in source_text for word in ("慢", "减速", "降低"))
        if up == down:
            return None
        intent = "chassis_speed_up" if up else "chassis_speed_down"
    elif action == "adjust_angular_speed":
        source_text = str(classified.get("text", ""))
        up = bool(extracted.get("speed_up")) or any(word in source_text for word in ("快", "加速", "提高"))
        down = bool(extracted.get("speed_down")) or any(word in source_text for word in ("慢", "减速", "降低"))
        if up == down:
            return None
        intent = "gimbal_speed_up" if up else "gimbal_speed_down"
    else:
        intent = ACTION_TO_INTENT.get(action)
    if intent is None:
        LOG.warning("unsupported extracted action rejected: %s", action)
        return None

    confidence = min(
        float(classified.get("bert_confidence", 0.0)),
        max(0.0, 1.0 - float(classified.get("fine_confusion", 1.0))),
    )
    parameters: dict[str, Any] = {
        "distance": -1.0,
        "unit": "",
        "distance_relative": False,
        "angle": -1.0,
        "direction": 0,
        "speed": "",
        "target_desc": "",
        "follow": intent == "start_following",
    }
    text = str(classified.get("text", ""))
    parsed_quantity = explicit_quantity(text)
    has_explicit_quantity = parsed_quantity is not None
    if parsed_quantity is not None:
        value, unit = parsed_quantity
        if action in ("turn_left", "turn_right", "look_left", "look_right", "look_up", "look_down", "orbit_left", "orbit_right"):
            parameters["angle"] = value
        else:
            parameters["distance"] = value
        parameters["unit"] = unit
    if action in ("orbit_left", "pan_left", "push_forward"):
        parameters["direction"] = 1
    elif action in ("orbit_right", "pan_right", "pull_back"):
        parameters["direction"] = -1
    if action == "set_follow_distance":
        absolute_words = ("设置为", "设为", "保持", "跟随距离")
        if has_explicit_quantity and any(word in text for word in absolute_words):
            parameters["distance"] = parsed_quantity[0]
        else:
            closer = bool(extracted.get("closer")) or any(
                token in text for token in ("近", "靠近", "走近")
            )
            farther = bool(extracted.get("farther")) or any(
                token in text for token in ("远", "远离", "退后")
            )
            if closer == farther:
                LOG.warning("follow-distance direction is ambiguous: %s", text)
                return None
            amount = parsed_quantity[0] if has_explicit_quantity else 0.2
            parameters["distance"] = (-1.0 if closer else 1.0) * amount
            parameters["distance_relative"] = True
        parameters["unit"] = parsed_quantity[1] if parsed_quantity else "m"
    if action in (
        "move_left", "move_right", "move_forward", "move_backward",
        "turn_left", "turn_right", "look_left", "look_right",
        "look_up", "look_down",
    ) and not has_explicit_quantity:
        parameters["distance"] = -1.0
        parameters["angle"] = -1.0
        parameters["unit"] = ""
    return {"intent": intent, "confidence": confidence, "parameters": parameters}


class SpeechRecognizer:
    def __init__(self, config: dict[str, Any]) -> None:
        if bool(config["offline"]):
            os.environ.setdefault("HF_HUB_OFFLINE", "1")
            os.environ.setdefault("TRANSFORMERS_OFFLINE", "1")
        module_root = Path(str(config["module_root"])).resolve()
        if not module_root.is_dir():
            raise FileNotFoundError(f"ASR module project not found: {module_root}")
        sys.path.insert(0, str(module_root))
        from listener import Listener

        self._listener = Listener(
            model_name=str(config["model"]), device=str(config["device"])
        )
        self._listener.load_model()

    def transcribe(self, wav_path: Path) -> str:
        return str(self._listener.transcribe(str(wav_path))).strip()


@dataclass
class AudioSegment:
    samples: Any
    sample_rate: int


class VadRecorder:
    """Bounded energy VAD with pre-roll; produces complete utterances."""

    def __init__(self, config: dict[str, Any]) -> None:
        import numpy as np
        import sounddevice as sd

        self._np = np
        self._sd = sd
        self._config = config
        self._sample_rate = int(config["sample_rate"])
        self._block_size = int(self._sample_rate * int(config["block_ms"]) / 1000)
        self._blocks: "queue.Queue[Any]" = queue.Queue(maxsize=64)
        self._segments: "queue.Queue[AudioSegment]" = queue.Queue(maxsize=2)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._segment_loop, daemon=True)

    def _audio_callback(self, indata, frames, time_info, status) -> None:
        if status:
            LOG.warning("audio callback status: %s", status)
        block = self._np.asarray(indata[:, 0], dtype=self._np.float32).copy()
        try:
            self._blocks.put_nowait(block)
        except queue.Full:
            try:
                self._blocks.get_nowait()
                self._blocks.put_nowait(block)
            except queue.Empty:
                pass

    def start(self):
        self._thread.start()
        stream = self._sd.InputStream(
            samplerate=self._sample_rate,
            blocksize=self._block_size,
            channels=1,
            dtype="float32",
            device=self._config["device"],
            callback=self._audio_callback,
        )
        stream.start()
        return stream

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)

    def get(self, timeout: float = 0.5) -> AudioSegment | None:
        try:
            return self._segments.get(timeout=timeout)
        except queue.Empty:
            return None

    def _segment_loop(self) -> None:
        block_ms = int(self._config["block_ms"])
        pre_blocks = max(1, int(self._config["pre_roll_ms"]) // block_ms)
        start_blocks = max(1, int(self._config["speech_start_ms"]) // block_ms)
        silence_blocks = max(1, int(self._config["silence_timeout_ms"]) // block_ms)
        max_blocks = max(1, int(float(self._config["max_utterance_sec"]) * 1000) // block_ms)
        min_samples = int(float(self._config["min_utterance_sec"]) * self._sample_rate)
        threshold = float(self._config["energy_threshold"])
        pre_roll: list[Any] = []
        utterance: list[Any] = []
        voiced_run = 0
        silent_run = 0
        recording = False

        while not self._stop.is_set():
            try:
                block = self._blocks.get(timeout=0.2)
            except queue.Empty:
                continue
            rms = float(self._np.sqrt(self._np.mean(self._np.square(block)) + 1e-12))
            voiced = rms >= threshold
            if not recording:
                pre_roll.append(block)
                pre_roll = pre_roll[-pre_blocks:]
                voiced_run = voiced_run + 1 if voiced else 0
                if voiced_run >= start_blocks:
                    recording = True
                    utterance = list(pre_roll)
                    silent_run = 0
            else:
                utterance.append(block)
                silent_run = 0 if voiced else silent_run + 1
                complete = silent_run >= silence_blocks or len(utterance) >= max_blocks
                if complete:
                    samples = self._np.concatenate(utterance)
                    if samples.size >= min_samples:
                        segment = AudioSegment(samples=samples, sample_rate=self._sample_rate)
                        try:
                            self._segments.put_nowait(segment)
                        except queue.Full:
                            LOG.warning("dropping stale utterance because ASR queue is full")
                    pre_roll = []
                    utterance = []
                    voiced_run = 0
                    silent_run = 0
                    recording = False


def write_wav(segment: AudioSegment, path: Path) -> None:
    import numpy as np

    pcm = np.clip(segment.samples, -1.0, 1.0)
    pcm = (pcm * 32767.0).astype("<i2")
    with wave.open(str(path), "wb") as output:
        output.setnchannels(1)
        output.setsampwidth(2)
        output.setframerate(segment.sample_rate)
        output.writeframes(pcm.tobytes())


class VoiceAgent:
    def __init__(self, config: dict[str, Any]) -> None:
        # Apply offline mode before either the classifier or ASR imports
        # transformers/huggingface_hub.  Both model stacks must obey it.
        if bool(config["asr"]["offline"]):
            os.environ.setdefault("HF_HUB_OFFLINE", "1")
            os.environ.setdefault("TRANSFORMERS_OFFLINE", "1")
        self._client = JetsonClient(config["jetson"])
        self._gate = IntentGate(config["classifier"], config["extractor"])
        self._recognizer = SpeechRecognizer(config["asr"])
        self._recorder = VadRecorder(config["audio"])
        self._duplicate_window = float(config["runtime"]["duplicate_window_sec"])
        self._last_text = ""
        self._last_text_at = 0.0
        self._stop = threading.Event()

    def stop(self, *_args: object) -> None:
        self._stop.set()

    def run(self) -> None:
        signal.signal(signal.SIGINT, self.stop)
        signal.signal(signal.SIGTERM, self.stop)
        stream = self._recorder.start()
        LOG.info("continuous microphone VAD started: %s", stream.device)
        LOG.info("Jetson bridge health: %s", "ok" if self._client.health() else "offline")
        try:
            while not self._stop.is_set():
                segment = self._recorder.get()
                if segment is None:
                    continue
                self._process(segment)
        finally:
            stream.stop()
            stream.close()
            self._recorder.stop()

    def _process(self, segment: AudioSegment) -> None:
        started = time.monotonic()
        fd, name = tempfile.mkstemp(suffix=".wav")
        os.close(fd)
        wav_path = Path(name)
        try:
            write_wav(segment, wav_path)
            text = self._recognizer.transcribe(wav_path)
        except Exception:
            LOG.exception("ASR failed")
            return
        finally:
            wav_path.unlink(missing_ok=True)
        if not text:
            return
        now = time.monotonic()
        if text == self._last_text and now - self._last_text_at < self._duplicate_window:
            LOG.info("local duplicate suppressed: %s", text)
            return
        self._last_text, self._last_text_at = text, now
        state = self._client.classifier_state()
        try:
            candidates, reason = self._gate.candidates(text, state)
        except Exception:
            LOG.exception("intent classification failed; command not sent")
            return
        if not candidates:
            LOG.info("intent rejected locally | state=%s text=%s reason=%s", state, text, reason)
            return
        try:
            for candidate in candidates:
                result = self._client.send_command(
                    text,
                    candidate["intent"],
                    candidate["confidence"],
                    candidate["parameters"],
                )
                LOG.info(
                    "candidate delivered | state=%s intent=%s confidence=%.3f accepted=%s duplicate=%s",
                    state, candidate["intent"], candidate["confidence"],
                    result.get("accepted"), result.get("duplicate", False),
                )
            LOG.info("voice pipeline latency_ms=%.1f", (time.monotonic() - started) * 1000.0)
        except Exception:
            LOG.exception("Jetson delivery failed; stale command discarded")


def list_devices() -> None:
    import sounddevice as sd

    for index, device in enumerate(sd.query_devices()):
        if device["max_input_channels"] > 0:
            print(f"{index}: {device['name']}")


def main() -> int:
    parser = argparse.ArgumentParser(description="FCR Windows voice edge agent")
    parser.add_argument("--config", default="config.json")
    parser.add_argument("--list-devices", action="store_true")
    parser.add_argument("--check-jetson", action="store_true")
    args = parser.parse_args()
    if args.list_devices:
        list_devices()
        return 0
    config_path = Path(args.config).resolve()
    config = load_config(config_path)
    setup_logging(config, config_path.parent)
    if args.check_jetson:
        client = JetsonClient(config["jetson"])
        print(json.dumps({
            "health": client.health(),
            "classifier_state": client.classifier_state(),
        }, ensure_ascii=False, indent=2))
        return 0
    LOG.info("loading voice agent configuration: %s", config_path)
    try:
        VoiceAgent(config).run()
    except Exception:
        LOG.exception("voice agent terminated")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
