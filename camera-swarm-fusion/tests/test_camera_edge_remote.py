"""CAMERA_PROCESSING=remote: config, wire format and the frame forwarder.

The full Pi -> camera-processor round trip (and remote rows == local rows) is
scripts/remote_processing_smoke.py, which needs a broker and camera-processor.
"""
import json
import struct
from types import SimpleNamespace

import numpy as np
import pytest

from camera_edge.config import EdgeAgentConfig
from camera_edge.frame_forwarder import FrameForwarder, build_config_message, encode_frame


def test_processing_defaults_to_local(monkeypatch):
    for name in ("CAMERA_PROCESSING", "CAMERA_FRAME_BROKER_IP", "CAMERA_FRAME_BROKER_PORT"):
        monkeypatch.delenv(name, raising=False)
    monkeypatch.setenv("MQTT_BROKER_IP", "10.0.0.1")
    cfg = EdgeAgentConfig.from_env()
    assert cfg.processing == "local" and not cfg.remote_processing
    assert cfg.frame_broker_ip == "10.0.0.1"  # blank -> the row broker
    assert cfg.frame_topic_prefix == "camera_frames"


def test_remote_settings_from_env(monkeypatch):
    monkeypatch.setenv("CAMERA_PROCESSING", " Remote ")
    monkeypatch.setenv("CAMERA_FRAME_BROKER_IP", "10.0.0.9")
    monkeypatch.setenv("CAMERA_FRAME_BROKER_PORT", "1884")
    cfg = EdgeAgentConfig.from_env()
    assert cfg.remote_processing
    assert (cfg.frame_broker_ip, cfg.frame_broker_port) == ("10.0.0.9", 1884)


def test_bad_processing_value_fails_fast(monkeypatch):
    monkeypatch.setenv("CAMERA_PROCESSING", "cloud")
    with pytest.raises(ValueError):
        EdgeAgentConfig.from_env()


def test_frame_encoding():
    payload = encode_frame({"run_id": "r", "frame_idx": 3}, b"\xff\xd8jpeg")
    (n,) = struct.unpack_from(">I", payload, 0)
    header = json.loads(payload[4:4 + n])
    assert header == {"v": 1, "run_id": "r", "frame_idx": 3}
    assert payload[4 + n:] == b"\xff\xd8jpeg"


def _config(**kw):
    values = dict(device_id="113", camera_name="front", client_type="CAMERA", aruco_dict="4x4_50",
                  marker_length_m=0.0485, marker_lengths_m={1: 0.1}, target_ids=[1, 2],
                  calibration_path="calib/x.yml", fps=15, width=1920, height=1080, plc_trigger=True)
    values.update(kw)
    return SimpleNamespace(**values)


def test_config_message_is_json_ready():
    K = np.array([[1000.0, 0, 960], [0, 1000.0, 540], [0, 0, 1]])
    msg = build_config_message(_config(), K, np.zeros((1, 5)), (1920, 1080))
    json.dumps(msg)
    assert msg["camera_matrix"][0] == [1000.0, 0.0, 960.0]
    assert msg["dist_coeffs"] == [0.0] * 5
    assert msg["marker_lengths_m"] == {"1": 0.1} and msg["target_ids"] == [1, 2]


class _FakeClient:
    def __init__(self, _client_id):
        self.published = []
        self.on_connect = self.on_disconnect = None

    def reconnect_delay_set(self, **_kw): pass
    def max_queued_messages_set(self, _n): pass
    def connect_async(self, *_a, **_kw): pass
    def loop_start(self): pass
    def loop_stop(self): pass
    def disconnect(self): self.on_disconnect(self, None, 0)

    def publish(self, topic, payload, qos=0, retain=False):
        self.published.append((topic, payload, qos, retain))
        return SimpleNamespace(rc=0)


def _forwarder():
    msg = {"device_id": "113", "camera_name": "front"}
    fwd = FrameForwarder("broker", 1883, "camera_frames", msg, client_factory=_FakeClient)
    fwd.open()
    return fwd, fwd._client


def test_forwarder_sends_config_on_connect_and_frames_after():
    fwd, client = _forwarder()
    sample = {"jpeg": b"jpg", "frame_idx": 0, "capture_time_ms": 1000}
    assert fwd.send(sample, "run") is False and fwd.dropped == 1  # not connected yet

    client.on_connect(client, None, {}, 0)
    topic, payload, qos, retain = client.published[0]
    assert (topic, retain, qos) == ("camera_frames/113_front/config", True, 1)

    assert fwd.send(sample, "run") is True
    topic, payload, _qos, retain = client.published[1]
    assert topic == "camera_frames/113_front/frame" and not retain
    (n,) = struct.unpack_from(">I", payload, 0)
    assert json.loads(payload[4:4 + n])["run_id"] == "run"
    assert payload[4 + n:] == b"jpg"

    client.on_disconnect(client, None, 0)
    assert fwd.send(sample, "run") is False and fwd.dropped == 2
    fwd.close()


def test_forwarder_refuses_the_ingested_prefix():
    with pytest.raises(ValueError):
        FrameForwarder("b", 1883, "camera", {"device_id": "1", "camera_name": "x"})
