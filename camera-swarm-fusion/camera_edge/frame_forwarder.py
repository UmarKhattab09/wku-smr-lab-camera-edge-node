"""Forward camera frames to the off-Pi camera-processor (CAMERA_PROCESSING=remote).

With remote processing the Pi only captures: camera_host grabs and JPEG-encodes
each frame, and camera_edge forwards those JPEG bytes unchanged (no decode, no
detection) to camera-processor, which runs on the AMS (wku-smr-lab-middleware,
camera_processor/). camera-processor does the ArUco detection + PnP pose this
container used to do and publishes the same rows to camera/<device>.

    CAM_PI (camera_host + camera_edge)  --camera_frames/<device>/frame-->
    camera-processor                    --camera/<device>-->  AMS (fastapi-app)

Wire format -- keep in step with camera_processor/protocol.py in the
middleware repo and bump VERSION on both sides together:

    camera_frames/<device>/config   retained JSON, re-sent on every (re)connect:
                                    calibration (camera_matrix, dist_coeffs) and
                                    the detection settings from this container's env
    camera_frames/<device>/frame    4-byte big-endian header length, JSON header
                                    {v, device_id, camera_name, run_id, frame_idx,
                                    capture_time_ms, sent_at_ms}, then the JPEG

<device> is <DEVICE_ID>_<CAMERA_NAME>, e.g. 113_front.
"""
from __future__ import annotations

import json
import logging
import struct
import threading
import time

import numpy as np

VERSION = 1

logger = logging.getLogger("camera_edge.forwarder")

_LEN = struct.Struct(">I")


def device_label(device_id, camera_name) -> str:
    return f"{device_id}_{camera_name}"


def encode_frame(header: dict, jpeg: bytes) -> bytes:
    raw = json.dumps({"v": VERSION, **header}, separators=(",", ":")).encode("utf-8")
    return _LEN.pack(len(raw)) + raw + bytes(jpeg)


def _matrix(value) -> list:
    return np.asarray(value, dtype=float).tolist()


def build_config_message(config, K, dist, image_size) -> dict:
    """Everything camera-processor needs to process this camera's frames."""
    lengths = config.marker_lengths_m or {}
    return {
        "v": VERSION,
        "device_id": str(config.device_id),
        "camera_name": str(config.camera_name),
        "client_type": str(config.client_type),
        "aruco_dict": config.aruco_dict,
        "marker_length_m": float(config.marker_length_m),
        "marker_lengths_m": {str(k): float(v) for k, v in lengths.items()} or None,
        "target_ids": list(config.target_ids) if config.target_ids else None,
        "camera_matrix": _matrix(K),
        "dist_coeffs": np.asarray(dist, dtype=float).reshape(-1).tolist(),
        "image_size": list(image_size) if image_size else None,
        "calibration_path": config.calibration_path,
        "fps": config.fps,
        "width": config.width,
        "height": config.height,
        "plc_trigger": config.plc_trigger,
    }


def _make_client(client_id: str):
    import paho.mqtt.client as mqtt

    if hasattr(mqtt, "CallbackAPIVersion"):  # paho-mqtt >= 2
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
    return mqtt.Client(client_id=client_id)


class FrameForwarder:
    """Publishes this camera's frames (and its retained config) over MQTT.

    Never blocks the capture loop: frames are QoS 0 and published only while
    connected; while the broker is unreachable they are dropped and counted
    (paho reconnects in the background).
    """

    def __init__(self, broker_ip: str, broker_port: int, topic_prefix: str,
                 config_message: dict, qos: int = 0, client_factory=None):
        self.broker_ip = broker_ip
        self.broker_port = broker_port
        self.prefix = topic_prefix.strip().strip("/")
        if not self.prefix or self.prefix.lower() == "camera":
            raise ValueError("CAMERA_FRAME_TOPIC_PREFIX must not be empty or 'camera'")
        self.config_message = config_message
        self.label = device_label(config_message["device_id"], config_message["camera_name"])
        self.frame_topic = f"{self.prefix}/{self.label}/frame"
        self.config_topic = f"{self.prefix}/{self.label}/config"
        self.qos = qos
        self._client_factory = client_factory or _make_client
        self._client = None
        self._connected = threading.Event()
        self._closing = False
        self.sent = 0
        self.dropped = 0
        self.last_error: str | None = None

    @property
    def connected(self) -> bool:
        return self._connected.is_set()

    def open(self) -> None:
        client = self._client_factory(f"camera-edge-{self.label}")
        client.on_connect = self._on_connect
        client.on_disconnect = self._on_disconnect
        client.reconnect_delay_set(min_delay=1, max_delay=10)
        # Bounded so a slow link can't buffer frames in RAM on the Pi.
        client.max_queued_messages_set(32)
        client.connect_async(self.broker_ip, self.broker_port, keepalive=30)
        client.loop_start()
        self._client = client
        logger.info("forwarding frames to %s:%s on %s (processing on camera-processor)",
                    self.broker_ip, self.broker_port, self.frame_topic)

    def _on_connect(self, client, _userdata, _flags, rc, _properties=None):
        ok = (not rc.is_failure) if hasattr(rc, "is_failure") else rc == 0
        if not ok:
            self.last_error = f"broker refused connection: {rc}"
            logger.warning("frame broker %s:%s refused the connection: %s", self.broker_ip, self.broker_port, rc)
            return
        # Retained, so camera-processor gets it whenever it (re)subscribes.
        client.publish(self.config_topic, json.dumps(self.config_message), qos=1, retain=True)
        self._connected.set()
        self.last_error = None
        logger.info("connected to frame broker %s:%s; config sent on %s",
                    self.broker_ip, self.broker_port, self.config_topic)

    def _on_disconnect(self, _client, _userdata, *args):
        self._connected.clear()
        if self._closing:
            return
        self.last_error = "disconnected from frame broker"
        logger.warning("lost frame broker %s:%s; reconnecting", self.broker_ip, self.broker_port)

    def wait_connected(self, timeout: float) -> bool:
        return self._connected.wait(timeout)

    def send(self, sample: dict, run_id: str) -> bool:
        if self._client is None or not self.connected:
            self.dropped += 1
            return False
        header = {
            "device_id": self.config_message["device_id"],
            "camera_name": self.config_message["camera_name"],
            "run_id": run_id,
            "frame_idx": int(sample["frame_idx"]),
            "capture_time_ms": int(sample["capture_time_ms"]),
            "sent_at_ms": int(time.time() * 1000),
        }
        info = self._client.publish(self.frame_topic, encode_frame(header, sample["jpeg"]), qos=self.qos)
        if info.rc != 0:
            self.dropped += 1
            self.last_error = f"publish failed rc={info.rc}"
            return False
        self.sent += 1
        return True

    def close(self) -> None:
        if self._client is None:
            return
        self._closing = True
        try:
            self._client.loop_stop()
            self._client.disconnect()
        except Exception:
            pass
        self._client = None
        self._connected.clear()
