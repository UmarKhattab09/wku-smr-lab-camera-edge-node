"""End-to-end check of CAMERA_PROCESSING=remote against a running camera-processor.

    CAM_PI (camera_host + camera_edge, here)  -> MQTT broker -> camera-processor
    -> camera/<device> rows                   (read back here and checked)

Needs an MQTT broker with camera-processor (wku-smr-lab-middleware,
camera_processor/) subscribed to it, e.g. the AMS stack, or locally:

    docker compose up -d mqtt-broker camera-processor     # in the middleware repo
    python scripts/remote_processing_smoke.py 127.0.0.1 1883

Runs the real camera_host (fake camera drawing ArUco markers 0, 1, 2) and the
real camera_edge agent in remote mode with a fake PLC playing two
Data_Start ... Db_Stop runs. Checks that the Pi forwards frames only during a
run, each run gets its own run_id with frames numbered from 0, rows come back
on camera/<device> for the target markers only, and every row matches what
CAMERA_PROCESSING=local computes from the same JPEG.
"""
from __future__ import annotations

import csv
import io
import json
import os
import struct
import sys
import tempfile
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
os.chdir(ROOT)

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import paho.mqtt.client as mqtt  # noqa: E402

import camera_host.fake_source as fake_source  # noqa: E402
from camera_edge.agent import EdgeAgent  # noqa: E402
from camera_edge.config import EdgeAgentConfig  # noqa: E402
from camera_edge.plc_signals import FakePLC  # noqa: E402
from camera_fusion.output import MqttOutput, NullOutput  # noqa: E402
from camera_host.manager import CameraHardwareService, build_hardware_factory  # noqa: E402
from camera_host.server import CameraRequestHandler, ThreadingUnixHTTPServer  # noqa: E402

_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
_FRAME = np.full((480, 640, 3), 255, np.uint8)
for _id, (_x, _y) in enumerate([(40, 40), (240, 140), (440, 240)]):
    _FRAME[_y:_y + 150, _x:_x + 150] = cv2.cvtColor(
        cv2.aruco.generateImageMarker(_dict, _id, 150), cv2.COLOR_GRAY2BGR
    )
_orig_read = fake_source.FakeCameraSource.read


def _read_with_markers(self):
    _img, ts, fid = _orig_read(self)
    return _FRAME.copy(), ts, fid


fake_source.FakeCameraSource.read = _read_with_markers


class _Recorder:
    """A local-mode output sink that keeps the rows MqttOutput would publish."""

    def __init__(self, device_id):
        self.rows = []
        self._out = MqttOutput(broker_ip="", broker_port=0, device_id=device_id,
                               client_type="CAMERA", camera_name="")

    def write_detection(self, ts_unix, frame_idx, marker_id, rvec, tvec, image_path, **kw):
        self.rows.append(self._out._build_row(ts_unix, frame_idx, marker_id, rvec, tvec, image_path,
                                              False, None, None, kw.get("length_m"), kw.get("capture_time")))


def _client(client_id):
    if hasattr(mqtt, "CallbackAPIVersion"):
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
    return mqtt.Client(client_id=client_id)


def main(argv) -> int:
    broker = argv[1] if len(argv) > 1 else "127.0.0.1"
    port = int(argv[2]) if len(argv) > 2 else 1883
    device_id, name = "999", f"smoke{os.getpid()}"
    label = f"{device_id}_{name}"

    tmp = Path(tempfile.mkdtemp(prefix="cam-remote-"))
    sock = str(tmp / "camera.sock")
    service = CameraHardwareService(
        backend_name="fake",
        hardware_factory=build_hardware_factory("fake", None, 15, 640, 480),
        jpeg_quality=85,
    )
    service.start()
    server = ThreadingUnixHTTPServer(sock, CameraRequestHandler, service)
    threading.Thread(target=server.serve_forever, daemon=True).start()

    # What crosses the wire: forwarded frames, and the rows camera-processor sends back
    frames, rows, lock = {}, [], threading.Lock()
    t0 = time.time()

    def on_message(_c, _u, msg):
        with lock:
            if msg.topic == f"camera_frames/{label}/frame":
                (n,) = struct.unpack_from(">I", msg.payload, 0)
                header = json.loads(msg.payload[4:4 + n])
                frames[(header["run_id"], header["frame_idx"])] = (time.time() - t0, header, msg.payload[4 + n:])
            elif msg.topic == f"camera/{label}":
                rows.append((time.time() - t0, next(csv.reader(io.StringIO(msg.payload.decode())))))

    sub = _client(f"smoke-sub-{os.getpid()}")
    sub.on_message = on_message
    sub.connect(broker, port)
    sub.subscribe([(f"camera_frames/{label}/frame", 0), (f"camera/{label}", 0)])
    sub.loop_start()

    timeline = [
        (1.0, "Data_Start", True), (1.2, "Data_Start", False),
        (2.5, "Db_Stop", True), (2.7, "Db_Stop", False),
        (3.5, "Data_Start", True), (3.7, "Data_Start", False),
        (5.0, "Db_Stop", True),
    ]
    os.environ.update({
        "CAMERA_SOCKET_PATH": sock,
        "CAMERA_SESSION_ROOT": str(tmp / "sessions"),
        "CAMERA_EDGE_HEALTH_PATH": str(tmp / "health.json"),
        "DEVICE_ID": device_id,
        "CAMERA_NAME": name,
        "CAMERA_FPS": "15",
        "CAMERA_WIDTH": "640",
        "CAMERA_HEIGHT": "480",
        "CAMERA_MARKER_LENGTH_M": "0.05",
        "CAMERA_MARKER_LENGTHS_M": "0:0.15,1:0.05,2:0.05",
        "CAMERA_TARGET_IDS": "1,2",
        "CAMERA_PLC_TRIGGER": "true",
        "CAMERA_PLC_POLL_S": "0.02",
        "CAMERA_RECONNECT_DELAY_S": "0.2",
        "CAMERA_PROCESSING": "remote",
        "MQTT_BROKER_IP": broker,
        "MQTT_BROKER_PORT": str(port),
    })
    config = EdgeAgentConfig.from_env()
    assert config.remote_processing and config.plc_trigger

    agent = EdgeAgent(config, plc_factory=lambda: FakePLC(timeline=timeline))
    agent.install_signal_handlers = lambda: None
    t0 = time.time()
    worker = threading.Thread(target=agent.run, daemon=True)
    worker.start()
    time.sleep(6.0)
    agent._stop_requested = True
    worker.join(timeout=15)
    time.sleep(1.5)  # let camera-processor finish the last frames
    sub.loop_stop()
    server.shutdown()
    server.server_close()
    service.close()

    ok = True

    def check(cond, msg):
        nonlocal ok
        print(("PASS  " if cond else "FAIL  ") + msg)
        ok &= bool(cond)

    runs = sorted({run for run, _ in frames})
    check(len(runs) == 2, f"two PLC runs -> two run ids ({runs})")
    for run in runs:
        idx = sorted(i for r, i in frames if r == run)
        check(idx and idx[0] == 0 and idx == list(range(len(idx))), f"{run}: {len(idx)} frames forwarded, numbered 0..")
    in_run = lambda t: 0.9 <= t <= 2.7 or 3.4 <= t <= 5.2  # noqa: E731
    check(frames and all(in_run(t) for t, _h, _j in frames.values()),
          "frames forwarded only between Data_Start and Db_Stop")
    check(not (tmp / "sessions").exists() or not any((tmp / "sessions").iterdir()),
          "nothing processed or saved on the Pi")

    check(rows, f"camera-processor published {len(rows)} rows on camera/{label}")
    check({r[3] for _t, r in rows} == {"1", "2"}, f"target markers only: {sorted({r[3] for _t, r in rows})}")
    check(all(len(r) == 13 and r[12] == device_id for _t, r in rows), "row layout: 13 fields, device_id last")

    # Same JPEG through the local (on-Pi) pipeline -> same numbers
    local = EdgeAgent(EdgeAgentConfig.from_env())
    local.config.save_frames = local.config.save_annotated = False
    local._forwarder = None
    mismatches = compared = 0
    by_frame = {}
    for _t, r in rows:
        by_frame.setdefault((r[0], r[1]), []).append(r)
    for (_run, frame_idx), (_t, header, jpeg) in frames.items():
        remote_rows = by_frame.get((str(frame_idx), f"{header['capture_time_ms'] / 1000:.6f}"))
        if not remote_rows:
            continue
        rec = _Recorder(device_id)
        local._process_frame({"jpeg": jpeg, "frame_idx": frame_idx, "capture_time_ms": header["capture_time_ms"]},
                             NullOutput(), rec)
        want = sorted([str(v) for i, v in enumerate(row) if i not in (2, 11)] for row in rec.rows)
        got = sorted([v for i, v in enumerate(row) if i not in (2, 11)] for row in remote_rows)
        compared += 1
        mismatches += want != got
        if want != got and mismatches == 1:
            print("  local :", want, "\n  remote:", got)
    check(compared and not mismatches, f"remote rows == local rows for {compared} frames ({mismatches} differ)")
    check(not worker.is_alive(), "agent stopped cleanly")
    print("\nOK" if ok else "\nFAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
