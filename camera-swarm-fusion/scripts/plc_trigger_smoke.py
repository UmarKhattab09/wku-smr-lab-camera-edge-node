"""End-to-end check of the camera_edge PLC trigger, no robot or camera needed.

Runs the real camera_host service (fake camera backend drawing ArUco markers)
on a temporary Unix socket, and the real camera_edge agent against it, with a
fake PLC playing back two Data_Start ... Db_Stop runs.

    python scripts/plc_trigger_smoke.py

Checks that: nothing is sent before Data_Start, sending stops at Db_Stop,
each run gets its own session folder, frames are numbered from 0 per run,
markers are detected, and the agent re-arms between runs.
"""
from __future__ import annotations

import csv
import os
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

import camera_host.fake_source as fake_source  # noqa: E402
from camera_edge.agent import EdgeAgent  # noqa: E402
from camera_edge.config import EdgeAgentConfig  # noqa: E402
from camera_edge.plc_signals import FakePLC  # noqa: E402
from camera_host.manager import CameraHardwareService, build_hardware_factory  # noqa: E402
from camera_host.server import CameraRequestHandler, ThreadingUnixHTTPServer  # noqa: E402

# Fake camera frames with markers 0, 1, 2 so detection actually runs.
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


class _NullMqtt:
    """Records what would have been published instead of needing a broker."""

    sent: list = []

    def __init__(self, **_kw):
        pass

    def open(self, _session_dir):
        pass

    def write_detection(self, ts_unix, frame_idx, marker_id, *a, **kw):
        _NullMqtt.sent.append((time.time(), frame_idx, marker_id))

    def close(self):
        pass


def main() -> int:
    tmp = Path(tempfile.mkdtemp(prefix="cam-plc-"))
    sock = str(tmp / "camera.sock")

    service = CameraHardwareService(
        backend_name="fake",
        hardware_factory=build_hardware_factory("fake", None, 15, 640, 480),
        jpeg_quality=85,
    )
    service.start()
    server = ThreadingUnixHTTPServer(sock, CameraRequestHandler, service)
    threading.Thread(target=server.serve_forever, daemon=True).start()

    timeline = [
        (1.0, "Data_Start", True), (1.2, "Data_Start", False),
        (2.5, "Db_Stop", True), (2.7, "Db_Stop", False),
        (3.5, "Data_Start", True), (3.7, "Data_Start", False),
        (5.0, "Db_Stop", True),
    ]

    env = {
        "CAMERA_SOCKET_PATH": sock,
        "CAMERA_SESSION_ROOT": str(tmp / "sessions"),
        "CAMERA_EDGE_HEALTH_PATH": str(tmp / "health.json"),
        "CAMERA_NAME": "smoke",
        "CAMERA_FPS": "15",
        "CAMERA_WIDTH": "640",
        "CAMERA_HEIGHT": "480",
        "CAMERA_MARKER_LENGTHS_M": "0:0.15,1:0.05,2:0.05",
        "CAMERA_TARGET_IDS": "1,2",
        "CAMERA_PLC_TRIGGER": "true",
        "CAMERA_PLC_POLL_S": "0.02",
        "CAMERA_RECONNECT_DELAY_S": "0.2",
    }
    os.environ.update(env)
    config = EdgeAgentConfig.from_env()
    assert config.plc_trigger

    import camera_edge.agent as agent_mod
    agent_mod.MqttOutput = _NullMqtt
    agent = EdgeAgent(config, plc_factory=lambda: FakePLC(timeline=timeline))
    agent.install_signal_handlers = lambda: None  # not on the main thread's signals

    t0 = time.time()
    worker = threading.Thread(target=agent.run, daemon=True)
    worker.start()
    time.sleep(6.0)
    agent._stop_requested = True
    worker.join(timeout=15)
    server.shutdown()
    server.server_close()
    service.close()

    ok = True

    def check(cond, msg):
        nonlocal ok
        print(("PASS  " if cond else "FAIL  ") + msg)
        ok &= bool(cond)

    sessions = sorted((tmp / "sessions").iterdir())
    check(len(sessions) == 2, f"two PLC runs -> two session folders ({len(sessions)})")
    for s in sessions:
        with open(s / "detections.csv") as f:
            rows = list(csv.DictReader(f))
        idx = sorted({int(r["frame_idx"]) for r in rows})
        ids = sorted({r["marker_id"] for r in rows})
        check(rows and idx[0] == 0, f"{s.name}: {len(idx)} frames, numbered from {idx[0] if idx else '-'}")
        check(ids == ["1", "2"], f"{s.name}: target markers detected {ids}")
        check(len(list((s / "frames").glob("*.jpg"))) == len(idx), f"{s.name}: frames saved")

    times = [t - t0 for t, _i, _m in _NullMqtt.sent]
    in_run = lambda t: 0.9 <= t <= 2.7 or 3.4 <= t <= 5.2  # noqa: E731  (+ one poll/frame slack)
    check(times and all(in_run(t) for t in times),
          f"published only between Data_Start and Db_Stop "
          f"({len(times)} rows, {min(times):.2f}s..{max(times):.2f}s)" if times else "published nothing")
    check(not worker.is_alive(), "agent stopped cleanly")
    print("\nOK" if ok else "\nFAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
