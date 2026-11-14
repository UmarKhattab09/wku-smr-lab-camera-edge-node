"""PLC trigger for camera_edge: config and signal watcher.

The full agent needs real OpenCV, which conftest.py stubs out, so the
end-to-end check lives in scripts/plc_trigger_smoke.py.
"""
import time

from camera_edge.config import EdgeAgentConfig
from camera_edge.plc_signals import FakePLC, PLCSignalWatcher


def _wait_until(cond, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.01)
    return False


def test_plc_trigger_off_by_default(monkeypatch):
    for name in ("CAMERA_PLC_TRIGGER", "CAMERA_PLC_PATH", "CAMERA_PLC_MAX_RUN_S"):
        monkeypatch.delenv(name, raising=False)
    cfg = EdgeAgentConfig.from_env()
    assert cfg.plc_trigger is False
    # Same defaults as imu_edge's IMU_PLC_*.
    assert cfg.plc_path == "192.168.1.103"
    assert (cfg.plc_tag_start, cfg.plc_tag_stop) == ("Data_Start", "Db_Stop")
    assert cfg.plc_poll_s == 0.05 and cfg.plc_max_run_s == 600


def test_plc_settings_from_env(monkeypatch):
    monkeypatch.setenv("CAMERA_PLC_TRIGGER", "true")
    monkeypatch.setenv("CAMERA_PLC_PATH", "10.0.0.5")
    monkeypatch.setenv("CAMERA_PLC_TAG_START", "Cam_Start")
    monkeypatch.setenv("CAMERA_PLC_POLL_S", "0.001")  # clamped like the IMU
    monkeypatch.setenv("CAMERA_PLC_MAX_RUN_S", "0")
    cfg = EdgeAgentConfig.from_env()
    assert cfg.plc_trigger and cfg.plc_path == "10.0.0.5"
    assert cfg.plc_tag_start == "Cam_Start"
    assert cfg.plc_poll_s == 0.01 and cfg.plc_max_run_s == 0


def test_watcher_counts_rising_edges_and_ignores_tare():
    timeline = [
        (0.10, "Data_Start", True),
        (0.20, "Data_Start", False),
        (0.30, "IMU_Tare", True),
        (0.40, "Db_Stop", True),  # stays ON: one edge
    ]
    watcher = PLCSignalWatcher(lambda: FakePLC(timeline=timeline), poll_s=0.01)
    watcher.start()
    try:
        assert _wait_until(lambda: watcher.stop_count == 1)
        time.sleep(0.2)
        assert (watcher.start_count, watcher.stop_count, watcher.tare_count) == (1, 1, 0)
        assert "IMU_Tare" not in watcher.snapshot()
    finally:
        watcher.close()


def test_watcher_ignores_tag_already_on_at_startup():
    watcher = PLCSignalWatcher(lambda: FakePLC(timeline=[(0.0, "Data_Start", True)]), poll_s=0.01)
    watcher.start()
    try:
        assert _wait_until(lambda: watcher.connected)
        time.sleep(0.1)
        assert watcher.start_count == 0
    finally:
        watcher.close()


def test_watcher_retries_when_plc_unreachable():
    def _refuse():
        raise ConnectionError("no route to PLC")

    watcher = PLCSignalWatcher(_refuse, poll_s=0.01, retry_delay_s=0.01)
    watcher.start()
    try:
        assert _wait_until(lambda: watcher.last_error == "no route to PLC")
        assert not watcher.connected
    finally:
        watcher.close()
