"""
Watch the robot cell's PLC for the signals that drive a capture run.

Ported from the IMU edge agent (imu_node_smr, branch plc_update,
imu_edge/plc_signals.py) so the cameras and the IMUs start and stop on the
same PLC pulses.

The PLC raises each signal as a short pulse, so the watcher counts each time
a tag turns ON (a rising edge) instead of looking at whether it is on now:

    Data_Start turns ON  -> start the run: detect, save and send camera data
    Db_Stop turns ON     -> stop the run, wait for the next Data_Start

IMU_Tare is only meaningful to the IMUs, so the camera does not watch it by
default (pass tag_tare to watch it anyway). A tag turning OFF again is
ignored. The watcher polls the tags in a background thread so the edge agent can
keep the camera host warm between signals.
"""
import threading
import time
from collections import namedtuple
from typing import Callable, Optional

FakeTag = namedtuple("FakeTag", "tag value type error")


class FakePLC:
    """
    Stand-in for pycomm3's LogixDriver, for testing without the robot.
    Plays back the PLC signal pattern seen in the lab with SJOINT1.
    """

    DEFAULT_TIMELINE = [
        (3.0, "Data_Start", True),   # start pulse
        (4.0, "Data_Start", False),
        (10.0, "IMU_Tare", True),    # tare pulse, after the jerk
        (11.0, "IMU_Tare", False),
        (25.0, "Db_Stop", True),     # end of the run (stays on)
    ]

    def __init__(self, timeline=None, speed: float = 1.0, **_ignored):
        self.timeline = sorted(timeline or self.DEFAULT_TIMELINE)
        self.speed = speed
        self._t0 = time.monotonic()

    def __enter__(self):
        self._t0 = time.monotonic()
        return self

    def __exit__(self, *exc):
        return False

    def read(self, *tags):
        elapsed = (time.monotonic() - self._t0) * self.speed
        state = {tag: False for _at, tag, _value in self.timeline}
        for at, tag, value in self.timeline:
            if elapsed >= at:
                state[tag] = value

        results = [
            FakeTag(tag, state[tag], "BOOL", None)
            if tag in state
            else FakeTag(tag, None, None, "Tag doesn't exist")
            for tag in tags
        ]
        return results[0] if len(results) == 1 else results


def build_plc_factory(path: str, fake: bool = False, fake_speed: float = 1.0) -> Callable:
    """Return a function that opens a connection to the PLC (or the stand-in)."""
    if fake:
        return lambda: FakePLC(speed=fake_speed)

    def _open_real():
        # Imported here so pycomm3 is only needed when the PLC trigger is used.
        from pycomm3 import LogixDriver

        return LogixDriver(path, init_connection=False, large_packets=False)

    return _open_real


class PLCSignalWatcher:
    def __init__(
        self,
        plc_factory: Callable,
        tag_start: str = "Data_Start",
        tag_stop: str = "Db_Stop",
        tag_tare: Optional[str] = None,
        poll_s: float = 0.05,
        retry_delay_s: float = 1.0,
    ):
        self._plc_factory = plc_factory
        self._tag_start = tag_start
        self._tag_stop = tag_stop
        self._tag_tare = tag_tare
        self._tags = tuple(t for t in (tag_start, tag_tare, tag_stop) if t)
        self._poll_s = poll_s
        self._retry_delay_s = retry_delay_s

        self._lock = threading.Lock()
        # How many times each tag has turned ON since the watcher started.
        self._counts = {tag: 0 for tag in self._tags}
        # The value each tag had at the last read.
        self._values = {tag: False for tag in self._tags}
        self._connected = False
        self._last_error: str | None = None

        self._closing = threading.Event()
        self._thread = threading.Thread(target=self._run, name="plc-signals", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        self._closing.set()
        self._thread.join(timeout=5)

    @property
    def start_count(self) -> int:
        """How many times Data_Start has turned ON."""
        return self._count(self._tag_start)

    @property
    def stop_count(self) -> int:
        """How many times Db_Stop has turned ON."""
        return self._count(self._tag_stop)

    @property
    def tare_count(self) -> int:
        """How many times IMU_Tare has turned ON (0 if it is not watched)."""
        return self._count(self._tag_tare) if self._tag_tare else 0

    def _count(self, tag: str) -> int:
        with self._lock:
            return self._counts[tag]

    def snapshot(self) -> str:
        """The current tag values, for log messages."""
        with self._lock:
            return " ".join(f"{tag}={self._values[tag]}" for tag in self._tags)

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._connected

    @property
    def last_error(self) -> str | None:
        with self._lock:
            return self._last_error

    @staticmethod
    def _as_bool(result) -> bool:
        return bool(result.value) if result.error is None else False

    def _run(self) -> None:
        first_read = True
        while not self._closing.is_set():
            try:
                with self._plc_factory() as plc:
                    with self._lock:
                        self._connected = True
                        self._last_error = None

                    while not self._closing.is_set():
                        results = plc.read(*self._tags)
                        # pycomm3 returns a bare Tag (not a list) for one tag.
                        if len(self._tags) == 1:
                            results = [results]
                        errors = [f"{r.tag}: {r.error}" for r in results if r.error]

                        with self._lock:
                            for tag, result in zip(self._tags, results):
                                value = self._as_bool(result)
                                # A tag that is already ON when the watcher
                                # starts is not counted: only a fresh OFF->ON
                                # change starts or stops anything.
                                if value and not self._values[tag] and not first_read:
                                    self._counts[tag] += 1
                                self._values[tag] = value
                            self._last_error = "; ".join(errors) or None
                        first_read = False

                        self._closing.wait(self._poll_s)
            except Exception as error:  # connection lost or refused: retry
                # Signals that arrive while disconnected are missed; the
                # last known values are kept so no false edge is seen on
                # reconnect.
                with self._lock:
                    self._connected = False
                    self._last_error = str(error)
                self._closing.wait(self._retry_delay_s)
