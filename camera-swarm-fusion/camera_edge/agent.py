from __future__ import annotations

import logging
from pathlib import Path
import signal
import time

import cv2
import numpy as np

from camera_fusion.detect import build_detector, detect_markers
from camera_fusion.output import CsvOutput, MqttOutput
from iiot_pipeline.services.calib import load_calib
from iiot_pipeline.services.storage import SessionStorage
from iiot_pipeline.strategies.localize_pnp import PnPLocalize

from .config import EdgeAgentConfig

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
)
logger = logging.getLogger("camera_edge")
from .health import EdgeHealthTracker
from .plc_signals import PLCSignalWatcher, build_plc_factory
from .service_client import LocalCameraServiceClient, ServiceError


class EdgeAgent:
    """Swarm-managed detection/pose/MQTT agent.

    Never touches /dev/video* or cv2.VideoCapture -- it only talks to
    camera_host over the Unix socket, then reuses the existing ArUco
    detection (camera_fusion.detect) and PnP pose estimation
    (iiot_pipeline.strategies.localize_pnp.PnPLocalize) unchanged, and
    publishes through the existing CsvOutput/MqttOutput sinks so the
    detections.csv schema and MQTT payload/topic stay exactly what the
    middleware already expects.
    """

    def __init__(self, config: EdgeAgentConfig, plc_factory=None):
        self.config = config
        self._plc_factory = plc_factory  # override the PLC connection (tests)
        self.client = LocalCameraServiceClient(config.socket_path)
        self.health = EdgeHealthTracker(config.health_path)
        self._stop_requested = False

        K, dist, _ = load_calib(config.calibration_path)
        self._localize = PnPLocalize(K, dist, config.marker_length_m)
        self._detector_state = build_detector(config.aruco_dict)
        self._target_ids = set(config.target_ids) if config.target_ids else None

    def install_signal_handlers(self) -> None:
        def _handle_signal(_signum, _frame):
            self._stop_requested = True

        signal.signal(signal.SIGINT, _handle_signal)
        signal.signal(signal.SIGTERM, _handle_signal)

    def run(self) -> None:
        self.install_signal_handlers()
        logger.info(
            "starting camera_edge camera_name=%s device_id=%s broker=%s:%s calibration=%s",
            self.config.camera_name, self.config.device_id,
            self.config.mqtt_broker_ip, self.config.mqtt_broker_port,
            self.config.calibration_path,
        )

        csv_output = CsvOutput()
        mqtt_output = MqttOutput(
            broker_ip=self.config.mqtt_broker_ip,
            broker_port=self.config.mqtt_broker_port,
            device_id=self.config.device_id,
            client_type=self.config.client_type,
            camera_name=self.config.camera_name,
        )

        if self.config.plc_trigger:
            # One MQTT connection for the container's lifetime, like imu_edge's
            # writer; a new session folder + detections.csv per PLC run.
            mqtt_output.open(Path(self.config.session_root))
            try:
                self._run_plc_triggered(mqtt_output)
            finally:
                mqtt_output.close()
                if self.config.stop_session_on_exit:
                    try:
                        self.client.stop_session()
                    except (ServiceError, OSError):
                        pass
            return

        session_dir = self._open_session_storage()

        csv_output.open(session_dir)
        mqtt_output.open(session_dir)

        try:
            while not self._stop_requested:
                try:
                    self._wait_until_ready()
                    if self.config.start_session_on_boot:
                        self._start_session()

                    if self.config.use_frame_stream:
                        self._consume_stream(csv_output, mqtt_output)
                    else:
                        self._poll_latest(csv_output, mqtt_output)
                except ServiceError as error:
                    logger.warning("service error: %s %s", error.code, error.message)
                    self.health.update(state="degraded", error_code=error.code, error_message=error.message)
                    time.sleep(self.config.reconnect_delay_s)
                except Exception as error:
                    logger.exception("unexpected failure in edge loop")
                    self.health.update(state="degraded", error_code="EDGE_FAILURE", error_message=str(error))
                    time.sleep(self.config.reconnect_delay_s)
        finally:
            csv_output.close()
            mqtt_output.close()

        if self.config.stop_session_on_exit:
            try:
                self.client.stop_session()
            except ServiceError:
                pass

    def _open_session_storage(self) -> Path:
        storage = SessionStorage(self.config.session_root, name=f"{self.config.camera_name}_session")
        session_dir = Path(storage.begin())
        self._frames_dir = storage.frames_dir
        self._storage = storage
        logger.info("session directory: %s", session_dir)

        storage.write_manifest({
            "camera_name": self.config.camera_name,
            "device_id": self.config.device_id,
            "calibration_path": self.config.calibration_path,
            "marker_length_m": self.config.marker_length_m,
            "aruco_dict": self.config.aruco_dict,
            "fps": self.config.fps,
            "width": self.config.width,
            "height": self.config.height,
            "session_id": self.config.session_id,
            "plc_trigger": self.config.plc_trigger,
        })
        return session_dir

    # ------------------------------------------------------------------
    # PLC trigger -- mirrors imu_edge.agent._run_plc_triggered. The PLC
    # sends short pulses, so only a tag turning ON matters:
    #
    #     Data_Start turns ON  -> start: detect, save and send frames
    #     Db_Stop turns ON     -> stop sending, wait for the next Data_Start
    #
    # One difference from the IMU: the camera_host session is started once
    # and left running between runs, because opening a USB camera takes
    # about a second and would cut off the start of every run. Instead each
    # run opens a fresh /v1/frame/stream (a new, empty queue on the host)
    # and drops any frame captured before the Data_Start edge.
    # ------------------------------------------------------------------

    def _run_plc_triggered(self, mqtt_output: MqttOutput) -> None:
        watcher = PLCSignalWatcher(
            self._plc_factory
            or build_plc_factory(
                self.config.plc_path,
                fake=self.config.plc_fake,
                fake_speed=self.config.plc_fake_speed,
            ),
            tag_start=self.config.plc_tag_start,
            tag_stop=self.config.plc_tag_stop,
            poll_s=self.config.plc_poll_s,
        )
        watcher.start()
        logger.info(
            "PLC trigger on: PLC %s, start on %s, stop on %s",
            self.config.plc_path, self.config.plc_tag_start, self.config.plc_tag_stop,
        )

        try:
            while not self._stop_requested:
                try:
                    self._wait_until_ready()
                    # Keep the camera warm between runs (no-op if already active).
                    self._start_session(quiet=True)
                    if not self._wait_for_plc_start(watcher):
                        break
                    self._capture_one_run(watcher, mqtt_output)
                except ServiceError as error:
                    logger.warning("service error: %s %s", error.code, error.message)
                    self.health.update(state="degraded", error_code=error.code, error_message=error.message)
                    time.sleep(self.config.reconnect_delay_s)
                except Exception as error:
                    logger.exception("unexpected failure in PLC-triggered loop")
                    self.health.update(state="degraded", error_code="EDGE_FAILURE", error_message=str(error))
                    time.sleep(self.config.reconnect_delay_s)
        finally:
            watcher.close()

    def _wait_for_plc_start(self, watcher: PLCSignalWatcher) -> bool:
        """Block until Data_Start turns ON. False if the agent is shutting down."""
        starts_seen = watcher.start_count
        last_health = 0.0
        while not self._stop_requested:
            if watcher.start_count > starts_seen:
                return True

            # Keep the health file fresh so the container stays healthy
            # however long the robot takes to start.
            if time.monotonic() - last_health > 5.0:
                detail = f"waiting for PLC tag {self.config.plc_tag_start}"
                if not watcher.connected:
                    detail = f"cannot reach PLC at {self.config.plc_path}: {watcher.last_error}"
                elif watcher.last_error:
                    detail = f"PLC tag problem: {watcher.last_error}"
                self.health.update(state="waiting", detail=detail)
                logger.info("%s   [%s]", detail, watcher.snapshot())
                last_health = time.monotonic()

            time.sleep(self.config.plc_poll_s)
        return False

    def _capture_one_run(self, watcher: PLCSignalWatcher, mqtt_output: MqttOutput) -> None:
        stops_seen = watcher.stop_count
        start_ms = int(time.time() * 1000)
        run_started = time.monotonic()
        max_run_s = self.config.plc_max_run_s
        first_frame_idx: int | None = None
        sent = 0
        dropped = 0
        end_reason = "camera_host stopped sending frames"

        csv_output = CsvOutput()
        session_dir = self._open_session_storage()
        csv_output.open(session_dir)
        logger.info(
            "PLC: %s ON -> detecting and sending   [%s]",
            self.config.plc_tag_start, watcher.snapshot(),
        )

        samples = self._samples()
        try:
            for sample in samples:
                if self._stop_requested:
                    end_reason = "the agent was stopped"
                    break
                if watcher.stop_count > stops_seen:
                    end_reason = f"{self.config.plc_tag_stop} turned ON"
                    break
                if max_run_s > 0 and time.monotonic() - run_started > max_run_s:
                    end_reason = (
                        f"no {self.config.plc_tag_stop} signal within "
                        f"{max_run_s:g} s (CAMERA_PLC_MAX_RUN_S)"
                    )
                    break

                # Frames captured before the Data_Start edge don't belong to the run.
                if sample["capture_time_ms"] < start_ms:
                    dropped += 1
                    continue

                # Number the frames of each run from 0, like the IMU samples.
                if first_frame_idx is None:
                    first_frame_idx = sample["frame_idx"]
                sample["frame_idx"] -= first_frame_idx
                self._process_frame(sample, csv_output, mqtt_output)
                sent += 1
        except Exception as error:
            end_reason = f"error: {type(error).__name__}: {error}"
            raise
        finally:
            samples.close()  # closes the host stream
            csv_output.close()
            logger.info(
                "PLC: run ended (%d frames sent, %d pre-start frames dropped); reason: %s; session %s",
                sent, dropped, end_reason, session_dir,
            )
            self.health.update(state="waiting", detail="run ended; re-armed")

    def _samples(self):
        """Frames from camera_host, from a fresh stream or by polling."""
        if self.config.use_frame_stream:
            with self.client.stream() as stream:
                yield from stream
            return

        last_frame_idx = None
        while True:
            frame = self.client.frame_latest()
            if frame["frame_idx"] != last_frame_idx:
                last_frame_idx = frame["frame_idx"]
                yield {
                    "jpeg": frame["jpeg"],
                    "frame_idx": frame["frame_idx"],
                    "capture_time_ms": frame["capture_time_ms"],
                }
            time.sleep(self.config.poll_interval_s)

    def _wait_until_ready(self) -> None:
        while not self._stop_requested:
            try:
                ready = self.client.readiness()
            except OSError as error:
                self.health.update(state="waiting", error_code="SERVICE_UNAVAILABLE", error_message=str(error))
                time.sleep(self.config.reconnect_delay_s)
                continue
            except ServiceError as error:
                self.health.update(state="waiting", error_code=error.code, error_message=error.message)
                time.sleep(self.config.reconnect_delay_s)
                continue

            if ready.get("ready"):
                return

            time.sleep(self.config.reconnect_delay_s)

    def _start_session(self, quiet: bool = False) -> None:
        try:
            self.client.start_session(
                self.config.session_id, self.config.fps, self.config.width, self.config.height
            )
            logger.info("session started: %s", self.config.session_id)
        except ServiceError as error:
            if error.code != "SESSION_ALREADY_ACTIVE":
                raise
            if not quiet:
                logger.info("session already active: %s", self.config.session_id)

    def _consume_stream(self, csv_output: CsvOutput, mqtt_output: MqttOutput) -> None:
        with self.client.stream() as stream:
            for sample in stream:
                if self._stop_requested:
                    break
                self._process_frame(sample, csv_output, mqtt_output)

    def _poll_latest(self, csv_output: CsvOutput, mqtt_output: MqttOutput) -> None:
        last_frame_idx = None
        while not self._stop_requested:
            frame = self.client.frame_latest()
            if frame["frame_idx"] != last_frame_idx:
                last_frame_idx = frame["frame_idx"]
                sample = {
                    "jpeg": frame["jpeg"],
                    "frame_idx": frame["frame_idx"],
                    "capture_time_ms": frame["capture_time_ms"],
                }
                self._process_frame(sample, csv_output, mqtt_output)
            time.sleep(self.config.poll_interval_s)

    def _process_frame(self, sample: dict, csv_output: CsvOutput, mqtt_output: MqttOutput) -> None:
        image = cv2.imdecode(np.frombuffer(sample["jpeg"], dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            return

        dets = detect_markers(image, self._detector_state)
        if self.config.marker_lengths_m:
            poses = self._localize.estimate_with_lengths(
                dets, self.config.marker_lengths_m, default_length=self.config.marker_length_m
            )
        else:
            poses = self._localize.estimate(dets)

        pose_map = {
            d.marker_id: poses[i] for i, d in enumerate(dets) if i < len(poses) and poses[i] is not None
        }

        capture_time = sample["capture_time_ms"] / 1000.0
        ts_unix = time.time()

        image_path = None
        if self.config.save_frames:
            frame_filename = f"f{sample['frame_idx']:06d}.jpg"
            frame_path = self._frames_dir / frame_filename
            cv2.imwrite(str(frame_path), image)
            try:
                image_path = str(frame_path.relative_to("/app"))
            except ValueError:
                image_path = str(frame_path)

        if dets and self.config.save_annotated:
            draw = image.copy()
            h, w = draw.shape[:2]
            ts_iso = time.strftime("%Y-%m-%dT%H:%M:%S")
            txt = f"#{sample['frame_idx']} {ts_iso} {w}x{h}"
            cv2.putText(draw, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)

            for det in dets:
                color = (0, 255, 0)  # green -- camera_edge has no LightGlue fallback, always aruco source
                corners_int = det.corners.astype(np.int32)
                cv2.polylines(draw, [corners_int], True, color, 2)
                center = corners_int.reshape(-1, 2).mean(axis=0)
                org = (int(center[0]), int(center[1]))
                cv2.putText(draw, str(det.marker_id), org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2, cv2.LINE_AA)

            for det in dets:
                pose = pose_map.get(det.marker_id)
                if pose is None:
                    continue
                try:
                    cv2.drawFrameAxes(
                        draw, self._localize.K, self._localize.dist,
                        pose.rvec, pose.tvec,
                        max(0.01, self.config.marker_length_m * 0.5),
                    )
                except Exception:
                    logger.warning("drawFrameAxes failed for marker_id=%s", det.marker_id)

            self._storage.save_annotated(sample["frame_idx"], draw)

        for det in dets:
            if self._target_ids is not None and det.marker_id not in self._target_ids:
                continue

            pose = pose_map.get(det.marker_id)
            rvec = pose.rvec if pose is not None else None
            tvec = pose.tvec if pose is not None else None

            length_m = self.config.marker_length_m
            if self.config.marker_lengths_m and det.marker_id in self.config.marker_lengths_m:
                length_m = self.config.marker_lengths_m[det.marker_id]

            for out in (csv_output, mqtt_output):
                out.write_detection(
                    ts_unix,
                    sample["frame_idx"],
                    det.marker_id,
                    rvec,
                    tvec,
                    image_path,
                    length_m=length_m,
                    capture_time=capture_time,
                )

        self.health.update(
            state="streaming",
            session_id=self.config.session_id,
            last_capture_time_ms=sample["capture_time_ms"],
            last_frame_idx=sample["frame_idx"],
            detections=len(dets),
        )
        logger.info(
            "frame=%s dets=%s saved=%s",
            sample["frame_idx"], len(dets), image_path if image_path else "none",
        )
