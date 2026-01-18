"""Manage rosbag-based teleoperation dataset capture sessions."""

from __future__ import annotations

import os
import shutil
import signal
import subprocess
import sys
import threading
from datetime import datetime
from pathlib import Path
from typing import Optional

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError

from .bag_converter import convert_bag_to_dataset
from .config import CaptureConfig, load_capture_config


class DataCaptureNode(Node):
    """Coordinates rosbag2 recording and dataset conversion."""

    def __init__(self) -> None:
        super().__init__("data_capture_manager")
        default_share: Optional[Path] = None
        try:
            default_share = Path(get_package_share_directory("data_capture_tools"))
        except PackageNotFoundError:
            default_share = Path(__file__).resolve().parent.parent / "config"
        default_config = str(default_share / "data_capture.yaml")
        config_path = (
            self.declare_parameter("config", default_config)
            .get_parameter_value()
            .string_value
        )
        self.config: CaptureConfig = load_capture_config(config_path)

        self._stop_event = threading.Event()
        self._stop_time_ns: Optional[int] = None
        self._snapped_time_ns: Optional[int] = None
        self._discard_fast: bool = False
        self._force_save: bool = False
        self._score: Optional[str] = None
        self._bag_process: Optional[subprocess.Popen] = None
        self._bag_stdout = None
        self._bag_stderr = None
        self._session_dir: Optional[Path] = None
        self._bag_uri: Optional[Path] = None
        self._viewer_processes: list[subprocess.Popen] = []
        self._aux_processes: list[subprocess.Popen] = []
        self._viewer_subscriptions: list = []
        self._viewer_frames: dict[str, object] = {}
        self._viewer_timer = None
        self._bridge = CvBridge()
        self._convert_after_record: bool = (
            self.declare_parameter("convert_after_record", False).value
        )
        self._prompt_discard_for_zero: bool = False
        self._user_thread = threading.Thread(
            target=self._watch_user_input, daemon=True
        )

    def run(self) -> None:
        self._prepare_session()
        self._cleanup_existing_viewers()
        self._start_internal_viewers()
        self._start_throttles_and_compressors()
        self._start_rosbag()
        self._user_thread.start()
        self.get_logger().info(
            "Recording in progress. Use Ctrl+C, the stop key, or the discard key to finish."
        )
        try:
            while rclpy.ok() and not self._stop_event.is_set():
                rclpy.spin_once(self, timeout_sec=0.2)
        except KeyboardInterrupt:
            self.get_logger().info("Keyboard interrupt received, stopping session.")
            self._stop_event.set()
            self._record_stop_time_if_missing()
        finally:
            self._record_stop_time_if_missing()
            self._finalize()

    def _prepare_session(self) -> None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        session_dir = self.config.output_root / self.config.task_name / timestamp
        session_dir.mkdir(parents=True, exist_ok=True)
        (session_dir / "bag").mkdir(exist_ok=True)
        (session_dir / "logs").mkdir(exist_ok=True)
        shutil.copy2(self.config.source_path, session_dir / "config.yaml")
        (session_dir / "prompt.txt").write_text(
            self.config.prompt + "\n", encoding="utf-8"
        )

        bag_uri = session_dir / "bag" / f"session_{timestamp}"
        self._session_dir = session_dir
        self._bag_uri = bag_uri

    def _start_rosbag(self) -> None:
        if self._bag_uri is None:
            raise RuntimeError("Session directory was not initialized")

        cmd = [
            "ros2",
            "bag",
            "record",
            "-s",
            self.config.bag.storage,
        ]
        if self.config.bag.compression_mode:
            cmd.extend(["--compression-mode", self.config.bag.compression_mode])
        if self.config.bag.compression_format:
            cmd.extend(["--compression-format", self.config.bag.compression_format])
        cmd.extend(["-o", str(self._bag_uri)])
        cmd.extend([topic.name for topic in self.config.topics])

        log_dir = self._session_dir / "logs" if self._session_dir else Path(".")
        stdout_path = log_dir / "rosbag_stdout.log"
        stderr_path = log_dir / "rosbag_stderr.log"
        self._bag_stdout = stdout_path.open("w", encoding="utf-8")
        self._bag_stderr = stderr_path.open("w", encoding="utf-8")
        self.get_logger().info(f"Launching rosbag2: {' '.join(cmd)}")
        self._bag_process = subprocess.Popen(
            cmd,
            stdout=self._bag_stdout,
            stderr=self._bag_stderr,
            env=os.environ.copy(),
        )

    def _watch_user_input(self) -> None:
        if not sys.stdin.isatty():
            self.get_logger().warning(
                "stdin is not a TTY; press Ctrl+C to stop recording instead."
            )
            return
        stop_keys = {"0", "1", "2"}
        self.get_logger().info(
            "Press 1 or 2 to stop and save with that score. Press 0 to stop and be asked whether to discard."
        )
        while not self._stop_event.is_set():
            try:
                line = sys.stdin.readline()
            except Exception:
                break
            if not line:
                continue
            stripped = line.strip()
            if stripped.lower() == "f":
                self._record_snapped_time()
                continue
            if stripped in stop_keys:
                if stripped == "0":
                    self.get_logger().info("Stop key 0 received; will prompt to discard or save.")
                    self._prompt_discard_for_zero = True
                    self._force_save = False
                    self._score = "0"
                else:
                    self.get_logger().info(f"Stop key received with score {stripped}.")
                    self._force_save = True  # Always save when a scored stop key is used.
                    self._score = stripped
                self._stop_event.set()
                self._record_stop_time_if_missing()
                break

    def _finalize(self) -> None:
        self._stop_rosbag(fast=self._discard_fast)
        self._stop_viewers()
        self._stop_aux_processes()
        self._cleanup_existing_viewers()  # ensure stragglers are gone
        if not (self._session_dir and self._bag_uri and self._bag_uri.exists()):
            self.get_logger().error("Bag output not found; skipping conversion.")
            return

        if self._discard_fast:
            self.get_logger().info("Discard flag set; skipping save and removing session directory.")
            shutil.rmtree(self._session_dir, ignore_errors=True)
            return

        if not self._force_save and not self._confirm_save():
            self.get_logger().info("Discarding captured data at user request.")
            shutil.rmtree(self._session_dir, ignore_errors=True)
            return

        if self._score is not None:
            try:
                (self._session_dir / "score.txt").write_text(f"{self._score}\n", encoding="utf-8")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f"Failed to write score.txt: {exc}")

        if self._prompt_discard_for_zero:
            if not self._confirm_keep_for_zero():
                self.get_logger().info("Score 0: user chose to discard (default). Removing session directory.")
                shutil.rmtree(self._session_dir, ignore_errors=True)
                return
            else:
                self.get_logger().info("Score 0: user chose to keep. Proceeding to save.")
                self._force_save = True

        if not self._convert_after_record:
            self.get_logger().info(
                "Conversion disabled (convert_after_record:=false); keeping bag only."
            )
            return

        try:
            convert_bag_to_dataset(
                self._bag_uri,
                self._session_dir,
                self.config,
                logger=self.get_logger(),
                cutoff_stamp_ns=self._stop_time_ns,
            )
            self.get_logger().info(
                f"Capture complete. Dataset stored in {self._session_dir}"
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to convert bag: {exc}")

    def _stop_rosbag(self, fast: bool = False) -> None:
        if self._bag_process and self._bag_process.poll() is None:
            self.get_logger().info("Stopping rosbag2 process...")
            self._bag_process.send_signal(signal.SIGINT)
            try:
                self._bag_process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                self.get_logger().warning(
                    "rosbag2 did not terminate after SIGINT; sending SIGTERM"
                )
                self._bag_process.send_signal(signal.SIGTERM)
                try:
                    self._bag_process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    self.get_logger().warning(
                        "rosbag2 still running after SIGTERM; sending SIGKILL"
                    )
                    self._bag_process.kill()
                    self._bag_process.wait(timeout=5)
        if self._bag_stdout:
            self._bag_stdout.close()
        if self._bag_stderr:
            self._bag_stderr.close()

    def _start_throttles_and_compressors(self) -> None:
        image_rate = self.config.image_throttle_hz
        other_rate = self.config.other_throttle_hz

        for topic in self.config.topics:
            if topic.mode == "image":
                # Subscribe directly to compressed images; upstream is expected to publish them.
                source_topic = f"{topic.name}/compressed"
                topic.type = "sensor_msgs/msg/CompressedImage"

                if image_rate is None or image_rate <= 0:
                    topic.name = source_topic
                    self.get_logger().info(
                        f"Skipping image throttle for {source_topic} (rate <= 0)"
                    )
                else:
                    throttled_topic = f"{source_topic}_throttled"
                    if self._spawn_throttle(source_topic, throttled_topic, image_rate):
                        topic.name = throttled_topic
                    else:
                        topic.name = source_topic
                        self.get_logger().warning(
                            f"Using unthrottled image topic {source_topic} (throttle helper failed)"
                        )
            else:
                source_topic = topic.name
                if other_rate is None or other_rate <= 0:
                    topic.name = source_topic
                    self.get_logger().info(
                        f"Skipping throttle for {source_topic} (rate <= 0)"
                    )
                else:
                    throttled_topic = f"{source_topic}_throttled"
                    if self._spawn_throttle(source_topic, throttled_topic, other_rate):
                        topic.name = throttled_topic
                    else:
                        topic.name = source_topic
                        self.get_logger().warning(
                            f"Using unthrottled topic {source_topic} (throttle helper failed)"
                        )

    def _spawn_throttle(self, input_topic: str, output_topic: str, rate: float) -> bool:
        cmd = [
            "ros2",
            "run",
            "topic_tools",
            "throttle",
            "messages",
            input_topic,
            str(rate),
            output_topic,
        ]
        ok, _ = self._spawn_aux_process(cmd, desc=f"throttle {input_topic} -> {output_topic} @ {rate} Hz")
        return ok

    def _spawn_aux_process(self, cmd: list[str], desc: str, required: bool = False) -> tuple[bool, Path]:
        log_dir = self._session_dir / "logs" if self._session_dir else Path(".")
        log_dir.mkdir(parents=True, exist_ok=True)
        slug = "".join(ch if ch.isalnum() else "_" for ch in desc)[:80] or "helper"
        log_path = log_dir / f"{slug}.log"

        try:
            with log_path.open("w", encoding="utf-8") as log_file:
                proc = subprocess.Popen(
                    cmd,
                    env=os.environ.copy(),
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                )
            self._aux_processes.append(proc)
            self.get_logger().info(
                f"Started helper: {desc} (pid {proc.pid}); logs: {log_path}"
            )

            # Briefly wait to ensure the helper stays alive; if it exits, treat as failure.
            threading.Event().wait(1.0)
            if proc.poll() is not None:
                msg = f"Helper '{desc}' exited early with code {proc.returncode}; see {log_path}"
                if required:
                    self.get_logger().error(msg)
                else:
                    self.get_logger().warning(msg)
                return False, log_path

            return True, log_path
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f"Failed to start helper '{desc}': {exc}; see {log_path}"
            )
            return False, log_path

    def _start_internal_viewers(self) -> None:
        if not self.config.viewer_topics:
            return

        for topic in self.config.viewer_topics:
            sub = self.create_subscription(
                Image,
                topic,
                self._make_image_callback(topic),
                10,
            )
            self._viewer_subscriptions.append(sub)
            self.get_logger().info(f"Viewer subscribed to {topic}")

        # Render at ~20 Hz to keep UI responsive without heavy CPU use.
        self._viewer_timer = self.create_timer(0.05, self._render_viewers)

    def _cleanup_existing_viewers(self) -> None:
        # Best-effort: close any lingering showimage windows/processes from prior runs.
        wmctrl_bin = shutil.which("wmctrl")
        if wmctrl_bin:
            try:
                subprocess.run(
                    [wmctrl_bin, "-c", "showimage"],
                    check=False,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except Exception:
                pass
        # Also try pkill as a fallback.
        subprocess.run(
            ["pkill", "-f", "image_tools showimage"],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    def _arrange_viewer_windows(self) -> None:
        # No-op: internal OpenCV viewer handles layout.
        return

    def _make_image_callback(self, topic: str):
        def _cb(msg: Image) -> None:
            try:
                frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                self._viewer_frames[topic] = frame
            except Exception as exc:  # noqa: BLE001
                self.get_logger().debug(f"Failed to decode image for {topic}: {exc}")

        return _cb

    def _render_viewers(self) -> None:
        if not self._viewer_frames:
            return

        # Arrange windows side-by-side at smaller size.
        positions = [(100, 100), (100, 500)]
        width, height = 640, 360

        for idx, (topic, frame) in enumerate(self._viewer_frames.items()):
            if frame is None:
                continue
            resized = cv2.resize(frame, (width, height)) if frame is not None else frame
            window_name = f"viewer: {topic}"
            cv2.imshow(window_name, resized)
            x, y = positions[idx] if idx < len(positions) else (idx * 240, 0)
            cv2.moveWindow(window_name, x, y)

        # Needed for imshow to update.
        cv2.waitKey(1)

    def _stop_viewers(self) -> None:
        if self._viewer_timer:
            self._viewer_timer.cancel()
            self._viewer_timer = None
        self._viewer_subscriptions.clear()
        self._viewer_frames.clear()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

    def _stop_aux_processes(self) -> None:
        for proc in self._aux_processes:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.terminate()
                    try:
                        proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        proc.kill()
        self._aux_processes.clear()

    def _confirm_save(self) -> bool:
        if not sys.stdin.isatty():
            self.get_logger().info("Non-interactive stdin; keeping captured data by default.")
            return True

        prompt = f"Save captured data in {self._session_dir}? [Y/n]: "
        try:
            response = input(prompt)
        except EOFError:
            return True

        return response.strip().lower() not in {"n", "no"}

    def _confirm_keep_for_zero(self) -> bool:
        """Prompt whether to keep data when stop key 0 was used (default discard)."""
        if not sys.stdin.isatty():
            self.get_logger().info(
                "Non-interactive stdin; defaulting to discard for stop key 0."
            )
            return False

        prompt = "Keep captured data for score 0? [y/N]: "
        try:
            response = input(prompt)
        except EOFError:
            return False

        return response.strip().lower() in {"y", "yes"}

    def _record_stop_time_if_missing(self) -> None:
        if self._stop_time_ns is None:
            try:
                self._stop_time_ns = self.get_clock().now().nanoseconds
            except Exception:
                # Fallback to wall time in nanoseconds if ROS clock is unavailable
                self._stop_time_ns = int(datetime.now().timestamp() * 1e9)

    def _record_snapped_time(self) -> None:
        if self._snapped_time_ns is not None:
            return

        try:
            snapped_time_ns = self.get_clock().now().nanoseconds
        except Exception:
            snapped_time_ns = int(datetime.now().timestamp() * 1e9)

        self._snapped_time_ns = snapped_time_ns
        self.get_logger().info(f"Snapped time recorded at {snapped_time_ns} ns.")

        if not self._session_dir:
            return

        snapped_file = self._session_dir / "snapped_time.txt"
        try:
            snapped_file.write_text(f"{snapped_time_ns}\n", encoding="utf-8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"Failed to write snapped_time.txt: {exc}")

        config_copy = self._session_dir / "config.yaml"
        if config_copy.exists():
            try:
                with config_copy.open("a", encoding="utf-8") as config_file:
                    config_file.write(f"\nsnapped_time_ns: {snapped_time_ns}\n")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f"Failed to append snapped_time to config.yaml: {exc}")


def main() -> None:
    rclpy.init()
    node = DataCaptureNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
