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

import rclpy
from rclpy.node import Node
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
        self._bag_process: Optional[subprocess.Popen] = None
        self._bag_stdout = None
        self._bag_stderr = None
        self._session_dir: Optional[Path] = None
        self._bag_uri: Optional[Path] = None
        self._user_thread = threading.Thread(
            target=self._watch_user_input, daemon=True
        )

    def run(self) -> None:
        self._prepare_session()
        self._start_rosbag()
        self._user_thread.start()
        self.get_logger().info(
            "Recording in progress. Use Ctrl+C or the configured stop key to finish."
        )
        try:
            while rclpy.ok() and not self._stop_event.is_set():
                rclpy.spin_once(self, timeout_sec=0.2)
        except KeyboardInterrupt:
            self.get_logger().info("Keyboard interrupt received, stopping session.")
            self._stop_event.set()
        finally:
            self._finalize()

    def _prepare_session(self) -> None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        session_dir = self.config.output_root / timestamp
        session_dir.mkdir(parents=True, exist_ok=True)
        (session_dir / "bag").mkdir(exist_ok=True)
        (session_dir / "logs").mkdir(exist_ok=True)
        shutil.copy2(self.config.source_path, session_dir / "config.yaml")

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
        key = self.config.stop_key
        self.get_logger().info(
            f"Press '{key}' followed by Enter to stop recording gracefully."
        )
        while not self._stop_event.is_set():
            try:
                line = sys.stdin.readline()
            except Exception:
                break
            if not line:
                continue
            if line.strip() == key:
                self.get_logger().info("Stop key received.")
                self._stop_event.set()
                break

    def _finalize(self) -> None:
        self._stop_rosbag()
        if self._session_dir and self._bag_uri and self._bag_uri.exists():
            try:
                convert_bag_to_dataset(
                    self._bag_uri,
                    self._session_dir,
                    self.config,
                    logger=self.get_logger(),
                )
                self.get_logger().info(
                    f"Capture complete. Dataset stored in {self._session_dir}"
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"Failed to convert bag: {exc}")
        else:
            self.get_logger().error("Bag output not found; skipping conversion.")

    def _stop_rosbag(self) -> None:
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
                self._bag_process.wait(timeout=10)
        if self._bag_stdout:
            self._bag_stdout.close()
        if self._bag_stderr:
            self._bag_stderr.close()


def main() -> None:
    rclpy.init()
    node = DataCaptureNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
