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
from std_msgs.msg import String, Int32  # 追加: 通信用メッセージ
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError

from .bag_converter import convert_bag_to_dataset
from .config import CaptureConfig, load_capture_config


class DataCaptureNode(Node):
    """Coordinates rosbag2 recording and dataset conversion."""

    def __init__(self) -> None:
        super().__init__("data_capture_manager")
        # --- 既存の初期化処理 ---
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

        # --- C++ノード連携用の設定 ---
        self._cmd_pub = self.create_publisher(String, "/robot_cmd", 10)
        self._phase_sub = self.create_subscription(Int32, "/current_phase", self._phase_cb, 10)
        self._sequence_finished_event = threading.Event()
        self._current_phase = 0
        self._is_active_session = False

        # --- 既存のメンバ変数 ---
        self._session_id_override = (
            self.declare_parameter("session_id", "").get_parameter_value().string_value
        )
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
        self._bridge = CvBridge()
        self._convert_after_record: bool = (
            self.declare_parameter("convert_after_record", False).value
        )
        self._user_thread = threading.Thread(
            target=self._watch_user_input, daemon=True
        )

    def _phase_cb(self, msg: Int32) -> None:
        """C++側からのフェーズ情報を受信."""
        self._current_phase = msg.data
        # 動作中（Phase > 0）から待機（Phase == 0）に変わった瞬間にイベントをセット
        if self._is_active_session and self._current_phase == 0:
            self._sequence_finished_event.set()

    def run(self) -> None:
        """メインの実行ループ."""
        self._cleanup_existing_viewers() #
        self._start_internal_viewers() #
        self._start_throttles_and_compressors() #
        
        self._user_thread.start()
        self.get_logger().info("Data Capture Node Ready. Press 'i' to init or 's' to start sequence.")
        
        try:
            while rclpy.ok() and not self._stop_event.is_set():
                rclpy.spin_once(self, timeout_sec=0.1)
        finally:
            self._finalize_all()

    def _watch_user_input(self) -> None:
        while not self._stop_event.is_set():
            print("\n" + "="*40)
            print(" [i]: Init Arm (Move to start pose)")
            print(" [s]: Start Trial (Record & Run)")
            print(" [q]: Quit")
            print("="*40)
            
            line = sys.stdin.readline().strip().lower()
            if not line: continue

            if line == 'i':
                self.get_logger().info("Sending 'init' command...")
                self._is_active_session = True # 完了検知を有効化
                self._sequence_finished_event.clear()
                self._cmd_pub.publish(String(data="init"))
                
                self.get_logger().info("Moving to init pose... Please wait.")
                self._sequence_finished_event.wait() # 動作完了までブロック
                self._is_active_session = False
                self.get_logger().info("Init pose reached.")

            elif line == 's':
                self._execute_trial() # 録画ありの施行
            
            elif line == 'q':
                self._stop_event.set()
                break

    def _execute_trial(self) -> None:
        """1回の施行（録画・動作・スコアリング）を実行."""
        self.get_logger().info("Starting new trial...")
        
        # 1. 準備と録画開始
        self._prepare_session() #
        self._start_rosbag() #
        
        # 2. ロボット動作開始命令
        self._is_active_session = True
        self._sequence_finished_event.clear()
        self._cmd_pub.publish(String(data="run"))
        
        # 3. 動作完了（Phase 0）を待機
        self.get_logger().info("Sequence in progress. Waiting for completion...")
        self._sequence_finished_event.wait()
        
        # 4. 録画停止
        self._record_stop_time_if_missing() #
        self._stop_rosbag() #
        self._is_active_session = False
        
        # 5. スコア入力待ち
        print("\nTrial finished. Enter score (1 or 2 to save, 0 to discard): ")
        score_line = sys.stdin.readline().strip()
        if score_line in {"0", "1", "2"}:
            self._score = score_line
            self._force_save = (score_line != "0")
            self._finalize_session()
        else:
            self.get_logger().warning("Invalid score. Discarding trial.")
            self._discard_session()

    def _finalize_session(self) -> None:
        """セッションごとのデータ保存処理."""
        # 元の _finalize() のロジックをベースにセッション保存
        if self._score is not None and self._session_dir:
            (self._session_dir / "score.txt").write_text(f"{self._score}\n")
            
        if self._force_save:
            self.get_logger().info(f"Saving session to {self._session_dir}")
            if self._convert_after_record:
                # 必要に応じて変換実行
                pass 
        else:
            self._discard_session()
        
        # 次の施行のためにリセット
        self._score = None
        self._force_save = False

    def _discard_session(self) -> None:
        """現在のセッションデータを破棄."""
        if self._session_dir and self._session_dir.exists():
            shutil.rmtree(self._session_dir, ignore_errors=True)
            self.get_logger().info("Session discarded.")

    def _finalize_all(self) -> None:
        """ノード終了時のクリーンアップ."""
        self._stop_viewers() #
        self._stop_aux_processes() #
        self._cleanup_existing_viewers() #


    def _prepare_session(self) -> None:
        timestamp = self._session_id_override or datetime.now().strftime("%Y%m%d_%H%M%S")
        session_dir = self.config.output_root / self.config.task_name / timestamp
        session_dir.mkdir(parents=True, exist_ok=True)
        (session_dir / "bag").mkdir(exist_ok=True)
        (session_dir / "logs").mkdir(exist_ok=True)
        shutil.copy2(self.config.source_path, session_dir / "config.yaml")
        (session_dir / "prompt.txt").write_text(
            self.config.prompt + "\n", encoding="utf-8"
        )
        (session_dir / "label.txt").write_text(
            self.config.label + "\n", encoding="utf-8"
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
