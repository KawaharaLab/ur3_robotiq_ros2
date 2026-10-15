#!/usr/bin/env python3
"""ROS 2 node that publishes GelSight Mini frames as sensor_msgs/Image messages."""

from __future__ import annotations

import argparse
import json
import time
import threading
import uuid
from typing import Optional

import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge

from .config import GSConfig, ConfigModel
from .utilities.gelsightmini import GelSightMini
from .timing_diagnostics import TimingWindow


class GelSightMiniPublisher(Node):
    def __init__(
        self,
        config: ConfigModel,
        topic_name: str,
        publish_rate_hz: float,
        frame_id: str,
        device_index: Optional[int] = None,
        device_path: Optional[str] = None,
        publish_compressed: bool = True,
        compressed_quality: int = 90,
        timing_diagnostics: bool = False,
        acquisition_records: bool = False,
    ) -> None:
        super().__init__("gelsight_mini_publisher")
        self._bridge = CvBridge()
        self._topic_name = topic_name
        self._frame_id = frame_id
        self._publish_rate_hz = publish_rate_hz if publish_rate_hz > 0 else 15.0
        self._publish_compressed = publish_compressed
        self._compressed_quality = max(1, min(100, compressed_quality))

        self._acquisition_publisher = None
        self._acquisition_stream_id = uuid.uuid4().hex if acquisition_records else None
        self._acquisition_publish_errors = 0
        self._acquisition_backend = "unknown"
        if acquisition_records:
            self._acquisition_publisher = self.create_publisher(String, "~/acquisition_timing", 100)
        self._timing = TimingWindow() if timing_diagnostics else None
        self._previous_read_end_ns = None
        self._diagnostics_publisher = None
        if self._timing is not None:
            self._diagnostics_publisher = self.create_publisher(
                String, "~/acquisition_diagnostics", 10
            )
            self._diagnostics_timer = self.create_timer(1.0, self._report_timing)

        self._cam_stream = GelSightMini(
            target_width=config.camera_width,
            target_height=config.camera_height,
            border_fraction=config.border_fraction,
            timing_diagnostics=timing_diagnostics,
            acquisition_clock=self._acquisition_clock if acquisition_records else None,
        )

        target_device = device_index if device_index is not None else config.default_camera_index
        log_target = device_path if device_path else target_device
        self.get_logger().info(f"Opening GelSight Mini camera (device: {log_target})")
        self._cam_stream.select_device(target_device, device_path=device_path)
        self._cam_stream.start()
        if acquisition_records and self._cam_stream.camera is not None:
            try:
                self._acquisition_backend = self._cam_stream.camera.cap.getBackendName()
            except Exception:
                pass

        self._publisher = self.create_publisher(Image, topic_name, 10)
        self._compressed_publisher: Optional[rclpy.publisher.Publisher] = None
        if self._publish_compressed:
            self._compressed_publisher = self.create_publisher(
                CompressedImage, f"{topic_name}/compressed", 10
            )
            
        # タイマーを廃止し、専用の読み取りスレッドを開始
        self._running = True
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()

    def _capture_loop(self) -> None:
        period = 1.0 / self._publish_rate_hz
        last_publish_time = 0.0

        while rclpy.ok() and self._running:
            # OpenCV側でフレームが届くまでブロックする（バッファを空に保つ）
            frame = self._cam_stream.update(0.0)
            if frame is None:
                if self._timing is not None:
                    self._timing.add(counters={"read_failures": 1})
                continue
            if self._timing is not None:
                measured = self._cam_stream.last_frame_timing
                read_end_ns = measured["read_end_monotonic_ns"]
                metrics = {key: value for key, value in measured.items() if key.endswith("_ms")}
                if self._previous_read_end_ns is not None:
                    metrics["read_return_interval_ms"] = (read_end_ns - self._previous_read_end_ns) / 1e6
                self._previous_read_end_ns = read_end_ns
                self._timing.add(counters={"frames_read": 1}, metrics=metrics)

            current_time = time.time()
            
            # 指定された publish_rate_hz の間隔に達していなければ、画像を読み捨てる
            if (current_time - last_publish_time) < period:
                if self._timing is not None:
                    self._timing.add(counters={"frames_throttled": 1})
                continue

            # 画像が届いた直後の時間をROSタイムスタンプにする
            if self._timing is not None:
                message_start_ns = time.monotonic_ns()
            msg = self._bridge.cv2_to_imgmsg(frame, encoding="rgb8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._frame_id
            if self._timing is not None:
                header_done_ns = time.monotonic_ns()
            self._publisher.publish(msg)
            if self._timing is not None:
                raw_done_ns = time.monotonic_ns()
                self._timing.add(counters={"raw_publish_calls": 1}, metrics={
                    "message_prepare_ms": (header_done_ns - message_start_ns) / 1e6,
                    "read_end_to_header_ms": (header_done_ns - read_end_ns) / 1e6,
                    "raw_publish_ms": (raw_done_ns - header_done_ns) / 1e6,
                })

            compressed_published = False
            if self._compressed_publisher:
                try:
                    if self._timing is not None:
                        encode_start_ns = time.monotonic_ns()
                    encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), self._compressed_quality]
                    bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    ok, buffer = cv2.imencode('.jpg', bgr, encode_params)
                    if self._timing is not None:
                        encode_end_ns = time.monotonic_ns()
                        self._timing.add(metrics={"jpeg_prepare_encode_ms": (encode_end_ns - encode_start_ns) / 1e6})
                    if not ok:
                        if self._timing is not None:
                            self._timing.add(counters={"jpeg_failures": 1})
                        self.get_logger().warn(
                            "Failed to JPEG-encode GelSight frame", throttle_duration_sec=2.0
                        )
                        self._publish_acquisition_record(msg, False)
                        continue
                    cmsg = CompressedImage()
                    cmsg.header = msg.header
                    cmsg.format = 'jpeg'
                    cmsg.data = buffer.tobytes()
                    self._compressed_publisher.publish(cmsg)
                    compressed_published = True
                    if self._timing is not None:
                        self._timing.add(counters={"compressed_publish_calls": 1}, metrics={
                            "compressed_message_publish_ms": (time.monotonic_ns() - encode_end_ns) / 1e6,
                        })
                except Exception as exc:  # noqa: BLE001
                    if self._timing is not None:
                        self._timing.add(counters={"jpeg_failures": 1})
                    self.get_logger().warn(
                        f"Compressed publish failed: {exc}", throttle_duration_sec=2.0
                    )
            
            self._publish_acquisition_record(msg, compressed_published)
            last_publish_time = current_time

    def _acquisition_clock(self):
        # Clock read is bracketed by monotonic readings: do not assume zero skew.
        before = time.monotonic_ns()
        ros_ns = self.get_clock().now().nanoseconds
        return [before, ros_ns, time.monotonic_ns()]

    def _publish_acquisition_record(self, msg, compressed_published):
        if self._acquisition_publisher is None:
            return
        stamp = msg.header.stamp
        record = {
            "schema_version": 1, "kind": "gelsight_frame",
            "stream_id": self._acquisition_stream_id,
            "frame_seq": self._cam_stream.frame_count,
            "header_ns": stamp.sec * 1_000_000_000 + stamp.nanosec,
            "image_topic": self._topic_name, "frame_id": self._frame_id,
            "opencv_version": cv2.__version__, "backend": self._acquisition_backend,
            "clock_type": str(self.get_clock().clock_type),
            "use_sim_time": bool(self.get_parameter("use_sim_time").value),
            "host_monotonic_clock": "time.monotonic_ns",
            "clock_sample_layout": ["monotonic_before_ns", "ros_ns", "monotonic_after_ns"],
            "stages": self._cam_stream.last_acquisition_timing,
            "compressed_published": compressed_published,
            "record_publish_errors_total": self._acquisition_publish_errors,
        }
        try:
            self._acquisition_publisher.publish(String(data=json.dumps(record, allow_nan=False)))
        except Exception as exc:  # additional metadata must not stop image acquisition
            self._acquisition_publish_errors += 1
            self.get_logger().error(f"Acquisition timing publish failed: {exc}", throttle_duration_sec=2.0)

    def _report_timing(self) -> None:
        report = self._timing.snapshot()
        report.update({
            "schema_version": 1, "kind": "gelsight", "node": self.get_name(),
            "frame_id": self._frame_id, "image_topic": self._topic_name,
            "report_ros_ns": self.get_clock().now().nanoseconds,
            "note": "Host-side durations. OpenCV read includes waiting and decoding; no exposure timestamp or calibrated sensor latency.",
        })
        self._diagnostics_publisher.publish(String(data=json.dumps(report)))

    def destroy_node(self) -> None:
        self.get_logger().info("Shutting down GelSight Mini publisher")
        self._running = False
        if hasattr(self, "_capture_thread") and self._capture_thread is not None:
            self._capture_thread.join(timeout=1.0)
            
        if self._cam_stream and self._cam_stream.camera:
            self._cam_stream.camera.release()
        super().destroy_node()


def _str2bool(val: str) -> bool:
    return str(val).lower() in {"1", "true", "t", "yes", "y"}


def parse_arguments() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description="Publish GelSight Mini tactile images to a ROS 2 topic."
    )
    parser.add_argument(
        "--gs-config",
        type=str,
        default="config/default_config.json",
        help="Path to the GelSight configuration JSON file.",
    )
    parser.add_argument(
        "--device-index",
        type=int,
        default=None,
        help="Camera index override. Falls back to default_camera_index from config.",
    )
    parser.add_argument(
        "--device-path",
        type=str,
        default=None,
        help="Explicit device path (e.g., /dev/v4l/by-id/...). Overrides device index on Linux.",
    )
    parser.add_argument(
        "--topic-name",
        type=str,
        default="gelsight/image_raw",
        help="ROS 2 topic to publish Image messages to.",
    )
    parser.add_argument(
        "--frame-id",
        type=str,
        default="gelsight_mini",
        help="Frame ID attached to the published Image messages.",
    )
    parser.add_argument(
        "--publish-rate",
        type=float,
        default=15.0,
        help="Desired publication rate in Hz.",
    )
    parser.add_argument(
        "--publish-compressed",
        type=_str2bool,
        default=True,
        help="Whether to publish JPEG-compressed images at <topic-name>/compressed (default: true).",
    )
    parser.add_argument(
        "--compressed-quality",
        type=int,
        default=90,
        help="JPEG quality (1-100) for compressed output.",
    )

    parser.add_argument(
        "--timing-diagnostics", type=_str2bool, default=False,
        help="Publish optional 1 s timing summaries; does not change image header semantics.",
    )
    parser.add_argument(
        "--acquisition-records", type=_str2bool, default=False,
        help="Publish per-image acquisition timing, separate from the original image header.",
    )
    return parser.parse_known_args()


def main() -> None:
    args, ros_args = parse_arguments()
    gs_config = GSConfig(args.gs_config)

    rclpy.init(args=ros_args)
    node = GelSightMiniPublisher(
        config=gs_config.config,
        topic_name=args.topic_name,
        publish_rate_hz=args.publish_rate,
        frame_id=args.frame_id,
        device_index=args.device_index,
        device_path=args.device_path,
        publish_compressed=args.publish_compressed,
        compressed_quality=args.compressed_quality,
        timing_diagnostics=args.timing_diagnostics,
        acquisition_records=args.acquisition_records,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()