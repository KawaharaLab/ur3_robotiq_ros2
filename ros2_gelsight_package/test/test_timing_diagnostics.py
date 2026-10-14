"""Test bounded statistics and actual acquisition paths without camera/ROS I/O."""
import ast
import json
from pathlib import Path
import types
import unittest
from typing import Optional

from ros2_gelsight_package.timing_diagnostics import TimingWindow


PACKAGE = Path(__file__).resolve().parents[1] / "ros2_gelsight_package"


def load_class(path, name, environment):
    parsed = ast.parse(path.read_text())
    statement = next(item for item in parsed.body if isinstance(item, ast.ClassDef) and item.name == name)
    module = ast.Module(body=[statement], type_ignores=[])
    exec(compile(module, str(path), "exec"), environment)
    return environment[name]


class FakeTime:
    def __init__(self):
        self.tick = 0
    def monotonic_ns(self):
        self.tick += 1_000_000
        return self.tick
    def time(self):
        return 100.0


class WindowTests(unittest.TestCase):
    def test_mean_min_max_and_reset_with_actual_window_duration(self):
        ticks = iter([0, 2_000_000_000, 3_000_000_000])
        window = TimingWindow(clock=lambda: next(ticks))
        window.add(counters={"frames": 2}, metrics={"read_ms": 1})
        window.add(counters={"frames": 1}, metrics={"read_ms": 5})
        snapshot = window.snapshot()
        self.assertEqual(snapshot["rates_hz"]["frames"], 1.5)
        self.assertEqual(snapshot["metrics"]["read_ms"], {"count": 2, "mean": 3.0, "min": 1, "max": 5})
        self.assertEqual(window.snapshot()["counters"], {})
        self.assertEqual(snapshot["counters"]["frames"], 3)

    def test_nonfinite_metrics_are_not_serialized_as_nan(self):
        window = TimingWindow()
        window.add(metrics={"a": float("nan"), "b": float("inf")})
        self.assertEqual(window.snapshot()["metrics"], {})


class AcquisitionPathTests(unittest.TestCase):
    def test_mini_update_keeps_frame_transform_and_no_timing_calls_when_disabled(self):
        outputs = []
        for enabled in (False, True):
            clock = FakeTime()
            environment = {"time": clock, "Optional": Optional, "MatLike": object,
                           "Camera": object, "log_message": lambda msg: None,
                           "cv2": types.SimpleNamespace(COLOR_BGR2RGB=1,
                               cvtColor=lambda image, code: ("rgb", image)),
                           "crop_and_resize": lambda **kwargs: (kwargs["image"], kwargs["target_size"], kwargs["border_fraction"])}
            cls = load_class(PACKAGE / "utilities/gelsightmini.py", "GelSightMini", environment)
            node = cls(timing_diagnostics=enabled)
            node.camera = types.SimpleNamespace(read_frame=lambda: "frame")
            outputs.append(node.update(0.0))
            self.assertEqual(node.frame_count, 1)
            if enabled:
                self.assertEqual(node.last_frame_timing["read_block_decode_ms"], 1.0)
                self.assertEqual(node.last_frame_timing["preprocess_ms"], 1.0)
            else:
                self.assertEqual(clock.tick, 0)
        self.assertEqual(outputs[0], outputs[1])

    def publisher(self, enabled, compressed=True, encode_ok=True, throttle=False):
        clock = FakeTime()
        class Message:
            def __init__(self):
                self.header = types.SimpleNamespace(stamp=None, frame_id=None)
        class Buffer:
            def tobytes(self):
                return b"jpeg"
        environment = {"time": clock, "Optional": Optional, "Node": object,
                       "ConfigModel": object, "json": json, "rclpy": types.SimpleNamespace(ok=lambda: True),
                       "CompressedImage": Message, "String": lambda **kwargs: types.SimpleNamespace(**kwargs),
                       "cv2": types.SimpleNamespace(IMWRITE_JPEG_QUALITY=1, COLOR_RGB2BGR=2,
                           cvtColor=lambda frame, code: frame, imencode=lambda *args: (encode_ok, Buffer()))}
        cls = load_class(PACKAGE / "ros2_gelsight_publisher.py", "GelSightMiniPublisher", environment)
        node = cls.__new__(cls)
        node._timing = TimingWindow() if enabled else None
        node._previous_read_end_ns = None
        node._publish_rate_hz = 25
        node._frame_id = "left"
        node._compressed_quality = 90
        node._running = True
        raw, jpeg = [], []
        node._bridge = types.SimpleNamespace(cv2_to_imgmsg=lambda *args, **kwargs: Message())
        node._publisher = types.SimpleNamespace(publish=raw.append)
        node._compressed_publisher = types.SimpleNamespace(publish=jpeg.append) if compressed else None
        node.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: "original_stamp"))
        node.get_logger = lambda: types.SimpleNamespace(warn=lambda *args, **kwargs: None)
        def update(dt):
            node._running = False
            if throttle:
                clock.time = lambda: 0.0
            return "unchanged_pixels"
        node._cam_stream = types.SimpleNamespace(update=update, last_frame_timing={
            "read_end_monotonic_ns": 0, "read_block_decode_ms": 53.3, "preprocess_ms": 2.0})
        node._capture_loop()
        return node, raw, jpeg, clock

    def test_publishing_preserves_header_and_raw_compressed_relationship(self):
        for enabled in (False, True):
            node, raw, jpeg, clock = self.publisher(enabled)
            self.assertEqual(len(raw), 1)
            self.assertEqual(len(jpeg), 1)
            self.assertEqual(raw[0].header.stamp, "original_stamp")
            self.assertIs(jpeg[0].header, raw[0].header)
            if enabled:
                counters = node._timing.snapshot()["counters"]
                self.assertEqual(counters["frames_read"], 1)
                self.assertEqual(counters["compressed_publish_calls"], 1)
            else:
                self.assertEqual(clock.tick, 0)

    def test_jpeg_failure_is_counted_without_undoing_raw_publish(self):
        node, raw, jpeg, _ = self.publisher(True, encode_ok=False)
        self.assertEqual(len(raw), 1)
        self.assertEqual(len(jpeg), 0)
        self.assertEqual(node._timing.snapshot()["counters"]["jpeg_failures"], 1)

    def test_compression_disabled_has_no_compressed_publish(self):
        node, raw, jpeg, _ = self.publisher(True, compressed=False)
        self.assertEqual(len(raw), 1)
        self.assertEqual(jpeg, [])
        self.assertNotIn("compressed_publish_calls", node._timing.snapshot()["counters"])

    def test_throttled_frame_is_counted_and_not_published(self):
        node, raw, jpeg, _ = self.publisher(True, throttle=True)
        self.assertEqual(raw, [])
        self.assertEqual(jpeg, [])
        self.assertEqual(node._timing.snapshot()["counters"]["frames_throttled"], 1)


if __name__ == "__main__":
    unittest.main()
