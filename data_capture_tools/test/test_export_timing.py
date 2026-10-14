"""Timing contract tests; exporter I/O adapters are faked when ROS is absent."""

import csv
import importlib
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

from data_capture_tools.export_timing import (
    ExportTimingIndex, export_is_complete, export_provenance, header_time,
)


def stamped(sec=1, nanosec=23):
    return types.SimpleNamespace(header=types.SimpleNamespace(
        stamp=types.SimpleNamespace(sec=sec, nanosec=nanosec), frame_id="left"))


def read_csv(path):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


class TimingContractTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.config_path = self.root / "config.yaml"
        self.config_path.write_text("sample configuration")
        self.config = types.SimpleNamespace(
            source_path=self.config_path, topics=[types.SimpleNamespace(name="/sensor")])
        self.provenance = export_provenance(self.root / "bag", self.config, (0, None))

    def test_header_integer_precision_and_no_invented_timestamp(self):
        self.assertEqual(header_time(stamped(1_788_527_055, 354_358_985))[0],
                         1_788_527_055_354_358_985)
        self.assertEqual(header_time(object()), (None, "unavailable", ""))
        self.assertEqual(header_time(stamped(0, 0))[1], "zero_header")
        self.assertEqual(header_time(stamped(1, 1_000_000_000))[1], "invalid_header")

    def test_complete_export_keeps_both_clocks_and_checks_artifacts(self):
        out = self.root / "out"
        with ExportTimingIndex(out, self.provenance) as index:
            artifact = out / "sensor.csv"
            artifact.write_text("original CSV content")
            index.record("/sensor", 1_000_000_050, stamped(), artifact, 1)
        rows = read_csv(out / "timing_index.csv")
        self.assertEqual(rows[0]["bag_receive_ns"], "1000000050")
        self.assertEqual(rows[0]["source_stamp_ns"], "1000000023")
        self.assertTrue(export_is_complete(out, self.provenance))
        artifact.write_text("truncated")
        self.assertFalse(export_is_complete(out, self.provenance))

    def test_missing_header_is_not_replaced_by_bag_time(self):
        out = self.root / "out"
        with ExportTimingIndex(out, self.provenance) as index:
            index.record("/sensor", 123, object())
        row = read_csv(out / "timing_index.csv")[0]
        self.assertEqual(row["source_stamp_ns"], "")
        self.assertEqual(row["source_stamp_semantics"], "unavailable")

    def test_exception_and_skipped_message_cannot_mark_complete(self):
        for mode in ("exception", "skipped", "empty"):
            out = self.root / mode
            with self.assertRaises((OSError, RuntimeError)):
                with ExportTimingIndex(out, self.provenance) as index:
                    if mode == "exception":
                        raise OSError("disk failure")
                    if mode == "skipped":
                        index.record("/sensor", 123, None, status="deserialize_failed")
            self.assertFalse(export_is_complete(out, self.provenance))
            self.assertEqual(json.loads((out / "export_manifest.json").read_text())["status"], "failed")

    def test_duplicate_image_name_fails_before_overwrite(self):
        out = self.root / "out"
        with self.assertRaises(ValueError):
            with ExportTimingIndex(out, self.provenance) as index:
                index.claim_image_path(out / "123.png")
                index.claim_image_path(out / "123.png")

    def test_config_change_or_index_corruption_requires_new_export(self):
        out = self.root / "out"
        with ExportTimingIndex(out, self.provenance) as index:
            index.record("/sensor", 123, stamped())
        self.config_path.write_text("changed")
        changed = export_provenance(self.root / "bag", self.config, (0, None))
        self.assertFalse(export_is_complete(out, changed))
        (out / "timing_index.csv").write_text("corrupt")
        self.assertFalse(export_is_complete(out, self.provenance))


class ExporterIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Simulate only transport dependencies; execute the actual exporter functions.
        class CompressedImage:
            def __init__(self):
                self.header = stamped().header
        cls.CompressedImage = CompressedImage
        cls.messages = []

        class Reader:
            def __init__(self):
                self.rows = iter(cls.messages)
                self.pending = None
            def open(self, *args):
                pass
            def has_next(self):
                try:
                    self.pending = next(self.rows)
                    return True
                except StopIteration:
                    return False
            def read_next(self):
                return self.pending

        class Bridge:
            def compressed_imgmsg_to_cv2(self, message):
                return b"decoded"
        cls.imwrite_ok = True
        def imwrite(name, pixels):
            if not cls.imwrite_ok:
                return False
            Path(name).write_bytes(b"exported image")
            return True
        def module(name, **attributes):
            value = types.ModuleType(name)
            value.__dict__.update(attributes)
            return value
        fake = {
            "cv2": module("cv2", imwrite=imwrite),
            "cv_bridge": module("cv_bridge", CvBridge=Bridge),
            "sensor_msgs": module("sensor_msgs"),
            "sensor_msgs.msg": module("sensor_msgs.msg", CompressedImage=CompressedImage, Image=type("Image", (), {})),
            "rclpy": module("rclpy"),
            "rclpy.serialization": module("rclpy.serialization", deserialize_message=lambda data, kind: data),
            "rosbag2_py": module("rosbag2_py", SequentialReader=Reader,
                                 ConverterOptions=lambda **kwargs: kwargs, StorageOptions=lambda **kwargs: kwargs),
            "rosidl_runtime_py": module("rosidl_runtime_py", message_to_ordereddict=lambda msg: msg.fields),
            "rosidl_runtime_py.utilities": module("rosidl_runtime_py.utilities", get_message=lambda name: object),
            "yaml": module("yaml"),
            "data_capture_tools.config": module("data_capture_tools.config", CaptureConfig=object,
                                                TopicSpec=object, load_capture_config=lambda p: None),
        }
        with patch.dict(sys.modules, fake):
            package = Path(__file__).resolve().parents[1] / "data_capture_tools"
            for attribute, name in (("converter", "bag_converter"), ("batch", "extract_batch_bag")):
                fullname = f"data_capture_tools.{name}"
                spec = importlib.util.spec_from_file_location(fullname, package / f"{name}.py")
                loaded = importlib.util.module_from_spec(spec)
                sys.modules[fullname] = loaded
                spec.loader.exec_module(loaded)
                setattr(cls, attribute, loaded)

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        config_path = self.root / "config.yaml"
        config_path.write_text("test")
        self.config = types.SimpleNamespace(
            source_path=config_path, bag=types.SimpleNamespace(storage="sqlite3"),
            topics=[types.SimpleNamespace(name="/image", type="sensor_msgs/msg/CompressedImage", mode="image", encoding="bgr8"),
                    types.SimpleNamespace(name="/force", type="geometry_msgs/msg/WrenchStamped", mode="csv", encoding=None)],
        )
        force = stamped()
        force.fields = {"header": {"stamp": {"sec": 1, "nanosec": 23}}, "value": 2.5}
        self.__class__.messages = [("/image", self.CompressedImage(), 1_000_000_050),
                                   ("/force", force, 1_000_000_060)]
        self.__class__.imwrite_ok = True

    def run_export(self, kind, out):
        if kind == "full":
            with patch.object(self.converter, "_maybe_decompress_file_bag", return_value=(self.root / "bag", None)):
                self.converter.convert_bag_to_dataset(self.root / "bag", out, self.config)
        else:
            self.batch.convert_trial_range(self.root / "bag", out, self.config, 1_000_000_000, 1_000_000_100)

    def test_both_paths_preserve_legacy_outputs_and_record_headers(self):
        for kind in ("full", "trial"):
            out = self.root / kind
            self.run_export(kind, out)
            self.assertTrue((out / "images/image/1000000050.png").is_file())
            legacy = read_csv(out / "csv/force.csv")
            self.assertEqual(legacy[0]["stamp_ns"], "1000000060")
            self.assertEqual(legacy[0]["force.value"], "2.5")
            self.assertNotIn("source_stamp_ns", legacy[0])
            timing = read_csv(out / "timing_index.csv")
            self.assertEqual(len(timing), 2)
            self.assertEqual(timing[0]["source_stamp_ns"], "1000000023")
            self.assertEqual(timing[1]["artifact_row"], "1")
            self.assertEqual(json.loads((out / "export_manifest.json").read_text())["status"], "complete")

    def test_image_write_failure_marks_both_exporters_failed(self):
        self.__class__.imwrite_ok = False
        for kind in ("full", "trial"):
            out = self.root / kind
            with self.assertRaises(OSError):
                self.run_export(kind, out)
            self.assertEqual(json.loads((out / "export_manifest.json").read_text())["status"], "failed")

    def test_column_mapping_cannot_remove_sidecar_source_time(self):
        mapping = {"/force": {"stamp_ns": "stamp_ns", "force.value": "value"}}
        for kind in ("full", "trial"):
            module = self.converter if kind == "full" else self.batch
            with patch.object(module, "_load_csv_field_map", return_value=mapping):
                out = self.root / kind
                self.run_export(kind, out)
            legacy = read_csv(out / "csv/force.csv")
            self.assertEqual(set(legacy[0]), {"stamp_ns", "value"})
            self.assertEqual(read_csv(out / "timing_index.csv")[1]["source_stamp_ns"], "1000000023")

    def test_trial_bounds_keep_legacy_bag_receive_selection(self):
        self.__class__.messages.append(("/force", stamped(), 1_000_000_101))
        out = self.root / "trial"
        self.run_export("trial", out)
        self.assertEqual(len(read_csv(out / "timing_index.csv")), 2)

    def test_deserialization_skip_is_not_a_successful_export(self):
        def broken(data, kind):
            raise ValueError("bad packet")
        for kind in ("full", "trial"):
            module = self.converter if kind == "full" else self.batch
            out = self.root / kind
            with patch.object(module, "deserialize_message", side_effect=broken):
                with self.assertRaises((RuntimeError, ValueError)):
                    self.run_export(kind, out)
            self.assertEqual(json.loads((out / "export_manifest.json").read_text())["status"], "failed")


if __name__ == "__main__":
    unittest.main()
