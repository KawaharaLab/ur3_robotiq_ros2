"""Plot real miniature SQLite/CDR bags without ROS, and preserve every source file."""

import importlib.util
import io
import json
import os
from pathlib import Path
import sqlite3
import struct
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from unittest.mock import patch
import csv

try:
    import cv2
    import numpy as np

    HAS_PLOTS = importlib.util.find_spec("matplotlib") is not None
except ImportError:
    HAS_PLOTS = False

REPO = Path(
    os.environ.get("OBJECT_PLOT_TEST_REPO", str(Path(__file__).resolve().parents[2]))
)
sys.path.insert(0, str(REPO / "data_capture_tools"))
SCRIPT = Path(
    os.environ.get(
        "OBJECT_PLOT_TEST_SCRIPT",
        str(REPO / "utils/diagnostics/plot_object_acquisition.py"),
    )
)
if HAS_PLOTS:
    exporter_path = Path(
        os.environ.get(
            "OBJECT_PLOT_TEST_EXPORTER",
            str(REPO / "data_capture_tools/data_capture_tools/compact_export.py"),
        )
    )
    exporter_spec = importlib.util.spec_from_file_location(
        "data_capture_tools.compact_export", exporter_path
    )
    exporter = importlib.util.module_from_spec(exporter_spec)
    sys.modules[exporter_spec.name] = exporter
    exporter_spec.loader.exec_module(exporter)
    spec = importlib.util.spec_from_file_location("object_plots", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)


class Encoder:
    def __init__(self):
        self.data = bytearray()

    def number(self, fmt, value, alignment=4):
        self.data.extend(b"\0" * ((-len(self.data)) % alignment))
        self.data.extend(struct.pack("<" + fmt, value))
        return self

    def string(self, value):
        raw = value.encode() + b"\0"
        self.number("I", len(raw))
        self.data.extend(raw)
        return self

    def header(self, h):
        return self.number("i", h // 10**9).number("I", h % 10**9).string("sensor")

    def finish(self):
        return b"\0\1\0\0" + self.data


def fixture(root):
    session = root / "object7" / "session"
    bag = session / "bag" / "bag"
    bag.mkdir(parents=True)
    types = {
        **dict.fromkeys(module.GS, "sensor_msgs/msg/CompressedImage"),
        **dict.fromkeys(module.FORCE, "geometry_msgs/msg/WrenchStamped"),
        module.WIDTH: "std_msgs/msg/Float32",
        module.STATUS: "robotiq_2f_gripper_msgs/msg/GripperStatus",
        module.ACTION: "robotiq_2f_gripper_msgs/msg/RobotActionEvent",
        "/trial_event": "robotiq_2f_gripper_msgs/msg/TrialEvent",
    }
    db = bag / "test.db3"
    con = sqlite3.connect(db)
    con.executescript(
        "CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT,type TEXT);"
        "CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB);"
    )
    ids = {t: i for i, t in enumerate(types, 1)}
    con.executemany(
        "INSERT INTO topics VALUES(?,?,?)", [(ids[t], t, v) for t, v in types.items()]
    )

    def add(topic, h, e):
        con.execute(
            "INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)",
            (ids[topic], h + 1_000_000, e.finish()),
        )

    trials = []
    epoch = 1_800_000_000_000_000_000
    for index in (1, 2):
        first = epoch + (index - 1) * 3_000_000_000
        start = first + 500_000_000
        end = first + 2_000_000_000
        for kind, h in [("START", start), ("END", end)]:
            add(
                "/trial_event",
                h,
                Encoder()
                .header(h)
                .string(kind)
                .number("i", index)
                .string("auto_depth_speed"),
            )
        for n in range(2001):
            h = first + n * 1_000_000
            delta = 0.4 if start + 200_000_000 <= h < start + 1_000_000_000 else 0.0
            if h == start + 700_000_000:
                delta += 8.0
            for side, topic in enumerate(module.FORCE):
                e = Encoder().header(h)
                for value in (
                    0.0,
                    (10.0 if side == 0 else -4.0) + delta,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ):
                    e.number("d", value, 8)
                add(topic, h, e)
        for n in range(41):
            h = first + n * 50_000_000
            pixels = np.zeros((24, 24, 3), dtype=np.uint8)
            pixels[:] = n + 20
            jpeg = cv2.imencode(".jpg", pixels)[1].tobytes()
            for topic in module.GS:
                e = Encoder().header(h).string("jpeg").number("I", len(jpeg))
                e.data.extend(jpeg)
                add(topic, h, e)
            add(module.WIDTH, h, Encoder().number("f", 40.0 - n * 0.2))
            e = Encoder().header(h)
            for value in (0, 0, 120, 100 + n, 0):
                e.number("B", value, 1)
            e.number("f", 5.0)
            add(module.STATUS, h, e)
        for phase, h, target in [
            (1, first, 0.04),
            (2, start + 0.05 * 1e9, 0.032),
            (3, start + 1.05 * 1e9, 0.04),
        ]:
            h = int(h)
            for step, event in enumerate(
                (
                    "auto_goal_sent",
                    "auto_action_succeeded",
                    "auto_dwell_start",
                    "auto_dwell_end",
                )
            ):
                stamp = h + step * 20_000_000
                add(
                    module.ACTION,
                    stamp,
                    Encoder()
                    .header(stamp)
                    .string(event)
                    .number("i", index)
                    .number("i", phase)
                    .string("gripper")
                    .number("d", target, 8)
                    .number("d", 0.1, 8)
                    .number("B", 1, 1),
                )
        trials.append(
            dict(
                id=f"base_{index:04}",
                kind="run",
                depth_mm=4.0,
                speed=0.1,
                hold_s=4.0,
                release_speed=0.1,
                repeat=index,
                status="complete",
                attempts=[
                    dict(
                        status="complete",
                        session=str(session),
                        start_ros_ns=first,
                        end_ros_ns=end + 1,
                        tare=dict(left=10.0, right=-4.0),
                    )
                ],
            )
        )
    con.commit()
    con.close()
    (bag / "metadata.yaml").write_text(
        json.dumps(
            dict(
                rosbag2_bagfile_information=dict(
                    storage_identifier="sqlite3", relative_file_paths=["test.db3"]
                )
            )
        )
    )
    job = dict(
        job_id="sample",
        status="complete",
        plan=dict(positions=[dict(id="base", trials=trials)]),
    )
    progress = root / "progress.json"
    progress.write_text(json.dumps(job))
    export_root = root / "export"
    export_root.mkdir()
    target = export_root / session.name
    selected = exporter.selected_attempts(job)[str(session.resolve())]
    exporter.export_session(session, selected, target, write=True)
    (export_root / "source_progress.json").write_text(json.dumps(job))
    return export_root, target, db, epoch


@unittest.skipUnless(HAS_PLOTS, "OpenCV, NumPy and Matplotlib required")
class PlotTests(unittest.TestCase):
    def test_integer_timestamp_precision(self):
        base = 1_800_000_000_000_000_000
        result = module.relative_seconds([base + 1, base + 2], base)
        np.testing.assert_array_equal(result, np.array([1e-9, 2e-9]))

    def test_conditions_include_speed_and_hold_but_ignore_repeat(self):
        condition = dict(
            kind="run", depth_mm=4.0, speed=0.1, hold_s=4.0, release_speed=0.1, repeat=1
        )
        first = module.condition_identity(condition)[1]
        self.assertEqual(first, module.condition_identity(dict(condition, repeat=2))[1])
        self.assertNotEqual(
            first, module.condition_identity(dict(condition, speed=0.2))[1]
        )
        self.assertNotEqual(
            first, module.condition_identity(dict(condition, hold_s=2.0))[1]
        )
        knead = dict(
            kind="knead",
            shallow_mm=2.0,
            deep_mm=6.0,
            cycles=5,
            loading_speed=0.1,
            unloading_speed=0.1,
            deep_hold_s=0.1,
            shallow_hold_s=0.1,
        )
        self.assertNotEqual(
            module.condition_identity(knead)[1],
            module.condition_identity(dict(knead, deep_hold_s=0.5))[1],
        )

    def test_saved_tare_full_rate_spike_and_headerless_width(self):
        with tempfile.TemporaryDirectory() as tmp:
            p, s, db, epoch = fixture(Path(tmp))
            selected = json.loads((s / "manifest.json").read_text())["windows"]
            windows, data, diagnostics = module.read_export_session(s, selected)
            self.assertFalse(diagnostics["read_errors"])
            signals = module.trial_signals(windows[0], data)
            self.assertEqual(signals["forces"]["left"]["baseline"], 10.0)
            self.assertEqual(signals["forces"]["right"]["baseline"], -4.0)
            self.assertEqual(len(signals["forces"]["left"]["values"]), 1501)
            self.assertAlmostEqual(max(signals["forces"]["left"]["values"]), 8.4)
            self.assertEqual(sum(signals["forces"]["left"]["spikes"]), 1)
            self.assertEqual(data[module.WIDTH][0][0], epoch + 1_000_000)
            self.assertEqual(data[module.STATUS][0][1], 100)

    def test_complete_png_generation_preserves_sources(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            p, s, db, _ = fixture(root)
            before = {
                path: path.read_bytes() for path in root.rglob("*") if path.is_file()
            }
            with redirect_stdout(io.StringIO()):
                code = module.main(
                    ["--export-dir", str(p), "--out", str(root / "plots")]
                )
            self.assertEqual(code, 0)
            report = json.loads((root / "plots/report.json").read_text())
            self.assertEqual(report["plotted_trials"], 2)
            self.assertEqual(len(report["comparison_plots"]), 1)
            self.assertEqual(report["force_display"], "signed")
            images = list((root / "plots").rglob("*.png"))
            self.assertEqual(len(images), 3)
            self.assertTrue(
                all(cv2.imread(str(image)).shape[1] > 1000 for image in images)
            )
            for path, content in before.items():
                self.assertEqual(path.read_bytes(), content)
            with self.assertRaises(SystemExit):
                module.main(["--export-dir", str(p), "--out", str(root / "plots")])

    def test_bad_image_reports_incomplete_and_continues(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            p, s, db, epoch = fixture(root)
            with (s / "image_index.csv").open(newline="") as stream:
                first_image = next(csv.DictReader(stream))["artifact_path"]
            (s / first_image).write_bytes(b"broken")
            with redirect_stdout(io.StringIO()):
                code = module.main(
                    [
                        "--export-dir",
                        str(p),
                        "--out",
                        str(root / "plots"),
                        "--max-trials",
                        "1",
                        "--absolute-force",
                    ]
                )
            self.assertEqual(code, 1)
            report = json.loads((root / "plots/report.json").read_text())
            self.assertEqual(report["status"], "incomplete")
            self.assertEqual(report["plotted_trials"], 1)
            self.assertEqual(report["force_display"], "absolute")
            self.assertTrue(report["sessions"][0]["read_errors"])

    def test_conflicting_csv_baseline_is_reported(self):
        with tempfile.TemporaryDirectory() as tmp:
            p, s, db, _ = fixture(Path(tmp))
            path = s / "force_left.csv"
            with path.open(newline="") as stream:
                reader = csv.DictReader(stream)
                fields = reader.fieldnames
                rows = list(reader)
            rows[0]["baseline_Fy"] = "999"
            with path.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                writer.writerows(rows)
            selected = json.loads((s / "manifest.json").read_text())["windows"]
            _, _, diagnostics = module.read_export_session(s, selected)
            self.assertEqual(diagnostics["read_errors"][module.FORCE[0]], 1)
            self.assertIn("disagree", diagnostics["examples"][0]["error"])

    def test_legacy_export_without_auxiliary_csv_is_supported(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            p, s, db, _ = fixture(root)
            for name in exporter.AUXILIARY_FIELDS:
                (s / name).unlink()
            with redirect_stdout(io.StringIO()):
                result = module.main(
                    [
                        "--export-dir",
                        str(p),
                        "--out",
                        str(root / "plots"),
                        "--max-trials",
                        "1",
                    ]
                )
            self.assertEqual(result, 0)
            report = json.loads((root / "plots/report.json").read_text())
            self.assertEqual(len(report["sessions"][0]["missing_auxiliary_csvs"]), 3)
            self.assertTrue(
                any(
                    "Gripper state unavailable" in n
                    for n in report["trials"][0]["notes"]
                )
            )

    def test_original_bag_can_be_absent_and_sqlite_is_never_opened(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            p, s, db, _ = fixture(root)
            db.unlink()
            with patch(
                "sqlite3.connect", side_effect=AssertionError("Plot must not read bags")
            ), redirect_stdout(io.StringIO()):
                result = module.main(
                    [
                        "--export-dir",
                        str(p),
                        "--out",
                        str(root / "plots"),
                        "--max-trials",
                        "1",
                    ]
                )
            self.assertEqual(result, 0)
            report = json.loads((root / "plots/report.json").read_text())
            self.assertFalse(report["original_bags_read"])

    def test_gaps_not_interpolated_or_used_as_sustained_contact(self):
        x, y = module.broken_line(
            np.array([0.0, 0.001, 0.1]), np.array([0.0, 3.0, 0.0]), 0.005
        )
        self.assertTrue(np.isnan(x[2]))
        self.assertTrue(np.isnan(y[2]))
        self.assertIsNone(
            module.sustained_time(np.array([0, 100_000_000]), [True, True])
        )

    def test_missing_tare_is_visible_and_not_reestimated_from_contact(self):
        with tempfile.TemporaryDirectory() as tmp:
            p, s, db, _ = fixture(Path(tmp))
            selected = json.loads((s / "manifest.json").read_text())["windows"]
            windows, data, _ = module.read_export_session(s, selected)
            windows[0]["attempt"]["tare"].pop("left")
            signals = module.trial_signals(windows[0], data)
            self.assertIsNone(signals["forces"]["left"]["baseline"])
            self.assertTrue(
                any("No recorded tare" in note for note in signals["notes"])
            )
            self.assertGreater(max(signals["forces"]["left"]["values"]), 18.0)


if __name__ == "__main__":
    unittest.main()
