"""Batch discovery, caching, interruption, triage, and a real no-ROS mini pipeline."""

import importlib.util
import io
import json
import os
from pathlib import Path
import signal
import sqlite3
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
from contextlib import redirect_stdout

REPO = Path(
    os.environ.get(
        "ACQUISITION_BATCH_TEST_REPO", str(Path(__file__).resolve().parents[2])
    )
)
sys.path.insert(0, str(REPO / "data_capture_tools"))
SCRIPT = Path(
    os.environ.get(
        "ACQUISITION_BATCH_TEST_SCRIPT",
        str(REPO / "utils/diagnostics/process_acquisition_batch.py"),
    )
)
spec = importlib.util.spec_from_file_location("acquisition_batch", SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
module.REPO = REPO
from data_capture_tools.object_plan import build_plan


def prepare_dataset(root):
    spec = importlib.util.spec_from_file_location(
        "plot_fixture",
        REPO / "data_capture_tools/test/test_object_acquisition_plots.py",
    )
    fixture_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fixture_module)
    if not fixture_module.HAS_PLOTS:
        raise unittest.SkipTest("Plot dependencies required")
    data = root / "data"
    data.mkdir()
    exports, session, db, epoch = fixture_module.fixture(data)
    old = module.read_json(data / "progress.json")
    plan = build_plan(
        dict(
            depths_mm=[4.0],
            loading_speeds=[0.1],
            repeats=2,
            positions=["base"],
            release_speed=0.1,
        )
    )
    old_rows = old["plan"]["positions"][0]["trials"]
    for row, original in zip(plan["positions"][0]["trials"], old_rows):
        row.update(status=original["status"], attempts=original["attempts"])
    job = dict(job_id="20261112_010000_abcdef01", status="complete", plan=plan)
    folder = data / "object7" / "automatic_jobs"
    folder.mkdir()
    progress = folder / (job["job_id"] + ".json")
    progress.write_text(json.dumps(job))
    connection = sqlite3.connect(db)
    counts = connection.execute(
        "SELECT t.name,t.type,COUNT(m.id) FROM topics t LEFT JOIN messages m ON m.topic_id=t.id GROUP BY t.id"
    ).fetchall()
    connection.close()
    metadata = dict(
        rosbag2_bagfile_information=dict(
            storage_identifier="sqlite3",
            relative_file_paths=["test.db3"],
            message_count=sum(n for _, _, n in counts),
            topics_with_message_count=[
                dict(topic_metadata=dict(name=t, type=typ), message_count=n)
                for t, typ, n in counts
            ],
        )
    )
    (db.parent / "metadata.yaml").write_text(json.dumps(metadata))
    return data, progress, job


class BatchTests(unittest.TestCase):
    def test_latest_incomplete_job_is_reported_without_falling_back(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            folder = root / "object1/automatic_jobs"
            folder.mkdir(parents=True)
            for name, status, mtime in [
                ("old", "complete", 100),
                ("new", "paused", 200),
            ]:
                path = folder / (name + ".json")
                path.write_text(
                    json.dumps(
                        dict(
                            job_id=name,
                            status=status,
                            plan=dict(
                                positions=[dict(trials=[dict(status="complete")])]
                            ),
                        )
                    )
                )
                os.utime(path, (mtime, mtime))
            result = module.discover(root, [])
            self.assertEqual(result[0]["job_id"], "new")
            self.assertEqual(result[0]["status"], "skipped_incomplete")

    def test_existing_object_results_are_preserved(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            original = root / "plots/object7"
            original.mkdir(parents=True)
            (original / "keep.txt").write_text("existing results")
            entry = dict(
                object_id="object7", job_id="job", progress_sha256="hash", stages={}
            )

            def run(command, log, stage, persist):
                stage["status"] = "finished"

            with patch.object(module, "run_child", side_effect=run):
                stage = module.execute_stage(
                    "plot",
                    entry,
                    root / "management",
                    root / "progress.json",
                    {},
                    "source",
                    lambda: None,
                    root / "exports",
                    result_root=root / "plots",
                )
            self.assertEqual(Path(stage["output"]), root / "plots/object7_002")
            self.assertEqual((original / "keep.txt").read_text(), "existing results")

    def test_dated_history_discovers_new_objects_and_does_not_overwrite(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            data = root / "data"
            data.mkdir()
            args = ["--data-root", str(data), "--exports-root", str(root / "exports")]

            def process(entry, *args):
                entry["status"] = "done"

            def entries(names):
                return [
                    dict(object_id=name, job_id="job", status="pending", stages={})
                    for name in names
                ]

            with patch.object(
                module, "discover", return_value=entries(["object1"])
            ), patch.object(
                module, "process_object", side_effect=process
            ), redirect_stdout(
                io.StringIO()
            ):
                self.assertEqual(module.main(args), 0)
            first = next((root / "postprocess").glob("*/batch_state.json"))
            before = first.read_bytes()
            with patch.object(
                module, "discover", return_value=entries(["object1", "object2"])
            ), patch.object(
                module, "process_object", side_effect=process
            ), redirect_stdout(
                io.StringIO()
            ):
                self.assertEqual(module.main(args), 0)
            self.assertEqual(first.read_bytes(), before)
            records = module.history_records(root / "postprocess")
            self.assertEqual(len(records), 2)
            rows = module.history_rows(records)
            self.assertEqual(sum(row["object_id"] == "object2" for row in rows), 1)
            self.assertEqual(
                len(list((root / "postprocess").glob("*/batch_state.json"))), 2
            )
            self.assertIn("object2", (root / "postprocess/history.csv").read_text())

    def test_triage_categories_keep_time_flags_distinct(self):
        self.assertEqual(
            module.category(dict(level="REVIEW", code="trial_coverage_gap")), "時刻確認"
        )
        self.assertEqual(
            module.category(dict(level="REVIEW", code="isolated_force_spike")),
            "波形確認",
        )
        self.assertEqual(
            module.category(dict(level="FAIL", code="raw_wrench_value_mismatch")),
            "優先確認",
        )
        self.assertEqual(
            module.category(
                dict(level="NOTE", code="existing_sensor_missing_but_timing_present")
            ),
            "補完情報",
        )

    def test_one_object_failure_does_not_stop_the_next_object(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            data = root / "data"
            data.mkdir()
            entries = [
                dict(object_id=name, job_id="job", status="pending", stages={})
                for name in ("object1", "object2")
            ]
            called = []

            def process(entry, *args):
                called.append(entry["object_id"])
                if entry["object_id"] == "object1":
                    raise RuntimeError("broken source")
                entry.update(
                    status="done",
                    stages=dict(
                        export=dict(result="complete"),
                        plot=dict(result="plots_generated"),
                    ),
                )

            with patch.object(module, "discover", return_value=entries), patch.object(
                module, "process_object", side_effect=process
            ), redirect_stdout(io.StringIO()):
                result = module.main(
                    [
                        "--data-root",
                        str(data),
                        "--exports-root",
                        str(root / "exports"),
                        "--out",
                        str(root / "batch"),
                    ]
                )
            self.assertEqual(result, 1)
            self.assertEqual(called, ["object1", "object2"])
            self.assertEqual(
                module.read_json(root / "batch/batch_state.json")["objects"][1][
                    "status"
                ],
                "done",
            )

    def test_interrupted_child_is_stopped_and_recorded(self):
        with tempfile.TemporaryDirectory() as tmp:
            stage = {}
            process = NS(
                pid=12345, returncode=None, poll=lambda: None, wait=lambda **kw: 0
            )
            with patch.object(
                module.subprocess, "Popen", return_value=process
            ), patch.object(
                module.time, "sleep", side_effect=KeyboardInterrupt
            ), patch.object(
                module.os, "killpg"
            ) as stop:
                with self.assertRaises(KeyboardInterrupt):
                    module.run_child(["dummy"], Path(tmp) / "log", stage, lambda: None)
            self.assertEqual(stage["status"], "interrupted")
            stop.assert_called_once_with(12345, signal.SIGINT)

    def test_real_pipeline_review_continues_and_resume_reuses_results(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            data, progress, job = prepare_dataset(root)
            args = [
                "--data-root",
                str(data),
                "--exports-root",
                str(root / "exports"),
                "--out",
                str(root / "batch"),
            ]
            before = progress.read_bytes()
            with redirect_stdout(io.StringIO()):
                result = module.main(args)
            state = module.read_json(root / "batch/batch_state.json")
            entry = state["objects"][0]
            self.assertEqual(entry["stages"]["quality"]["result"], "REVIEW", state)
            self.assertEqual(entry["stages"]["export"]["result"], "complete", state)
            self.assertEqual(
                entry["stages"]["plot"]["result"], "plots_generated", state
            )
            self.assertEqual(result, 0, state)
            self.assertEqual(state["status"], "completed_with_findings")
            self.assertEqual(
                Path(entry["stages"]["quality"]["output"]),
                (root / "quality_check/object7").resolve(),
            )
            self.assertEqual(
                Path(entry["stages"]["plot"]["output"]),
                (root / "plots/object7").resolve(),
            )
            pngs = list((root / "plots/object7/trials").glob("*.png"))
            self.assertEqual(len(pngs), 2)
            self.assertTrue((root / "batch/summary.html").is_file())
            self.assertTrue((root / "batch/review_trials.csv").is_file())
            self.assertEqual(progress.read_bytes(), before)
            with patch.object(
                module,
                "run_child",
                side_effect=AssertionError("Completed stages should be reused"),
            ), redirect_stdout(io.StringIO()):
                result = module.main(args + ["--resume"])
            self.assertEqual(result, 0)
            resumed = module.read_json(root / "batch/batch_state.json")["objects"][0]
            self.assertEqual(resumed["stages"]["export"]["result"], "既存出力を再利用")
            self.assertIn("elapsed_seconds", resumed["stages"]["export"])
            export_path = Path(resumed["stages"]["export"]["output"])
            self.assertTrue(
                module.reusable_export(export_path, module.file_hash(progress), job)
            )
            self.assertFalse(module.reusable_export(export_path, "wrong-hash", job))
            # A separate run must reuse all three stages, while preserving the first summary.
            original_summary = (root / "batch/summary.html").read_bytes()
            second_args = args[:-2] + ["--out", str(root / "batch2")]
            with patch.object(
                module,
                "run_child",
                side_effect=AssertionError("New run should reuse unchanged results"),
            ), redirect_stdout(io.StringIO()):
                self.assertEqual(module.main(second_args), 0)
            self.assertEqual(
                (root / "batch/summary.html").read_bytes(), original_summary
            )
            second = module.read_json(root / "batch2/batch_state.json")
            self.assertTrue(
                all(
                    stage.get("reused")
                    for stage in second["objects"][0]["stages"].values()
                )
            )
            self.assertEqual(len(module.history_records(root / "postprocess")), 2)
            self.assertTrue((root / "postprocess/index.html").is_file())
            found = io.StringIO()
            with redirect_stdout(found):
                self.assertEqual(
                    module.main(
                        [
                            "--history-root",
                            str(root / "postprocess"),
                            "--find",
                            "object7",
                            job["job_id"],
                        ]
                    ),
                    0,
                )
            self.assertIn("Matches: 2", found.getvalue())
            (next(export_path.glob("*/force_left.csv"))).unlink()
            self.assertFalse(
                module.reusable_export(export_path, module.file_hash(progress), job)
            )

    def test_reexport_flag_still_reuses_its_own_completed_export_on_resume(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            data, progress, job = prepare_dataset(root)
            args = [
                "--data-root",
                str(data),
                "--exports-root",
                str(root / "exports"),
                "--out",
                str(root / "batch"),
                "--skip-quality",
                "--reexport",
            ]
            with redirect_stdout(io.StringIO()):
                self.assertEqual(module.main(args), 0)
            with patch.object(
                module, "run_child", side_effect=AssertionError("Do not reexport twice")
            ), redirect_stdout(io.StringIO()):
                self.assertEqual(module.main(args + ["--resume"]), 0)
            new_args = list(args)
            new_args[new_args.index("--out") + 1] = str(root / "another_run")
            with redirect_stdout(io.StringIO()):
                self.assertEqual(module.main(new_args), 0)
            next_run = module.read_json(root / "another_run/batch_state.json")
            self.assertFalse(next_run["objects"][0]["stages"]["export"]["reused"])


if __name__ == "__main__":
    unittest.main()
