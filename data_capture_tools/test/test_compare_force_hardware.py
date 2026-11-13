"""Exact condition matching, no invented hold times, robust candidates and real plots."""

import csv
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
import numpy as np

SCRIPT = (
    Path(__file__).resolve().parents[2] / "utils/diagnostics/compare_force_hardware.py"
)
spec = importlib.util.spec_from_file_location("compare_hardware", SCRIPT)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def fixture(root, job, repeats, force, events=True, spike=False, depth=4.0):
    session = root / "session"
    session.mkdir(parents=True)
    windows = []
    epoch = 1_700_000_000_000_000_000
    condition = dict(
        kind="run",
        depth_mm=depth,
        speed=0.0,
        hold_s=1.0,
        preparation_speed=0.0,
        release_speed=0.0,
        trial_open_clearance_mm=15.0,
    )
    for i in range(repeats):
        start = epoch + i * 3_000_000_000
        windows.append(
            dict(
                trial_id=f"base_{i+1:04d}",
                position_id="base",
                start_ns=start,
                end_ns=start + 2_000_000_000,
                conditions=condition,
                attempt=dict(
                    session="/source/object46/" + job + "_base_session",
                    tare=dict(left=10.0, right=20.0),
                ),
            )
        )
    (session / "manifest.json").write_text(
        json.dumps(dict(status="complete", windows=windows))
    )
    (root / "source_progress.json").write_text(
        json.dumps(
            dict(
                job_id=job,
                plan=dict(
                    settings=dict(hard_limit_n=30.0),
                    positions=[dict(id="base", reference=dict(width_m=0.04))],
                ),
            )
        )
    )
    for side, baseline in [("left", 10.0), ("right", 20.0)]:
        with (session / f"force_{side}.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=[
                    "trial_id",
                    "source_stamp_ns",
                    "Fy",
                    "baseline_Fy",
                    "baseline_relative_Fy",
                ],
            )
            writer.writeheader()
            for w in windows:
                for n in range(2001):
                    value = -force if 300 <= n <= 1600 else 0.0
                    if side == "left" and spike and n == 800:
                        value = -1000.0
                    writer.writerow(
                        dict(
                            trial_id=w["trial_id"],
                            source_stamp_ns=w["start_ns"] + n * 1_000_000,
                            Fy=value + baseline,
                            baseline_Fy=baseline,
                            baseline_relative_Fy=value,
                        )
                    )
    if events:
        with (session / "robot_action_event.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(
                stream, fieldnames=["trial_id", "source_stamp_ns", "phase_id", "event"]
            )
            writer.writeheader()
            for w in windows:
                for name, dt in [
                    ("auto_dwell_start", 500_000_000),
                    ("auto_dwell_end", 1_500_000_000),
                ]:
                    writer.writerow(
                        dict(
                            trial_id=w["trial_id"],
                            source_stamp_ns=w["start_ns"] + dt,
                            phase_id=2,
                            event=name,
                        )
                    )
    return root


class ComparisonTests(unittest.TestCase):
    def test_hampel_masks_impulse_but_not_step_and_does_not_modify_input(self):
        t = np.arange(1000) / 1000.0
        y = np.zeros(1000)
        y[500:] = -8.0
        y[700] = -1000.0
        original = y.copy()
        mask, _, _ = m.hampel(t, y, 21, 6.0, 2.5, 0.005)
        self.assertTrue(mask[700])
        self.assertFalse(mask[500])
        np.testing.assert_array_equal(y, original)

    def test_hampel_does_not_span_time_gaps(self):
        t = np.r_[np.arange(10) / 1000.0, 1.0 + np.arange(10) / 1000.0]
        y = np.zeros(20)
        y[9] = 1000
        mask, _, _ = m.hampel(t, y, 21, 6.0, 2.5, 0.005)
        self.assertFalse(mask.any())

    def test_condition_matching_ignores_repeat_not_speed_release_or_position(self):
        a = dict(
            position_id="base",
            conditions=dict(kind="run", depth_mm=4, speed=0, hold_s=4, release_speed=0),
        )
        b = json.loads(json.dumps(a))
        b["conditions"]["depth_mm"] = 4.0
        self.assertEqual(m.group_key(a), m.group_key(b))
        b["conditions"]["release_speed"] = 0.1
        self.assertNotEqual(m.group_key(a), m.group_key(b))
        b = json.loads(json.dumps(a))
        b["position_id"] = "up"
        self.assertNotEqual(m.group_key(a), m.group_key(b))

    def test_real_export_comparison_finds_known_difference_without_spike_bias(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            old = fixture(root / "old", "oldjob", 2, 6.0, spike=True)
            new = fixture(root / "new", "newjob", 3, 8.0)
            out = root / "out"
            self.assertEqual(
                m.main(
                    [
                        "--old-export",
                        str(old),
                        "--new-export",
                        str(new),
                        "--out",
                        str(out),
                    ]
                ),
                0,
            )
            r = json.loads((out / "report.json").read_text())
            self.assertEqual(
                (r["selected_old_trials"], r["selected_new_trials"]), (2, 3)
            )
            self.assertEqual(len(r["matched_groups"]), 1)
            self.assertEqual(r["trials_without_hold_events"], [])
            self.assertTrue((out / r["matched_groups"][0]["plot"]).is_file())
            with (out / "group_differences.csv").open() as stream:
                diffs = list(csv.DictReader(stream))
            late = [r for r in diffs if r["metric"] == "late_Fy_n"]
            self.assertEqual(len(late), 2)
            self.assertTrue(
                all(abs(float(r["new_minus_old"]) + 2.0) < 1e-9 for r in late)
            )
            with (out / "spike_candidates.csv").open() as stream:
                spikes = list(csv.DictReader(stream))
            self.assertEqual(len(spikes), 2)
            self.assertTrue(all(r["dataset"] == "old" for r in spikes))

    def test_missing_old_events_are_not_reconstructed(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            old = fixture(root / "old", "old", 1, 6.0, events=False)
            new = fixture(root / "new", "new", 1, 6.0)
            out = root / "out"
            self.assertEqual(
                m.main(
                    [
                        "--old-export",
                        str(old),
                        "--new-export",
                        str(new),
                        "--out",
                        str(out),
                    ]
                ),
                0,
            )
            r = json.loads((out / "report.json").read_text())
            self.assertEqual(
                r["trials_without_hold_events"],
                [dict(dataset="old", trial_id="base_0001")],
            )
            with (out / "group_differences.csv").open() as stream:
                diffs = list(csv.DictReader(stream))
            self.assertTrue(all(not r["new_minus_old"] for r in diffs))

    def test_no_matching_conditions_fail_without_creating_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            old = fixture(root / "old", "old", 1, 6.0)
            new = fixture(root / "new", "new", 1, 6.0, depth=6.0)
            with self.assertRaises(SystemExit):
                m.main(
                    [
                        "--old-export",
                        str(old),
                        "--new-export",
                        str(new),
                        "--out",
                        str(root / "out"),
                    ]
                )
            self.assertFalse((root / "out").exists())

    def test_tare_mismatch_is_not_silently_corrected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            old = fixture(root / "old", "old", 1, 6.0)
            new = fixture(root / "new", "new", 1, 6.0)
            p = old / "session/force_left.csv"
            s = p.read_text().replace(",10.0,", ",11.0,")
            p.write_text(s)
            with self.assertRaises(SystemExit):
                m.main(
                    [
                        "--old-export",
                        str(old),
                        "--new-export",
                        str(new),
                        "--out",
                        str(root / "out"),
                    ]
                )

    def test_knead_holds_keep_cycles_separate(self):
        descriptor = dict(window=dict(conditions=dict(kind="knead")), actions=[])
        for phase in range(200, 204):
            descriptor["actions"] += [
                dict(
                    phase_id=str(phase),
                    source_stamp_ns=str(phase * 1_000_000),
                    event="auto_dwell_start",
                ),
                dict(
                    phase_id=str(phase),
                    source_stamp_ns=str(phase * 1_000_000 + 500_000),
                    event="auto_dwell_end",
                ),
            ]
        holds = m.hold_windows(descriptor)
        self.assertEqual(
            [r[0] for r in holds],
            ["cycle_01_deep", "cycle_01_shallow", "cycle_02_deep", "cycle_02_shallow"],
        )

    def test_low_coverage_does_not_produce_hold_value(self):
        headers = np.arange(10, dtype=np.int64) * 1_000_000
        value, coverage = m.interval_median(
            headers, np.ones(10), np.ones(10, dtype=bool), 0, 100_000_000, 1_000_000
        )
        self.assertIsNone(value)
        self.assertLess(coverage, 0.8)


if __name__ == "__main__":
    unittest.main()
