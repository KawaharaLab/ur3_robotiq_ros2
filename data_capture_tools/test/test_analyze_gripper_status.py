"""Tests for the offline Robotiq status analyzer."""

import csv

import pytest

from data_capture_tools.analyze_gripper_status import load_csv, summarize


def test_load_and_summarize_flattened_csv(tmp_path):
    """Flattened ROS fields and timing statistics are interpreted correctly."""
    path = tmp_path / "status.csv"
    prefix = "robotiq_2f_gripper_status"
    fieldnames = [
        "stamp_ns",
        f"{prefix}.header.stamp.sec",
        f"{prefix}.header.stamp.nanosec",
        *(f"{prefix}.{name}" for name in (
            "g_obj", "g_flt", "g_pr", "g_po", "g_cu",
            "read_duration_ms",
        )),
    ]
    rows = [
        (0, 0, 0, 20, 10, 30, 1.0),
        (50_000_000, 1, 0, 21, 11, 31, 2.0),
        (100_000_000, 2, 7, 22, 12, 32, 3.0),
        (250_000_000, 3, 0, 23, 13, 33, 4.0),
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(fieldnames)
        for stamp_ns, *values in rows:
            writer.writerow([stamp_ns, 1, stamp_ns, *values])

    result = summarize(load_csv(path), long_gap_ms=100.0)

    assert result["sample_count"] == 4
    assert result["effective_hz"] == pytest.approx(12.0)
    assert result["interval_median_ms"] == pytest.approx(50.0)
    assert result["interval_p95_ms"] == pytest.approx(140.0)
    assert result["interval_p99_ms"] == pytest.approx(148.0)
    assert result["interval_max_ms"] == pytest.approx(150.0)
    assert result["long_gap_count"] == 1
    assert result["g_po"] == {
        "min": 10, "max": 13, "unique": [10, 11, 12, 13]
    }
    assert result["g_pr"] == {
        "min": 20, "max": 23, "unique": [20, 21, 22, 23]
    }
    assert result["g_cu"] == {
        "min": 30, "max": 33, "unique": [30, 31, 32, 33]
    }
    assert result["g_obj_counts"] == {0: 1, 1: 1, 2: 1, 3: 1}
    assert result["g_flt_nonzero_count"] == 1
    assert result["read_duration_ms"]["max"] == pytest.approx(4.0)
