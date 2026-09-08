"""Summarize timing and raw-register values from Robotiq status data."""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


STATUS_TOPIC = "/robotiq_2f_gripper/status"


@dataclass(frozen=True)
class StatusSample:
    """One stamped Robotiq status sample."""

    stamp_ns: int
    g_obj: int
    g_flt: int
    g_pr: int
    g_po: int
    g_cu: int
    read_duration_ms: float | None = None


def _find_column(fieldnames: Sequence[str], field: str) -> str | None:
    """Find an exact or flattened ROS message field name."""
    if field in fieldnames:
        return field
    suffix = f".{field}"
    matches = [name for name in fieldnames if name.endswith(suffix)]
    return matches[0] if len(matches) == 1 else None


def load_csv(path: Path) -> list[StatusSample]:
    """Load status samples from a bag-converter CSV."""
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        fields = reader.fieldnames or []
        columns = {
            name: _find_column(fields, name)
            for name in (
                "g_obj",
                "g_flt",
                "g_pr",
                "g_po",
                "g_cu",
                "read_duration_ms",
            )
        }
        missing = [
            name
            for name in ("g_obj", "g_flt", "g_pr", "g_po", "g_cu")
            if columns[name] is None
        ]
        if missing:
            raise ValueError(f"missing status columns: {', '.join(missing)}")

        sec_column = _find_column(fields, "header.stamp.sec")
        nanosec_column = _find_column(fields, "header.stamp.nanosec")
        if sec_column is None:
            sec_column = next(
                (
                    name
                    for name in fields
                    if name.endswith(".header.stamp.sec")
                ),
                None,
            )
        if nanosec_column is None:
            nanosec_column = next(
                (
                    name
                    for name in fields
                    if name.endswith(".header.stamp.nanosec")
                ),
                None,
            )
        if not (sec_column and nanosec_column) and "stamp_ns" not in fields:
            raise ValueError("CSV has neither header.stamp nor stamp_ns")

        samples = []
        for row in reader:
            if sec_column and nanosec_column:
                stamp_ns = int(row[sec_column]) * 1_000_000_000
                stamp_ns += int(row[nanosec_column])
            else:
                stamp_ns = int(row["stamp_ns"])
            duration_column = columns["read_duration_ms"]
            duration = (
                float(row[duration_column])
                if duration_column and row[duration_column] != ""
                else None
            )
            samples.append(
                StatusSample(
                    stamp_ns=stamp_ns,
                    g_obj=int(row[columns["g_obj"]]),
                    g_flt=int(row[columns["g_flt"]]),
                    g_pr=int(row[columns["g_pr"]]),
                    g_po=int(row[columns["g_po"]]),
                    g_cu=int(row[columns["g_cu"]]),
                    read_duration_ms=duration,
                )
            )
    return samples


def load_bag(path: Path, topic: str = STATUS_TOPIC) -> list[StatusSample]:
    """Load status samples directly from a rosbag2 URI."""
    try:
        from rclpy.serialization import deserialize_message
        from robotiq_2f_gripper_msgs.msg import GripperStatus
        from rosbag2_py import (
            ConverterOptions,
            SequentialReader,
            StorageOptions,
        )
        from .bag_converter import _maybe_decompress_file_bag
    except ImportError as exc:
        raise RuntimeError(
            "ROS 2 Python packages must be sourced to read a bag"
        ) from exc

    bag_to_read, temp_handle = _maybe_decompress_file_bag(path)
    try:
        reader = SequentialReader()
        reader.open(
            StorageOptions(uri=str(bag_to_read), storage_id="sqlite3"),
            ConverterOptions(
                input_serialization_format="cdr",
                output_serialization_format="cdr",
            ),
        )
        available = {item.name for item in reader.get_all_topics_and_types()}
        if topic not in available:
            raise ValueError(f"bag does not contain {topic}")

        samples = []
        while reader.has_next():
            current_topic, data, _ = reader.read_next()
            if current_topic != topic:
                continue
            message = deserialize_message(data, GripperStatus)
            stamp_ns = message.header.stamp.sec * 1_000_000_000
            stamp_ns += message.header.stamp.nanosec
            samples.append(
                StatusSample(
                    stamp_ns=stamp_ns,
                    g_obj=message.g_obj,
                    g_flt=message.g_flt,
                    g_pr=message.g_pr,
                    g_po=message.g_po,
                    g_cu=message.g_cu,
                    read_duration_ms=message.read_duration_ms,
                )
            )
        return samples
    finally:
        if temp_handle:
            temp_handle.cleanup()


def _percentile(values: Sequence[float], percentile: float) -> float:
    """Calculate a linearly interpolated percentile."""
    ordered = sorted(values)
    position = (len(ordered) - 1) * percentile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def summarize(
    samples: Iterable[StatusSample],
    long_gap_ms: float | None = None,
) -> dict[str, object]:
    """Calculate timing and register statistics."""
    ordered = sorted(samples, key=lambda sample: sample.stamp_ns)
    intervals = [
        (right.stamp_ns - left.stamp_ns) / 1_000_000.0
        for left, right in zip(ordered, ordered[1:])
    ]
    positive_intervals = [value for value in intervals if value > 0.0]
    median = (
        _percentile(positive_intervals, 0.5)
        if positive_intervals else None
    )
    threshold = long_gap_ms
    if threshold is None and median is not None:
        threshold = 2.0 * median

    timing = {
        "effective_hz": (
            (len(ordered) - 1) * 1_000_000_000.0
            / (ordered[-1].stamp_ns - ordered[0].stamp_ns)
            if len(ordered) > 1
            and ordered[-1].stamp_ns > ordered[0].stamp_ns
            else None
        ),
        "interval_median_ms": median,
        "interval_p95_ms": (
            _percentile(positive_intervals, 0.95)
            if positive_intervals else None
        ),
        "interval_p99_ms": (
            _percentile(positive_intervals, 0.99)
            if positive_intervals else None
        ),
        "interval_max_ms": (
            max(positive_intervals) if positive_intervals else None
        ),
        "long_gap_threshold_ms": threshold,
        "long_gap_count": (
            sum(value > threshold for value in positive_intervals)
            if threshold is not None else 0
        ),
        "nonpositive_interval_count": len(intervals) - len(positive_intervals),
    }

    result: dict[str, object] = {"sample_count": len(ordered), **timing}
    for field in ("g_po", "g_pr", "g_cu"):
        values = [getattr(sample, field) for sample in ordered]
        result[field] = {
            "min": min(values) if values else None,
            "max": max(values) if values else None,
            "unique": sorted(set(values)),
        }
    result["g_obj_counts"] = {
        state: sum(sample.g_obj == state for sample in ordered)
        for state in range(4)
    }
    result["g_flt_nonzero_count"] = sum(
        sample.g_flt != 0 for sample in ordered
    )

    durations = [
        sample.read_duration_ms for sample in ordered
        if sample.read_duration_ms is not None
    ]
    result["read_duration_ms"] = {
        "count": len(durations),
        "median": _percentile(durations, 0.5) if durations else None,
        "p95": _percentile(durations, 0.95) if durations else None,
        "p99": _percentile(durations, 0.99) if durations else None,
        "max": max(durations) if durations else None,
    }
    return result


def _format_value(value: object) -> str:
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)


def print_summary(summary: dict[str, object]) -> None:
    """Print a stable human-readable report."""
    for key, value in summary.items():
        if isinstance(value, dict):
            print(f"{key}:")
            for child_key, child_value in value.items():
                print(f"  {child_key}: {_format_value(child_value)}")
        else:
            print(f"{key}: {_format_value(value)}")


def main() -> None:
    """Run the command-line status analyzer."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "input", type=Path, help="status CSV or rosbag2 directory"
    )
    parser.add_argument("--topic", default=STATUS_TOPIC)
    parser.add_argument(
        "--long-gap-ms",
        type=float,
        help="long-gap threshold; default is twice the measured median",
    )
    args = parser.parse_args()
    if args.long_gap_ms is not None and (
        not math.isfinite(args.long_gap_ms) or args.long_gap_ms <= 0.0
    ):
        parser.error("--long-gap-ms must be finite and greater than zero")

    try:
        samples = (
            load_csv(args.input)
            if args.input.is_file() else load_bag(args.input, args.topic)
        )
        if not samples:
            raise ValueError("no status samples found")
    except (OSError, RuntimeError, ValueError) as exc:
        parser.error(str(exc))
    print_summary(summarize(samples, args.long_gap_ms))


if __name__ == "__main__":
    main()
