#!/usr/bin/env python3
"""Audit ROS bag timing without modifying the input bag.

The bag timestamp is the rosbag2 recorder receive timestamp.  Header offsets are
reported as ``bag_timestamp - message_header_timestamp``.  No timestamp
correction is written or applied to the source data.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import sqlite3
import subprocess
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


FT_LEFT = "/force_torque/left"
FT_RIGHT = "/force_torque/right"
GS_LEFT = "/gelsight/left/image_raw/compressed"
GS_RIGHT = "/gelsight/right/image_raw/compressed"
RS_WRIST = "/camera_wrist/realsense2_camera/color/image_raw/compressed"
RS_FIXED = "/camera_fixed/realsense2_camera/color/image_raw/compressed"
GRIPPER_FLOAT = "/robotiq_2f_gripper/finger_distance_mm"
GRIPPER_STATUS = "/robotiq_2f_gripper/status"
ACTION_EVENTS = "/robot_action_event"
TRIAL_EVENTS = "/trial_event"
LEGACY_TRIAL_MARKERS = "/trial_marker"
CURRENT_PHASE = "/current_phase"

CORE_TOPICS = (
    FT_LEFT,
    FT_RIGHT,
    GS_LEFT,
    GS_RIGHT,
    RS_WRIST,
    RS_FIXED,
    "/joint_states",
    GRIPPER_FLOAT,
    GRIPPER_STATUS,
    ACTION_EVENTS,
    TRIAL_EVENTS,
    LEGACY_TRIAL_MARKERS,
    CURRENT_PHASE,
)


@dataclass
class TopicSamples:
    name: str
    type_name: str
    bag_ns: list[int] = field(default_factory=list)
    header_ns: list[Optional[int]] = field(default_factory=list)
    values: list[float] = field(default_factory=list)
    value_ns: list[int] = field(default_factory=list)
    events: list[dict[str, Any]] = field(default_factory=list)


def _stamp_ns(stamp: Any) -> Optional[int]:
    if stamp is None:
        return None
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value if value > 0 else None


def _message_header_ns(msg: Any) -> Optional[int]:
    header = getattr(msg, "header", None)
    if header is not None:
        return _stamp_ns(getattr(header, "stamp", None))
    transforms = getattr(msg, "transforms", None)
    if transforms:
        return _stamp_ns(transforms[0].header.stamp)
    return None


def _percentile(values: np.ndarray, q: float) -> Optional[float]:
    return float(np.percentile(values, q)) if values.size else None


def _median_hz(times_ns: np.ndarray) -> Optional[float]:
    if times_ns.size < 2:
        return None
    dt = np.diff(times_ns.astype(np.float64)) / 1e9
    dt = dt[dt > 0]
    if not dt.size:
        return None
    return float(1.0 / np.median(dt))


def _offset_drift_ms_per_minute(times_ns: np.ndarray, offsets_ns: np.ndarray) -> Optional[float]:
    if times_ns.size < 3 or times_ns[-1] == times_ns[0]:
        return None
    order = np.argsort(times_ns)
    times_ns = times_ns[order]
    offsets_ns = offsets_ns[order]
    window = max(1, times_ns.size // 10)
    first_offset_ms = float(np.median(offsets_ns[:window]) / 1e6)
    last_offset_ms = float(np.median(offsets_ns[-window:]) / 1e6)
    first_time = float(np.median(times_ns[:window]))
    last_time = float(np.median(times_ns[-window:]))
    elapsed_min = (last_time - first_time) / 60e9
    if elapsed_min <= 0:
        return None
    return (last_offset_ms - first_offset_ms) / elapsed_min


def _topic_metrics(samples: TopicSamples) -> dict[str, Any]:
    bag = np.asarray(samples.bag_ns, dtype=np.int64)
    valid_pairs = [
        (b, h) for b, h in zip(samples.bag_ns, samples.header_ns) if h is not None
    ]
    header_available = bool(valid_pairs)
    timeline = np.asarray(
        [h for _, h in valid_pairs], dtype=np.int64
    ) if header_available else bag
    diffs = np.diff(timeline) if timeline.size >= 2 else np.asarray([], dtype=np.int64)
    positive = diffs[diffs > 0]
    median_dt = float(np.median(positive)) if positive.size else None
    result: dict[str, Any] = {
        "topic": samples.name,
        "type": samples.type_name,
        "messages": len(samples.bag_ns),
        "median_hz": _median_hz(timeline),
        "header_available": header_available,
        "timestamp_basis": "header" if header_available else "bag_receive",
        "span_sec": (
            float((bag[-1] - bag[0]) / 1e9) if bag.size >= 2 else 0.0
        ),
        "backward_jumps": int(np.count_nonzero(diffs < 0)),
        "duplicate_timestamps": int(np.count_nonzero(diffs == 0)),
        "large_gaps": (
            int(np.count_nonzero(diffs > 1.5 * median_dt))
            if median_dt is not None else 0
        ),
    }
    if header_available:
        pair_bag = np.asarray([b for b, _ in valid_pairs], dtype=np.int64)
        pair_header = np.asarray([h for _, h in valid_pairs], dtype=np.int64)
        offset = pair_bag - pair_header
        result["bag_minus_header_median_ms"] = float(np.median(offset) / 1e6)
        result["bag_minus_header_iqr_ms"] = float(
            (np.percentile(offset, 75) - np.percentile(offset, 25)) / 1e6
        )
        result["bag_minus_header_p95_abs_ms"] = float(np.percentile(np.abs(offset), 95) / 1e6)
        result["bag_minus_header_max_abs_ms"] = float(np.max(np.abs(offset)) / 1e6)
        result["offset_drift_ms_per_minute"] = _offset_drift_ms_per_minute(
            pair_header, offset
        )
    else:
        result.update({
            "bag_minus_header_median_ms": None,
            "bag_minus_header_iqr_ms": None,
            "bag_minus_header_p95_abs_ms": None,
            "bag_minus_header_max_abs_ms": None,
            "offset_drift_ms_per_minute": None,
        })
    return result


def _nearest_offsets_ms(a_ns: list[int], b_ns: list[int]) -> dict[str, Any]:
    """Return A-nearest-B offsets; sign is A timestamp minus B timestamp."""
    a = np.asarray(a_ns, dtype=np.int64)
    b = np.asarray(b_ns, dtype=np.int64)
    if not a.size or not b.size:
        return {"pairs": 0}
    b = np.sort(b)
    a = a[(a >= b[0]) & (a <= b[-1])]
    if not a.size:
        return {"pairs": 0}
    index = np.searchsorted(b, a)
    right = np.minimum(index, b.size - 1)
    left = np.maximum(index - 1, 0)
    choose_right = np.abs(a - b[right]) < np.abs(a - b[left])
    nearest = np.where(choose_right, b[right], b[left])
    offsets = (a - nearest).astype(np.float64) / 1e6
    return {
        "pairs": int(offsets.size),
        "median_ms": float(np.median(offsets)),
        "iqr_ms": float(np.percentile(offsets, 75) - np.percentile(offsets, 25)),
        "p95_abs_ms": float(np.percentile(np.abs(offsets), 95)),
        "max_abs_ms": float(np.max(np.abs(offsets))),
        "definition": "A header timestamp minus nearest B header timestamp",
    }


def _observed_signal_lag_ms(
    reference: TopicSamples,
    target: TopicSamples,
    max_lag_sec: float,
    sample_hz: float = 100.0,
) -> Optional[dict[str, Any]]:
    """Cross-correlate sampled signals; positive lag means target occurs later."""
    if len(reference.values) < 10 or len(target.values) < 10:
        return None
    rt = np.asarray(reference.value_ns, dtype=np.float64) / 1e9
    tt = np.asarray(target.value_ns, dtype=np.float64) / 1e9
    start = max(rt[0], tt[0])
    end = min(rt[-1], tt[-1])
    if end - start < 1.0:
        return None
    grid = np.arange(start, end, 1.0 / sample_hz)
    r = np.interp(grid, rt, np.asarray(reference.values, dtype=np.float64))
    t = np.interp(grid, tt, np.asarray(target.values, dtype=np.float64))
    r = np.abs(r - np.median(r))
    t = np.abs(t - np.median(t))
    r = (r - np.mean(r)) / (np.std(r) + 1e-12)
    t = (t - np.mean(t)) / (np.std(t) + 1e-12)
    max_steps = max(1, int(max_lag_sec * sample_hz))
    best: tuple[float, int] = (-math.inf, 0)
    for lag in range(-max_steps, max_steps + 1):
        if lag > 0:
            x, y = r[:-lag], t[lag:]
        elif lag < 0:
            x, y = r[-lag:], t[:lag]
        else:
            x, y = r, t
        score = float(np.mean(x * y))
        if score > best[0]:
            best = (score, lag)
    return {
        "target_lag_ms": float(best[1] * 1000.0 / sample_hz),
        "normalized_correlation": best[0],
        "reliable": best[0] >= 0.3,
        "positive_lag_means": "target signal occurs later than reference",
        "correction_applied": False,
    }


def _decode_activity(
    msg: Any, previous: Optional[np.ndarray]
) -> tuple[Optional[float], Optional[np.ndarray]]:
    try:
        import cv2
        encoded = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_GRAYSCALE)
        if image is None:
            return None, previous
        small = cv2.resize(image, (80, 60), interpolation=cv2.INTER_AREA).astype(
            np.float32
        )
        if previous is None:
            return 0.0, small
        return float(np.mean(np.abs(small - previous))), small
    except Exception:
        return None, previous


def _event_dict(msg: Any, stamp_ns: int) -> dict[str, Any]:
    keys = (
        "event", "trial_index", "phase_id", "action_type", "mode",
        "target_position", "target_speed", "success",
    )
    result = {"stamp_ns": stamp_ns}
    for key in keys:
        if hasattr(msg, key):
            result[key] = getattr(msg, key)
    if hasattr(msg, "data") and isinstance(msg.data, str):
        result["data"] = msg.data
    return result


class BagDatabase:
    """Read sqlite3 rosbag data, decompressing only into a temporary directory."""

    def __init__(self, bag_path: Path):
        self.bag_path = bag_path.expanduser().resolve()
        self._temp: Optional[tempfile.TemporaryDirectory[str]] = None

    def __enter__(self) -> Path:
        if self.bag_path.is_file() and self.bag_path.suffix == ".db3":
            return self.bag_path
        directory = self.bag_path if self.bag_path.is_dir() else self.bag_path.parent
        databases = sorted(directory.glob("*.db3"))
        if databases:
            return databases[0]
        compressed = sorted(directory.glob("*.db3.zstd"))
        if not compressed:
            raise FileNotFoundError(f"No .db3 or .db3.zstd file found under {directory}")
        if shutil.which("zstd") is None:
            raise RuntimeError("zstd executable is required to inspect a file-compressed bag")
        self._temp = tempfile.TemporaryDirectory(prefix="sync_qa_")
        output = Path(self._temp.name) / compressed[0].name.removesuffix(".zstd")
        with output.open("wb") as stream:
            subprocess.run(
                ["zstd", "-d", "-c", str(compressed[0])],
                stdout=stream,
                check=True,
            )
        return output

    def __exit__(self, *_: object) -> None:
        if self._temp is not None:
            self._temp.cleanup()


def read_bag(
    bag_path: Path, decode_images: bool = False
) -> tuple[dict[str, TopicSamples], list[str]]:
    warnings: list[str] = []
    result: dict[str, TopicSamples] = {}
    previous_images: dict[str, Optional[np.ndarray]] = {GS_LEFT: None, GS_RIGHT: None}
    with BagDatabase(bag_path) as database:
        connection = sqlite3.connect(f"file:{database}?mode=ro", uri=True)
        topics = {
            int(row[0]): (str(row[1]), str(row[2]))
            for row in connection.execute("SELECT id, name, type FROM topics")
        }
        for _, (name, type_name) in topics.items():
            result[name] = TopicSamples(name=name, type_name=type_name)
        type_cache: dict[str, Any] = {}
        unavailable_types: set[str] = set()
        query = "SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp"
        for topic_id, bag_ns, serialized in connection.execute(query):
            name, type_name = topics[int(topic_id)]
            samples = result[name]
            samples.bag_ns.append(int(bag_ns))
            if type_name in unavailable_types:
                samples.header_ns.append(None)
                continue
            try:
                msg_type = type_cache.setdefault(type_name, get_message(type_name))
                msg = deserialize_message(serialized, msg_type)
            except Exception as exc:
                unavailable_types.add(type_name)
                samples.header_ns.append(None)
                warnings.append(f"Could not deserialize {type_name}: {exc}")
                continue
            header_ns = _message_header_ns(msg)
            samples.header_ns.append(header_ns)
            value_stamp = header_ns if header_ns is not None else int(bag_ns)
            if name in (FT_LEFT, FT_RIGHT):
                samples.values.append(float(msg.wrench.force.y))
                samples.value_ns.append(value_stamp)
            elif name == GRIPPER_FLOAT:
                samples.values.append(float(msg.data))
                samples.value_ns.append(value_stamp)
            elif name == GRIPPER_STATUS:
                # Same conversion used by the driver; this is a derived aperture,
                # not calibrated indentation.
                raw = int(msg.g_po)
                if raw <= 200:
                    width_m = (
                        -3.84615e-07 * raw ** 2
                        - 5.67622e-04 * raw
                        + 0.142692
                    )
                elif raw <= 226:
                    width_m = (
                        8.92857e-06 * raw ** 2
                        - 4.38036e-03 * raw
                        + 0.533911
                    )
                else:
                    width_m = 0.0
                samples.values.append(width_m * 1000.0)
                samples.value_ns.append(value_stamp)
            elif name == "/joint_states" and getattr(msg, "position", None):
                samples.values.append(float(msg.position[0]))
                samples.value_ns.append(value_stamp)
            elif name == CURRENT_PHASE:
                samples.values.append(float(msg.data))
                samples.value_ns.append(value_stamp)
            elif name in (ACTION_EVENTS, TRIAL_EVENTS, LEGACY_TRIAL_MARKERS):
                samples.events.append(_event_dict(msg, value_stamp))
            elif decode_images and name in (GS_LEFT, GS_RIGHT):
                activity, previous_images[name] = _decode_activity(msg, previous_images[name])
                if activity is not None:
                    samples.values.append(activity)
                    samples.value_ns.append(value_stamp)
        connection.close()
    return result, warnings


def _event_coverage(topics: dict[str, TopicSamples]) -> dict[str, Any]:
    events = topics.get(ACTION_EVENTS, TopicSamples("", "")).events
    names = [str(event.get("event", "")) for event in events]
    required = (
        "trial_run_received",
        "phase1_goal_sent",
        "phase1_action_succeeded",
        "phase2_close_goal_sent",
        "phase2_close_action_succeeded",
        "hold_start",
        "hold_end",
        "phase3_open_goal_sent",
        "phase3_open_action_succeeded",
        "sequence_finished",
    )
    coverage = {name: names.count(name) for name in required}
    starts = [e["stamp_ns"] for e in events if e.get("event") == "hold_start"]
    ends = [e["stamp_ns"] for e in events if e.get("event") == "hold_end"]
    hold_durations = [
        (end - start) / 1e9 for start, end in zip(starts, ends) if end >= start
    ]
    legacy = topics.get(LEGACY_TRIAL_MARKERS, TopicSamples("", "")).events
    trial = topics.get(TRIAL_EVENTS, TopicSamples("", "")).events
    return {
        "required_event_counts": coverage,
        "complete": bool(events) and all(coverage.values()),
        "hold_duration_sec": hold_durations,
        "stamped_trial_events": len(trial),
        "legacy_unstamped_trial_markers": len(legacy),
    }


def _loading_coverage(topics: dict[str, TopicSamples]) -> list[dict[str, Any]]:
    events = topics.get(ACTION_EVENTS, TopicSamples("", "")).events
    starts = [e for e in events if e.get("event") == "phase2_close_goal_sent"]
    ends = [e for e in events if e.get("event") == "phase2_close_action_succeeded"]
    reports: list[dict[str, Any]] = []
    for start, end in zip(starts, ends):
        lo, hi = int(start["stamp_ns"]), int(end["stamp_ns"])
        if hi < lo:
            continue
        entry: dict[str, Any] = {
            "trial_index": int(start.get("trial_index", 0)),
            "close_goal_to_action_success_sec": (hi - lo) / 1e9,
        }
        contact_candidates: list[int] = []
        for topic_name in (FT_LEFT, FT_RIGHT):
            samples = topics.get(topic_name)
            if samples is None or not samples.values:
                entry[f"{topic_name}_samples"] = 0
                continue
            times = np.asarray(samples.value_ns, dtype=np.int64)
            values = np.abs(np.asarray(samples.values, dtype=np.float64))
            in_loading = (times >= lo) & (times <= hi)
            entry[f"{topic_name}_samples"] = int(np.count_nonzero(in_loading))
            baseline = values[(times >= lo - 500_000_000) & (times < lo)]
            if baseline.size:
                noise = np.median(np.abs(baseline - np.median(baseline)))
                threshold = float(np.median(baseline) + max(0.03, 6.0 * noise))
                found = np.flatnonzero(in_loading & (values >= threshold))
                if found.size:
                    contact_candidates.append(int(times[found[0]]))
        gel_topics = (
            (GS_LEFT, "gelsight_left_frames"),
            (GS_RIGHT, "gelsight_right_frames"),
        )
        for topic_name, label in gel_topics:
            samples = topics.get(topic_name)
            times = np.asarray(
                [
                    h if h is not None else b
                    for b, h in zip(samples.bag_ns, samples.header_ns)
                ],
                dtype=np.int64,
            ) if samples else np.asarray([], dtype=np.int64)
            entry[label] = int(np.count_nonzero((times >= lo) & (times <= hi)))
        gripper = topics.get(GRIPPER_STATUS) or topics.get(GRIPPER_FLOAT)
        gt = (
            np.asarray(gripper.value_ns, dtype=np.int64)
            if gripper else np.asarray([], dtype=np.int64)
        )
        entry["gripper_state_samples"] = int(np.count_nonzero((gt >= lo) & (gt <= hi)))
        if contact_candidates:
            contact_ns = min(contact_candidates)
            entry["contact_to_loading_end_sec"] = (hi - contact_ns) / 1e9
            entry["contact_detection"] = "first FT threshold crossing (QA estimate only)"
        else:
            entry["contact_to_loading_end_sec"] = None
        reports.append(entry)
    return reports


def audit_bag(
    bag_path: Path, decode_images: bool = False
) -> tuple[dict[str, Any], dict[str, TopicSamples]]:
    topics, warnings = read_bag(bag_path, decode_images=decode_images)
    metrics = [_topic_metrics(topics[name]) for name in sorted(topics)]
    bag_starts = [sample.bag_ns[0] for sample in topics.values() if sample.bag_ns]
    bag_ends = [sample.bag_ns[-1] for sample in topics.values() if sample.bag_ns]
    overall_start = min(bag_starts) if bag_starts else 0
    overall_end = max(bag_ends) if bag_ends else 0
    for item in metrics:
        sample = topics[item["topic"]]
        item["bag_start_delay_sec"] = (
            (sample.bag_ns[0] - overall_start) / 1e9 if sample.bag_ns else None
        )
        item["bag_end_early_sec"] = (
            (overall_end - sample.bag_ns[-1]) / 1e9 if sample.bag_ns else None
        )
    sync: dict[str, Any] = {}
    for label, left_name, right_name in (
        ("ft_left_minus_nearest_right", FT_LEFT, FT_RIGHT),
        ("gelsight_left_minus_nearest_right", GS_LEFT, GS_RIGHT),
        ("realsense_wrist_minus_nearest_fixed", RS_WRIST, RS_FIXED),
    ):
        left = topics.get(left_name)
        right = topics.get(right_name)
        left_times = [h for h in left.header_ns if h is not None] if left else []
        right_times = [h for h in right.header_ns if h is not None] if right else []
        sync[label] = _nearest_offsets_ms(left_times, right_times)
    if FT_LEFT in topics and FT_RIGHT in topics:
        sync["ft_right_observed_signal_lag"] = _observed_signal_lag_ms(
            topics[FT_LEFT], topics[FT_RIGHT], max_lag_sec=0.25
        )
    if decode_images:
        ft_gel_pairs = (
            (GS_LEFT, "ft_left_to_gelsight_left_observed_lag"),
            (GS_RIGHT, "ft_right_to_gelsight_right_observed_lag"),
        )
        for gs_name, label in ft_gel_pairs:
            ft_name = FT_LEFT if gs_name == GS_LEFT else FT_RIGHT
            if ft_name in topics and gs_name in topics:
                sync[label] = _observed_signal_lag_ms(
                    topics[ft_name], topics[gs_name], max_lag_sec=1.0, sample_hz=50.0
                )
                if sync[label] is not None:
                    sync[label]["interpretation"] = (
                        "Observed signal lag includes physical deformation/force "
                        "response and must not be used as a timestamp correction."
                    )
    for item in metrics:
        if not item["header_available"] and item["topic"] in CORE_TOPICS:
            warnings.append(
                f"{item['topic']}: no header; only bag receive time is available"
            )
        if item["backward_jumps"]:
            warnings.append(
                f"{item['topic']}: {item['backward_jumps']} backward "
                "header/timestamp jumps"
            )
        p95 = item.get("bag_minus_header_p95_abs_ms")
        if p95 is not None and p95 > 100.0:
            warnings.append(f"{item['topic']}: p95 |bag-header| is {p95:.3f} ms")
        if item["topic"] in CORE_TOPICS:
            if (item["bag_start_delay_sec"] or 0.0) > 0.5:
                warnings.append(
                    f"{item['topic']}: starts {item['bag_start_delay_sec']:.3f} s after bag start"
                )
            if (item["bag_end_early_sec"] or 0.0) > 0.5:
                warnings.append(
                    f"{item['topic']}: ends {item['bag_end_early_sec']:.3f} s before bag end"
                )
    for name, value in sync.items():
        if isinstance(value, dict) and value.get("reliable") is False:
            warnings.append(
                f"{name}: cross-correlation is weak "
                f"({value['normalized_correlation']:.3f}); lag is not identifiable"
            )
    report = {
        "input_bag": str(bag_path.expanduser().resolve()),
        "offset_definition": "bag recorder receive timestamp minus message header timestamp",
        "timestamp_correction_applied": False,
        "topic_timing": metrics,
        "event_coverage": _event_coverage(topics),
        "loading_coverage": _loading_coverage(topics),
        "synchronization": sync,
        "warnings": sorted(set(warnings)),
    }
    return report, topics


def _fmt(value: Any, digits: int = 3) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def markdown_report(report: dict[str, Any]) -> str:
    lines = [
        "# Synchronization QA",
        "",
        f"Input: `{report['input_bag']}`",
        "",
        "No timestamp correction was applied. Offsets are `bag receive - header`.",
        "",
        "## Topic timing",
        "",
        "| topic | msgs | median Hz | header | median offset ms | IQR ms | "
        "p95 abs ms | max abs ms | drift ms/min | backward | duplicate | large gaps |",
        "|---|---:|---:|:---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for item in report["topic_timing"]:
        lines.append(
            "| {topic} | {messages} | {hz} | {header} | {median} | {iqr} | "
            "{p95} | {max_abs} | {drift} | {backward} | {duplicate} | "
            "{gaps} |".format(
                topic=item["topic"],
                messages=item["messages"],
                hz=_fmt(item["median_hz"]),
                header="yes" if item["header_available"] else "no",
                median=_fmt(item["bag_minus_header_median_ms"]),
                iqr=_fmt(item["bag_minus_header_iqr_ms"]),
                p95=_fmt(item["bag_minus_header_p95_abs_ms"]),
                max_abs=_fmt(item["bag_minus_header_max_abs_ms"]),
                drift=_fmt(item["offset_drift_ms_per_minute"]),
                backward=item["backward_jumps"],
                duplicate=item["duplicate_timestamps"],
                gaps=item["large_gaps"],
            )
        )
    event = report["event_coverage"]
    lines.extend(["", "## Event coverage", ""])
    for name, count in event["required_event_counts"].items():
        lines.append(f"- {name}: {count}")
    lines.append(f"- hold duration(s): {event['hold_duration_sec'] or 'n/a'}")
    lines.extend(["", "## Loading coverage", ""])
    if report["loading_coverage"]:
        for entry in report["loading_coverage"]:
            encoded = json.dumps(entry, ensure_ascii=False)
            lines.append(f"- trial {entry.get('trial_index', 0)}: `{encoded}`")
    else:
        lines.append("- unavailable: stamped close/action boundary events are absent")
    lines.extend(["", "## Synchronization", ""])
    for name, value in report["synchronization"].items():
        lines.append(f"- {name}: `{json.dumps(value, ensure_ascii=False)}`")
    lines.extend(["", "## Warnings", ""])
    if report["warnings"]:
        lines.extend(f"- {warning}" for warning in report["warnings"])
    else:
        lines.append("- none")
    return "\n".join(lines) + "\n"


def save_timeline(topics: dict[str, TopicSamples], output: Path) -> None:
    import matplotlib.pyplot as plt

    nonempty = [s for s in topics.values() if s.bag_ns]
    if not nonempty:
        raise ValueError("Bag has no messages")
    origin = min(min(s.bag_ns) for s in nonempty)
    fig, axes = plt.subplots(5, 1, figsize=(14, 11), sharex=True, constrained_layout=True)

    gripper = topics.get(GRIPPER_STATUS) or topics.get(GRIPPER_FLOAT)
    if gripper and gripper.values:
        axes[0].plot((np.asarray(gripper.value_ns) - origin) / 1e9, gripper.values, lw=1)
    axes[0].set_ylabel("width [mm]")
    axes[0].set_title("Gripper aperture (driver-derived; not indentation)")

    action = topics.get(ACTION_EVENTS)
    if action:
        for event in action.events:
            x = (event["stamp_ns"] - origin) / 1e9
            axes[1].axvline(x, lw=0.8)
            axes[1].text(x, 0.05, str(event.get("event", "")), rotation=90, fontsize=6)
    legacy = topics.get(LEGACY_TRIAL_MARKERS)
    if legacy and not (action and action.events):
        for event in legacy.events:
            axes[1].axvline((event["stamp_ns"] - origin) / 1e9, lw=0.8)
    phase = topics.get(CURRENT_PHASE)
    if phase and phase.values:
        axes[1].step(
            (np.asarray(phase.value_ns) - origin) / 1e9,
            phase.values,
            where="post",
            color="black",
            lw=0.8,
        )
    axes[1].set_ylabel("events")
    if not (phase and phase.values):
        axes[1].set_yticks([])

    for name, color in ((FT_LEFT, "tab:blue"), (FT_RIGHT, "tab:orange")):
        sample = topics.get(name)
        if sample and sample.values:
            axes[2].plot(
                (np.asarray(sample.value_ns) - origin) / 1e9,
                sample.values,
                lw=0.6,
                label=name.rsplit("/", 1)[-1],
                color=color,
            )
    axes[2].set_ylabel("FT Fy [N]")
    axes[2].legend(loc="upper right")

    gel_topics = ((GS_LEFT, "tab:blue"), (GS_RIGHT, "tab:orange"))
    for y, (name, color) in enumerate(gel_topics):
        sample = topics.get(name)
        if sample:
            times = np.asarray([
                h if h is not None else b
                for b, h in zip(sample.bag_ns, sample.header_ns)
            ])
            axes[3].scatter(
                (times - origin) / 1e9,
                np.full(times.size, y),
                s=2,
                color=color,
                label=name.rsplit("/", 3)[1],
            )
    axes[3].set_ylabel("GelSight frames")
    axes[3].set_yticks([0, 1], ["left", "right"])

    joints = topics.get("/joint_states")
    if joints and joints.values:
        axes[4].plot((np.asarray(joints.value_ns) - origin) / 1e9, joints.values, lw=0.6)
    axes[4].set_ylabel("joint[0] [rad]")
    axes[4].set_xlabel("time from first bag message [s]")
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=150)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="Bag directory or uncompressed .db3 file")
    parser.add_argument("--json", type=Path, help="Write machine-readable report")
    parser.add_argument("--markdown", type=Path, help="Write Markdown report")
    parser.add_argument("--timeline", type=Path, help="Write timeline PNG")
    parser.add_argument(
        "--decode-images", action="store_true",
        help="Decode GelSight JPEGs and estimate observed FT/image-activity lag (slower)",
    )
    args = parser.parse_args()
    report, topics = audit_bag(args.bag, decode_images=args.decode_images)
    rendered = markdown_report(report)
    print(rendered, end="")
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(
            json.dumps(report, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
    if args.markdown:
        args.markdown.parent.mkdir(parents=True, exist_ok=True)
        args.markdown.write_text(rendered, encoding="utf-8")
    if args.timeline:
        save_timeline(topics, args.timeline)


if __name__ == "__main__":
    main()
