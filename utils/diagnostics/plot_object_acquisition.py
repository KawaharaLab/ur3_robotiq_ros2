#!/usr/bin/env python3
"""Read extract_all JPEG/CSV exports and write trial PNGs and condition comparisons."""
import argparse
from bisect import bisect_left, bisect_right
from collections import Counter, defaultdict
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import sys
import textwrap
import time

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "data_capture_tools"))
from data_capture_tools.compact_export import FORCE, GS
from data_capture_tools.object_plan import atomic_json

ACTION = "/robot_action_event"
WIDTH = "/robotiq_2f_gripper/finger_distance_mm"
STATUS = "/robotiq_2f_gripper/status"
SIDE_COLORS = ("tab:blue", "tab:orange")
CONDITION_KEYS = (
    "kind",
    "depth_mm",
    "speed",
    "hold_s",
    "shallow_mm",
    "deep_mm",
    "cycles",
    "loading_speed",
    "unloading_speed",
    "deep_hold_s",
    "shallow_hold_s",
    "preparation_speed",
    "release_speed",
    "trial_open_clearance_mm",
    "variant",
)


def safe_name(value):
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value)).strip("_") or "unknown"


def relative_seconds(headers, start_ns):
    # Subtract integer nanoseconds first, preserving sub-microsecond differences.
    return (np.asarray(headers, dtype=np.int64) - np.int64(start_ns)).astype(
        float
    ) / 1e9


def condition_identity(condition):
    values = {key: condition[key] for key in CONDITION_KEYS if key in condition}
    encoded = json.dumps(values, sort_keys=True, allow_nan=False)
    kind = "press" if condition["kind"] == "run" else "knead"
    if kind == "press":
        title = f"press depth={condition['depth_mm']:g}mm speed={condition['speed']:g} hold={condition['hold_s']:g}s"
    else:
        title = (
            f"knead {condition['shallow_mm']:g}->{condition['deep_mm']:g}mm "
            f"load/release={condition['loading_speed']:g}/{condition['unloading_speed']:g} "
            f"hold={condition['deep_hold_s']:g}/{condition['shallow_hold_s']:g}s cycles={condition['cycles']}"
        )
    title += f" final_release={condition.get('release_speed', .3):g}"
    identity = hashlib.sha256(encoded.encode()).hexdigest()[:10]
    return kind, safe_name(title) + "_" + identity, title, values


def select_rows(rows, low, high):
    # Source arrays are sorted once per bag before trial slicing.
    headers = [row[0] for row in rows]
    return rows[bisect_left(headers, low) : bisect_right(headers, high)]


def export_identity(session, manifest):
    """Identifiers come from copied metadata; original bag paths are never opened."""
    copied_progress = session.parent / "source_progress.json"
    job = json.loads(copied_progress.read_text()) if copied_progress.is_file() else {}
    windows = manifest["windows"]
    source_session = Path(windows[0]["attempt"]["session"])
    return source_session.parent.name, job.get("job_id", source_session.name)


def read_export_session(session, windows):
    selected = {window["trial_id"]: window for window in windows}
    if len(selected) != len(windows):
        raise ValueError("Duplicate trial IDs in manifest")
    data = {topic: [] for topic in GS + FORCE + (WIDTH, STATUS, ACTION)}
    errors = Counter()
    examples = []
    missing_auxiliary = []

    def rows_from(filename, topic, decoder, optional=False):
        path = session / filename
        if not path.is_file():
            if optional:
                missing_auxiliary.append(filename)
                return
            raise FileNotFoundError("Required export file missing: " + str(path))
        with path.open(newline="", encoding="utf-8") as stream:
            for number, row in enumerate(csv.DictReader(stream), 2):
                if row.get("trial_id") not in selected:
                    continue
                try:
                    decoder(row)
                except Exception as exc:
                    errors[topic] += 1
                    if len(examples) < 10:
                        examples.append(
                            dict(
                                file=str(path),
                                row=number,
                                error=type(exc).__name__ + ": " + str(exc),
                            )
                        )

    def decode_image(row):
        topic = row["topic"]
        if topic not in GS:
            raise ValueError("Unexpected GelSight topic")
        header = int(row["source_stamp_ns"])
        relative = Path(row["artifact_path"])
        path = (session / relative).resolve()
        if (
            relative.is_absolute()
            or ".." in relative.parts
            or session.resolve() not in path.parents
        ):
            raise ValueError("Image path outside export session")
        image = cv2.imdecode(
            np.frombuffer(path.read_bytes(), dtype=np.uint8), cv2.IMREAD_COLOR
        )
        if image is None:
            raise ValueError("JPEG decode failed: " + str(path))
        gray = cv2.resize(
            cv2.cvtColor(image, cv2.COLOR_BGR2GRAY),
            (56, 56),
            interpolation=cv2.INTER_AREA,
        ).astype(np.float32)
        thumb = cv2.cvtColor(cv2.resize(image, (112, 112)), cv2.COLOR_BGR2RGB)
        data[topic].append((header, gray, thumb))

    rows_from("image_index.csv", "GelSight", decode_image)
    for side, topic in zip(("left", "right"), FORCE):

        def decode_force(row, side=side, topic=topic):
            header, value = int(row["source_stamp_ns"]), float(row["Fy"])
            window = selected[row["trial_id"]]
            baseline = window["attempt"].get("tare", {}).get(side)
            csv_baseline = row.get("baseline_Fy")
            if csv_baseline not in (None, "") and baseline is not None:
                if not math.isclose(
                    float(csv_baseline), baseline, rel_tol=0, abs_tol=1e-9
                ):
                    raise ValueError("CSV baseline and manifest tare disagree")
            if baseline is not None and row.get("baseline_relative_Fy") not in (
                None,
                "",
            ):
                relative = float(row["baseline_relative_Fy"])
                if math.isfinite(value) and not math.isclose(
                    relative, value - baseline, rel_tol=0, abs_tol=1e-9
                ):
                    raise ValueError("CSV relative Fy and manifest tare disagree")
            data[topic].append((header, value))

        rows_from("force_" + side + ".csv", topic, decode_force)

    def decode_width(row):
        data[WIDTH].append((int(row["bag_receive_ns"]), float(row["width_mm"])))

    def decode_status(row):
        data[STATUS].append(
            (
                int(row["source_stamp_ns"]),
                int(row["g_po"]),
                int(row["g_pr"]),
                int(row["g_flt"]),
            )
        )

    def decode_action(row):
        data[ACTION].append(
            (
                int(row["source_stamp_ns"]),
                row["event"],
                int(row["trial_index"]),
                int(row["phase_id"]),
                float(row["target_position"]),
                float(row["target_speed"]),
                row["success"].lower() in ("true", "1"),
            )
        )

    rows_from("gripper_width.csv", WIDTH, decode_width, optional=True)
    rows_from("gripper_status.csv", STATUS, decode_status, optional=True)
    rows_from("robot_action_event.csv", ACTION, decode_action, optional=True)
    for rows in data.values():
        rows.sort(key=lambda row: row[0])
    return (
        windows,
        data,
        dict(
            export_session=str(session),
            read_errors=dict(errors),
            examples=examples,
            missing_auxiliary_csvs=missing_auxiliary,
        ),
    )


def sustained_time(headers, mask, duration_s=0.03, max_gap_s=0.005):
    begin = None
    for n, flag in enumerate(mask):
        if not flag:
            begin = None
            continue
        if begin is None or (n and headers[n] - headers[n - 1] > max_gap_s * 1e9):
            begin = n
        if headers[n] - headers[begin] >= duration_s * 1e9:
            return int(headers[begin])
    return None


def broken_line(times, values, gap_s):
    """Insert NaN only for display so lines do not bridge long time gaps."""
    x, y = [], []
    for n, (t, value) in enumerate(zip(times, values)):
        if n and t - times[n - 1] > gap_s:
            x.append(np.nan)
            y.append(np.nan)
        x.append(t)
        y.append(value)
    return x, y


def trial_signals(window, data):
    start, end = window["start_ns"], window["end_ns"]
    low = max(window["attempt"]["start_ros_ns"], start - 500_000_000)
    notes, forces, images = [], {}, {}
    contacts, detachments = [], []
    actions = [
        row
        for row in select_rows(data[ACTION], low, end)
        if row[2] == window["trial_index"]
    ]
    if not actions:
        notes.append("Action events unavailable in this export")
    if not data[WIDTH] and not data[STATUS]:
        notes.append("Gripper state unavailable in this export")
    releases = [row[0] for row in actions if row[1] == "auto_goal_sent" and row[3] == 3]
    release = releases[-1] if releases else None
    for side, topic in zip(("left", "right"), FORCE):
        rows = select_rows(data[topic], low, end)
        stamps = np.asarray([row[0] for row in rows], dtype=np.int64)
        raw = np.asarray([row[1] for row in rows], dtype=float)
        baseline = window["attempt"].get("tare", {}).get(side)
        if baseline is None or not np.isfinite(baseline):
            notes.append("No recorded tare for " + side + "; showing raw Fy")
            baseline = 0.0
            baseline_available = False
        else:
            baseline_available = True
        values = raw - baseline
        times = relative_seconds(stamps, start)
        before = values[(stamps < start) & np.isfinite(values)]
        sigma = (
            1.4826 * float(np.median(np.abs(before - np.median(before))))
            if len(before) >= 3
            else None
        )
        threshold = max(0.05, 6 * sigma) if sigma is not None else 0.05
        finite = np.isfinite(values)
        if not len(rows):
            notes.append("Missing force " + side)
        if not np.all(finite):
            notes.append("Nonfinite Fy " + side)
        valid = (stamps >= start) & finite
        contact = (
            sustained_time(stamps, valid & (np.abs(values) >= threshold))
            if baseline_available
            else None
        )
        if contact is not None:
            contacts.append(contact)
        if release is not None and baseline_available:
            before_release = (
                (stamps >= release - 250_000_000) & (stamps <= release) & finite
            )
            peak = (
                float(np.percentile(np.abs(values[before_release]), 95))
                if np.any(before_release)
                else 0.0
            )
            detach = sustained_time(
                stamps,
                (stamps >= release)
                & finite
                & (np.abs(values) <= max(threshold, 0.1 * peak)),
            )
            if detach is not None:
                detachments.append(detach)
        spikes = np.zeros(len(values), dtype=bool)
        if len(values) >= 3:
            spikes[1:-1] = (
                finite[:-2]
                & finite[1:-1]
                & finite[2:]
                & (np.abs(values[:-2] - values[2:]) < 0.5)
                & (
                    np.minimum(
                        np.abs(values[1:-1] - values[:-2]),
                        np.abs(values[1:-1] - values[2:]),
                    )
                    > 2.5
                )
            )
        forces[side] = dict(
            headers=stamps,
            times=times,
            values=values,
            spikes=spikes,
            baseline=baseline if baseline_available else None,
            baseline_sigma_n=sigma,
            threshold_n=threshold,
        )
    # At most two sides contribute to each median. Keep nanoseconds as integers.
    contact_ns = (min(contacts) + max(contacts)) // 2 if contacts else None
    detachment_ns = (min(detachments) + max(detachments)) // 2 if detachments else None
    # A common force-peak time makes the image strips directly comparable.
    peaks = [
        (abs(value), int(h))
        for force in forces.values()
        for h, value in zip(force["headers"], force["values"])
        if start <= h <= end and np.isfinite(value)
    ]
    peak_time = max(peaks)[1] if peaks else start
    for side, topic in zip(("left", "right"), GS):
        rows = select_rows(data[topic], low, end)
        if not rows:
            notes.append("Missing GelSight " + side)
            images[side] = dict(times=np.array([]), values=np.array([]), frames=[])
            continue
        references = [row for row in rows if row[0] < start][-5:]
        if not references and contact_ns is not None:
            references = [row for row in rows if row[0] < contact_ns][-5:]
            if references:
                notes.append(
                    "GS " + side + " reference uses frames before estimated contact"
                )
        if not references:
            references = rows[:5]
            notes.append(
                "GS "
                + side
                + " reference uses first trial frames; unloaded state unverified"
            )
        reference = np.median(np.stack([row[1] for row in references]), axis=0)
        images[side] = dict(
            times=relative_seconds([row[0] for row in rows], start),
            values=np.asarray(
                [np.mean(np.abs(row[1] - reference)) for row in rows], dtype=float
            ),
            frames=[
                (label, min(rows, key=lambda row: abs(row[0] - stamp)))
                for label, stamp in [
                    ("reference", references[-1][0]),
                    ("force peak", peak_time),
                    ("end", end),
                ]
            ],
        )
    return dict(
        forces=forces,
        images=images,
        width=select_rows(data[WIDTH], low, end),
        status=select_rows(data[STATUS], low, end),
        actions=actions,
        notes=notes,
        contact_ns=contact_ns,
        detachment_ns=detachment_ns,
    )


def phase_label(phase):
    if phase == 1:
        return "prepare"
    if phase == 2:
        return "press"
    if phase == 3:
        return "release"
    return "deep" if phase % 2 == 0 else "shallow"


def draw_trial(window, signals, output, title, absolute):
    figure = plt.figure(figsize=(14, 11))
    try:
        grid = figure.add_gridspec(4, 6, height_ratios=[1.25, 1.0, 0.95, 0.9])
        axes = [figure.add_subplot(grid[n, :]) for n in range(3)]
        for ax in axes[1:]:
            ax.sharex(axes[0])
        start = window["start_ns"]
        for side, color in zip(("left", "right"), SIDE_COLORS):
            force = signals["forces"][side]
            values = np.abs(force["values"]) if absolute else force["values"]
            x, y = broken_line(force["times"], values, 0.005)
            label = side + (" raw Fy" if force["baseline"] is None else "")
            axes[0].plot(x, y, color=color, lw=0.8, label=label)
            spike = force["spikes"]
            if np.any(spike):
                axes[0].scatter(
                    force["times"][spike],
                    values[spike],
                    color="crimson",
                    marker="x",
                    s=26,
                )
            image = signals["images"][side]
            x, y = broken_line(image["times"], image["values"], 0.1)
            axes[1].plot(x, y, color=color, lw=1.0, label=side)
        axes[0].set_ylabel(("|delta Fy|" if absolute else "delta Fy") + " [N]")
        axes[1].set_ylabel("GelSight mean abs difference\n[gray levels, 56x56 proxy]")
        if signals["width"]:
            headers, values = zip(*signals["width"])
            axes[2].plot(
                relative_seconds(headers, start),
                values,
                color="tab:green",
                lw=1.0,
                label="reported jaw mm (bag time)",
            )
        goals = [row for row in signals["actions"] if row[1] == "auto_goal_sent"]
        if goals:
            axes[2].step(
                relative_seconds([row[0] for row in goals] + [window["end_ns"]], start),
                [row[4] * 1000 for row in goals] + [goals[-1][4] * 1000],
                where="post",
                color="black",
                ls="--",
                lw=0.9,
                label="commanded jaw mm",
            )
        if not signals["width"] and not signals["status"] and not goals:
            axes[2].text(
                0.5,
                0.5,
                "Gripper/action records unavailable in this export",
                ha="center",
                va="center",
                transform=axes[2].transAxes,
            )
        if signals["status"]:
            twin = axes[2].twinx()
            twin.plot(
                relative_seconds([row[0] for row in signals["status"]], start),
                [row[1] for row in signals["status"]],
                color="gray",
                alpha=0.5,
                lw=0.6,
            )
            twin.set_ylabel("g_po [raw count]", color="gray")
        axes[2].set_ylabel("Jaw width [reported mm]")
        axes[2].set_xlabel(
            "Seconds from TrialEvent START (Header stamps, jaw mm uses bag receive time)"
        )
        for ax in axes:
            ax.grid(alpha=0.25)
            ax.axvline(0, color="black", ls=":", lw=0.8)
            if ax.get_legend_handles_labels()[0]:
                ax.legend(loc="upper right", fontsize=8)
            for name, style in (("contact_ns", "--"), ("detachment_ns", ":")):
                if signals[name] is not None:
                    ax.axvline(
                        (signals[name] - start) / 1e9, color="purple", ls=style, lw=1.0
                    )
        # Shade commanded motion/hold intervals; these do not define physical contact.
        pending = {}
        for row in signals["actions"]:
            h, event, _, phase, _, _, _ = row
            if event in ("auto_goal_sent", "auto_dwell_start"):
                pending[phase, event] = h
            elif event in ("auto_action_succeeded", "auto_dwell_end"):
                first = (
                    "auto_goal_sent"
                    if event == "auto_action_succeeded"
                    else "auto_dwell_start"
                )
                begin = pending.pop((phase, first), None)
                if begin is not None:
                    for ax in axes:
                        ax.axvspan(
                            (begin - start) / 1e9,
                            (h - start) / 1e9,
                            color="steelblue" if first == "auto_goal_sent" else "gold",
                            alpha=0.09,
                        )
        for row in goals:
            axes[2].axvline((row[0] - start) / 1e9, color="black", alpha=0.2, lw=0.5)
            axes[2].text(
                (row[0] - start) / 1e9,
                0.97,
                phase_label(row[3]),
                transform=axes[2].get_xaxis_transform(),
                rotation=90,
                va="top",
                fontsize=7,
            )
        for n, side in enumerate(("left", "right")):
            for j in range(3):
                ax = figure.add_subplot(grid[3, n * 3 + j])
                frames = signals["images"][side]["frames"]
                if frames:
                    label, row = frames[j]
                    ax.imshow(row[2])
                    ax.set_title(
                        f"{side}: {label}\nt={(row[0]-start)/1e9:.3f}s", fontsize=8
                    )
                else:
                    ax.text(0.5, 0.5, side + "\nno image", ha="center", va="center")
                ax.axis("off")
        figure.suptitle(
            "\n".join(textwrap.fill(line, 115) for line in title.splitlines()),
            fontsize=11,
        )
        note = (
            "Unsmoothed Wrench with recorded tare. Red x: spike candidate. Blank line segments: time gaps. "
            "Purple --/: force-threshold contact/detachment estimates. Blue/gold: command/hold intervals.\n"
            "GS proxy and jaw mm are diagnostic quantities. Source timestamps and Wrench samples are preserved for this visualization."
        )
        if signals["notes"]:
            note += "\n" + ". ".join(signals["notes"])
        figure.text(0.015, 0.008, note, fontsize=7)
        figure.tight_layout(rect=[0, 0.065, 1, 0.94])
        figure.savefig(output, dpi=120)
    finally:
        plt.close(figure)


def draw_comparison(rows, output, title, absolute):
    positions = list(dict.fromkeys(row["position"] for row in rows))
    figure, axes = plt.subplots(
        len(positions),
        2,
        squeeze=False,
        figsize=(15, 3 * len(positions) + 1),
        sharex=True,
        sharey="col",
    )
    try:
        for n, position in enumerate(positions):
            matched = [row for row in rows if row["position"] == position]
            for j, row in enumerate(matched):
                color = plt.get_cmap("tab10")(j % 10)
                for side, style in (("left", "-"), ("right", "--")):
                    force = row["forces"][side]
                    values = np.abs(force["values"]) if absolute else force["values"]
                    x, y = broken_line(force["times"], values, 0.005)
                    label = row["trial_id"] + " " + side
                    axes[n, 0].plot(x, y, color=color, ls=style, lw=0.7, label=label)
                    image = row["images"][side]
                    x, y = broken_line(image["times"], image["values"], 0.1)
                    axes[n, 1].plot(x, y, color=color, ls=style, lw=0.9, label=label)
            axes[n, 0].set_ylabel(
                position + "\n" + ("|delta Fy|" if absolute else "delta Fy") + " [N]"
            )
            axes[n, 1].set_ylabel(position + "\nGS difference proxy")
            for ax in axes[n]:
                ax.axvline(0, color="black", ls=":", lw=0.8)
                ax.grid(alpha=0.25)
                ax.legend(fontsize=7, loc="upper right")
        for ax in axes[-1]:
            ax.set_xlabel("Seconds from TrialEvent START, actual elapsed time")
        figure.suptitle(
            textwrap.fill(title, 115)
            + "\nSame full condition | color=trial, solid=left, dashed=right",
            fontsize=11,
        )
        figure.tight_layout(rect=[0, 0, 1, 0.92])
        figure.savefig(output, dpi=120)
    finally:
        plt.close(figure)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--export-dir",
        type=Path,
        required=True,
        help="extract_all output directory, or one exported session",
    )
    parser.add_argument("--out", type=Path, required=True, help="New plot directory")
    parser.add_argument("--position", choices=("base", "front", "back", "up"))
    parser.add_argument(
        "--trial-id", action="append", help="Exact trial ID; repeatable"
    )
    parser.add_argument("--max-trials", type=int)
    parser.add_argument("--absolute-force", action="store_true")
    args = parser.parse_args(argv)
    if args.out.exists():
        parser.error("--out exists; choose a new directory")
    if args.max_trials is not None and args.max_trials < 1:
        parser.error("--max-trials must be positive")
    source, output = args.export_dir.expanduser().resolve(), args.out.resolve()
    if output == source or source in output.parents:
        parser.error("Output must be outside the input export directory")
    paths = (
        [source / "manifest.json"]
        if (source / "manifest.json").is_file()
        else sorted(source.glob("*/manifest.json"))
    )
    if not paths:
        parser.error(
            "No exported session manifests. Select the directory produced by extract_all --write"
        )
    selections, skipped, seen = [], [], set()
    remaining = args.max_trials
    for path in paths:
        manifest = json.loads(path.read_text())
        if manifest.get("status") != "complete":
            skipped.append(
                dict(export_session=str(path.parent), status=manifest.get("status"))
            )
            continue
        windows = [
            window
            for window in manifest["windows"]
            if (not args.position or window["position_id"] == args.position)
            and (not args.trial_id or window["trial_id"] in args.trial_id)
        ]
        if remaining is not None:
            windows = windows[:remaining]
            remaining -= len(windows)
        if not windows:
            continue
        object_id, job_id = export_identity(path.parent, manifest)
        for window in windows:
            key = (object_id, job_id, window["trial_id"])
            if key in seen:
                parser.error("Duplicate exported trial: " + str(key))
            seen.add(key)
        selections.append((path.parent, windows, object_id, job_id))
    if not selections:
        parser.error("No completed exported trials match filters")
    if args.trial_id and not set(args.trial_id) <= {
        w["trial_id"] for _, ws, _, _ in selections for w in ws
    }:
        parser.error("Requested trial IDs not present after filters")
    output.mkdir(parents=True)
    if (source / "source_progress.json").is_file():
        (output / "source_progress.json").write_bytes(
            (source / "source_progress.json").read_bytes()
        )
    rows, comparisons = [], defaultdict(list)
    report = dict(
        schema_version=2,
        status="running",
        input_format="extract_all",
        export_dir=str(source),
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        force_display="absolute" if args.absolute_force else "signed",
        trials=rows,
        sessions=[],
        skipped_exports=skipped,
        purpose="Visual inspection; plot generation success is not a quality pass",
        read_only_sources=True,
        smoothing=False,
        timestamp_correction=False,
        raw_packet_recovery=False,
        original_bags_read=False,
    )
    atomic_json(output / "report.json", report)
    started = time.monotonic()
    print("Exports: " + str(source), flush=True)
    try:
        for n, (session, selected, object_id, job_id) in enumerate(selections, 1):
            print(
                f"[{n}/{len(selections)}] Plotting {session.name} ({len(selected)} trials)",
                flush=True,
            )
            try:
                windows, data, diagnostics = read_export_session(session, selected)
                report["sessions"].append(diagnostics)
                for window in windows:
                    kind, identity, label, condition = condition_identity(
                        window["conditions"]
                    )
                    directory = (
                        output / "trials" / safe_name(object_id) / kind / identity
                    )
                    directory.mkdir(parents=True, exist_ok=True)
                    path = directory / (
                        safe_name(job_id)
                        + "__"
                        + safe_name(window["trial_id"])
                        + ".png"
                    )
                    signals = trial_signals(window, data)
                    draw_trial(
                        window,
                        signals,
                        path,
                        f"{object_id} | {job_id} | {window['trial_id']} | repeat={window['conditions'].get('repeat')}\n{label}",
                        args.absolute_force,
                    )
                    row = dict(
                        trial_id=window["trial_id"],
                        object_id=object_id,
                        job_id=job_id,
                        position=window["position_id"],
                        kind=kind,
                        condition=condition,
                        condition_id=identity,
                        plot=str(path.relative_to(output)),
                        export_session=str(session),
                        start_ns=window["start_ns"],
                        end_ns=window["end_ns"],
                        contact_estimate_s=(
                            (signals["contact_ns"] - window["start_ns"]) / 1e9
                            if signals["contact_ns"]
                            else None
                        ),
                        detachment_estimate_s=(
                            (signals["detachment_ns"] - window["start_ns"]) / 1e9
                            if signals["detachment_ns"]
                            else None
                        ),
                        notes=signals["notes"],
                        force_statistics={
                            side: dict(
                                samples=len(v["times"]),
                                baseline_Fy=v["baseline"],
                                pre_start_MAD_sigma_n=v["baseline_sigma_n"],
                                spike_candidates=int(np.count_nonzero(v["spikes"])),
                            )
                            for side, v in signals["forces"].items()
                        },
                    )
                    rows.append(row)
                    comparisons[object_id, job_id, kind, identity, label].append(
                        dict(
                            trial_id=window["trial_id"],
                            position=window["position_id"],
                            forces=signals["forces"],
                            images={
                                side: dict(times=v["times"], values=v["values"])
                                for side, v in signals["images"].items()
                            },
                        )
                    )
                    atomic_json(output / "report.json", report)
            except Exception as exc:
                report["sessions"].append(
                    dict(
                        export_session=str(session),
                        error=type(exc).__name__ + ": " + str(exc),
                    )
                )
        report["comparison_plots"] = []
        for (object_id, job_id, kind, identity, label), traces in comparisons.items():
            directory = (
                output / "comparisons" / safe_name(object_id) / safe_name(job_id) / kind
            )
            directory.mkdir(parents=True, exist_ok=True)
            path = directory / (identity + ".png")
            draw_comparison(
                traces, path, object_id + " | " + label, args.absolute_force
            )
            report["comparison_plots"].append(str(path.relative_to(output)))
    except KeyboardInterrupt:
        report["interrupted"] = True
    except Exception as exc:
        report["error"] = type(exc).__name__ + ": " + str(exc)
    report["expected_trials"] = sum(len(windows) for _, windows, _, _ in selections)
    report["plotted_trials"] = len(rows)
    report["status"] = (
        "incomplete"
        if (
            report.get("interrupted")
            or report.get("error")
            or skipped
            or len(rows) != report["expected_trials"]
            or any(s.get("error") or s.get("read_errors") for s in report["sessions"])
        )
        else "plots_generated"
    )
    report["trials_with_notes"] = sum(bool(row["notes"]) for row in rows)
    report["elapsed_seconds"] = time.monotonic() - started
    atomic_json(output / "report.json", report)
    with (output / "trial_plot_index.csv").open("x", newline="") as stream:
        fields = [
            "object_id",
            "job_id",
            "trial_id",
            "position",
            "kind",
            "condition_id",
            "plot",
            "export_session",
            "contact_estimate_s",
            "detachment_estimate_s",
            "notes",
        ]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(
            {
                key: json.dumps(row[key]) if key == "notes" else row[key]
                for key in fields
            }
            for row in rows
        )
    print(
        f"{report['status']}: {len(rows)}/{report['expected_trials']} trial PNGs; {len(report.get('comparison_plots', []))} comparisons",
        flush=True,
    )
    print("Report: " + str(output / "report.json"), flush=True)
    return 0 if report["status"] == "plots_generated" else 1


if __name__ == "__main__":
    raise SystemExit(main())
