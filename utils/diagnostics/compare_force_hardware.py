#!/usr/bin/env python3
"""Compare matched old/new extract_all exports, without reading bags or rewriting data."""
import argparse
from collections import defaultdict
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np


def load(path):
    return json.loads(path.read_text())


def fingerprint(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def finite_json(value):
    if isinstance(value, dict):
        return {k: finite_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_json(v) for v in value]
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(value) else None
    if isinstance(value, np.integer):
        return int(value)
    return value


def canonical(condition):
    kind = condition["kind"]
    result = dict(
        kind=kind,
        preparation_speed=float(condition.get("preparation_speed", 0.3)),
        release_speed=float(condition.get("release_speed", 0.3)),
        trial_open_clearance_mm=condition.get("trial_open_clearance_mm"),
    )
    if result["trial_open_clearance_mm"] is not None:
        result["trial_open_clearance_mm"] = float(result["trial_open_clearance_mm"])
    names = (
        ("depth_mm", "speed", "hold_s")
        if kind == "run"
        else (
            "shallow_mm",
            "deep_mm",
            "loading_speed",
            "unloading_speed",
            "deep_hold_s",
            "shallow_hold_s",
        )
    )
    if kind not in ("run", "knead"):
        raise ValueError("Unsupported action kind: " + kind)
    for name in names:
        result[name] = float(condition[name])
    if kind == "knead":
        result["cycles"] = int(condition["cycles"])
    result["variant"] = condition.get(
        "variant", "depth_speed" if kind == "run" else "knead"
    )
    return result


def condition_label(spec):
    c = spec["condition"]
    if c["kind"] == "run":
        text = f"press {c['depth_mm']:g}mm speed={c['speed']:g} hold={c['hold_s']:g}s"
    else:
        text = f"knead {c['shallow_mm']:g}-{c['deep_mm']:g}mm cycles={c['cycles']} speed={c['loading_speed']:g}/{c['unloading_speed']:g} hold={c['deep_hold_s']:g}/{c['shallow_hold_s']:g}s"
    return spec["position"] + " | " + text


def group_key(window):
    # Repeat/trial IDs and init width are deliberately not matching keys.
    # Width is an observed covariate exported separately.
    return json.dumps(
        dict(position=window["position_id"], condition=canonical(window["conditions"])),
        sort_keys=True,
    )


def discover(root, wanted_job):
    manifests = (
        [root / "manifest.json"]
        if (root / "manifest.json").is_file()
        else sorted(
            set(root.glob("*/manifest.json")) | set(root.glob("*/*/manifest.json"))
        )
    )
    trials, skipped, jobs, objects = [], [], set(), set()
    seen = set()
    for path in manifests:
        manifest = load(path)
        if manifest.get("status") != "complete":
            skipped.append(str(path))
            continue
        progress_file = path.parent.parent / "source_progress.json"
        progress = load(progress_file) if progress_file.is_file() else {}
        if not progress.get("job_id"):
            raise ValueError(
                "Missing copied source_progress.json beside export sessions: "
                + str(path)
            )
        job = progress["job_id"]
        if wanted_job and job != wanted_job:
            continue
        references = {
            p["id"]: p.get("reference") or {} for p in progress["plan"]["positions"]
        }
        for window in manifest["windows"]:
            object_id = Path(window["attempt"]["session"]).parent.name
            jobs.add(job)
            objects.add(object_id)
            identity = (job, window["trial_id"])
            if identity in seen:
                raise ValueError(
                    "Duplicate exported trial; choose a narrower export directory: "
                    + str(identity)
                )
            seen.add(identity)
            trials.append(
                dict(
                    window=window,
                    session=path.parent,
                    manifest=path,
                    progress=progress_file,
                    job_id=job,
                    object_id=object_id,
                    plan_settings=progress["plan"].get("settings", {}),
                    reference_width_mm=(
                        references[window["position_id"]]["width_m"] * 1000
                        if references.get(window["position_id"], {}).get("width_m")
                        is not None
                        and window["attempt"].get("init_id")
                        and Path(
                            references[window["position_id"]].get("init_path", "")
                        ).name
                        == window["attempt"]["init_id"]
                        else None
                    ),
                )
            )
    if len(jobs) != 1:
        raise ValueError(
            "Select exactly one job with --old-job/--new-job or a narrower export directory: "
            + str(sorted(jobs))
        )
    if len(objects) != 1:
        raise ValueError(
            "Select exactly one physical object per input: " + str(sorted(objects))
        )
    return trials, skipped


def hampel(times, values, window, multiplier, floor_n, gap_s):
    """Candidate mask only; no interpolation/replacement. Do not span timing gaps."""
    mask = np.zeros(len(values), dtype=bool)
    medians = np.full(len(values), np.nan)
    thresholds = np.full(len(values), np.nan)
    if not len(values):
        return mask, medians, thresholds
    breaks = np.r_[
        0,
        np.flatnonzero((np.diff(times) > gap_s) | (np.diff(times) <= 0)) + 1,
        len(values),
    ]
    for low, high in zip(breaks, breaks[1:]):
        raw = values[low:high]
        if len(raw) < window:
            continue
        views = np.lib.stride_tricks.sliding_window_view(raw, window)
        # Bounded temporary allocations even for a long continuous recording.
        for offset in range(0, len(views), 20000):
            chunk = views[offset : offset + 20000]
            with np.errstate(invalid="ignore"):
                med = np.median(chunk, axis=1)
                mad = np.median(np.abs(chunk - med[:, None]), axis=1)
            centers = np.arange(len(chunk)) + low + window // 2 + offset
            limit = np.maximum(floor_n, multiplier * 1.4826 * mad)
            medians[centers], thresholds[centers] = med, limit
            mask[centers] = np.isfinite(values[centers]) & (
                np.abs(values[centers] - med) > limit
            )
    return mask, medians, thresholds


def read_samples(descriptors, sources):
    by_session = defaultdict(dict)
    for descriptor in descriptors:
        by_session[descriptor["session"]][descriptor["window"]["trial_id"]] = descriptor
    for session, wanted in by_session.items():
        actions = defaultdict(list)
        action_path = session / "robot_action_event.csv"
        if action_path.is_file():
            sources[str(action_path)] = fingerprint(action_path)
            with action_path.open(newline="") as stream:
                for row in csv.DictReader(stream):
                    if row["trial_id"] in wanted:
                        actions[row["trial_id"]].append(row)
        for side in ("left", "right"):
            path = session / ("force_" + side + ".csv")
            sources[str(path)] = fingerprint(path)
            samples = defaultdict(list)
            with path.open(newline="") as stream:
                for row in csv.DictReader(stream):
                    if row["trial_id"] not in wanted:
                        continue
                    descriptor = wanted[row["trial_id"]]
                    window = descriptor["window"]
                    header = int(row["source_stamp_ns"])
                    if not window["start_ns"] <= header <= window["end_ns"]:
                        continue
                    baseline = window["attempt"].get("tare", {}).get(side)
                    if baseline is None or not math.isfinite(baseline):
                        raise ValueError(
                            "Missing finite recorded tare: "
                            + row["trial_id"]
                            + " "
                            + side
                        )
                    raw = float(row["Fy"])
                    if row.get("baseline_Fy") not in ("", None) and not math.isclose(
                        float(row["baseline_Fy"]), baseline, rel_tol=0, abs_tol=1e-9
                    ):
                        raise ValueError("CSV/manifest baseline mismatch: " + str(path))
                    if (
                        math.isfinite(raw)
                        and row.get("baseline_relative_Fy") not in ("", None)
                        and not math.isclose(
                            float(row["baseline_relative_Fy"]),
                            raw - baseline,
                            rel_tol=0,
                            abs_tol=1e-9,
                        )
                    ):
                        raise ValueError(
                            "CSV baseline-relative Fy mismatch: " + str(path)
                        )
                    samples[row["trial_id"]].append((header, raw - baseline))
            for trial_id, descriptor in wanted.items():
                data = samples[trial_id]
                if not data:
                    raise ValueError("No force data: " + trial_id + " " + side)
                descriptor.setdefault("forces", {})[side] = data
                descriptor["actions"] = actions[trial_id]
                descriptor["actions_available"] = action_path.is_file()


def hold_windows(descriptor):
    result, active = [], {}
    kind = descriptor["window"]["conditions"]["kind"]
    for row in sorted(descriptor["actions"], key=lambda r: int(r["source_stamp_ns"])):
        phase = int(row["phase_id"])
        if not ((kind == "run" and phase == 2) or (kind == "knead" and phase >= 200)):
            continue
        stamp = int(row["source_stamp_ns"])
        if row["event"] == "auto_dwell_start":
            if phase in active:
                raise ValueError("Duplicate hold start")
            active[phase] = stamp
        elif row["event"] == "auto_dwell_end":
            if phase not in active:
                raise ValueError("Hold end without start")
            start = active.pop(phase)
            if stamp <= start:
                raise ValueError("Invalid hold duration")
            label = (
                "press"
                if kind == "run"
                else f"cycle_{(phase-200)//2+1:02d}_"
                + ("deep" if phase % 2 == 0 else "shallow")
            )
            result.append((label, start, stamp))
    if active:
        raise ValueError("Unpaired hold event")
    return result


def interval_median(headers, values, keep, low, high, dt_ns):
    selected = (headers >= low) & (headers <= high)
    valid = selected & keep
    expected = max(1.0, (high - low) / dt_ns)
    coverage = min(1.0, int(valid.sum()) / expected)
    return (
        float(np.median(values[valid]))
        if valid.sum() >= 10 and coverage >= 0.8
        else None
    ), coverage


def analyze(descriptor, dataset, args, candidates):
    window = descriptor["window"]
    start = window["start_ns"]
    holds = hold_windows(descriptor)
    descriptor["hold_count"] = len(holds)
    descriptor["expected_hold_count"] = (
        1
        if window["conditions"]["kind"] == "run"
        else 2 * int(window["conditions"]["cycles"])
    )
    metrics, phases = [], []
    for side, points in descriptor["forces"].items():
        headers = np.array([r[0] for r in points], dtype=np.int64)
        values = np.array([r[1] for r in points], dtype=float)
        times = np.array([(int(h) - start) / 1e9 for h in headers])
        spikes, rolling, limits = hampel(
            times,
            values,
            args.hampel_window,
            args.hampel_sigma,
            args.spike_floor_n,
            0.005,
        )
        valid = np.isfinite(values) & ~spikes
        descriptor.setdefault("series", {})[side] = dict(
            times=times, values=values, mask=spikes, valid=valid
        )
        differences = np.diff(headers)
        positive = differences[(differences > 0) & (differences <= 5_000_000)]
        dt_ns = float(np.median(positive)) if len(positive) else 1_000_000.0
        metrics.append(
            dict(
                dataset=dataset,
                job_id=descriptor["job_id"],
                trial_id=window["trial_id"],
                group=descriptor["group"],
                position=window["position_id"],
                side=side,
                reference_width_mm=descriptor["reference_width_mm"],
                recorded_tare_Fy_n=window["attempt"]["tare"][side],
                init_id=window["attempt"].get("init_id"),
                source_session=window["attempt"]["session"],
                samples=len(values),
                candidate_samples=int(spikes.sum()),
                candidate_fraction=float(spikes.mean()),
                invalid=int((~np.isfinite(values)).sum()),
                header_gaps_over_5ms=int((differences > 5_000_000).sum()),
                max_header_gap_ms=float(max(differences, default=0) / 1e6),
                header_nonincreasing=int((differences <= 0).sum()),
                raw_abs_peak_n=(
                    float(np.max(np.abs(values[np.isfinite(values)])))
                    if np.isfinite(values).any()
                    else None
                ),
                retained_abs_p95_n=(
                    float(np.percentile(np.abs(values[valid]), 95))
                    if valid.any()
                    else None
                ),
                retained_over_30n_samples=int((valid & (np.abs(values) > 30.0)).sum()),
                action_events_available=descriptor["actions_available"],
                hold_count=len(holds),
                expected_hold_count=descriptor["expected_hold_count"],
            )
        )
        for index in np.flatnonzero(spikes):
            candidates.append(
                dict(
                    dataset=dataset,
                    trial_id=window["trial_id"],
                    side=side,
                    source_stamp_ns=int(headers[index]),
                    baseline_relative_Fy_n=float(values[index]),
                    local_median_n=float(rolling[index]),
                    candidate_threshold_n=float(limits[index]),
                )
            )
        for label, low, high in holds:
            duration = (high - low) / 1e9
            # Identical early/late definitions for both datasets, based on actual events.
            edge = min(20_000_000, int((high - low) * 0.1))
            width = int((high - low) * 0.25)
            early, ce = interval_median(
                headers, values, valid, low + edge, low + edge + width, dt_ns
            )
            late, cl = interval_median(
                headers, values, valid, high - width, high, dt_ns
            )
            phases.append(
                dict(
                    dataset=dataset,
                    group=descriptor["group"],
                    trial_id=window["trial_id"],
                    side=side,
                    phase=label,
                    duration_s=duration,
                    early_Fy_n=early,
                    late_Fy_n=late,
                    early_coverage=ce,
                    late_coverage=cl,
                    relaxation_fraction=(
                        (abs(early) - abs(late)) / abs(early)
                        if early is not None and late is not None and abs(early) >= 0.2
                        else None
                    ),
                )
            )
    return metrics, phases


def write_csv(path, rows):
    fields = list(dict.fromkeys(k for row in rows for k in row))
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(finite_json(rows))


def plot_group(path, key, descriptors, phases):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    spec = json.loads(key)
    condition = spec["condition"]
    fig, axes = plt.subplots(3, 2, figsize=(15, 11))
    colors = {"old": "tab:red", "new": "tab:blue"}
    try:
        for col, side in enumerate(("left", "right")):
            for dataset in ("old", "new"):
                selected = [d for d in descriptors if d["dataset"] == dataset]
                for index, d in enumerate(selected):
                    values = d["series"][side]
                    t = values["times"]
                    raw = values["values"]
                    for row in (0, 1):
                        y = raw.copy()
                        if row == 1:
                            y[~values["valid"]] = np.nan
                        # Display breaks only: do not bridge invalid samples or timing gaps.
                        x, y_out = [], []
                        for n, (a, b) in enumerate(zip(t, y)):
                            if n and (a - t[n - 1] > 0.005 or a <= t[n - 1]):
                                x.append(np.nan)
                                y_out.append(np.nan)
                            x.append(a)
                            y_out.append(b)
                        axes[row, col].plot(
                            x,
                            y_out,
                            color=colors[dataset],
                            alpha=0.55,
                            linewidth=0.7,
                            label=(
                                dataset + f" (n={len(selected)})"
                                if index == 0
                                else None
                            ),
                        )
            axes[0, col].set_title(side + " | raw, full amplitude")
            axes[1, col].set_title(
                side + " | Hampel candidates masked; no interpolation or low-pass"
            )
            for row in (0, 1):
                axes[row, col].set_ylabel("Recorded-tare-relative Fy [N]")
                axes[row, col].set_xlabel(
                    "Seconds from TrialEvent START (no time warping)"
                )
                axes[row, col].legend()
            relevant = [p for p in phases if p["side"] == side]
            labels = sorted({p["phase"] for p in relevant})
            for dataset, offset in [("old", -0.12), ("new", 0.12)]:
                for n, label in enumerate(labels):
                    vals = [
                        p["late_Fy_n"]
                        for p in relevant
                        if p["dataset"] == dataset
                        and p["phase"] == label
                        and p["late_Fy_n"] is not None
                    ]
                    if vals:
                        axes[2, col].scatter(
                            np.full(len(vals), n + offset),
                            vals,
                            color=colors[dataset],
                            s=20,
                            alpha=0.65,
                            label=dataset if n == 0 else None,
                        )
            axes[2, col].set_xticks(range(len(labels)), labels, rotation=45, ha="right")
            axes[2, col].set_ylabel("Late-hold median Fy [N] per trial")
            availability = {
                dataset: sum(
                    d["hold_count"] > 0 for d in descriptors if d["dataset"] == dataset
                )
                for dataset in ("old", "new")
            }
            axes[2, col].set_title(
                "Event-based holds: old="
                + str(availability["old"])
                + ", new="
                + str(availability["new"])
                + " trials"
            )
            if not all(availability.values()):
                axes[2, col].text(
                    0.02,
                    0.03,
                    "Missing events: do not infer old/new hold equivalence",
                    transform=axes[2, col].transAxes,
                    fontsize=9,
                )
            for ax in axes[:, col]:
                ax.grid(alpha=0.2)
        fig.suptitle(
            condition_label(spec)
            + f" | prep={condition['preparation_speed']:g}, release={condition['release_speed']:g}, clearance={condition['trial_open_clearance_mm']} mm"
            + "\nRed=old; blue=new. Diagnostic comparison, not a calibration/equivalence certificate.",
            fontsize=10,
        )
        fig.tight_layout(rect=(0, 0, 1, 0.94))
        fig.savefig(path, dpi=130)
    finally:
        plt.close(fig)


def summarize_phases(rows):
    groups = defaultdict(dict)
    for row in rows:
        key = (row["group"], row["side"], row["phase"])
        groups[key].setdefault(row["dataset"], []).append(row)
    output = []
    for (group, side, phase), datasets in sorted(groups.items()):
        for metric in ("early_Fy_n", "late_Fy_n", "relaxation_fraction"):
            row = dict(group=group, side=side, phase=phase, metric=metric)
            medians = {}
            for dataset in ("old", "new"):
                values = [
                    r[metric]
                    for r in datasets.get(dataset, [])
                    if r[metric] is not None
                ]
                row[dataset + "_n_trials"] = len(values)
                row[dataset + "_median"] = float(np.median(values)) if values else None
                row[dataset + "_min"] = min(values) if values else None
                row[dataset + "_max"] = max(values) if values else None
                medians[dataset] = row[dataset + "_median"]
            old, new = medians["old"], medians["new"]
            row["new_minus_old"] = (
                new - old if old is not None and new is not None else None
            )
            row["relative_difference"] = (
                (new - old) / abs(old)
                if old is not None
                and new is not None
                and metric != "relaxation_fraction"
                and abs(old) >= 0.2
                else None
            )
            row["absolute_force_change_n"] = (
                abs(new) - abs(old)
                if old is not None
                and new is not None
                and metric != "relaxation_fraction"
                else None
            )
            row["absolute_force_relative_change"] = (
                (abs(new) - abs(old)) / abs(old)
                if old is not None
                and new is not None
                and metric != "relaxation_fraction"
                and abs(old) >= 0.2
                else None
            )
            output.append(row)
    return output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--old-export", type=Path, required=True)
    parser.add_argument("--new-export", type=Path, required=True)
    parser.add_argument("--old-job")
    parser.add_argument("--new-job")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument(
        "--hampel-window",
        type=int,
        default=21,
        help="Samples, not milliseconds; nominal ~21 ms at 1000 Hz",
    )
    parser.add_argument("--hampel-sigma", type=float, default=6.0)
    parser.add_argument("--spike-floor-n", type=float, default=2.5)
    args = parser.parse_args(argv)
    if args.out.exists():
        parser.error("--out exists; choose a new directory")
    if (
        args.hampel_window < 3
        or args.hampel_window % 2 != 1
        or args.hampel_window > 501
    ):
        parser.error("--hampel-window must be odd, 3..501")
    for value in (args.hampel_sigma, args.spike_floor_n):
        if not math.isfinite(value) or value <= 0:
            parser.error("Hampel thresholds must be finite and positive")
    try:
        old, old_skipped = discover(args.old_export, args.old_job)
        new, new_skipped = discover(args.new_export, args.new_job)
        if old[0]["object_id"] != new[0]["object_id"]:
            raise ValueError("Different object IDs; cannot automatically compare")
        old_groups = {group_key(d["window"]) for d in old}
        new_groups = {group_key(d["window"]) for d in new}
        matched = old_groups & new_groups
        if not matched:
            raise ValueError("No exactly matched positions/action conditions")
        old = [d for d in old if group_key(d["window"]) in matched]
        new = [d for d in new if group_key(d["window"]) in matched]
        sources = {}
        for dataset, descriptors in [("old", old), ("new", new)]:
            for d in descriptors:
                d["dataset"] = dataset
                d["key"] = group_key(d["window"])
                d["group"] = hashlib.sha256(d["key"].encode()).hexdigest()[:12]
                for p in (d["manifest"], d["progress"]):
                    sources[str(p)] = fingerprint(p)
            read_samples(descriptors, sources)
    except (OSError, ValueError, KeyError) as exc:
        parser.error(str(exc))
    args.out.mkdir(parents=True)
    candidates, metrics, phases = [], [], []
    groups = defaultdict(list)
    for descriptor in old + new:
        a, b = analyze(descriptor, descriptor["dataset"], args, candidates)
        metrics.extend(a)
        phases.extend(b)
        groups[descriptor["key"]].append(descriptor)
    (args.out / "plots").mkdir()
    records = []
    for n, (key, descriptors) in enumerate(sorted(groups.items()), 1):
        group = descriptors[0]["group"]
        spec = json.loads(key)
        c = spec["condition"]
        depth = (
            f"{c['depth_mm']:g}mm"
            if c["kind"] == "run"
            else f"{c['shallow_mm']:g}-{c['deep_mm']:g}mm"
        )
        target = (
            args.out
            / "plots"
            / (spec["position"] + "_" + c["kind"] + "_" + depth + "_" + group + ".png")
        )
        selected = [p for p in phases if p["group"] == group]
        plot_group(target, key, descriptors, selected)
        records.append(
            dict(
                group=group,
                label=condition_label(json.loads(key)),
                **json.loads(key),
                plot="plots/" + target.name,
                old_trials=[
                    d["window"]["trial_id"]
                    for d in descriptors
                    if d["dataset"] == "old"
                ],
                new_trials=[
                    d["window"]["trial_id"]
                    for d in descriptors
                    if d["dataset"] == "new"
                ],
            )
        )
        print(f"[{n}/{len(groups)}] {group}", flush=True)
    write_csv(
        args.out / "comparison_index.csv",
        [
            dict(
                group=r["group"],
                label=r["label"],
                plot=r["plot"],
                old_trials=len(r["old_trials"]),
                new_trials=len(r["new_trials"]),
            )
            for r in records
        ],
    )
    write_csv(args.out / "trial_metrics.csv", metrics)
    write_csv(args.out / "hold_metrics.csv", phases)
    write_csv(args.out / "group_differences.csv", summarize_phases(phases))
    write_csv(args.out / "spike_candidates.csv", candidates)
    report = dict(
        status="comparison_generated",
        object_id=old[0]["object_id"],
        old_job=old[0]["job_id"],
        new_job=new[0]["job_id"],
        matched_groups=records,
        selected_old_trials=len(old),
        selected_new_trials=len(new),
        old_plan_settings=old[0]["plan_settings"],
        new_plan_settings=new[0]["plan_settings"],
        unmatched_old_conditions=[json.loads(k) for k in sorted(old_groups - matched)],
        unmatched_new_conditions=[json.loads(k) for k in sorted(new_groups - matched)],
        skipped_incomplete_exports=old_skipped + new_skipped,
        parameters=dict(
            hampel_window_samples=args.hampel_window,
            hampel_sigma=args.hampel_sigma,
            spike_floor_n=args.spike_floor_n,
            gap_break_ms=5.0,
            minimum_metric_coverage=0.8,
        ),
        trials_without_hold_events=[
            dict(dataset=d["dataset"], trial_id=d["window"]["trial_id"])
            for d in old + new
            if not d["hold_count"]
        ],
        trials_with_partial_hold_events=[
            dict(
                dataset=d["dataset"],
                trial_id=d["window"]["trial_id"],
                observed=d["hold_count"],
                expected=d["expected_hold_count"],
            )
            for d in old + new
            if 0 < d["hold_count"] < d["expected_hold_count"]
        ],
        input_sha256=sources,
        script_sha256=fingerprint(Path(__file__)),
        limitations=[
            "No automatic equivalence verdict or significance test.",
            "Hampel flags are candidates, not validated noise labels; long bursts can remain.",
            "Metrics exclude candidate samples; no interpolation, smoothing, time warping or amplitude normalization.",
            "Hold metrics require actual action events; no timing inference for older exports.",
            "Repeated cycles/samples are not independent trials.",
            "Mounting/init/session differences remain confounded with hardware changes.",
            "Only final complete exported attempts are analyzed; abort counts must be evaluated separately.",
            "No original bag access or physical force calibration; GelSight contact images not assessed.",
        ],
    )
    (args.out / "report.json").write_text(
        json.dumps(finite_json(report), ensure_ascii=False, indent=2, allow_nan=False)
    )
    lines = [
        f"Matched groups: {len(groups)}; old trials: {len(old)}; new trials: {len(new)}"
    ]
    for dataset in ("old", "new"):
        for side in ("left", "right"):
            selected = [
                r for r in metrics if r["dataset"] == dataset and r["side"] == side
            ]
            lines.append(
                f"{dataset} {side}: candidate samples {sum(r['candidate_samples'] for r in selected)}/{sum(r['samples'] for r in selected)}; flagged trials {sum(r['candidate_samples']>0 for r in selected)}/{len(selected)}"
            )
    lines.append(
        "Trials without event-based hold metrics: "
        + str(len(report["trials_without_hold_events"]))
    )
    lines.extend(report["limitations"])
    (args.out / "summary.txt").write_text("\n".join(lines) + "\n")
    print("\n".join(lines[:6]))
    print("Report: " + str((args.out / "report.json").resolve()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
