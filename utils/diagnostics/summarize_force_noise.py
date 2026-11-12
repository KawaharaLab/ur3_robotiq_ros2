#!/usr/bin/env python3
"""Summarize existing plot reports and latest progress files; never filter raw data."""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path


def load(path):
    return json.loads(path.read_text())


def summarize(plots, data):
    trials, triggers, sources = {}, [], []
    # Keep the newest report per trial, so rerendering a job does not double count.
    reports = sorted(
        plots.rglob("report.json"), key=lambda p: (p.stat().st_mtime_ns, str(p))
    )
    for path in reports:
        report = load(path)
        if report.get("input_format") != "extract_all compact export" and not (
            isinstance(report.get("trials"), list)
            and report.get("original_bags_read") is False
        ):
            continue
        used = False
        for row in report.get("trials", []):
            if not isinstance(row, dict) or "force_statistics" not in row:
                continue
            key = (row["object_id"], row["job_id"], row["trial_id"])
            trials[key] = row
            used = True
        if used:
            sources.append(str(path))
    rows = {}
    for (obj, job, _), trial in trials.items():
        row = rows.setdefault(
            (obj, job), dict(object_id=obj, job_id=job, plotted_trials=0)
        )
        row["plotted_trials"] += 1
        for side in ("left", "right"):
            stats = trial["force_statistics"].get(side, {})
            if "spike_candidates" not in stats or "samples" not in stats:
                continue
            for name, increment in [
                ("evaluated_trials", 1),
                ("flagged_trials", int(stats["spike_candidates"] > 0)),
                ("candidate_samples", stats["spike_candidates"]),
                ("samples", stats["samples"]),
            ]:
                field = side + "_" + name
                row[field] = row.get(field, 0) + increment
    progress_sources = []
    for folder in sorted(data.glob("*/automatic_jobs")):
        paths = list(folder.glob("*.json"))
        if not paths:
            continue
        path = max(paths, key=lambda p: (p.stat().st_mtime_ns, p.name))
        progress_sources.append(str(path))
        job = load(path)
        obj, job_id = folder.parent.name, job["job_id"]
        rows.setdefault(
            (obj, job_id), dict(object_id=obj, job_id=job_id, plotted_trials=0)
        )
        for pos in job["plan"]["positions"]:
            for trial in pos["trials"]:
                for index, attempt in enumerate(trial.get("attempts", []), 1):
                    fault = attempt.get("force_trigger")
                    if not fault:
                        continue
                    confirmation = fault.get("confirmation") or {}
                    first, last = confirmation.get("first_header_ns"), confirmation.get(
                        "last_header_ns"
                    )
                    triggers.append(
                        dict(
                            object_id=obj,
                            job_id=job_id,
                            position=pos["id"],
                            trial_id=trial["id"],
                            attempt=index,
                            reason=fault.get("reason"),
                            side=fault.get("side"),
                            limit_n=fault.get("limit_n"),
                            confirm_ms=fault.get("confirm_ms"),
                            samples=confirmation.get("samples"),
                            observed_until_trigger_ms=(
                                (last - first) / 1e6
                                if first is not None and last is not None
                                else None
                            ),
                            peak_until_trigger_n=confirmation.get("peak_relative_n"),
                            session=attempt.get("session"),
                        )
                    )
    for (obj, job), row in rows.items():
        counts = Counter(
            t["side"] for t in triggers if t["object_id"] == obj and t["job_id"] == job
        )
        for side in ("left", "right"):
            row[side + "_force_trigger_count"] = counts[side]
    return dict(
        objects=[rows[k] for k in sorted(rows)],
        triggers=triggers,
        plot_reports=sources,
        progress_files=progress_sources,
        notes=[
            "Spike candidates are flagged samples, not confirmed noise or independent events.",
            "Trigger duration ends at detection: not the full excursion duration.",
            "Progress uses the latest job per object. Older plotted jobs remain separate.",
            "No quality finding classification or raw waveform filtering is performed.",
        ],
    )


def write_csv(path, rows):
    with path.open("w", newline="") as stream:
        fields = list(dict.fromkeys(key for row in rows for key in row))
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plots-root", type=Path, required=True)
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    for folder in (args.plots_root, args.data_root):
        if not folder.is_dir():
            parser.error("Directory not found: " + str(folder))
    if args.out.exists():
        parser.error("--out already exists; choose a new directory")
    report = summarize(args.plots_root, args.data_root)
    if not report["objects"]:
        parser.error("No plot reports or automatic_jobs found")
    args.out.mkdir(parents=True)
    (args.out / "summary.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2)
    )
    write_csv(args.out / "objects.csv", report["objects"])
    write_csv(args.out / "force_triggers.csv", report["triggers"])
    print("Report: " + str((args.out / "summary.json").resolve()))
    print("Object/job pairs:", len(report["objects"]))
    print("Force triggers:", len(report["triggers"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
