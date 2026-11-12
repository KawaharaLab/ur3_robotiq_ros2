#!/usr/bin/env python3
"""Sequential overnight quality -> extract_all backend -> plots, with resumable stages."""
import argparse
from collections import Counter
import csv
from datetime import datetime
import uuid
import fcntl
import hashlib
import html
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "data_capture_tools"))
from data_capture_tools.compact_export import selected_attempts
from data_capture_tools.object_plan import atomic_json

CATEGORIES = ("優先確認", "波形確認", "時刻確認", "補完情報")


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def signature(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()


def safe_name(value):
    if not value or value in (".", "..") or Path(value).name != value:
        raise ValueError("Invalid path component: " + str(value))
    return value


def read_json(path):
    return json.loads(Path(path).read_text())


def source_signature(job):
    """Cheap change detection, without hashing all JPEG/database contents."""
    entries = []
    for session in sorted(selected_attempts(job)):
        root = Path(session)
        if not root.is_dir():
            entries.append([session, "missing"])
            continue
        for path in sorted(root.rglob("*")):
            if path.is_file():
                st = path.stat()
                entries.append([str(path), st.st_size, st.st_mtime_ns])
    return signature(entries)


def discover(data_root, objects):
    folders = (
        [data_root / name / "automatic_jobs" for name in objects]
        if objects
        else sorted(data_root.glob("*/automatic_jobs"))
    )
    results = []
    for folder in folders:
        entry = dict(object_id=folder.parent.name, stages={})
        paths = sorted(
            folder.glob("*.json"), key=lambda p: (p.stat().st_mtime_ns, p.name)
        )
        if not paths:
            entry.update(status="input_error", error="No automatic_jobs progress JSON")
        else:
            path = paths[-1]
            entry["progress"] = str(path.resolve())
            try:
                job = read_json(path)
                rows = [
                    row
                    for position in job["plan"]["positions"]
                    for row in position["trials"]
                ]
                entry.update(
                    job_id=safe_name(job["job_id"]),
                    progress_sha256=file_hash(path),
                    planned_trials=len(rows),
                    complete_trials=sum(r["status"] == "complete" for r in rows),
                )
                if job.get("status") != "complete" or entry["complete_trials"] != len(
                    rows
                ):
                    entry.update(
                        status="skipped_incomplete",
                        error="Latest job is incomplete; older jobs are not substituted",
                    )
                else:
                    selected_attempts(job)
                    entry["status"] = "pending"
            except Exception as exc:
                entry.update(
                    status="input_error", error=type(exc).__name__ + ": " + str(exc)
                )
        results.append(entry)
    return results


def reusable_export(path, progress_hash, job):
    """Reuse only a completed matching export whose recorded artifact sizes agree."""
    try:
        report = read_json(path / "report.json")
        if (
            report.get("mode") != "write"
            or report.get("status") != "complete"
            or report.get("job_id") != job["job_id"]
            or report.get("progress_sha256") != progress_hash
            or file_hash(path / "source_progress.json") != progress_hash
        ):
            return False
        groups = selected_attempts(job)
        expected = {row["trial_id"] for rows in groups.values() for row in rows}
        actual = set()
        if report.get("exported_or_previewed_trials") != len(expected):
            return False
        for session, rows in groups.items():
            target = path / Path(session).name
            manifest = read_json(target / "manifest.json")
            if manifest.get("status") != "complete" or not manifest.get(
                "artifact_sizes"
            ):
                return False
            metadata = list((Path(session) / "bag").rglob("metadata.yaml"))
            if len(metadata) == 1 and manifest.get(
                "source_metadata_sha256"
            ) != file_hash(metadata[0]):
                return False
            windows = manifest["windows"]
            if {w["trial_id"] for w in windows} != {r["trial_id"] for r in rows}:
                return False
            for window in windows:
                if window["trial_id"] in actual:
                    return False
                actual.add(window["trial_id"])
            for relative, size in manifest["artifact_sizes"].items():
                rel = Path(relative)
                artifact = target / rel
                if (
                    rel.is_absolute()
                    or ".." in rel.parts
                    or not artifact.is_file()
                    or artifact.stat().st_size != size
                ):
                    return False
        return actual == expected
    except (OSError, ValueError, KeyError, TypeError):
        return False


def stage_command(kind, progress, output, export_path=None):
    if kind == "quality":
        return [
            sys.executable,
            str(REPO / "utils/diagnostics/check_object_acquisition.py"),
            "--progress",
            str(progress),
            "--out",
            str(output),
        ]
    if kind == "export":
        # This is the same main() used by ros2 run data_capture_tools extract_all.
        return [
            sys.executable,
            "-m",
            "data_capture_tools.compact_export",
            "--progress",
            str(progress),
            "--out",
            str(output),
            "--write",
        ]
    return [
        sys.executable,
        str(REPO / "utils/diagnostics/plot_object_acquisition.py"),
        "--compact-layout",
        "--export-dir",
        str(export_path),
        "--out",
        str(output),
    ]


def run_child(command, log, stage, persist):
    process = None
    stage.update(
        status="running", command=command, log=str(log), started_time_ns=time.time_ns()
    )
    persist()
    environment = os.environ.copy()
    environment["PYTHONPATH"] = (
        str(REPO / "data_capture_tools")
        + os.pathsep
        + environment.get("PYTHONPATH", "")
    )
    environment["PYTHONUNBUFFERED"] = "1"
    try:
        with log.open("x") as stream:
            process = subprocess.Popen(
                command,
                stdout=stream,
                stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,
                env=environment,
                start_new_session=True,
            )
            heartbeat = time.monotonic()
            while process.poll() is None:
                time.sleep(0.25)
                if time.monotonic() - heartbeat >= 60:
                    print("  Still running. Log: " + str(log), flush=True)
                    heartbeat = time.monotonic()
            stage["exit_code"] = process.returncode
        stage["status"] = "finished"
    except KeyboardInterrupt:
        stage["status"] = "interrupted"
        raise
    except Exception as exc:
        stage.update(status="error", error=type(exc).__name__ + ": " + str(exc))
    finally:
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3.0)
            except ProcessLookupError:
                pass
        stage["finished_time_ns"] = time.time_ns()
        stage["elapsed_seconds"] = (
            stage["finished_time_ns"] - stage["started_time_ns"]
        ) / 1e9
        persist()


def category(issue):
    code = issue.get("code", "")
    if issue.get("level") == "NOTE":
        return "補完情報"
    if issue.get("level") == "FAIL" or any(
        word in code
        for word in (
            "unrecoverable",
            "missing_sensor",
            "trial_sensor_missing",
            "invalid_jpeg",
            "mismatch",
            "parse_error",
            "nonfinite",
            "audit_failed",
        )
    ):
        return "優先確認"
    if any(word in code for word in ("gap", "delay", "header", "rate", "clock")):
        return "時刻確認"
    return "波形確認"


def review_rows(entry):
    quality_stage = entry["stages"].get("quality", {})
    quality_file = Path(quality_stage.get("output", "")) / "summary.json"
    if not quality_file.is_file():
        return []
    quality = read_json(quality_file)
    job = read_json(entry["snapshot"])
    selected = selected_attempts(job)
    plot_stage = entry["stages"].get("plot", {})
    plot_file = Path(plot_stage.get("output", "")) / "report.json"
    plot_report = read_json(plot_file) if plot_file.is_file() else {}
    plots = {
        r["trial_id"]: str(Path(plot_stage["output"]) / r["plot"])
        for r in plot_report.get("trials", [])
    }
    result = []
    issues = [(None, issue) for issue in quality.get("issues", [])]
    issues.extend(
        (session["session"], issue)
        for session in quality.get("sessions", [])
        for issue in session["issues"]
    )
    for session, issue in issues:
        trial_ids = []
        candidates = selected.get(session, [])
        original = issue.get("original", {})
        if issue.get("trial_id"):
            trial_ids = [issue["trial_id"]]
        elif "trial_index" in issue or "trial_index" in original:
            index = issue.get("trial_index", original.get("trial_index"))
            trial_ids = [
                row["trial_id"] for row in candidates if row["trial_index"] == index
            ]
        elif "start_ns" in issue and "end_ns" in issue:
            trial_ids = [
                row["trial_id"]
                for row in candidates
                if row["attempt"]["start_ros_ns"]
                <= issue["start_ns"]
                <= issue["end_ns"]
                <= row["attempt"]["end_ros_ns"]
            ]
        for trial_id in trial_ids or [""]:
            result.append(
                dict(
                    object_id=entry["object_id"],
                    job_id=entry.get("job_id", ""),
                    category=category(issue),
                    level=issue["level"],
                    code=issue["code"],
                    trial_id=trial_id,
                    topic=issue.get("topic", original.get("topic", "")),
                    session=session or "",
                    plot=plots.get(trial_id, ""),
                    details=json.dumps(issue, ensure_ascii=False),
                )
            )
    return result


def link(path, root, label):
    target = html.escape(os.path.relpath(path, root), quote=True)
    return '<a href="' + target + '">' + html.escape(label) + "</a>"


def history_records(history_root):
    records = []
    for path in sorted((history_root / "runs").glob("*.json")):
        try:
            records.append(read_json(path))
        except (OSError, ValueError):
            continue
    return sorted(records, key=lambda r: r.get("started_time_ns", 0), reverse=True)


def history_rows(records):
    rows = []
    for run in records:
        when = (
            datetime.fromtimestamp(run["started_time_ns"] / 1e9)
            .astimezone()
            .isoformat(timespec="seconds")
        )
        for entry in run["objects"]:
            row = dict(
                run_id=run["run_id"],
                processed_at=when,
                object_id=entry["object_id"],
                job_id=entry.get("job_id", ""),
                status=entry["status"],
                progress=entry.get("progress", ""),
                summary=str(Path(run["run_directory"]) / "summary.html"),
            )
            for kind in ("quality", "export", "plot"):
                stage = entry["stages"].get(kind, {})
                row[kind] = stage.get("result", "未実行")
                row[kind + "_action"] = (
                    "再利用"
                    if stage.get("reused")
                    else (
                        "省略"
                        if stage.get("status") == "skipped"
                        else "実行" if stage.get("started_time_ns") else "未実行"
                    )
                )
                row[kind + "_output"] = stage.get("output", "")
                row[kind + "_origin_run"] = stage.get("origin_run", "")
            rows.append(row)
    return rows


def update_history(state, root):
    history = Path(state["history_root"])
    records_dir = history / "runs"
    records_dir.mkdir(parents=True, exist_ok=True)
    record = dict(state, run_directory=str(root))
    # The path hash also distinguishes explicit output folders with the same basename.
    record_path = records_dir / (signature(str(root))[:24] + ".json")
    atomic_json(record_path, record)
    rows = history_rows(history_records(history))
    fields = [
        "run_id",
        "processed_at",
        "object_id",
        "job_id",
        "status",
        "progress",
        "summary",
    ]
    for kind in ("quality", "export", "plot"):
        fields += [kind, kind + "_action", kind + "_output", kind + "_origin_run"]
    temporary = history / "history.csv.tmp"
    with temporary.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    os.replace(temporary, history / "history.csv")
    table = []
    for row in rows:
        cells = [
            html.escape(row[key])
            for key in ("processed_at", "object_id", "job_id", "status")
        ]
        for kind in ("quality", "export", "plot"):
            label = row[kind + "_action"] + " / " + row[kind]
            target = row[kind + "_output"]
            cells.append(link(target, history, label) if target else html.escape(label))
        cells.append(link(row["summary"], history, "この実行の詳細"))
        cells.append(html.escape(row["progress"]))
        table.append(
            "<tr>" + "".join("<td>" + cell + "</td>" for cell in cells) + "</tr>"
        )
    page = """<!doctype html><html lang="ja"><meta charset="utf-8"><title>データ処理の履歴</title>
<style>body{font-family:sans-serif;margin:24px}td,th{border:1px solid #ccc;padding:8px}table{border-collapse:collapse}input{padding:8px;width:32em;max-width:90%}</style>
<h1>データ処理の履歴</h1><p>処理日時と収集ジョブIDは別々に記録します。再利用した結果の作成元はhistory.csvで確認できます。</p>
<input id="filter" placeholder="物体名・処理日・収集ジョブIDなどで検索" aria-label="履歴を検索">
<table><thead><tr><th>処理日時</th><th>物体</th><th>収集ジョブID</th><th>状態</th><th>品質</th><th>展開</th><th>プロット</th><th>詳細</th><th>元の進捗JSON</th></tr></thead><tbody>"""
    page += (
        "\n".join(table)
        + """</tbody></table><script>
document.getElementById('filter').addEventListener('input',function(){
const words=this.value.toLowerCase().trim().split(/\\s+/);
document.querySelectorAll('tbody tr').forEach(row=>{row.hidden=!words.every(word=>row.textContent.toLowerCase().includes(word));});
});</script></html>"""
    )
    temporary = history / "index.html.tmp"
    temporary.write_text(page)
    os.replace(temporary, history / "index.html")


def import_legacy_runs(history):
    """Register earlier management folders without moving or rewriting them."""
    records = history / "runs"
    records.mkdir(parents=True, exist_ok=True)
    for path in [
        history / "batch_state.json",
        *sorted(history.glob("*/batch_state.json")),
    ]:
        if not path.is_file():
            continue
        target = records / (signature(str(path.parent.resolve()))[:24] + ".json")
        if target.exists():
            continue
        try:
            old = read_json(path)
            old.setdefault("run_id", path.parent.name)
            old["run_directory"] = str(path.parent.resolve())
            atomic_json(target, old)
        except (OSError, ValueError):
            continue


def seed_previous_results(entries, history, root):
    records = history_records(history)
    for entry in entries:
        if entry["status"] != "pending":
            continue
        for record in records:
            if record.get("run_directory") == str(root):
                continue
            for old in record["objects"]:
                if old["object_id"] == entry["object_id"] and old.get(
                    "progress_sha256"
                ) == entry.get("progress_sha256"):
                    for kind, stage in old["stages"].items():
                        if (
                            kind not in entry["stages"]
                            and stage.get("status") == "finished"
                        ):
                            entry["stages"][kind] = dict(stage)
                            entry["stages"][kind].setdefault(
                                "origin_run", record["run_id"]
                            )


def save_summary(state, root):
    reviews = []
    for entry in state["objects"]:
        if entry.get("snapshot"):
            try:
                reviews.extend(review_rows(entry))
            except Exception as exc:
                entry["summary_error"] = str(exc)
    atomic_json(root / "batch_state.json", state)
    with (root / "review_trials.csv").open("w", newline="") as stream:
        fields = [
            "object_id",
            "job_id",
            "category",
            "level",
            "code",
            "trial_id",
            "topic",
            "session",
            "plot",
            "details",
        ]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(reviews)
    lines = [
        f"一括処理の状態　{state['status']}",
        "品質判定と処理の完了状態は別々に表示します。",
        "優先確認 → 波形確認 → 時刻確認の順に図を見てください。補完情報は前処理用に残します。",
        "",
    ]
    table = []
    for entry in state["objects"]:
        stages = entry["stages"]
        quality = stages.get("quality", {}).get("result", "未実行")
        export = stages.get("export", {}).get("result", "未実行")
        plot = stages.get("plot", {}).get("result", "未実行")
        matches = [r for r in reviews if r["object_id"] == entry["object_id"]]
        counts = dict(Counter(r["category"] for r in matches))
        actions = ", ".join(
            kind + "再利用" for kind, stage in stages.items() if stage.get("reused")
        )
        lines.append(
            f"{entry['object_id']}　処理={entry['status']}　品質={quality}　展開={export}　図={plot}　指摘={counts}　{actions}"
        )
        if entry.get("error"):
            lines.append("  " + entry["error"])
        urls = []
        for kind, stage in stages.items():
            if stage.get("log"):
                urls.append(link(stage["log"], root, kind + "ログ"))
        quality_output = stages.get("quality", {}).get("output")
        if quality_output and (Path(quality_output) / "summary.txt").is_file():
            urls.append(link(Path(quality_output) / "summary.txt", root, "品質の詳細"))
        if entry.get("plot_notes", {}).get("missing_auxiliary_csvs"):
            urls.append("旧展開データのためグリッパ・動作パネルの一部を省略")
        chosen = []
        chosen_codes = set()
        for review in sorted(
            matches, key=lambda r: (CATEGORIES.index(r["category"]), r["code"])
        ):
            if (
                review["plot"]
                and review["plot"] not in chosen
                and review["code"] not in chosen_codes
            ):
                chosen.append(review["plot"])
                chosen_codes.add(review["code"])
                urls.append(
                    link(
                        review["plot"],
                        root,
                        review["category"] + " " + review["trial_id"],
                    )
                )
                if len(chosen) == 3:
                    break
        if stages.get("plot", {}).get("output"):
            urls.append(
                link(
                    Path(stages["plot"]["output"]) / "trial_plot_index.csv",
                    root,
                    "全図の一覧",
                )
            )
        table.append(
            "<tr><td>"
            + html.escape(entry["object_id"])
            + "</td><td>"
            + html.escape(entry["status"])
            + "</td><td>"
            + html.escape(quality)
            + "</td><td>"
            + html.escape(export)
            + "</td><td>"
            + html.escape(plot)
            + "</td><td>"
            + html.escape(str(counts))
            + "</td><td>"
            + "<br>".join(urls)
            + "</td></tr>"
        )
    (root / "summary.txt").write_text("\n".join(lines) + "\n")
    page = """<!doctype html><html lang="ja"><meta charset="utf-8"><title>収集後の処理結果</title>
<style>body{font-family:sans-serif;margin:24px}table{border-collapse:collapse}td,th{border:1px solid #ccc;padding:8px;vertical-align:top}a{line-height:1.7}</style>
<h1>収集後の処理結果</h1><p>優先確認 → 波形確認 → 時刻確認の順に図を確認してください。補完情報は前処理で利用します。</p>
<p>REVIEWは取り直しの自動判定ではありません。短いHeader間隔の乱れも含みます。物理的な同期精度は別途評価が必要です。</p>
<table><tr><th>物体</th><th>処理</th><th>品質判定</th><th>展開</th><th>プロット</th><th>指摘数</th><th>ログ・代表図</th></tr>"""
    elapsed = (
        (
            state.get("finished_time_ns", time.time_ns())
            if state["status"] != "running"
            else time.time_ns()
        )
        - state["started_time_ns"]
    ) / 1e9
    overview = (
        "<p>状態　"
        + html.escape(state["status"])
        + "　物体数　"
        + str(len(state["objects"]))
        + "　経過　"
        + str(round(elapsed / 60, 1))
        + "分</p>"
    )
    page = page.replace("<table>", overview + "<table>", 1)
    (root / "summary.html").write_text(
        page
        + "\n".join(table)
        + "</table><p>詳細はreview_trials.csvと各品質レポートを確認してください。</p></html>"
    )
    if state.get("history_root"):
        update_history(state, root)


def execute_stage(
    kind,
    entry,
    root,
    progress,
    code_hashes,
    source_id,
    persist,
    export_path=None,
    result_root=None,
):
    stages = entry["stages"]
    key = signature(
        dict(
            kind=kind,
            progress=entry["progress_sha256"],
            code=code_hashes,
            sources=source_id,
            export_path=str(export_path) if export_path else None,
        )
    )
    previous = stages.get(kind, {})
    report_name = "summary.json" if kind == "quality" else "report.json"
    previous_report = Path(previous.get("output", "")) / report_name
    if (
        previous.get("key") == key
        and previous.get("status") == "finished"
        and previous_report.is_file()
        and (
            result_root is None
            or Path(previous.get("output", "")).parent == Path(result_root)
        )
    ):
        saved = read_json(previous_report)
        reusable = saved.get("status") in (
            ("PASS", "PASS_WITH_NOTES", "REVIEW")
            if kind == "quality"
            else ("plots_generated",)
        )
        if reusable and kind == "quality":
            names = (
                "payload_integrity.json",
                "acquisition_quality.json",
                "gelsight_comparison.json",
                "timing_validation.json",
                "trials.json",
            )
            reusable = all(
                all(
                    (Path(session["report_directory"]) / name).is_file()
                    for name in names
                )
                for session in saved.get("sessions", [])
            )
        if reusable and kind == "plot":
            artifacts = [row["plot"] for row in saved.get("trials", [])] + saved.get(
                "comparison_plots", []
            )
            reusable = bool(artifacts) and all(
                (Path(previous["output"]) / rel).is_file()
                and (Path(previous["output"]) / rel).stat().st_size > 0
                for rel in artifacts
            )
        if reusable:
            previous["reused"] = True
            print("  Reusing " + kind + " result: " + previous["output"], flush=True)
            return previous
    history = entry.setdefault("stage_history", {}).setdefault(kind, [])
    if previous:
        history.append(previous)
    number = len(history) + 1
    base = root / "objects" / safe_name(entry["object_id"]) / safe_name(entry["job_id"])
    base.mkdir(parents=True, exist_ok=True)
    # Human-facing results live directly below quality_check/ or plots/.
    # Never overwrite results created by an earlier job or interrupted attempt.
    result_root = Path(result_root) if result_root else root / kind
    result_root.mkdir(parents=True, exist_ok=True)
    object_name = safe_name(entry["object_id"])
    number = 1
    output = result_root / object_name
    while output.exists():
        number += 1
        output = result_root / (object_name + f"_{number:03d}")
    stage = dict(
        key=key,
        output=str(output),
        status="pending",
        reused=False,
        origin_run=root.name,
    )
    stages[kind] = stage
    command = stage_command(kind, progress, output, export_path)
    run_child(command, base / (kind + f"_{number:03d}.log"), stage, persist)
    path = output / report_name
    saved = read_json(path) if path.is_file() else {}
    stage["result"] = saved.get("status", "実行エラー")
    if kind != "quality" and stage.get("exit_code") != 0:
        stage["result"] = saved.get("status", "実行エラー")
    if not saved:
        stage["status"] = "error"
    persist()
    return stage


def process_object(entry, root, config, persist):
    if entry["status"] in ("input_error", "skipped_incomplete"):
        return
    progress = Path(entry["progress"])
    if file_hash(progress) != entry["progress_sha256"]:
        entry.update(
            status="input_error",
            error="Progress changed since this batch was planned; use a new output directory",
        )
        persist()
        return
    job = read_json(progress)
    entry.pop("error", None)
    base = root / "objects" / safe_name(entry["object_id"]) / safe_name(entry["job_id"])
    base.mkdir(parents=True, exist_ok=True)
    snapshot = base / "progress.json"
    if snapshot.exists() and file_hash(snapshot) != entry["progress_sha256"]:
        raise ValueError("Saved progress snapshot changed")
    if not snapshot.exists():
        snapshot.write_bytes(progress.read_bytes())
    entry.update(snapshot=str(snapshot), status="running")
    sources = source_signature(job)
    code_files = [
        REPO / "data_capture_tools/data_capture_tools" / name
        for name in ("object_plan.py", "compact_export.py", "acquisition_quality.py")
    ]
    code_files += [
        REPO / "utils/diagnostics" / name
        for name in (
            "check_object_acquisition.py",
            "compare_gelsight_local_bag.py",
            "verify_acquisition_timing.py",
        )
    ]
    hashes = {str(p.relative_to(REPO)): file_hash(p) for p in code_files}
    if config["quality"]:
        execute_stage(
            "quality",
            entry,
            root,
            snapshot,
            hashes,
            sources,
            persist,
            result_root=config["quality_root"],
        )
    else:
        entry["stages"]["quality"] = dict(status="skipped", result="省略")
        persist()
    export_root = Path(config["exports_root"])
    object_exports = export_root / safe_name(entry["object_id"])
    candidates = [object_exports]
    if object_exports.is_dir():
        candidates += sorted(
            p
            for p in object_exports.iterdir()
            if p.is_dir() and (p / "report.json").is_file()
        )
    candidates += [p.parent for p in sorted(export_root.glob("*/report.json"))]
    candidates += [p.parent for p in sorted(export_root.glob("*/*/report.json"))]
    old = entry["stages"].get("export", {}).get("output")
    if old:
        candidates.insert(0, Path(old))
    previous_export = entry["stages"].get("export", {})
    if config["reexport"]:
        candidates = (
            [Path(old)]
            if old
            and previous_export.get("created_by_batch")
            and previous_export.get("created_in_run") == str(root)
            else []
        )
    usable = next(
        (p for p in candidates if reusable_export(p, entry["progress_sha256"], job)),
        None,
    )
    if usable is not None:
        reused = (
            dict(previous_export)
            if previous_export.get("output") == str(usable)
            else {}
        )
        reused.update(
            status="finished",
            result="既存出力を再利用",
            output=str(usable),
            created_by_batch=previous_export.get("created_by_batch", False),
            created_in_run=previous_export.get("created_in_run"),
            reused=True,
            origin_run=previous_export.get("origin_run", "existing export"),
        )
        entry["stages"]["export"] = reused
        export_path = usable
        persist()
    else:
        # The export backend owns the same layout as extract_all. Keep old/partial outputs.
        object_exports.mkdir(parents=True, exist_ok=True)
        number = 1
        export_path = object_exports / entry["job_id"]
        while export_path.exists():
            number += 1
            export_path = object_exports / (entry["job_id"] + f"_retry{number}")
        stage = dict(
            status="pending",
            output=str(export_path),
            created_by_batch=True,
            created_in_run=str(root),
            reused=False,
            origin_run=root.name,
        )
        if previous_export:
            entry.setdefault("stage_history", {}).setdefault("export", []).append(
                previous_export
            )
        entry["stages"]["export"] = stage
        run_child(
            stage_command("export", snapshot, export_path),
            base / (f"export_{time.time_ns()}.log"),
            stage,
            persist,
        )
        saved = (
            read_json(export_path / "report.json")
            if (export_path / "report.json").is_file()
            else {}
        )
        stage["result"] = saved.get("status", "実行エラー")
        persist()
    # Plot every complete session available, even if another session failed extraction.
    if any(export_path.glob("*/manifest.json")):
        plot_hashes = dict(
            hashes,
            plot=file_hash(REPO / "utils/diagnostics/plot_object_acquisition.py"),
        )
        execute_stage(
            "plot",
            entry,
            root,
            snapshot,
            plot_hashes,
            sources,
            persist,
            export_path,
            result_root=config["plots_root"],
        )
    else:
        entry["stages"]["plot"] = dict(
            status="skipped", result="展開済みセッションなし"
        )
    plot_output = entry["stages"].get("plot", {}).get("output")
    if plot_output and (Path(plot_output) / "report.json").is_file():
        generated = read_json(Path(plot_output) / "report.json")
        entry["plot_notes"] = dict(
            trials_with_notes=generated.get("trials_with_notes", 0),
            missing_auxiliary_csvs=sorted(
                {
                    name
                    for row in generated.get("sessions", [])
                    for name in row.get("missing_auxiliary_csvs", [])
                }
            ),
        )
    if source_signature(job) != sources:
        entry["error"] = "Source files changed during postprocessing"
    entry["status"] = (
        "done"
        if (
            entry["stages"]["export"].get("result") in ("complete", "既存出力を再利用")
            and entry["stages"]["plot"].get("result") == "plots_generated"
            and not entry.get("error")
        )
        else "processing_error"
    )
    persist()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path)
    parser.add_argument("--exports-root", type=Path)
    parser.add_argument(
        "--out",
        type=Path,
        help="Run directory; default: HISTORY_ROOT/YYYYMMDD_HHMMSS_unique",
    )
    parser.add_argument(
        "--objects", nargs="+", help="Optional task names, e.g. object1 object7"
    )
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--skip-quality", action="store_true")
    parser.add_argument(
        "--reexport",
        action="store_true",
        help="Create a fresh export even when an older complete one exists",
    )
    parser.add_argument("--plots-root", type=Path, help="Default: DATA_ROOT/../plots")
    parser.add_argument(
        "--quality-root", type=Path, help="Default: DATA_ROOT/../quality_check"
    )
    parser.add_argument(
        "--history-root", type=Path, help="Default: DATA_ROOT/../postprocess"
    )
    parser.add_argument(
        "--find", nargs="+", help="Search processing history; does not process data"
    )
    args = parser.parse_args(argv)
    if args.find:
        if not args.history_root and not args.data_root:
            parser.error("--find requires --history-root or --data-root")
        history = (
            args.history_root or args.data_root.resolve().parent / "postprocess"
        ).resolve()
        words = [word.lower() for word in args.find]
        rows = history_rows(history_records(history))
        matches = [
            row
            for row in rows
            if all(
                word in json.dumps(row, ensure_ascii=False).lower() for word in words
            )
        ]
        for row in matches:
            print(
                f"{row['processed_at']}  {row['object_id']}  {row['job_id']}  {row['status']}"
            )
            print("  " + row["summary"])
        print(f"Matches: {len(matches)}")
        return 0
    if not args.data_root or not args.exports_root:
        parser.error("--data-root and --exports-root are required for processing")
    parent = args.data_root.resolve().parent
    history = (args.history_root or parent / "postprocess").resolve()
    config = dict(
        plots_root=str((args.plots_root or parent / "plots").resolve()),
        quality_root=str((args.quality_root or parent / "quality_check").resolve()),
        data_root=str(args.data_root.resolve()),
        exports_root=str(args.exports_root.resolve()),
        objects=sorted(set(args.objects)) if args.objects else [],
        quality=not args.skip_quality,
        reexport=args.reexport,
    )
    if not args.data_root.is_dir():
        parser.error("--data-root not found")
    for name in config["objects"]:
        safe_name(name)
    if args.resume and args.out is None:
        parser.error("--resume requires --out with the exact run directory")
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:6]
    root = args.out.resolve() if args.out else history / run_id
    if root.exists() and not args.resume:
        parser.error("--out exists; use --resume or a new directory")
    if args.resume and not (root / "batch_state.json").is_file():
        parser.error("No batch_state.json to resume")
    history.mkdir(parents=True, exist_ok=True)
    root.mkdir(parents=True, exist_ok=True)
    with (history / ".history.lock").open("a+") as history_lock, (
        root / ".run.lock"
    ).open("a+") as lock:
        try:
            fcntl.flock(history_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error("Another batch is running in this history/output directory")
        import_legacy_runs(history)
        if args.resume:
            state = read_json(root / "batch_state.json")
            # Preserve earlier layout when resuming an existing batch.
            for key in ("plots_root", "quality_root"):
                state["configuration"].setdefault(key, config[key])
            if state["configuration"] != config:
                parser.error("Resume arguments differ from the saved configuration")
        else:
            entries = discover(Path(config["data_root"]), config["objects"])
            if not entries:
                parser.error("No automatic_jobs directories found")
            state = dict(
                schema_version=1,
                status="planned",
                configuration=config,
                objects=entries,
                started_time_ns=time.time_ns(),
            )
            seed_previous_results(entries, history, root)
        state.setdefault("run_id", root.name)
        state.setdefault("history_root", str(history))
        if state["history_root"] != str(history):
            parser.error("--history-root differs from the resumed run")
        print("Run: " + str(root), flush=True)
        print("History: " + str(history / "index.html"), flush=True)

        def persist():
            save_summary(state, root)

        persist()

        def stop(signum, frame):
            raise KeyboardInterrupt()

        previous = signal.signal(signal.SIGTERM, stop)
        state["status"] = "running"
        persist()
        try:
            for index, entry in enumerate(state["objects"], 1):
                print(
                    f"[{index}/{len(state['objects'])}] {entry['object_id']} {entry.get('job_id','')}",
                    flush=True,
                )
                try:
                    process_object(entry, root, config, persist)
                except KeyboardInterrupt:
                    raise
                except Exception as exc:
                    entry.update(
                        status="processing_error",
                        error=type(exc).__name__ + ": " + str(exc),
                    )
                    persist()
            failures = any(
                e["status"] in ("processing_error", "input_error")
                or e["stages"].get("quality", {}).get("result")
                in ("FAIL", "実行エラー")
                for e in state["objects"]
            )
            if failures:
                state["status"] = "completed_with_errors"
            elif any(e["status"] == "skipped_incomplete" for e in state["objects"]):
                state["status"] = "completed_with_skips"
            elif any(
                e["stages"].get("quality", {}).get("result") == "REVIEW"
                for e in state["objects"]
            ):
                state["status"] = "completed_with_findings"
            else:
                state["status"] = "completed"
        except KeyboardInterrupt:
            state["status"] = "interrupted"
        finally:
            signal.signal(signal.SIGTERM, previous)
            state["finished_time_ns"] = time.time_ns()
            persist()
        print("Summary: " + str(root / "summary.html"), flush=True)
        return 1 if state["status"] in ("completed_with_errors", "interrupted") else 0


if __name__ == "__main__":
    raise SystemExit(main())
