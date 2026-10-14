"""Additive export provenance, without changing legacy filenames or CSV fields."""

from __future__ import annotations

import csv
import hashlib
import json
import os
from pathlib import Path


INDEX_NAME = "timing_index.csv"
MANIFEST_NAME = "export_manifest.json"
FIELDS = (
    "topic", "topic_message_index", "bag_receive_ns", "source_stamp_ns",
    "source_stamp_semantics", "frame_id", "artifact_path", "artifact_row",
    "export_status", "error",
)


def _file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def header_time(msg):
    """Do not invent a source time for headerless or multi-header messages."""
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None, "unavailable", ""
    sec, nanosec = int(stamp.sec), int(stamp.nanosec)
    if not 0 <= nanosec < 1_000_000_000:
        return None, "invalid_header", str(getattr(header, "frame_id", ""))
    value = sec * 1_000_000_000 + nanosec
    return (value if value != 0 else None,
            "publisher_header" if value != 0 else "zero_header",
            str(getattr(header, "frame_id", "")))


def export_provenance(source_bag, config, bounds):
    source = Path(config.source_path)
    return {
        "source_bag": str(Path(source_bag).resolve()),
        "config_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "topics": [vars(spec) for spec in config.topics],
        "bag_receive_bounds_ns": list(bounds),
    }


def _atomic_json(path, data):
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def export_is_complete(output_dir, provenance):
    """Missing/old/failed or visibly truncated exports must not be skipped."""
    output_dir = Path(output_dir)
    try:
        manifest = json.loads((output_dir / MANIFEST_NAME).read_text(encoding="utf-8"))
        if (manifest.get("schema_version") != 1 or manifest.get("status") != "complete"
                or manifest.get("provenance") != provenance):
            return False
        index = output_dir / INDEX_NAME
        if _file_sha256(index) != manifest["index_sha256"]:
            return False
        return all((output_dir / name).is_file()
                   and (output_dir / name).stat().st_size == size
                   for name, size in manifest["artifact_sizes"].items())
    except (OSError, ValueError, KeyError, TypeError):
        return False


class ExportTimingIndex:
    """One row per selected message; complete is written only after artifacts exist.

    publisher_header preserves a timestamp, not a claim of exposure/ADC timing.
    This records export quality, not proof of sensor synchronization or bag completeness.
    """

    def __init__(self, output_dir, provenance):
        self.output_dir = Path(output_dir)
        self.provenance = provenance
        self.counts = {spec["name"]: 0 for spec in provenance["topics"]}
        self.failures = 0
        self.artifacts = set()
        self.image_paths = set()

    def __enter__(self):
        self.output_dir.mkdir(parents=True, exist_ok=True)
        # Invalidate a previous completion before touching any exported artifact.
        self._manifest("in_progress")
        self.stream = (self.output_dir / (INDEX_NAME + ".tmp")).open(
            "w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.stream, fieldnames=FIELDS)
        self.writer.writeheader()
        return self

    def _manifest(self, status, error="", **extra):
        _atomic_json(self.output_dir / MANIFEST_NAME, {
            "schema_version": 1, "status": status, "provenance": self.provenance,
            "timestamp_correction_applied": False,
            "legacy_stamp_ns_basis": "bag_receive",
            "source_stamp_semantics": "publisher_header; physical meaning depends on producer",
            "selected_message_counts": self.counts, "failed_messages": self.failures,
            "configured_topics_without_messages": [k for k, v in self.counts.items() if v == 0],
            "quality_note": "Completion covers export only, not sensor synchronization or source bag completeness.",
            "error": error, **extra,
        })

    def claim_image_path(self, path):
        """Fail rather than silently overwrite two frames with the same bag stamp."""
        path = Path(path)
        if path in self.image_paths:
            raise ValueError(f"Duplicate image output path within this export: {path}")
        self.image_paths.add(path)

    def record(self, topic, bag_receive_ns, msg, artifact_path=None,
               artifact_row=None, status="ok", error=""):
        self.counts[topic] = self.counts.get(topic, 0) + 1
        source_ns, semantics, frame_id = header_time(msg)
        relative = ""
        if artifact_path is not None:
            relative = str(Path(artifact_path).relative_to(self.output_dir))
            self.artifacts.add(relative)
        if status != "ok":
            self.failures += 1
        self.writer.writerow(dict(zip(FIELDS, (
            topic, self.counts[topic], int(bag_receive_ns), source_ns, semantics,
            frame_id, relative, artifact_row, status, str(error),
        ))))

    def __exit__(self, exc_type, exc, traceback):
        self.stream.close()
        os.replace(self.output_dir / (INDEX_NAME + ".tmp"), self.output_dir / INDEX_NAME)
        if exc is not None:
            self._manifest("failed", str(exc))
            return False
        if self.failures or not any(self.counts.values()):
            error = f"Export incomplete: {self.failures} failed messages, {sum(self.counts.values())} selected messages"
            self._manifest("failed", error)
            raise RuntimeError(error)
        try:
            sizes = {name: (self.output_dir / name).stat().st_size for name in sorted(self.artifacts)}
            digest = _file_sha256(self.output_dir / INDEX_NAME)
        except OSError as error:
            self._manifest("failed", str(error))
            raise
        self._manifest("complete", artifact_sizes=sizes, index_sha256=digest)
        return False
