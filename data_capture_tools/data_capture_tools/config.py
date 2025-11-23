"""Utilities for loading and validating data capture configuration files."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, List, Optional

import yaml


@dataclass
class TopicSpec:
    """Configuration for a topic that should be captured."""

    name: str
    type: str
    mode: str  # "image" or "csv"
    encoding: Optional[str] = None  # Only used for image topics


@dataclass
class BagRecorderConfig:
    """rosbag2 recording options."""

    storage: str = "sqlite3"
    compression_mode: Optional[str] = None
    compression_format: Optional[str] = None


@dataclass
class CaptureConfig:
    """Top-level configuration for a capture session."""

    output_root: Path
    stop_key: str
    topics: List[TopicSpec]
    bag: BagRecorderConfig
    raw: dict[str, Any]
    source_path: Path


def _validate_topic(entry: dict[str, Any]) -> TopicSpec:
    if "name" not in entry or "type" not in entry or "mode" not in entry:
        raise ValueError(
            "Each topic entry must contain 'name', 'type', and 'mode' fields"
        )

    mode = entry["mode"].lower()
    if mode not in {"image", "csv"}:
        raise ValueError("Topic mode must be either 'image' or 'csv'")

    encoding = entry.get("encoding") if mode == "image" else None

    return TopicSpec(
        name=str(entry["name"]),
        type=str(entry["type"]),
        mode=mode,
        encoding=encoding,
    )


def load_capture_config(config_path: str | Path) -> CaptureConfig:
    """Load and validate a capture configuration YAML file."""

    path = Path(config_path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"Capture config '{path}' does not exist")

    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}

    topics_data = data.get("topics")
    if not topics_data:
        raise ValueError("Config must list at least one topic under 'topics'")

    topics = [_validate_topic(entry) for entry in topics_data]

    bag_cfg = data.get("bag_record", {})
    bag = BagRecorderConfig(
        storage=bag_cfg.get("storage", "sqlite3"),
        compression_mode=bag_cfg.get("compression_mode"),
        compression_format=bag_cfg.get("compression_format"),
    )

    output_root = Path(data.get("output_root", "./data")).expanduser().resolve()
    stop_key = str(data.get("stop_key", "q"))

    return CaptureConfig(
        output_root=output_root,
        stop_key=stop_key,
        topics=topics,
        bag=bag,
        raw=data,
        source_path=path,
    )
