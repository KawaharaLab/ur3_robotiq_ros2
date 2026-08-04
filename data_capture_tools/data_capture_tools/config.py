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
class PoseConfig:
    """TCPの座標と姿勢を保持するクラス"""
    x: float
    y: float
    z: float
    rx: float
    ry: float
    rz: float

@dataclass
class CaptureConfig:
    """Top-level configuration for a capture session."""

    output_root: Path
    task_name: str
    prompt: str
    label: str
    target_diameter: float
    push_depth: float
    gripper_offset: float
    arm_offset: float
    initial_arm_pose: List[float]
    # --- 追加: センタリングパラメータ ---
    contact_threshold: float
    contact_margin: float
    centering_step_grip: float
    centering_step_arm: float
    centering_min_step_arm: float
    centering_min_step_grip: float
    safe_margin: float      # ★追加
    # ----------------------------------
    # --- 追加: base_tcp_pose を定義 ---
    base_tcp_pose: PoseConfig
    viewer_topics: list[str]
    image_throttle_hz: float
    other_throttle_hz: float
    enable_image_compression: bool
    stop_key: str
    discard_key: str
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
    task_name = str(data.get("task_name", "pick_and_place"))
    prompt = str(
        data.get(
            "prompt",
            "pick the blue tape and place it in the orange box.",
        )
    )
    label = str(data.get("label", ""))
    viewer_topics = [str(item) for item in data.get("viewer_topics", [])]
    target_diameter = float(data.get("target_diameter", 0.05))
    push_depth = float(data.get("push_depth", 0.002)) # 追加
    gripper_offset = float(data.get("gripper_offset", 0.006)) # 追加
    arm_offset = float(data.get("arm_offset", 0.0)) # 追加
    # --- 追加: センタリングパラメータの読み出し ---
    contact_threshold = float(data.get("contact_threshold", 0.25))
    contact_margin = float(data.get("contact_margin", 0.05))
    centering_step_grip = float(data.get("centering_step_grip", 0.001))
    centering_step_arm = float(data.get("centering_step_arm", 0.001))
    centering_min_step_arm = float(data.get("centering_min_step_arm", 0.0001))
    centering_min_step_grip = float(data.get("centering_min_step_grip", 0.0005))
    safe_margin = float(data.get("safe_margin", 0.020))
    # --------------------------------------------
    initial_arm_pose = data.get("initial_arm_pose", [1.7317156838287737, -1.40045219180025, 1.1475539831862718, -1.3247049022636963, -1.5844098949604524, 0.9487609813841176])
    image_throttle_hz = float(data.get("image_throttle_hz", 10.0))
    other_throttle_hz = float(data.get("other_throttle_hz", 100.0))
    enable_image_compression = bool(data.get("enable_image_compression", True))
    stop_key = str(data.get("stop_key", "q"))
    discard_key = str(data.get("discard_key", "x"))
    
    # base_tcp_pose の読み出しと構造体化
    base_pose_data = data.get("base_tcp_pose", {})
    base_tcp_pose = PoseConfig(
        x=float(base_pose_data.get("x", 0.0)),
        y=float(base_pose_data.get("y", 0.0)),
        z=float(base_pose_data.get("z", 0.0)),
        rx=float(base_pose_data.get("rx", 0.0)),
        ry=float(base_pose_data.get("ry", 0.0)),
        rz=float(base_pose_data.get("rz", 0.0)),
    )

    return CaptureConfig(
        output_root=output_root,
        task_name=task_name,
        prompt=prompt,
        label=label,
        target_diameter=target_diameter,
        push_depth=push_depth,               # 追加
        gripper_offset=gripper_offset, # 追加
        arm_offset=arm_offset,             # 追加
        # --- 追加: 戻り値に含める ---
        contact_threshold=contact_threshold,
        contact_margin=contact_margin,
        centering_step_grip=centering_step_grip,
        centering_step_arm=centering_step_arm,
        centering_min_step_arm=centering_min_step_arm,
        centering_min_step_grip=centering_min_step_grip,
        safe_margin=safe_margin, # ★追加
        # ----------------------------
        initial_arm_pose=initial_arm_pose,
        base_tcp_pose=base_tcp_pose,         # 追加
        viewer_topics=viewer_topics,
        image_throttle_hz=image_throttle_hz,
        other_throttle_hz=other_throttle_hz,
        enable_image_compression=enable_image_compression,
        stop_key=stop_key,
        discard_key=discard_key,
        topics=topics,
        bag=bag,
        raw=data,
        source_path=path,
    )
