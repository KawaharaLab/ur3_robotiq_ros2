"""Extract trials from a batch rosbag2 recording based on /trial_marker using rosbag2_py."""

import csv
import json
import logging
import datetime
from pathlib import Path
from collections import defaultdict
from typing import Dict, List, Optional

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py import message_to_ordereddict
import yaml

# 既存の構成要素をインポート
from .config import CaptureConfig, load_capture_config
from .bag_converter import (
    _sanitize_topic, _flatten, _format_csv_value, 
    _load_csv_field_map, _prepare_message_types, 
    _maybe_decompress_file_bag
)

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("batch_extractor")

# --- ハードコードされたパスの設定 ---
TOOL_DIR = Path("/home/tsumura/my_robotiq_ws/src/ur3_robotiq_ros2/data_capture_tools")
DEFAULT_CONFIG_PATH = TOOL_DIR / "config" / "data_capture_extract.yaml"

def get_trial_intervals(bag_to_read: Path, storage_id: str) -> List[dict]:
    """Bagを走査して試行の区間(開始/終了)とパラメータを特定する[cite: 2]"""
    intervals = []
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_to_read), storage_id=storage_id),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    )

    from std_msgs.msg import String
    current_trial = None

    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic == "/trial_marker":
            msg = deserialize_message(data, String)
            parts = msg.data.split(',')
            action = parts[0] # START or END
            
            info = {"action": action}
            for p in parts[1:]:
                if ':' in p:
                    k, v = p.split(':', 1)
                    info[k] = v

            if action == "START":
                current_trial = {
                    "trial_id": info.get("trial", "unknown"),
                    "start_ns": stamp,
                    "params": info
                }
            elif action == "END" and current_trial:
                current_trial["end_ns"] = stamp
                intervals.append(current_trial)
                current_trial = None
    return intervals

def convert_trial_range(
    bag_uri: Path,
    output_dir: Path,
    config: CaptureConfig,
    start_ns: int,
    end_ns: int,
    logger=None
) -> None:
    """指定された時間範囲のみをデータセットとして書き出す[cite: 2]"""
    output_dir.mkdir(parents=True, exist_ok=True)
    
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_uri), storage_id=config.bag.storage),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    )

    topic_specs = {spec.name: spec for spec in config.topics}
    message_types = _prepare_message_types(topic_specs.values())
    bridge = CvBridge()
    image_dirs: Dict[str, Path] = {}
    csv_rows_by_topic = defaultdict(list)
    
    # CSVフィールドマップのロード[cite: 2]
    csv_config_path = output_dir.parent / "csv_fields.yaml"
    csv_field_map = _load_csv_field_map(csv_config_path if csv_config_path.exists() else None, logger)

    while reader.has_next():
        topic, data, stamp = reader.read_next()
        
        if not (start_ns <= stamp <= end_ns):
            continue
            
        spec = topic_specs.get(topic)
        if spec is None:
            continue

        msg_cls = message_types[topic]
        message = deserialize_message(data, msg_cls)

        if spec.mode == "image" or spec.type.endswith("CompressedImage"):
            folder = image_dirs.get(topic)
            if folder is None:
                folder = output_dir / "images" / _sanitize_topic(topic)
                folder.mkdir(parents=True, exist_ok=True)
                image_dirs[topic] = folder

            image = bridge.compressed_imgmsg_to_cv2(message) if isinstance(message, CompressedImage) \
                    else bridge.imgmsg_to_cv2(message, desired_encoding=spec.encoding or "bgr8")
            cv2.imwrite(str(folder / f"{stamp}.png"), image)
        else:
            row = {"stamp_ns": stamp}
            flattened = _flatten(message_to_ordereddict(message))
            prefix = _sanitize_topic(topic)
            for key, value in flattened.items():
                column = f"{prefix}.{key}" if key else prefix
                row[column] = _format_csv_value(value)
            
            field_map = csv_field_map.get(topic)
            if field_map:
                row = {dst: (stamp if src == "stamp_ns" else row.get(src)) for src, dst in field_map.items()}
            csv_rows_by_topic[prefix].append(row)

    csv_dir = output_dir / "csv"
    csv_dir.mkdir(parents=True, exist_ok=True)
    for prefix, rows in csv_rows_by_topic.items():
        if not rows: continue
        fieldnames = ["stamp_ns"] + sorted({k for r in rows for k in r.keys() if k != "stamp_ns"})
        with (csv_dir / f"{prefix}.csv").open("w", newline="", encoding="utf-8") as h:
            writer = csv.DictWriter(h, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True, help="メタデータがあるBagディレクトリ")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH), help="Capture YAMLのパス (デフォルトはハードコードされたパス)[cite: 2]")
    args = parser.parse_args()

    bag_path = Path(args.bag)
    config = load_capture_config(Path(args.config))
    
    # 解凍処理[cite: 2]
    bag_to_read, temp_handle = _maybe_decompress_file_bag(bag_path, config, logger)

    try:
        logger.info(f"Using config: {args.config}[cite: 2]")
        logger.info("Scanning for trial markers...")
        intervals = get_trial_intervals(bag_to_read, config.bag.storage)
        logger.info(f"Found {len(intervals)} trials in this bag.")

        for trial in intervals:
            trial_id = trial["trial_id"]
            # bag/.. はバッチのタイムスタンプフォルダ。その下に trial_X を作成[cite: 2]
            trial_output_dir = bag_path.parent.parent / f"trial_{trial_id}"
            
            logger.info(f">>> Extracting Trial {trial_id} to {trial_output_dir.name}")
            convert_trial_range(
                bag_to_read, trial_output_dir, config,
                trial["start_ns"], trial["end_ns"], logger
            )
            
            with (trial_output_dir / "params.json").open("w", encoding="utf-8") as f:
                json.dump(trial["params"], f, indent=4)

    finally:
        if temp_handle:
            temp_handle.cleanup()

if __name__ == "__main__":
    main()