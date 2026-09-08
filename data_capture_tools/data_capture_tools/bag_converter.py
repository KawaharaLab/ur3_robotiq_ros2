"""Convert rosbag2 recordings into image files and CSV logs."""

from __future__ import annotations

import datetime
import logging
import os

import csv
import json
import shutil
import subprocess
import tempfile
from collections import defaultdict
from pathlib import Path
from typing import Dict, Iterable, Mapping, MutableMapping, Optional

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message
import yaml

from .config import CaptureConfig, TopicSpec


def _sanitize_topic(topic: str) -> str:
    return topic.strip("/").replace("/", "_") or "root"


def _flatten(value, prefix: str = "", out: Optional[MutableMapping[str, object]] = None):
    if out is None:
        out = {}

    if isinstance(value, Mapping):
        for key, inner in value.items():
            next_prefix = f"{prefix}.{key}" if prefix else str(key)
            _flatten(inner, next_prefix, out)
    elif isinstance(value, (list, tuple)):
        for idx, inner in enumerate(value):
            next_prefix = f"{prefix}[{idx}]"
            _flatten(inner, next_prefix, out)
    else:
        key = prefix if prefix else "value"
        out[key] = value
    return out


def _format_csv_value(value):
    if isinstance(value, (list, tuple, dict)):
        return json.dumps(value, ensure_ascii=False)
    return value


def _load_csv_field_map(path: Optional[Path], logger=None) -> dict[str, dict[str, str]]:
    if path is None or not path.exists():
        return {}

    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    topics = data.get("topics", {}) if isinstance(data, dict) else {}
    field_map: dict[str, dict[str, str]] = {}
    for topic, entry in topics.items():
        if not isinstance(entry, dict):
            continue
        fields = entry.get("fields", {})
        if isinstance(fields, dict):
            field_map[str(topic)] = {str(k): str(v) for k, v in fields.items()}

    if logger:
        logger.info(f"Loaded CSV field map for {len(field_map)} topics from {path}")
    return field_map


def _prepare_message_types(topic_specs: Iterable[TopicSpec]) -> Dict[str, type]:
    mapping: Dict[str, type] = {}
    for spec in topic_specs:
        mapping[spec.name] = get_message(spec.type)
    return mapping


def _maybe_decompress_file_bag(
    bag_uri: Path,
    config: Optional[CaptureConfig] = None,
    logger=None,
    allow_corrupt_zstd: bool = False,
) -> tuple[Path, Optional[tempfile.TemporaryDirectory]]:
    """Decompress bags recorded with file compression if needed."""

    metadata_path = bag_uri / "metadata.yaml"
    if not metadata_path.exists():
        return bag_uri, None

    metadata = yaml.safe_load(metadata_path.read_text())
    bag_info = metadata.get("rosbag2_bagfile_information", {})
    relative_paths = bag_info.get("relative_file_paths", []) or []
    mode = str(bag_info.get("compression_mode", "")).lower()
    has_zstd = any(path.endswith(".zstd") for path in relative_paths) or any(
        bag_uri.glob("*.zstd")
    )
    if mode != "file" and not has_zstd:
        return bag_uri, None

    temp_dir = tempfile.TemporaryDirectory(prefix="bag_decompress_")
    target_dir = Path(temp_dir.name) / bag_uri.name
    target_dir.mkdir(parents=True, exist_ok=True)

    try:
        _decompress_bag_contents(
            bag_uri,
            target_dir,
            metadata,
            bag_info,
            logger,
            allow_corrupt_zstd,
        )
    except Exception:
        temp_dir.cleanup()
        raise

    return target_dir, temp_dir


def _decompress_bag_contents(
    source_dir: Path,
    target_dir: Path,
    metadata: dict,
    bag_info: dict,
    logger=None,
    allow_corrupt_zstd: bool = False,
) -> None:
    """Copy bag contents and expand any *.zstd segments using the zstd CLI."""

    zstd_bin = shutil.which("zstd")
    if zstd_bin is None:
        raise RuntimeError(
            "Unable to decompress rosbag: 'zstd' command not found. Install the 'zstd' package or record without file compression."
        )

    if logger:
        logger.info(
            "Detected file-compressed bag. Falling back to 'zstd -d' to expand segments."
        )

    # Copy any auxiliary files (e.g., calibration data) verbatim.
    for item in source_dir.iterdir():
        if item.name == "metadata.yaml" or item.name.endswith(".zstd"):
            continue
        destination = target_dir / item.name
        if item.is_dir():
            shutil.copytree(item, destination)
        else:
            shutil.copy2(item, destination)

    rel_paths = bag_info.get("relative_file_paths", []) or [
        path.name for path in source_dir.glob("*.db3*")
    ]

    updated_relative_paths: list[str] = []
    skipped_paths: set[str] = set()
    for rel_path in rel_paths:
        rel_path = str(rel_path)
        src_path = source_dir / rel_path
        if not src_path.exists():
            raise FileNotFoundError(
                f"metadata.yaml references '{rel_path}', but the file does not exist in {source_dir}"
            )

        if rel_path.endswith(".zstd"):
            dest_name = rel_path[: -len(".zstd")]
            dest_path = target_dir / dest_name
            dest_path.parent.mkdir(parents=True, exist_ok=True)
            cmd = [
                zstd_bin,
                "-d",
                "--force",
                "-o",
                str(dest_path),
                str(src_path),
            ]
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                check=False,
            )
            if result.returncode != 0:
                stderr = result.stderr.strip()
                stdout = result.stdout.strip()
                detail = stderr or stdout or "no output"
                if allow_corrupt_zstd:
                    skipped_paths.add(rel_path)
                    if logger:
                        logger.warning(
                            f"Skipping corrupt zstd segment {src_path}: {detail}"
                        )
                    continue
                raise RuntimeError(
                    f"zstd failed (exit {result.returncode}) for {src_path}: {detail}"
                )
            if logger and result.stderr:
                logger.debug(result.stderr.strip())
            updated_relative_paths.append(dest_name)
        else:
            dest_path = target_dir / rel_path
            dest_path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src_path, dest_path)
            updated_relative_paths.append(rel_path)

    bag_info["relative_file_paths"] = updated_relative_paths
    bag_info["compression_mode"] = "NONE"
    bag_info["compression_format"] = "NONE"

    for file_entry in bag_info.get("files", []) or []:
        path_value = file_entry.get("path")
        if not isinstance(path_value, str):
            continue
        if path_value.endswith(".zstd"):
            path_value = path_value[: -len(".zstd")]
            file_entry["path"] = path_value
        if any(
            path_value == skipped[: -len(".zstd")] or path_value == skipped
            for skipped in skipped_paths
        ):
            file_entry["path"] = None

    if skipped_paths:
        bag_info["files"] = [
            entry
            for entry in bag_info.get("files", []) or []
            if entry.get("path")
        ]

    target_metadata = target_dir / "metadata.yaml"
    target_metadata.write_text(yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8")


def convert_bag_to_dataset(
    bag_uri: Path,
    output_dir: Path,
    config: CaptureConfig,
    logger=None,
    cutoff_stamp_ns: Optional[int] = None,
    allow_corrupt_zstd: bool = False,
) -> None:
    """Read a rosbag2 recording and emit PNG/CSV artifacts."""

    if logger:
        logger.info(f"Converting bag '{bag_uri}' into '{output_dir}'")

    bag_to_read, temp_handle = _maybe_decompress_file_bag(
        bag_uri,
        config,
        logger,
        allow_corrupt_zstd,
    )

    try:
        reader = SequentialReader()
        storage_options = StorageOptions(
            uri=str(bag_to_read),
            storage_id=config.bag.storage,
        )
        converter_options = ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        )
        reader.open(storage_options, converter_options)

        topic_specs = {spec.name: spec for spec in config.topics}
        message_types = _prepare_message_types(topic_specs.values())

        bridge = CvBridge()
        image_dirs: Dict[str, Path] = {}
        csv_rows_by_topic: dict[str, list[dict[str, object]]] = defaultdict(list)
        skipped_messages: dict[str, int] = defaultdict(int)
        csv_field_map = _load_csv_field_map(
            getattr(config, "csv_config_path", None),
            logger,
        )

        while reader.has_next():
            topic, data, stamp = reader.read_next()
            if cutoff_stamp_ns is not None and stamp > cutoff_stamp_ns:
                continue
            spec = topic_specs.get(topic)
            if spec is None:
                continue

            msg_cls = message_types[topic]
            try:
                message = deserialize_message(data, msg_cls)
            except Exception as exc:  # noqa: BLE001
                skipped_messages[topic] += 1
                if logger and skipped_messages[topic] == 1:
                    logger.warning(
                        f"Skipping messages for topic {topic}: deserialization failed ({exc})"
                    )
                continue

            # Treat explicit image modes and compressed image types as image outputs.
            if spec.mode == "image" or spec.type.endswith("CompressedImage"):
                folder = image_dirs.get(topic)
                if folder is None:
                    folder = output_dir / "images" / _sanitize_topic(topic)
                    folder.mkdir(parents=True, exist_ok=True)
                    image_dirs[topic] = folder

                if isinstance(message, CompressedImage):
                    image = bridge.compressed_imgmsg_to_cv2(message)
                else:
                    image = bridge.imgmsg_to_cv2(
                        message,
                        desired_encoding=spec.encoding or "bgr8",
                    )

                filename = folder / f"{stamp}.png"
                if not cv2.imwrite(str(filename), image) and logger:
                    logger.warning(
                        f"Failed to write image for topic {topic} at stamp {stamp}"
                    )
            else:
                row = {
                    "stamp_ns": stamp,
                }
                flattened = _flatten(message_to_ordereddict(message))
                prefix = _sanitize_topic(topic)
                for key, value in flattened.items():
                    column = f"{prefix}.{key}" if key else prefix
                    row[column] = _format_csv_value(value)
                field_map = csv_field_map.get(topic)
                if field_map:
                    remapped = {}
                    for src, dst in field_map.items():
                        if src == "stamp_ns":
                            remapped[dst] = stamp
                        else:
                            remapped[dst] = row.get(src)
                    row = remapped
                csv_rows_by_topic[prefix].append(row)

        csv_dir = output_dir / "csv"
        csv_dir.mkdir(parents=True, exist_ok=True)
        for prefix, rows in csv_rows_by_topic.items():
            if not rows:
                continue
            data_fields = sorted(
                {key for row in rows for key in row.keys() if key != "stamp_ns"}
            )
            fieldnames = ["stamp_ns", *data_fields]
            csv_path = csv_dir / f"{prefix}.csv"
            with csv_path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(rows)
            if logger:
                logger.info(f"Wrote {len(rows)} rows to {csv_path}")

        if logger and skipped_messages:
            total_skipped = sum(skipped_messages.values())
            topics = ", ".join(
                f"{name} ({count})" for name, count in skipped_messages.items()
            )
            logger.warning(
                f"Skipped {total_skipped} messages due to deserialization errors: {topics}"
            )
    finally:
        if temp_handle:
            temp_handle.cleanup()

def find_bags_ultra_fast(root_path: Path, force: bool):
    """
    ROS2 Bagの構造を活かした高速スキャナ。
    .success ファイルがある場合は処理済みとみなす。
    """
    targets = []
    skipped_paths = []
    
    for root, dirs, files in os.walk(root_path):
        root_p = Path(root)

        # 統一ルール: .success があれば処理済みとみなす
        if ".success" in files and not force:
            skipped_paths.append(str(root_p))
            # 枝切り: images, csv, bag など配下の探索をすべてスキップ
            dirs.clear() 
            continue

        if "bag" in dirs:
            bag_root = root_p / "bag"
            metas = list(bag_root.rglob("metadata.yaml"))
            targets.extend(metas)
            # bagを見つけたらその横のフォルダは見なくて良い
            if "images" in dirs: dirs.remove("images")
            if "csv" in dirs: dirs.remove("csv")

    return targets, skipped_paths

def cli_main():
    """成功・失敗のマーカーファイルを生成しながら変換を実行する。"""
    import argparse
    
    now_str = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    TOOL_DIR = Path("/home/tsumura/my_robotiq_ws/src/ur3_robotiq_ros2/data_capture_tools")
    LOG_FILE = TOOL_DIR / f"conversion_{now_str}.log"
    FIXED_CONFIG_PATH = TOOL_DIR / "config" / "data_capture_extract.yaml"

    parser = argparse.ArgumentParser(description="Convert rosbag2 data into PNG/CSV")
    parser.add_argument("--root", required=True, help="探索を開始するルートパス")
    parser.add_argument("--force", action="store_true", help="強制的に再処理")
    parser.add_argument("--skip-corrupt-zstd", action="store_true", help="解凍失敗時にスキップ")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(message)s',
        handlers=[logging.FileHandler(LOG_FILE, encoding='utf-8'), logging.StreamHandler()]
    )
    logger = logging.getLogger("bag_converter")

    from .config import load_capture_config
    root_dir = Path(args.root).resolve()
    cfg = load_capture_config(FIXED_CONFIG_PATH)

    bag_metadatas, skipped_folders = find_bags_ultra_fast(root_dir, args.force)
    
    stats = {"success": 0, "fail": 0, "skip": len(skipped_folders), "total": len(bag_metadatas)}
    failed_paths = []

    if not bag_metadatas:
        logger.info("✨ No new bags found to process.")
        return

    logger.info(f"🚀 Starting process for {stats['total']} sessions.")

    for i, metadata_path in enumerate(bag_metadatas, 1):
        bag_path = metadata_path.parent
        output_dir = bag_path.parent.parent
        
        # 以前の失敗マーカーがあれば削除しておく
        if (output_dir / ".failed").exists():
            (output_dir / ".failed").unlink()

        logger.info(f"[{i}/{stats['total']}] Processing: {output_dir.name}")

        try:
            csv_config_path = output_dir / "csv_fields.yaml"
            cfg.csv_config_path = csv_config_path if csv_config_path.exists() else None

            convert_bag_to_dataset(bag_path, output_dir, cfg, allow_corrupt_zstd=args.skip_corrupt_zstd)
            
            # --- 成功マーカーの作成 ---
            (output_dir / ".success").touch()
            stats["success"] += 1
            
        except Exception as e:
            logger.error(f"❌ Error in {output_dir.name}: {e}")
            
            # --- 失敗マーカーの作成 (エラー内容を記録) ---
            with open(output_dir / ".failed", "w", encoding="utf-8") as f:
                f.write(f"Timestamp: {now_str}\nError: {str(e)}")
            
            stats["fail"] += 1
            failed_paths.append(str(bag_path))
            
    # --- サマリー出力 ---
    # (前回と同様のレポート処理。中略)
    logger.info(f"Summary: Success {stats['success']}, Fail {stats['fail']}, Skip {stats['skip']}")

# usage: ros2 run data_capture_tools bag_to_dataset --root /path/to/data
