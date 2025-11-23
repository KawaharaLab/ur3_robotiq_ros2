"""Convert rosbag2 recordings into image files and CSV logs."""

from __future__ import annotations

import csv
import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import Dict, Iterable, Mapping, MutableMapping, Optional

import cv2
from cv_bridge import CvBridge
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


def _prepare_message_types(topic_specs: Iterable[TopicSpec]) -> Dict[str, type]:
    mapping: Dict[str, type] = {}
    for spec in topic_specs:
        mapping[spec.name] = get_message(spec.type)
    return mapping


def _maybe_decompress_file_bag(
    bag_uri: Path, config: CaptureConfig, logger=None
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
                check=True,
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
        if isinstance(path_value, str) and path_value.endswith(".zstd"):
            file_entry["path"] = path_value[: -len(".zstd")]

    target_metadata = target_dir / "metadata.yaml"
    target_metadata.write_text(yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8")


def convert_bag_to_dataset(
    bag_uri: Path,
    output_dir: Path,
    config: CaptureConfig,
    logger=None,
) -> None:
    """Read a rosbag2 recording and emit PNG/CSV artifacts."""

    if logger:
        logger.info(f"Converting bag '{bag_uri}' into '{output_dir}'")

    bag_to_read, temp_handle = _maybe_decompress_file_bag(bag_uri, config, logger)

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
        csv_rows: list[dict[str, object]] = []

        while reader.has_next():
            topic, data, stamp = reader.read_next()
            spec = topic_specs.get(topic)
            if spec is None:
                continue

            msg_cls = message_types[topic]
            message = deserialize_message(data, msg_cls)

            if spec.mode == "image":
                folder = image_dirs.get(topic)
                if folder is None:
                    folder = output_dir / "images" / _sanitize_topic(topic)
                    folder.mkdir(parents=True, exist_ok=True)
                    image_dirs[topic] = folder
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
                    row[column] = value
                csv_rows.append(row)

        csv_dir = output_dir / "csv"
        csv_dir.mkdir(parents=True, exist_ok=True)
        if csv_rows:
            data_fields = sorted(
                {key for row in csv_rows for key in row.keys() if key != "stamp_ns"}
            )
            fieldnames = ["stamp_ns", *data_fields]
            csv_path = csv_dir / "timeseries.csv"
            with csv_path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(csv_rows)
            if logger:
                logger.info(f"Wrote {len(csv_rows)} rows to {csv_path}")
    finally:
        if temp_handle:
            temp_handle.cleanup()


def cli_main():
    """Entry point for manual conversion via ros2 run."""

    import argparse

    parser = argparse.ArgumentParser(description="Convert rosbag2 data into PNG/CSV outputs")
    parser.add_argument("--bag", required=True, help="Path to the rosbag2 directory (metadata.yaml parent)")
    parser.add_argument("--config", required=True, help="Path to the capture YAML config")
    parser.add_argument("--output", required=True, help="Directory to place converted artifacts")
    args = parser.parse_args()

    from .config import load_capture_config

    cfg = load_capture_config(args.config)
    convert_bag_to_dataset(Path(args.bag), Path(args.output), cfg)