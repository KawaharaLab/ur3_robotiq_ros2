#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TARGET_TOPICS = {
    "/force_torque/left": "left_force.csv",
    "/force_torque/right": "right_force.csv",
}


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Extract left/right WrenchStamped topics from a ROS 2 bag."
    )
    parser.add_argument(
        "bag_path",
        help="Bag directory or .db3 path",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Output directory. Default: <bag directory>/csv",
    )
    args = parser.parse_args()

    supplied_path = Path(args.bag_path).expanduser().resolve()
    bag_dir = supplied_path.parent if supplied_path.suffix == ".db3" else supplied_path

    if not bag_dir.exists():
        raise FileNotFoundError(f"Bag directory does not exist: {bag_dir}")

    output_dir = (
        Path(args.output_dir).expanduser().resolve()
        if args.output_dir
        else bag_dir / "csv"
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_dir),
        storage_id="sqlite3",
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)

    topic_types = {
        item.name: item.type
        for item in reader.get_all_topics_and_types()
    }

    missing = [topic for topic in TARGET_TOPICS if topic not in topic_types]
    if missing:
        available = "\n".join(sorted(topic_types))
        raise RuntimeError(
            "Required topics were not found:\n"
            + "\n".join(missing)
            + "\n\nAvailable topics:\n"
            + available
        )

    message_classes = {
        topic: get_message(topic_types[topic])
        for topic in TARGET_TOPICS
    }

    files = {}
    writers = {}
    counts = {topic: 0 for topic in TARGET_TOPICS}

    header = [
        "bag_time_ns",
        "header_time_ns",
        "elapsed_s",
        "frame_id",
        "force_x",
        "force_y",
        "force_z",
        "torque_x",
        "torque_y",
        "torque_z",
    ]

    first_bag_time_ns = None

    try:
        for topic, filename in TARGET_TOPICS.items():
            handle = (output_dir / filename).open("w", newline="")
            files[topic] = handle
            writer = csv.writer(handle)
            writer.writerow(header)
            writers[topic] = writer

        while reader.has_next():
            topic, serialized_data, bag_time_ns = reader.read_next()

            if topic not in TARGET_TOPICS:
                continue

            if first_bag_time_ns is None:
                first_bag_time_ns = int(bag_time_ns)

            msg = deserialize_message(
                serialized_data,
                message_classes[topic],
            )

            header_time_ns = stamp_to_ns(msg.header.stamp)
            elapsed_s = (int(bag_time_ns) - first_bag_time_ns) / 1e9

            writers[topic].writerow([
                int(bag_time_ns),
                header_time_ns,
                f"{elapsed_s:.9f}",
                msg.header.frame_id,
                msg.wrench.force.x,
                msg.wrench.force.y,
                msg.wrench.force.z,
                msg.wrench.torque.x,
                msg.wrench.torque.y,
                msg.wrench.torque.z,
            ])
            counts[topic] += 1
    finally:
        for handle in files.values():
            handle.close()

    print(f"Bag directory: {bag_dir}")
    print(f"Output directory: {output_dir}")
    for topic, count in counts.items():
        print(f"{topic}: {count} messages -> {output_dir / TARGET_TOPICS[topic]}")


if __name__ == "__main__":
    main()
