#!/usr/bin/env python3
"""Read-only, 60 s collection of optional acquisition summaries and header timing."""
import argparse
import csv
import json
from pathlib import Path
import platform
import statistics
import time


SENSORS = {
    "/gelsight/left/image_raw/compressed": "sensor_msgs/msg/CompressedImage",
    "/gelsight/right/image_raw/compressed": "sensor_msgs/msg/CompressedImage",
    "/force_torque/left": "geometry_msgs/msg/WrenchStamped",
    "/force_torque/right": "geometry_msgs/msg/WrenchStamped",
}
DIAGNOSTICS = [f"/{node}/acquisition_diagnostics" for node in (
    "gelsight_mini_publisher_1", "gelsight_mini_publisher_2",
    "mms101_driver_1", "mms101_driver_2")]


def stats(values):
    if not values:
        return {"count": 0}
    ordered = sorted(values)
    return {"count": len(values), "median": statistics.median(values),
            "p95": ordered[round((len(ordered) - 1) * 0.95)],
            "min": ordered[0], "max": ordered[-1]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=60)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    if not 0 < args.seconds <= 600:
        parser.error("--seconds must be in (0, 600]")
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    import rclpy
    from rosidl_runtime_py.utilities import get_message
    from std_msgs.msg import String

    args.out.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = Node("acquisition_diagnostics_recorder")
    data = {topic: [] for topic in SENSORS}
    diagnostic_counts = {topic: 0 for topic in DIAGNOSTICS}
    subscription_errors = {}
    malformed = []
    subscriptions = []
    collecting = False
    diagnostic_file = (args.out / "diagnostics.jsonl").open("w", encoding="utf-8")

    def sensor_callback(topic):
        def receive(msg):
            if collecting:
                mono, wall = time.monotonic_ns(), time.time_ns()
                stamp = msg.header.stamp
                data[topic].append((mono, wall, stamp.sec * 1_000_000_000 + stamp.nanosec))
        return receive

    def diagnostic_callback(topic):
        def receive(msg):
            if not collecting:
                return
            try:
                report = json.loads(msg.data)
                diagnostic_file.write(json.dumps({"topic": topic, "receive_wall_ns": time.time_ns(),
                                                  "report": report}) + "\n")
                diagnostic_counts[topic] += 1
            except ValueError as error:
                malformed.append({"topic": topic, "error": str(error)})
        return receive

    start = None
    try:
        for topic, name in SENSORS.items():
            try:
                subscriptions.append(node.create_subscription(
                    get_message(name), topic, sensor_callback(topic), qos_profile_sensor_data))
            except Exception as error:
                subscription_errors[topic] = str(error)
        for topic in DIAGNOSTICS:
            subscriptions.append(node.create_subscription(
                String, topic, diagnostic_callback(topic), qos_profile_sensor_data))
        warmup_end = time.monotonic() + 3
        while time.monotonic() < warmup_end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
        endpoints = {topic: [{"node": info.node_name, "namespace": info.node_namespace,
                              "type": info.topic_type, "qos": str(info.qos_profile)}
                             for info in node.get_publishers_info_by_topic(topic)]
                     for topic in [*SENSORS, *DIAGNOSTICS]}
        collecting = True
        start = time.monotonic()
        print(f"Collecting for {args.seconds:g} s. No robot commands; image pixels are not saved.", flush=True)
        while time.monotonic() - start < args.seconds and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        collecting = False
        diagnostic_file.close()
        metrics = {}
        with (args.out / "sensor_times.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["topic", "arrival_monotonic_ns", "callback_wall_ns", "header_ns"])
            for topic, rows in data.items():
                writer.writerows((topic, *row) for row in rows)
                headers = [row[2] for row in rows]
                intervals = [(b-a)/1e6 for a, b in zip(headers, headers[1:])]
                metrics[topic] = {
                    "messages": len(rows),
                    "receive_hz": (len(rows)-1)*1e9/(rows[-1][0]-rows[0][0]) if len(rows)>1 else None,
                    "header_interval_ms": stats(intervals),
                    "callback_wall_minus_header_ms": stats([(r[1]-r[2])/1e6 for r in rows]),
                    "duplicate_headers": len(headers)-len(set(headers)),
                    "backward_headers": sum(value < 0 for value in intervals),
                }
        result = {
            "schema_version": 1, "host": platform.node(), "requested_seconds": args.seconds,
            "actual_seconds": time.monotonic()-start if start is not None else 0,
            "sensors": metrics, "diagnostic_report_counts": diagnostic_counts,
            "subscription_errors": subscription_errors, "malformed_reports": malformed,
            "publisher_endpoints": locals().get("endpoints", {}),
            "missing_sensors": [key for key, value in data.items() if not value],
            "missing_diagnostics": [key for key, value in diagnostic_counts.items() if not value],
            "note": "Extra subscriber load is present. Header timing and host processing statistics do not calibrate exposure/ADC latency or physical synchronization.",
        }
        (args.out / "summary.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(result, indent=2))
        print(f"Result folder: {args.out.resolve()}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
