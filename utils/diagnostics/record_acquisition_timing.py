#!/usr/bin/env python3
"""Read-only short acquisition timing check. No robot commands or image pixels saved."""
import argparse
import json
from pathlib import Path
import time
from verify_acquisition_timing import NODES, SENSORS, TIMING, validate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--out', type=Path, default=Path('acquisition_timing_check'))
    args = parser.parse_args()
    if not 0 < args.seconds <= 180:
        parser.error('--seconds must be in (0,180]')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String
    from sensor_msgs.msg import CompressedImage
    from geometry_msgs.msg import WrenchStamped
    args.out.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = Node('acquisition_timing_check')
    collecting = False
    records = []
    sensors = {t: [] for t in SENSORS}
    malformed = []
    subscriptions = []
    def receive_sensor(topic):
        def callback(msg):
            if not collecting: return
            stamp = msg.header.stamp
            row = {'header_ns': stamp.sec*1_000_000_000+stamp.nanosec,
                   'arrival_monotonic_ns': time.monotonic_ns()}
            if isinstance(msg, WrenchStamped):
                w = msg.wrench
                row['wrench'] = [w.force.x,w.force.y,w.force.z,w.torque.x,w.torque.y,w.torque.z]
            sensors[topic].append(row)
        return callback
    def receive_timing(topic):
        def callback(msg):
            if not collecting: return
            try:
                report = json.loads(msg.data)
                records.append((topic, report))
            except ValueError as exc:
                malformed.append(str(exc))
        return callback
    with (args.out/'timing_records.jsonl').open('w') as stream:
        for i, topic in enumerate(SENSORS):
            subscriptions.append(node.create_subscription(CompressedImage if i<2 else WrenchStamped,
                topic, receive_sensor(topic), QoSProfile(
                    depth=128 if i<2 else 4096,
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE)))
        for topic in TIMING:
            subscriptions.append(node.create_subscription(String, topic, receive_timing(topic), QoSProfile(
                depth=100, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE)))
        try:
            warmup = time.monotonic()+3
            while rclpy.ok() and time.monotonic()<warmup:
                rclpy.spin_once(node, timeout_sec=.05)
            collecting = True
            print(f'Collecting {args.seconds:g} s; no robot commands. Output: {args.out.resolve()}', flush=True)
            end = time.monotonic()+args.seconds
            while rclpy.ok() and time.monotonic()<end:
                rclpy.spin_once(node, timeout_sec=.05)
        except KeyboardInterrupt:
            pass
        finally:
            collecting = False
        # File I/O and JSON encoding belong after acquisition, not in 1 kHz callbacks.
        for topic, report in records:
            stream.write(json.dumps({'topic': topic, 'report': report})+'\n')
    (args.out/'sensor_headers.json').write_text(json.dumps(sensors))
    result = validate(records, sensors)
    result['malformed_reports'] = malformed
    if malformed: result['schema_and_mapping_checks_passed'] = False
    (args.out/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
    return 0 if result['schema_and_mapping_checks_passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
