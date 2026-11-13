#!/usr/bin/env python3
"""Read-only live force ranges; no bag/files, no motion, no filtering of samples."""
import argparse
from collections import deque
import math
import statistics
import time


def describe(values, baseline, threshold, jump_threshold, previous=None):
    finite = [v for v in values if math.isfinite(v)]
    steps = []
    last = previous
    for value in values:
        if math.isfinite(value):
            if last is not None and math.isfinite(last):
                steps.append(abs(value - last))
            last = value
        else:
            last = None
    return dict(
        n=len(values),
        invalid=len(values) - len(finite),
        median=statistics.median(finite) if finite else None,
        minimum=min(finite) if finite else None,
        maximum=max(finite) if finite else None,
        over=sum(abs(v - baseline) >= threshold for v in finite),
        jumps=sum(v >= jump_threshold for v in steps),
        max_jump=max(steps, default=0.0),
        last=last,
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--baseline-seconds", type=float, default=2.0)
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--threshold-n", type=float, default=30.0)
    parser.add_argument("--jump-n", type=float, default=2.5)
    args = parser.parse_args(argv)
    for name, value in vars(args).items():
        if not math.isfinite(value) or value <= 0:
            parser.error(name + " must be positive and finite")
    if args.interval > 10 or args.baseline_seconds > 30:
        parser.error("interval <= 10 s and baseline-seconds <= 30 s required")
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import WrenchStamped

    rclpy.init()
    node = None
    try:
        node = Node("force_live_readonly_monitor")
        qos = QoSProfile(depth=2000, reliability=ReliabilityPolicy.BEST_EFFORT)
        queues = {side: deque(maxlen=60000) for side in ("left", "right")}
        totals = {
            side: dict(
                samples=0,
                invalid=0,
                over=0,
                jumps=0,
                queue_overflow=0,
                peak_delta=0.0,
                max_jump=0.0,
            )
            for side in queues
        }
        baseline = {}
        previous = {side: None for side in queues}

        def receive(side, message):
            q = queues[side]
            if len(q) == q.maxlen:
                totals[side]["queue_overflow"] += 1
            q.append(message.wrench.force.y)

        subscriptions = [
            node.create_subscription(
                WrenchStamped,
                "/force_torque/" + side,
                lambda msg, side=side: receive(side, msg),
                qos,
            )
            for side in queues
        ]
        print(
            "READ ONLY: no files, no robot commands. Fy only. Keep unloaded and still during baseline.",
            flush=True,
        )
        print("Waiting for both force topics (10 s timeout)...", flush=True)
        deadline = time.monotonic() + 10
        while not all(queues.values()):
            if time.monotonic() > deadline:
                print(
                    "Missing force topic(s): "
                    + ", ".join(side for side, q in queues.items() if not q)
                )
                return 1
            rclpy.spin_once(node, timeout_sec=0.05)
        for q in queues.values():
            q.clear()
        deadline = time.monotonic() + args.baseline_seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        for side, q in queues.items():
            finite = [v for v in q if math.isfinite(v)]
            if not finite:
                print("No finite baseline: " + side)
                return 1
            baseline[side] = statistics.median(finite)
            print(
                f"{side} baseline Fy={baseline[side]:.4f} N (host offset only; sensor tare unchanged)"
            )
            q.clear()
        started = time.monotonic()
        next_report = started + args.interval
        print(
            "Each row: raw Fy median / min / max; |Fy-baseline| threshold count; adjacent-sample jumps.",
            flush=True,
        )
        while rclpy.ok() and time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            if now < next_report and now - started < args.seconds:
                continue
            for side, q in queues.items():
                values = list(q)
                q.clear()
                row = describe(
                    values,
                    baseline[side],
                    args.threshold_n,
                    args.jump_n,
                    previous[side],
                )
                previous[side] = row["last"]
                total = totals[side]
                for key, count in [
                    ("samples", row["n"]),
                    ("invalid", row["invalid"]),
                    ("over", row["over"]),
                    ("jumps", row["jumps"]),
                ]:
                    total[key] += count
                total["max_jump"] = max(total["max_jump"], row["max_jump"])
                finite = [abs(v - baseline[side]) for v in values if math.isfinite(v)]
                total["peak_delta"] = max([total["peak_delta"]] + finite)
                if row["median"] is None:
                    print(
                        f'{now-started:6.1f}s {side:5s} NO FINITE DATA n={row["n"]}',
                        flush=True,
                    )
                else:
                    print(
                        f'{now-started:6.1f}s {side:5s} n={row["n"]:4d} median={row["median"]:9.3f} min={row["minimum"]:9.3f} max={row["maximum"]:9.3f} N  over={row["over"]:4d} jumps={row["jumps"]:4d} max_jump={row["max_jump"]:9.3f} invalid={row["invalid"]}',
                        flush=True,
                    )
            next_report = now + args.interval
        print(
            "TOTALS (threshold/jump flags are diagnostic candidates, not proof of noise):"
        )
        for side in totals:
            print(side, totals[side], flush=True)
        print(
            "Independent ROS subscriber: missing deliveries cannot be ruled out; no physical bandwidth/latency claim."
        )
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
