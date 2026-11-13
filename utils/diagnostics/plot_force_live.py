#!/usr/bin/env python3
"""Short read-only ROS force capture -> waveform PNG and small CSV; no rosbag/motion."""
import argparse
import csv
import json
import math
from pathlib import Path
import time


def draw(rows, output, title, normal_range_n, start_ros_ns):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    fig, axes = plt.subplots(3, 2, figsize=(16, 10), squeeze=False)
    report = {}
    try:
        for col, side in enumerate(("left", "right")):
            values = rows[side]
            if not values:
                for ax in axes[:, col]:
                    ax.text(0.5, 0.5, "NO DATA", ha="center", transform=ax.transAxes)
                report[side] = dict(samples=0)
                continue
            times = np.array([(r[0] - start_ros_ns) / 1e9 for r in values])
            raw = np.array([r[2] for r in values])
            finite = np.isfinite(raw)
            first = raw[(times <= times[0] + 2.0) & finite]
            baseline = float(np.median(first)) if len(first) else None
            if baseline is None:
                for ax in axes[:, col]:
                    ax.text(
                        0.5,
                        0.5,
                        "NO FINITE BASELINE",
                        ha="center",
                        transform=ax.transAxes,
                    )
                report[side] = dict(samples=len(values), invalid=int((~finite).sum()))
                continue
            delta = raw - baseline
            # Insert display breaks; preserve every received row in the CSV.
            x, y = [], []
            for n, (t, v) in enumerate(zip(times, delta)):
                if n and (t - times[n - 1] > 0.005 or t <= times[n - 1]):
                    x.append(float("nan"))
                    y.append(float("nan"))
                x.append(t)
                y.append(v)
            color = "tab:blue" if side == "left" else "tab:orange"
            for ax in axes[:, col]:
                ax.plot(x, y, color=color, linewidth=0.65, marker=".", markersize=1)
                ax.grid(alpha=0.25)
                ax.set_ylabel("Fy - initial median [N]")
                ax.set_xlabel("Source Header time from monitor start [s]")
            axes[0, col].set_title(side + " | full amplitude range")
            axes[1, col].set_title(
                side
                + f" | fixed display range +/-{normal_range_n:g} N (outside values still in CSV/top panel)"
            )
            axes[1, col].set_ylim(-normal_range_n, normal_range_n)
            # A jump is a diagnostic candidate, not a noise classification.
            changes = np.abs(np.diff(raw))
            changes[~np.isfinite(changes)] = -1
            k = int(np.argmax(changes)) + 1 if len(changes) and max(changes) >= 0 else 0
            center = times[k]
            axes[2, col].set_xlim(center - 0.10, center + 0.10)
            local = delta[(times >= center - 0.10) & (times <= center + 0.10) & finite]
            if len(local):
                lo, hi = float(local.min()), float(local.max())
                margin = max((hi - lo) * 0.1, 0.2)
                axes[2, col].set_ylim(lo - margin, hi + margin)
            axes[2, col].set_title(
                side + f" | largest adjacent-sample change, t={center:.3f}s, +/-100 ms"
            )
            gaps = np.diff(times)
            report[side] = dict(
                samples=len(values),
                invalid=int((~finite).sum()),
                baseline_Fy_n=baseline,
                raw_min_n=float(raw[finite].min()),
                raw_max_n=float(raw[finite].max()),
                header_gaps_over_5ms=int((gaps > 0.005).sum()),
                header_nonincreasing=int((gaps <= 0).sum()),
                largest_step_n=float(changes[k - 1]) if k else None,
                largest_step_at_s=float(center),
            )
        fig.suptitle(
            title
            + "\nUnfiltered received Fy; initial 2 s median subtracted for display only. Blank lines: Header gaps >5 ms.",
            fontsize=12,
        )
        fig.tight_layout(rect=(0, 0, 1, 0.94))
        fig.savefig(output, dpi=140)
    finally:
        plt.close(fig)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--title", default="Force wiring diagnostic")
    parser.add_argument("--normal-range-n", type=float, default=20.0)
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or not 2 < args.seconds <= 300:
        parser.error("--seconds must be >2 and <=300")
    if not math.isfinite(args.normal_range_n) or args.normal_range_n <= 0:
        parser.error("--normal-range-n must be positive and finite")
    if args.out.exists():
        parser.error("--out exists; choose a new condition name")
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import WrenchStamped

    # Check plotting dependencies before accessing the topics.
    import matplotlib
    import numpy

    rclpy.init()
    node = None
    rows = {side: [] for side in ("left", "right")}
    metadata = dict(
        title=args.title,
        requested_seconds=args.seconds,
        no_robot_commands=True,
        filtered=False,
        capture_incomplete=False,
        note="Independent subscriber; no guarantee against ROS delivery loss. CSV preserves Header and host receive times.",
    )
    try:
        node = Node("force_waveform_readonly_monitor")
        qos = QoSProfile(depth=10000, reliability=ReliabilityPolicy.BEST_EFFORT)

        def receive(side, message):
            rows[side].append(
                (
                    message.header.stamp.sec * 10**9 + message.header.stamp.nanosec,
                    time.monotonic_ns(),
                    message.wrench.force.y,
                )
            )

        subscriptions = [
            node.create_subscription(
                WrenchStamped,
                "/force_torque/" + side,
                lambda msg, side=side: receive(side, msg),
                qos,
            )
            for side in rows
        ]
        print(
            "Waiting for both force topics (10 s timeout). No robot commands.",
            flush=True,
        )
        deadline = time.monotonic() + 10.0
        while not all(rows.values()):
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "Both force topics are required; start the force driver"
                )
            rclpy.spin_once(node, timeout_sec=0.02)
        rows = {side: [] for side in rows}
        args.out.mkdir(parents=True)
        metadata["start_ros_ns"] = node.get_clock().now().nanoseconds
        started = time.monotonic()
        metadata["start_receive_monotonic_ns"] = time.monotonic_ns()
        print(
            f"Capturing {args.seconds:g} s. Keep unloaded/still for the first 2 s, then perform your diagnostic. PNG generated at end.",
            flush=True,
        )
        try:
            while rclpy.ok() and time.monotonic() - started < args.seconds:
                rclpy.spin_once(node, timeout_sec=0.02)
        except KeyboardInterrupt:
            metadata["capture_incomplete"] = True
        metadata["actual_seconds"] = time.monotonic() - started
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    with (args.out / "force.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["side", "source_header_ns", "receive_monotonic_ns", "Fy_n"])
        for side, values in rows.items():
            for row in values:
                writer.writerow((side,) + row)
    metadata["statistics"] = draw(
        rows,
        args.out / "waveform.png",
        args.title,
        args.normal_range_n,
        metadata["start_ros_ns"],
    )
    (args.out / "summary.json").write_text(
        json.dumps(metadata, indent=2, allow_nan=False)
    )
    print("PNG: " + str((args.out / "waveform.png").resolve()), flush=True)
    return 0 if all(rows.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
