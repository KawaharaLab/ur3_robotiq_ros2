from __future__ import annotations

import ast
import csv
from pathlib import Path
from typing import Optional, Sequence
import argparse

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray


class SimpleTrajectoryReplay(Node):
    """Replay joint positions from a single CSV file (no state alignment)."""

    def __init__(self, forward_mode: bool = False) -> None:
        super().__init__("follow_trajectory_simple")
        self._forward_mode = forward_mode

        # Parameters
        self._csv_path = Path(self.declare_parameter("replay_csv_path", "/home/user/ur3_robotiq_ros2/data/lan_example/20260110_183809/csv/scaled_joint_trajectory_controller_joint_trajectory.csv").value)
        stride_param = self.declare_parameter("replay_stride", 1).get_parameter_value().integer_value
        self._stride = max(1, stride_param)
        self._replay_period = float(self.declare_parameter("replay_period", 0.05).value)  # default 20 Hz
        self._arm_enabled = bool(self.declare_parameter("arm_enabled", True).value)
        self._arm_command_topic = self.declare_parameter(
            "arm_command_topic", "/scaled_joint_trajectory_controller/joint_trajectory"
        ).value
        self._forward_command_topic = self.declare_parameter(
            "forward_command_topic", "/forward_position_controller/commands"
        ).value if self._forward_mode else None
        self._arm_joint_names = list(
            self.declare_parameter(
                "arm_joint_names",
                [
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_1_joint",
                    "wrist_2_joint",
                    "wrist_3_joint",
                ],
            ).get_parameter_value().string_array_value
        )
        self._arm_action_indices = list(
            self.declare_parameter("arm_action_indices", [0, 1, 2, 3, 4, 5]).get_parameter_value().integer_array_value
        )

        # Hardcoded constants from the first CSV row
        self._frame_id = "world"
        self._time_from_start_sec = 0
        self._time_from_start_nsec = 50_000_000  # 0.05s

        self._arm_pub = None if (self._forward_mode or not self._arm_enabled or not self._arm_command_topic) else self.create_publisher(
            JointTrajectory, self._arm_command_topic, 5
        )
        self._forward_pub = None
        if self._forward_mode and self._forward_command_topic:
            self._forward_pub = self.create_publisher(Float64MultiArray, self._forward_command_topic, 5)

        # Data buffers
        self._positions: list[list[float]] = []
        self._frame_idx = 0

        # Load CSV
        self._load_csv()

        period = self._replay_period if self._replay_period > 0.0 else 0.05
        self._timer = self.create_timer(period, self._timer_cb)

        self.get_logger().info(
            f"Simple replay ready: frames={len(self._positions)}, period={period:.3f}s, stride={self._stride}, csv={self._csv_path}"
        )

    # ------------------------------------------------------------------
    # Dataset loading
    # ------------------------------------------------------------------
    def _load_csv(self) -> None:
        if not self._csv_path or not self._csv_path.is_file():
            raise RuntimeError(f"Replay CSV path not found: {self._csv_path}")

        with self._csv_path.open(newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            # Identify the positions field
            pos_field = None
            preferred_fields = [
                "forward_position_controller_commands.data",
                "trajectory_point_0_positions",
            ] if self._forward_mode else [
                "trajectory_point_0_positions",
            ]

            for field in reader.fieldnames or []:
                if field in preferred_fields:
                    pos_field = field
                    break
                if not self._forward_mode and "points[0].positions" in field:
                    pos_field = field
                if pos_field is None and ("position" in field or "positions" in field):
                    pos_field = field

            if pos_field is None:
                raise RuntimeError("CSV must contain a positions column (e.g., forward_position_controller_commands.data or scaled_joint_trajectory_controller_joint_trajectory.points[0].positions)")

            for row in reader:
                raw = row.get(pos_field, "")
                if not raw:
                    continue
                try:
                    arr = ast.literal_eval(raw)
                    positions = [float(x) for x in arr]
                    self._positions.append(positions)
                except Exception as exc:  # pragma: no cover
                    self.get_logger().warning(f"Failed to parse positions '{raw}': {exc}")
        if not self._positions:
            raise RuntimeError("No positions parsed from CSV")

    # ------------------------------------------------------------------
    # Timer callback
    # ------------------------------------------------------------------
    def _timer_cb(self) -> None:
        if not self._positions:
            return
        frame_state = self._positions[self._frame_idx]
        self._frame_idx = (self._frame_idx + self._stride) % len(self._positions)

        self._publish_arm(frame_state)

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------
    def _publish_arm(self, frame_state: Sequence[float]) -> None:
        joint_names: list[str] = []
        positions: list[float] = []
        for idx in self._arm_action_indices:
            if idx >= len(self._arm_joint_names) or idx >= len(frame_state):
                continue
            joint_names.append(self._arm_joint_names[idx])
            positions.append(float(frame_state[idx]))
        if not joint_names:
            return

        if self._forward_mode and self._forward_pub is not None:
            msg = Float64MultiArray()
            msg.data = positions
            msg.layout.data_offset = 0
            self._forward_pub.publish(msg)
            return

        if self._arm_pub is None:
            return
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = self._time_from_start_sec
        point.time_from_start.nanosec = self._time_from_start_nsec
        msg = JointTrajectory()
        msg.header.frame_id = self._frame_id
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.joint_names = joint_names
        msg.points.append(point)
        self._arm_pub.publish(msg)

def main(args: Optional[Sequence[str]] = None) -> None:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--forward", action="store_true")
    parsed, remaining = parser.parse_known_args(args)
    rclpy.init(args=remaining)
    try:
        node = SimpleTrajectoryReplay(forward_mode=parsed.forward)
    except Exception as exc:  # pragma: no cover
        print(f"Failed to start SimpleTrajectoryReplay: {exc}")
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
