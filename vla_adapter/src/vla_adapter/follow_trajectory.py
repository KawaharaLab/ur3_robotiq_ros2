from __future__ import annotations

import json
from pathlib import Path
import argparse
import csv
from typing import Optional, Sequence, TextIO

import numpy as np
import pyarrow.parquet as pq
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64, Float64MultiArray
from rclpy.action import ActionClient
from robotiq_2f_gripper_msgs.action import MoveTwoFingerGripper


class FollowTrajectoryNode(Node):
	"""Replay a recorded lerobot trajectory directly to the robot."""

	def __init__(self, forward_mode: bool = False, debug_mode: bool = False, debug_csv_path: Optional[Path] = None) -> None:
		super().__init__("follow_trajectory")
		self._forward_mode = forward_mode

		# Parameters
		self._dataset_path = Path(self.declare_parameter("replay_dataset_path", "/home/user/ur3_robotiq_ros2/data/lan_ur3_lerobot_example_forward").value)
		# self._dataset_path = Path(self.declare_parameter("replay_dataset_path", "/home/user/ur3_robotiq_ros2/data/lan_ur3_lerobot_example_old").value)
		stride_param = self.declare_parameter("replay_stride", 4).get_parameter_value().integer_value
		self._stride = max(1, stride_param)
		self._replay_period = float(self.declare_parameter("replay_period", 0.0).value)
		self._action_execution_period = float(self.declare_parameter("action_execution_period", 0.05).value)
		canonical_joint_order = [
			"shoulder_pan_joint",
			"shoulder_lift_joint",
			"elbow_joint",
			"wrist_1_joint",
			"wrist_2_joint",
			"wrist_3_joint",
		]
		self._arm_joint_names = list(
			self.declare_parameter(
				"arm_joint_names",
				canonical_joint_order,
			).get_parameter_value().string_array_value
		)
		self._policy_arm_joint_order = list(
			self.declare_parameter(
				"policy_arm_joint_order",
				canonical_joint_order,
			).get_parameter_value().string_array_value
		)
		self._policy_arm_index = {name: idx for idx, name in enumerate(self._policy_arm_joint_order)}
		default_arm_indices = list(range(len(self._policy_arm_joint_order)))
		self._arm_action_indices = list(
			self.declare_parameter("arm_action_indices", default_arm_indices).get_parameter_value().integer_array_value
		)
		self._gripper_action_indices = list(
			self.declare_parameter("gripper_action_indices", [len(self._policy_arm_joint_order)]).get_parameter_value().integer_array_value
		)
		self._arm_enabled = bool(self.declare_parameter("arm_enabled", True).value)
		self._arm_command_topic = self.declare_parameter(
			"arm_command_topic", "/scaled_joint_trajectory_controller/joint_trajectory"
		).value
		self._forward_command_topic = self.declare_parameter(
			"forward_command_topic", "/forward_position_controller/commands"
		).value if self._forward_mode else None
		self._gripper_command_topic = self.declare_parameter(
			"gripper_command_topic", "/robotiq_gripper/command"
		).value
		self._gripper_action_name = self.declare_parameter(
			"gripper_action_name", "/robotiq_2f_gripper_action"
		).value
		self._gripper_position_scale = float(self.declare_parameter("gripper_position_scale", 0.14).value)
		self._gripper_target_speed = float(self.declare_parameter("gripper_target_speed", 0.5).value)
		self._gripper_target_force = float(self.declare_parameter("gripper_target_force", 0.5).value)
		self._gripper_action_deadband = float(self.declare_parameter("gripper_action_deadband", 0.002).value)

		self._start_time_ns = self.get_clock().now().nanoseconds
		self._debug_mode = bool(self.declare_parameter("debug_mode", debug_mode).value)
		default_debug_csv = str(debug_csv_path or (self._dataset_path / "debug_publish.csv"))
		self._debug_csv_path = Path(self.declare_parameter("debug_csv_path", default_debug_csv).value)
		self._debug_file: TextIO | None = None
		self._debug_writer: csv.DictWriter | None = None

		# Publishers / action client
		self._arm_pub = None if (self._forward_mode or not self._arm_enabled or not self._arm_command_topic) else self.create_publisher(
			JointTrajectory, self._arm_command_topic, 5
		)
		self._forward_pub = None
		if self._forward_mode and self._forward_command_topic:
			self._forward_pub = self.create_publisher(Float64MultiArray, self._forward_command_topic, 5)
		if self._gripper_command_topic:
			msg_type = Float64MultiArray if len(self._gripper_action_indices) > 1 else Float64
			self._gripper_pub = self.create_publisher(msg_type, self._gripper_command_topic, 5)
		else:
			self._gripper_pub = None
		self._gripper_action_client: ActionClient | None = None
		self._last_gripper_target: Optional[float] = None
		if self._gripper_action_name:
			self._gripper_action_client = ActionClient(self, MoveTwoFingerGripper, self._gripper_action_name)

		# Replay buffers
		self._frames: list[dict[str, np.ndarray]] = []
		self._frame_idx = 0
		self._pending_actions: Optional[np.ndarray] = None
		self._pending_absolute_positions: Optional[np.ndarray] = None
		self._pending_action_index = 0
		self._stop_after_pending = False

		# Load dataset
		self._load_dataset()
		self._start_time_ns = self.get_clock().now().nanoseconds
		self._init_debug_logging()

		# Timers
		period = self._replay_period if self._replay_period > 0 else self._computed_period
		self._replay_timer_handle = self.create_timer(period, self._replay_timer)
		self._action_timer_handle = self.create_timer(self._action_execution_period, self._action_timer)

		self.get_logger().info(
			f"Replay ready: frames={len(self._frames)}, period={period:.3f}s, stride={self._stride}, arm_enabled={self._arm_enabled}, dataset={self._dataset_path}"
		)

	# ------------------------------------------------------------------
	# Dataset loading
	# ------------------------------------------------------------------
	def _load_dataset(self) -> None:
		if not self._dataset_path or not self._dataset_path.is_dir():
			raise RuntimeError(f"Replay dataset path not found: {self._dataset_path}")

		info_path = self._dataset_path / "meta" / "info.json"
		fps = 5.0
		if info_path.is_file():
			try:
				with info_path.open(encoding="utf-8") as handle:
					info = json.load(handle)
				fps = float(info.get("fps", fps))
			except Exception as exc:  # pragma: no cover
				self.get_logger().warning(f"Failed to read info.json: {exc}")

		parquet_files = sorted(self._dataset_path.glob("data/**/*.parquet"))
		if not parquet_files:
			raise RuntimeError(f"No parquet files under {self._dataset_path}")

		table = pq.read_table(parquet_files[0])
		states = np.stack(table.column("state").to_pylist()).astype(np.float32)
		actions = np.stack(table.column("actions").to_pylist()).astype(np.float32)
		count = min(len(states), len(actions))
		self._frames = [{"state": states[i], "actions": actions[i]} for i in range(count)]
		self._frame_idx = 0
		self._computed_period = self._stride / fps if fps > 0 else 0.8

	def _init_debug_logging(self) -> None:
		if not self._debug_mode:
			return
		try:
			self._debug_csv_path.parent.mkdir(parents=True, exist_ok=True)
			self._debug_file = self._debug_csv_path.open("w", newline="", encoding="utf-8")
			self._debug_writer = csv.DictWriter(self._debug_file, fieldnames=["stamp_ns", "channel", "data"])
			self._debug_writer.writeheader()
		except Exception as exc:  # pragma: no cover
			self.get_logger().warning(f"Failed to start debug logging: {exc}")
			self._debug_mode = False

	def _debug_step_ns(self) -> int:
		return self.get_clock().now().nanoseconds - self._start_time_ns

	def _log_debug(self, channel: str, payload: dict) -> None:
		if not self._debug_mode or self._debug_writer is None:
			return
		if channel == "arm_joint_trajectory":	
			try:
				self._debug_writer.writerow({"stamp_ns": self._debug_step_ns(), "channel": channel, "data": json.dumps(payload)})
				if self._debug_file is not None:
					self._debug_file.flush()
			except Exception as exc:  # pragma: no cover
				self.get_logger().warning(f"Failed to write debug log: {exc}")
				self._debug_mode = False

	# ------------------------------------------------------------------
	# Replay + publish
	# ------------------------------------------------------------------
	def _replay_timer(self) -> None:
		if not self._frames or self._stop_after_pending:
			return
		if self._frame_idx >= len(self._frames):
			self._stop_after_pending = True
			if self._replay_timer_handle is not None:
				self._replay_timer_handle.cancel()
			return
		frame = self._frames[self._frame_idx]
		next_idx = self._frame_idx + self._stride
		self._frame_idx = min(next_idx, len(self._frames))

		base_positions = frame["state"][: len(self._policy_arm_joint_order)]
		actions = frame["actions"]  # (horizon, dim)
		policy_len = len(self._policy_arm_joint_order)
		if actions.shape[1] < policy_len:
			self.get_logger().warning("Action dimension smaller than arm joints; skipping frame")
			return

		horizon = actions.shape[0]
		delta_positions = actions[:horizon, :policy_len]
		absolute_positions = base_positions + delta_positions

		self._pending_actions = actions[:horizon].copy()
		self._pending_absolute_positions = absolute_positions
		self._pending_action_index = 0

		if next_idx >= len(self._frames):
			self._stop_after_pending = True
			if self._replay_timer_handle is not None:
				self._replay_timer_handle.cancel()

	def _action_timer(self) -> None:
		if self._pending_actions is None:
			return
		pending_len = len(self._pending_actions)
		if self._pending_action_index >= pending_len:
			self._pending_actions = None
			self._pending_absolute_positions = None
			self._pending_action_index = 0
			if self._stop_after_pending:
				self._shutdown_when_idle()
			return

		idx = self._pending_action_index
		action_vec = self._pending_actions[idx]
		abs_positions = (
			None
			if self._pending_absolute_positions is None or idx >= len(self._pending_absolute_positions)
			else self._pending_absolute_positions[idx]
		)
		self._pending_action_index += 1
		if abs_positions is None:
			return
		self._publish_action_vector(action_vec, abs_positions)
		if self._pending_action_index >= pending_len and self._stop_after_pending:
			self._shutdown_when_idle()

	def _publish_action_vector(self, action_vec: np.ndarray, absolute_positions: Sequence[float]) -> None:
		flat = np.asarray(action_vec).ravel()

		csv_joint_order = [
			"shoulder_pan_joint",
			"shoulder_lift_joint",
			"elbow_joint",
			"wrist_1_joint",
			"wrist_2_joint",
			"wrist_3_joint",
		]
		positions: list[float] = []
		for name in csv_joint_order:
			idx = self._policy_arm_index.get(name)
			if idx is None or idx >= len(absolute_positions):
				self.get_logger().warning(
					f"Missing joint {name} in policy positions dim={len(absolute_positions)}"
				)
				return
			positions.append(float(absolute_positions[idx]))

		if self._forward_mode and self._forward_pub is not None:
			msg = Float64MultiArray()
			msg.data = positions
			msg.layout.data_offset = 0
			self._forward_pub.publish(msg)
			# self._log_debug("forward_arm", {"positions": positions})

		elif self._arm_pub is not None:
			arm_cmd = JointTrajectory()
			arm_cmd.header.frame_id = "world"
			arm_cmd.header.stamp.sec = 0
			arm_cmd.header.stamp.nanosec = 0
			arm_cmd.joint_names = csv_joint_order
			point = JointTrajectoryPoint()
			point.positions = positions
			point.velocities = []
			point.accelerations = []
			point.effort = []
			point.time_from_start.sec = 0
			point.time_from_start.nanosec = 50_000_000  # 0.05 s, fixed to CSV first row
			arm_cmd.points.append(point)
			self._arm_pub.publish(arm_cmd)
			self._log_debug("arm_joint_trajectory", positions)

		if self._gripper_action_indices:
			grip_vals = [float(flat[idx]) for idx in self._gripper_action_indices if idx < len(flat)]
			if not grip_vals:
				return
			if not self._send_gripper_goal(grip_vals[0]):
				self._publish_gripper_fallback(grip_vals)

	# ------------------------------------------------------------------
	# Gripper helpers
	# ------------------------------------------------------------------
	def _send_gripper_goal(self, normalized_opening: float) -> bool:
		if self._gripper_action_client is None:
			return False
		if not self._gripper_action_client.wait_for_server(timeout_sec=0.0):
			return False
		target = max(0.0, min(1.0, 1.0 - normalized_opening)) * self._gripper_position_scale
		if self._last_gripper_target is not None and abs(target - self._last_gripper_target) < self._gripper_action_deadband:
			return True
		goal = MoveTwoFingerGripper.Goal()
		goal.target_position = target
		goal.target_speed = self._gripper_target_speed
		goal.target_force = self._gripper_target_force
		self._gripper_action_client.send_goal_async(goal)
		self._last_gripper_target = target
		# self._log_debug(
		# 	"gripper_action_goal",
		# 	{"target_position": target, "target_speed": self._gripper_target_speed, "target_force": self._gripper_target_force},
		# )
		return True

	def _publish_gripper_fallback(self, values: list[float]) -> None:
		if self._gripper_pub is None:
			return
		if len(self._gripper_action_indices) > 1:
			msg_multi = Float64MultiArray()
			msg_multi.data = values
			self._gripper_pub.publish(msg_multi)
			# self._log_debug("gripper_topic", {"values": values, "mode": "multi"})
		else:
			msg_scalar = Float64()
			msg_scalar.data = values[0]
			self._gripper_pub.publish(msg_scalar)
			# self._log_debug("gripper_topic", {"values": values, "mode": "single"})

	def close_debug_log(self) -> None:
		if self._debug_file is not None:
			self._debug_file.close()
		self._debug_file = None
		self._debug_writer = None

	def _shutdown_when_idle(self) -> None:
		if self._action_timer_handle is not None:
			self._action_timer_handle.cancel()
		self.get_logger().info("Replay finished; shutting down after one full pass")
		self.close_debug_log()
		rclpy.shutdown()


def main(args: Optional[Sequence[str]] = None) -> None:
	parser = argparse.ArgumentParser(add_help=False)
	parser.add_argument("--forward", action="store_true")
	parser.add_argument("--debug", action="store_true")
	parser.add_argument("--debug-csv", type=str, default="")
	parsed, remaining = parser.parse_known_args(args)
	rclpy.init(args=remaining)
	try:
		node = FollowTrajectoryNode(
			forward_mode=parsed.forward,
			debug_mode=parsed.debug,
			debug_csv_path=Path(parsed.debug_csv) if parsed.debug_csv else None,
		)
	except Exception as exc:  # pragma: no cover
		print(f"Failed to start FollowTrajectoryNode: {exc}")
		rclpy.shutdown()
		return
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.close_debug_log()
		node.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()


if __name__ == "__main__":
	main()
