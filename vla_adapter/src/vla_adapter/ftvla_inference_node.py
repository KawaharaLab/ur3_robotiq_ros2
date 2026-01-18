from __future__ import annotations

import csv
import sys
import threading
import time
from collections import deque
import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Protocol, Sequence

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, JointState, CompressedImage
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32, Float64, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from robotiq_2f_gripper_msgs.action import MoveTwoFingerGripper

# Ensure the bundled OpenPI sources are importable without an editable install.
REPO_ROOT = Path(__file__).resolve().parents[3]
print(REPO_ROOT)
OPENPI_PATHS = [
	REPO_ROOT / "external" / "openpi" / "src",
	REPO_ROOT / "external" / "openpi" / "packages" / "openpi-client" / "src",
]
for candidate in OPENPI_PATHS:
	if candidate.is_dir():
		sys.path.append(str(candidate))

from openpi.policies import policy_config as _policy_config
from openpi.shared import download as _download
from openpi.training import config as _config


class PolicyHandle(Protocol):
	"""Minimal interface shared by local and remote OpenPI policies."""

	def infer(self, obs: dict) -> dict:  # noqa: D401 - protocol signature only.
		...


@dataclass(slots=True, frozen=True)
class ImageSlot:
	"""Mapping between ROS camera streams and OpenPI camera keys."""

	subscription_topic: str
	policy_key: str
	is_compressed: bool = False


class FTVLAInferenceNode(Node):
	"""Bridge ROS topics into Pi0 policy calls and publish the resulting commands."""

	def __init__(self, forward_mode: bool = False) -> None:
		super().__init__("ftvla_inference_node")
		self._bridge = CvBridge()
		self._lock = threading.Lock()
		self._forward_mode = forward_mode

		# QoS policies tuned per stream type.
		qos_camera = QoSProfile(depth=5)
		qos_camera.history = QoSHistoryPolicy.KEEP_LAST
		qos_camera.reliability = QoSReliabilityPolicy.BEST_EFFORT
		qos_state = QoSProfile(depth=10)
		qos_state.history = QoSHistoryPolicy.KEEP_LAST
		qos_state.reliability = QoSReliabilityPolicy.RELIABLE
		qos_scalar = QoSProfile(depth=5)
		qos_scalar.history = QoSHistoryPolicy.KEEP_LAST
		qos_scalar.reliability = QoSReliabilityPolicy.BEST_EFFORT
		qos_arm_cmd = QoSProfile(depth=5)
		qos_arm_cmd.history = QoSHistoryPolicy.KEEP_LAST
		qos_arm_cmd.reliability = QoSReliabilityPolicy.BEST_EFFORT

		# Topic + policy wiring parameters.
		self._image_encoding = self.declare_parameter("image_encoding", "rgb8").value
		self._channels_last = self.declare_parameter("channels_last", True).value

		wrist_topic = self.declare_parameter(
			"wrist_camera_topic", "/camera_wrist/realsense2_camera/color/image_raw"
		).value
		wrist_use_compressed = bool(self.declare_parameter("wrist_use_compressed", False).value)
		fixed_topic = self.declare_parameter(
			"fixed_camera_topic", "/camera_fixed/realsense2_camera/color/image_raw"
		).value
		fixed_use_compressed = bool(self.declare_parameter("fixed_use_compressed", False).value)
		self._image_slots = {
			"wrist": ImageSlot(
				subscription_topic=wrist_topic,
				policy_key=self.declare_parameter("policy_wrist_image_key", "cam_left_wrist").value,
				is_compressed=wrist_use_compressed,
			),
			"fixed": ImageSlot(
				subscription_topic=fixed_topic,
				policy_key=self.declare_parameter("policy_fixed_image_key", "cam_high").value,
				is_compressed=fixed_use_compressed,
			),
		}

		# Fixed crop windows (width,height,x_offset,y_offset) tuned for 640x360 inputs.
		self._image_crops = {
			"wrist": (360, 360, 230, 0),
			"fixed": (360, 360, 170, 0),
		}

		self._arm_joint_topic = self.declare_parameter("arm_joint_topic", "/joint_states").value
		self._gripper_joint_topic = self.declare_parameter("gripper_joint_topic", "").value
		self._gripper_distance_topic = self.declare_parameter(
			"gripper_distance_topic", "/robotiq_2f_gripper/finger_distance_mm"
		).value

		self._arm_joint_names = self._get_str_list_param(
			"arm_joint_names",
			default=[
				"shoulder_pan_joint",
				"shoulder_lift_joint",
				"elbow_joint",
				"wrist_1_joint",
				"wrist_2_joint",
				"wrist_3_joint",
			],
		)
		canonical_policy_joint_order = [
			"shoulder_pan_joint",
			"shoulder_lift_joint",
			"elbow_joint",
			"wrist_1_joint",
			"wrist_2_joint",
			"wrist_3_joint",
		]
		self._policy_arm_joint_order = self._get_str_list_param(
			"policy_arm_joint_order", default=canonical_policy_joint_order
		)
		self._policy_arm_index = {name: idx for idx, name in enumerate(self._policy_arm_joint_order)}
		self._gripper_joint_names = self._get_str_list_param(
			"gripper_joint_names", default=["robotiq_finger_distance"]
		)
		self._gripper_distance_joint_name = self._gripper_joint_names[0] if self._gripper_joint_names else None
		self._gripper_joint_set = set(self._gripper_joint_names)
		self._gripper_distance_scale = float(self.declare_parameter("gripper_distance_scale", 140.0).value) # input
		self._gripper_position_scale = float(self.declare_parameter("gripper_position_scale", 0.14).value) # output
		self._gripper_target_speed = float(self.declare_parameter("gripper_target_speed", 0.5).value)
		self._gripper_target_force = float(self.declare_parameter("gripper_target_force", 0.5).value)
		self._gripper_action_deadband = float(self.declare_parameter("gripper_action_deadband", 0.002).value)
		self._debug_enabled = bool(self.declare_parameter("debug", False).value)
		self._debug_dataset_dir = Path(
			self.declare_parameter(
				"debug_dataset_dir",
				str(REPO_ROOT / "data" / "example"),
			).value
		)
		self._debug_csv_filename = self.declare_parameter("debug_csv_filename", "example.csv").value
		self._debug_single_shot_done = False
		self._debug_shutdown_requested = False
		self._debug_joint_positions: Dict[str, float] = {}

		self._io_log_path = Path(
			self.declare_parameter(
				"io_log_path", str(REPO_ROOT / "data" / "inference_io_log.csv")
			).value
		)
		self._input_log_dir = Path(
			self.declare_parameter(
				"input_log_dir", str(REPO_ROOT / "data" / "inference_inputs")
			).value
		)
		self._input_log_dir.mkdir(parents=True, exist_ok=True)
		self._input_index_path = self._input_log_dir / "inputs.csv"
		self._inference_sequence = 0

		state_order = self._get_str_list_param("state_joint_order", default=[])
		if state_order:
			self._state_joint_order = state_order
		else:
			self._state_joint_order = [*self._policy_arm_joint_order, *self._gripper_joint_names]

		self._max_data_age = Duration(seconds=self.declare_parameter("max_data_age", 0.5).value)
		self._action_execution_period = float(self.declare_parameter("action_execution_period", 0.05).value)
		actions_per_inference_param = self.declare_parameter("actions_per_inference", 16)
		self._actions_per_inference = max(1, actions_per_inference_param.get_parameter_value().integer_value)
		default_inference_period = self._actions_per_inference * self._action_execution_period
		self._inference_period = float(
			self.declare_parameter("inference_period", default_inference_period).value
		)
		self._prompt = self.declare_parameter("default_prompt", "pick the cable connector and insert it into the white socket until it snaps. Then, push the red button.").value
		# self._prompt = self.declare_parameter("default_prompt", "Pick up the blue object and place it in the orange box.").value

		self._publish_horizon_index = self.declare_parameter("publish_horizon_index", 0).get_parameter_value().integer_value
		default_arm_action_indices: list[int] = []
		for joint in self._arm_joint_names:
			idx = self._policy_arm_index.get(joint)
			if idx is None:
				self.get_logger().warning(f"Joint {joint} missing from policy order")
				continue
			default_arm_action_indices.append(idx)
		if not default_arm_action_indices:
			default_arm_action_indices = list(range(len(self._policy_arm_joint_order)))
		self._arm_action_indices = self._get_int_list_param(
			"arm_action_indices", default=default_arm_action_indices
		)
		default_gripper_indices = [len(self._arm_joint_names)] if self._gripper_joint_names else []
		self._gripper_action_indices = self._get_int_list_param(
			"gripper_action_indices", default=default_gripper_indices
		)
		self._gripper_uses_multiarray = len(self._gripper_action_indices) > 1

		self._ft_horizon = int(self.declare_parameter("ft_horizon", 200).value)
		self._ft_topics = {
			"left": self.declare_parameter("left_ft_topic", "/force_torque/left").value,
			"right": self.declare_parameter("right_ft_topic", "/force_torque/right").value,
		}
		self._ft_buffers = {side: deque(maxlen=self._ft_horizon) for side in ("left", "right")}
		self._ft_last_stamp: Dict[str, Optional[Time]] = {side: None for side in ("left", "right")}

		self._arm_command_topic = self.declare_parameter(
			"arm_command_topic", "/scaled_joint_trajectory_controller/joint_trajectory"
		).value
		self._forward_command_topic = self.declare_parameter(
			"forward_command_topic", "/forward_position_controller/commands_vla"
		).value if self._forward_mode else None
		self._gripper_command_topic = self.declare_parameter(
			"gripper_command_topic", "/robotiq_gripper/command"
		).value
		self._gripper_mux_topic = self.declare_parameter(
			"gripper_mux_topic", "/robotiq_gripper/command_vla"
		).value
		self._gripper_mux_enabled = bool(self.declare_parameter("gripper_mux_enabled", True).value)
		self._gripper_action_name = self.declare_parameter(
			"gripper_action_name", "/robotiq_2f_gripper_action"
		).value

		self._arm_pub = None if (self._forward_mode or not self._arm_command_topic) else self.create_publisher(
			JointTrajectory, self._arm_command_topic, qos_arm_cmd
		)
		self._forward_pub = None
		if self._forward_mode and self._forward_command_topic:
			self._forward_pub = self.create_publisher(Float64MultiArray, self._forward_command_topic, qos_arm_cmd)
		self._gripper_mux_pub = None
		if self._gripper_command_topic:
			gripper_type = Float64MultiArray if self._gripper_uses_multiarray else Float64
			self._gripper_pub = self.create_publisher(gripper_type, self._gripper_command_topic, 10)
		else:
			self._gripper_pub = None
		if self._gripper_mux_enabled and self._gripper_mux_topic:
			self._gripper_mux_pub = self.create_publisher(Float64, self._gripper_mux_topic, 10)

		self._gripper_action_client: ActionClient | None = None
		self._last_gripper_target: float | None = None
		if self._gripper_action_name:
			self._gripper_action_client = ActionClient(self, MoveTwoFingerGripper, self._gripper_action_name)

		# Latest data caches guarded by _lock.
		self._latest_images: Dict[str, tuple[Time, np.ndarray]] = {}
		self._latest_joint_positions: Dict[str, float] = {}
		self._last_joint_stamp: Optional[Time] = None
		self._pending_actions: Optional[np.ndarray] = None
		self._pending_action_index = 0
		self._pending_absolute_positions: Optional[np.ndarray] = None
		self._debug_dataset_row: dict[str, str] | None = None

		if self._debug_enabled:
			try:
				self._initialize_debug_dataset()
				self.get_logger().info(f"Debug dataset preloaded from {self._debug_dataset_dir}")
			except Exception as exc:
				self.get_logger().error(f"Failed to prepare debug dataset: {exc}")
				self._debug_enabled = False

		# Subscriptions.
		if self._image_slots["wrist"].subscription_topic:
			msg_type = CompressedImage if self._image_slots["wrist"].is_compressed else Image
			self.create_subscription(
				msg_type,
				self._image_slots["wrist"].subscription_topic,
				lambda msg: self._image_callback("wrist", msg),
				qos_camera,
			)
		if self._image_slots["fixed"].subscription_topic:
			msg_type = CompressedImage if self._image_slots["fixed"].is_compressed else Image
			self.create_subscription(
				msg_type,
				self._image_slots["fixed"].subscription_topic,
				lambda msg: self._image_callback("fixed", msg),
				qos_camera,
			)
		if self._arm_joint_topic:
			self.create_subscription(
				JointState,
				self._arm_joint_topic,
				self._joint_state_callback,
				qos_state,
			)
		if self._gripper_joint_topic and self._gripper_joint_topic != self._arm_joint_topic:
			self.create_subscription(
				JointState,
				self._gripper_joint_topic,
				self._joint_state_callback,
				qos_state,
			)
		if self._gripper_distance_topic and self._gripper_distance_joint_name:
			self.create_subscription(
				Float32,
				self._gripper_distance_topic,
				self._gripper_distance_callback,
				qos_scalar,
			)

		for side, topic in self._ft_topics.items():
			if topic:
				self.create_subscription(
					WrenchStamped,
					topic,
					lambda msg, side=side: self._ft_callback(side, msg),
					qos_scalar,
				)

		# Load the OpenPI policy directly on this machine (GPU-friendly).
		self._policy = self._create_local_policy()

		# Periodic inference loop.
		self._inference_timer_handle = self.create_timer(self._inference_period, self._inference_timer)
		self._action_timer_handle = None
		if not self._debug_enabled:
			self._action_timer_handle = self.create_timer(self._action_execution_period, self._action_timer)

		self.get_logger().info(
			f"FT-VLA inference node ready (policy={self._policy_descriptor}, arm_topic={self._arm_joint_topic},"
			f" wrist_cam={self._image_slots['wrist'].subscription_topic},"
			f" fixed_cam={self._image_slots['fixed'].subscription_topic})"
		)

	# ---------------------------------------------------------------------
	# Parameter helpers
	# ---------------------------------------------------------------------
	def _get_str_list_param(self, name: str, *, default: Sequence[str]) -> list[str]:
		value = self.declare_parameter(name, default).get_parameter_value()
		if value.string_array_value:
			return list(value.string_array_value)
		if value.string_value:
			return [value.string_value]
		return list(default)

	def _get_int_list_param(self, name: str, *, default: Sequence[int]) -> list[int]:
		value = self.declare_parameter(name, default).get_parameter_value()
		if value.integer_array_value:
			return [int(v) for v in value.integer_array_value]
		if value.integer_value:
			return [int(value.integer_value)]
		return [int(v) for v in default]

	# ---------------------------------------------------------------------
	# Debug helpers
	# ---------------------------------------------------------------------
	def _initialize_debug_dataset(self) -> None:
		csv_path = self._debug_dataset_dir / self._debug_csv_filename
		if not csv_path.is_file():
			raise FileNotFoundError(f"Debug CSV not found: {csv_path}")
		with csv_path.open(newline="", encoding="utf-8") as handle:
			reader = csv.DictReader(handle)
			try:
				row = next(reader)
			except StopIteration as exc:
				raise RuntimeError(f"Debug CSV {csv_path} has no rows") from exc

		self._debug_dataset_row = row
		self._load_debug_state_from_row(row)
		self._load_debug_images_from_row(row)
		self._last_joint_stamp = self.get_clock().now()

	def _load_debug_state_from_row(self, row: dict[str, str]) -> None:
		missing = []
		state: Dict[str, float] = {}
		for joint in self._state_joint_order:
			value_raw = row.get(joint)
			if value_raw is None:
				missing.append(joint)
				continue
			value = value_raw.strip()
			if not value:
				missing.append(joint)
				continue
			try:
				state[joint] = float(value)
			except ValueError as exc:
				raise ValueError(f"Invalid value for joint {joint} in debug CSV: {value}") from exc
		if missing:
			raise ValueError(
				"Debug CSV missing joint columns: " + ",".join(missing),
			)
		self._latest_joint_positions.update(state)
		self._debug_joint_positions = state

	def _load_debug_images_from_row(self, row: dict[str, str]) -> None:
		now = self.get_clock().now()
		for slot_name in self._image_slots.keys():
			column = f"{slot_name}_image"
			image_rel_raw = row.get(column)
			if not image_rel_raw:
				continue
			image_rel = image_rel_raw.strip()
			if not image_rel:
				continue
			image_path = self._debug_dataset_dir / image_rel
			if not image_path.is_file():
				raise FileNotFoundError(f"Debug image missing: {image_path}")
			image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
			if image is None:
				raise RuntimeError(f"Failed to load debug image: {image_path}")
			image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
			if not self._channels_last:
				image = np.transpose(image, (2, 0, 1))
			self._latest_images[slot_name] = (now, np.ascontiguousarray(image))

	def _log_debug_action(self, absolute_positions: Sequence[float], flat_vec: np.ndarray) -> None:
		arm_targets = {
			self._policy_arm_joint_order[idx]: round(absolute_positions[idx], 4)
			for idx in self._arm_action_indices
			if idx < len(self._policy_arm_joint_order)
		}
		self.get_logger().info(f"Debug arm command -> {arm_targets}")
		gripper_values = [float(flat_vec[idx]) for idx in self._gripper_action_indices if idx < len(flat_vec)]
		if gripper_values:
			self.get_logger().info(f"Debug gripper delta -> {gripper_values}")

	def _shutdown_debug_mode(self) -> None:
		if self._debug_shutdown_requested:
			return
		self._debug_shutdown_requested = True
		self.get_logger().info("Debug mode complete. Shutting down rclpy ...")
		if rclpy.ok():
			rclpy.shutdown()

	def _run_debug_action_sequence(self, actions: np.ndarray) -> None:
		sequence = self._extract_action_sequence(actions)
		if sequence.size == 0:
			self.get_logger().warning("Debug mode: no actions to execute")
			self._debug_single_shot_done = True
			self._shutdown_debug_mode()
			return
		max_len = min(self._actions_per_inference, sequence.shape[0])
		base_policy_positions: list[Optional[float]] = [
			self._latest_joint_positions.get(name) for name in self._policy_arm_joint_order
		]
		for step_idx in range(max_len):
			action_vec = sequence[step_idx]
			if any(pos is None for pos in base_policy_positions):
				continue
			flat_vec = np.asarray(action_vec).ravel()
			policy_len = len(base_policy_positions)
			abs_positions = [
				float(base_policy_positions[j_idx]) + float(flat_vec[j_idx])
				for j_idx in range(min(policy_len, len(flat_vec)))
			]
			self._publish_action_vector(action_vec, abs_positions)
		self._debug_single_shot_done = True
		self.get_logger().info(f"Debug inference emitted {max_len} action vectors")
		self._shutdown_debug_mode()

	# ---------------------------------------------------------------------
	# Policy wiring
	# ---------------------------------------------------------------------
	def _create_local_policy(self) -> PolicyHandle:
		config_name = self.declare_parameter("policy_config_name", "pi0_ur3_robotiq_ft").value
		checkpoint_dir = self.declare_parameter(
			"policy_checkpoint_dir", "/home/user/openpi/checkpoints/pi0_ur3_robotiq_ft/30000"
		).value
		pytorch_device_param = self.declare_parameter("policy_pytorch_device", "auto").value
		default_prompt_override = self.declare_parameter("policy_default_prompt", self._prompt).value
		# sample_steps = self.declare_parameter("policy_sample_steps", 10).get_parameter_value().integer_value
		# sample_kwargs = {"num_steps": sample_steps} if sample_steps > 0 else {}

		train_config = _config.get_config(config_name)
		pytorch_device = None if pytorch_device_param in ("", "auto") else pytorch_device_param
		policy = _policy_config.create_trained_policy(
			train_config,
			checkpoint_dir,
			# sample_kwargs=sample_kwargs,
			# default_prompt=default_prompt_override,
			# pytorch_device=pytorch_device,
		)
		self._policy_descriptor = f"{config_name} @ {checkpoint_dir}"
		try:
			metadata = policy.metadata
			self.get_logger().info(f"Loaded OpenPI policy metadata: {metadata}")
		except Exception:
			print("Failed to retrieve policy metadata with exception:", sys.exc_info()[1])
			pass
		return policy

	# ---------------------------------------------------------------------
	# Subscriptions
	# ---------------------------------------------------------------------
	def _image_callback(self, slot: str, msg: Image) -> None:
		if self._debug_enabled:
			return
		try:
			if isinstance(msg, CompressedImage):
				cv_image = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding=self._image_encoding)
			else:
				cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding=self._image_encoding)
		except CvBridgeError as exc:
			self.get_logger().warning(f"Failed to convert {slot} image: {exc}")
			return

		cv_image = self._crop_image(slot, cv_image)

		if not self._channels_last:
			cv_image = np.transpose(cv_image, (2, 0, 1))

		stamp = Time.from_msg(msg.header.stamp) if msg.header.stamp else self.get_clock().now()

		with self._lock:
			self._latest_images[slot] = (stamp, np.ascontiguousarray(cv_image))

	def _joint_state_callback(self, msg: JointState) -> None:
		if self._debug_enabled:
			return
		positions = dict(zip(msg.name, msg.position))
		stamp = Time.from_msg(msg.header.stamp) if msg.header.stamp else self.get_clock().now()
		with self._lock:
			self._latest_joint_positions.update(positions)
			self._last_joint_stamp = stamp

	def _gripper_distance_callback(self, msg: Float32) -> None:
		"""Handle Float32 gripper distance topics (e.g. finger_distance_mm)."""
		if not self._gripper_distance_joint_name:
			return
		if self._debug_enabled:
			return
		stamp = self.get_clock().now()
		with self._lock:
			self._latest_joint_positions[self._gripper_distance_joint_name] = float(msg.data)
			self._last_joint_stamp = stamp

	def _ft_callback(self, side: str, msg: WrenchStamped) -> None:
		if self._debug_enabled:
			return
		wrench = msg.wrench
		vec = np.array(
			[
				float(wrench.force.x),
				float(wrench.force.y),
				float(wrench.force.z),
				float(wrench.torque.x),
				float(wrench.torque.y),
				float(wrench.torque.z),
			],
			dtype=np.float32,
		)
		stamp = Time.from_msg(msg.header.stamp) if msg.header.stamp else self.get_clock().now()
		with self._lock:
			self._ft_buffers[side].append(vec)
			self._ft_last_stamp[side] = stamp

	# ---------------------------------------------------------------------
	# Inference loop
	# ---------------------------------------------------------------------
	def _inference_timer(self) -> None:
		if self._debug_enabled and self._debug_single_shot_done:
			return
		with self._lock:
			obs, arm_snapshot = self._build_observation_locked()
		if obs is None or arm_snapshot is None:
			print("No observation available for inference")
			return
		snapshot_stamp = self.get_clock().now()
		self._record_inference_input(obs, snapshot_stamp)
		print("obs: ", obs["state"])
		try:
			start_time = time.perf_counter()
			result = self._policy.infer(obs)
			duration_ms = (time.perf_counter() - start_time) * 1000.0
			print(f"Inference latency: {duration_ms:.1f} ms")
		except Exception as exc:  # pragma: no cover - depends on server
			self.get_logger().error(f"Policy inference failed: {exc}")
			return

		actions = result.get("actions")
		if actions is None:
			self.get_logger().warning(f"Policy response missing 'actions' key: {list(result.keys())}")
			return
		action_array = np.asarray(actions)
		print(action_array[0])
		snapshot_action = self._extract_log_action(action_array)
		self._log_io(obs.get("state"), snapshot_action)
		if self._debug_enabled:
			self._run_debug_action_sequence(action_array)
			return
		queued = self._queue_actions(action_array, arm_snapshot)

	def _build_observation_locked(self) -> tuple[Optional[dict], Optional[list[float]]]:
		now = self.get_clock().now()

		wrist = self._latest_images.get("wrist")
		fixed = self._latest_images.get("fixed")

		# if not self._is_fresh(wrist, now) or not self._is_fresh(fixed, now):
		# 	print("Images are not fresh enough for inference")
		# 	return None

		# if self._last_joint_stamp is None or now - self._last_joint_stamp > self._max_data_age:
		# 	print("Joint states are not fresh enough for inference")
		# 	return None

		state_vector = self._build_state_vector()
		if state_vector is None:
			print("State vector is not available for inference")
			return None, None

		arm_snapshot: list[Optional[float]] = [self._latest_joint_positions.get(name) for name in self._policy_arm_joint_order]
		if any(val is None for val in arm_snapshot):
			missing = [self._policy_arm_joint_order[idx] for idx, val in enumerate(arm_snapshot) if val is None]
			self.get_logger().debug(f"Waiting for arm joints: {','.join(missing)}")
			return None, None

		images: Dict[str, np.ndarray] = {}
		if wrist:
			images[self._image_slots["wrist"].policy_key] = wrist[1]
		if fixed:
			images[self._image_slots["fixed"].policy_key] = fixed[1]

		force_torques = {
			"left": self._build_ft_timeseries("left"),
			"right": self._build_ft_timeseries("right"),
			"left_ft": self._build_ft_timeseries("left"),
			"right_ft": self._build_ft_timeseries("right"),
		}
		self.get_logger().debug(
			f"FT shapes -> left {force_torques['left'].shape}, right {force_torques['right'].shape}"
		)

		obs = {
			"state": state_vector,
			"images": images,
			"force_torques": force_torques,
		}
		if self._prompt:
			obs["prompt"] = self._prompt
		return obs, [float(v) for v in arm_snapshot]

	def _is_fresh(self, entry: Optional[tuple[Time, np.ndarray]], now: Time) -> bool:
		if entry is None:
			return False
		stamp, _ = entry
		return (now - stamp) <= self._max_data_age

	def _build_state_vector(self) -> Optional[np.ndarray]:
		data = []
		missing = []
		source = self._debug_joint_positions if self._debug_enabled and self._debug_joint_positions else self._latest_joint_positions
		for name in self._state_joint_order:
			if (value := source.get(name)) is None:
				print(f"Missing joint position for {name}")
				missing.append(name)
			else:
				if name in self._gripper_joint_set and self._gripper_distance_scale > 0.0:
					value = 1.0 - value / self._gripper_distance_scale
				data.append(value)

		if missing:
			self.get_logger().debug(f"Waiting for joints: {','.join(missing)}")
			return None

		return np.asarray(data, dtype=np.float32)

	def _extract_log_action(self, actions: np.ndarray) -> Optional[np.ndarray]:
		array = np.asarray(actions)
		if array.ndim == 3:
			if array.shape[0] == 0:
				return None
			array = array[0]
		elif array.ndim == 2:
			pass
		elif array.ndim == 1:
			array = array.reshape(1, -1)
		else:
			return None

		if self._publish_horizon_index >= array.shape[0]:
			return None

		return np.asarray(array[self._publish_horizon_index], dtype=np.float32)

	def _log_io(self, state_vec: Optional[np.ndarray], action_vec: Optional[np.ndarray]) -> None:
		if state_vec is None or action_vec is None:
			return
		try:
			self._io_log_path.parent.mkdir(parents=True, exist_ok=True)
		except Exception:
			return

		with self._lock:
			positions = {name: self._latest_joint_positions.get(name) for name in self._policy_arm_joint_order}
			gripper_raw = None
			if self._gripper_distance_joint_name is not None:
				gripper_raw = self._latest_joint_positions.get(self._gripper_distance_joint_name)

		row = {}
		for name in self._policy_arm_joint_order:
			row[f"in_{name}"] = positions.get(name)

		if gripper_raw is not None and self._gripper_distance_scale > 0.0:
			row["in_gripper_norm"] = 1.0 - float(gripper_raw) / float(self._gripper_distance_scale)
		else:
			row["in_gripper_norm"] = None

		flat_action = np.asarray(action_vec).ravel()
		for idx, name in enumerate(self._policy_arm_joint_order):
			if idx < len(flat_action):
				row[f"out_{name}"] = float(flat_action[idx])
			else:
				row[f"out_{name}"] = None

		grip_vals = [float(flat_action[idx]) for idx in self._gripper_action_indices if idx < len(flat_action)] if flat_action.size else []
		row["out_gripper_norm"] = grip_vals[0] if grip_vals else None

		fieldnames = list(row.keys())
		try:
			file_exists = self._io_log_path.is_file()
			with self._io_log_path.open("a", newline="", encoding="utf-8") as csvfile:
				writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
				if not file_exists:
					writer.writeheader()
				writer.writerow(row)
		except Exception:
			return

	def _record_inference_input(self, obs: dict, stamp: Time) -> None:
		"""Persist the exact inputs seen by the policy for inspection."""
		try:
			inference_id = f"{stamp.nanoseconds}_{self._inference_sequence}"
			self._inference_sequence += 1
			timestamp_sec = float(stamp.nanoseconds) / 1e9

			image_paths: Dict[str, str] = {}
			for key, image in obs.get("images", {}).items():
				img_path = self._input_log_dir / f"{inference_id}_{key}.png"
				self._save_image(image, img_path)
				image_paths[key] = img_path.name

			force_paths: Dict[str, str] = {}
			force_torques = obs.get("force_torques", {}) or {}
			for side in ("left", "right"):
				ft = force_torques.get(side)
				if ft is None:
					continue
				ft_path = self._input_log_dir / f"{inference_id}_{side}_ft.csv"
				self._save_force_timeseries(ft, ft_path)
				force_paths[side] = ft_path.name

			state_vec = obs.get("state")
			row: Dict[str, float | str | None] = {
				"inference_id": inference_id,
				"timestamp_sec": timestamp_sec,
			}
			for slot in self._image_slots.values():
				row[f"image_{slot.policy_key}"] = image_paths.get(slot.policy_key)
			row.update({"ft_left_file": force_paths.get("left"), "ft_right_file": force_paths.get("right")})
			if state_vec is not None:
				for idx, name in enumerate(self._state_joint_order):
					if idx < len(state_vec):
						row[f"state_{name}"] = float(state_vec[idx])

			fieldnames = list(row.keys())
			file_exists = self._input_index_path.is_file()
			with self._input_index_path.open("a", newline="", encoding="utf-8") as csvfile:
				writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
				if not file_exists:
					writer.writeheader()
				writer.writerow(row)
		except Exception as exc:
			self.get_logger().warning(f"Failed to record inference input: {exc}")

	def _save_image(self, image: np.ndarray, path: Path) -> None:
		"""Persist image arrays to PNG for later inspection."""
		arr = np.asarray(image)
		if arr.ndim == 3 and not self._channels_last and arr.shape[0] in (1, 3):
			arr = np.transpose(arr, (1, 2, 0))
		if arr.dtype != np.uint8:
			arr = np.clip(arr, 0, 255).astype(np.uint8)
		if arr.ndim == 3 and arr.shape[2] == 3:
			arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
		cv2.imwrite(str(path), arr)

	def _save_force_timeseries(self, ft: np.ndarray, path: Path) -> None:
		"""Write force-torque horizon to CSV with step indices."""
		arr = np.asarray(ft, dtype=np.float32)
		if arr.ndim == 1:
			arr = arr.reshape(6, -1)
		with path.open("w", newline="", encoding="utf-8") as handle:
			writer = csv.writer(handle)
			writer.writerow(["step", "fx", "fy", "fz", "tx", "ty", "tz"])
			if arr.shape[0] < 6:
				return
			steps = arr.shape[1]
			for idx in range(steps):
				writer.writerow([idx, arr[0, idx], arr[1, idx], arr[2, idx], arr[3, idx], arr[4, idx], arr[5, idx]])

	def _crop_image(self, slot: str, image: np.ndarray) -> np.ndarray:
		params = self._image_crops.get(slot)
		if not params:
			return image
		width, height, x_off, y_off = params
		h, w = image.shape[:2]
		if h < height or w < width:
			return image
		x0 = min(max(0, x_off), w - width)
		y0 = min(max(0, y_off), h - height)
		return image[y0 : y0 + height, x0 : x0 + width]

	def _build_ft_timeseries(self, side: str) -> np.ndarray:
		buffer = self._ft_buffers[side]
		horizon = self._ft_horizon
		if not buffer:
			return np.zeros((6, horizon), dtype=np.float32)
		arr = np.asarray(buffer, dtype=np.float32).T  # (6, count)
		count = arr.shape[1]
		if count < horizon:
			pad = np.repeat(arr[:, :1], horizon - count, axis=1)
			arr = np.concatenate([pad, arr], axis=1)
		return arr[:, -horizon:]

	# ---------------------------------------------------------------------
	# Publishing
	# ---------------------------------------------------------------------
	def _queue_actions(self, actions: np.ndarray, base_positions: Sequence[float]) -> int:
		sequence = self._extract_action_sequence(actions)
		if sequence.size == 0:
			self.get_logger().warning("Policy returned no executable actions")
			return 0
		policy_len = len(self._policy_arm_joint_order)
		if sequence.shape[1] < policy_len:
			self.get_logger().warning(
				f"Action vector dim {sequence.shape[1]} smaller than policy joints {policy_len}"
			)
			return 0
		base_array = np.asarray(base_positions, dtype=np.float32)
		if base_array.shape[0] != policy_len:
			self.get_logger().warning("Base joint snapshot length mismatch; skipping action queue")
			return 0

		# Use the full policy horizon (e.g., 50 steps) and overwrite any pending queue.
		absolute_positions = base_array + sequence[:, :policy_len]
		with self._lock:
			self._pending_actions = sequence.copy()
			self._pending_absolute_positions = absolute_positions
			self._pending_action_index = 0
			return len(self._pending_actions)

	def _extract_action_sequence(self, actions: np.ndarray) -> np.ndarray:
		array = np.asarray(actions)
		if array.ndim == 3:
			if array.shape[0] == 0:
				return np.empty((0, 0), dtype=np.float32)
			array = array[0]
		elif array.ndim == 2:
			pass
		elif array.ndim == 1:
			array = array.reshape(1, -1)
		else:
			self.get_logger().warning(f"Unexpected action tensor shape: {array.shape}")
			return np.empty((0, 0), dtype=np.float32)

		if self._publish_horizon_index >= array.shape[0]:
			self.get_logger().warning(
				f"Publish index {self._publish_horizon_index} outside action horizon {array.shape[0]}"
			)
			return np.empty((0, 0), dtype=np.float32)

		return np.ascontiguousarray(array[self._publish_horizon_index :], dtype=np.float32)

	def _action_timer(self) -> None:
		with self._lock:
			if self._pending_actions is None:
				# No queued actions; nothing to publish.
				return
			if self._pending_action_index >= len(self._pending_actions):
				# Finished current batch; clear queue.
				self._pending_actions = None
				self._pending_absolute_positions = None
				self._pending_action_index = 0
				return
			idx = self._pending_action_index
			action_vec = self._pending_actions[idx]
			abs_positions = None
			if self._pending_absolute_positions is not None and idx < len(self._pending_absolute_positions):
				abs_positions = self._pending_absolute_positions[idx]
			self._pending_action_index += 1

		if abs_positions is None:
			self.get_logger().debug("No base snapshot available; skipping action publish")
			return
		self._publish_action_vector(action_vec, abs_positions)

	def _publish_action_vector(
		self,
		action_vec: np.ndarray,
		absolute_policy_positions: Sequence[float],
	) -> None:
		flat_vec = np.asarray(action_vec).ravel()
		policy_len = len(self._policy_arm_joint_order)
		if len(flat_vec) < policy_len:
			self.get_logger().warning(
				f"Action vector dim {len(flat_vec)} smaller than policy joints {policy_len}"
			)
			return
		if len(absolute_policy_positions) < policy_len:
			self.get_logger().warning("Absolute positions length smaller than policy joints")
			return

		if self._debug_enabled:
			self._log_debug_action(absolute_policy_positions, flat_vec)
			return

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
			if idx is None or idx >= len(absolute_policy_positions):
				self.get_logger().warning(
					f"Missing joint {name} in policy positions dim={len(absolute_policy_positions)}"
				)
				return
			positions.append(float(absolute_policy_positions[idx]))

		if self._forward_mode and self._forward_pub is not None:
			msg = Float64MultiArray()
			msg.data = positions
			msg.layout.data_offset = 0
			self._forward_pub.publish(msg)
			# keep going to allow gripper publish

		if self._arm_pub is not None and self._arm_action_indices:
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
			point.time_from_start.nanosec = 50_000_000
			arm_cmd.points.append(point)
			self._arm_pub.publish(arm_cmd)

		if self._gripper_action_indices:
			gripper_values = [float(flat_vec[idx]) for idx in self._gripper_action_indices if idx < len(flat_vec)]
			if gripper_values:
				if not self._send_gripper_goal(gripper_values[0]):
					self._publish_gripper_fallback(gripper_values)

	def _send_gripper_goal(self, normalized_opening: float) -> bool:
		"""Send gripper command via action interface. Returns True if handled."""
		clamped = max(0.0, min(1.0, 1.0 - normalized_opening))
		target = clamped * self._gripper_position_scale

		# If mux publishing is enabled, send to mux topic and stop here.
		if self._gripper_mux_pub is not None:
			msg = Float64()
			msg.data = target
			self._gripper_mux_pub.publish(msg)
			self._last_gripper_target = target
			return True

		if self._gripper_action_client is None:
			return False

		if not self._gripper_action_client.wait_for_server(timeout_sec=0.0):
			self.get_logger().warning(f"Waiting for {self._gripper_action_name} server")
			return False

		if self._debug_enabled:
			self.get_logger().info(
				f"Debug gripper target -> {target:.4f} (normalized={normalized_opening:.4f})"
			)
			return True
		if self._last_gripper_target is not None and abs(target - self._last_gripper_target) < self._gripper_action_deadband:
			return True

		goal = MoveTwoFingerGripper.Goal()
		goal.target_position = target
		goal.target_speed = self._gripper_target_speed
		goal.target_force = self._gripper_target_force
		self._gripper_action_client.send_goal_async(goal)
		self._last_gripper_target = target
		return True

	def _publish_gripper_fallback(self, gripper_values: list[float]) -> None:
		"""Fallback to publishing on legacy topics if action control is unavailable."""
		if self._debug_enabled:
			self.get_logger().info(f"Debug gripper fallback -> {gripper_values}")
			return
		if self._gripper_pub is None:
			return
		if self._gripper_uses_multiarray:
			msg_multi = Float64MultiArray()
			msg_multi.data = gripper_values
			self._gripper_pub.publish(msg_multi)
		else:
			msg_scalar = Float64()
			msg_scalar.data = gripper_values[0]
			self._gripper_pub.publish(msg_scalar)

def main(args: Optional[Sequence[str]] = None) -> None:
	parser = argparse.ArgumentParser(add_help=False)
	parser.add_argument("--forward", action="store_true")
	parsed, remaining = parser.parse_known_args(args)
	rclpy.init(args=remaining)
	node = FTVLAInferenceNode(forward_mode=parsed.forward)
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
