from __future__ import annotations

import csv
import sys
import threading
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
from sensor_msgs.msg import Image, JointState
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


class Pi0InferenceNode(Node):
	"""Bridge ROS topics into Pi0 policy calls and publish the resulting commands."""

	def __init__(self) -> None:
		super().__init__("pi0_inference_node")

		self._bridge = CvBridge()
		self._lock = threading.Lock()

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
			"wrist_camera_topic", "camera/camera_wrist/color/image_raw"
		).value
		fixed_topic = self.declare_parameter(
			"fixed_camera_topic", "camera/camera_fixed/color/image_raw"
		).value
		self._image_slots = {
			"wrist": ImageSlot(
				subscription_topic=wrist_topic,
				policy_key=self.declare_parameter("policy_wrist_image_key", "cam_left_wrist").value,
			),
			"fixed": ImageSlot(
				subscription_topic=fixed_topic,
				policy_key=self.declare_parameter("policy_fixed_image_key", "cam_high").value,
			),
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
		self._prompt = self.declare_parameter("default_prompt", "Pick up the blue cube.").value
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

		self._arm_command_topic = self.declare_parameter(
			"arm_command_topic", "/scaled_joint_trajectory_controller/joint_trajectory"
		).value
		self._gripper_command_topic = self.declare_parameter(
			"gripper_command_topic", "/robotiq_gripper/command"
		).value
		self._gripper_action_name = self.declare_parameter(
			"gripper_action_name", "/robotiq_2f_gripper_action"
		).value

		self._arm_pub = None if not self._arm_command_topic else self.create_publisher(
			JointTrajectory, self._arm_command_topic, qos_arm_cmd
		)
		if self._gripper_command_topic:
			gripper_type = Float64MultiArray if self._gripper_uses_multiarray else Float64
			self._gripper_pub = self.create_publisher(gripper_type, self._gripper_command_topic, 10)
		else:
			self._gripper_pub = None

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
			self.create_subscription(
				Image,
				self._image_slots["wrist"].subscription_topic,
				lambda msg: self._image_callback("wrist", msg),
				qos_camera,
			)
		if self._image_slots["fixed"].subscription_topic:
			self.create_subscription(
				Image,
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

		# Load the OpenPI policy directly on this machine (GPU-friendly).
		self._policy = self._create_local_policy()

		# Periodic inference loop.
		self._inference_timer_handle = self.create_timer(self._inference_period, self._inference_timer)
		self._action_timer_handle = None
		if not self._debug_enabled:
			self._action_timer_handle = self.create_timer(self._action_execution_period, self._action_timer)

		self.get_logger().info(
			f"Pi0 inference node ready (policy={self._policy_descriptor}, arm_topic={self._arm_joint_topic},"
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
		current_policy_positions: list[Optional[float]] = [
			self._latest_joint_positions.get(name) for name in self._policy_arm_joint_order
		]
		for step_idx in range(max_len):
			action_vec = sequence[step_idx]
			self._publish_action_vector(action_vec, current_policy_positions)
			if any(pos is None for pos in current_policy_positions):
				continue
			flat_vec = np.asarray(action_vec).ravel()
			policy_len = len(current_policy_positions)
			for joint_idx in range(min(policy_len, len(flat_vec))):
				current_policy_positions[joint_idx] = (
					float(current_policy_positions[joint_idx]) + float(flat_vec[joint_idx])
				)
		self._debug_single_shot_done = True
		self.get_logger().info(f"Debug inference emitted {max_len} action vectors")
		self._shutdown_debug_mode()

	# ---------------------------------------------------------------------
	# Policy wiring
	# ---------------------------------------------------------------------
	def _create_local_policy(self) -> PolicyHandle:
		config_name = self.declare_parameter("policy_config_name", "pi0_ur3_robotiq").value
		checkpoint_uri = self.declare_parameter(
			"policy_checkpoint_uri", "gs://openpi-assets/checkpoints/pi0_base_pytorch"
		).value
		pytorch_device_param = self.declare_parameter("policy_pytorch_device", "auto").value
		default_prompt_override = self.declare_parameter("policy_default_prompt", self._prompt).value
		# sample_steps = self.declare_parameter("policy_sample_steps", 10).get_parameter_value().integer_value
		# sample_kwargs = {"num_steps": sample_steps} if sample_steps > 0 else {}

		checkpoint_dir = _download.maybe_download(checkpoint_uri)
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
			cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding=self._image_encoding)
		except CvBridgeError as exc:
			self.get_logger().warning(f"Failed to convert {slot} image: {exc}")
			return

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

	# ---------------------------------------------------------------------
	# Inference loop
	# ---------------------------------------------------------------------
	def _inference_timer(self) -> None:
		if self._debug_enabled and self._debug_single_shot_done:
			return
		with self._lock:
			obs = self._build_observation_locked()
		if obs is None:
			print("No observation available for inference")
			return
		print("obs: ", obs["state"])
		try:
			result = self._policy.infer(obs)
		except Exception as exc:  # pragma: no cover - depends on server
			self.get_logger().error(f"Policy inference failed: {exc}")
			return

		actions = result.get("actions")
		if actions is None:
			self.get_logger().warning(f"Policy response missing 'actions' key: {list(result.keys())}")
			return
		action_array = np.asarray(actions)
		print(action_array[0])
		if self._debug_enabled:
			self._run_debug_action_sequence(action_array)
			return
		queued = self._queue_actions(action_array)

	def _build_observation_locked(self) -> Optional[dict]:
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
			return None

		images: Dict[str, np.ndarray] = {}
		if wrist:
			images[self._image_slots["wrist"].policy_key] = wrist[1]
		if fixed:
			images[self._image_slots["fixed"].policy_key] = fixed[1]

		obs = {
			"state": state_vector,
			"images": images,
		}
		if self._prompt:
			obs["prompt"] = self._prompt
		return obs

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

	# ---------------------------------------------------------------------
	# Publishing
	# ---------------------------------------------------------------------
	def _queue_actions(self, actions: np.ndarray) -> int:
		sequence = self._extract_action_sequence(actions)
		if sequence.size == 0:
			self.get_logger().warning("Policy returned no executable actions")
			return 0
		max_len = min(self._actions_per_inference, sequence.shape[0])
		with self._lock:
			self._pending_actions = sequence[:max_len].copy()
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
				self._pending_action_index = 0
				return
			action_vec = self._pending_actions[self._pending_action_index]
			self._pending_action_index += 1
			current_policy_positions = [
				self._latest_joint_positions.get(name) for name in self._policy_arm_joint_order
			]

		self._publish_action_vector(action_vec, current_policy_positions)

	def _publish_action_vector(
		self,
		action_vec: np.ndarray,
		current_policy_positions: Sequence[Optional[float]],
	) -> None:
		flat_vec = np.asarray(action_vec).ravel()
		policy_len = len(self._policy_arm_joint_order)
		if len(flat_vec) < policy_len:
			self.get_logger().warning(
				f"Action vector dim {len(flat_vec)} smaller than policy joints {policy_len}"
			)
			return
		if any(pos is None for pos in current_policy_positions):
			self.get_logger().debug("Waiting for complete joint state before applying deltas")
			return
		absolute_policy_positions = [
			float(current_policy_positions[idx]) + float(flat_vec[idx]) for idx in range(policy_len)
		]

		if self._debug_enabled:
			self._log_debug_action(absolute_policy_positions, flat_vec)
			return

		if self._arm_pub is not None and self._arm_action_indices:
			joint_names = [
				self._policy_arm_joint_order[idx]
				for idx in self._arm_action_indices
				if idx < len(self._policy_arm_joint_order)
			]
			positions = [
				absolute_policy_positions[idx]
				for idx in self._arm_action_indices
				if idx < len(absolute_policy_positions)
			]
			if not joint_names or not positions:
				self.get_logger().warning(
					f"No arm command indices overlapped with policy joints dim={len(absolute_policy_positions)}"
				)
			else:
				arm_cmd = JointTrajectory()
				arm_cmd.joint_names = joint_names
				point = JointTrajectoryPoint()
				point.positions = positions
				point.time_from_start.sec = int(self._action_execution_period)
				point.time_from_start.nanosec = int((self._action_execution_period - int(self._action_execution_period)) * 1e9)
				arm_cmd.points.append(point)
				self._arm_pub.publish(arm_cmd)

		if self._gripper_action_indices:
			gripper_values = [float(flat_vec[idx]) for idx in self._gripper_action_indices if idx < len(flat_vec)]
			if gripper_values:
				if not self._send_gripper_goal(gripper_values[0]):
					self._publish_gripper_fallback(gripper_values)

	def _send_gripper_goal(self, normalized_opening: float) -> bool:
		"""Send gripper command via action interface. Returns True if handled."""
		if self._gripper_action_client is None:
			return False

		if not self._gripper_action_client.wait_for_server(timeout_sec=0.0):
			self.get_logger().warning(f"Waiting for {self._gripper_action_name} server")
			return False

		clamped = max(0.0, min(1.0, 1.0 - normalized_opening))
		target = clamped * self._gripper_position_scale
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
	rclpy.init(args=args)
	node = Pi0InferenceNode()
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
