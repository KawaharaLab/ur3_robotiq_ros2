from __future__ import annotations

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

		# Topic + policy wiring parameters.
		self._image_height = self.declare_parameter("image_height", 224).get_parameter_value().integer_value
		self._image_width = self.declare_parameter("image_width", 224).get_parameter_value().integer_value
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
				"shoulder_lift_joint",
				"elbow_joint",
				"wrist_1_joint",
				"wrist_2_joint",
				"wrist_3_joint",
				"shoulder_pan_joint",
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

		state_order = self._get_str_list_param("state_joint_order", default=[])
		if state_order:
			self._state_joint_order = state_order
		else:
			self._state_joint_order = [*self._policy_arm_joint_order, *self._gripper_joint_names]

		self._state_pad_length = self.declare_parameter("state_pad_length", 14).get_parameter_value().integer_value
		self._max_data_age = Duration(seconds=self.declare_parameter("max_data_age", 0.5).value)
		self._action_execution_period = float(self.declare_parameter("action_execution_period", 0.05).value)
		actions_per_inference_param = self.declare_parameter("actions_per_inference", 16)
		self._actions_per_inference = max(1, actions_per_inference_param.get_parameter_value().integer_value)
		default_inference_period = self._actions_per_inference * self._action_execution_period
		self._inference_period = float(
			self.declare_parameter("inference_period", default_inference_period).value
		)
		self._prompt = self.declare_parameter("default_prompt", "Pick up the blue object and place it in the orange box.").value

		self._publish_horizon_index = self.declare_parameter("publish_horizon_index", 0).get_parameter_value().integer_value
		default_arm_action_indices: list[int] = []
		for joint in self._arm_joint_names:
			idx = self._policy_arm_index.get(joint)
			if idx is None:
				self.get_logger().warning("Joint %s missing from policy order", joint)
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
			"arm_command_topic", "/forward_position_controller/commands"
		).value
		self._gripper_command_topic = self.declare_parameter(
			"gripper_command_topic", "/robotiq_gripper/command"
		).value
		self._gripper_action_name = self.declare_parameter(
			"gripper_action_name", "/robotiq_2f_gripper_action"
		).value

		self._arm_pub = None if not self._arm_command_topic else self.create_publisher(
			Float64MultiArray, self._arm_command_topic, 10
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
		self.create_timer(self._inference_period, self._inference_timer)
		self.create_timer(self._action_execution_period, self._action_timer)

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
	# Policy wiring
	# ---------------------------------------------------------------------
	def _create_local_policy(self) -> PolicyHandle:
		config_name = self.declare_parameter("policy_config_name", "pi0_ur3_robotiq").value
		checkpoint_uri = self.declare_parameter(
			"policy_checkpoint_uri", "gs://openpi-assets/checkpoints/pi0_base"
		).value
		pytorch_device_param = self.declare_parameter("policy_pytorch_device", "auto").value
		default_prompt_override = self.declare_parameter("policy_default_prompt", self._prompt).value
		sample_steps = self.declare_parameter("policy_sample_steps", 10).get_parameter_value().integer_value
		sample_kwargs = {"num_steps": sample_steps} if sample_steps > 0 else {}

		checkpoint_dir = _download.maybe_download(checkpoint_uri)
		train_config = _config.get_config(config_name)
		pytorch_device = None if pytorch_device_param in ("", "auto") else pytorch_device_param
		policy = _policy_config.create_trained_policy(
			train_config,
			checkpoint_dir,
			sample_kwargs=sample_kwargs,
			default_prompt=default_prompt_override or None,
			pytorch_device=pytorch_device,
		)
		self._policy_descriptor = f"{config_name} @ {checkpoint_dir}"
		try:
			metadata = policy.metadata
			self.get_logger().info("Loaded OpenPI policy metadata: %s", metadata)
		except Exception:
			print("Failed to retrieve policy metadata with exception:", sys.exc_info()[1])
			pass
		return policy

	# ---------------------------------------------------------------------
	# Subscriptions
	# ---------------------------------------------------------------------
	def _image_callback(self, slot: str, msg: Image) -> None:
		try:
			cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding=self._image_encoding)
		except CvBridgeError as exc:
			self.get_logger().warning("Failed to convert %s image: %s", slot, exc)
			return

		if self._image_height > 0 and self._image_width > 0:
			cv_image = cv2.resize(cv_image, (self._image_width, self._image_height), interpolation=cv2.INTER_LINEAR)

		if not self._channels_last:
			cv_image = np.transpose(cv_image, (2, 0, 1))

		stamp = Time.from_msg(msg.header.stamp) if msg.header.stamp else self.get_clock().now()

		with self._lock:
			self._latest_images[slot] = (stamp, np.ascontiguousarray(cv_image))

	def _joint_state_callback(self, msg: JointState) -> None:
		positions = dict(zip(msg.name, msg.position))
		stamp = Time.from_msg(msg.header.stamp) if msg.header.stamp else self.get_clock().now()
		with self._lock:
			self._latest_joint_positions.update(positions)
			self._last_joint_stamp = stamp

	def _gripper_distance_callback(self, msg: Float32) -> None:
		"""Handle Float32 gripper distance topics (e.g. finger_distance_mm)."""
		if not self._gripper_distance_joint_name:
			return
		stamp = self.get_clock().now()
		with self._lock:
			self._latest_joint_positions[self._gripper_distance_joint_name] = float(msg.data)
			self._last_joint_stamp = stamp

	# ---------------------------------------------------------------------
	# Inference loop
	# ---------------------------------------------------------------------
	def _inference_timer(self) -> None:
		with self._lock:
			obs = self._build_observation_locked()
		if obs is None:
			print("No observation available for inference")
			return
		print("obs: ", obs["state"])
		try:
			result = self._policy.infer(obs)
		except Exception as exc:  # pragma: no cover - depends on server
			self.get_logger().error("Policy inference failed: %s", exc)
			return

		actions = result.get("actions")
		if actions is None:
			self.get_logger().warning("Policy response missing 'actions' key: %s", result.keys())
			return
		print(actions[0])
		self._queue_actions(np.asarray(actions))

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
		for name in self._state_joint_order:
			if (value := self._latest_joint_positions.get(name)) is None:
				print(f"Missing joint position for {name}")
				missing.append(name)
			else:
				if name in self._gripper_joint_set and self._gripper_distance_scale > 0.0:
					value = 1.0 - value / self._gripper_distance_scale
				data.append(value)

		if missing:
			self.get_logger().debug(f"Waiting for joints: {','.join(missing)}")
			return None

		state = np.zeros(self._state_pad_length, dtype=np.float32)
		state[: len(data)] = data
		return state

	# ---------------------------------------------------------------------
	# Publishing
	# ---------------------------------------------------------------------
	def _queue_actions(self, actions: np.ndarray) -> None:
		sequence = self._extract_action_sequence(actions)
		if sequence.size == 0:
			self.get_logger().warning("Policy returned no executable actions")
			return
		max_len = min(self._actions_per_inference, sequence.shape[0])
		with self._lock:
			self._pending_actions = sequence[:max_len].copy()
			self._pending_action_index = 0

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
			self.get_logger().warning("Unexpected action tensor shape: %s", array.shape)
			return np.empty((0, 0), dtype=np.float32)

		if self._publish_horizon_index >= array.shape[0]:
			self.get_logger().warning(
				"Publish index %d outside action horizon %d",
				self._publish_horizon_index,
				array.shape[0],
			)
			return np.empty((0, 0), dtype=np.float32)

		return np.ascontiguousarray(array[self._publish_horizon_index :], dtype=np.float32)

	def _action_timer(self) -> None:
		with self._lock:
			if self._pending_actions is None:
				return
			if self._pending_action_index >= len(self._pending_actions):
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
				"Action vector dim %d smaller than policy joints %d", len(flat_vec), policy_len
			)
			return
		if any(pos is None for pos in current_policy_positions):
			self.get_logger().debug("Waiting for complete joint state before applying deltas")
			return
		absolute_policy_positions = [
			float(current_policy_positions[idx]) + float(flat_vec[idx]) for idx in range(policy_len)
		]

		if self._arm_pub is not None and self._arm_action_indices:
			arm_cmd = Float64MultiArray()
			arm_cmd.data = [
				absolute_policy_positions[idx]
				for idx in self._arm_action_indices
				if idx < len(absolute_policy_positions)
			]
			if not arm_cmd.data:
				self.get_logger().warning(
					"No arm command indices overlapped with policy joints dim=%d",
					len(absolute_policy_positions),
				)
			else:
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
			self.get_logger().warning("Waiting for %s server", self._gripper_action_name)
			return False

		clamped = max(0.0, min(1.0, 1.0 - normalized_opening))
		target = clamped * self._gripper_position_scale
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
		rclpy.shutdown()


if __name__ == "__main__":
	main()
