from __future__ import annotations

import sys
import threading
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float64
from robotiq_2f_gripper_msgs.action import MoveTwoFingerGripper


class GripperCommandMuxNode(Node):
    """Mux two gripper command topics into a single action goal stream."""

    def __init__(self) -> None:
        super().__init__("gripper_command_mux_node")

        qos = QoSProfile(depth=5)
        qos.history = QoSHistoryPolicy.KEEP_LAST
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self._topic_vla = self.declare_parameter(
            "vla_topic", "/robotiq_gripper/command_vla"
        ).value
        self._topic_teleop = self.declare_parameter(
            "teleop_topic", "/robotiq_gripper/command_teleop"
        ).value
        self._action_name = self.declare_parameter(
            "action_name", "/robotiq_2f_gripper_action"
        ).value
        self._target_speed = float(self.declare_parameter("target_speed", 0.5).value)
        self._target_force = float(self.declare_parameter("target_force", 0.5).value)
        self._deadband = float(self.declare_parameter("deadband", 0.002).value)
        self._active = self.declare_parameter("active_source", "vla").value  # "vla" or "teleop"

        self._last_target: float | None = None

        self._action_client: ActionClient = ActionClient(self, MoveTwoFingerGripper, self._action_name)

        self.create_subscription(Float64, self._topic_vla, lambda msg: self._cb(msg, "vla"), qos)
        self.create_subscription(Float64, self._topic_teleop, lambda msg: self._cb(msg, "teleop"), qos)

        self._lock = threading.Lock()
        self._shutdown = threading.Event()
        self._keyboard_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._keyboard_thread.start()

        self.get_logger().info(
            f"Gripper mux ready. sources: vla={self._topic_vla}, teleop={self._topic_teleop}, action={self._action_name}, active={self._active}"
        )

    def _cb(self, msg: Float64, source: str) -> None:
        with self._lock:
            if source != self._active:
                return
        target = float(msg.data)
        if self._last_target is not None and abs(target - self._last_target) < self._deadband:
            return
        if not self._action_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warning_once(f"Waiting for {self._action_name} server")
            return
        goal = MoveTwoFingerGripper.Goal()
        goal.target_position = target
        goal.target_speed = self._target_speed
        goal.target_force = self._target_force
        self._action_client.send_goal_async(goal)
        self._last_target = target

    def _keyboard_loop(self) -> None:
        prompt = (
            "[gripper mux] type 'v' for VLA, 't' for teleop, 'q' to quit. "
            f"(current={self._active})\n"
        )
        sys.stdout.write(prompt)
        sys.stdout.flush()
        while not self._shutdown.is_set():
            line = sys.stdin.readline()
            if not line:
                break
            cmd = line.strip().lower()
            if cmd == "v":
                self._set_active("vla")
            elif cmd == "t":
                self._set_active("teleop")
            elif cmd == "q":
                rclpy.shutdown()
                break
            else:
                sys.stdout.write("[gripper mux] use v/t/q\n")
                sys.stdout.flush()

    def _set_active(self, source: str) -> None:
        with self._lock:
            if source == self._active:
                return
            self._active = source
        self.get_logger().info(f"Active gripper source -> {source}")

    def destroy_node(self) -> bool:
        self._shutdown.set()
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = GripperCommandMuxNode()
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
