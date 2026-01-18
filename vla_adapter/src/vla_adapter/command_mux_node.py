from __future__ import annotations

import sys
import threading
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float64, Float64MultiArray
from robotiq_2f_gripper_msgs.action import MoveTwoFingerGripper


class CommandMuxNode(Node):
    """Multiplex two command streams into one, switchable via keyboard input."""

    def __init__(self) -> None:
        super().__init__("command_mux_node")

        qos = QoSProfile(depth=5)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos.history = QoSHistoryPolicy.KEEP_LAST

        self._topic_vla = self.declare_parameter(
            "vla_topic", "/forward_position_controller/commands_vla"
        ).value
        self._topic_teleop = self.declare_parameter(
            "teleop_topic", "/forward_position_controller/commands_teleop"
        ).value
        self._output_topic = self.declare_parameter(
            "output_topic", "/forward_position_controller/commands"
        ).value
        self._active = self.declare_parameter("active_source", "teleop").value  # "vla" or "teleop"

        # Gripper mux parameters
        self._gripper_topic_vla = self.declare_parameter(
            "gripper_vla_topic", "/robotiq_gripper/command_vla"
        ).value
        self._gripper_topic_teleop = self.declare_parameter(
            "gripper_teleop_topic", "/robotiq_gripper/command_teleop"
        ).value
        self._gripper_action_name = self.declare_parameter(
            "gripper_action_name", "/robotiq_2f_gripper_action"
        ).value
        self._gripper_target_speed = float(self.declare_parameter("gripper_target_speed", 0.5).value)
        self._gripper_target_force = float(self.declare_parameter("gripper_target_force", 0.5).value)
        self._gripper_deadband = float(self.declare_parameter("gripper_deadband", 0.002).value)

        self._gripper_last_target: float | None = None
        self._gripper_action = ActionClient(self, MoveTwoFingerGripper, self._gripper_action_name)

        self._pub = self.create_publisher(Float64MultiArray, self._output_topic, qos)
        self.create_subscription(Float64MultiArray, self._topic_vla, self._vla_cb, qos)
        self.create_subscription(Float64MultiArray, self._topic_teleop, self._teleop_cb, qos)

        # Gripper subscriptions
        self.create_subscription(Float64, self._gripper_topic_vla, lambda msg: self._gripper_cb(msg, "vla"), qos)
        self.create_subscription(Float64, self._gripper_topic_teleop, lambda msg: self._gripper_cb(msg, "teleop"), qos)

        self._lock = threading.Lock()
        self._shutdown = threading.Event()
        self._keyboard_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._keyboard_thread.start()

        self.get_logger().info(
            f"Mux ready. sources: vla={self._topic_vla}, teleop={self._topic_teleop}, output={self._output_topic}, active={self._active}"
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _vla_cb(self, msg: Float64MultiArray) -> None:
        self._maybe_forward(msg, source="vla")

    def _teleop_cb(self, msg: Float64MultiArray) -> None:
        self._maybe_forward(msg, source="teleop")

    def _maybe_forward(self, msg: Float64MultiArray, source: str) -> None:
        with self._lock:
            if source != self._active:
                return
        self._pub.publish(msg)

    def _gripper_cb(self, msg: Float64, source: str) -> None:
        with self._lock:
            if source != self._active:
                return
        target = float(msg.data)
        if self._gripper_last_target is not None and abs(target - self._gripper_last_target) < self._gripper_deadband:
            return
        if not self._gripper_action.wait_for_server(timeout_sec=0.0):
            self.get_logger().warning_once(f"Waiting for {self._gripper_action_name} server")
            return
        goal = MoveTwoFingerGripper.Goal()
        goal.target_position = target
        goal.target_speed = self._gripper_target_speed
        goal.target_force = self._gripper_target_force
        self._gripper_action.send_goal_async(goal)
        self._gripper_last_target = target

    # ------------------------------------------------------------------
    # Keyboard handling
    # ------------------------------------------------------------------
    def _keyboard_loop(self) -> None:
        prompt = (
            "[mux] type 'v' for VLA, 't' for teleop, 'q' to quit. "
            f"(current={self._active})\n"
        )
        sys.stdout.write(prompt)
        sys.stdout.flush()
        while not self._shutdown.is_set():
            line = sys.stdin.readline()
            if not line:
                # EOF, stop reading
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
                sys.stdout.write("[mux] use v/t/q\n")
                sys.stdout.flush()

    def _set_active(self, source: str) -> None:
        with self._lock:
            if source == self._active:
                return
            self._active = source
        self.get_logger().info(f"Active source -> {source}")

    # ------------------------------------------------------------------
    def destroy_node(self) -> bool:
        self._shutdown.set()
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = CommandMuxNode()
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
