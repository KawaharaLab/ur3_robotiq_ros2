#!/usr/bin/env python3
"""ROS 2 node that publishes GelSight Mini frames as sensor_msgs/Image messages."""

from __future__ import annotations

import argparse
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from .config import GSConfig, ConfigModel
from .utilities.gelsightmini import GelSightMini


class GelSightMiniPublisher(Node):
    def __init__(
        self,
        config: ConfigModel,
        topic_name: str,
        publish_rate_hz: float,
        frame_id: str,
        device_index: Optional[int] = None,
    ) -> None:
        super().__init__("gelsight_mini_publisher")
        self._bridge = CvBridge()
        self._topic_name = topic_name
        self._frame_id = frame_id
        self._publish_rate_hz = publish_rate_hz if publish_rate_hz > 0 else 30.0

        self._cam_stream = GelSightMini(
            target_width=config.camera_width,
            target_height=config.camera_height,
            border_fraction=config.border_fraction,
        )

        target_device = device_index if device_index is not None else config.default_camera_index
        self.get_logger().info(f"Opening GelSight Mini camera (index: {target_device})")
        self._cam_stream.select_device(target_device)
        self._cam_stream.start()

        self._publisher = self.create_publisher(Image, topic_name, 10)
        timer_period = 1.0 / self._publish_rate_hz
        self._timer = self.create_timer(timer_period, self._publish_frame)

    def _publish_frame(self) -> None:
        frame = self._cam_stream.update(0.0)
        if frame is None:
            self.get_logger().warn(
                "No frame received from GelSight Mini camera",
                throttle_duration_sec=2.0,
            )
            return

        msg = self._bridge.cv2_to_imgmsg(frame, encoding="rgb8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        self._publisher.publish(msg)

    def destroy_node(self) -> None:
        self.get_logger().info("Shutting down GelSight Mini publisher")
        if hasattr(self, "_timer") and self._timer is not None:
            self._timer.cancel()
        if self._cam_stream and self._cam_stream.camera:
            self._cam_stream.camera.release()
        super().destroy_node()


def parse_arguments() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description="Publish GelSight Mini tactile images to a ROS 2 topic."
    )
    parser.add_argument(
        "--gs-config",
        type=str,
        default="config/default_config.json",
        help="Path to the GelSight configuration JSON file.",
    )
    parser.add_argument(
        "--device-index",
        type=int,
        default=None,
        help="Camera index override. Falls back to default_camera_index from config.",
    )
    parser.add_argument(
        "--topic-name",
        type=str,
        default="gelsight/image_raw",
        help="ROS 2 topic to publish Image messages to.",
    )
    parser.add_argument(
        "--frame-id",
        type=str,
        default="gelsight_mini",
        help="Frame ID attached to the published Image messages.",
    )
    parser.add_argument(
        "--publish-rate",
        type=float,
        default=30.0,
        help="Desired publication rate in Hz.",
    )

    return parser.parse_known_args()


def main() -> None:
    args, ros_args = parse_arguments()

    gs_config = GSConfig(args.gs_config)

    rclpy.init(args=ros_args)
    node = GelSightMiniPublisher(
        config=gs_config.config,
        topic_name=args.topic_name,
        publish_rate_hz=args.publish_rate,
        frame_id=args.frame_id,
        device_index=args.device_index,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()