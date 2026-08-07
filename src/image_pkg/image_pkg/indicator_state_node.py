#!/usr/bin/env python3
"""Publish a colour-recognised device status as a JSON semantic observation."""
import json

import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from image_pkg.indicator_state import classify_indicator


class IndicatorStateNode(Node):
    """Map a configured camera ROI's red/amber/green lamp to device state."""

    def __init__(self):
        super().__init__("indicator_state")
        defaults = {
            "image_topic": "/bench_camera/image_raw",
            "state_topic": "/perception/device_states",
            "device_id": "pcb_test_fixture",
            "roi": [0, 0, 0, 0],
            "min_pixels": 20,
            "dominance": 0.60,
            "publish_only_on_change": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.bridge = CvBridge()
        self.device_id = str(self.get_parameter("device_id").value)
        self.roi = tuple(self.get_parameter("roi").value)
        if len(self.roi) != 4:
            raise ValueError("roi must be [x, y, width, height]")
        self.min_pixels = int(self.get_parameter("min_pixels").value)
        self.dominance = float(self.get_parameter("dominance").value)
        self.publish_only_on_change = bool(
            self.get_parameter("publish_only_on_change").value)
        self._last_state = None
        self.publisher = self.create_publisher(
            String, str(self.get_parameter("state_topic").value), 10)
        self.create_subscription(
            Image, str(self.get_parameter("image_topic").value),
            self._image_cb, qos_profile_sensor_data)

    def _image_cb(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            result = classify_indicator(
                image, self.roi, self.min_pixels, self.dominance)
        except (CvBridgeError, ValueError) as exc:
            self.get_logger().warning(f"indicator frame skipped: {exc}")
            return
        if self.publish_only_on_change and result["state"] == self._last_state:
            return
        self._last_state = result["state"]
        output = String()
        output.data = json.dumps({
            "device_id": self.device_id,
            "state": result["state"],
            "color": result["color"],
            "confidence": result["confidence"],
            "pixel_counts": result["pixel_counts"],
            "frame_id": message.header.frame_id,
            "stamp": message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
        })
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = IndicatorStateNode()
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
