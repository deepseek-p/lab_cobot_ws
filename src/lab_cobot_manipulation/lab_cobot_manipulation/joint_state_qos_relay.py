#!/usr/bin/env python3
"""Bridge Gazebo best-effort joint states to a reliable MoveIt topic."""

import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState


class JointStateQosRelay(Node):
    """Republish joint states with a QoS compatible with MoveIt's monitor."""

    def __init__(self, input_topic: str, output_topic: str) -> None:
        super().__init__("joint_state_qos_relay")
        sensor_qos = QoSProfile(
            depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        reliable_qos = QoSProfile(
            depth=100, reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.publisher = self.create_publisher(
            JointState, output_topic, reliable_qos
        )
        self.subscription = self.create_subscription(
            JointState, input_topic, self.publisher.publish, sensor_qos
        )
        self.get_logger().info(
            f"Relaying {input_topic} (best effort) to {output_topic} (reliable)"
        )


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", default="/joint_states")
    parser.add_argument("--output", default="/moveit_joint_states")
    parsed, ros_args = parser.parse_known_args(args=args)
    rclpy.init(args=ros_args)
    node = JointStateQosRelay(parsed.input, parsed.output)
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
