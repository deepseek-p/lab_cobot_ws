#!/usr/bin/env python3
"""Publish passive wheel joint states for MoveIt current state completeness."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


WHEEL_JOINTS = [
    "wheel_fl_joint",
    "wheel_fr_joint",
    "wheel_rl_joint",
    "wheel_rr_joint",
]


class WheelJointStatePublisher(Node):
    def __init__(self):
        super().__init__("wheel_joint_state_publisher")
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(0.1, self._publish)

    def _publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = WHEEL_JOINTS
        msg.position = [0.0] * len(WHEEL_JOINTS)
        msg.velocity = [0.0] * len(WHEEL_JOINTS)
        msg.effort = [0.0] * len(WHEEL_JOINTS)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = WheelJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
