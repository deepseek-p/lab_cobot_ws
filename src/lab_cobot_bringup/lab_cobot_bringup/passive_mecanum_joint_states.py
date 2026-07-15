#!/usr/bin/env python3
"""Publish neutral states for unactuated mecanum suspension and roller joints."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def passive_joint_names():
    wheels = ("front_left", "front_right", "back_left", "back_right")
    rollers = [
        f"{wheel}_barrel_{index}_joint"
        for wheel in wheels
        for index in range(15)
    ]
    return rollers


def shutdown_if_running():
    if rclpy.ok():
        rclpy.shutdown()


class PassiveMecanumJointStates(Node):
    def __init__(self):
        super().__init__("passive_mecanum_joint_states")
        self._names = passive_joint_names()
        self._publisher = self.create_publisher(JointState, "/joint_states", 10)
        self._timer = self.create_timer(0.05, self._publish)

    def _publish(self):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = self._names
        message.position = [0.0] * len(self._names)
        self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = PassiveMecanumJointStates()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        shutdown_if_running()


if __name__ == "__main__":
    main()
