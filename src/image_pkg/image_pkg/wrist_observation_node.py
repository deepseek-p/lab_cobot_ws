#!/usr/bin/env python3
"""Move the UR arm to a safe wrist-camera observation/calibration pose."""
from __future__ import annotations

import rclpy
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.node import Node
from std_msgs.msg import String


JOINTS = [
    "ur_shoulder_pan_joint", "ur_shoulder_lift_joint", "ur_elbow_joint",
    "ur_wrist_1_joint", "ur_wrist_2_joint", "ur_wrist_3_joint",
]
# Raised, forward-looking pose.  TCP +Z and wrist optical +Z are collinear.
OBSERVATION_POSE = [0.0, -1.57, 1.61, -1.57, -1.45, 0.0]


class WristObservation(Node):
    def __init__(self):
        super().__init__("wrist_observation")
        self.declare_parameter("trajectory_topic", "/joint_trajectory_controller/joint_trajectory")
        self.declare_parameter("duration_sec", 3.0)
        self.declare_parameter("joint_positions", OBSERVATION_POSE)
        self.pub = self.create_publisher(
            JointTrajectory, self.get_parameter("trajectory_topic").value, 10)
        self.status = self.create_publisher(String, "/image_pkg/wrist_observation/status", 10)
        self.timer = self.create_timer(0.5, self._send_once)
        self.sent = False

    def _send_once(self):
        if self.sent:
            return
        values = [float(v) for v in self.get_parameter("joint_positions").value]
        if len(values) != len(JOINTS):
            raise ValueError("joint_positions must contain six UR joint values")
        msg = JointTrajectory()
        msg.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = values
        seconds = float(self.get_parameter("duration_sec").value)
        point.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        msg.points = [point]
        self.pub.publish(msg)
        status = String(data=f"OBSERVATION_POSE_SENT:duration={seconds:.1f}s")
        self.status.publish(status)
        self.get_logger().info(status.data)
        self.sent = True


def main(args=None):
    rclpy.init(args=args)
    node = WristObservation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
