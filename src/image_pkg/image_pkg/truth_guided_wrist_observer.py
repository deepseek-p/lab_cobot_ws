#!/usr/bin/env python3
"""Move the wrist camera above one Gazebo-truth object using MoveIt IK."""
import math
from threading import Thread
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from pymoveit2 import MoveIt2

JOINTS = ["ur_shoulder_pan_joint", "ur_shoulder_lift_joint", "ur_elbow_joint",
          "ur_wrist_1_joint", "ur_wrist_2_joint", "ur_wrist_3_joint"]

class TruthGuidedObserver(Node):
    def __init__(self):
        super().__init__("truth_guided_wrist_observer")
        self.declare_parameter("target_model", "aruco_sample")
        self.declare_parameter("view_height_m", 0.35)
        self.odom = None; self.models = {}
        self.create_subscription(Odometry, "/odom", self._odom, 10)
        self.create_subscription(ModelStates, "/gazebo/model_states", self._models, 10)
        self.cb = ReentrantCallbackGroup()
        self.moveit = MoveIt2(self, JOINTS, "base_link", "ur_tool0", "ur_manipulator", self.cb)
        self.timer = self.create_timer(0.2, self._go)
        self.done = False
    def _odom(self, m): self.odom = m
    def _models(self, m): self.models = {n:p for n,p in zip(m.name,m.pose)}
    def _go(self):
        if self.done or self.odom is None: return
        model = self.models.get(str(self.get_parameter("target_model").value))
        if model is None: return
        q = self.odom.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        dx, dy = model.position.x-self.odom.pose.pose.position.x, model.position.y-self.odom.pose.pose.position.y
        # Object point in base coordinates; optical axis points vertically down.
        x = math.cos(yaw)*dx + math.sin(yaw)*dy
        y = -math.sin(yaw)*dx + math.cos(yaw)*dy
        z = model.position.z + float(self.get_parameter("view_height_m").value)
        self.get_logger().info(f"VIEW:{model.position.x:.3f},{model.position.y:.3f},{model.position.z:.3f} -> base {x:.3f},{y:.3f},{z:.3f}")
        self.moveit.move_to_pose(position=[x,y,z], quat_xyzw=[1.0,0.0,0.0,0.0], cartesian=False)
        self.done = True

def main(args=None):
    rclpy.init(args=args); n=TruthGuidedObserver()
    ex=rclpy.executors.MultiThreadedExecutor(2); ex.add_node(n)
    t=Thread(target=ex.spin,daemon=True); t.start()
    try:
        while rclpy.ok() and not n.done: rclpy.spin_once(n,timeout_sec=.1)
        if n.done: n.moveit.wait_until_executed()
    finally: n.destroy_node(); rclpy.shutdown()
if __name__ == "__main__": main()
