#!/usr/bin/env python3
"""抓取/放置执行:pymoveit2 规划 UR5e 运动 + 真空吸盘吸附。

PickPlace 类供 mission_node 复用;main() 提供独立节点(订阅简单指令做冒烟测试)。
实际执行需运行时(move_group + Gazebo 控制器 + /suction/switch service)。

约定:
- 关节 ur_ 前缀,规划组 ur_manipulator,基座 ur_base_link,末端 ur_tool0
- pose 为 base_link(底盘)系下的目标;调用方负责把感知 TF 转到该系
"""
import time
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import SetBool

from pymoveit2 import MoveIt2

UR_JOINTS = [
    "ur_shoulder_pan_joint", "ur_shoulder_lift_joint", "ur_elbow_joint",
    "ur_wrist_1_joint", "ur_wrist_2_joint", "ur_wrist_3_joint",
]
# 末端朝下抓取的姿态(绕 x 轴 180°,使 tool0 z 轴朝下);运行时按实际标定微调
DOWN_QUAT = [1.0, 0.0, 0.0, 0.0]
HOME_CONFIG = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]


class PickPlace(Node):
    def __init__(self):
        super().__init__("pick_place_node")
        self.declare_parameter("approach_height", 0.12)
        self.approach_height = float(self.get_parameter("approach_height").value)

        cb = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=UR_JOINTS,
            base_link_name="ur_base_link",
            end_effector_name="ur_tool0",
            group_name="ur_manipulator",
            callback_group=cb,
        )
        self.moveit2.max_velocity = 0.3
        self.moveit2.max_acceleration = 0.3
        self.suction_cli = self.create_client(SetBool, "/suction/switch")
        self.get_logger().info("PickPlace 初始化完成")

    # ---- 基础动作 ----
    def _suction(self, on: bool) -> bool:
        if not self.suction_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("真空 service /suction/switch 不可用")
            return False
        req = SetBool.Request()
        req.data = on
        fut = self.suction_cli.call_async(req)
        # 靠外部 MultiThreadedExecutor 后台 spin 完成 future(避免双重 spin)
        t0 = time.time()
        while not fut.done() and (time.time() - t0) < 3.0:
            time.sleep(0.05)
        ok = fut.done() and fut.result() is not None and fut.result().success
        self.get_logger().info(f"真空 {'开' if on else '关'}: {'成功' if ok else '失败'}")
        return ok

    def _move(self, pos, quat=DOWN_QUAT, frame_id="base_link") -> bool:
        # frame_id="base_link":pos 为底盘系坐标(感知/放置点都在此系),
        # 由 MoveIt 用 planning scene 的 TF 换算到规划基座 ur_base_link。
        # 不显式指定则默认按 ur_base_link 解释,会把目标整体抬高一个立柱高度→抓空。
        self.moveit2.move_to_pose(position=list(pos), quat_xyzw=quat, frame_id=frame_id)
        return bool(self.moveit2.wait_until_executed())

    # ---- 复合动作(供 mission 调用)----
    def pick(self, pos) -> bool:
        """approach 上方 → 下降 → 吸附 → 抬起。pos=[x,y,z] in base_link。"""
        above = [pos[0], pos[1], pos[2] + self.approach_height]
        if not self._move(above):
            return False
        if not self._move(list(pos)):
            return False
        self._suction(True)
        return self._move(above)

    def place(self, pos) -> bool:
        """移动到放置点上方 → 下降 → 释放 → 抬起。"""
        above = [pos[0], pos[1], pos[2] + self.approach_height]
        if not self._move(above):
            return False
        if not self._move(list(pos)):
            return False
        self._suction(False)
        return self._move(above)

    def go_home(self) -> bool:
        self.moveit2.move_to_configuration(HOME_CONFIG)
        return bool(self.moveit2.wait_until_executed())


def main():
    rclpy.init()
    node = PickPlace()
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    Thread(target=executor.spin, daemon=True).start()
    node.get_logger().info("pick_place_node 就绪(等待 mission 调用或手动测试)")
    try:
        while rclpy.ok():
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
