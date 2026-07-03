#!/usr/bin/env python3
"""
Pick-and-place execution with MoveIt2 and the parallel gripper backend.

PickPlace 类供 mission_node 复用;main() 提供独立节点(订阅简单指令做冒烟测试)。
实际执行需运行时(move_group + Gazebo 控制器 + gripper attach bridge)。

约定:
- 关节 ur_ 前缀,规划组 ur_manipulator,基座 ur_base_link,末端 ur_tool0
- pose 为 base_link(底盘)系下的 gripper_tcp 目标;本类负责换算 ur_tool0 目标
"""
import time
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from lab_cobot_manipulation.gripper_driver import (
    SimAttachGripperDriver,
)
from pymoveit2 import MoveIt2

UR_JOINTS = [
    "ur_shoulder_pan_joint", "ur_shoulder_lift_joint", "ur_elbow_joint",
    "ur_wrist_1_joint", "ur_wrist_2_joint", "ur_wrist_3_joint",
]
# 末端朝下抓取的姿态(绕 x 轴 180°,使 tool0 z 轴朝下);运行时按实际标定微调
DOWN_QUAT = [1.0, 0.0, 0.0, 0.0]
GRIPPER_TCP_LINK = "gripper_tcp"
HOME_CONFIG = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
DEFAULT_APPROACH_HEIGHT = 0.06
DEFAULT_APPROACH_TOLERANCE_POSITION = 0.005
DEFAULT_APPROACH_TOLERANCE_ORIENTATION = 0.2
DEFAULT_GRASP_TOLERANCE_ORIENTATION = 0.05
DEFAULT_MOVE_TIMEOUT_SEC = 45.0
PICK_TCP_Z_CLEARANCE = 0.02
GRIPPER_CLOSE_SETTLE_SEC = 0.8
ARM_MAX_VELOCITY_SCALING = 0.75
ARM_MAX_ACCELERATION_SCALING = 0.75
ARM_ALLOWED_PLANNING_TIME_SEC = 3.0
ARM_NUM_PLANNING_ATTEMPTS = 3
GO_HOME_MAX_ATTEMPTS = 2
GO_HOME_RETRY_DELAY_SEC = 0.2


def configure_moveit_for_pick_place(moveit2) -> None:
    moveit2.max_velocity = ARM_MAX_VELOCITY_SCALING
    moveit2.max_acceleration = ARM_MAX_ACCELERATION_SCALING
    moveit2.allowed_planning_time = ARM_ALLOWED_PLANNING_TIME_SEC
    moveit2.num_planning_attempts = ARM_NUM_PLANNING_ATTEMPTS


class PickPlace(Node):
    def __init__(self):
        super().__init__("pick_place_node")
        self.declare_parameter("approach_height", DEFAULT_APPROACH_HEIGHT)
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
        configure_moveit_for_pick_place(self.moveit2)
        self.gripper = SimAttachGripperDriver(
            self,
            command_settle_sec=GRIPPER_CLOSE_SETTLE_SEC,
        )
        self.get_logger().info("PickPlace 初始化完成")

    # ---- 基础动作 ----
    def _move(
        self,
        pos,
        quat=DOWN_QUAT,
        frame_id="base_link",
        target_link=GRIPPER_TCP_LINK,
        tolerance_position=0.001,
        tolerance_orientation=0.001,
        timeout_sec=DEFAULT_MOVE_TIMEOUT_SEC,
    ) -> bool:
        # frame_id="base_link":pos 为底盘系坐标(感知/放置点都在此系),
        # 由 MoveIt 用 planning scene 的 TF 换算到规划基座 ur_base_link。
        # 不显式指定则默认按 ur_base_link 解释,会把目标整体抬高一个立柱高度→抓空。
        self.moveit2.move_to_pose(
            position=list(pos),
            quat_xyzw=quat,
            frame_id=frame_id,
            target_link=target_link,
            tolerance_position=tolerance_position,
            tolerance_orientation=tolerance_orientation,
        )
        return bool(self.moveit2.wait_until_executed(timeout_sec=timeout_sec))

    def _pick_tcp_target(self, pos):
        return [pos[0], pos[1], pos[2] + PICK_TCP_Z_CLEARANCE]

    def _move_approach(self, pos) -> bool:
        return self._move(
            pos,
            target_link=GRIPPER_TCP_LINK,
            tolerance_position=DEFAULT_APPROACH_TOLERANCE_POSITION,
            tolerance_orientation=DEFAULT_APPROACH_TOLERANCE_ORIENTATION,
            timeout_sec=DEFAULT_MOVE_TIMEOUT_SEC,
        )

    # ---- 复合动作(供 mission 调用)----
    def pick(self, pos) -> bool:
        """Open → approach → descend → validate/attach → close → lift."""
        target = self._pick_tcp_target(pos)
        above = [target[0], target[1], target[2] + self.approach_height]
        if not self.gripper.open():
            return False
        if not self._move_approach(above):
            return False
        if not self._move(
            target,
            target_link=GRIPPER_TCP_LINK,
            tolerance_orientation=DEFAULT_GRASP_TOLERANCE_ORIENTATION,
            timeout_sec=DEFAULT_MOVE_TIMEOUT_SEC,
        ):
            return False
        if not self.gripper.acquire_object():
            return False
        if not self.gripper.close():
            return False
        return self._move_approach(above)

    def place(self, pos) -> bool:
        """Approach → descend → detach/open → lift."""
        target = list(pos)
        above = [target[0], target[1], target[2] + self.approach_height]
        if not self._move_approach(above):
            return False
        if not self._move(
            target,
            target_link=GRIPPER_TCP_LINK,
            tolerance_orientation=DEFAULT_GRASP_TOLERANCE_ORIENTATION,
            timeout_sec=DEFAULT_MOVE_TIMEOUT_SEC,
        ):
            return False
        if not self.gripper.release_object():
            return False
        if not self.gripper.open():
            return False
        return self._move_approach(above)

    def go_home(self) -> bool:
        for attempt in range(GO_HOME_MAX_ATTEMPTS):
            self.moveit2.move_to_configuration(HOME_CONFIG)
            if self.moveit2.wait_until_executed(timeout_sec=DEFAULT_MOVE_TIMEOUT_SEC):
                return True
            if attempt + 1 < GO_HOME_MAX_ATTEMPTS:
                time.sleep(GO_HOME_RETRY_DELAY_SEC)
        return False


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
