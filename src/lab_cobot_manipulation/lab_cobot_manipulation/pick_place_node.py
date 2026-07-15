#!/usr/bin/env python3
"""
Pick-and-place execution with MoveIt2 and the parallel gripper backend.

PickPlace 类供 mission_node 复用;mission_node 是跨工位任务的正式入口。
实际执行需运行时(move_group + Gazebo 控制器 + contact grasp 插件);
`sim_attach` 仅保留为显式调试后端。

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
    CONTACT_BACKEND,
    DEFAULT_TARGET_OBJECT,
    make_gripper_driver,
)
from lab_cobot_manipulation.refine_select import select_refined_position
from lab_cobot_manipulation.scene_obstacles import (
    CARRIED_SAMPLE_BOX_ID,
    PlanningSceneClient,
    STATION_SURFACE_BOX_ID,
    make_attach_scene,
    make_detach_scene,
    make_world_box_scene,
    station_surface_box,
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
OBSERVE_CONFIG = [
    -0.116421,
    -0.807952,
    0.425992,
    -1.337190,
    4.581185,
    -1.844921,
]
DEFAULT_APPROACH_HEIGHT = 0.04
DEFAULT_APPROACH_TOLERANCE_POSITION = 0.005
DEFAULT_APPROACH_TOLERANCE_ORIENTATION = 0.2
DEFAULT_GRASP_TOLERANCE_POSITION = 0.005
DEFAULT_GRASP_TOLERANCE_ORIENTATION = 0.2
TACTILE_GRASP_TOLERANCE_ORIENTATION = DEFAULT_GRASP_TOLERANCE_ORIENTATION
DEFAULT_MOVE_TIMEOUT_SEC = 45.0
MOVE_RESULT_GRACE_SEC = 2.0
PICK_TCP_Z_CLEARANCE = 0.06
TACTILE_PICK_TCP_Z_CLEARANCE = 0.0125
TACTILE_PICK_TCP_VISUAL_Y_LIMIT = 0.018
TACTILE_PICK_LATERAL_RETRY_STEP = 0.006
TACTILE_PICK_RETRY_Y_LIMIT = 0.036
TACTILE_APPROACH_HEIGHT = 0.130
# 悬空释放余量:place 下降只到名义放置点上方该高度即松爪,物块自由落到台面。
# 根因回归:带焊物块被压向台面时固定关节与接触约束冲突,求解器会给物块
# 注入巨大速度(实测弹飞 181 m/s);悬空释放从机制上避免该约束冲突。
PLACE_RELEASE_CLEARANCE = 0.02
TACTILE_PLACE_RELEASE_CLEARANCE = 0.025
# 2026-07-12 DG-2 双次实测持有偏移约 -15.5mm,相对旧标定 -65mm 抬高约 50mm。
TACTILE_PLACE_TCP_Z_COMPENSATION = 0.05
TACTILE_PLACE_DROP_SETTLE_SEC = 0.3
GRIPPER_CLOSE_SETTLE_SEC = 0.8
ARM_MAX_VELOCITY_SCALING = 0.75
ARM_MAX_ACCELERATION_SCALING = 0.75
TACTILE_ARM_MAX_VELOCITY_SCALING = 0.30
TACTILE_ARM_MAX_ACCELERATION_SCALING = 0.30
ARM_ALLOWED_PLANNING_TIME_SEC = 3.0
ARM_NUM_PLANNING_ATTEMPTS = 3
GO_HOME_MAX_ATTEMPTS = 2
GO_HOME_RETRY_DELAY_SEC = 0.2
APPROACH_MOVE_MAX_ATTEMPTS = 2
GRASP_DESCENT_MAX_ATTEMPTS = 2


def configure_moveit_for_pick_place(moveit2, use_tactile_grasp=False) -> None:
    if use_tactile_grasp:
        moveit2.max_velocity = TACTILE_ARM_MAX_VELOCITY_SCALING
        moveit2.max_acceleration = TACTILE_ARM_MAX_ACCELERATION_SCALING
    else:
        moveit2.max_velocity = ARM_MAX_VELOCITY_SCALING
        moveit2.max_acceleration = ARM_MAX_ACCELERATION_SCALING
    moveit2.allowed_planning_time = ARM_ALLOWED_PLANNING_TIME_SEC
    moveit2.num_planning_attempts = ARM_NUM_PLANNING_ATTEMPTS


def _format_pose_target(pos) -> str:
    return "(%.3f, %.3f, %.3f)" % (float(pos[0]), float(pos[1]), float(pos[2]))


class PickPlace(Node):
    def __init__(
        self,
        target_object: str = DEFAULT_TARGET_OBJECT,
        use_tactile_grasp: bool = False,
        use_planning_scene_obstacles: bool = True,
    ):
        super().__init__("pick_place_node")
        self.declare_parameter("approach_height", DEFAULT_APPROACH_HEIGHT)
        self.declare_parameter("gripper_backend", CONTACT_BACKEND)
        self.declare_parameter("target_object", str(target_object))
        self.declare_parameter("use_tactile_grasp", bool(use_tactile_grasp))
        self.declare_parameter(
            "use_planning_scene_obstacles", bool(use_planning_scene_obstacles)
        )
        self.approach_height = float(self.get_parameter("approach_height").value)
        self.gripper_backend = str(self.get_parameter("gripper_backend").value)
        self.target_object = str(self.get_parameter("target_object").value)
        self.use_tactile_grasp = bool(
            self.get_parameter("use_tactile_grasp").value
        )
        self.use_planning_scene_obstacles = bool(
            self.get_parameter("use_planning_scene_obstacles").value
        )

        cb = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=UR_JOINTS,
            base_link_name="ur_base_link",
            end_effector_name="ur_tool0",
            group_name="ur_manipulator",
            callback_group=cb,
        )
        configure_moveit_for_pick_place(
            self.moveit2,
            use_tactile_grasp=self.use_tactile_grasp,
        )
        self.gripper = make_gripper_driver(
            self,
            backend=self.gripper_backend,
            command_settle_sec=GRIPPER_CLOSE_SETTLE_SEC,
            target_object=self.target_object,
            use_tactile_grasp=self.use_tactile_grasp,
        )
        # 规划场景障碍注入客户端;禁用时为 None,全部调用点走降级路径。
        self.scene_client = (
            PlanningSceneClient(self)
            if self.use_planning_scene_obstacles
            else None
        )
        self.get_logger().info("PickPlace 初始化完成")

    # ---- 规划场景障碍(台面盒/持物样件附着盒) ----
    def _apply_scene_diff(self, scene, label) -> bool:
        if self.scene_client is None:
            return False
        if self.scene_client.apply(scene):
            return True
        # 注入失败只降级回旧行为(规划对环境盲),不阻断抓取任务。
        self.get_logger().warn(f"planning scene {label} apply failed")
        return False

    def _inject_station_surface(self, pos) -> None:
        if self.scene_client is None:
            return
        scene = make_world_box_scene(
            STATION_SURFACE_BOX_ID, station_surface_box(pos), "base_link"
        )
        if self._apply_scene_diff(scene, "surface add"):
            self.get_logger().info("planning scene surface box injected")

    def _attach_carried_sample(self) -> None:
        if self.scene_client is None:
            return
        scene = make_attach_scene(CARRIED_SAMPLE_BOX_ID)
        if self._apply_scene_diff(scene, "sample attach"):
            self.get_logger().info("planning scene carried sample attached")

    def _detach_carried_sample(self) -> None:
        if self.scene_client is None:
            return
        scene = make_detach_scene(CARRIED_SAMPLE_BOX_ID)
        if self._apply_scene_diff(scene, "sample detach"):
            self.get_logger().info("planning scene carried sample detached")

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
        cartesian=False,
    ) -> bool:
        # frame_id="base_link":pos 为底盘系坐标(感知/放置点都在此系),
        # 由 MoveIt 用 planning scene 的 TF 换算到规划基座 ur_base_link。
        # 不显式指定则默认按 ur_base_link 解释,会把目标整体抬高一个立柱高度→抓空。
        self.get_logger().info(
            "MoveIt target link=%s frame=%s pos=%s"
            % (target_link, frame_id, _format_pose_target(pos))
        )
        self.moveit2.move_to_pose(
            position=list(pos),
            quat_xyzw=quat,
            frame_id=frame_id,
            target_link=target_link,
            tolerance_position=tolerance_position,
            tolerance_orientation=tolerance_orientation,
            cartesian=cartesian,
        )
        executed = bool(self.moveit2.wait_until_executed(timeout_sec=timeout_sec))
        if not executed and MOVE_RESULT_GRACE_SEC > 0.0:
            time.sleep(MOVE_RESULT_GRACE_SEC)
            executed = bool(
                self.moveit2.wait_until_executed(timeout_sec=MOVE_RESULT_GRACE_SEC)
            )
        if not executed:
            self.get_logger().warn(
                "MoveIt target failed link=%s frame=%s pos=%s"
                % (target_link, frame_id, _format_pose_target(pos))
            )
        return executed

    def _pick_tcp_target(self, pos):
        clearance = PICK_TCP_Z_CLEARANCE
        y_target = pos[1]
        if self.use_tactile_grasp:
            clearance = TACTILE_PICK_TCP_Z_CLEARANCE
            limit = TACTILE_PICK_TCP_VISUAL_Y_LIMIT
            y_target = min(max(pos[1], -limit), limit)
        return [pos[0], y_target, pos[2] + clearance]

    def _offset_tactile_pick_target(self, target, lateral_offset):
        adjusted = list(target)
        y_target = target[1] + lateral_offset
        adjusted[1] = min(
            max(y_target, -TACTILE_PICK_RETRY_Y_LIMIT),
            TACTILE_PICK_RETRY_Y_LIMIT,
        )
        return adjusted

    def _tactile_pick_lateral_retry_offsets(self, target_y=0.0):
        step = TACTILE_PICK_LATERAL_RETRY_STEP

        def dedupe_clamped(offsets):
            seen = {round(float(target_y), 6)}
            result = []
            for offset in offsets:
                adjusted = min(
                    max(target_y + offset, -TACTILE_PICK_RETRY_Y_LIMIT),
                    TACTILE_PICK_RETRY_Y_LIMIT,
                )
                key = round(float(adjusted), 6)
                if key in seen:
                    continue
                seen.add(key)
                result.append(offset)
            return result

        left_touch, right_touch = self.gripper.last_tactile_contact_sides()
        if left_touch and not right_touch:
            return dedupe_clamped([-step, -2.0 * step, step, 2.0 * step])
        if right_touch and not left_touch:
            return dedupe_clamped([step, 2.0 * step, -step, -2.0 * step])
        if target_y > step * 0.5:
            return dedupe_clamped([-step, step])
        if target_y < -step * 0.5:
            return dedupe_clamped([step, -step])
        return dedupe_clamped([step, -step])

    def _pick_approach_target(self, target):
        approach_height = self.approach_height
        if self.use_tactile_grasp:
            approach_height = max(approach_height, TACTILE_APPROACH_HEIGHT)
        return [
            target[0],
            target[1],
            target[2] + approach_height,
        ]

    def _pick_lift_target(self, target):
        approach_height = self.approach_height
        if self.use_tactile_grasp:
            approach_height = max(approach_height, TACTILE_APPROACH_HEIGHT)
        return [
            target[0],
            target[1],
            target[2] + approach_height,
        ]

    def _move_approach(self, pos, cartesian=False) -> bool:
        for attempt in range(APPROACH_MOVE_MAX_ATTEMPTS):
            if self._move(
                pos,
                target_link=GRIPPER_TCP_LINK,
                tolerance_position=DEFAULT_APPROACH_TOLERANCE_POSITION,
                tolerance_orientation=DEFAULT_APPROACH_TOLERANCE_ORIENTATION,
                timeout_sec=DEFAULT_MOVE_TIMEOUT_SEC,
                cartesian=cartesian,
            ):
                return True
            if attempt + 1 < APPROACH_MOVE_MAX_ATTEMPTS:
                self.get_logger().warn(
                    "MoveIt approach transient failure, retrying"
                )
                time.sleep(GO_HOME_RETRY_DELAY_SEC)
        return False

    def _move_grasp_descent(self, target) -> bool:
        for attempt in range(GRASP_DESCENT_MAX_ATTEMPTS):
            if self._move(
                target,
                target_link=GRIPPER_TCP_LINK,
                tolerance_position=DEFAULT_GRASP_TOLERANCE_POSITION,
                tolerance_orientation=(
                    TACTILE_GRASP_TOLERANCE_ORIENTATION
                    if self.use_tactile_grasp
                    else DEFAULT_GRASP_TOLERANCE_ORIENTATION
                ),
                timeout_sec=DEFAULT_MOVE_TIMEOUT_SEC,
                cartesian=True,
            ):
                return True
            if attempt + 1 < GRASP_DESCENT_MAX_ATTEMPTS:
                self.get_logger().warn(
                    "MoveIt grasp descent transient failure, retrying"
                )
                time.sleep(GO_HOME_RETRY_DELAY_SEC)
        return False

    # ---- 复合动作(供 mission 调用)----
    def pick(self, pos, refine_cb=None) -> bool:
        """Open → approach → descend → validate/attach → close → lift."""
        # 台面盒先于第一次臂规划注入,否则 approach 弧仍对台面盲。
        self._inject_station_surface(pos)
        target = self._pick_tcp_target(pos)
        retry_offsets = [0.0]
        retry_offsets_extended = False
        for retry_index, lateral_offset in enumerate(retry_offsets):
            attempt_target = target
            if self.use_tactile_grasp:
                attempt_target = self._offset_tactile_pick_target(
                    target, lateral_offset
                )
            result = self._pick_once(
                pos,
                attempt_target,
                refine_cb=refine_cb,
                lateral_offset=lateral_offset,
            )
            if result == "ok":
                return True
            if result != "acquire_failed" or not self.use_tactile_grasp:
                return False
            if not retry_offsets_extended:
                retry_offsets.extend(
                    self._tactile_pick_lateral_retry_offsets(target[1])
                )
                retry_offsets_extended = True
            if retry_index + 1 >= len(retry_offsets):
                return False
        return False

    def _pick_once(self, pos, target, refine_cb=None, lateral_offset=0.0) -> str:
        above = self._pick_approach_target(target)
        self.get_logger().info(
            "Pick start detected=%s target=%s approach=%s backend=%s"
            % (
                _format_pose_target(pos),
                _format_pose_target(target),
                _format_pose_target(above),
                self.gripper_backend,
            )
        )
        if not self.gripper.open():
            self.get_logger().warn("Pick failed: gripper open rejected")
            return "failed"
        if not self._move_approach(above):
            self.get_logger().warn("Pick failed: approach move failed")
            return "failed"
        if refine_cb is not None:
            try:
                refined = refine_cb()
                selected, used_refine, reason = select_refined_position(
                    pos,
                    refined,
                )
            except Exception:  # noqa: BLE001
                selected = list(pos)
                used_refine = False
                reason = "callback_exception"
            if used_refine:
                self.get_logger().info(
                    "refine=hit dx=%.3f dy=%.3f dz=%.3f"
                    % (
                        selected[0] - pos[0],
                        selected[1] - pos[1],
                        selected[2] - pos[2],
                    )
                )
            else:
                self.get_logger().info(f"refine=miss({reason})")
            target = self._pick_tcp_target(selected)
            if self.use_tactile_grasp:
                target = self._offset_tactile_pick_target(
                    target,
                    lateral_offset,
                )
        lift = self._pick_lift_target(target)
        # 下降段笛卡尔直线:关节空间规划的横向弧会扫飞轻质物块(实测)
        if not self._move_grasp_descent(target):
            self.get_logger().warn("Pick failed: grasp descent failed")
            return "failed"
        if not self.gripper.acquire_object():
            if self.use_tactile_grasp:
                self.gripper.open()
                self._move_approach(above, cartesian=True)
            self.get_logger().warn("Pick failed: object acquire rejected")
            return "acquire_failed"
        # 持物段规划护航:样件附着盒随 acquire 成功立即挂上。
        self._attach_carried_sample()
        if not self.use_tactile_grasp and not self.gripper.close():
            self.gripper.release_object()
            self._detach_carried_sample()
            self.get_logger().warn("Pick failed: gripper close rejected")
            return "failed"
        if not self._move_approach(lift, cartesian=True):
            self.gripper.release_object()
            self._detach_carried_sample()
            self.get_logger().warn("Pick failed: lift move failed")
            return "failed"
        self.get_logger().info("Pick complete")
        return "ok"

    def place(self, pos) -> bool:
        """Approach → descend to release height → detach/open → lift."""
        # 下降只到 pos.z + PLACE_RELEASE_CLEARANCE 即释放(悬空释放),
        # 物块自由落到台面,避免带焊下压引发固定关节 vs 接触约束冲突。
        target = list(pos)
        # B 台盒同样先于 approach 注入(place_pose 只取 xy,z 为常量台顶)。
        self._inject_station_surface(pos)
        release_clearance = PLACE_RELEASE_CLEARANCE
        tcp_z_compensation = 0.0
        if self.use_tactile_grasp:
            release_clearance = TACTILE_PLACE_RELEASE_CLEARANCE
            tcp_z_compensation = TACTILE_PLACE_TCP_Z_COMPENSATION
        release = [
            target[0],
            target[1],
            target[2] + release_clearance - tcp_z_compensation,
        ]
        above = [target[0], target[1], target[2] + self.approach_height]
        self.get_logger().info(
            "Place start target=%s release=%s approach=%s"
            % (
                _format_pose_target(target),
                _format_pose_target(release),
                _format_pose_target(above),
            )
        )
        if not self._move_approach(above, cartesian=self.use_tactile_grasp):
            self.get_logger().warn("Place failed: approach move failed")
            return False
        # 持物下降段笛卡尔直线:横向弧会带着焊接物块扫掠台面(同 pick 根因)
        if not self._move(
            release,
            target_link=GRIPPER_TCP_LINK,
            tolerance_position=DEFAULT_GRASP_TOLERANCE_POSITION,
            tolerance_orientation=DEFAULT_GRASP_TOLERANCE_ORIENTATION,
            timeout_sec=DEFAULT_MOVE_TIMEOUT_SEC,
            cartesian=True,
        ):
            self.get_logger().warn("Place failed: descent move failed")
            return False
        if not self.gripper.release_object():
            self.get_logger().warn("Place failed: release rejected")
            return False
        self._detach_carried_sample()
        if self.use_tactile_grasp and TACTILE_PLACE_DROP_SETTLE_SEC > 0.0:
            time.sleep(TACTILE_PLACE_DROP_SETTLE_SEC)
        if not self.gripper.open():
            self.get_logger().warn("Place failed: gripper open rejected")
            return False
        lifted = self._move_approach(above, cartesian=True)
        if not lifted:
            self.get_logger().warn("Place failed: lift move failed")
            return False
        self.get_logger().info("Place complete")
        return True

    def go_home(self) -> bool:
        for attempt in range(GO_HOME_MAX_ATTEMPTS):
            self.moveit2.move_to_configuration(HOME_CONFIG)
            if self.moveit2.wait_until_executed(timeout_sec=DEFAULT_MOVE_TIMEOUT_SEC):
                return True
            if attempt + 1 < GO_HOME_MAX_ATTEMPTS:
                time.sleep(GO_HOME_RETRY_DELAY_SEC)
        return False

    def move_to_observe(self) -> bool:
        """Move the arm to the fixed wrist-camera observation configuration."""
        for attempt in range(GO_HOME_MAX_ATTEMPTS):
            self.moveit2.move_to_configuration(OBSERVE_CONFIG)
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
    node.get_logger().info("pick_place_node 就绪")
    try:
        while rclpy.ok():
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
