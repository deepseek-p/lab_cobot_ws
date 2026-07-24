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
from action_msgs.msg import GoalStatus
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
    DYNAMIC_ARM_OBSTACLE_BOX_ID,
    PlanningSceneClient,
    STATION_SURFACE_BOX_ID,
    make_attach_scene,
    make_detach_scene,
    make_dynamic_obstacle_scene,
    make_remove_dynamic_obstacle_scene,
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
# Gazebo 在低实时因子（深度相机、MoveIt、接触传感器同时运行）下，首条
# approach 轨迹实测可到 166 秒墙钟。若此时取消 action，后续重试会与旧目标竞争。
# 240 秒仍为有界等待，并为 WSL 调度抖动保留充足余量。
DEFAULT_MOVE_TIMEOUT_SEC = 240.0
MOVE_RESULT_GRACE_SEC = 5.0
PICK_TCP_Z_CLEARANCE = 0.06
TACTILE_PICK_TCP_Z_CLEARANCE = 0.0125
TACTILE_PICK_TCP_VISUAL_Y_LIMIT = 0.018
TACTILE_PICK_LATERAL_BIAS = 0.006
TACTILE_PICK_LATERAL_RETRY_STEP = 0.006
TACTILE_PICK_RETRY_Y_LIMIT = 0.036
TACTILE_APPROACH_HEIGHT = 0.060
# 悬空释放余量:place 下降只到名义放置点上方该高度即松爪,物块自由落到台面。
# 根因回归:带焊物块被压向台面时固定关节与接触约束冲突,求解器会给物块
# 注入巨大速度(实测弹飞 181 m/s);悬空释放从机制上避免该约束冲突。
PLACE_RELEASE_CLEARANCE = 0.02
TACTILE_PLACE_RELEASE_CLEARANCE = 0.025
# G4 A->B 放置使用 base_link 坐标时，额外下压补偿会把释放 TCP 拉到
# 目标点下方，容易触发 MoveIt 下降段执行失败；触觉放置仍采用上方悬空释放。
TACTILE_PLACE_TCP_Z_COMPENSATION = 0.0
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
HOLD_MONITOR_PERIOD_SEC = 0.1


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


def _wait_for_moveit_result(
    moveit2,
    timeout_sec: float,
    previous_send_future=None,
    previous_result_future=None,
) -> bool:
    """Wait for MoveIt's action future without depending on a ROS-time Rate.

    pymoveit2.wait_until_executed() sleeps on a Rate created from the node's
    clock.  In a throttled Gazebo simulation that sleep can mask a completed
    action result until the caller's wall-clock timeout.  Recent pymoveit2
    versions retain the result future internally, so poll that future using
    wall time.  Keep the public method as a compatibility fallback for test
    doubles and older pymoveit2 releases.
    """
    send_name = "_MoveIt2__send_goal_future_move_action"
    result_name = "_MoveIt2__get_result_future_move_action"
    state_name = "_MoveIt2__last_execution_succeeded"
    if not hasattr(moveit2, send_name):
        return bool(moveit2.wait_until_executed(timeout_sec=timeout_sec))

    deadline = time.monotonic() + max(float(timeout_sec), 0.0)
    seen_current_goal = previous_send_future is None
    while time.monotonic() < deadline:
        send_future = getattr(moveit2, send_name, None)
        if send_future is not None and send_future is not previous_send_future:
            seen_current_goal = True

        result_future = getattr(moveit2, result_name, None)
        if (
            seen_current_goal
            and result_future is not None
            and result_future is not previous_result_future
            and result_future.done()
        ):
            try:
                return result_future.result().status == GoalStatus.STATUS_SUCCEEDED
            except Exception:  # noqa: BLE001
                return False

        if (
            seen_current_goal
            and send_future is not None
            and hasattr(send_future, "done")
            and send_future.done()
        ):
            # A rejected goal does not have a result future.  The pymoveit2
            # response callback has already recorded the final status.
            if getattr(moveit2, result_name, None) is None and not getattr(
                moveit2, "_MoveIt2__is_motion_requested", False
            ):
                return bool(getattr(moveit2, state_name, False))
        if (
            seen_current_goal
            and not getattr(moveit2, "_MoveIt2__is_motion_requested", False)
            and not getattr(moveit2, "_MoveIt2__is_executing", False)
            and (
                previous_send_future is None
                or getattr(moveit2, result_name, None) is not previous_result_future
            )
        ):
            return bool(getattr(moveit2, state_name, False))
        time.sleep(0.01)

    goal_handle = getattr(moveit2, "_MoveIt2__move_goal_handle", None)
    if goal_handle is not None:
        goal_handle.cancel_goal_async()
    reset = getattr(moveit2, "force_reset_executing_state", None)
    if reset is not None:
        reset()
    return False


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
        self._hold_monitor_active = False
        self._hold_monitor_fault = False
        self._hold_monitor_timer = self.create_timer(
            HOLD_MONITOR_PERIOD_SEC, self._monitor_held_object
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

    def update_dynamic_arm_obstacle(
        self,
        center,
        size,
        frame_id="base_link",
        object_id=DYNAMIC_ARM_OBSTACLE_BOX_ID,
    ) -> bool:
        """Add or update one dynamic obstacle used by arm planning."""
        if self.scene_client is None:
            return False
        scene = make_dynamic_obstacle_scene(
            center,
            size,
            frame_id=frame_id,
            object_id=object_id,
        )
        ok = self._apply_scene_diff(scene, "dynamic obstacle update")
        if ok:
            self.get_logger().info(
                "planning scene dynamic obstacle updated id=%s frame=%s"
                % (object_id, frame_id)
            )
        return ok

    def clear_dynamic_arm_obstacle(
        self,
        object_id=DYNAMIC_ARM_OBSTACLE_BOX_ID,
    ) -> bool:
        """Remove the dynamic obstacle used by arm planning."""
        if self.scene_client is None:
            return False
        scene = make_remove_dynamic_obstacle_scene(object_id)
        ok = self._apply_scene_diff(scene, "dynamic obstacle remove")
        if ok:
            self.get_logger().info(
                "planning scene dynamic obstacle removed id=%s" % object_id
            )
        return ok

    # ---- 持有监控(抓取插件 heartbeat) ----
    def _gripper_confirms_holding(self) -> bool:
        checker = getattr(self.gripper, "is_holding_object", None)
        # 兼容第三方/旧调试后端；正式 contact 后端必须实现实时确认。
        if checker is None:
            return True
        try:
            return bool(checker())
        except Exception:  # noqa: BLE001
            return False

    def _start_hold_monitor(self) -> None:
        refresh = getattr(self.gripper, "refresh_holding_watchdog", None)
        if refresh is not None:
            refresh()
        self._hold_monitor_active = True
        self._hold_monitor_fault = False
        self._monitor_held_object()

    def _stop_hold_monitor(self) -> None:
        self._hold_monitor_active = False

    def _monitor_held_object(self) -> None:
        """Run while carrying; the callback executor keeps this alive during moves."""
        if not getattr(self, "_hold_monitor_active", False):
            return
        if self._gripper_confirms_holding():
            return
        if not self._hold_monitor_fault:
            self._hold_monitor_fault = True
            self.get_logger().error(
                "持有监控失败：抓取插件未确认物块仍附着，终止后续抓放步骤"
            )

    def _holding_is_healthy(self) -> bool:
        # place() 的单独单元测试/旧调试入口可能没有先执行 pick()；只有
        # 已进入持物段时才要求心跳，正式 mission 路径会在 pick 后激活它。
        if not getattr(self, "_hold_monitor_active", False):
            return True
        self._monitor_held_object()
        return (
            not getattr(self, "_hold_monitor_fault", False)
        )

    def _handle_hold_lost(self) -> None:
        """Remove the planning-scene carried box after an externally detected loss."""
        if not getattr(self, "_hold_monitor_fault", False):
            return
        self._stop_hold_monitor()
        self._detach_carried_sample()
        self.get_logger().error("抓放任务失败：物块在搬运期间不再被持有")

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
        send_name = "_MoveIt2__send_goal_future_move_action"
        result_name = "_MoveIt2__get_result_future_move_action"
        previous_send_future = getattr(self.moveit2, send_name, None)
        previous_result_future = getattr(self.moveit2, result_name, None)
        self.moveit2.move_to_pose(
            position=list(pos),
            quat_xyzw=quat,
            frame_id=frame_id,
            target_link=target_link,
            tolerance_position=tolerance_position,
            tolerance_orientation=tolerance_orientation,
            cartesian=cartesian,
        )
        executed = _wait_for_moveit_result(
            self.moveit2,
            timeout_sec,
            previous_send_future=previous_send_future,
            previous_result_future=previous_result_future,
        )
        if not executed and MOVE_RESULT_GRACE_SEC > 0.0:
            time.sleep(MOVE_RESULT_GRACE_SEC)
            executed = _wait_for_moveit_result(
                self.moveit2,
                MOVE_RESULT_GRACE_SEC,
                previous_send_future=previous_send_future,
                previous_result_future=previous_result_future,
            )
        if not executed:
            self.get_logger().warn(
                "MoveIt target failed link=%s frame=%s pos=%s"
                % (target_link, frame_id, _format_pose_target(pos))
            )
        if getattr(self, "_hold_monitor_active", False) and not self._holding_is_healthy():
            self._handle_hold_lost()
            return False
        return executed

    def _pick_tcp_target(self, pos):
        clearance = PICK_TCP_Z_CLEARANCE
        y_target = pos[1]
        if self.use_tactile_grasp:
            clearance = TACTILE_PICK_TCP_Z_CLEARANCE
            limit = TACTILE_PICK_TCP_VISUAL_Y_LIMIT
            y_target = min(max(pos[1] + TACTILE_PICK_LATERAL_BIAS, -limit), limit)
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
            restore_scaling = None
            if self.use_tactile_grasp and hasattr(self, "moveit2"):
                restore_scaling = (
                    self.moveit2.max_velocity,
                    self.moveit2.max_acceleration,
                )
                # 触觉抓取只要求接触/下降段慢速。approach/lift/transfer 离物体和
                # 台面有安全高度，使用普通速度可避免 G4 专项被高空移动拖成超时。
                self.moveit2.max_velocity = ARM_MAX_VELOCITY_SCALING
                self.moveit2.max_acceleration = ARM_MAX_ACCELERATION_SCALING
            if self._move(
                pos,
                target_link=GRIPPER_TCP_LINK,
                tolerance_position=DEFAULT_APPROACH_TOLERANCE_POSITION,
                tolerance_orientation=DEFAULT_APPROACH_TOLERANCE_ORIENTATION,
                timeout_sec=DEFAULT_MOVE_TIMEOUT_SEC,
                cartesian=cartesian,
            ):
                if restore_scaling is not None:
                    (
                        self.moveit2.max_velocity,
                        self.moveit2.max_acceleration,
                    ) = restore_scaling
                return True
            if restore_scaling is not None:
                (
                    self.moveit2.max_velocity,
                    self.moveit2.max_acceleration,
                ) = restore_scaling
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
        self._start_hold_monitor()
        if not self._holding_is_healthy():
            self._handle_hold_lost()
            self.get_logger().warn("Pick failed: hold monitor rejected attachment")
            return "failed"
        if not self.use_tactile_grasp and not self.gripper.close():
            self._stop_hold_monitor()
            self.gripper.release_object()
            self._detach_carried_sample()
            self.get_logger().warn("Pick failed: gripper close rejected")
            return "failed"
        if not self._move_approach(lift, cartesian=True):
            self._stop_hold_monitor()
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
        if not self._holding_is_healthy():
            self._handle_hold_lost()
            self.get_logger().warn("Place failed: carried object is no longer held")
            return False
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
            self._handle_hold_lost()
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
            self._handle_hold_lost()
            self.get_logger().warn("Place failed: descent move failed")
            return False
        # 已到达释放位后，release_object() 会等待 Gazebo 插件确认“解除持有”。
        # 这段等待期间 hold_status 从 holding 切到 released 是正常语义；
        # 若继续运行持有定时器，会把正常释放边界误报成“搬运中丢失”。
        self._stop_hold_monitor()
        if not self.gripper.release_object():
            self._start_hold_monitor()
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
