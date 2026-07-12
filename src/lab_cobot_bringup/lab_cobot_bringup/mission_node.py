#!/usr/bin/env python3
"""
跨工位抓取任务编排节点.

驱动 task_state_machine,串联:Nav2(BasicNavigator)导航 → 感知(obj TF)→
MoveIt 抓取(PickPlace)→ 导航 → 放置 → 返回 home。

订阅 /task/instruction(std_msgs/String)触发;发布 /task/status(当前状态)。
运行时依赖:move_group、Nav2、aruco_detector、Gazebo 控制器、
contact grasp 插件;`gripper_attach_bridge` 仅为显式 sim_attach 调试路径。

注:本节点为运行时编排,需完整系统启动后验证;依赖较多,属集成层。
"""
import os
import time
import math
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import tf2_ros
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from lab_cobot_bringup.task_planner import PlannerConfig, plan_actions
from lab_cobot_bringup.task_state_machine import SequentialTask, TaskState
from lab_cobot_navigation.waypoints import get_waypoint, yaw_to_quat
from lab_cobot_manipulation.pick_place_node import PickPlace
from lab_cobot_manipulation.gripper_driver import DEFAULT_TARGET_OBJECT

RETREAT_TOPIC = "/cmd_vel"
RAW_ODOM_TOPIC = "/odom"
DETECTION_TOPIC_TEMPLATE = "/perception/aruco_{object_id}/pose"
WRIST_DETECTION_TOPIC_TEMPLATE = "/perception/wrist/aruco_{object_id}/pose"
RETREAT_LINEAR_X = -0.18
RETREAT_DURATION_SEC = 6.0
RETREAT_PUBLISH_PERIOD_SEC = 0.05
RETREAT_STOP_SEC = 0.5
# base_link 系 TCP 放置点。z=0.725 + 悬空释放余量 0.02(pick_place 侧)
# 使物块底面名义高出台面约 5cm 自由落下,覆盖视觉 z 误差带(±1.5cm),
# 避免带焊物块压入台面引发约束爆炸(E2E 实测弹飞根因)。
DEFAULT_PLACE_POSE = [0.82, 0.20, 0.725]
PLACE_BASE_TARGET_POSE = (-2.0, 0.62, math.pi / 2.0)
# Leave reach margin for the vertical gripper pose.  At 0.78 m the detected
# target plus TCP approach offset sits on the UR5e workspace boundary.
DOCK_TARGET_X = 0.62
DOCK_TARGET_Y = 0.0
DOCK_TOLERANCE_X = 0.05
DOCK_TOLERANCE_Y = 0.065
DOCK_GAIN_X = 0.8
DOCK_GAIN_Y = 0.8
DOCK_MAX_LINEAR_X = 0.08
DOCK_MAX_LINEAR_Y = 0.08
DOCK_TIMEOUT_SEC = 20.0
DOCK_PUBLISH_PERIOD_SEC = 0.05
DOCK_STOP_SEC = 0.3
DETECTION_MAX_AGE_SEC = 1.0
REFINE_WAIT_SEC = 2.5
REFINE_POLL_SEC = 0.05
NAV_TIMEOUT_SEC = 60.0
NAV_SERVER_WAIT_SEC = 20.0
NAV_STARTUP_WAIT_SEC = 120.0
NAV_STARTUP_POLL_SEC = 1.0
NAV_ACTIVE_WAIT_SEC = 30.0
NAV_ACTIVE_POLL_SEC = 0.5
NAV_ACTIVE_CALL_TIMEOUT_SEC = 2.0
NAV_TF_READY_WAIT_SEC = 12.0
NAV_TF_READY_LOOKUP_SEC = 0.1
NAV_TF_READY_POLL_SEC = 0.2
PICK_NAV_HANDOFF_MIN_X = 0.70
PICK_NAV_HANDOFF_MAX_X = 0.90
PICK_NAV_HANDOFF_MAX_ABS_Y = 0.12
PICK_NAV_HANDOFF_MAX_STATION_DISTANCE = 0.35
STATION_B_TABLE_MIN_X = -2.4
STATION_B_TABLE_MAX_X = -1.6
STATION_B_TABLE_FRONT_Y = 1.2
STATION_B_TABLE_BACK_Y = 1.8
STATION_B_SAFE_DROP_FRONT_Y = 1.35
STATION_B_SAFE_DROP_BACK_Y = 1.65
NAV_HANDOFF_STOP_SEC = 0.3
STATION_DOCK_TOLERANCE_X = 0.06
STATION_DOCK_TOLERANCE_Y = 0.06
STATION_DOCK_TOLERANCE_YAW = 0.15
STATION_DOCK_GAIN_X = 0.8
STATION_DOCK_GAIN_Y = 0.8
STATION_DOCK_GAIN_YAW = 1.4
STATION_DOCK_MAX_LINEAR_X = 0.16
STATION_DOCK_MAX_LINEAR_Y = 0.16
STATION_DOCK_MAX_ANGULAR = 0.45
STATION_DOCK_YAW_FIRST_THRESHOLD = 0.35
STATION_DOCK_YAW_FIRST_LINEAR_SCALE = 0.35
STATION_DOCK_TIMEOUT_SEC = 80.0
STATION_DOCK_PUBLISH_PERIOD_SEC = 0.05
STATION_DOCK_STOP_SEC = 0.3
PLACE_NAV_HANDOFF_MAX_DISTANCE = 0.30
PLACE_DOCK_TOLERANCE_X = 0.06
PLACE_DOCK_TOLERANCE_Y = 0.035
PLACE_DOCK_TOLERANCE_YAW = 0.15
PLACE_DOCK_GAIN_X = 0.9
PLACE_DOCK_GAIN_Y = 0.9
PLACE_DOCK_GAIN_YAW = 1.6
PLACE_DOCK_MAX_LINEAR_X = 0.12
PLACE_DOCK_MAX_LINEAR_Y = 0.10
PLACE_DOCK_MAX_ANGULAR = 0.45
PLACE_DOCK_TIMEOUT_SEC = 20.0
PLACE_DOCK_PUBLISH_PERIOD_SEC = 0.05
PLACE_DOCK_STOP_SEC = 0.3
HOME_NAV_HANDOFF_MAX_DISTANCE = 0.25
HOME_NAV_HANDOFF_MAX_ABS_YAW = 0.40
WORKTABLE_STATIONS = frozenset(("station_a", "station_b"))
WORKTABLE_FRONT_Y = 1.20
CHASSIS_LENGTH = 0.42
CHASSIS_WIDTH = 0.30
WORKTABLE_CLEARANCE = 0.35
WORKTABLE_MIN_EXIT_SPEED = 0.03


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _angle_wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def transform_stamp_is_fresh(
    now_stamp,
    transform_stamp,
    max_age_sec: float = DETECTION_MAX_AGE_SEC,
) -> bool:
    age_sec = _stamp_seconds(now_stamp) - _stamp_seconds(transform_stamp)
    return -0.25 <= age_sec <= max_age_sec


def _chassis_map_y_half_extent(yaw: float) -> float:
    return (abs(math.sin(yaw)) * CHASSIS_LENGTH * 0.5
            + abs(math.cos(yaw)) * CHASSIS_WIDTH * 0.5)


def station_safe_base_y(yaw: float, station: str) -> float:
    if station not in WORKTABLE_STATIONS:
        raise ValueError(f"{station} is not a worktable station")
    return WORKTABLE_FRONT_Y - WORKTABLE_CLEARANCE - _chassis_map_y_half_extent(yaw)


def worktable_clearance(base_pose, station: str) -> float:
    if station not in WORKTABLE_STATIONS:
        return math.inf
    _x, y, yaw = [float(v) for v in base_pose]
    return WORKTABLE_FRONT_Y - (y + _chassis_map_y_half_extent(yaw))


def _limit_worktable_approach(cmd: Twist, base_pose, station: str) -> Twist:
    if station not in WORKTABLE_STATIONS or base_pose is None:
        return cmd
    yaw = float(base_pose[2])
    clearance = worktable_clearance(base_pose, station)
    vx_map = math.cos(yaw) * cmd.linear.x - math.sin(yaw) * cmd.linear.y
    vy_map = math.sin(yaw) * cmd.linear.x + math.cos(yaw) * cmd.linear.y
    if clearance < WORKTABLE_CLEARANCE - 1.0e-6:
        vx_map = 0.0
        vy_map = -max(
            WORKTABLE_MIN_EXIT_SPEED,
            STATION_DOCK_GAIN_X * (WORKTABLE_CLEARANCE - clearance),
        )
        cmd.angular.z = 0.0
    elif vy_map > 0.0:
        remaining = max(0.0, clearance - WORKTABLE_CLEARANCE)
        vy_map = min(vy_map, STATION_DOCK_GAIN_X * remaining)
    cmd.linear.x = math.cos(yaw) * vx_map + math.sin(yaw) * vy_map
    cmd.linear.y = -math.sin(yaw) * vx_map + math.cos(yaw) * vy_map
    return cmd


def dock_velocity_for_object(object_pose, base_pose=None, station="station_a"):
    error_x = float(object_pose[0]) - DOCK_TARGET_X
    error_y = float(object_pose[1]) - DOCK_TARGET_Y
    cmd = Twist()

    done = (
        abs(error_x) <= DOCK_TOLERANCE_X
        and abs(error_y) <= DOCK_TOLERANCE_Y
    )
    at_safety_line = (
        base_pose is not None
        and station in WORKTABLE_STATIONS
        and worktable_clearance(base_pose, station) >= WORKTABLE_CLEARANCE - 1.0e-6
        and worktable_clearance(base_pose, station) <= WORKTABLE_CLEARANCE + 1.0e-6
    )
    done = done or (at_safety_line and abs(error_y) <= DOCK_TOLERANCE_Y)
    if done:
        return True, cmd

    cmd.linear.x = _clamp(DOCK_GAIN_X * error_x, DOCK_MAX_LINEAR_X)
    cmd.linear.y = _clamp(DOCK_GAIN_Y * error_y, DOCK_MAX_LINEAR_Y)
    _limit_worktable_approach(cmd, base_pose, station)
    return False, cmd


def pick_navigation_handoff_ready(object_pose) -> bool:
    if object_pose is None:
        return False
    x = float(object_pose[0])
    y = float(object_pose[1])
    return (
        PICK_NAV_HANDOFF_MIN_X <= x <= PICK_NAV_HANDOFF_MAX_X
        and abs(y) <= PICK_NAV_HANDOFF_MAX_ABS_Y
    )


def pick_map_handoff_ready(base_pose) -> bool:
    if base_pose is None:
        return False
    station_x, station_y, _station_yaw = _station_base_pose("station_a")
    return math.hypot(
        float(base_pose[0]) - station_x,
        float(base_pose[1]) - station_y,
    ) <= PICK_NAV_HANDOFF_MAX_STATION_DISTANCE


def _base_target_to_map(base_pose, target_xy):
    base_x, base_y, yaw = base_pose
    target_x, target_y = target_xy
    return (
        base_x + target_x * math.cos(yaw) - target_y * math.sin(yaw),
        base_y + target_x * math.sin(yaw) + target_y * math.cos(yaw),
    )


def _station_b_table_contains(map_x: float, map_y: float) -> bool:
    return (
        STATION_B_TABLE_MIN_X <= map_x <= STATION_B_TABLE_MAX_X
        and STATION_B_TABLE_FRONT_Y <= map_y <= STATION_B_TABLE_BACK_Y
    )


def _station_b_safe_drop_contains(map_x: float, map_y: float) -> bool:
    return (
        STATION_B_TABLE_MIN_X <= map_x <= STATION_B_TABLE_MAX_X
        and STATION_B_SAFE_DROP_FRONT_Y <= map_y <= STATION_B_SAFE_DROP_BACK_Y
    )


def _station_base_pose(station: str):
    wp = get_waypoint(station)
    return (float(wp["x"]), float(wp["y"]), float(wp["yaw"]))


def place_navigation_handoff_ready(base_pose, place_pose) -> bool:
    if base_pose is None:
        return False
    station_x, station_y, _station_yaw = _station_base_pose("station_b")
    distance_to_station = math.hypot(
        float(base_pose[0]) - station_x,
        float(base_pose[1]) - station_y,
    )
    if distance_to_station <= PLACE_NAV_HANDOFF_MAX_DISTANCE:
        return True

    map_x, map_y = _base_target_to_map(base_pose, place_pose[:2])
    return _station_b_table_contains(map_x, map_y)


def station_dock_velocity_for_base(base_pose, station: str):
    cmd = Twist()
    if base_pose is None:
        return False, cmd

    base_x, base_y, yaw = [float(v) for v in base_pose]
    target_x, target_y, target_yaw = _station_base_pose(station)
    if station in WORKTABLE_STATIONS:
        target_y = station_safe_base_y(yaw, station)
    error_x_map = target_x - base_x
    error_y_map = target_y - base_y
    error_yaw = _angle_wrap(target_yaw - yaw)

    clearance = worktable_clearance(base_pose, station)
    longitudinal_done = abs(error_y_map) <= STATION_DOCK_TOLERANCE_Y
    if station in WORKTABLE_STATIONS:
        longitudinal_done = (
            WORKTABLE_CLEARANCE - 1.0e-6 <= clearance
            <= WORKTABLE_CLEARANCE + STATION_DOCK_TOLERANCE_Y
        )
    done = (
        abs(error_x_map) <= STATION_DOCK_TOLERANCE_X
        and longitudinal_done
        and abs(error_yaw) <= STATION_DOCK_TOLERANCE_YAW
    )
    if done:
        return True, cmd

    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    error_x_base = cos_yaw * error_x_map + sin_yaw * error_y_map
    error_y_base = -sin_yaw * error_x_map + cos_yaw * error_y_map
    linear_scale = (
        STATION_DOCK_YAW_FIRST_LINEAR_SCALE
        if abs(error_yaw) > STATION_DOCK_YAW_FIRST_THRESHOLD
        else 1.0
    )

    cmd.linear.x = _clamp(
        STATION_DOCK_GAIN_X * error_x_base * linear_scale,
        STATION_DOCK_MAX_LINEAR_X,
    )
    cmd.linear.y = _clamp(
        STATION_DOCK_GAIN_Y * error_y_base * linear_scale,
        STATION_DOCK_MAX_LINEAR_Y,
    )
    cmd.angular.z = _clamp(
        STATION_DOCK_GAIN_YAW * error_yaw,
        STATION_DOCK_MAX_ANGULAR,
    )
    _limit_worktable_approach(cmd, base_pose, station)
    return False, cmd


def place_dock_velocity_for_base(base_pose, target_pose=None, place_pose=None):
    cmd = Twist()
    if base_pose is None:
        return False, cmd
    if target_pose is None:
        target_pose = PLACE_BASE_TARGET_POSE
    if place_pose is None:
        place_pose = DEFAULT_PLACE_POSE

    base_x, base_y, yaw = [float(v) for v in base_pose]
    target_x, target_y, target_yaw = [float(v) for v in target_pose]
    error_x_map = target_x - base_x
    error_y_map = station_safe_base_y(yaw, "station_b") - base_y
    error_yaw = _angle_wrap(target_yaw - yaw)
    place_x, place_y = _base_target_to_map(base_pose, place_pose[:2])
    drop_target_on_table = _station_b_safe_drop_contains(place_x, place_y)

    clearance = worktable_clearance(base_pose, "station_b")
    longitudinal_done = (
        WORKTABLE_CLEARANCE - 1.0e-6 <= clearance
        <= WORKTABLE_CLEARANCE + PLACE_DOCK_TOLERANCE_Y
    )
    done = (
        abs(error_x_map) <= PLACE_DOCK_TOLERANCE_X
        and longitudinal_done
        and abs(error_yaw) <= PLACE_DOCK_TOLERANCE_YAW
        and drop_target_on_table
    )
    if done:
        return True, cmd

    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    error_x_base = cos_yaw * error_x_map + sin_yaw * error_y_map
    error_y_base = -sin_yaw * error_x_map + cos_yaw * error_y_map

    cmd.linear.x = _clamp(PLACE_DOCK_GAIN_X * error_x_base, PLACE_DOCK_MAX_LINEAR_X)
    cmd.linear.y = _clamp(PLACE_DOCK_GAIN_Y * error_y_base, PLACE_DOCK_MAX_LINEAR_Y)
    cmd.angular.z = _clamp(PLACE_DOCK_GAIN_YAW * error_yaw, PLACE_DOCK_MAX_ANGULAR)
    _limit_worktable_approach(cmd, base_pose, "station_b")
    return False, cmd


def home_navigation_handoff_ready(base_pose) -> bool:
    if base_pose is None:
        return False
    x, y, yaw = [float(v) for v in base_pose]
    return (
        math.hypot(x, y) <= HOME_NAV_HANDOFF_MAX_DISTANCE
        and abs(_angle_wrap(yaw)) <= HOME_NAV_HANDOFF_MAX_ABS_YAW
    )


def requires_departure_retreat(state: TaskState) -> bool:
    return state == TaskState.PICK


def base_pose_from_odom_msg(msg: Odometry):
    q = msg.pose.pose.orientation
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    return [
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        yaw,
    ]


def build_planner_config(
    llm_enabled: bool,
    api_base: str,
    model: str,
    timeout_sec: float,
) -> PlannerConfig:
    """构造规划器配置;API key 仅从环境变量 LLM_API_KEY 读入,不落参数."""
    return PlannerConfig(
        llm_enabled=llm_enabled,
        api_base=api_base,
        model=model,
        timeout_sec=timeout_sec,
        api_key=os.environ.get("LLM_API_KEY"),
    )


class MissionNode(Node):
    def __init__(self):
        super().__init__("mission_node")
        self.declare_parameter("object_id", 0)
        # object_id 是感知 marker id;target_object 是 Gazebo 抓取模型名。
        self.declare_parameter("target_object", DEFAULT_TARGET_OBJECT)
        self.declare_parameter("use_tactile_grasp", False)
        self.declare_parameter("use_refine_detect", False)
        self.declare_parameter("place_pose", DEFAULT_PLACE_POSE)  # base_link 系放置点
        self.object_id = int(self.get_parameter("object_id").value)
        self.target_object = str(self.get_parameter("target_object").value)
        self.use_tactile_grasp = bool(
            self.get_parameter("use_tactile_grasp").value
        )
        self._use_refine_detect = bool(
            self.get_parameter("use_refine_detect").value
        )
        self.place_pose = list(self.get_parameter("place_pose").value)
        # LLM 任务拆解:默认关闭,E2E/CI 离线;演示时 llm_enabled:=true 打开
        self.declare_parameter("llm_enabled", False)
        self.declare_parameter("llm_api_base", "https://api.deepseek.com")
        self.declare_parameter("llm_model", "deepseek-chat")
        self.declare_parameter("llm_timeout_sec", 10.0)
        self._planner_config = build_planner_config(
            llm_enabled=bool(self.get_parameter("llm_enabled").value),
            api_base=str(self.get_parameter("llm_api_base").value),
            model=str(self.get_parameter("llm_model").value),
            timeout_sec=float(self.get_parameter("llm_timeout_sec").value),
        )

        self.nav = BasicNavigator()
        # 导航栈就绪探测:wait_for_server 只保证 action server 存在(configure
        # 即创建),不保证节点 active;inactive 时发 goal 会被拒(实测:manager
        # 延后启动后 mission 首 goal 与 bt_navigator activate 窗口对撞)。
        self._bt_state_client = self.create_client(
            GetState, "/bt_navigator/get_state"
        )
        self.pp = PickPlace(
            target_object=self.target_object,
            use_tactile_grasp=self.use_tactile_grasp,
        )
        self.retreat_pub = self.create_publisher(Twist, RETREAT_TOPIC, 10)
        self._latest_odom_pose = None
        self._latest_detection_pose = None
        self._latest_wrist_detection_pose = None
        self.create_subscription(Odometry, RAW_ODOM_TOPIC, self._on_odom, 10)
        self.create_subscription(
            PoseStamped,
            DETECTION_TOPIC_TEMPLATE.format(object_id=self.object_id),
            self._on_detection_pose,
            10,
        )
        if self._use_refine_detect:
            self.create_subscription(
                PoseStamped,
                WRIST_DETECTION_TOPIC_TEMPLATE.format(object_id=self.object_id),
                self._on_wrist_detection_pose,
                10,
            )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.status_pub = self.create_publisher(String, "/task/status", 10)
        self.create_subscription(String, "/task/instruction", self._on_instruction, 10)
        self._busy = False
        self.get_logger().info("mission_node 就绪,等待 /task/instruction")

    def _on_instruction(self, msg: String):
        if self._busy:
            self.get_logger().warn("任务进行中,忽略新指令")
            return
        self.get_logger().info(f"收到指令: {msg.data}")
        self._instruction = msg.data
        self._busy = True
        Thread(target=self._run_mission, daemon=True).start()

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_odom_pose = base_pose_from_odom_msg(msg)

    def _on_detection_pose(self, msg: PoseStamped) -> None:
        self._latest_detection_pose = msg

    def _on_wrist_detection_pose(self, msg: PoseStamped) -> None:
        self._latest_wrist_detection_pose = msg

    def _run_mission(self):
        try:
            instruction = getattr(self, "_instruction", "")
            config = getattr(self, "_planner_config", None)
            result = plan_actions(instruction, config)
            self.get_logger().info(
                f"任务拆解[{result.source}]: {[s.name for s in result.steps]}"
            )
            # 冷启动时 Nav2 生命周期节点可能仍在 configure/activate。
            # 基础设施就绪不是业务步骤失败，不能消耗 NAV_TO_PICK 重试。
            if not self._wait_for_navigation_ready():
                self.get_logger().error("Nav2 启动超时,任务尚未开始")
                return
            task = SequentialTask(result.steps, max_retries=1)
            task.start()
            self._publish(task.state)
            while not task.is_terminal():
                ok = self._execute(task.state)
                task.on_result(ok)
                self._publish(task.state)
            if task.state == TaskState.FAILED:
                self._failsafe_cleanup()
            self.get_logger().info(f"任务结束: {task.state.name}")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"任务执行异常: {e}")
        finally:
            self._busy = False

    def _execute(self, state: TaskState) -> bool:
        try:
            if state == TaskState.NAV_TO_PICK:
                return (
                    self._navigate("station_a")
                    and self._dock_to_station_pose("station_a")
                    and self._dock_to_pick_target()
                )
            if state == TaskState.DETECT:
                return self._detect() is not None
            if state == TaskState.PICK:
                pose = self._detect()
                refine_cb = (
                    self._make_refine_cb()
                    if getattr(self, "_use_refine_detect", False)
                    else None
                )
                ok = pose is not None and self.pp.pick(
                    pose,
                    refine_cb=refine_cb,
                )
                return self._finish_station_step(state, ok)
            if state == TaskState.NAV_TO_PLACE:
                return (
                    self._navigate("station_b")
                    and self._dock_to_station_pose("station_b")
                    and self._dock_to_place_target()
                )
            if state == TaskState.PLACE:
                ok = self.pp.place(self.place_pose)
                return self._finish_station_step(state, ok)
            if state == TaskState.RETURN_HOME:
                if not self.pp.go_home():
                    return False
                return self._navigate("home") and self._dock_to_station_pose("home")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"步骤 {state.name} 异常: {e}")
            return False
        return False

    def _finish_station_step(self, state: TaskState, ok: bool) -> bool:
        if ok and requires_departure_retreat(state):
            return self._retreat_from_station()
        return ok

    def _duration_elapsed(self, start, duration_sec: float) -> bool:
        elapsed = self.get_clock().now() - start
        timeout = rclpy.duration.Duration(seconds=duration_sec)
        return elapsed.nanoseconds >= timeout.nanoseconds

    def _publish_cmd_for_duration(self, cmd: Twist, duration_sec: float) -> None:
        start = self.get_clock().now()
        while not self._duration_elapsed(start, duration_sec):
            self.retreat_pub.publish(cmd)
            time.sleep(RETREAT_PUBLISH_PERIOD_SEC)

    def _failsafe_cleanup(self) -> None:
        self.get_logger().warn("任务失败,执行终态兜底清理")
        try:
            self.pp.gripper.release_object()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"兜底 detach 失败: {e}")
        try:
            self._stop_base(DOCK_STOP_SEC)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"兜底停底盘失败: {e}")
        try:
            self.pp.go_home()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"兜底 go_home 失败: {e}")

    def _retreat_from_station(self) -> bool:
        self.get_logger().info("离开工位前退避")
        cmd = Twist()
        cmd.linear.x = RETREAT_LINEAR_X
        self._publish_cmd_for_duration(cmd, RETREAT_DURATION_SEC)

        stop = Twist()
        self._publish_cmd_for_duration(stop, RETREAT_STOP_SEC)
        return True

    def _dock_to_pick_target(self) -> bool:
        self.get_logger().info("视觉停靠到样件")
        start = self.get_clock().now()
        last_pose = None
        while not self._duration_elapsed(start, DOCK_TIMEOUT_SEC):
            pose = self._detect()
            if pose is not None:
                last_pose = pose
                done, cmd = dock_velocity_for_object(
                    pose,
                    base_pose=self._base_pose_in_map(timeout_sec=0.05),
                    station="station_a",
                )
                if done:
                    self._stop_base(DOCK_STOP_SEC)
                    self.get_logger().info(
                        f"视觉停靠完成 obj=({pose[0]:.3f},{pose[1]:.3f},{pose[2]:.3f})"
                    )
                    return True
                self.retreat_pub.publish(cmd)
            time.sleep(DOCK_PUBLISH_PERIOD_SEC)

        self._stop_base(DOCK_STOP_SEC)
        if last_pose is None:
            self.get_logger().warn("视觉停靠失败: 未检测到 obj_0")
        else:
            self.get_logger().warn(
                "视觉停靠超时: "
                f"obj=({last_pose[0]:.3f},{last_pose[1]:.3f},{last_pose[2]:.3f})"
            )
        return False

    def _dock_to_station_pose(self, station: str) -> bool:
        self.get_logger().info(f"地图精停到 {station}")
        start = self.get_clock().now()
        last_pose = None
        while not self._duration_elapsed(start, STATION_DOCK_TIMEOUT_SEC):
            pose = self._base_pose_in_map(timeout_sec=0.05)
            if pose is not None:
                last_pose = pose
                done, cmd = station_dock_velocity_for_base(pose, station)
                if done:
                    self._stop_base(STATION_DOCK_STOP_SEC)
                    self.get_logger().info(
                        f"地图精停完成 base=({pose[0]:.3f},{pose[1]:.3f},"
                        f"{math.degrees(pose[2]):.1f}deg)"
                    )
                    return True
                self.retreat_pub.publish(cmd)
            time.sleep(STATION_DOCK_PUBLISH_PERIOD_SEC)

        self._stop_base(STATION_DOCK_STOP_SEC)
        if last_pose is None:
            self.get_logger().warn(f"地图精停失败: 未获取 {station} base_link map 位姿")
        else:
            self.get_logger().warn(
                f"地图精停超时 {station}: "
                f"base=({last_pose[0]:.3f},{last_pose[1]:.3f},"
                f"{math.degrees(last_pose[2]):.1f}deg)"
            )
        return False

    def _dock_to_place_target(self) -> bool:
        self.get_logger().info("放置停靠到B工位")
        start = self.get_clock().now()
        last_pose = None
        target_pose = PLACE_BASE_TARGET_POSE
        while not self._duration_elapsed(start, PLACE_DOCK_TIMEOUT_SEC):
            pose = self._base_pose_in_map(timeout_sec=0.05)
            if pose is not None:
                last_pose = pose
                done, cmd = place_dock_velocity_for_base(
                    pose,
                    target_pose,
                    self.place_pose,
                )
                if done:
                    self._stop_base(PLACE_DOCK_STOP_SEC)
                    place_x, place_y = _base_target_to_map(pose, self.place_pose[:2])
                    self.get_logger().info(
                        "放置停靠完成 "
                        f"base=({pose[0]:.3f},{pose[1]:.3f},{math.degrees(pose[2]):.1f}deg) "
                        f"place_map=({place_x:.3f},{place_y:.3f})"
                    )
                    return True
                self.retreat_pub.publish(cmd)
            time.sleep(PLACE_DOCK_PUBLISH_PERIOD_SEC)

        self._stop_base(PLACE_DOCK_STOP_SEC)
        if last_pose is None:
            self.get_logger().warn("放置停靠失败: 未获取 base_link map 位姿")
        else:
            self.get_logger().warn(
                "放置停靠超时: "
                f"base=({last_pose[0]:.3f},{last_pose[1]:.3f},"
                f"{math.degrees(last_pose[2]):.1f}deg)"
            )
        return False

    def _stop_base(self, duration_sec: float) -> None:
        stop = Twist()
        self._publish_cmd_for_duration(stop, duration_sec)

    def _navigate(self, station: str) -> bool:
        wp = get_waypoint(station)
        self.get_logger().info(f"导航到 {station} ({wp['x']:.2f},{wp['y']:.2f})")
        # 有界探测:BasicNavigator.goToPose 内部 wait_for_server 是无限等待,
        # bt_navigator 偶发未就绪会让任务挂死到 E2E 420s 超时(重复统计实测)。
        # 探测失败按导航失败处理,交给状态机重试与 FAILED 兜底。
        if not self.nav.nav_to_pose_client.wait_for_server(
            timeout_sec=NAV_SERVER_WAIT_SEC
        ):
            self.get_logger().warn(f"导航服务未就绪,放弃导航到 {station}")
            return False
        if not self._wait_for_nav_active():
            self.get_logger().warn(f"bt_navigator 未 active,放弃导航到 {station}")
            return False
        if not self._wait_for_navigation_tf(station):
            return False
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.nav.get_clock().now().to_msg()
        goal.pose.position.x = wp["x"]
        goal.pose.position.y = wp["y"]
        qx, qy, qz, qw = yaw_to_quat(wp["yaw"])
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw
        self.nav.goToPose(goal)
        t0 = self.get_clock().now()
        timeout = rclpy.duration.Duration(seconds=NAV_TIMEOUT_SEC)
        # BasicNavigator.isTaskComplete 内部 spin nav 节点;mission 由外部 executor spin,此处仅等待(避免双重 spin)
        while not self.nav.isTaskComplete():
            if self._navigation_handoff_ready(station):
                self.get_logger().info(f"导航到 {station} 已满足任务交接条件")
                self.nav.cancelTask()
                self._stop_base(NAV_HANDOFF_STOP_SEC)
                return True
            if (self.get_clock().now() - t0).nanoseconds > timeout.nanoseconds:
                self.get_logger().warn(f"导航到 {station} 超时，取消")
                self.nav.cancelTask()
                self._stop_base(NAV_HANDOFF_STOP_SEC)
                return False
            time.sleep(0.2)
        return self.nav.getResult() == TaskResult.SUCCEEDED

    def _wait_for_navigation_ready(
        self,
        timeout_sec: float = NAV_STARTUP_WAIT_SEC,
    ) -> bool:
        """Wait for cold-start Nav2 without consuming mission retries."""
        deadline = time.monotonic() + timeout_sec
        logged = False
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            probe = min(NAV_STARTUP_POLL_SEC, remaining)
            if self.nav.nav_to_pose_client.wait_for_server(timeout_sec=probe):
                remaining = max(0.0, deadline - time.monotonic())
                if self._wait_for_nav_active(timeout_sec=remaining):
                    return True
            if not logged:
                self.get_logger().info("等待 Nav2 action 与生命周期 active")
                logged = True
        return False

    def _wait_for_nav_active(
        self,
        timeout_sec: float = NAV_ACTIVE_WAIT_SEC,
    ) -> bool:
        """Wait until bt_navigator lifecycle state reaches active."""
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            result = None
            if self._bt_state_client.service_is_ready():
                try:
                    future = self._bt_state_client.call_async(GetState.Request())
                    call_deadline = min(
                        deadline,
                        time.monotonic() + NAV_ACTIVE_CALL_TIMEOUT_SEC,
                    )
                    while (
                        rclpy.ok()
                        and not future.done()
                        and time.monotonic() < call_deadline
                    ):
                        remaining = max(
                            0.0, call_deadline - time.monotonic()
                        )
                        time.sleep(min(0.05, remaining))
                    if future.done():
                        result = future.result()
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(
                        f"bt_navigator 状态探测失败,继续等待: {exc}"
                    )
                if (
                    result is not None
                    and result.current_state.id
                    == LifecycleState.PRIMARY_STATE_ACTIVE
                ):
                    return True
            remaining = max(0.0, deadline - time.monotonic())
            time.sleep(min(NAV_ACTIVE_POLL_SEC, remaining))
        return False

    def _wait_for_navigation_tf(self, station: str) -> bool:
        start = self.get_clock().now()
        while not self._duration_elapsed(start, NAV_TF_READY_WAIT_SEC):
            if self._base_pose_in_map(timeout_sec=NAV_TF_READY_LOOKUP_SEC) is not None:
                return True
            time.sleep(NAV_TF_READY_POLL_SEC)
        self.get_logger().warn(f"导航到 {station} 前 map TF 未就绪")
        return False

    def _navigation_handoff_ready(self, station: str) -> bool:
        if station == "station_a":
            return pick_navigation_handoff_ready(
                self._detect(timeout_sec=0.05)
            ) or pick_map_handoff_ready(
                self._base_pose_in_map(timeout_sec=0.05)
            )
        if station == "station_b":
            return place_navigation_handoff_ready(
                self._base_pose_in_map(timeout_sec=0.05),
                self.place_pose,
            )
        if station == "home":
            return home_navigation_handoff_ready(
                self._base_pose_in_map(timeout_sec=0.05),
            )
        return False

    def _base_pose_in_map(self, timeout_sec: float = 2.0):
        return self._base_pose_in_frame("map", timeout_sec)

    def _base_pose_in_odom(self, timeout_sec: float = 2.0):
        del timeout_sec
        if self._latest_odom_pose is None:
            return None
        return list(self._latest_odom_pose)

    def _base_pose_in_frame(self, frame: str, timeout_sec: float = 2.0):
        try:
            t = self.tf_buffer.lookup_transform(
                frame, "base_link", rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout_sec),
            )
            q = t.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            return [
                t.transform.translation.x,
                t.transform.translation.y,
                yaw,
            ]
        except Exception:
            return None

    def _detect(self, timeout_sec: float = 2.0):
        pose = self._latest_detection_pose
        if pose is not None and pose.header.frame_id == "base_link":
            if transform_stamp_is_fresh(
                self.get_clock().now().to_msg(),
                pose.header.stamp,
            ):
                return [
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z,
                ]
            self.get_logger().warn(
                f"未检测到 obj_{self.object_id}: 感知位姿已过期"
            )

        frame = f"obj_{self.object_id}"
        try:
            t = self.tf_buffer.lookup_transform(
                "base_link", frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout_sec),
            )
            if not transform_stamp_is_fresh(
                self.get_clock().now().to_msg(),
                t.header.stamp,
            ):
                self.get_logger().warn(f"未检测到 {frame}: TF 已过期")
                return None
            return [
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            ]
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"未检测到 {frame}: {e}")
            return None

    def _make_refine_cb(self):
        def wait_for_refined_position():
            start = self.get_clock().now()
            start_seconds = _stamp_seconds(start.to_msg())
            while not self._duration_elapsed(start, REFINE_WAIT_SEC):
                pose = self._latest_wrist_detection_pose
                if pose is not None and pose.header.frame_id == "base_link":
                    pose_seconds = _stamp_seconds(pose.header.stamp)
                    if pose_seconds > start_seconds:
                        now_seconds = _stamp_seconds(
                            self.get_clock().now().to_msg()
                        )
                        age_seconds = now_seconds - pose_seconds
                        if -0.25 <= age_seconds < DETECTION_MAX_AGE_SEC:
                            return [
                                pose.pose.position.x,
                                pose.pose.position.y,
                                pose.pose.position.z,
                            ]
                time.sleep(REFINE_POLL_SEC)
            return None

        return wait_for_refined_position

    def _publish(self, state: TaskState):
        m = String()
        m.data = state.name
        self.status_pub.publish(m)


def main():
    rclpy.init()
    node = MissionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node.pp)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.pp.destroy_node()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
