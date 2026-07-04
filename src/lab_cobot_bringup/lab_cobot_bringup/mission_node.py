#!/usr/bin/env python3
"""
跨工位抓取任务编排节点.

驱动 task_state_machine,串联:Nav2(BasicNavigator)导航 → 感知(obj TF)→
MoveIt 抓取(PickPlace)→ 导航 → 放置 → 返回 home。

订阅 /task/instruction(std_msgs/String)触发;发布 /task/status(当前状态)。
运行时依赖:move_group、Nav2、aruco_detector、Gazebo 控制器、gripper attach bridge。

注:本节点为运行时编排,需完整系统启动后验证;依赖较多,属集成层。
"""
import time
import math
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
import tf2_ros

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from lab_cobot_bringup.task_state_machine import CrossStationTask, TaskState
from lab_cobot_navigation.waypoints import get_waypoint, yaw_to_quat
from lab_cobot_manipulation.pick_place_node import PickPlace

RETREAT_TOPIC = "/cmd_vel_nav"
RETREAT_LINEAR_X = -0.18
RETREAT_DURATION_SEC = 6.0
RETREAT_PUBLISH_PERIOD_SEC = 0.05
RETREAT_STOP_SEC = 0.5
DEFAULT_PLACE_POSE = [0.82, 0.20, 0.630]
DOCK_TARGET_X = 0.78
DOCK_TARGET_Y = 0.0
DOCK_TOLERANCE_X = 0.035
DOCK_TOLERANCE_Y = 0.065
DOCK_GAIN_X = 0.8
DOCK_GAIN_Y = 0.8
DOCK_MAX_LINEAR_X = 0.08
DOCK_MAX_LINEAR_Y = 0.08
DOCK_TIMEOUT_SEC = 20.0
DOCK_PUBLISH_PERIOD_SEC = 0.05
DOCK_STOP_SEC = 0.3
NAV_TIMEOUT_SEC = 60.0
PICK_NAV_HANDOFF_MIN_X = 0.70
PICK_NAV_HANDOFF_MAX_X = 0.90
PICK_NAV_HANDOFF_MAX_ABS_Y = 0.12
STATION_B_TABLE_MIN_X = -2.4
STATION_B_TABLE_MAX_X = -1.6
STATION_B_TABLE_FRONT_Y = 1.2
STATION_B_TABLE_BACK_Y = 1.8
NAV_HANDOFF_STOP_SEC = 0.3
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


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _angle_wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def dock_velocity_for_object(object_pose):
    error_x = float(object_pose[0]) - DOCK_TARGET_X
    error_y = float(object_pose[1]) - DOCK_TARGET_Y
    cmd = Twist()

    done = (
        abs(error_x) <= DOCK_TOLERANCE_X
        and abs(error_y) <= DOCK_TOLERANCE_Y
    )
    if done:
        return True, cmd

    cmd.linear.x = _clamp(DOCK_GAIN_X * error_x, DOCK_MAX_LINEAR_X)
    cmd.linear.y = _clamp(DOCK_GAIN_Y * error_y, DOCK_MAX_LINEAR_Y)
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


def place_dock_velocity_for_base(base_pose, target_pose=None, place_pose=None):
    cmd = Twist()
    if base_pose is None:
        return False, cmd
    if target_pose is None:
        target_pose = _station_base_pose("station_b")
    if place_pose is None:
        place_pose = DEFAULT_PLACE_POSE

    base_x, base_y, yaw = [float(v) for v in base_pose]
    target_x, target_y, target_yaw = [float(v) for v in target_pose]
    error_x_map = target_x - base_x
    error_y_map = target_y - base_y
    error_yaw = _angle_wrap(target_yaw - yaw)
    place_x, place_y = _base_target_to_map(base_pose, place_pose[:2])
    drop_target_on_table = _station_b_table_contains(place_x, place_y)

    done = (
        abs(error_x_map) <= PLACE_DOCK_TOLERANCE_X
        and abs(error_y_map) <= PLACE_DOCK_TOLERANCE_Y
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
    return state in {TaskState.PICK, TaskState.PLACE}


class MissionNode(Node):
    def __init__(self):
        super().__init__("mission_node")
        self.declare_parameter("object_id", 0)
        self.declare_parameter("place_pose", DEFAULT_PLACE_POSE)  # base_link 系放置点
        self.object_id = int(self.get_parameter("object_id").value)
        self.place_pose = list(self.get_parameter("place_pose").value)

        self.nav = BasicNavigator()
        self.pp = PickPlace()
        self.retreat_pub = self.create_publisher(Twist, RETREAT_TOPIC, 10)
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
        self._busy = True
        Thread(target=self._run_mission, daemon=True).start()

    def _run_mission(self):
        try:
            task = CrossStationTask(max_retries=1)
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
                return self._navigate("station_a") and self._dock_to_pick_target()
            if state == TaskState.DETECT:
                return self._detect() is not None
            if state == TaskState.PICK:
                pose = self._detect()
                ok = pose is not None and self.pp.pick(pose)
                return self._finish_station_step(state, ok)
            if state == TaskState.NAV_TO_PLACE:
                return self._navigate("station_b") and self._dock_to_place_target()
            if state == TaskState.PLACE:
                ok = self.pp.place(self.place_pose)
                return self._finish_station_step(state, ok)
            if state == TaskState.RETURN_HOME:
                nav_ok = self._navigate("home")
                return nav_ok and self.pp.go_home()
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
                done, cmd = dock_velocity_for_object(pose)
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

    def _dock_to_place_target(self) -> bool:
        self.get_logger().info("放置停靠到B工位")
        start = self.get_clock().now()
        last_pose = None
        target_pose = _station_base_pose("station_b")
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
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.nav.get_clock().now().to_msg()
        goal.pose.position.x = wp["x"]
        goal.pose.position.y = wp["y"]
        qx, qy, qz, qw = yaw_to_quat(wp["yaw"])
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw
        self.get_logger().info(f"导航到 {station} ({wp['x']:.2f},{wp['y']:.2f})")
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

    def _navigation_handoff_ready(self, station: str) -> bool:
        if station == "station_a":
            return pick_navigation_handoff_ready(self._detect(timeout_sec=0.05))
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
        try:
            t = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time(),
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
        frame = f"obj_{self.object_id}"
        try:
            t = self.tf_buffer.lookup_transform(
                "base_link", frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout_sec),
            )
            return [
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            ]
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"未检测到 {frame}: {e}")
            return None

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
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
