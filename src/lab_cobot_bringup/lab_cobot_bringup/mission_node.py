#!/usr/bin/env python3
"""跨工位抓取任务编排节点。

驱动 task_state_machine,串联:Nav2(BasicNavigator)导航 → 感知(obj TF)→
MoveIt 抓取(PickPlace)→ 导航 → 放置 → 返回 home。

订阅 /task/instruction(std_msgs/String)触发;发布 /task/status(当前状态)。
运行时依赖:move_group、Nav2、aruco_detector、Gazebo 控制器、/suction/switch。

注:本节点为运行时编排,需完整系统启动后验证;依赖较多,属集成层。
"""
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import tf2_ros

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from lab_cobot_bringup.task_state_machine import CrossStationTask, TaskState
from lab_cobot_navigation.waypoints import get_waypoint, yaw_to_quat
from lab_cobot_manipulation.pick_place_node import PickPlace


class MissionNode(Node):
    def __init__(self):
        super().__init__("mission_node")
        self.declare_parameter("object_id", 0)
        self.declare_parameter("place_pose", [0.0, 0.45, 0.85])  # base_link 系放置点
        self.object_id = int(self.get_parameter("object_id").value)
        self.place_pose = list(self.get_parameter("place_pose").value)

        self.nav = BasicNavigator()
        self.pp = PickPlace()
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
        Thread(target=self._run_mission, daemon=True).start()

    def _run_mission(self):
        self._busy = True
        task = CrossStationTask(max_retries=1)
        task.start()
        self._publish(task.state)
        while not task.is_terminal():
            ok = self._execute(task.state)
            task.on_result(ok)
            self._publish(task.state)
        self.get_logger().info(f"任务结束: {task.state.name}")
        self._busy = False

    def _execute(self, state: TaskState) -> bool:
        try:
            if state == TaskState.NAV_TO_PICK:
                return self._navigate("station_a")
            if state == TaskState.DETECT:
                return self._detect() is not None
            if state == TaskState.PICK:
                pose = self._detect()
                return pose is not None and self.pp.pick(pose)
            if state == TaskState.NAV_TO_PLACE:
                return self._navigate("station_b")
            if state == TaskState.PLACE:
                return self.pp.place(self.place_pose)
            if state == TaskState.RETURN_HOME:
                nav_ok = self._navigate("home")
                return nav_ok and self.pp.go_home()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"步骤 {state.name} 异常: {e}")
            return False
        return False

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
        while not self.nav.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.nav.getResult() == TaskResult.SUCCEEDED

    def _detect(self):
        frame = f"obj_{self.object_id}"
        try:
            t = self.tf_buffer.lookup_transform(
                "base_link", frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
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
