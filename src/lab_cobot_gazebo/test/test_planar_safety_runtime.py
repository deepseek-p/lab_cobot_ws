"""Gazebo integration regression for both table safety zones."""

import math
import os
import signal
import subprocess
import tempfile
import time

import rclpy
from gazebo_msgs.srv import GetEntityState
from std_msgs.msg import Float64MultiArray


WHEELS_X_POSITIVE = [-6.0, -6.0, -6.0, -6.0]
WHEELS_X_NEGATIVE = [6.0, 6.0, 6.0, 6.0]
WHEELS_Y_POSITIVE = [6.0, -6.0, -6.0, 6.0]
WHEELS_Y_NEGATIVE = [-6.0, 6.0, 6.0, -6.0]


def _pose_data(pose, table_center_x):
    q = pose.orientation
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    half_x = abs(math.cos(yaw)) * 0.21 + abs(math.sin(yaw)) * 0.15
    half_y = abs(math.sin(yaw)) * 0.21 + abs(math.cos(yaw)) * 0.15
    assert abs(pose.position.x - table_center_x) <= 0.4 + 0.35 + half_x
    return 1.2 - (pose.position.y + half_y)


def test_real_plugin_stops_at_both_tables_and_allows_departure():
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = "73"
    os.environ["ROS_DOMAIN_ID"] = "73"
    launch_log = tempfile.TemporaryFile(mode="w+", encoding="utf-8")
    launch = subprocess.Popen(
        ["ros2", "launch", "lab_cobot_gazebo", "world.launch.py", "gui:=false"],
        env=environment,
        stdout=launch_log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        text=True,
    )
    rclpy.init()
    node = rclpy.create_node("planar_safety_runtime_test")
    publisher = node.create_publisher(
        Float64MultiArray, "/wheel_velocity_controller/commands", 10
    )
    client = node.create_client(GetEntityState, "/gazebo/get_entity_state")

    def pose():
        request = GetEntityState.Request()
        request.name = "lab_cobot"
        request.reference_frame = "world"
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
        response = future.result()
        assert response is not None and response.success
        return response.state.pose

    def wait_for_robot():
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            request = GetEntityState.Request()
            request.name = "lab_cobot"
            request.reference_frame = "world"
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
            response = future.result()
            if response is not None and response.success:
                return
            time.sleep(0.1)
        raise AssertionError("lab_cobot entity did not spawn")

    def drive(values, seconds, stop=None):
        message = Float64MultiArray(data=values)
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.02)
            if stop is not None and stop(pose()):
                break
            time.sleep(0.03)
        publisher.publish(Float64MultiArray(data=[0.0] * 4))
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.02)

    try:
        assert client.wait_for_service(timeout_sec=30.0), "Gazebo state service missing"
        wait_for_robot()
        drive(WHEELS_X_POSITIVE, 10.0, lambda value: value.position.x >= 2.0)
        drive(WHEELS_Y_POSITIVE, 6.0)
        blocked_a = pose()
        clearance_a = _pose_data(blocked_a, 2.0)
        assert clearance_a >= 0.34
        drive(WHEELS_Y_POSITIVE, 2.0)
        assert _pose_data(pose(), 2.0) >= clearance_a - 0.002
        drive(WHEELS_Y_NEGATIVE, 2.0)
        assert _pose_data(pose(), 2.0) >= clearance_a + 0.2

        drive(WHEELS_X_NEGATIVE, 12.0, lambda value: value.position.x <= -2.0)
        drive(WHEELS_Y_POSITIVE, 6.0)
        blocked_b = pose()
        clearance_b = _pose_data(blocked_b, -2.0)
        assert clearance_b >= 0.34
        drive(WHEELS_Y_POSITIVE, 2.0)
        assert _pose_data(pose(), -2.0) >= clearance_b - 0.002
        drive(WHEELS_Y_NEGATIVE, 2.0)
        assert _pose_data(pose(), -2.0) >= clearance_b + 0.2
        assert launch.poll() is None, "Gazebo exited before safety checks completed"
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(launch.pid, signal.SIGINT)
        try:
            launch.wait(timeout=15.0)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)
            launch.wait(timeout=5.0)
        launch_log.seek(0)
        output = launch_log.read()
        launch_log.close()
        assert "lab_cobot_planar_drive" in output
