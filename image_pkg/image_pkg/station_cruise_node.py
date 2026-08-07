#!/usr/bin/env python3
"""Navigate the robot through all work zones with a configurable dwell time."""
import math
import time
import json

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_ROUTE = [
    "home", "station_a", "inspection_zone", "tooling_zone", "aging_zone",
    "station_b", "home",
]
DEFAULT_WAYPOINTS = [
    "home=4.50,-4.20,0.0",
    "station_a=-4.30,2.38,1.57079632679",
    "inspection_zone=4.10,1.10,1.57079632679",
    "tooling_zone=-4.10,-3.40,1.57079632679",
    "aging_zone=0.20,3.10,1.57079632679",
    "station_b=0.30,-3.11,1.57079632679",
]

# The five work areas occupy the central part of the 14 x 14 m map.  The
# odometry fallback has no Nav2 costmap, so it must never connect workstations
# with a straight line through tables or the high-voltage fence.  These are
# collision-clear intermediate points around the perimeter / aisle.  End
# points remain the documented camera-facing approach poses above.
DEFAULT_SAFE_CORRIDORS = [
    "home>station_a=6.00,-4.20;6.00,5.70;-5.70,5.70;-5.70,2.38",
    "station_a>inspection_zone=-5.70,2.38;-5.70,5.70;6.00,5.70;6.00,-0.30;4.10,-0.30",
    "inspection_zone>tooling_zone=6.00,1.10;6.00,-4.80;-5.70,-4.80;-5.70,-3.40",
    "tooling_zone>aging_zone=-5.70,-3.40;-5.70,3.10;-1.00,3.10",
    "aging_zone>station_b=-1.00,3.10;-1.00,-4.80;0.30,-4.80",
    "station_b>home=4.50,-3.11",
]

STATION_LABELS = {
    "station_a": {"aruco_sample", "material_spare_igbt", "material_grease_can"},
    "tooling_zone": {"tooling_fixture_box", "tooling_hand_tools"},
    "aging_zone": {"aging_rack"},
    "station_b": {"board_test_fixture"},
    "inspection_zone": {"high_voltage_probe_kit"},
}
LABEL_TO_MODEL = {
    "aruco_sample": "aruco_sample", "material_spare_igbt": "material_spare_igbt",
    "material_grease_can": "material_grease_can", "tooling_fixture_box": "tooling_fixture_box",
    "tooling_hand_tools": "tooling_hand_tools", "aging_rack": "aging_rack",
    "board_test_fixture": "board_test_fixture", "high_voltage_probe_kit": "high_voltage_probe_kit",
}


class StationCruise(Node):
    """Run the fixed work-zone route and dwell at each non-home stop."""

    def __init__(self):
        super().__init__("image_pkg_station_cruise")
        self.declare_parameter("dwell_seconds", 5.0)
        self.declare_parameter("navigation_timeout_seconds", 120.0)
        self.declare_parameter("odom_navigation_timeout_seconds", 120.0)
        self.declare_parameter("odom_fallback_on_nav_failure", True)
        self.declare_parameter("odom_goal_tolerance_m", 0.18)
        self.declare_parameter("odom_max_speed_mps", 0.35)
        self.declare_parameter("odom_max_yaw_rate_rps", 0.8)
        self.declare_parameter("odom_yaw_tolerance_rad", 0.06)
        self.declare_parameter("route", DEFAULT_ROUTE)
        self.declare_parameter("waypoints", DEFAULT_WAYPOINTS)
        self.declare_parameter("safe_corridors", DEFAULT_SAFE_CORRIDORS)
        self.declare_parameter("detection_topic", "/yolo/detections")
        self.declare_parameter("require_station_detection", True)
        self.declare_parameter("per_object_observe_seconds", 3.0)
        self.dwell_seconds = float(self.get_parameter("dwell_seconds").value)
        self.timeout_seconds = float(
            self.get_parameter("navigation_timeout_seconds").value)
        self.odom_timeout_seconds = float(
            self.get_parameter("odom_navigation_timeout_seconds").value)
        if (self.dwell_seconds < 0.0 or self.timeout_seconds <= 0.0
                or self.odom_timeout_seconds <= 0.0):
            raise ValueError(
                "dwell_seconds must be >= 0 and navigation timeouts must be > 0")
        self.route = [str(name) for name in self.get_parameter("route").value]
        self.waypoints = _parse_waypoints(self.get_parameter("waypoints").value)
        self.safe_corridors = _parse_corridors(
            self.get_parameter("safe_corridors").value)
        unknown = [name for name in self.route if name not in self.waypoints]
        if unknown:
            raise ValueError(f"route contains unknown waypoint(s): {unknown}")
        self.status_pub = self.create_publisher(String, "/image_pkg/cruise/status", 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.arm_pub = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10)
        self.odom = None
        self.truth_positions = {}
        self.model_poses = {}
        self.detected_labels = set()
        self.create_subscription(Odometry, "/odom", self._odom_cb, 20)
        self.create_subscription(ModelStates, "/gazebo/model_states", self._models_cb, 10)
        self.create_subscription(String, self.get_parameter("detection_topic").value,
                                 self._detections_cb, 10)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def run(self):
        self._status("WAITING_FOR_NAV2")
        while rclpy.ok() and not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("waiting for navigate_to_pose action server")
        if not rclpy.ok():
            return False
        for index, name in enumerate(self.route):
            # The robot normally starts at home.  Nav2 may legitimately reject
            # a zero-length NavigateToPose goal; treat the initial home point
            # as already reached so it cannot abort the whole cruise.
            if index == 0 and name == "home":
                self._status("ARRIVED:home:initial")
                continue
            self._status(f"NAVIGATING:{name}")
            previous_name = self.route[index - 1]
            reached = self._navigate(name)
            if not reached and bool(self.get_parameter("odom_fallback_on_nav_failure").value):
                self._status(f"NAV2_FAILED_FALLBACK_ODOM:{name}")
                reached = self._navigate_odom(name, previous_name)
            if not reached:
                self._status(f"FAILED:navigation:{name}")
                return False
            if name != "home":
                self._status(f"ARRIVED:{name}:dwell={self.dwell_seconds:.1f}s")
                self._observe_station(name)
            elif index == len(self.route) - 1:
                self._status("DONE:home")
        return True

    def _navigate(self, name):
        future = self.nav_client.send_goal_async(_goal(name, self.waypoints[name], self))
        if not _wait_for_future(self, future, self.timeout_seconds):
            return False
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False
        result_future = goal_handle.get_result_async()
        if not _wait_for_future(self, result_future, self.timeout_seconds):
            goal_handle.cancel_goal_async()
            return False
        return result_future.result().status == GoalStatus.STATUS_SUCCEEDED

    def _odom_cb(self, msg):
        self.odom = msg

    def _models_cb(self, msg):
        self.model_poses = {name: pose for name, pose in zip(msg.name, msg.pose)}
        self.truth_positions = {
            name: (pose.position.x, pose.position.y, pose.position.z)
            for name, pose in zip(msg.name, msg.pose)
        }

    def _detections_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            self.detected_labels.update(
                str(item.get("label", "")).strip()
                for item in payload.get("detections", []))
        except (TypeError, ValueError, json.JSONDecodeError):
            return

    def _navigate_odom(self, name, previous_name):
        """Use a collision-clear corridor when Nav2 lacks a map->odom TF."""
        target_x, target_y, target_yaw = self.waypoints[name]
        tolerance = float(self.get_parameter("odom_goal_tolerance_m").value)
        max_speed = float(self.get_parameter("odom_max_speed_mps").value)
        # Nav2 can be unavailable (for example no map->odom transform) while
        # odometry fallback remains able to cover a long inter-station leg.
        # Keep its timeout independent from the short Nav2 failover timeout.
        deadline = time.monotonic() + self.odom_timeout_seconds
        path = self.safe_corridors.get((previous_name, name), [])
        path = [*path, (target_x, target_y)]
        for point_x, point_y in path:
            if not self._drive_odom_point(point_x, point_y, tolerance, max_speed, deadline):
                self._stop_robot()
                return False
        # At the final camera-facing approach pose, turn the robot to the
        # configured yaw.  This is essential for seeing the table/ground item.
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom is None:
                continue
            pose = self.odom.pose.pose
            yaw_error = _wrap_angle(target_yaw - _yaw_from_quaternion(pose.orientation))
            if abs(yaw_error) <= float(self.get_parameter("odom_yaw_tolerance_rad").value):
                self._stop_robot()
                return True
            command = Twist()
            command.angular.z = max(-float(self.get_parameter("odom_max_yaw_rate_rps").value),
                                    min(float(self.get_parameter("odom_max_yaw_rate_rps").value), 1.6 * yaw_error))
            self.cmd_pub.publish(command)
        self._stop_robot()
        return False

    def _drive_odom_point(self, target_x, target_y, tolerance, max_speed, deadline):
        """Translate holonomically to one clear intermediate point."""
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom is None:
                continue
            pose = self.odom.pose.pose
            dx, dy = target_x - pose.position.x, target_y - pose.position.y
            distance = math.hypot(dx, dy)
            if distance <= tolerance:
                self._stop_robot()
                return True
            yaw = _yaw_from_quaternion(pose.orientation)
            # Convert world/odom vector to body-frame holonomic velocity.
            speed = min(max_speed, max(0.06, 0.55 * distance))
            command = Twist()
            command.linear.x = speed * (math.cos(yaw) * dx + math.sin(yaw) * dy) / distance
            command.linear.y = speed * (-math.sin(yaw) * dx + math.cos(yaw) * dy) / distance
            self.cmd_pub.publish(command)
        return False

    def _stop_robot(self):
        self.cmd_pub.publish(Twist())

    def _aim_wrist_at_truth(self, label):
        """Aim the wrist camera at one object from same-frame Gazebo truth.

        The object and robot poses come from one ``ModelStates`` message, so
        their relative vector does not mix Gazebo world and odom coordinates.
        The vector controls shoulder-pan / wrist-yaw (bearing), arm elevation
        (object height), and reach profile (range).  This is deliberately a
        per-object trajectory rather than a single station-wide fixed pose.
        """
        object_pose = self.model_poses.get(LABEL_TO_MODEL[label])
        robot_pose = self.model_poses.get("lab_cobot")
        if object_pose is None or robot_pose is None:
            self.get_logger().warning(f"No Gazebo truth pose for observation target {label}")
            return False
        yaw = _yaw_from_quaternion(robot_pose.orientation)
        dx = object_pose.position.x - robot_pose.position.x
        dy = object_pose.position.y - robot_pose.position.y
        # World -> mobile-base planar coordinates. +X is forward.
        forward = math.cos(yaw) * dx + math.sin(yaw) * dy
        lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
        distance = math.hypot(forward, lateral)
        bearing = math.atan2(lateral, max(forward, 0.05))
        ground_target = object_pose.position.z < 0.25

        msg = JointTrajectory()
        msg.joint_names = [
            "ur_shoulder_pan_joint", "ur_shoulder_lift_joint", "ur_elbow_joint",
            "ur_wrist_1_joint", "ur_wrist_2_joint", "ur_wrist_3_joint"]
        point = JointTrajectoryPoint()
        # Keep a safe, downward-looking base posture.  Bearing and distance
        # make the optical axis sweep each object rather than preserving the
        # old one-size-fits-all pose.
        pan = max(-0.55, min(0.55, bearing * 0.75))
        reach_adjust = max(-0.18, min(0.18, (distance - 1.30) * 0.22))
        if ground_target:
            point.positions = [pan, -1.10 - reach_adjust, 1.90 + reach_adjust,
                               -2.35, -1.57, -pan]
        else:
            point.positions = [pan, -1.57 - reach_adjust, 1.61 + reach_adjust,
                               -1.57, -1.45, -pan]
        point.time_from_start = Duration(sec=3)
        msg.points = [point]
        self.arm_pub.publish(msg)
        time.sleep(3.2)
        self.get_logger().info(
            "OBSERVATION_TARGET:%s range=%.3fm bearing=%.3frad height=%.3fm"
            % (label, distance, bearing, object_pose.position.z))
        return True

    def _observe_station(self, station):
        expected = STATION_LABELS.get(station, set())
        observed = set()
        for label in sorted(expected):
            self.detected_labels.clear()
            self._status(f"OBSERVING:{station}:{label}")
            if not self._aim_wrist_at_truth(label):
                continue
            deadline = time.monotonic() + max(
                self.dwell_seconds, float(self.get_parameter("per_object_observe_seconds").value))
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if label in self.detected_labels:
                    observed.add(label)
                    # Retain a short stable interval so RGB-D/YOLO can emit
                    # a timestamped 3-D estimate for this same observation.
                    time.sleep(0.5)
                    break
        missing = expected - observed
        if missing:
            self._status(f"OBSERVATION_INCOMPLETE:{station}:missing={','.join(sorted(missing))}")
        else:
            self._status(f"OBSERVATION_CONFIRMED:{station}")

    def _status(self, text):
        message = String()
        message.data = text
        self.status_pub.publish(message)
        self.get_logger().info(text)


def _parse_waypoints(entries):
    waypoints = {}
    for entry in entries:
        name, separator, values = str(entry).partition("=")
        fields = values.split(",") if separator else []
        if not name.strip() or len(fields) != 3:
            raise ValueError(f"invalid waypoint entry: {entry!r}")
        try:
            waypoints[name.strip()] = tuple(float(value) for value in fields)
        except ValueError as exc:
            raise ValueError(f"invalid waypoint coordinates: {entry!r}") from exc
    return waypoints


def _parse_corridors(entries):
    corridors = {}
    for entry in entries:
        legs, separator, values = str(entry).partition("=")
        start, arrow, end = legs.partition(">")
        if not separator or not arrow or not start.strip() or not end.strip():
            raise ValueError(f"invalid safe corridor entry: {entry!r}")
        points = []
        for pair in values.split(";"):
            fields = pair.split(",")
            if len(fields) != 2:
                raise ValueError(f"invalid safe corridor point: {pair!r}")
            try:
                points.append((float(fields[0]), float(fields[1])))
            except ValueError as exc:
                raise ValueError(f"invalid safe corridor point: {pair!r}") from exc
        corridors[(start.strip(), end.strip())] = points
    return corridors


def _goal(name, waypoint, node):
    x, y, yaw = waypoint
    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = "map"
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y
    goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return goal


def _yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _wait_for_future(node, future, timeout_seconds):
    deadline = time.monotonic() + timeout_seconds
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    return future.done()


def main(args=None):
    rclpy.init(args=args)
    cruise = None
    try:
        cruise = StationCruise()
        cruise.run()
    except KeyboardInterrupt:
        if cruise is not None:
            cruise._status("CANCELLED")
    finally:
        if cruise is not None:
            cruise.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
