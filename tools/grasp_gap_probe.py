#!/usr/bin/env python3
"""Measure held-sample grasp overlap and table clearance in Gazebo."""

import argparse
import math
import sys
import time


SAMPLE_LINK_SUFFIX = "aruco_sample::link"
LEFT_FINGER_LINK_SUFFIX = "lab_cobot::gripper_left_finger"
RIGHT_FINGER_LINK_SUFFIX = "lab_cobot::gripper_right_finger"
SAMPLE_HALF_HEIGHT_M = 0.035
FINGER_HALF_HEIGHT_M = 0.0375
STATION_A_TABLE_TOP_Z_M = 0.75
MIN_OVERLAP_MM = 28.0
MIN_TABLE_GAP_MM = 10.0
REQUIRED_PICK_START_COUNT = 1
STACK_READY_SETTLE_SEC = 2.0


def _parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Attach to a running lab_cobot stack, publish one task instruction, "
            "and capture grasp geometry at NAV_TO_PLACE."
        )
    )
    parser.add_argument(
        "--instruction",
        default="把样件从A送到B",
        help="Task instruction published until the mission starts.",
    )
    parser.add_argument(
        "--attach-only",
        action="store_true",
        help="Observe an already running mission without publishing an instruction.",
    )
    parser.add_argument(
        "--timeout-sec",
        type=float,
        default=420.0,
        help="Maximum time to wait for NAV_TO_PLACE.",
    )
    return parser.parse_args()


def _pose_by_suffix(msg, suffix):
    matches = [index for index, name in enumerate(msg.name) if name.endswith(suffix)]
    if len(matches) != 1:
        return None
    return msg.pose[matches[0]]


def _xyz(pose):
    point = pose.position
    return point.x, point.y, point.z


def _touches_sample(msg):
    for state in getattr(msg, "states", []):
        names = (str(state.collision1_name), str(state.collision2_name))
        if any(name.startswith("aruco_sample::") for name in names):
            return True
    return False


def _evaluate(snapshot, pick_start_count, tcp_map_z):
    sample_pose = _pose_by_suffix(snapshot, SAMPLE_LINK_SUFFIX)
    left_pose = _pose_by_suffix(snapshot, LEFT_FINGER_LINK_SUFFIX)
    right_pose = _pose_by_suffix(snapshot, RIGHT_FINGER_LINK_SUFFIX)
    missing = []
    for label, pose in (
        ("sample", sample_pose),
        ("left_finger", left_pose),
        ("right_finger", right_pose),
    ):
        if pose is None:
            missing.append(label)
    if missing:
        raise RuntimeError("Missing or ambiguous link states: " + ", ".join(missing))

    sample_xyz = _xyz(sample_pose)
    left_xyz = _xyz(left_pose)
    right_xyz = _xyz(right_pose)
    finger_xyz = tuple(
        (left_value + right_value) / 2.0
        for left_value, right_value in zip(left_xyz, right_xyz)
    )
    overlap_mm = 1000.0 * (
        sample_xyz[2] + SAMPLE_HALF_HEIGHT_M
        - (finger_xyz[2] - FINGER_HALF_HEIGHT_M)
    )
    table_gap_mm = 1000.0 * (
        finger_xyz[2] - FINGER_HALF_HEIGHT_M - STATION_A_TABLE_TOP_Z_M
    )
    # Gazebo world 与 map 在本场景共享竖直原点;AMCL 只校正平面 x/y/yaw。
    held_offset_z = sample_xyz[2] - tcp_map_z
    checks = {
        "overlap": overlap_mm >= MIN_OVERLAP_MM,
        "table_gap": table_gap_mm > MIN_TABLE_GAP_MM,
        "pick_start": pick_start_count == REQUIRED_PICK_START_COUNT,
    }
    return {
        "sample_xyz": sample_xyz,
        "left_xyz": left_xyz,
        "right_xyz": right_xyz,
        "finger_xyz": finger_xyz,
        "tcp_map_z": tcp_map_z,
        "held_offset_z": held_offset_z,
        "overlap_mm": overlap_mm,
        "table_gap_mm": table_gap_mm,
        "pick_start_count": pick_start_count,
        "checks": checks,
    }


def _format_xyz(values):
    return "(%.6f, %.6f, %.6f)" % values


def _print_result(result):
    print("SAMPLE_CENTER_WORLD", _format_xyz(result["sample_xyz"]), flush=True)
    print("LEFT_FINGER_CENTER_WORLD", _format_xyz(result["left_xyz"]), flush=True)
    print("RIGHT_FINGER_CENTER_WORLD", _format_xyz(result["right_xyz"]), flush=True)
    print("FINGER_CENTER_WORLD", _format_xyz(result["finger_xyz"]), flush=True)
    print("GRIPPER_TCP_MAP_Z", "%.6f" % result["tcp_map_z"], flush=True)
    print(
        "HELD_SAMPLE_CENTER_FROM_TCP_Z",
        "%.6f" % result["held_offset_z"],
        flush=True,
    )
    print("OVERLAP_MM", "%.3f" % result["overlap_mm"], flush=True)
    print("TABLE_GAP_MM", "%.3f" % result["table_gap_mm"], flush=True)
    print("PICK_START_COUNT", result["pick_start_count"], flush=True)
    print(
        "CHECK_OVERLAP",
        "PASS" if result["checks"]["overlap"] else "FAIL",
        ">= %.1fmm" % MIN_OVERLAP_MM,
        flush=True,
    )
    print(
        "CHECK_TABLE_GAP",
        "PASS" if result["checks"]["table_gap"] else "FAIL",
        "> %.1fmm" % MIN_TABLE_GAP_MM,
        flush=True,
    )
    print(
        "CHECK_PICK_START",
        "PASS" if result["checks"]["pick_start"] else "FAIL",
        "== %d" % REQUIRED_PICK_START_COUNT,
        flush=True,
    )
    passed = all(result["checks"].values())
    print("RESULT", "PASS" if passed else "FAIL", flush=True)
    return passed


def _run(args):
    import rclpy
    from gazebo_msgs.msg import ContactsState, LinkStates, ModelStates
    from rcl_interfaces.msg import Log
    from rclpy.duration import Duration
    from rclpy.node import Node
    from std_msgs.msg import String
    import tf2_ros

    class GraspGapProbe(Node):
        def __init__(self):
            super().__init__("grasp_gap_probe")
            self.latest_links = None
            self.snapshot = None
            self.capture_requested = False
            self.statuses = []
            self.pick_start_count = 0
            self.latest_models = None
            self.max_sample_speed = 0.0
            self.max_speed_pose = None
            self.left_contact_hits = 0
            self.right_contact_hits = 0
            self.contact_statuses = []
            self.runtime_logs = []
            self.instruction_pub = self.create_publisher(
                String, "/task/instruction", 10
            )
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.create_subscription(
                LinkStates, "/gazebo/link_states", self._links_cb, 10
            )
            self.create_subscription(
                ModelStates, "/gazebo/model_states", self._models_cb, 10
            )
            self.create_subscription(String, "/task/status", self._status_cb, 10)
            self.create_subscription(
                String,
                "/gripper/contact/status",
                self._contact_status_cb,
                10,
            )
            self.create_subscription(
                ContactsState,
                "/gripper/left_finger_contacts",
                self._left_contacts_cb,
                10,
            )
            self.create_subscription(
                ContactsState,
                "/gripper/right_finger_contacts",
                self._right_contacts_cb,
                10,
            )
            self.create_subscription(Log, "/rosout", self._rosout_cb, 10)

        def _links_cb(self, msg):
            self.latest_links = msg
            if self.capture_requested and self.snapshot is None:
                self.snapshot = msg

        def _status_cb(self, msg):
            status = str(msg.data)
            self.statuses.append(status)
            print("TASK_STATUS", status, flush=True)
            if status == "NAV_TO_PLACE" and self.snapshot is None:
                self.capture_requested = True

        def _models_cb(self, msg):
            self.latest_models = msg
            if "aruco_sample" not in msg.name:
                return
            index = msg.name.index("aruco_sample")
            twist = msg.twist[index]
            speed = math.sqrt(
                twist.linear.x * twist.linear.x
                + twist.linear.y * twist.linear.y
                + twist.linear.z * twist.linear.z
            )
            if speed > self.max_sample_speed:
                self.max_sample_speed = speed
                self.max_speed_pose = _xyz(msg.pose[index])

        def _contact_status_cb(self, msg):
            self.contact_statuses.append(str(msg.data))

        def _left_contacts_cb(self, msg):
            if _touches_sample(msg):
                self.left_contact_hits += 1

        def _right_contacts_cb(self, msg):
            if _touches_sample(msg):
                self.right_contact_hits += 1

        def _rosout_cb(self, msg):
            text = str(msg.msg)
            logger_name = str(msg.name)
            if "pick_place_node" in logger_name or "mission_node" in logger_name:
                self.runtime_logs.append(f"{logger_name}: {text}")
            if "Pick start" not in text:
                return
            self.pick_start_count += 1
            print("PICK_START_LOG", self.pick_start_count, text, flush=True)

        def publish_instruction(self, instruction):
            msg = String()
            msg.data = instruction
            self.instruction_pub.publish(msg)

        def stack_ready(self):
            publisher_topics = (
                "/task/status",
                "/gazebo/link_states",
                "/gazebo/model_states",
            )
            if any(not self.get_publishers_info_by_topic(topic) for topic in publisher_topics):
                return False
            if not self.get_subscriptions_info_by_topic("/task/instruction"):
                return False
            paired_topics = (
                "/gripper/left_finger_contacts",
                "/gripper/right_finger_contacts",
                "/gripper/contact/status",
            )
            for topic in paired_topics:
                if not self.get_publishers_info_by_topic(topic):
                    return False
                # 本探针占一个订阅;第二个订阅必须来自运行时 driver。
                if len(self.get_subscriptions_info_by_topic(topic)) < 2:
                    return False
            return True

        def tcp_map_z(self):
            transform = self.tf_buffer.lookup_transform(
                "map",
                "gripper_tcp",
                rclpy.time.Time(),
                timeout=Duration(seconds=2.0),
            )
            return transform.transform.translation.z

        def print_diagnostics(self):
            latest_sample = None
            if self.latest_models is not None and "aruco_sample" in self.latest_models.name:
                index = self.latest_models.name.index("aruco_sample")
                latest_sample = _xyz(self.latest_models.pose[index])
            print("CONTACT_HITS", self.left_contact_hits, self.right_contact_hits, flush=True)
            print("CONTACT_STATUSES", self.contact_statuses[-12:], flush=True)
            print("RUNTIME_LOGS", self.runtime_logs[-40:], flush=True)
            print("MAX_SAMPLE_SPEED", "%.6f" % self.max_sample_speed, flush=True)
            print("MAX_SPEED_SAMPLE_POSE", self.max_speed_pose, flush=True)
            print("LATEST_SAMPLE_POSE", latest_sample, flush=True)

    rclpy.init()
    node = GraspGapProbe()
    try:
        deadline = time.monotonic() + args.timeout_sec
        last_publish = 0.0
        ready_since = None
        ready_announced = False
        while time.monotonic() < deadline:
            now = time.monotonic()
            if node.stack_ready():
                if ready_since is None:
                    ready_since = now
            else:
                ready_since = None
                ready_announced = False
            ready_to_publish = (
                ready_since is not None
                and now - ready_since >= STACK_READY_SETTLE_SEC
            )
            if ready_to_publish and not ready_announced:
                print("STACK_READY", flush=True)
                ready_announced = True
            if (
                not args.attach_only
                and not node.statuses
                and ready_to_publish
                and now - last_publish >= 1.0
            ):
                node.publish_instruction(args.instruction)
                last_publish = now
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.snapshot is not None:
                # Drain rosout briefly so the final Pick start log cannot lag the status callback.
                drain_deadline = time.monotonic() + 0.5
                while time.monotonic() < drain_deadline:
                    rclpy.spin_once(node, timeout_sec=0.05)
                result = _evaluate(
                    node.snapshot,
                    node.pick_start_count,
                    node.tcp_map_z(),
                )
                node.print_diagnostics()
                return 0 if _print_result(result) else 1
            if "FAILED" in node.statuses:
                node.print_diagnostics()
                print("RESULT FAIL mission reached FAILED", flush=True)
                return 1
        node.print_diagnostics()
        print(
            "RESULT FAIL timed out waiting for NAV_TO_PLACE; statuses=%s"
            % node.statuses,
            flush=True,
        )
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    """Run the grasp gap probe against a live simulation stack."""
    args = _parse_args()
    try:
        return _run(args)
    except (KeyboardInterrupt, RuntimeError) as exc:
        print("RESULT FAIL", str(exc), flush=True)
        return 2


if __name__ == "__main__":
    sys.exit(main())
