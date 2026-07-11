"""Honest full-stack launch test: no truth-pose perception, no attach bridge."""
from pathlib import Path
import subprocess
import time
import unittest

from gazebo_msgs.msg import ContactsState
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetLinkProperties
import launch
import launch_testing
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import Log
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray


TASK_TIMEOUT_SEC = 420.0
STATION_B_TABLE_MIN_X = -2.4
STATION_B_TABLE_MAX_X = -1.6
STATION_B_TABLE_FRONT_Y = 1.2
STATION_B_TABLE_BACK_Y = 1.8
TABLETOP_OBJECT_MIN_Z = 0.70
DL_MODEL_PATH = Path("~/lab_cobot_models/yolo_world_lab_slim.pt").expanduser()


def _gazebo_residue_error(pid_output):
    """Format the Gazebo residue cleanup diagnostic."""
    pids = " ".join(str(pid_output).split()) or "unknown"
    return (
        "检测到残留 gzserver 进程(pid: "
        f"{pids}),会污染本次 E2E。"
        "请先清理: pkill -9 -x gzserver; pkill -9 -x gzclient"
    )


def _dl_launch_arguments():
    return {
        "use_dl_perception": "true" if DL_MODEL_PATH.exists() else "false",
        "dl_device": "cpu",
        "dl_imgsz": "640",
    }


@pytest.mark.launch_test
def generate_test_description():
    # 预检:残留 gzserver 会让第二个 Gazebo 起不来,任务卡 NAV_TO_PICK
    # 直到 420s 超时(实测)。fail-fast 并给出清理命令,不浪费迷惑性等待。
    probe = subprocess.run(
        ["pgrep", "-x", "gzserver"], capture_output=True, text=True
    )
    if probe.returncode == 0:
        raise RuntimeError(_gazebo_residue_error(probe.stdout))
    bringup = get_package_share_directory("lab_cobot_bringup")
    nav = get_package_share_directory("lab_cobot_navigation")
    launch_file = Path(bringup) / "launch" / "lab_cobot.launch.py"
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_file)),
        launch_arguments={
            "gui": "false",
            "use_rviz": "false",
            "launch_mission": "true",
            "use_truth_pose": "false",
            "use_sim_attach": "false",
            "map": str(Path(nav) / "maps" / "map.yaml"),
            **_dl_launch_arguments(),
        }.items(),
    )
    return launch.LaunchDescription([stack, launch_testing.actions.ReadyToTest()]), {}


class TestHonestE2E(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_reaches_done_without_truth_pose_or_attach_bridge(self):
        node = rclpy.create_node("honest_e2e_probe")
        statuses = []
        detections = []
        model_states = []
        contact_statuses = []
        contact_snapshots = []
        runtime_logs = []
        object_detections = []
        pub = node.create_publisher(String, "/task/instruction", 10)
        node.create_subscription(
            String,
            "/task/status",
            lambda msg: statuses.append(str(msg.data)),
            10,
        )
        node.create_subscription(
            PoseStamped,
            "/perception/aruco_0/pose",
            lambda msg: detections.append(msg),
            10,
        )
        node.create_subscription(
            ModelStates,
            "/gazebo/model_states",
            lambda msg: model_states.append(msg),
            10,
        )
        node.create_subscription(
            String,
            "/gripper/contact/status",
            lambda msg: self._record_contact_status(
                contact_statuses,
                contact_snapshots,
                model_states,
                msg,
            ),
            10,
        )
        node.create_subscription(
            Log,
            "/rosout",
            lambda msg: self._record_runtime_log(runtime_logs, msg),
            10,
        )
        node.create_subscription(
            Detection3DArray,
            "/perception/objects",
            lambda msg: object_detections.append(msg),
            10,
        )
        # T-5 翻默认后默认路径为触觉抓取:记录双指与样件的真实接触对
        finger_touches = {"left": 0, "right": 0}
        node.create_subscription(
            ContactsState,
            "/gripper/left_finger_contacts",
            lambda msg: self._record_finger_touch(finger_touches, "left", msg),
            10,
        )
        node.create_subscription(
            ContactsState,
            "/gripper/right_finger_contacts",
            lambda msg: self._record_finger_touch(finger_touches, "right", msg),
            10,
        )

        try:
            self._assert_truth_pose_disabled(node)
            self._assert_attach_bridge_not_running(node)
            self._assert_wrist_detector_not_running(node)
            self._assert_refine_detect_disabled(node)
            started = time.monotonic()
            last_publish = 0.0
            while time.monotonic() - started < TASK_TIMEOUT_SEC:
                now = time.monotonic()
                if now - last_publish >= 1.0:
                    msg = String()
                    msg.data = "把样件从A送到B"
                    pub.publish(msg)
                    last_publish = now
                rclpy.spin_once(node, timeout_sec=0.2)
                if "DONE" in statuses:
                    self._assert_camera_detection_was_used(detections)
                    self._assert_dl_perception_mode(node, object_detections)
                    self._assert_object_is_on_station_b_table(
                        model_states,
                        runtime_logs,
                        contact_statuses,
                        contact_snapshots,
                    )
                    self._assert_only_sample_was_attached(contact_statuses)
                    self._assert_both_fingers_touched_sample(finger_touches)
                    self._assert_object_gravity_stays_enabled(node)
                    return
                if "FAILED" in statuses:
                    self.fail(
                        "honest e2e reached FAILED; "
                        f"statuses={statuses}; "
                        f"contact_statuses={contact_statuses[-8:]}; "
                        f"contact_snapshots={contact_snapshots[-8:]}; "
                        f"object_pose={self._latest_object_pose_text(model_states)}; "
                        f"object_twist={self._latest_object_twist_text(model_states)}; "
                        f"runtime_logs={runtime_logs[-20:]}"
                    )
            self.fail(f"timed out waiting for DONE; statuses={statuses}")
        finally:
            node.destroy_node()

    def _assert_truth_pose_disabled(self, node):
        client = node.create_client(GetParameters, "/aruco_detector/get_parameters")
        self.assertTrue(
            client.wait_for_service(timeout_sec=80.0),
            "aruco_detector parameter service did not appear",
        )
        request = GetParameters.Request()
        request.names = ["use_gazebo_model_pose"]
        # 刚启动时 DDS discovery 未稳定,单次 service call 偶发丢响应
        # (实测 20s 超时假失败),重试消解启动竞态。
        future = None
        for _attempt in range(3):
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=20.0)
            if future.done():
                break
        self.assertTrue(
            future is not None and future.done(),
            "timed out reading aruco_detector parameters",
        )
        values = future.result().values
        self.assertEqual(len(values), 1)
        self.assertEqual(values[0].type, ParameterType.PARAMETER_BOOL)
        self.assertFalse(values[0].bool_value)

    def _assert_attach_bridge_not_running(self, node):
        names = {name for name, _namespace in node.get_node_names_and_namespaces()}
        self.assertNotIn("gripper_attach_bridge", names)

    def _assert_wrist_detector_not_running(self, node):
        names = {name for name, _namespace in node.get_node_names_and_namespaces()}
        self.assertNotIn("wrist_aruco_detector", names)

    def _assert_refine_detect_disabled(self, node):
        client = node.create_client(GetParameters, "/mission_node/get_parameters")
        self.assertTrue(
            client.wait_for_service(timeout_sec=80.0),
            "mission_node parameter service did not appear",
        )
        request = GetParameters.Request()
        request.names = ["use_refine_detect"]
        future = None
        for _attempt in range(3):
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=20.0)
            if future.done():
                break
        self.assertTrue(
            future is not None and future.done(),
            "timed out reading mission_node parameters",
        )
        values = future.result().values
        self.assertEqual(len(values), 1)
        self.assertEqual(values[0].type, ParameterType.PARAMETER_BOOL)
        self.assertFalse(values[0].bool_value)

    def _assert_dl_perception_mode(self, node, object_detections):
        names = {name for name, _namespace in node.get_node_names_and_namespaces()}
        if DL_MODEL_PATH.exists():
            self.assertGreater(
                len(object_detections),
                0,
                "DL model exists but no /perception/objects messages were observed",
            )
            return
        self.assertNotIn("object_detector", names)
        print(f"DL perception disabled because model is missing: {DL_MODEL_PATH}")

    def _assert_camera_detection_was_used(self, detections):
        base_link_detections = [
            msg for msg in detections
            if msg.header.frame_id == "base_link"
        ]
        self.assertGreater(
            len(base_link_detections),
            0,
            "no base_link-frame ArUco detections were observed",
        )

    @staticmethod
    def _record_finger_touch(finger_touches, side, msg):
        """Count contact states where the finger touches the sample."""
        for state in msg.states:
            if (
                str(state.collision1_name).startswith("aruco_sample::")
                or str(state.collision2_name).startswith("aruco_sample::")
            ):
                finger_touches[side] += 1
                return

    def _assert_both_fingers_touched_sample(self, finger_touches):
        """Require real dual-finger contact on the tactile default path."""
        # T-4 判据口径:PICK 期间左右指都须出现 aruco_sample:: 接触对。
        # 隔空取物(封套命中但指面未接触)在默认路径必须不可能发生。
        self.assertGreater(
            finger_touches["left"],
            0,
            f"left finger never touched the sample; finger_touches={finger_touches}",
        )
        self.assertGreater(
            finger_touches["right"],
            0,
            f"right finger never touched the sample; finger_touches={finger_touches}",
        )

    def _assert_only_sample_was_attached(self, contact_statuses):
        attached = [
            status for status in contact_statuses
            if str(status).startswith("attached ")
        ]
        for status in attached:
            self.assertEqual(status, "attached aruco_sample")

    def _assert_object_is_on_station_b_table(
        self,
        model_states,
        runtime_logs=None,
        contact_statuses=None,
        contact_snapshots=None,
    ):
        self.assertGreater(len(model_states), 0, "no Gazebo model states received")
        latest = model_states[-1]
        self.assertIn("aruco_sample", latest.name)
        index = latest.name.index("aruco_sample")
        position = latest.pose[index].position
        context = (
            "object_pose=(%.3f, %.3f, %.3f); object_twist=%s; "
            "contact_statuses=%s; contact_snapshots=%s; runtime_logs=%s"
            % (
                position.x,
                position.y,
                position.z,
                self._latest_object_twist_text(model_states),
                (contact_statuses or [])[-8:],
                (contact_snapshots or [])[-8:],
                (runtime_logs or [])[-20:],
            )
        )
        self.assertGreaterEqual(position.x, STATION_B_TABLE_MIN_X, context)
        self.assertLessEqual(position.x, STATION_B_TABLE_MAX_X, context)
        self.assertGreaterEqual(position.y, STATION_B_TABLE_FRONT_Y, context)
        self.assertLessEqual(position.y, STATION_B_TABLE_BACK_Y, context)
        self.assertGreater(position.z, TABLETOP_OBJECT_MIN_Z, context)

    def _latest_object_pose_text(self, model_states):
        if not model_states:
            return "no_model_states"
        latest = model_states[-1]
        if "aruco_sample" not in latest.name:
            return "aruco_sample_missing"
        index = latest.name.index("aruco_sample")
        position = latest.pose[index].position
        return "(%.3f, %.3f, %.3f)" % (position.x, position.y, position.z)

    def _latest_object_twist_text(self, model_states):
        if not model_states:
            return "no_model_states"
        latest = model_states[-1]
        if "aruco_sample" not in latest.name:
            return "aruco_sample_missing"
        index = latest.name.index("aruco_sample")
        linear = latest.twist[index].linear
        angular = latest.twist[index].angular
        return "lin=(%.3f,%.3f,%.3f),ang=(%.3f,%.3f,%.3f)" % (
            linear.x,
            linear.y,
            linear.z,
            angular.x,
            angular.y,
            angular.z,
        )

    def _record_contact_status(
        self,
        contact_statuses,
        contact_snapshots,
        model_states,
        msg,
    ):
        status = str(msg.data)
        contact_statuses.append(status)
        contact_snapshots.append(
            "%s pose=%s twist=%s"
            % (
                status,
                self._latest_object_pose_text(model_states),
                self._latest_object_twist_text(model_states),
            )
        )

    def _record_runtime_log(self, runtime_logs, msg):
        if msg.name not in {"mission_node", "pick_place_node"}:
            return
        text = str(msg.msg)
        if "任务进行中,忽略新指令" in text:
            return
        interesting = (
            "Pick" in text
            or "Place" in text
            or "MoveIt target" in text
            or "夹爪" in text
            or "视觉" in text
            or "地图" in text
            or "任务" in text
            or "步骤" in text
        )
        if interesting:
            runtime_logs.append(f"{msg.name}: {text}")

    def _assert_object_gravity_stays_enabled(self, node):
        client = node.create_client(GetLinkProperties, "/gazebo/get_link_properties")
        self.assertTrue(
            client.wait_for_service(timeout_sec=10.0),
            "Gazebo link properties service did not appear",
        )
        request = GetLinkProperties.Request()
        request.link_name = "aruco_sample::link"
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        self.assertTrue(future.done(), "timed out reading object link properties")
        response = future.result()
        self.assertTrue(response.success, response.status_message)
        self.assertTrue(response.gravity_mode)
