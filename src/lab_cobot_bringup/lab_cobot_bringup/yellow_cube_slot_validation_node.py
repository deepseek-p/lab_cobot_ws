#!/usr/bin/env python3
"""Independent validation chain for yellow cube insertion into the middle rack slot."""
from __future__ import annotations

import math
import time
from threading import Lock, Thread

import rclpy
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from lab_cobot_bringup.mission_node import (
    NAV_ACTIVE_POLL_SEC,
    NAV_HANDOFF_STOP_SEC,
    NAV_SERVER_WAIT_SEC,
    NAV_TF_READY_POLL_SEC,
    navigation_goals_for_station,
    station_dock_velocity_for_base,
    yaw_to_quat,
)
from lab_cobot_manipulation.pick_place_node import (
    DEFAULT_APPROACH_TOLERANCE_ORIENTATION,
    DEFAULT_APPROACH_TOLERANCE_POSITION,
    DEFAULT_GRASP_TOLERANCE_ORIENTATION,
    DEFAULT_GRASP_TOLERANCE_POSITION,
    DEFAULT_MOVE_TIMEOUT_SEC,
    DOWN_QUAT,
    GRIPPER_TCP_LINK,
    POST_ATTACH_HOLD_CONFIRM_TIMEOUT_SEC,
    PickPlace,
)
from lab_cobot_manipulation.scene_obstacles import (
    CARRIED_SAMPLE_BOX_ID,
    make_attach_scene,
    make_detach_scene,
    make_remove_world_box_scene,
    make_world_box_scene,
)
from lab_cobot_manipulation.yellow_cube_slot_config import (
    AGING_RACK_COLLISIONS,
    AGING_RACK_WORLD_POSE,
    AGING_ZONE_TABLE_WORLD,
    BASE_LINK_WORLD_Z,
    CARTESIAN_EEF_STEP,
    CARTESIAN_FRACTION_MIN,
    COMMAND,
    DOWN_QUAT_XYZW,
    FAILURE_STATES,
    MATERIAL_CUBE_YELLOW_COLLISION_SIZE,
    PLACE_SETTLE_SEC,
    PLACE_VALIDATION_XY_TOLERANCE,
    PLACE_VALIDATION_YAW_TOLERANCE,
    PLACE_VALIDATION_Z_TOLERANCE,
    SLOT_ALIGNMENT_YAW,
    SLOT_ALIGNMENT_QUAT_XYZW,
    SLOT_QUAT_XYZW,
    STATION_A_TABLE_WORLD,
    STATUS_SEQUENCE,
    STATUS_TOPIC,
    TARGET_OBJECT,
    TARGET_TOPIC,
    YELLOW_CUBE_SLOT_VALIDATION_BASE_POSE,
    YELLOW_CUBE_TACTILE_MAX_POSITION,
    YELLOW_CUBE_TACTILE_START_POSITION,
    YELLOW_CUBE_TACTILE_STEP_POSITION,
    YELLOW_NAV_ACTIVE_TIMEOUT_SEC,
    YELLOW_NAV_TF_READY_TIMEOUT_SEC,
    YELLOW_STATION_DOCK_MAX_SEC,
    YELLOW_STATION_DOCK_SETTLE_SEC,
    YELLOW_STATION_A_FINE_DOCK_LINEAR_GAIN,
    YELLOW_STATION_A_FINE_DOCK_MAX_LINEAR_SPEED,
    YELLOW_STATION_A_FINE_DOCK_MIN_SAFE_X,
    YELLOW_STATION_A_FINE_DOCK_PROGRESS_EPS,
    YELLOW_STATION_A_FINE_DOCK_STALLED_SEC,
    YELLOW_STATION_A_FINE_DOCK_TARGET_CUBE_BASE_X,
    YELLOW_STATION_A_FINE_DOCK_TARGET_CUBE_BASE_Y,
    YELLOW_STATION_A_FINE_DOCK_TIMEOUT_SEC,
    YELLOW_STATION_A_FINE_DOCK_X_TOLERANCE,
    YELLOW_STATION_A_FINE_DOCK_Y_TOLERANCE,
    cube_center_world,
    grasp_tcp_world,
    insert_final_tcp_world,
    insert_stage1_tcp_world,
    insert_stage2_tcp_world,
    insert_shallow_tcp_world,
    lift_tcp_world,
    middle_slot_bounds_world,
    middle_slot_center_world,
    pre_grasp_high_tcp_world,
    pre_grasp_tcp_world,
    pre_slot_high_tcp_world,
    pre_slot_tcp_world,
    vertical_retreat_tcp_world,
    world_to_base,
)

BASE_FOOTPRINT_ENTITY = "lab_cobot::base_footprint"
STATION_A_RETREAT_TARGET_CLEARANCE = 0.600
STATION_A_RETREAT_MAX_LINEAR_SPEED = 0.080
STATION_A_RETREAT_MIN_LINEAR_SPEED = 0.030
STATION_A_RETREAT_LINEAR_GAIN = 0.55
STATION_A_RETREAT_TIMEOUT_SEC = 120.0
STATION_A_RETREAT_STALLED_SEC = 6.0
STATION_A_RETREAT_PROGRESS_EPS = 0.010
STATION_A_RETREAT_WRONG_DIRECTION_EPS = 0.030
NAV_RUNNING_LOG_INTERVAL_SEC = 30.0
AGING_PLACE_TARGET_SLOT_BASE_X = 0.735
AGING_PLACE_SLOT_BASE_X_MIN = 0.720
AGING_PLACE_SLOT_BASE_X_MAX = 0.750
AGING_PLACE_SLOT_BASE_Y_TOLERANCE = 0.040
AGING_PLACE_FINE_DOCK_MAX_SPEED = 0.050
AGING_PLACE_FINE_DOCK_MIN_SPEED = 0.018
AGING_PLACE_FINE_DOCK_LINEAR_GAIN = 0.45
AGING_PLACE_FINE_DOCK_TIMEOUT_SEC = 120.0
AGING_PLACE_FINE_DOCK_STALLED_SEC = 10.0
AGING_PLACE_FINE_DOCK_PROGRESS_EPS = 0.006
AGING_PLACE_FINE_DOCK_WRONG_DIRECTION_EPS = 0.020
AGING_PLACE_FINE_DOCK_MAX_APPROACH_DISTANCE = 0.250
AGING_PLACE_FINE_DOCK_MIN_TABLE_CLEARANCE = 0.100
AGING_PLACE_FINE_DOCK_FOOTPRINT = (
    (0.28, 0.31),
    (0.28, -0.31),
    (-0.28, -0.31),
    (-0.28, 0.31),
)


def _yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _angle_wrap(angle: float) -> float:
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def _symmetric_yaw_error(
    actual_yaw: float,
    expected_yaw: float,
    symmetry_period: float = math.pi / 2.0,
) -> float:
    return min(
        abs(_angle_wrap(float(actual_yaw) - (float(expected_yaw) + k * symmetry_period)))
        for k in range(4)
    )


def _pose_stamped(x: float, y: float, yaw: float) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    qx, qy, qz, qw = yaw_to_quat(float(yaw))
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    return msg


class YellowCubeSlotValidation(PickPlace):
    """Validation-only chain.  It does not use perception or formal mission topics."""

    def __init__(self):
        super().__init__(
            target_object=TARGET_OBJECT,
            use_tactile_grasp=True,
            use_planning_scene_obstacles=True,
            tactile_start_position=YELLOW_CUBE_TACTILE_START_POSITION,
            tactile_step_position=YELLOW_CUBE_TACTILE_STEP_POSITION,
            tactile_max_position=YELLOW_CUBE_TACTILE_MAX_POSITION,
            tactile_log_prefix="YELLOW_CUBE_SLOT_TACTILE",
            gripper_open_positions=[0.0, 0.0],
            expected_object_width_mm=MATERIAL_CUBE_YELLOW_COLLISION_SIZE[1] * 1000.0,
            pick_tcp_z_clearance=0.018,
            node_name="yellow_cube_slot_validation_node",
        )
        self._status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self._target_sub = self.create_subscription(
            String, TARGET_TOPIC, self._on_target, 10
        )
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._gazebo_state = self.create_client(GetEntityState, "/gazebo/get_entity_state")
        self._bt_state_client = self.create_client(GetState, "/bt_navigator/get_state")
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._nav = BasicNavigator()
        self._running_lock = Lock()
        self._running = False
        self.declare_parameter("stop_after_station_a_fine_dock", False)
        self.declare_parameter("stop_after_yellow_lift", False)
        self.declare_parameter("stop_after_nav_aging_zone", False)
        self.declare_parameter("stop_after_nav_leg1_start", False)
        self.declare_parameter("stop_after_aging_fine_dock", False)
        self.declare_parameter("stop_after_pre_slot_high", False)
        self._stop_after_nav_leg1_start_reached = False
        self._publish_status("YELLOW_SLOT_TASK_READY")

    def _on_target(self, msg: String) -> None:
        if str(msg.data).strip() != COMMAND:
            return
        with self._running_lock:
            if self._running:
                self.get_logger().warn("yellow cube slot validation already running")
                return
            self._running = True
        Thread(target=self._run_guarded, daemon=True).start()

    def _run_guarded(self) -> None:
        try:
            self._run_sequence()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"yellow cube slot validation crashed: {exc}")
            self._publish_status("FAILED_PLACE_VALIDATION")
        finally:
            with self._running_lock:
                self._running = False

    def _publish_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)
        self.get_logger().info(status)

    def _gazebo_entity_pose(self, entity_name: str):
        if not self._gazebo_state.wait_for_service(timeout_sec=0.05):
            return None
        request = GetEntityState.Request()
        request.name = entity_name
        request.reference_frame = "world"
        future = self._gazebo_state.call_async(request)
        deadline = time.monotonic() + 0.20
        while time.monotonic() < deadline and not future.done():
            time.sleep(0.01)
        if not future.done() or future.result() is None or not future.result().success:
            return None
        return future.result().state.pose

    def _gazebo_base_pose(self):
        pose = self._gazebo_entity_pose(BASE_FOOTPRINT_ENTITY)
        if pose is None:
            return None
        return {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "yaw": _yaw_from_quat(pose.orientation),
        }

    def _base_pose_in_map(self):
        gazebo_pose = self._gazebo_base_pose()
        if gazebo_pose is not None:
            return gazebo_pose
        try:
            tf = self._tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            return {"x": float(t.x), "y": float(t.y), "yaw": _yaw_from_quat(q)}
        except TransformException:
            return dict(YELLOW_CUBE_SLOT_VALIDATION_BASE_POSE)

    def _world_point_to_base(self, point, base_pose=None) -> list[float]:
        return world_to_base(point, self._base_pose_in_map() if base_pose is None else base_pose)

    def _apply_world_box(self, object_id: str, center, size) -> bool:
        if self.scene_client is None:
            return True
        scene = make_world_box_scene(
            object_id,
            {"center": list(center), "size": list(size)},
            "base_link",
        )
        return self._apply_scene_diff(scene, f"{object_id} add")

    def _remove_world_box(self, object_id: str) -> bool:
        if self.scene_client is None:
            return True
        return self._apply_scene_diff(
            make_remove_world_box_scene(object_id),
            f"{object_id} remove",
        )

    def _apply_scene_for_base(self, include_cube: bool) -> bool:
        base_pose = self._base_pose_in_map()
        boxes = [
            (
                "yellow_slot_station_a_table",
                STATION_A_TABLE_WORLD.center,
                STATION_A_TABLE_WORLD.size,
            ),
            (
                "yellow_slot_aging_zone_table",
                AGING_ZONE_TABLE_WORLD.center,
                AGING_ZONE_TABLE_WORLD.size,
            ),
        ]
        for name, box in AGING_RACK_COLLISIONS.items():
            local = box.center
            world = (
                AGING_RACK_WORLD_POSE.x + local[0],
                AGING_RACK_WORLD_POSE.y + local[1],
                AGING_RACK_WORLD_POSE.z + local[2],
            )
            boxes.append((f"yellow_slot_rack_{name}", world, box.size))
        if include_cube:
            boxes.append((
                TARGET_OBJECT,
                cube_center_world(),
                MATERIAL_CUBE_YELLOW_COLLISION_SIZE,
            ))
        ok = True
        for object_id, world_center, size in boxes:
            ok = self._apply_world_box(
                object_id,
                self._world_point_to_base(world_center, base_pose),
                size,
            ) and ok
        return ok

    def _attach_yellow_cube_scene(self) -> None:
        self._remove_world_box(TARGET_OBJECT)
        if self.scene_client is not None:
            self._apply_scene_diff(make_attach_scene(CARRIED_SAMPLE_BOX_ID), "yellow cube attach")

    def _detach_yellow_cube_scene(self) -> None:
        if self.scene_client is not None:
            self._apply_scene_diff(make_detach_scene(CARRIED_SAMPLE_BOX_ID), "yellow cube detach")
        self._apply_world_box(
            TARGET_OBJECT,
            self._world_point_to_base(middle_slot_center_world()),
            MATERIAL_CUBE_YELLOW_COLLISION_SIZE,
        )

    def _move_ompl(self, label: str, target: list[float], quat=DOWN_QUAT_XYZW) -> bool:
        self._publish_status(label)
        return self._move(
            target,
            quat=quat,
            target_link=GRIPPER_TCP_LINK,
            tolerance_position=DEFAULT_APPROACH_TOLERANCE_POSITION,
            tolerance_orientation=DEFAULT_APPROACH_TOLERANCE_ORIENTATION,
            timeout_sec=DEFAULT_MOVE_TIMEOUT_SEC,
            cartesian=False,
            stabilize_wrist=True,
            local_speed=False,
            fallback_to_ompl=True,
        )

    def _move_cartesian(
        self,
        label: str,
        target: list[float],
        quat=DOWN_QUAT_XYZW,
        eef_step: float = CARTESIAN_EEF_STEP,
    ) -> bool:
        self._publish_status(label)
        ok = self._move(
            target,
            quat=quat,
            target_link=GRIPPER_TCP_LINK,
            tolerance_position=DEFAULT_GRASP_TOLERANCE_POSITION,
            tolerance_orientation=DEFAULT_GRASP_TOLERANCE_ORIENTATION,
            timeout_sec=DEFAULT_MOVE_TIMEOUT_SEC,
            cartesian=True,
            stabilize_wrist=False,
            local_speed=True,
            fallback_to_ompl=False,
        )
        fraction = 1.0 if ok else 0.0
        self.get_logger().info(
            "%s Cartesian: step=%.3f fraction=%.3f target=(%.4f, %.4f, %.4f)"
            % (label, eef_step, fraction, target[0], target[1], target[2])
        )
        return fraction >= CARTESIAN_FRACTION_MIN

    def _wait_for_nav_active(self) -> bool:
        deadline = time.monotonic() + YELLOW_NAV_ACTIVE_TIMEOUT_SEC
        while time.monotonic() < deadline:
            if self._bt_state_client.wait_for_service(timeout_sec=0.2):
                future = self._bt_state_client.call_async(GetState.Request())
                end = time.monotonic() + 0.5
                while time.monotonic() < end and not future.done():
                    time.sleep(0.02)
                if future.done() and future.result() is not None:
                    if future.result().current_state.label == "active":
                        return True
            time.sleep(NAV_ACTIVE_POLL_SEC)
        return False

    def _wait_for_navigation_tf(self) -> bool:
        deadline = time.monotonic() + YELLOW_NAV_TF_READY_TIMEOUT_SEC
        while time.monotonic() < deadline:
            try:
                self._tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
                return True
            except TransformException:
                time.sleep(NAV_TF_READY_POLL_SEC)
        return False

    @staticmethod
    def _station_a_table_south_edge_y() -> float:
        return STATION_A_TABLE_WORLD.center[1] - STATION_A_TABLE_WORLD.size[1] / 2.0

    def _station_a_table_edge_clearance(self, base_pose) -> float:
        return self._station_a_table_south_edge_y() - float(base_pose["y"])

    def _aging_zone_first_goal_pose(self) -> tuple[str, PoseStamped]:
        goal_name, waypoint, _handoff = navigation_goals_for_station("aging_zone")[0]
        return goal_name, _pose_stamped(waypoint["x"], waypoint["y"], waypoint["yaw"])

    def _station_a_retreat_plan_check(self) -> bool:
        goal_name, goal = self._aging_zone_first_goal_pose()
        if not self._nav.compute_path_to_pose_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(
                "YELLOW_STATION_A_RETREAT_PLAN_CHECK goal=%s success=false "
                "path_pose_count=0 error=planner_server_unavailable"
                % goal_name
            )
            return False
        start = PoseStamped()
        start.header.frame_id = "map"
        start.header.stamp = self._nav.get_clock().now().to_msg()
        path = self._nav.getPath(start, goal, use_start=False)
        pose_count = 0 if path is None else len(path.poses)
        success = path is not None and pose_count > 0
        self.get_logger().info(
            "YELLOW_STATION_A_RETREAT_PLAN_CHECK goal=%s success=%s "
            "path_pose_count=%d"
            % (goal_name, "true" if success else "false", pose_count)
        )
        return success

    def _transport_safe(self) -> bool:
        self._publish_status("ARM_TRANSPORT_SAFE")
        ok = bool(self.go_home())
        if not ok:
            return False
        if not self._holding_is_healthy():
            self._handle_hold_lost()
            return False
        return True

    def _retreat_from_station_a_table(self) -> bool:
        self._publish_status("STATION_A_RETREAT_START")
        start_pose = self._base_pose_in_map()
        if start_pose is None:
            self._cmd_pub.publish(Twist())
            self._publish_status("STATION_A_RETREAT_TF_FAILED")
            return False
        cube_before = self._gazebo_cube_pose()
        if cube_before is not None:
            self.get_logger().info(
                "CUBE_WORLD_BEFORE_RETREAT x=%.4f y=%.4f z=%.4f"
                % (
                    cube_before.position.x,
                    cube_before.position.y,
                    cube_before.position.z,
                )
            )
        self.get_logger().info(
            "BASE_BEFORE_RETREAT x=%.4f y=%.4f yaw=%.4f"
            % (start_pose["x"], start_pose["y"], start_pose["yaw"])
        )

        initial_distance = self._station_a_table_edge_clearance(start_pose)
        best_distance = initial_distance
        last_progress = time.monotonic()
        start_wall = time.monotonic()

        while time.monotonic() - start_wall <= STATION_A_RETREAT_TIMEOUT_SEC:
            if not self._holding_is_healthy():
                self._cmd_pub.publish(Twist())
                self._handle_hold_lost()
                self._publish_status("FAILED_HOLDING")
                return False
            pose = self._base_pose_in_map()
            if pose is None:
                self._cmd_pub.publish(Twist())
                self._publish_status("STATION_A_RETREAT_TF_FAILED")
                return False
            distance = self._station_a_table_edge_clearance(pose)
            if distance >= STATION_A_RETREAT_TARGET_CLEARANCE:
                self._cmd_pub.publish(Twist())
                time.sleep(YELLOW_STATION_DOCK_SETTLE_SEC)
                after = self._base_pose_in_map()
                cube_after = self._gazebo_cube_pose()
                if after is not None:
                    self.get_logger().info(
                        "BASE_AFTER_RETREAT x=%.4f y=%.4f yaw=%.4f"
                        % (after["x"], after["y"], after["yaw"])
                    )
                if cube_after is not None:
                    self.get_logger().info(
                        "CUBE_WORLD_AFTER_RETREAT x=%.4f y=%.4f z=%.4f"
                        % (
                            cube_after.position.x,
                            cube_after.position.y,
                            cube_after.position.z,
                        )
                    )
                if not self._station_a_retreat_plan_check():
                    self._cmd_pub.publish(Twist())
                    self._publish_status("FAILED_STATION_A_RETREAT_NAV_VALIDATION")
                    return False
                self.get_logger().info("POST_RETREAT_START_CELL_STATE=PLANNABLE")
                self.get_logger().info("POST_RETREAT_GLOBAL_PLAN_AVAILABLE=true")
                self._publish_status("STATION_A_RETREAT_SAFE")
                return True

            if distance + STATION_A_RETREAT_PROGRESS_EPS > best_distance:
                best_distance = distance
                last_progress = time.monotonic()
            elif time.monotonic() - last_progress > STATION_A_RETREAT_STALLED_SEC:
                self._cmd_pub.publish(Twist())
                self.get_logger().warn(
                    "YELLOW_STATION_A_RETREAT_STALLED distance=%.4f "
                    "best_distance=%.4f target_clearance=%.4f"
                    % (distance, best_distance, STATION_A_RETREAT_TARGET_CLEARANCE)
                )
                self._publish_status("STATION_A_RETREAT_STALLED")
                return False

            if distance + STATION_A_RETREAT_WRONG_DIRECTION_EPS < best_distance:
                self._cmd_pub.publish(Twist())
                self.get_logger().warn(
                    "YELLOW_STATION_A_RETREAT_WRONG_DIRECTION distance=%.4f "
                    "best_distance=%.4f initial_distance=%.4f"
                    % (distance, best_distance, initial_distance)
                )
                self._publish_status("STATION_A_RETREAT_WRONG_DIRECTION")
                return False

            remaining = STATION_A_RETREAT_TARGET_CLEARANCE - distance
            speed = self._clamp(
                STATION_A_RETREAT_LINEAR_GAIN * remaining,
                STATION_A_RETREAT_MAX_LINEAR_SPEED,
            )
            if abs(speed) < STATION_A_RETREAT_MIN_LINEAR_SPEED:
                speed = STATION_A_RETREAT_MIN_LINEAR_SPEED
            yaw = float(pose["yaw"])
            world_vx = 0.0
            world_vy = -abs(speed)
            cmd = Twist()
            cmd.linear.x = math.cos(yaw) * world_vx + math.sin(yaw) * world_vy
            cmd.linear.y = -math.sin(yaw) * world_vx + math.cos(yaw) * world_vy
            cmd.angular.z = 0.0
            self.get_logger().info(
                "YELLOW_STATION_A_RETREAT base_x=%.4f base_y=%.4f base_yaw=%.4f "
                "table_edge_distance=%.4f target_clearance=%.3f "
                "cmd_x=%.4f cmd_y=%.4f cmd_yaw=%.4f"
                % (
                    pose["x"],
                    pose["y"],
                    pose["yaw"],
                    distance,
                    STATION_A_RETREAT_TARGET_CLEARANCE,
                    cmd.linear.x,
                    cmd.linear.y,
                    cmd.angular.z,
                )
            )
            self._cmd_pub.publish(cmd)
            time.sleep(0.05)

        self._cmd_pub.publish(Twist())
        self._publish_status("STATION_A_RETREAT_TIMEOUT")
        return False

    def _navigate_aging_zone(self) -> bool:
        self._stop_after_nav_leg1_start_reached = False
        self._publish_status("NAV_AGING_ZONE")
        base = self._gazebo_base_pose()
        if base is not None:
            self.get_logger().info(
                "BASE_BEFORE_NAV x=%.4f y=%.4f yaw=%.4f"
                % (base["x"], base["y"], base["yaw"])
            )
        goals = navigation_goals_for_station("aging_zone")
        self.get_logger().info(
            "YELLOW_AGING_NAV_MODE=PUBLIC_AGING_NAV_LEGS NAV_GOAL_COUNT=%d"
            % len(goals)
        )

        for goal_index, (goal_name, waypoint, _handoff_station) in enumerate(goals, start=1):
            self.get_logger().info(
                "NAV_LEG_%d_START goal=%s x=%.3f y=%.3f yaw=%.3f"
                % (
                    goal_index,
                    goal_name,
                    waypoint["x"],
                    waypoint["y"],
                    waypoint["yaw"],
                )
            )
            if not self._nav.nav_to_pose_client.wait_for_server(
                timeout_sec=NAV_SERVER_WAIT_SEC
            ):
                return False
            if not self._wait_for_nav_active():
                return False
            if not self._wait_for_navigation_tf():
                return False
            goal = _pose_stamped(waypoint["x"], waypoint["y"], waypoint["yaw"])
            self._nav.goToPose(goal)
            if goal_index == 1 and bool(
                self.get_parameter("stop_after_nav_leg1_start").value
            ):
                self._publish_status("YELLOW_NAV_LEG_1_STARTED")
                time.sleep(1.0)
                self._nav.cancelTask()
                self._cmd_pub.publish(Twist())
                self._stop_after_nav_leg1_start_reached = True
                return True
            if not self._wait_for_nav_leg_result(goal_index, goal_name):
                return False
            if NAV_HANDOFF_STOP_SEC > 0.0:
                stop = Twist()
                self._cmd_pub.publish(stop)
                time.sleep(NAV_HANDOFF_STOP_SEC)
            self.get_logger().info(
                "NAV_LEG_%d_DONE goal=%s" % (goal_index, goal_name)
            )

        base = self._gazebo_base_pose()
        if base is not None:
            self.get_logger().info(
                "NAV_END_BASE_POSE x=%.4f y=%.4f yaw=%.4f"
                % (base["x"], base["y"], base["yaw"])
            )
        self._publish_status("NAV_AGING_ZONE_SUCCESS")
        return True

    def _wait_for_nav_leg_result(self, goal_index: int, goal_name: str) -> bool:
        start_wall = time.monotonic()
        next_log_wall = start_wall + NAV_RUNNING_LOG_INTERVAL_SEC
        while rclpy.ok():
            if self._nav.isTaskComplete():
                result = self._nav.getResult()
                if result == TaskResult.SUCCEEDED:
                    return True
                self.get_logger().warn(
                    "NAV_LEG_%d_RESULT goal=%s result=%s"
                    % (goal_index, goal_name, result)
                )
                return False
            if not self._holding_is_healthy():
                self._handle_hold_lost()
                self._nav.cancelTask()
                return False
            now = time.monotonic()
            if now >= next_log_wall:
                self.get_logger().info(
                    "NAV_LEG_%d_RUNNING goal=%s elapsed_wall=%.1f"
                    % (goal_index, goal_name, now - start_wall)
                )
                next_log_wall = now + NAV_RUNNING_LOG_INTERVAL_SEC
            time.sleep(0.1)
        self.get_logger().warn(
            "NAV_LEG_%d_WAIT_STOPPED goal=%s reason=ROS_SHUTDOWN"
            % (goal_index, goal_name)
        )
        return False

    def _fine_dock(self, station: str) -> bool:
        deadline = time.monotonic() + YELLOW_STATION_DOCK_MAX_SEC
        while time.monotonic() < deadline:
            pose = self._base_pose_in_map()
            if pose is None:
                time.sleep(0.05)
                continue
            if not self._holding_is_healthy():
                self._handle_hold_lost()
                self._cmd_pub.publish(Twist())
                return False
            done, twist = station_dock_velocity_for_base(
                [pose["x"], pose["y"], pose["yaw"]],
                station,
            )
            if done:
                stop = Twist()
                self._cmd_pub.publish(stop)
                time.sleep(YELLOW_STATION_DOCK_SETTLE_SEC)
                return True
            self._cmd_pub.publish(twist)
            time.sleep(0.05)
        self._cmd_pub.publish(Twist())
        return False

    def _aging_station_dock(self) -> bool:
        self._publish_status("YELLOW_AGING_STATION_DOCK_START")
        before = self._gazebo_base_pose()
        if before is not None:
            self.get_logger().info(
                "BASE_BEFORE_AGING_STATION_DOCK x=%.4f y=%.4f yaw=%.4f"
                % (before["x"], before["y"], before["yaw"])
            )
        if not self._holding_is_healthy():
            self._handle_hold_lost()
            self._publish_status("FAILED_HOLDING")
            return False
        if not self._fine_dock("aging_zone"):
            self._publish_status("FAILED_AGING_STATION_DOCK")
            return False
        after = self._gazebo_base_pose()
        if after is not None:
            self.get_logger().info(
                "BASE_AFTER_AGING_STATION_DOCK x=%.4f y=%.4f yaw=%.4f"
                % (after["x"], after["y"], after["yaw"])
            )
        if not self._holding_is_healthy():
            self._handle_hold_lost()
            self._publish_status("FAILED_HOLDING")
            return False
        self._publish_status("YELLOW_AGING_STATION_DOCK_DONE")
        return True

    def _get_actual_base_pose_for_aging_place(self):
        pose = self._gazebo_base_pose()
        if pose is None:
            self.get_logger().error(
                "YELLOW_AGING_PLACE_ACTUAL_BASE_POSE_UNAVAILABLE "
                "reason=base_footprint_entity_unavailable entity=%s"
                % BASE_FOOTPRINT_ENTITY
            )
            return None
        return {
            "x": pose["x"],
            "y": pose["y"],
            "z": BASE_LINK_WORLD_Z,
            "yaw": pose["yaw"],
            "source": BASE_FOOTPRINT_ENTITY,
        }

    def _middle_slot_base_center(self, base_pose: dict) -> dict:
        slot = world_to_base(middle_slot_center_world(), base_pose)
        return {"x": slot[0], "y": slot[1], "z": slot[2]}

    @staticmethod
    def _aging_place_transform_footprint(base_pose: dict) -> list[tuple[float, float]]:
        yaw = float(base_pose["yaw"])
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        points = []
        for local_x, local_y in AGING_PLACE_FINE_DOCK_FOOTPRINT:
            points.append((
                float(base_pose["x"]) + cos_yaw * local_x - sin_yaw * local_y,
                float(base_pose["y"]) + sin_yaw * local_x + cos_yaw * local_y,
            ))
        return points

    @staticmethod
    def _aging_place_point_rect_distance(
        x: float,
        y: float,
        min_x: float,
        max_x: float,
        min_y: float,
        max_y: float,
    ) -> float:
        dx = max(min_x - x, 0.0, x - max_x)
        dy = max(min_y - y, 0.0, y - max_y)
        return math.hypot(dx, dy)

    @staticmethod
    def _aging_place_segment_intersects(
        a: tuple[float, float],
        b: tuple[float, float],
        c: tuple[float, float],
        d: tuple[float, float],
    ) -> bool:
        def orient(p, q, r):
            return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

        return orient(a, b, c) * orient(a, b, d) < 0.0 and orient(c, d, a) * orient(c, d, b) < 0.0

    def _aging_place_table_clearance(self, base_pose: dict) -> tuple[bool, float]:
        table = AGING_ZONE_TABLE_WORLD
        half_x = table.size[0] / 2.0
        half_y = table.size[1] / 2.0
        min_x = table.center[0] - half_x
        max_x = table.center[0] + half_x
        min_y = table.center[1] - half_y
        max_y = table.center[1] + half_y
        footprint = self._aging_place_transform_footprint(base_pose)
        rect = [(min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y)]

        for x, y in footprint:
            if min_x <= x <= max_x and min_y <= y <= max_y:
                return True, 0.0
        for i in range(len(footprint)):
            for j in range(len(rect)):
                if self._aging_place_segment_intersects(
                    footprint[i],
                    footprint[(i + 1) % len(footprint)],
                    rect[j],
                    rect[(j + 1) % len(rect)],
                ):
                    return True, 0.0
        clearance = min(
            self._aging_place_point_rect_distance(x, y, min_x, max_x, min_y, max_y)
            for x, y in footprint
        )
        return False, clearance

    def _aging_fine_dock_command(self, slot_base: dict) -> tuple[bool, Twist, float, float]:
        slot_x = float(slot_base["x"])
        slot_y = float(slot_base["y"])
        error_x = slot_x - AGING_PLACE_TARGET_SLOT_BASE_X
        error_y = slot_y
        done = (
            AGING_PLACE_SLOT_BASE_X_MIN <= slot_x <= AGING_PLACE_SLOT_BASE_X_MAX
            and abs(slot_y) <= AGING_PLACE_SLOT_BASE_Y_TOLERANCE
        )
        cmd = Twist()
        if done:
            return True, cmd, error_x, error_y
        cmd.linear.x = self._clamp(
            AGING_PLACE_FINE_DOCK_LINEAR_GAIN * error_x,
            AGING_PLACE_FINE_DOCK_MAX_SPEED,
        )
        cmd.linear.y = self._clamp(
            AGING_PLACE_FINE_DOCK_LINEAR_GAIN * error_y,
            AGING_PLACE_FINE_DOCK_MAX_SPEED,
        )
        if abs(cmd.linear.x) < AGING_PLACE_FINE_DOCK_MIN_SPEED and abs(error_x) > 0.005:
            cmd.linear.x = math.copysign(AGING_PLACE_FINE_DOCK_MIN_SPEED, error_x)
        if abs(cmd.linear.y) < AGING_PLACE_FINE_DOCK_MIN_SPEED and abs(error_y) > 0.005:
            cmd.linear.y = math.copysign(AGING_PLACE_FINE_DOCK_MIN_SPEED, error_y)
        cmd.angular.z = 0.0
        return False, cmd, error_x, error_y

    def _aging_place_fine_dock(self) -> bool:
        self._publish_status("YELLOW_AGING_FINE_DOCK_START")
        if not self._holding_is_healthy():
            self._handle_hold_lost()
            self._publish_status("FAILED_HOLDING")
            return False
        start_pose = self._get_actual_base_pose_for_aging_place()
        if start_pose is None:
            self._cmd_pub.publish(Twist())
            self._publish_status("FAILED_YELLOW_AGING_FINE_DOCK_BASE_POSE")
            return False

        start_x = float(start_pose["x"])
        start_y = float(start_pose["y"])
        best_slot_x = math.inf
        last_progress = time.monotonic()
        start_wall = time.monotonic()

        while rclpy.ok() and time.monotonic() - start_wall <= AGING_PLACE_FINE_DOCK_TIMEOUT_SEC:
            if not self._holding_is_healthy():
                self._cmd_pub.publish(Twist())
                self._handle_hold_lost()
                self._publish_status("FAILED_HOLDING")
                return False
            base_pose = self._get_actual_base_pose_for_aging_place()
            if base_pose is None:
                self._cmd_pub.publish(Twist())
                self._publish_status("FAILED_YELLOW_AGING_FINE_DOCK_BASE_POSE")
                return False
            slot_base = self._middle_slot_base_center(base_pose)
            collision, table_clearance = self._aging_place_table_clearance(base_pose)
            if collision or table_clearance < AGING_PLACE_FINE_DOCK_MIN_TABLE_CLEARANCE:
                self._cmd_pub.publish(Twist())
                self._publish_status("YELLOW_AGING_FINE_DOCK_TABLE_CLEARANCE_REJECT")
                return False

            moved = math.hypot(
                float(base_pose["x"]) - start_x,
                float(base_pose["y"]) - start_y,
            )
            if moved > AGING_PLACE_FINE_DOCK_MAX_APPROACH_DISTANCE:
                self._cmd_pub.publish(Twist())
                self._publish_status("FAILED_YELLOW_AGING_FINE_DOCK_DISTANCE_LIMIT")
                return False

            done, cmd, error_x, error_y = self._aging_fine_dock_command(slot_base)
            slot_x = float(slot_base["x"])
            slot_y = float(slot_base["y"])
            if done:
                self._cmd_pub.publish(Twist())
                time.sleep(YELLOW_STATION_DOCK_SETTLE_SEC)
                self.get_logger().info(
                    "YELLOW_AGING_FINE_DOCK_FINAL_SLOT_BASE x=%.4f y=%.4f z=%.4f"
                    % (slot_x, slot_y, slot_base["z"])
                )
                self._publish_status("YELLOW_AGING_FINE_DOCK_SUCCESS")
                return True

            if slot_x + AGING_PLACE_FINE_DOCK_PROGRESS_EPS < best_slot_x:
                best_slot_x = slot_x
                last_progress = time.monotonic()
            elif time.monotonic() - last_progress > AGING_PLACE_FINE_DOCK_STALLED_SEC:
                self._cmd_pub.publish(Twist())
                self._publish_status("YELLOW_AGING_FINE_DOCK_STALLED")
                return False
            if slot_x > best_slot_x + AGING_PLACE_FINE_DOCK_WRONG_DIRECTION_EPS:
                self._cmd_pub.publish(Twist())
                self._publish_status("FAILED_YELLOW_AGING_FINE_DOCK_WRONG_DIRECTION")
                return False

            self.get_logger().info(
                "YELLOW_AGING_FINE_DOCK base_x=%.4f base_y=%.4f base_yaw=%.4f "
                "slot_x=%.4f slot_y=%.4f error_x=%.4f error_y=%.4f "
                "table_clearance=%.4f cmd_x=%.4f cmd_y=%.4f cmd_yaw=%.4f"
                % (
                    base_pose["x"],
                    base_pose["y"],
                    base_pose["yaw"],
                    slot_x,
                    slot_y,
                    error_x,
                    error_y,
                    table_clearance,
                    cmd.linear.x,
                    cmd.linear.y,
                    cmd.angular.z,
                )
            )
            self._cmd_pub.publish(cmd)
            time.sleep(0.05)

        self._cmd_pub.publish(Twist())
        self._publish_status("YELLOW_AGING_FINE_DOCK_TIMEOUT")
        return False

    def _placement_targets_after_aging_fine_dock(self):
        base_pose = self._get_actual_base_pose_for_aging_place()
        if base_pose is None:
            self._publish_status("FAILED_YELLOW_AGING_FINE_DOCK_BASE_POSE")
            return None
        slot_base = self._middle_slot_base_center(base_pose)
        self.get_logger().info(
            "BASE_AFTER_AGING_FINE_DOCK x=%.4f y=%.4f yaw=%.4f source=%s"
            % (
                base_pose["x"],
                base_pose["y"],
                base_pose["yaw"],
                base_pose["source"],
            )
        )
        self.get_logger().info(
            "YELLOW_MIDDLE_SLOT_BASE_AFTER_FINE_DOCK x=%.4f y=%.4f z=%.4f"
            % (slot_base["x"], slot_base["y"], slot_base["z"])
        )
        targets = {
            "pre_slot_high": self._world_point_to_base(pre_slot_high_tcp_world(), base_pose),
            "pre_slot": self._world_point_to_base(pre_slot_tcp_world(), base_pose),
            "stage1": self._world_point_to_base(insert_stage1_tcp_world(), base_pose),
            "stage2": self._world_point_to_base(insert_stage2_tcp_world(), base_pose),
            "final": self._world_point_to_base(insert_final_tcp_world(), base_pose),
            "retreat": self._world_point_to_base(vertical_retreat_tcp_world(), base_pose),
        }
        pre_slot_high = targets["pre_slot_high"]
        self.get_logger().info(
            "PRE_SLOT_HIGH_TARGET_AFTER_FINE_DOCK x=%.4f y=%.4f z=%.4f"
            % (pre_slot_high[0], pre_slot_high[1], pre_slot_high[2])
        )
        return targets

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        limit = abs(float(limit))
        return max(-limit, min(limit, float(value)))

    def _cube_base_with_actual_base_for_fine_dock(self):
        base_pose = self._base_pose_in_map()
        cube_base = world_to_base(cube_center_world(), base_pose)
        return (
            {"x": cube_base[0], "y": cube_base[1], "z": cube_base[2]},
            dict(base_pose),
        )

    def _fine_dock_station_a_for_cube(self) -> bool:
        before = self._cube_base_with_actual_base_for_fine_dock()
        cube_before, base_before = before
        self.get_logger().info(
            "YELLOW_STATION_A_FINE_DOCK_BASE_BEFORE x=%.4f y=%.4f yaw=%.4f"
            % (base_before["x"], base_before["y"], base_before["yaw"])
        )
        self.get_logger().info(
            "YELLOW_STATION_A_FINE_DOCK_CUBE_BEFORE x=%.4f y=%.4f z=%.4f"
            % (cube_before["x"], cube_before["y"], cube_before["z"])
        )
        self.get_logger().info(
            "YELLOW_STATION_A_FINE_DOCK_TARGET_CUBE_BASE target_x=%.3f target_y=%.3f"
            % (
                YELLOW_STATION_A_FINE_DOCK_TARGET_CUBE_BASE_X,
                YELLOW_STATION_A_FINE_DOCK_TARGET_CUBE_BASE_Y,
            )
        )

        start = time.monotonic()
        last_progress = start
        best_error = math.inf
        while time.monotonic() - start <= YELLOW_STATION_A_FINE_DOCK_TIMEOUT_SEC:
            cube_base, actual_base = self._cube_base_with_actual_base_for_fine_dock()
            cube_x = float(cube_base["x"])
            cube_y = float(cube_base["y"])
            if cube_x < YELLOW_STATION_A_FINE_DOCK_MIN_SAFE_X:
                self._cmd_pub.publish(Twist())
                self.get_logger().warn(
                    "YELLOW_STATION_A_FINE_DOCK_TOO_CLOSE cube_x=%.4f min_safe_x=%.4f"
                    % (cube_x, YELLOW_STATION_A_FINE_DOCK_MIN_SAFE_X)
                )
                return False

            error_x = cube_x - YELLOW_STATION_A_FINE_DOCK_TARGET_CUBE_BASE_X
            error_y = cube_y - YELLOW_STATION_A_FINE_DOCK_TARGET_CUBE_BASE_Y
            error_norm = math.hypot(error_x, error_y)
            if error_norm + YELLOW_STATION_A_FINE_DOCK_PROGRESS_EPS < best_error:
                best_error = error_norm
                last_progress = time.monotonic()
            elif time.monotonic() - last_progress > YELLOW_STATION_A_FINE_DOCK_STALLED_SEC:
                self._cmd_pub.publish(Twist())
                self.get_logger().warn(
                    "YELLOW_STATION_A_FINE_DOCK_STALLED base_x=%.4f base_y=%.4f "
                    "base_yaw=%.4f cube_x=%.4f cube_y=%.4f error_x=%.4f "
                    "error_y=%.4f best_error=%.4f"
                    % (
                        actual_base["x"],
                        actual_base["y"],
                        actual_base["yaw"],
                        cube_x,
                        cube_y,
                        error_x,
                        error_y,
                        best_error,
                    )
                )
                return False

            if (
                abs(error_x) <= YELLOW_STATION_A_FINE_DOCK_X_TOLERANCE
                and abs(error_y) <= YELLOW_STATION_A_FINE_DOCK_Y_TOLERANCE
            ):
                self._cmd_pub.publish(Twist())
                time.sleep(YELLOW_STATION_DOCK_SETTLE_SEC)
                cube_after, base_after = self._cube_base_with_actual_base_for_fine_dock()
                self.get_logger().info(
                    "YELLOW_STATION_A_FINE_DOCK_BASE_AFTER x=%.4f y=%.4f yaw=%.4f"
                    % (base_after["x"], base_after["y"], base_after["yaw"])
                )
                self.get_logger().info(
                    "YELLOW_STATION_A_FINE_DOCK_CUBE_AFTER x=%.4f y=%.4f z=%.4f"
                    % (cube_after["x"], cube_after["y"], cube_after["z"])
                )
                return True

            cmd = Twist()
            cmd.linear.x = self._clamp(
                YELLOW_STATION_A_FINE_DOCK_LINEAR_GAIN * error_x,
                YELLOW_STATION_A_FINE_DOCK_MAX_LINEAR_SPEED,
            )
            cmd.linear.y = self._clamp(
                YELLOW_STATION_A_FINE_DOCK_LINEAR_GAIN * error_y,
                YELLOW_STATION_A_FINE_DOCK_MAX_LINEAR_SPEED,
            )
            self.get_logger().info(
                "YELLOW_STATION_A_FINE_DOCK base_x=%.4f base_y=%.4f base_yaw=%.4f "
                "cube_x=%.4f cube_y=%.4f error_x=%.4f error_y=%.4f "
                "cmd_x=%.4f cmd_y=%.4f cmd_yaw=%.4f"
                % (
                    actual_base["x"],
                    actual_base["y"],
                    actual_base["yaw"],
                    cube_x,
                    cube_y,
                    error_x,
                    error_y,
                    cmd.linear.x,
                    cmd.linear.y,
                    cmd.angular.z,
                )
            )
            self._cmd_pub.publish(cmd)
            time.sleep(0.05)

        self._cmd_pub.publish(Twist())
        self.get_logger().warn("YELLOW_STATION_A_FINE_DOCK_TIMEOUT")
        return False

    def _gazebo_cube_pose(self):
        return self._gazebo_entity_pose(TARGET_OBJECT)

    def _validate_place(self) -> bool:
        pose = self._gazebo_cube_pose()
        if pose is None:
            return False
        target = middle_slot_center_world()
        bounds = middle_slot_bounds_world()
        dx = pose.position.x - target[0]
        dy = pose.position.y - target[1]
        dz = pose.position.z - target[2]
        actual_yaw = _yaw_from_quat(pose.orientation)
        expected_yaw = SLOT_ALIGNMENT_YAW
        yaw_error = _symmetric_yaw_error(actual_yaw, expected_yaw)
        inside = (
            bounds["x"][0] <= pose.position.x <= bounds["x"][1]
            and bounds["y"][0] <= pose.position.y <= bounds["y"][1]
            and bounds["z"][0] <= pose.position.z <= bounds["z"][1]
            and abs(dx) <= PLACE_VALIDATION_XY_TOLERANCE
            and abs(dy) <= PLACE_VALIDATION_XY_TOLERANCE
            and abs(dz) <= PLACE_VALIDATION_Z_TOLERANCE
            and yaw_error <= PLACE_VALIDATION_YAW_TOLERANCE
        )
        self.get_logger().info(
            "YELLOW_CUBE_SLOT_PLACE_RESULT dx=%.4f dy=%.4f dz=%.4f "
            "actual_yaw=%.4f expected_yaw=%.4f yaw_error=%.4f "
            "yaw_tolerance=%.4f inside=%s"
            % (
                dx,
                dy,
                dz,
                actual_yaw,
                expected_yaw,
                yaw_error,
                PLACE_VALIDATION_YAW_TOLERANCE,
                inside,
            )
        )
        if inside:
            self._publish_status("YELLOW_CUBE_SLOT_PLACE_VALID")
        return inside

    def _run_sequence(self) -> None:
        self._publish_status("START")
        self._publish_status("SKIP_INITIAL_NAV_STATION_A")
        self._apply_scene_for_base(include_cube=True)

        self._publish_status("STATION_A_FINE_DOCK_START")
        if not self._fine_dock_station_a_for_cube():
            self._publish_status("FAILED_STATION_A_FINE_DOCK")
            return
        self._publish_status("STATION_A_FINE_DOCK_DONE")
        if bool(self.get_parameter("stop_after_station_a_fine_dock").value):
            return

        pre_high = self._world_point_to_base(pre_grasp_high_tcp_world())
        pre = self._world_point_to_base(pre_grasp_tcp_world())
        grasp = self._world_point_to_base(grasp_tcp_world())
        lift = self._world_point_to_base(lift_tcp_world())

        if not self._move_ompl("PRE_GRASP_HIGH", pre_high, DOWN_QUAT):
            self._publish_status("FAILED_PRE_GRASP")
            return
        if not self._move_ompl("PRE_GRASP", pre, DOWN_QUAT):
            self._publish_status("FAILED_PRE_GRASP")
            return
        if not self._move_cartesian("DESCEND_GRASP", grasp, DOWN_QUAT):
            self._publish_status("FAILED_DESCEND_GRASP")
            return

        self._publish_status("GRIPPER_CLOSE")
        if not self.gripper.acquire_object():
            self.gripper.open()
            self._publish_status("FAILED_GRASP")
            return
        self._publish_status("ATTACHED")
        self._attach_yellow_cube_scene()
        if not self.gripper.wait_until_holding(POST_ATTACH_HOLD_CONFIRM_TIMEOUT_SEC):
            self.gripper.release_object()
            self.gripper.open()
            self._publish_status("FAILED_HOLDING")
            return
        self._start_hold_monitor()
        self._publish_status("HOLDING")

        if not self._move_cartesian("LIFT", lift, DOWN_QUAT):
            self._publish_status("FAILED_LIFT")
            return
        if bool(self.get_parameter("stop_after_yellow_lift").value):
            self._publish_status("YELLOW_GRASP_ONLY_SUCCESS")
            return
        if not self._transport_safe():
            self._publish_status("FAILED_TRANSPORT_SAFE")
            return
        if not self._retreat_from_station_a_table():
            return
        if not self._navigate_aging_zone():
            self._publish_status("FAILED_NAV_AGING_ZONE")
            return
        if self._stop_after_nav_leg1_start_reached:
            return
        if bool(self.get_parameter("stop_after_nav_aging_zone").value):
            self._publish_status("YELLOW_NAV_AGING_ZONE_SUCCESS")
            return

        if not self._aging_station_dock():
            return
        if not self._aging_place_fine_dock():
            return
        if bool(self.get_parameter("stop_after_aging_fine_dock").value):
            self._publish_status("YELLOW_AGING_FINE_DOCK_ONLY_SUCCESS")
            return

        self._apply_scene_for_base(include_cube=False)
        targets = self._placement_targets_after_aging_fine_dock()
        if targets is None:
            return
        pre_slot_high = targets["pre_slot_high"]
        pre_slot = targets["pre_slot"]
        stage1 = targets["stage1"]
        stage2 = targets["stage2"]
        final = targets["final"]
        retreat = targets["retreat"]

        if not self._move_ompl("PRE_SLOT_HIGH", pre_slot_high, SLOT_QUAT_XYZW):
            self._publish_status("FAILED_PRE_SLOT")
            return
        if bool(self.get_parameter("stop_after_pre_slot_high").value):
            self._publish_status("YELLOW_PRE_SLOT_HIGH_SUCCESS")
            return
        self._publish_status("SLOT_ALIGNMENT_ROTATE_START")
        if not self._move_ompl(
            "SLOT_ALIGNMENT_ROTATE_DONE",
            pre_slot_high,
            SLOT_ALIGNMENT_QUAT_XYZW,
        ):
            self._publish_status("FAILED_SLOT_ALIGNMENT_ROTATE")
            return
        if not self._move_ompl("PRE_SLOT", pre_slot, SLOT_ALIGNMENT_QUAT_XYZW):
            self._publish_status("FAILED_PRE_SLOT")
            return
        if not self._move_cartesian("INSERT_STAGE1", stage1, SLOT_ALIGNMENT_QUAT_XYZW):
            self._publish_status("FAILED_INSERT_STAGE1")
            return
        if not self._move_cartesian("INSERT_STAGE2", stage2, SLOT_ALIGNMENT_QUAT_XYZW):
            self._publish_status("FAILED_INSERT_STAGE2")
            return
        if not self._move_cartesian("INSERT_FINAL", final, SLOT_ALIGNMENT_QUAT_XYZW):
            self._publish_status("FAILED_INSERT_FINAL")
            return

        self._publish_status("RELEASE")
        self._stop_hold_monitor()
        if not self.gripper.release_object():
            self._publish_status("FAILED_RELEASE")
            return
        self._detach_yellow_cube_scene()
        if not self.gripper.open():
            self._publish_status("FAILED_RELEASE")
            return
        self._publish_status("DETACHED")
        self._publish_status("SETTLE")
        time.sleep(PLACE_SETTLE_SEC)
        if not self._validate_place():
            self._publish_status("FAILED_PLACE_VALIDATION")
            return
        if not self._move_cartesian("VERTICAL_RETREAT", retreat, SLOT_ALIGNMENT_QUAT_XYZW):
            self._publish_status("FAILED_VERTICAL_RETREAT")
            return
        self._publish_status("YELLOW_CUBE_SLOT_SUCCESS")


def main(args=None):
    rclpy.init(args=args)
    node = YellowCubeSlotValidation()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
