"""G5 arm-level dynamic obstacle avoidance demo.

This standalone demo verifies the MoveIt half of arm obstacle avoidance:
it injects a box collision object into PlanningScene, then asks MoveIt to plan
or execute a UR5e TCP pose. It does not touch mission or pick/place code.
"""
import argparse
import csv
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from threading import Thread


THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

from g5_arm_dynamic_obstacle_probe import BoxSpec, make_box_scene, make_remove_scene


UR_JOINTS = [
    "ur_shoulder_pan_joint",
    "ur_shoulder_lift_joint",
    "ur_elbow_joint",
    "ur_wrist_1_joint",
    "ur_wrist_2_joint",
    "ur_wrist_3_joint",
]
DOWN_QUAT = [1.0, 0.0, 0.0, 0.0]
DEFAULT_TARGET = [0.45, 0.0, 0.62]
DEFAULT_BOX_CENTER = [0.42, 0.0, 0.52]
DEFAULT_BOX_SIZE = [0.20, 0.28, 0.28]
HOME_CONFIG = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
MOVEIT_SUCCESS = 1


@dataclass(frozen=True)
class PlanOutcome:
    ok: bool
    error_code: int
    trajectory_points: int
    planning_latency_s: float


@dataclass(frozen=True)
class ApplySceneOutcome:
    ok: bool
    service_wait_s: float
    call_latency_s: float

    @property
    def total_latency_s(self) -> float:
        return self.service_wait_s + self.call_latency_s


@dataclass(frozen=True)
class RunMeasurement:
    stamp: str
    case_label: str
    expected_verdict: str
    expected_met: bool
    verdict: str
    obstacle_id: str
    obstacle_frame: str
    obstacle_center: tuple[float, float, float]
    obstacle_size: tuple[float, float, float]
    target_frame: str
    target_link: str
    target: tuple[float, float, float]
    start_joints: tuple[float, ...]
    apply_scene_ok: bool
    apply_scene_latency_s: float
    apply_scene_service_wait_s: float
    apply_scene_call_latency_s: float
    planning_ok: bool
    moveit_error_code: int
    trajectory_points: int
    planning_latency_s: float
    total_latency_s: float


def _wait_for_moveit_result(moveit2, timeout_s: float) -> bool:
    deadline = time.monotonic() + max(float(timeout_s), 0.0)
    result_name = "_MoveIt2__get_result_future_move_action"
    state_name = "_MoveIt2__last_execution_succeeded"
    while time.monotonic() < deadline:
        result_future = getattr(moveit2, result_name, None)
        if result_future is not None and result_future.done():
            try:
                return bool(getattr(moveit2, state_name, False))
            except Exception:  # noqa: BLE001
                return False
        if (
            not getattr(moveit2, "_MoveIt2__is_motion_requested", False)
            and not getattr(moveit2, "_MoveIt2__is_executing", False)
            and result_future is not None
        ):
            return bool(getattr(moveit2, state_name, False))
        time.sleep(0.02)
    return False


def _apply_scene_with_node(node, scene, timeout_s: float) -> bool:
    from moveit_msgs.srv import ApplyPlanningScene

    client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not client.wait_for_service(timeout_sec=timeout_s):
        node.get_logger().error("/apply_planning_scene service unavailable")
        return False
    request = ApplyPlanningScene.Request()
    request.scene = scene
    future = client.call_async(request)
    deadline = time.monotonic() + timeout_s
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.02)
    if not future.done():
        future.cancel()
        return False
    response = future.result()
    return response is not None and bool(response.success)


def _timed_apply_scene_with_node(node, scene, timeout_s: float) -> ApplySceneOutcome:
    from moveit_msgs.srv import ApplyPlanningScene

    client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    wait_start = time.monotonic()
    if not client.wait_for_service(timeout_sec=timeout_s):
        node.get_logger().error("/apply_planning_scene service unavailable")
        return ApplySceneOutcome(False, time.monotonic() - wait_start, 0.0)
    service_wait_s = time.monotonic() - wait_start

    request = ApplyPlanningScene.Request()
    request.scene = scene
    call_start = time.monotonic()
    future = client.call_async(request)
    deadline = time.monotonic() + timeout_s
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.02)
    call_latency_s = time.monotonic() - call_start
    if not future.done():
        future.cancel()
        return ApplySceneOutcome(False, service_wait_s, call_latency_s)
    response = future.result()
    return ApplySceneOutcome(
        response is not None and bool(response.success),
        service_wait_s,
        call_latency_s,
    )


def _move_to_pose_async(
    moveit2,
    *,
    position,
    quat_xyzw,
    frame_id,
    target_link,
    tolerance_position,
    tolerance_orientation,
    plan_only: bool,
) -> bool:
    goal_name = "_MoveIt2__move_action_goal"
    goal = getattr(moveit2, goal_name, None)
    if goal is None:
        return False
    goal.planning_options.plan_only = bool(plan_only)
    moveit2.move_to_pose(
        position=list(position),
        quat_xyzw=list(quat_xyzw),
        frame_id=frame_id,
        target_link=target_link,
        tolerance_position=tolerance_position,
        tolerance_orientation=tolerance_orientation,
        cartesian=False,
    )
    return True


def _make_pose_constraints(args):
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
    from shape_msgs.msg import SolidPrimitive

    constraints = Constraints()

    position_constraint = PositionConstraint()
    position_constraint.header.frame_id = args.target_frame
    position_constraint.link_name = args.target_link
    position_constraint.constraint_region.primitives.append(SolidPrimitive())
    position_constraint.constraint_region.primitives[0].type = SolidPrimitive.SPHERE
    position_constraint.constraint_region.primitives[0].dimensions = [
        float(args.position_tolerance)
    ]
    pose = Pose()
    pose.position.x = float(args.target[0])
    pose.position.y = float(args.target[1])
    pose.position.z = float(args.target[2])
    pose.orientation.w = 1.0
    position_constraint.constraint_region.primitive_poses.append(pose)
    position_constraint.weight = 1.0

    orientation_constraint = OrientationConstraint()
    orientation_constraint.header.frame_id = args.target_frame
    orientation_constraint.link_name = args.target_link
    orientation_constraint.orientation.x = float(args.quat[0])
    orientation_constraint.orientation.y = float(args.quat[1])
    orientation_constraint.orientation.z = float(args.quat[2])
    orientation_constraint.orientation.w = float(args.quat[3])
    orientation_constraint.absolute_x_axis_tolerance = float(
        args.orientation_tolerance
    )
    orientation_constraint.absolute_y_axis_tolerance = float(
        args.orientation_tolerance
    )
    orientation_constraint.absolute_z_axis_tolerance = float(
        args.orientation_tolerance
    )
    orientation_constraint.weight = 1.0

    constraints.position_constraints.append(position_constraint)
    constraints.orientation_constraints.append(orientation_constraint)
    return constraints


def _plan_pose_with_service(node, args) -> PlanOutcome:
    from moveit_msgs.srv import GetMotionPlan
    from sensor_msgs.msg import JointState

    client = node.create_client(GetMotionPlan, "/plan_kinematic_path")
    if not client.wait_for_service(timeout_sec=args.scene_timeout):
        node.get_logger().error("/plan_kinematic_path service unavailable")
        return PlanOutcome(False, 0, 0, 0.0)

    stamp = node.get_clock().now().to_msg()
    request = GetMotionPlan.Request()
    mpr = request.motion_plan_request
    mpr.workspace_parameters.header.frame_id = "ur_base_link"
    mpr.workspace_parameters.header.stamp = stamp
    mpr.workspace_parameters.min_corner.x = -1.0
    mpr.workspace_parameters.min_corner.y = -1.0
    mpr.workspace_parameters.min_corner.z = -1.0
    mpr.workspace_parameters.max_corner.x = 1.0
    mpr.workspace_parameters.max_corner.y = 1.0
    mpr.workspace_parameters.max_corner.z = 1.0
    mpr.group_name = "ur_manipulator"
    mpr.num_planning_attempts = int(args.planning_attempts)
    mpr.allowed_planning_time = float(args.planning_time)
    mpr.max_velocity_scaling_factor = float(args.max_velocity)
    mpr.max_acceleration_scaling_factor = float(args.max_acceleration)

    start = JointState()
    start.header.stamp = stamp
    start.name = list(UR_JOINTS)
    start.position = [float(value) for value in args.start_joints]
    mpr.start_state.joint_state = start
    mpr.goal_constraints = [_make_pose_constraints(args)]

    plan_start = time.monotonic()
    future = client.call_async(request)
    deadline = time.monotonic() + float(args.timeout)
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.02)
    planning_latency_s = time.monotonic() - plan_start
    if not future.done():
        future.cancel()
        node.get_logger().error("planning service timed out")
        return PlanOutcome(False, 0, 0, planning_latency_s)
    response = future.result()
    if response is None:
        return PlanOutcome(False, 0, 0, planning_latency_s)
    result = response.motion_plan_response
    point_count = len(result.trajectory.joint_trajectory.points)
    if result.error_code.val != MOVEIT_SUCCESS:
        node.get_logger().warn(
            f"planning failed with MoveIt error code {result.error_code.val}"
        )
        return PlanOutcome(False, result.error_code.val, point_count, planning_latency_s)
    return PlanOutcome(
        point_count > 0,
        result.error_code.val,
        point_count,
        planning_latency_s,
    )


def _write_measurement(measurement: RunMeasurement, out_dir: Path) -> tuple[Path, Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / f"g5_arm_avoidance_{measurement.stamp}.csv"
    md_path = out_dir / f"g5_arm_avoidance_{measurement.stamp}.md"
    control_latency_s = max(
        0.0,
        measurement.total_latency_s - measurement.apply_scene_service_wait_s,
    )

    row = {
        "stamp": measurement.stamp,
        "case_label": measurement.case_label,
        "expected_verdict": measurement.expected_verdict,
        "expected_met": str(measurement.expected_met),
        "verdict": measurement.verdict,
        "obstacle_id": measurement.obstacle_id,
        "obstacle_frame": measurement.obstacle_frame,
        "obstacle_center": " ".join(f"{v:.6f}" for v in measurement.obstacle_center),
        "obstacle_size": " ".join(f"{v:.6f}" for v in measurement.obstacle_size),
        "target_frame": measurement.target_frame,
        "target_link": measurement.target_link,
        "target": " ".join(f"{v:.6f}" for v in measurement.target),
        "start_joints": " ".join(f"{v:.6f}" for v in measurement.start_joints),
        "apply_scene_ok": str(measurement.apply_scene_ok),
        "apply_scene_latency_ms": f"{measurement.apply_scene_latency_s * 1000.0:.3f}",
        "apply_scene_service_wait_ms": (
            f"{measurement.apply_scene_service_wait_s * 1000.0:.3f}"
        ),
        "apply_scene_call_latency_ms": (
            f"{measurement.apply_scene_call_latency_s * 1000.0:.3f}"
        ),
        "planning_ok": str(measurement.planning_ok),
        "moveit_error_code": str(measurement.moveit_error_code),
        "trajectory_points": str(measurement.trajectory_points),
        "planning_latency_ms": f"{measurement.planning_latency_s * 1000.0:.3f}",
        "total_latency_ms": f"{measurement.total_latency_s * 1000.0:.3f}",
        "control_latency_ms": f"{control_latency_s * 1000.0:.3f}",
    }
    with csv_path.open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=list(row))
        writer.writeheader()
        writer.writerow(row)

    lines = [
        f"# G5 机械臂动态避障测量报告 {measurement.stamp}",
        f"- 场景: {measurement.case_label}",
        f"- 预期结果: {measurement.expected_verdict}",
        f"- 预期满足: {measurement.expected_met}",
        f"- 结果: {measurement.verdict}",
        f"- 障碍盒: id={measurement.obstacle_id}, frame={measurement.obstacle_frame}, "
        f"center={list(measurement.obstacle_center)}, size={list(measurement.obstacle_size)}",
        f"- 目标 TCP: frame={measurement.target_frame}, link={measurement.target_link}, "
        f"target={list(measurement.target)}",
        f"- 起始关节: {list(measurement.start_joints)}",
        f"- PlanningScene 注入: ok={measurement.apply_scene_ok}, "
        f"latency={measurement.apply_scene_latency_s * 1000.0:.1f} ms",
        f"- PlanningScene 服务发现: "
        f"{measurement.apply_scene_service_wait_s * 1000.0:.1f} ms",
        f"- PlanningScene apply 调用: "
        f"{measurement.apply_scene_call_latency_s * 1000.0:.1f} ms",
        f"- MoveIt 规划: ok={measurement.planning_ok}, "
        f"error_code={measurement.moveit_error_code}, "
        f"trajectory_points={measurement.trajectory_points}, "
        f"latency={measurement.planning_latency_s * 1000.0:.1f} ms",
        f"- 总响应延时: {measurement.total_latency_s * 1000.0:.1f} ms",
        f"- 服务已连接后的控制响应延时: {control_latency_s * 1000.0:.1f} ms",
        "- 口径: 手动障碍触发 -> /apply_planning_scene 成功返回 -> "
        "/plan_kinematic_path 返回规划结果。",
        f"- CSV: {csv_path}",
    ]
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return csv_path, md_path


def run(args) -> int:
    try:
        import rclpy
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.parameter import Parameter
        from pymoveit2 import MoveIt2
    except ImportError as exc:
        raise RuntimeError(
            "Run after sourcing ROS 2, MoveIt, and this workspace install/setup.bash"
        ) from exc

    rclpy.init()
    node = rclpy.create_node("g5_arm_avoidance_demo")
    try:
        node.set_parameters([
            Parameter("use_sim_time", Parameter.Type.BOOL, bool(args.use_sim_time))
        ])
    except Exception:  # noqa: BLE001
        node.declare_parameter("use_sim_time", bool(args.use_sim_time))
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        run_start = time.monotonic()
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
        box = BoxSpec(
            object_id=args.obstacle_id,
            frame_id=args.obstacle_frame,
            center=tuple(args.obstacle_center),
            size=tuple(args.obstacle_size),
        )
        apply_outcome = _timed_apply_scene_with_node(
            node, make_box_scene(box), timeout_s=args.scene_timeout)
        if not apply_outcome.ok:
            node.get_logger().error("failed to add G5 obstacle box")
            return 1
        node.get_logger().info(
            "G5 obstacle added id=%s frame=%s center=%s size=%s"
            % (box.object_id, box.frame_id, list(box.center), list(box.size))
        )

        if args.execute:
            moveit2 = MoveIt2(
                node=node,
                joint_names=UR_JOINTS,
                base_link_name="ur_base_link",
                end_effector_name="ur_tool0",
                group_name="ur_manipulator",
                callback_group=ReentrantCallbackGroup(),
            )
            moveit2.max_velocity = args.max_velocity
            moveit2.max_acceleration = args.max_acceleration
            moveit2.allowed_planning_time = args.planning_time
            moveit2.num_planning_attempts = args.planning_attempts
            sent = _move_to_pose_async(
                moveit2,
                position=list(args.target),
                quat_xyzw=list(args.quat),
                frame_id=args.target_frame,
                target_link=args.target_link,
                tolerance_position=args.position_tolerance,
                tolerance_orientation=args.orientation_tolerance,
                plan_only=False,
            )
            if not sent:
                node.get_logger().error("failed to prepare MoveIt pose goal")
                return 1
            planning_start = time.monotonic()
            ok = _wait_for_moveit_result(moveit2, args.timeout)
            outcome = PlanOutcome(
                ok=ok,
                error_code=MOVEIT_SUCCESS if ok else 0,
                trajectory_points=0,
                planning_latency_s=time.monotonic() - planning_start,
            )
            verdict = "EXECUTE_OK" if ok else "EXECUTE_FAILED"
        else:
            outcome = _plan_pose_with_service(node, args)
            ok = outcome.ok
            verdict = "PLAN_OK" if ok else "PLAN_FAILED"

        print(f"G5_ARM_AVOIDANCE_{verdict}")
        expected_met = (
            args.expected_verdict == "ANY"
            or args.expected_verdict == verdict
        )
        measurement = RunMeasurement(
            stamp=stamp,
            case_label=args.case_label,
            expected_verdict=args.expected_verdict,
            expected_met=expected_met,
            verdict=verdict,
            obstacle_id=box.object_id,
            obstacle_frame=box.frame_id,
            obstacle_center=tuple(float(v) for v in box.center),
            obstacle_size=tuple(float(v) for v in box.size),
            target_frame=args.target_frame,
            target_link=args.target_link,
            target=tuple(float(v) for v in args.target),
            start_joints=tuple(float(v) for v in args.start_joints),
            apply_scene_ok=apply_outcome.ok,
            apply_scene_latency_s=apply_outcome.total_latency_s,
            apply_scene_service_wait_s=apply_outcome.service_wait_s,
            apply_scene_call_latency_s=apply_outcome.call_latency_s,
            planning_ok=outcome.ok,
            moveit_error_code=outcome.error_code,
            trajectory_points=outcome.trajectory_points,
            planning_latency_s=outcome.planning_latency_s,
            total_latency_s=time.monotonic() - run_start,
        )
        csv_path, md_path = _write_measurement(measurement, args.out_dir)
        print(f"G5_ARM_AVOIDANCE_CSV={csv_path}")
        print(f"G5_ARM_AVOIDANCE_REPORT={md_path}")
        if args.expected_verdict != "ANY":
            return 0 if expected_met else 2
        return 0 if ok else 2
    except KeyboardInterrupt:
        print("G5_ARM_AVOIDANCE_INTERRUPTED")
        return 130
    finally:
        if args.cleanup:
            try:
                _apply_scene_with_node(
                    node,
                    make_remove_scene(args.obstacle_id),
                    timeout_s=max(1.0, args.scene_timeout),
                )
            except Exception:  # noqa: BLE001
                pass
        try:
            executor.shutdown()
            executor_thread.join(timeout=1.0)
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="G5 机械臂 MoveIt 动态障碍避障验证")
    parser.add_argument("--target", nargs=3, type=float, default=DEFAULT_TARGET)
    parser.add_argument("--case-label", default="manual")
    parser.add_argument(
        "--expected-verdict",
        choices=[
            "ANY",
            "PLAN_OK",
            "PLAN_FAILED",
            "EXECUTE_OK",
            "EXECUTE_FAILED",
        ],
        default="ANY",
    )
    parser.add_argument("--target-frame", default="base_link")
    parser.add_argument("--target-link", default="gripper_tcp")
    parser.add_argument("--quat", nargs=4, type=float, default=DOWN_QUAT)
    parser.add_argument("--obstacle-id", default="g5_dynamic_box")
    parser.add_argument("--obstacle-frame", default="base_link")
    parser.add_argument("--obstacle-center", nargs=3, type=float, default=DEFAULT_BOX_CENTER)
    parser.add_argument("--obstacle-size", nargs=3, type=float, default=DEFAULT_BOX_SIZE)
    parser.add_argument("--planning-time", type=float, default=5.0)
    parser.add_argument("--planning-attempts", type=int, default=10)
    parser.add_argument("--start-joints", nargs=6, type=float, default=HOME_CONFIG)
    parser.add_argument("--max-velocity", type=float, default=0.30)
    parser.add_argument("--max-acceleration", type=float, default=0.30)
    parser.add_argument("--position-tolerance", type=float, default=0.005)
    parser.add_argument("--orientation-tolerance", type=float, default=0.2)
    parser.add_argument("--scene-timeout", type=float, default=5.0)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--out-dir", type=Path, default=Path(__file__).parent / "results")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--use-sim-time", action="store_true", default=True)
    parser.add_argument("--no-use-sim-time", action="store_false", dest="use_sim_time")
    parser.add_argument("--no-cleanup", action="store_false", dest="cleanup")
    parser.set_defaults(cleanup=True)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
