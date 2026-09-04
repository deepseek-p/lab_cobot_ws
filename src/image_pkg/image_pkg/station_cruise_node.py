#!/usr/bin/env python3
"""Navigate the robot through all work zones with a configurable dwell time."""
import math
import time
import json
from pathlib import Path
from threading import Thread

import cv2
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped, Twist
from moveit_msgs.msg import CollisionObject, PlanningScene, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image, JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
import tf2_ros
from pymoveit2 import MoveIt2


DEFAULT_ROUTE = [
    # The high-voltage probe kit is intentionally excluded from this
    # seven-object benchmark: it is inside the fenced ground zone and needs a
    # dedicated over-fence camera pose, not a table-workstation test.
    "home", "station_a", "tooling_zone", "aging_zone", "station_b", "home",
]
REMAINING_WORKSTATION_ROUTE = [
    "home", "tooling_zone", "aging_zone", "station_b", "home",
]
LATE_WORKSTATION_ROUTE = ["home", "aging_zone", "station_b", "home"]
STATION_A_ONLY_ROUTE = ["home", "station_a", "home"]
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
    # Gazebo spawns the chassis at world origin, whereas ``home`` is the
    # documented charging/return pad.  Reach that pad through the east/south
    # aisle before a cruise; a direct diagonal would cut through station B.
    "initial>home=1.30,0.00;1.30,-4.80;4.50,-4.80",
    "home>station_a=2.00,-4.20;2.00,2.38;-3.20,2.38",
    "station_a>home=-3.20,2.38;2.00,2.38;2.00,-4.20",
    # Direct route for re-testing the four objects outside station A.  It
    # stays in the south/east aisle, rather than first traversing the long
    # home -> station_a north-side route.
    "home>tooling_zone=-4.10,-4.20",
    "home>aging_zone=2.00,-4.20;2.00,3.10;1.00,3.10",
    "aging_zone>home=1.00,3.10;2.00,3.10;2.00,-4.20",
    "tooling_zone>home=-5.70,-3.40;-5.70,-4.80;4.50,-4.80;4.50,-4.20",
    "station_a>tooling_zone=-5.20,2.38;-5.20,-3.40",
    "station_a>inspection_zone=-5.70,2.38;-5.70,5.70;6.00,5.70;6.00,-0.30;4.10,-0.30",
    "inspection_zone>tooling_zone=6.00,1.10;6.00,-4.80;-5.70,-4.80;-5.70,-3.40",
    "tooling_zone>aging_zone=-2.00,-3.40;-2.00,3.10;-1.00,3.10",
    "aging_zone>station_b=1.50,3.10;1.50,-3.11",
    "station_b>home=4.50,-3.11",
]

STATION_LABELS = {
    "station_a": {"aruco_sample", "material_cube_red", "material_cube_green",
                  "material_cube_blue", "material_cube_yellow"},
    "tooling_zone": {"tooling_fixture_box", "tooling_hand_tools",
                     "board_test_fixture", "high_voltage_probe_kit",
                     "material_spare_igbt"},
    "aging_zone": {"aging_rack", "pcb_board"},
    "station_b": {"test_tube_rack", "test_tube", "beaker",
                  "erlenmeyer_flask", "graduated_cylinder"},
}
STATION_COARSE_LABEL = {
    "station_a": "material_cube_yellow",
    "tooling_zone": "high_voltage_probe_kit",
    "aging_zone": "aging_rack",
    "station_b": "test_tube_rack",
}
LABEL_TO_STATION = {
    label: station for station, labels in STATION_LABELS.items()
    for label in labels
}
STATION_TABLE_MODEL = {
    "station_a": "station_a_table",
    "tooling_zone": "tooling_zone_table",
    "aging_zone": "aging_zone_table",
    "station_b": "station_b_table",
}
# Side of the table used by the collision-clear approach corridor.  The final
# coordinate is calculated from the live table/object pose; these values only
# choose which aisle to use.
PREFERRED_OBSERVATION_SIDE = {
    "station_a": "south",
    "tooling_fixture_box": "north",
    "tooling_hand_tools": "north",
    "board_test_fixture": "south",
    "high_voltage_probe_kit": "south",
    "material_spare_igbt": "south",
    "aging_zone": "north",
    "station_b": "south",
}
LABEL_TO_MODEL = {
    "aruco_sample": "aruco_sample",
    "material_cube_red": "material_cube_red",
    "material_cube_green": "material_cube_green",
    "material_cube_blue": "material_cube_blue",
    "material_cube_yellow": "material_cube_yellow",
    "tooling_fixture_box": "tooling_fixture_box",
    "tooling_hand_tools": "tooling_hand_tools",
    "board_test_fixture": "board_test_fixture",
    "high_voltage_probe_kit": "high_voltage_probe_kit",
    "material_spare_igbt": "material_spare_igbt",
    "aging_rack": "aging_rack", "pcb_board": "pcb_board",
    # A single close wrist view is used for each repeated category.  The
    # evaluator preserves all physical instances and pairs by nearest origin.
    "test_tube_rack": "test_tube_rack_1", "test_tube": "test_tube_1",
    "beaker": "beaker_1", "erlenmeyer_flask": "erlenmeyer_flask",
    "graduated_cylinder": "graduated_cylinder",
}
# Collision-clear chassis paths for a close wrist-camera observation.  The
# final point of each path is derived from the mapped object position and the
# 1.6 x 1.2 m table / high-voltage boundary clearance; intermediate points
# keep the base from cutting through a workstation.  These are world/odom
# coordinates in this Gazebo world.
OBSERVATION_BASE_PATHS = {
    "aruco_sample": [(-4.16, 2.90)],
    "material_cube_red": [(-4.50, 2.90)],
    "material_cube_green": [(-4.30, 2.90)],
    "material_cube_blue": [(-4.50, 2.90)],
    "material_cube_yellow": [(-4.30, 2.90)],
    # The original east/west tooling views left the object 0.86 m to the
    # side of the UR base.  Adding the required camera height made the IK
    # radius exceed the UR workspace, so MoveIt could never sample a goal.
    # Approach from the north aisle instead: the final point stays outside
    # the 1.6 x 1.2 m table, puts the object 0.69--0.79 m ahead of the arm,
    # and is followed by an explicit base yaw alignment below.
    "tooling_fixture_box": [(-5.40, -3.50), (-5.40, -1.10), (-3.88, -1.10)],
    "tooling_hand_tools": [(-5.40, -3.50), (-5.40, -1.10), (-4.36, -1.10)],
    # These three objects are on the south row.  Observing them from the
    # north aisle put their centres 1.28 m from the arm base (unreachable).
    # Enter around the west edge and use the south aisle instead.
    "board_test_fixture": [(-5.40, -3.50), (-4.70, -3.50)],
    "high_voltage_probe_kit": [(-5.40, -3.50), (-4.30, -3.50)],
    "material_spare_igbt": [(-5.40, -3.50), (-3.62, -3.50)],
    # The chassis is 0.55 x 0.50 m.  y=5.08 left only 0.28 m between its
    # centre and the table edge, so small controller error could put the base
    # into the table.  Keep a measured 0.40 m edge clearance instead.
    "aging_rack": [(1.30, 3.10), (1.30, 5.20), (0.20, 5.20)],
    "pcb_board": [(1.30, 3.10), (1.30, 5.20), (0.55, 5.20)],
    # Station B is approached from its open south aisle.  The former north
    # route crossed the table footprint and inherited an invalid arm state
    # after the aging-zone failure.
    "test_tube_rack": [(1.30, -3.11), (0.48, -2.72)],
    "test_tube": [(1.30, -3.11), (0.36, -2.72)],
    "beaker": [(1.30, -3.11), (0.35, -2.72)],
    "erlenmeyer_flask": [(1.30, -3.11), (0.75, -2.72)],
    "graduated_cylinder": [(1.30, -3.11), (-0.30, -2.72)],
}
OBSERVATION_RETURN_PATHS = {
    "station_a": [(-5.40, 2.90), (-5.40, 2.38), (-4.30, 2.38)],
    "tooling_zone": [(-5.40, -1.10), (-5.40, -3.50), (-4.10, -3.50)],
    "aging_zone": [(1.30, 5.20), (1.30, 3.10), (0.20, 3.10)],
    "station_b": [(1.30, -2.72), (1.30, -3.11), (0.30, -3.11)],
}
UR_JOINTS = [
    "ur_shoulder_pan_joint", "ur_shoulder_lift_joint", "ur_elbow_joint",
    "ur_wrist_1_joint", "ur_wrist_2_joint", "ur_wrist_3_joint",
]
# Collision-safe transport pose.  A targeted wrist observation leaves the
# camera/gripper extended over the workstation.  The light Gazebo test pieces
# are dynamic, so moving the chassis sideways in that state can sweep them off
# the table and invalidate every later truth pose.  Chassis motion is therefore
# forbidden until this configuration has executed successfully.
SAFE_TRANSPORT_CONFIG = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
# Raised forward-looking station overview.  Unlike a truth-targeted Cartesian
# coarse pose, this joint-space pose cannot sweep laterally across table items.
COARSE_RAISED_CONFIG = [-0.116421, -0.807952, 0.425992,
                        -1.337190, -1.701999, -1.844921]

# Conservative local-axis bounds around each Gazebo model origin.  These are
# used only to choose and validate a camera view; they are never substituted
# for a visual pose estimate.  Each tuple is (xmin, xmax, ymin, ymax, zmin,
# zmax) in metres.
OBJECT_LOCAL_BOUNDS = {
    "aruco_sample": (-0.035, 0.035, -0.035, 0.035, -0.035, 0.035),
    "material_cube_red": (-0.035, 0.035, -0.035, 0.035, -0.035, 0.035),
    "material_cube_green": (-0.035, 0.035, -0.035, 0.035, -0.035, 0.035),
    "material_cube_blue": (-0.035, 0.035, -0.035, 0.035, -0.035, 0.035),
    "material_cube_yellow": (-0.035, 0.035, -0.035, 0.035, -0.035, 0.035),
    "tooling_fixture_box": (-0.060, 0.060, -0.170, 0.170, -0.100, 0.100),
    "tooling_hand_tools": (-0.060, 0.060, -0.170, 0.170, 0.000, 0.048),
    "board_test_fixture": (-0.160, 0.160, -0.110, 0.110, 0.000, 0.100),
    "high_voltage_probe_kit": (-0.170, 0.170, -0.035, 0.035, 0.000, 0.080),
    "material_spare_igbt": (-0.070, 0.070, -0.190, 0.190, -0.060, 0.060),
    "aging_rack": (-0.170, 0.170, -0.112, 0.112, -0.050, 0.055),
    "pcb_board": (-0.090, 0.090, -0.080, 0.080, 0.000, 0.145),
    "test_tube_rack": (-0.155, 0.155, -0.050, 0.050, 0.000, 0.066),
    "test_tube": (-0.013, 0.013, -0.013, 0.013, 0.000, 0.125),
    "beaker": (-0.033, 0.033, -0.033, 0.033, 0.000, 0.102),
    "erlenmeyer_flask": (-0.035, 0.035, -0.035, 0.035, 0.000, 0.120),
    "graduated_cylinder": (-0.023, 0.023, -0.023, 0.023, -0.070, 0.070),
}

# Spawn references are used only as an execution-safety invariant.  Dynamic
# Gazebo props can be launched off a table by a chassis/arm collision.  Once
# that happens, following the new live truth would make the robot chase the
# fallen prop across the room and would also invalidate the benchmark's
# workstation assumption.  Reject such a run instead of treating the moved
# model as a legitimate camera target.
EXPECTED_MODEL_WORLD_POSITIONS = {
    "tooling_fixture_box": (-3.88, -2.04, 0.80),
    "tooling_hand_tools": (-4.36, -1.96, 0.75),
    "board_test_fixture": (-4.70, -2.60, 0.75),
    "high_voltage_probe_kit": (-4.30, -2.60, 0.75),
    "material_spare_igbt": (-3.62, -2.60, 0.81),
    "aging_rack": (0.20, 4.26, 0.80),
    "pcb_board": (0.55, 4.30, 0.75),
    "test_tube_rack_1": (0.48, -1.85, 0.75),
    "test_tube_1": (0.36, -1.85, 0.762),
    "graduated_cylinder": (-0.30, -1.95, 0.82),
    "beaker_1": (0.35, -1.95, 0.75),
    "erlenmeyer_flask": (0.75, -1.95, 0.75),
}

TABLE_COLLISION_SIZES = {
    "station_a_table": (1.60, 1.20, 0.75),
    "tooling_zone_table": (1.60, 1.20, 0.75),
    "aging_zone_table": (1.60, 1.20, 0.75),
    "station_b_table": (1.60, 1.20, 0.75),
}
COLLISION_MODEL_LABELS = {
    "aruco_sample": "aruco_sample",
    "material_cube_red": "material_cube_red",
    "material_cube_green": "material_cube_green",
    "material_cube_blue": "material_cube_blue",
    "material_cube_yellow": "material_cube_yellow",
    "tooling_fixture_box": "tooling_fixture_box",
    "tooling_hand_tools": "tooling_hand_tools",
    "board_test_fixture": "board_test_fixture",
    "high_voltage_probe_kit": "high_voltage_probe_kit",
    "material_spare_igbt": "material_spare_igbt",
    "aging_rack": "aging_rack",
    "pcb_board": "pcb_board",
    "test_tube_rack_1": "test_tube_rack",
    "test_tube_rack_2": "test_tube_rack",
    **{"test_tube_%d" % index: "test_tube" for index in range(1, 10)},
    "graduated_cylinder": "graduated_cylinder",
    **{"beaker_%d" % index: "beaker" for index in range(1, 5)},
    "erlenmeyer_flask": "erlenmeyer_flask",
    "erlenmeyer_flask_2": "erlenmeyer_flask",
}


class StationCruise(Node):
    """Run the fixed work-zone route and dwell at each non-home stop."""

    def __init__(self):
        super().__init__("image_pkg_station_cruise")
        self.declare_parameter("dwell_seconds", 5.0)
        # In the supplied simulation Nav2 may accept a map goal while its
        # controller retains a stale command after cancellation.  The
        # Gazebo-truth safety-corridor controller is the deterministic default
        # for benchmark runs; enable Nav2 explicitly only after its map/odom
        # stack has been validated.
        self.declare_parameter("prefer_nav2_navigation", False)
        # Nav2 is retained when its map TF is healthy.  If it is not, fail
        # quickly into the Gazebo-world closed-loop safe-corridor controller.
        self.declare_parameter("navigation_timeout_seconds", 5.0)
        self.declare_parameter("nav2_startup_timeout_seconds", 8.0)
        self.declare_parameter("sensor_startup_timeout_seconds", 10.0)
        self.declare_parameter("odom_navigation_timeout_seconds", 600.0)
        self.declare_parameter("odom_fallback_on_nav_failure", True)
        self.declare_parameter("odom_goal_tolerance_m", 0.18)
        self.declare_parameter("gazebo_waypoint_tolerance_m", 0.30)
        self.declare_parameter("nav2_truth_settle_timeout_seconds", 5.0)
        self.declare_parameter("odom_max_speed_mps", 0.35)
        self.declare_parameter("odom_max_yaw_rate_rps", 0.8)
        self.declare_parameter("odom_yaw_tolerance_rad", 0.06)
        # ``/cmd_vel`` is owned by cmd_vel_safety_mux.  Publishing the
        # corridor controller directly there races its 30 Hz zero command;
        # use the mux's dedicated priority input instead.
        self.declare_parameter("base_command_topic", "/cmd_vel_safety")
        # Minimal/headless bringup can omit cmd_vel_safety_mux.  In that case
        # the Gazebo drive plugin still listens on /cmd_vel.  Use it only when
        # the configured safety input has no subscriber, so a full bringup
        # never bypasses the safety mux.
        self.declare_parameter("base_command_fallback_topic", "/cmd_vel")
        self.declare_parameter("route", DEFAULT_ROUTE)
        self.declare_parameter("remaining_workstations_only", False)
        self.declare_parameter("late_workstations_only", False)
        self.declare_parameter("station_a_only", False)
        # Optional exact-label filter for focused acceptance reruns.  The
        # blank default keeps the complete station inventory.
        self.declare_parameter("target_labels", [""])
        self.declare_parameter("waypoints", DEFAULT_WAYPOINTS)
        self.declare_parameter("safe_corridors", DEFAULT_SAFE_CORRIDORS)
        self.declare_parameter("detection_topic", "/yolo/detections")
        # The base-mounted RGB-D camera provides a broad, coarse world-frame
        # target hint.  It is used only to aim the wrist camera; all reported
        # localisation samples remain wrist RGB-D measurements.
        self.declare_parameter("overview_pose_topic", "/yolo/bench/poses")
        self.declare_parameter("overview_detection_topic", "/yolo/bench/detections")
        self.declare_parameter("overview_camera_info_topic", "/bench_camera/camera_info")
        self.declare_parameter("use_overview_hints", True)
        self.declare_parameter("overview_hint_max_age_sec", 2.0)
        self.declare_parameter("prefer_simulation_truth_for_observation", True)
        self.declare_parameter("use_overview_bearing_with_sim_truth", False)
        self.declare_parameter("overview_hint_truth_validation_m", 0.12)
        self.declare_parameter("pipeline_metrics_topic", "/perception/yolo/pipeline_metrics")
        self.declare_parameter("wrist_image_topic", "/wrist_camera/image_raw")
        self.declare_parameter("wrist_camera_info_topic", "/wrist_camera/camera_info")
        # An empty value disables disk capture.  Tests set this to an
        # image_pkg dataset folder, keeping real wrist-view samples separate
        # from generated benchmark summaries.
        self.declare_parameter("observation_capture_dir", "")
        self.declare_parameter("observation_capture_settle_seconds", 0.7)
        # CPU inference (and especially a 960 px recovery pass) trails the RGB
        # topic by more than 0.6 s.  Captures happen only after the base and arm
        # settle, so a bounded 2.5 s approximate-time match is both safe and
        # prevents a valid stationary detection being discarded as stale.
        self.declare_parameter("detection_sync_tolerance_seconds", 2.5)
        # CPU inference of the 17-class model can take 4--6 s while Gazebo,
        # MoveIt and both camera pipelines are active.  A three-second wait
        # discarded correct boxes that arrived just after a settled capture.
        self.declare_parameter("fresh_detection_wait_seconds", 8.0)
        self.declare_parameter("require_centered_yolo_before_statistics", True)
        # In simulation, the model origin projection is a better aiming target
        # than the centre of an asymmetric YOLO rectangle.  This switch only
        # validates that a correctly classified, unclipped target is in a safe
        # observation view; Gazebo truth is never forwarded to the pose node.
        self.declare_parameter("allow_truth_centered_view_acceptance", True)
        # 45 px is 7.0% of a 640 px image and still keeps the complete object
        # well inside the central field.  A 20 px gate was tighter than the
        # measured MoveIt/optical-frame projection residual (31--36 px), so it
        # rejected correctly aimed, fully visible objects before 3-D sampling.
        # An 80 px radius is 12.5% of the 640 px wrist image and keeps the
        # target in the central field while accommodating wide assemblies.
        # Centroid/pose accuracy does not improve by rejecting an otherwise
        # complete box at 46--75 px from the exact optical centre.
        self.declare_parameter("target_center_tolerance_pixels", 80.0)
        # Two percent plus the explicit edge-touch test below still rejects
        # clipped boxes.  Imported wide meshes can be complete with only a
        # 20--48 px side margin, so the former 8% rule was not a valid proxy
        # for occlusion.
        self.declare_parameter("target_edge_margin_fraction", 0.02)
        self.declare_parameter("max_target_area_fraction", 0.78)
        self.declare_parameter("require_station_detection", True)
        self.declare_parameter("per_object_observe_seconds", 8.0)
        # The CPU detector and RGB-D temporal median can finish after the
        # first accepted YOLO frame.  Keep the arm still long enough for the
        # same-view 3-D estimate to enter the multiview cache before moving.
        self.declare_parameter("post_view_rgbd_settle_seconds", 5.0)
        self.declare_parameter("coarse_observe_seconds", 3.0)
        # 0.45 m is outside the reachable workspace at several table-edge
        # approach poses.  A 0.30 m optical clearance remains a raised,
        # station-level view while preserving enough field of view.
        self.declare_parameter("coarse_camera_height_m", 0.30)
        self.declare_parameter("min_stable_detection_frames", 2)
        # Camera-optical clearance above the object, not TCP height.  The
        # tooling north-aisle pose is deliberately close enough for 0.15--
        # 0.30 m clearance; this keeps the 0.85 m UR workspace feasible.
        # Try the widest useful field of view first.  Starting at 0.15 m made
        # 7 cm objects fill/cross the image boundary, so a valid geometric aim
        # could still be unusable by YOLO.  Lower poses remain fallbacks for IK.
        self.declare_parameter("camera_view_heights_m", [0.45, 0.38, 0.30, 0.22])
        self.declare_parameter("max_dynamic_camera_height_m", 0.50)
        self.declare_parameter("target_extent_fraction", 0.72)
        self.declare_parameter("fine_camera_xy_tolerance_m", 0.035)
        self.declare_parameter("fine_camera_min_view_distance_m", 0.30)
        self.declare_parameter("fine_camera_max_view_distance_m", 0.50)
        # Two independent azimuths reduce the visible-surface-centroid bias
        # and provide a validation view that was not used to centre the first.
        self.declare_parameter(
            "fine_view_azimuth_offsets_rad", [-0.16, 0.0, 0.16])
        self.declare_parameter("minimum_confirmed_views", 3)
        self.declare_parameter("use_oblique_fine_views", True)
        self.declare_parameter("oblique_camera_planar_reach_m", 0.58)
        self.declare_parameter("base_body_half_extent_m", 0.30)
        # The mecanum body/wheels and a yawed arm base protrude beyond the old
        # 0.30 m square approximation.  A 0.25 m table-edge margin prevents
        # the observed table contact that launched dynamic props off-station.
        # This is clearance from the chassis envelope to the workstation,
        # not from the chassis centre.  Fifteen centimetres remains collision
        # clear while letting the wrist camera enter the required 0.30--0.50 m
        # observation shell around targets near the far table edge.
        self.declare_parameter("base_table_clearance_m", 0.15)
        self.declare_parameter("max_base_tilt_rad", 0.035)
        self.declare_parameter("max_target_spawn_displacement_m", 0.20)
        # Thin tools can occupy only 0.1--0.3% of a 640x480 wrist frame even
        # when fully visible.  Class/centre gating happens in the detector;
        # accept these measured boxes for the bounded recenter controller.
        self.declare_parameter("recenter_min_box_area_fraction", 0.001)
        # Must not be higher than the target-gated recovery thresholds in
        # yolo_world_node (the smallest audited class threshold is 0.001).
        # Association is already constrained to the named class and central
        # optical-axis gate; bounded motion plus rollback protects against a
        # weak proposal moving the camera in the wrong direction.
        self.declare_parameter("recenter_min_confidence", 0.001)
        self.declare_parameter("recenter_gain", 0.45)
        self.declare_parameter("recenter_max_step_m", 0.03)
        self.declare_parameter("recenter_max_total_m", 0.08)
        self.declare_parameter("observation_motion_timeout_seconds", 35.0)
        self.declare_parameter("arm_stow_timeout_seconds", 25.0)
        self.declare_parameter("state_validity_service", "/check_state_validity")
        self.declare_parameter("apply_planning_scene_service", "/apply_planning_scene")
        self.declare_parameter("state_validity_timeout_seconds", 3.0)
        # Healthy Gazebo position-control jitter is about 0.02--0.08 rad/s;
        # the observed physics divergence was 11--275 rad/s.  This threshold
        # separates the two while still requiring a continuous settle window.
        self.declare_parameter("max_arm_joint_speed_rps", 0.20)
        self.declare_parameter("arm_settle_seconds", 0.35)
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
        if bool(self.get_parameter("remaining_workstations_only").value):
            self.route = list(REMAINING_WORKSTATION_ROUTE)
        if bool(self.get_parameter("late_workstations_only").value):
            self.route = list(LATE_WORKSTATION_ROUTE)
        if bool(self.get_parameter("station_a_only").value):
            self.route = list(STATION_A_ONLY_ROUTE)
        self.waypoints = _parse_waypoints(self.get_parameter("waypoints").value)
        self.safe_corridors = _parse_corridors(
            self.get_parameter("safe_corridors").value)
        unknown = [name for name in self.route if name not in self.waypoints]
        if unknown:
            raise ValueError(f"route contains unknown waypoint(s): {unknown}")
        self.status_pub = self.create_publisher(String, "/image_pkg/cruise/status", 10)
        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("base_command_topic").value), 10)
        fallback_topic = str(
            self.get_parameter("base_command_fallback_topic").value).strip()
        primary_topic = str(self.get_parameter("base_command_topic").value).strip()
        self.cmd_fallback_pub = (
            self.create_publisher(Twist, fallback_topic, 10)
            if fallback_topic and fallback_topic != primary_topic else None
        )
        self._reported_command_fallback = False
        self.odom = None
        self.truth_positions = {}
        self.model_poses = {}
        self.detected_labels = set()
        self.detection_counts = {}
        self.pipeline_counts = {}
        self._counted_pipeline_events = set()
        self.overview_hints = {}
        self.overview_boxes = {}
        self.overview_camera_info = None
        # Incremental world-frame correction derived only from a synchronised
        # wrist YOLO box.  This closes the last few image pixels left by the
        # broad camera's coarse depth estimate; Gazebo truth is never used.
        self.wrist_centering_offsets = {}
        self.wrist_centering_state = {}
        self._counted_detection_timestamps = set()
        self.latest_detection_payload = None
        self.latest_wrist_image = None
        self.wrist_camera_info = None
        self.current_arm_joints = {}
        self.current_arm_velocities = {}
        self.latest_joint_state = None
        self._last_joint_positions = {}
        self._last_joint_state_monotonic = None
        self.bridge = CvBridge()
        self.capture_index = 0
        self.create_subscription(Odometry, "/odom", self._odom_cb, 20)
        self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb,
            qos_profile_sensor_data)
        self.create_subscription(ModelStates, "/gazebo/model_states", self._models_cb, 10)
        self.create_subscription(
            Image, self.get_parameter("wrist_image_topic").value,
            self._wrist_image_cb, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, self.get_parameter("wrist_camera_info_topic").value,
            self._camera_info_cb, qos_profile_sensor_data)
        self.create_subscription(String, self.get_parameter("detection_topic").value,
                                 self._detections_cb, 10)
        self.create_subscription(
            String, self.get_parameter("overview_pose_topic").value,
            self._overview_poses_cb, 10)
        self.create_subscription(
            String, self.get_parameter("overview_detection_topic").value,
            self._overview_detections_cb, 10)
        self.create_subscription(
            CameraInfo, self.get_parameter("overview_camera_info_topic").value,
            self._overview_camera_info_cb, qos_profile_sensor_data)
        self.create_subscription(
            String, self.get_parameter("pipeline_metrics_topic").value,
            self._pipeline_metrics_cb, 50)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.state_validity_client = self.create_client(
            GetStateValidity,
            str(self.get_parameter("state_validity_service").value))
        self.apply_scene_client = self.create_client(
            ApplyPlanningScene,
            str(self.get_parameter("apply_planning_scene_service").value))
        self.moveit_callback_group = ReentrantCallbackGroup()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # MoveIt plans in the UR arm base.  ``base_link`` is deliberately not
        # used here: this Gazebo model publishes ``base_footprint`` and
        # ``ur_base_link``, but no TF frame named ``base_link``.
        self.moveit = MoveIt2(
            node=self,
            joint_names=UR_JOINTS,
            base_link_name="ur_base_link",
            end_effector_name="ur_tool0",
            group_name="ur_manipulator",
            callback_group=self.moveit_callback_group,
        )
        self.moveit.max_velocity = 0.35
        self.moveit.max_acceleration = 0.35
        self.moveit.allowed_planning_time = 4.0
        self.moveit.num_planning_attempts = 3

    def run(self):
        self._status("WAITING_FOR_NAV2")
        nav2_deadline = time.monotonic() + float(
            self.get_parameter("nav2_startup_timeout_seconds").value)
        nav2_available = False
        if bool(self.get_parameter("prefer_nav2_navigation").value):
            while rclpy.ok() and time.monotonic() < nav2_deadline:
                if self.nav_client.wait_for_server(timeout_sec=1.0):
                    nav2_available = True
                    break
                self.get_logger().info("waiting for navigate_to_pose action server")
        else:
            self.get_logger().info("NAV2_BYPASSED_SAFE_CORRIDOR")
        if not rclpy.ok():
            return False
        if not nav2_available:
            if not bool(self.get_parameter("odom_fallback_on_nav_failure").value):
                self._status("FAILED:navigation:nav2_unavailable")
                return False
            self._status("NAV2_UNAVAILABLE_FALLBACK_ODOM")
        sensor_deadline = time.monotonic() + float(
            self.get_parameter("sensor_startup_timeout_seconds").value)
        self._status("WAITING_FOR_GAZEBO_AND_JOINT_STATES")
        while rclpy.ok() and time.monotonic() < sensor_deadline:
            if self.model_poses.get("lab_cobot") is not None \
                    and self.latest_joint_state is not None:
                break
            time.sleep(0.05)
        if (self.model_poses.get("lab_cobot") is None
                or self.latest_joint_state is None):
            self._status("FAILED:startup:sensor_state_unavailable")
            return False
        self._status("SENSOR_STATE_READY")
        for index, name in enumerate(self.route):
            # Nav2 may reject a zero-length initial-home goal, but Gazebo
            # currently spawns the robot at world origin rather than on the
            # documented home pad.  Only skip this leg after checking physical
            # world truth; otherwise first use the dedicated safe corridor.
            if index == 0 and name == "home":
                if self._gazebo_at_waypoint("home"):
                    self._status("ARRIVED:home:initial")
                    continue
                if not self._stow_arm_before_base_motion("navigation:initial>home"):
                    self._status("FAILED:unsafe_arm:navigation:initial>home")
                    return False
                self._status("NAVIGATING:home:initial_alignment")
                reached = self._navigate("home") if nav2_available else False
                if not reached and bool(self.get_parameter("odom_fallback_on_nav_failure").value):
                    self._status("NAV2_FAILED_FALLBACK_ODOM:home:initial_alignment")
                    reached = self._navigate_odom("home", "initial")
                if not reached:
                    self._status("FAILED:navigation:home:initial_alignment")
                    return False
                self._status("ARRIVED:home:initial_alignment")
                continue
            previous_name = self.route[index - 1]
            if not self._stow_arm_before_base_motion(
                    f"navigation:{previous_name}>{name}"):
                self._status(f"FAILED:unsafe_arm:navigation:{previous_name}>{name}")
                return False
            self._status(f"NAVIGATING:{name}")
            reached = self._navigate(name) if nav2_available else False
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
        if result_future.result().status != GoalStatus.STATUS_SUCCEEDED:
            return False
        # A misaligned map->odom transform can make Nav2 report success while
        # the Gazebo chassis has not moved to the requested workstation.  Do
        # not start camera evaluation until the physical simulation agrees.
        return self._wait_for_gazebo_waypoint(name)

    def _wait_for_gazebo_waypoint(self, name):
        target_x, target_y, _ = self.waypoints[name]
        tolerance = float(self.get_parameter("gazebo_waypoint_tolerance_m").value)
        deadline = time.monotonic() + float(
            self.get_parameter("nav2_truth_settle_timeout_seconds").value)
        while rclpy.ok() and time.monotonic() < deadline:
            robot_pose = self.model_poses.get("lab_cobot")
            if robot_pose is not None and math.hypot(
                    robot_pose.position.x - target_x,
                    robot_pose.position.y - target_y) <= tolerance:
                return True
            time.sleep(0.05)
        self.get_logger().warning(
            "Nav2 reported %s reached but Gazebo chassis did not reach its waypoint" % name)
        return False

    def _gazebo_at_waypoint(self, name):
        """Return whether the physical Gazebo chassis is already at a waypoint."""
        pose = self.model_poses.get("lab_cobot")
        if pose is None:
            return False
        target_x, target_y, _ = self.waypoints[name]
        tolerance = float(self.get_parameter("gazebo_waypoint_tolerance_m").value)
        return math.hypot(pose.position.x - target_x, pose.position.y - target_y) <= tolerance

    def _odom_cb(self, msg):
        self.odom = msg

    def _joint_state_cb(self, msg):
        now = time.monotonic()
        dt = (None if self._last_joint_state_monotonic is None
              else now - self._last_joint_state_monotonic)
        # gazebo_ros2_control's reported velocity is not consistent with its
        # joint position in this model (for example wrist_3 reports about
        # 0.6 rad/s while changing only ~1e-4 rad over 0.8 s).  Stability must
        # therefore use measured position change between received samples.
        for name, position in zip(msg.name, msg.position):
            if name not in UR_JOINTS:
                continue
            value = float(position)
            if dt is not None and dt > 1e-4 and name in self._last_joint_positions:
                velocity = _wrap_angle(value - self._last_joint_positions[name]) / dt
            else:
                velocity = float("inf")
            self.current_arm_joints[name] = value
            self.current_arm_velocities[name] = velocity
            self._last_joint_positions[name] = value
        self._last_joint_state_monotonic = now
        self.latest_joint_state = msg

    def _models_cb(self, msg):
        self.model_poses = {name: pose for name, pose in zip(msg.name, msg.pose)}
        self.truth_positions = {
            name: (pose.position.x, pose.position.y, pose.position.z)
            for name, pose in zip(msg.name, msg.pose)
        }

    def _detections_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            timestamp = float(payload.get("timestamp"))
            labels = {
                str(item.get("label", "")).strip()
                for item in payload.get("detections", [])
                if isinstance(item, dict)
            }
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        self.detected_labels = labels
        self.latest_detection_payload = payload
        # Count only independent source images.  A callback can be repeated
        # by ROS transport, which must not make one box look like a stable
        # observation.
        for label in labels:
            key = (label, timestamp)
            if key not in self._counted_detection_timestamps:
                self._counted_detection_timestamps.add(key)
                self.detection_counts[label] = self.detection_counts.get(label, 0) + 1

    def _overview_poses_cb(self, msg):
        """Cache the broad camera's world estimates as wrist-aiming hints."""
        try:
            payload = json.loads(msg.data)
            timestamp = float(payload.get("timestamp"))
            if str(payload.get("frame_id", "")) != "world":
                return
            values = payload.get("detections", [])
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        for item in values:
            if not isinstance(item, dict):
                continue
            label = str(item.get("label", "")).strip()
            position = item.get("position", [])
            if label not in LABEL_TO_MODEL or len(position) != 3:
                continue
            try:
                xyz = tuple(float(value) for value in position)
            except (TypeError, ValueError):
                continue
            if all(math.isfinite(value) for value in xyz):
                self.overview_hints[label] = {
                    "position": xyz, "timestamp": timestamp,
                    "confidence": float(item.get("confidence", 0.0)),
                }

    def _overview_camera_info_cb(self, msg):
        self.overview_camera_info = msg

    def _overview_detections_cb(self, msg):
        """Cache a live overview bearing even when broad-camera depth fails."""
        try:
            payload = json.loads(msg.data)
            timestamp = float(payload.get("timestamp"))
            detections = payload.get("detections", [])
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        for item in detections:
            if not isinstance(item, dict):
                continue
            label = str(item.get("label", "")).strip()
            box = item.get("box", [])
            if label not in LABEL_TO_MODEL or len(box) != 4:
                continue
            try:
                x1, y1, x2, y2 = (float(value) for value in box)
            except (TypeError, ValueError):
                continue
            self.overview_boxes[label] = {
                "center": ((x1 + x2) / 2.0, (y1 + y2) / 2.0),
                "timestamp": timestamp,
                "confidence": float(item.get("confidence", 0.0)),
            }

    def _pipeline_metrics_cb(self, msg):
        """Count wrist stages by source image, never camera-rate duplicates."""
        try:
            payload = json.loads(msg.data)
            stage = str(payload.get("stage", ""))
            label = str(payload.get("label", ""))
            timestamp = float(payload.get("measurement_stamp_sec"))
            source = str(payload.get("camera_source", "wrist"))
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if source != "wrist" or stage not in {
                "yolo_box", "valid_depth", "world_transform", "pose_ready"}:
            return
        key = (label, stage, timestamp)
        if key not in self._counted_pipeline_events:
            self._counted_pipeline_events.add(key)
            counts = self.pipeline_counts.setdefault(label, {})
            counts[stage] = counts.get(stage, 0) + 1

    def _wrist_image_cb(self, msg):
        """Keep the latest wrist frame for per-observation evidence capture."""
        self.latest_wrist_image = msg

    def _camera_info_cb(self, msg):
        if msg.k[0] > 0.0 and msg.k[4] > 0.0 and msg.width > 0 and msg.height > 0:
            self.wrist_camera_info = msg

    def _navigate_odom(self, name, previous_name):
        """Use a collision-clear corridor when Nav2 lacks a map->odom TF."""
        # Validate once while the chassis is stopped.  Gazebo reports small
        # arm-joint controller velocities while the whole mobile base moves;
        # rechecking those values at every corridor vertex falsely classified
        # a safely stowed arm as moving.  Every caller stows the arm first.
        if not self._arm_safe_for_base_motion():
            self._status("BASE_MOTION_REFUSED:arm_not_stable_or_valid")
            self._stop_robot()
            return False
        target_x, target_y, target_yaw = self.waypoints[name]
        tolerance = float(self.get_parameter("odom_goal_tolerance_m").value)
        max_speed = float(self.get_parameter("odom_max_speed_mps").value)
        # Nav2 can be unavailable (for example no map->odom transform) while
        # odometry fallback remains able to cover a long inter-station leg.
        # Keep its timeout independent from the short Nav2 failover timeout.
        path = self.safe_corridors.get((previous_name, name), [])
        path = [*path, (target_x, target_y)]
        for point_x, point_y in path:
            # Gazebo can run below real-time while YOLO/MoveIt are active.
            # Give each collision-clear corridor leg its own timeout instead
            # of spending one shared deadline on the entire multi-leg route.
            deadline = time.monotonic() + self.odom_timeout_seconds
            if not self._drive_gazebo_point(point_x, point_y, tolerance, max_speed, deadline):
                self._stop_robot()
                return False
        # At the final camera-facing approach pose, turn the robot to the
        # configured yaw.  This is essential for seeing the table/ground item.
        deadline = time.monotonic() + self.odom_timeout_seconds
        while rclpy.ok() and time.monotonic() < deadline:
            time.sleep(0.05)
            pose = self.model_poses.get("lab_cobot")
            if pose is None:
                continue
            yaw_error = _wrap_angle(target_yaw - _yaw_from_quaternion(pose.orientation))
            if abs(yaw_error) <= float(self.get_parameter("odom_yaw_tolerance_rad").value):
                self._stop_robot()
                return True
            command = Twist()
            command.angular.z = max(-float(self.get_parameter("odom_max_yaw_rate_rps").value),
                                    min(float(self.get_parameter("odom_max_yaw_rate_rps").value), 1.6 * yaw_error))
            self._publish_base_command(command)
        self._stop_robot()
        return False

    def _drive_gazebo_point(self, target_x, target_y, tolerance, max_speed, deadline):
        """Translate to a Gazebo-world point without mixing it with odom.

        The documented work-zone waypoints and safety corridors are Gazebo
        world coordinates.  During AMCL warm-up odom may be offset, so using
        odom as the feedback source can drive the physical base out of bounds.
        ``/gazebo/model_states`` supplies the same world frame as the target.
        """
        while rclpy.ok() and time.monotonic() < deadline:
            time.sleep(0.05)
            pose = self.model_poses.get("lab_cobot")
            if pose is None:
                continue
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
            self._publish_base_command(command)
        return False

    def _stop_robot(self):
        self._publish_base_command(Twist())

    def _publish_base_command(self, command):
        """Publish through the safety input, or a verified headless fallback."""
        if self.cmd_pub.get_subscription_count() > 0:
            self.cmd_pub.publish(command)
            return
        if (self.cmd_fallback_pub is not None
                and self.cmd_fallback_pub.get_subscription_count() > 0):
            if not self._reported_command_fallback:
                self.get_logger().warning(
                    "Configured base command topic has no subscriber; "
                    "using headless fallback topic")
                self._reported_command_fallback = True
            self.cmd_fallback_pub.publish(command)
            return
        self.get_logger().error(
            "No subscriber on either configured base command topic",
            throttle_duration_sec=2.0)

    def _move_base_for_observation(self, label):
        """Move outside the table/fence to the camera's close observation base."""
        if not self._target_is_at_workstation(label):
            return False
        path = list(OBSERVATION_BASE_PATHS.get(label, []))
        if not path:
            return True
        dynamic_point = self._dynamic_observation_base_point(label)
        if dynamic_point is not None:
            path[-1] = dynamic_point
        if not self._arm_safe_for_base_motion():
            self._status("BASE_MOTION_REFUSED:arm_not_stable_or_valid")
            self._stop_robot()
            return False
        # The first object at each station enters a collision-clear observation
        # aisle through all path points.  Once already in that aisle, forcing
        # every following object through the entry path would make the robot
        # backtrack several metres for no safety benefit.  Keep only the final
        # along-aisle point when the live Gazebo pose proves that the base is
        # already on the correct side of the workstation.
        robot = self.model_poses.get("lab_cobot")
        if robot is not None and len(path) > 1:
            if label in {"tooling_fixture_box", "tooling_hand_tools"} \
                    and robot.position.y > -1.65:
                path = [path[-1]]
            elif label in {"board_test_fixture", "high_voltage_probe_kit",
                           "material_spare_igbt"} and robot.position.y < -3.00:
                path = [path[-1]]
            elif label in STATION_LABELS["aging_zone"] and robot.position.y > 4.75:
                path = [path[-1]]
            elif label in STATION_LABELS["station_b"] and robot.position.y < -2.50:
                path = [path[-1]]
        tolerance = min(0.12, float(self.get_parameter("odom_goal_tolerance_m").value))
        for x, y in path:
            deadline = time.monotonic() + self.odom_timeout_seconds
            self._status("OBSERVATION_BASE:%s:%.2f,%.2f" % (label, x, y))
            if not self._drive_gazebo_point(x, y, tolerance, 0.20, deadline):
                self._stop_robot()
                self.get_logger().warning("Cannot reach close observation base for %s" % label)
                return False
            if not self._base_is_upright(label):
                self._stop_robot()
                return False
        return self._face_observation_target(label, deadline)

    def _target_is_at_workstation(self, label):
        """Reject a dynamic prop that has already been knocked off its station."""
        model_name = LABEL_TO_MODEL.get(label)
        expected = EXPECTED_MODEL_WORLD_POSITIONS.get(model_name)
        pose = self.model_poses.get(model_name)
        if expected is None or pose is None:
            return pose is not None
        values = (pose.position.x, pose.position.y, pose.position.z)
        if not all(math.isfinite(value) for value in values):
            self._status("TARGET_TRUTH_INVALID:%s" % label)
            return False
        displacement = math.sqrt(sum(
            (values[index] - expected[index]) ** 2 for index in range(3)))
        limit = float(self.get_parameter(
            "max_target_spawn_displacement_m").value)
        if displacement > limit:
            self._status(
                "TARGET_DISPLACED:%s:distance=%.3f:xyz=%.3f,%.3f,%.3f" %
                (label, displacement, *values))
            return False
        return True

    def _base_is_upright(self, context):
        """Do not move/aim the arm after a chassis-to-workstation collision."""
        pose = self.model_poses.get("lab_cobot")
        if pose is None:
            return False
        q = pose.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm < 1e-9 or not math.isfinite(norm):
            self._status("BASE_POSE_INVALID:%s" % context)
            return False
        x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
        # Angle between the robot's local +Z axis and Gazebo world +Z.
        upright_cosine = max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))
        tilt = math.acos(upright_cosine)
        limit = float(self.get_parameter("max_base_tilt_rad").value)
        if tilt > limit:
            self._status(
                "BASE_TILT_REJECTED:%s:tilt_rad=%.4f:limit=%.4f" %
                (context, tilt, limit))
            return False
        return True

    def _dynamic_observation_base_point(self, label):
        """Return a live collision-clear edge pose aligned with one object.

        Hard-coded edge coordinates accumulated 0.10--0.20 m of unnecessary
        clearance.  That is enough to make a top-down wrist target unreachable.
        Keep the selected safe aisle, but calculate its final point from the
        current table/object poses and the actual chassis envelope.
        """
        station = LABEL_TO_STATION.get(label)
        table_name = STATION_TABLE_MODEL.get(station)
        table = self.model_poses.get(table_name)
        target = self.model_poses.get(LABEL_TO_MODEL.get(label, ""))
        size = TABLE_COLLISION_SIZES.get(table_name)
        if table is None or target is None or size is None:
            return None
        side = PREFERRED_OBSERVATION_SIDE.get(
            label, PREFERRED_OBSERVATION_SIDE.get(station))
        clearance = (
            float(self.get_parameter("base_body_half_extent_m").value)
            + float(self.get_parameter("base_table_clearance_m").value))
        half_x, half_y = float(size[0]) / 2.0, float(size[1]) / 2.0
        inset = 0.12
        x_low, x_high = table.position.x - half_x + inset, table.position.x + half_x - inset
        y_low, y_high = table.position.y - half_y + inset, table.position.y + half_y - inset
        if side == "north":
            point = (max(x_low, min(x_high, target.position.x)),
                     table.position.y + half_y + clearance)
        elif side == "south":
            point = (max(x_low, min(x_high, target.position.x)),
                     table.position.y - half_y - clearance)
        elif side == "east":
            point = (table.position.x + half_x + clearance,
                     max(y_low, min(y_high, target.position.y)))
        elif side == "west":
            point = (table.position.x - half_x - clearance,
                     max(y_low, min(y_high, target.position.y)))
        else:
            return None
        self._status(
            "OBSERVATION_BASE_DYNAMIC:%s:%s:%.3f,%.3f" %
            (label, side, point[0], point[1]))
        return point

    def _face_observation_target(self, label, deadline):
        """Yaw the mobile base so the target is in front of the UR arm.

        Translating to a close point alone left the arm's target lateral to
        its workspace.  The final yaw is calculated from current Gazebo truth
        solely to generate an observation pose; RGB/YOLO still decides
        whether the target is actually visible and centred.
        """
        target = self._observation_target_position(label)
        if target is None:
            return False
        yaw_tolerance = float(self.get_parameter("odom_yaw_tolerance_rad").value)
        max_yaw_rate = float(self.get_parameter("odom_max_yaw_rate_rps").value)
        while rclpy.ok() and time.monotonic() < deadline:
            time.sleep(0.05)
            robot = self.model_poses.get("lab_cobot")
            if robot is None:
                continue
            robot_yaw = _yaw_from_quaternion(robot.orientation)
            desired_yaw = math.atan2(
                target[1] - robot.position.y,
                target[0] - robot.position.x)
            overview = self.overview_boxes.get(label)
            info = self.overview_camera_info
            has_sim_truth = self.model_poses.get(LABEL_TO_MODEL[label]) is not None
            if (overview is not None and info is not None
                    and (not has_sim_truth or bool(self.get_parameter(
                        "use_overview_bearing_with_sim_truth").value))):
                now = self.get_clock().now().nanoseconds / 1e9
                age = now - float(overview["timestamp"])
                if -0.1 <= age <= float(
                        self.get_parameter("overview_hint_max_age_sec").value):
                    try:
                        u, v = overview["center"]
                        ray_camera = (
                            (u - float(info.k[2])) / float(info.k[0]),
                            (v - float(info.k[5])) / float(info.k[4]),
                            1.0,
                        )
                        base_from_camera = self.tf_buffer.lookup_transform(
                            "base_footprint", str(info.header.frame_id), Time(),
                            timeout=Duration(seconds=0.1))
                        rotation = base_from_camera.transform.rotation
                        ray_base = _rotate_point(
                            ray_camera,
                            (rotation.x, rotation.y, rotation.z, rotation.w))
                        bearing = math.atan2(ray_base[1], ray_base[0])
                        if abs(bearing) <= 1.2:
                            desired_yaw = robot_yaw + bearing
                            self.get_logger().info(
                                "OVERVIEW_BEARING_USED:%s:error_rad=%.3f" %
                                (label, bearing), throttle_duration_sec=1.0)
                    except Exception as exc:
                        self.get_logger().debug(
                            "Overview bearing unavailable for %s: %s" %
                            (label, exc))
            yaw_error = _wrap_angle(desired_yaw - robot_yaw)
            if abs(yaw_error) <= yaw_tolerance:
                self._stop_robot()
                self._status("OBSERVATION_BASE_ALIGNED:%s" % label)
                return True
            command = Twist()
            command.angular.z = max(-max_yaw_rate, min(max_yaw_rate, 1.6 * yaw_error))
            self._publish_base_command(command)
        self._stop_robot()
        self.get_logger().warning("Cannot align observation base for %s" % label)
        return False

    def _observation_target_position(self, label):
        """Select a fresh overview hint, with truth only as simulation fallback.

        This selection drives *only* the robot/camera observation pose.  It
        never reaches the YOLO/RGB-D pose message or the error evaluator.
        """
        hint = self.overview_hints.get(label)
        target = self.model_poses.get(LABEL_TO_MODEL[label])
        truth_position = None if target is None else (
            target.position.x, target.position.y, target.position.z)
        if (truth_position is not None and bool(self.get_parameter(
                "prefer_simulation_truth_for_observation").value)):
            if hint is not None:
                separation = math.sqrt(sum(
                    (float(hint["position"][index]) - truth_position[index]) ** 2
                    for index in range(3)))
                limit = float(self.get_parameter(
                    "overview_hint_truth_validation_m").value)
                self._status(
                    "OVERVIEW_HINT_%s:%s:truth_separation=%.3f" %
                    ("CONFIRMED" if separation <= limit else "REJECTED",
                     label, separation))
            # Gazebo truth controls only the camera observation pose.  It is
            # never published as a visual estimate or accepted by benchmark.
            return self._apply_wrist_centering_offset(label, truth_position)
        if bool(self.get_parameter("use_overview_hints").value) and hint:
            now = self.get_clock().now().nanoseconds / 1e9
            maximum_age = float(
                self.get_parameter("overview_hint_max_age_sec").value)
            age = now - hint["timestamp"]
            if age >= -0.1 and age <= maximum_age:
                position = self._apply_wrist_centering_offset(
                    label, hint["position"])
                self._status(
                    "OVERVIEW_HINT_USED:%s:xyz=%.3f,%.3f,%.3f:confidence=%.3f" % (
                        label, *position, hint["confidence"]))
                return position
        if target is None:
            return None
        self._status("OVERVIEW_HINT_UNAVAILABLE:%s:using_simulation_observation_fallback" % label)
        return self._apply_wrist_centering_offset(
            label, (target.position.x, target.position.y, target.position.z))

    def _apply_wrist_centering_offset(self, label, position):
        offset = self.wrist_centering_offsets.get(label, (0.0, 0.0, 0.0))
        return tuple(float(position[index]) + float(offset[index])
                     for index in range(3))

    def _update_wrist_centering_offset(self, label, view_height, details):
        """Convert a measured wrist-box pixel offset to one world XY correction.

        The correction is an observation-control command only.  It is made
        from the detection box, focal length and live TF; it neither reads nor
        writes object truth or the evaluated 3-D position.
        """
        box = details.get("box_pixels") if isinstance(details, dict) else None
        if not box or self.wrist_camera_info is None:
            return False
        image_width, image_height = details.get("image_size", [0, 0])
        if image_width <= 0 or image_height <= 0:
            return False
        area_fraction = float(details.get("box_area_fraction", 0.0))
        confidence = float(details.get("best_confidence", 0.0))
        if (bool(details.get("box_touches_image_edge", True))
                or area_fraction < float(self.get_parameter(
                    "recenter_min_box_area_fraction").value)
                or confidence < float(self.get_parameter(
                    "recenter_min_confidence").value)
                or self.detection_counts.get(label, 0) < 2):
            self._status(
                "WRIST_PIXEL_RECENTER_REJECTED:%s:edge=%s:area=%.4f:confidence=%.3f:frames=%d" % (
                    label, details.get("box_touches_image_edge", True),
                    area_fraction, confidence,
                    self.detection_counts.get(label, 0)))
            return False
        try:
            x1, y1, x2, y2 = (float(value) for value in box)
            info = self.wrist_camera_info
            fx, fy = float(info.k[0]), float(info.k[4])
            if fx <= 0.0 or fy <= 0.0:
                return False
            box_center = ((x1 + x2) / 2.0, (y1 + y2) / 2.0)
            projection = details.get("truth_projection", {})
            projected_center = projection.get("projected_center_pixels")
            # In simulation the model projection is allowed to steer only the
            # camera.  Composite/upright objects can have a valid YOLO box
            # whose visual centre is far from the model origin (PCB backboard
            # is the clearest case); centring that box moves the optical axis
            # away from the requested model-origin observation.
            control_center = (
                tuple(float(value) for value in projected_center)
                if (bool(self.get_parameter(
                    "prefer_simulation_truth_for_observation").value)
                    and isinstance(projected_center, (list, tuple))
                    and len(projected_center) == 2)
                else box_center
            )
            # Optical +X/+Y displacement of the observed target.  Moving the
            # camera by the same lateral vector brings that target toward the
            # image centre.  Clearance is a conservative range estimate for
            # the near top-down inspection configuration.
            clearance = max(0.12, float(view_height))
            optical_vector = (
                (control_center[0] - image_width / 2.0) * clearance / fx,
                (control_center[1] - image_height / 2.0) * clearance / fy,
                0.0,
            )
            base_from_camera = self.tf_buffer.lookup_transform(
                "base_footprint", "wrist_camera_optical_frame", Time(),
                timeout=Duration(seconds=0.2))
            base_vector = _rotate_point(
                optical_vector,
                (base_from_camera.transform.rotation.x,
                 base_from_camera.transform.rotation.y,
                 base_from_camera.transform.rotation.z,
                 base_from_camera.transform.rotation.w))
            robot = self.model_poses.get("lab_cobot")
            if robot is None:
                return False
            world_vector = _rotate_point(
                base_vector,
                (robot.orientation.x, robot.orientation.y,
                 robot.orientation.z, robot.orientation.w))
        except Exception as exc:
            self.get_logger().warning(
                "Cannot form wrist pixel-centering correction: %s" % exc,
                throttle_duration_sec=2.0)
            return False
        previous = self.wrist_centering_offsets.get(label, (0.0, 0.0, 0.0))
        pixel_error = math.hypot(
            control_center[0] - image_width / 2.0,
            control_center[1] - image_height / 2.0)
        state = self.wrist_centering_state.get(label)
        if state is not None and pixel_error > state["pixel_error"] + 5.0:
            self.wrist_centering_offsets[label] = state["previous_offset"]
            self.wrist_centering_state.pop(label, None)
            self._status(
                "WRIST_PIXEL_RECENTER_ROLLBACK:%s:error=%.1f:previous=%.1f" %
                (label, pixel_error, state["pixel_error"]))
            return False
        # Bound a single correction and the accumulated adjustment so a false
        # YOLO box cannot command the camera outside its verified workspace.
        gain = float(self.get_parameter("recenter_gain").value)
        step_limit = float(self.get_parameter("recenter_max_step_m").value)
        total_limit = float(self.get_parameter("recenter_max_total_m").value)
        # ``wrist_centering_offsets`` translates the commanded camera/aim pose,
        # not the measured object.  Its image motion is therefore opposite to
        # an object-point displacement in the optical frame.  The previous
        # positive sign consistently increased a 93 px error to 161 px in the
        # second independent view; command the inverse displacement.
        step = tuple(max(-step_limit, min(step_limit, -gain * value))
                     for value in world_vector)
        updated = tuple(max(-total_limit, min(total_limit, previous[index] + step[index]))
                        for index in range(3))
        self.wrist_centering_state[label] = {
            "pixel_error": pixel_error,
            "previous_offset": previous,
        }
        self.wrist_centering_offsets[label] = updated
        self._status(
            "WRIST_PIXEL_RECENTER:%s:step=%.4f,%.4f:total=%.4f,%.4f" % (
                label, step[0], step[1], updated[0], updated[1]))
        return True

    def _advance_wrist_search_offset(self, label, attempt):
        """Apply a bounded generic raster step when the target has no box."""
        pattern = [
            (0.025, 0.0), (0.0, 0.025), (-0.025, 0.0), (0.0, -0.025),
            (0.025, 0.025), (-0.025, 0.025), (-0.025, -0.025),
            (0.025, -0.025),
        ]
        dx_base, dy_base = pattern[int(attempt) % len(pattern)]
        robot = self.model_poses.get("lab_cobot")
        if robot is None:
            return False
        world_step = _rotate_point(
            (dx_base, dy_base, 0.0),
            (robot.orientation.x, robot.orientation.y,
             robot.orientation.z, robot.orientation.w))
        self.wrist_centering_offsets[label] = world_step
        self.wrist_centering_state.pop(label, None)
        self._status(
            "WRIST_RASTER_SEARCH:%s:step=%.4f,%.4f" %
            (label, world_step[0], world_step[1]))
        return True

    def _return_base_after_observation(self, station):
        path = OBSERVATION_RETURN_PATHS.get(station, [])
        if not path:
            return True
        if not self._arm_safe_for_base_motion():
            self._status("BASE_MOTION_REFUSED:arm_not_stable_or_valid")
            self._stop_robot()
            return False
        tolerance = min(0.12, float(self.get_parameter("odom_goal_tolerance_m").value))
        for x, y in path:
            deadline = time.monotonic() + self.odom_timeout_seconds
            if not self._drive_gazebo_point(x, y, tolerance, 0.20, deadline):
                self._stop_robot()
                return False
        return True

    def _refresh_local_planning_scene(self, context):
        """Register live tables and objects in the stopped robot base frame.

        A headless run has no map->base TF, so a PlanningScene expressed in
        ``map`` is rejected.  Gazebo truth is used here only to build collision
        geometry for safe arm planning.  The scene is refreshed whenever the
        chassis is stopped; it never enters the visual pose or error result.
        """
        robot = self.model_poses.get("lab_cobot")
        timeout = float(self.get_parameter("state_validity_timeout_seconds").value)
        if robot is None:
            self.get_logger().error("Cannot refresh PlanningScene: no robot Gazebo pose")
            return False
        if not self.apply_scene_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("MoveIt apply-planning-scene service is unavailable")
            return False

        robot_q = (robot.orientation.x, robot.orientation.y,
                   robot.orientation.z, robot.orientation.w)
        inverse_robot_q = _quat_inverse(robot_q)
        objects = []

        def append_box(object_id, model_pose, size, local_center):
            offset_world = _rotate_point(local_center, (
                model_pose.orientation.x, model_pose.orientation.y,
                model_pose.orientation.z, model_pose.orientation.w))
            center_world = (
                model_pose.position.x + offset_world[0],
                model_pose.position.y + offset_world[1],
                model_pose.position.z + offset_world[2],
            )
            center_base = _inverse_pose_point(center_world, robot)
            model_q = (model_pose.orientation.x, model_pose.orientation.y,
                       model_pose.orientation.z, model_pose.orientation.w)
            base_q = _quat_multiply(inverse_robot_q, model_q)
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [max(0.005, float(value)) for value in size]
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = center_base
            pose.orientation.x, pose.orientation.y = base_q[0], base_q[1]
            pose.orientation.z, pose.orientation.w = base_q[2], base_q[3]
            item = CollisionObject()
            item.header.frame_id = "base_footprint"
            item.id = "image_pkg_%s" % object_id
            item.primitives = [primitive]
            item.primitive_poses = [pose]
            item.operation = CollisionObject.ADD
            objects.append(item)

        for name, size in TABLE_COLLISION_SIZES.items():
            model_pose = self.model_poses.get(name)
            if model_pose is not None:
                append_box(name, model_pose, size, (0.0, 0.0, size[2] / 2.0))
        for model_name, label in COLLISION_MODEL_LABELS.items():
            model_pose = self.model_poses.get(model_name)
            if model_pose is None:
                continue
            xmin, xmax, ymin, ymax, zmin, zmax = OBJECT_LOCAL_BOUNDS[label]
            # A small safety pad accounts for imported mesh/bounds mismatch.
            size = (xmax - xmin + 0.01, ymax - ymin + 0.01,
                    zmax - zmin + 0.005)
            center = ((xmin + xmax) / 2.0, (ymin + ymax) / 2.0,
                      (zmin + zmax) / 2.0)
            append_box(model_name, model_pose, size, center)

        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        request.scene.world.collision_objects = objects
        future = self.apply_scene_client.call_async(request)
        if not _wait_for_future(self, future, timeout):
            self.get_logger().error("PlanningScene refresh timed out: %s" % context)
            return False
        result = future.result()
        if result is None or not result.success:
            self.get_logger().error("PlanningScene refresh rejected: %s" % context)
            return False
        self._status("PLANNING_SCENE_REFRESHED:%s:objects=%d" % (context, len(objects)))
        return True

    def _stow_arm_before_base_motion(self, context):
        """Retract the UR arm and only then permit any chassis translation."""
        timeout = float(self.get_parameter("arm_stow_timeout_seconds").value)
        if not self._refresh_local_planning_scene("stow:%s" % context):
            self._status(f"ARM_STOW_FAILED:{context}:planning_scene")
            self._stop_robot()
            return False
        if not self._state_is_valid(SAFE_TRANSPORT_CONFIG, "transport_goal"):
            self._status(f"ARM_STOW_FAILED:{context}:invalid_transport_goal")
            self._stop_robot()
            return False
        for attempt in range(1, 4):
            self._status(f"ARM_STOWING:{context}:attempt={attempt}")
            self.moveit.move_to_configuration(SAFE_TRANSPORT_CONFIG)
            if self.moveit.wait_until_executed(timeout_sec=timeout):
                stationary = self._wait_for_arm_stationary()
                if not stationary:
                    self._status(f"ARM_STOW_REJECTED:{context}:joint_velocity")
                    self._stop_robot()
                    return False
                state_valid = self._state_is_valid(None, "transport_result")
                if stationary and state_valid:
                    self._status(f"ARM_STOWED:{context}")
                    return True
                self._status(f"ARM_STOW_REJECTED:{context}:moveit_state_invalid")
                self._stop_robot()
                return False
            time.sleep(0.3)
        # Never publish a raw controller trajectory after MoveIt rejects the
        # current state.  That old recovery path bypassed collision checking
        # and was the source of upper-arm/gripper/wheel contacts.
        self._status(f"ARM_STOW_FAILED:{context}:moveit_only")
        self._stop_robot()
        return False

    def _wait_for_arm_stationary(self, timeout_seconds=3.0):
        """Require continuously low measured joint velocity before evaluation."""
        deadline = time.monotonic() + float(timeout_seconds)
        stable_since = None
        speed_limit = float(self.get_parameter("max_arm_joint_speed_rps").value)
        settle = float(self.get_parameter("arm_settle_seconds").value)
        last_maximum = float("inf")
        while rclpy.ok() and time.monotonic() < deadline:
            if all(name in self.current_arm_velocities for name in UR_JOINTS):
                maximum = max(abs(self.current_arm_velocities[name]) for name in UR_JOINTS)
                last_maximum = maximum
                if math.isfinite(maximum) and maximum <= speed_limit:
                    stable_since = stable_since or time.monotonic()
                    if time.monotonic() - stable_since >= settle:
                        return True
                else:
                    stable_since = None
            time.sleep(0.05)
        self.get_logger().error(
            "Arm did not settle: max_velocity=%.4f rad/s, limit=%.4f rad/s" %
            (last_maximum, speed_limit))
        return False

    def _state_is_valid(self, target_positions=None, context="current"):
        """Ask MoveIt whether a current or proposed arm state is collision-free."""
        timeout = float(self.get_parameter("state_validity_timeout_seconds").value)
        if not self.state_validity_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("MoveIt state-validity service is unavailable")
            return False
        request = GetStateValidity.Request()
        request.group_name = "ur_manipulator"
        request.robot_state = RobotState()
        request.robot_state.is_diff = True
        request.robot_state.joint_state = JointState()
        if target_positions is None:
            if self.latest_joint_state is None:
                self.get_logger().error(
                    "Cannot validate %s: no /joint_states sample" % context)
                return False
            request.robot_state.joint_state.header = self.latest_joint_state.header
            request.robot_state.joint_state.name = list(self.latest_joint_state.name)
            request.robot_state.joint_state.position = list(self.latest_joint_state.position)
        else:
            request.robot_state.joint_state.name = list(UR_JOINTS)
            request.robot_state.joint_state.position = [float(v) for v in target_positions]
        future = self.state_validity_client.call_async(request)
        if not _wait_for_future(self, future, timeout):
            self.get_logger().error("MoveIt state-validity request timed out: %s" % context)
            return False
        result = future.result()
        valid = result is not None and bool(result.valid)
        if not valid:
            contacts = [] if result is None else [
                "%s/%s" % (item.contact_body_1, item.contact_body_2)
                for item in result.contacts[:4]
            ]
            self.get_logger().error(
                "MoveIt rejected %s state; contacts=%s" %
                (context, ",".join(contacts) if contacts else "unknown"))
        return valid

    def _arm_safe_for_base_motion(self):
        return self._wait_for_arm_stationary() and self._state_is_valid(
            None, "before_base_motion")

    def _raise_arm_for_coarse_observation(self, context):
        """Use a collision-safe joint pose for the first station overview."""
        if not self._refresh_local_planning_scene("coarse:%s" % context):
            self._status(f"COARSE_ARM_RAISE_FAILED:{context}:planning_scene")
            return False
        if not self._state_is_valid(COARSE_RAISED_CONFIG, "coarse_goal"):
            self._status(f"COARSE_ARM_RAISE_FAILED:{context}:invalid_goal")
            return False
        self._status(f"COARSE_ARM_RAISING:{context}")
        self.moveit.move_to_configuration(COARSE_RAISED_CONFIG)
        timeout = float(self.get_parameter("observation_motion_timeout_seconds").value)
        if self.moveit.wait_until_executed(timeout_sec=timeout):
            if (self._wait_for_arm_stationary()
                    and self._state_is_valid(None, "coarse_result")):
                self._status(f"COARSE_ARM_RAISED:{context}")
                return True
        self._status(f"COARSE_ARM_RAISE_FAILED:{context}")
        return False

    def _aim_wrist_at_truth(
            self, label, view_heights=None, verify_above=True,
            azimuth_offset=0.0):
        """Put the wrist camera above one object with a MoveIt observation pose.

        Gazebo truth is used *only* to choose a reachable observation pose;
        it is not injected into the detector or error metric.  The final
        detection confirmation below still comes solely from wrist RGB-D/YOLO.
        """
        object_pose = self.model_poses.get(LABEL_TO_MODEL[label])
        robot_pose = self.model_poses.get("lab_cobot")
        target_world = self._observation_target_position(label)
        if object_pose is None or robot_pose is None or target_world is None:
            self.get_logger().warning(f"No Gazebo truth pose for observation target {label}")
            return False
        if not self._refresh_local_planning_scene("fine:%s" % label):
            return False
        yaw = _yaw_from_quaternion(robot_pose.orientation)
        dx = target_world[0] - robot_pose.position.x
        dy = target_world[1] - robot_pose.position.y
        # World -> mobile-base planar coordinates. +X is forward.
        forward = math.cos(yaw) * dx + math.sin(yaw) * dy
        lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
        distance = math.hypot(forward, lateral)
        if distance > 1.75:
            self.get_logger().warning(
                "Observation target %s is %.3fm from arm base; navigation approach is invalid"
                % (label, distance))
            return False

        # The wrist optical frame is not parallel to gripper_tcp.  Specify the
        # desired optical centre and +Z viewing direction, then compose the
        # live TCP<-optical transform in _tool_target_from_optical_target.
        object_in_footprint = [
            forward, lateral, object_pose.position.z - robot_pose.position.z,
        ]
        bounds = OBJECT_LOCAL_BOUNDS[label]
        size_x = bounds[1] - bounds[0]
        size_y = bounds[3] - bounds[2]
        object_yaw = _yaw_from_quaternion(object_pose.orientation)
        optical_x_yaw = object_yaw + (math.pi / 2.0 if size_y > size_x else 0.0)
        # Express the object's long horizontal axis in the robot base.  It is
        # used as an image-X hint so elongated tools fit the 640-pixel width.
        long_world = (math.cos(optical_x_yaw), math.sin(optical_x_yaw), 0.0)
        long_base = _rotate_point(long_world, _quat_inverse((
            robot_pose.orientation.x, robot_pose.orientation.y,
            robot_pose.orientation.z, robot_pose.orientation.w)))
        timeout = float(self.get_parameter("observation_motion_timeout_seconds").value)
        heights = (list(view_heights) if view_heights is not None
                   else self._candidate_view_heights(label))
        for view_height in heights:
            height = float(view_height)
            if bool(self.get_parameter("use_oblique_fine_views").value):
                # A camera directly above a table-centre target makes the UR
                # reach the full base-to-object distance plus the inspection
                # height.  Place it between the base and target instead and
                # look obliquely at the model origin.  The chassis remains
                # outside the table while the wrist stays near 0.58 m planar
                # reach, where top-down IK previously failed at 0.85--0.90 m.
                bearing = math.atan2(lateral, forward) + float(azimuth_offset)
                unit_forward = math.cos(bearing)
                unit_lateral = math.sin(bearing)
                desired_planar_reach = min(
                    float(self.get_parameter(
                        "oblique_camera_planar_reach_m").value),
                    max(0.36, distance - 0.10),
                )
                optical_target_in_footprint = [
                    unit_forward * desired_planar_reach,
                    unit_lateral * desired_planar_reach,
                    target_world[2] - robot_pose.position.z + height,
                ]
                view_vector = tuple(
                    object_in_footprint[index]
                    - optical_target_in_footprint[index]
                    for index in range(3))
                optical_quaternion = _look_at_optical_quaternion(
                    view_vector, long_base)
            else:
                optical_target_in_footprint = [
                    forward, lateral,
                    target_world[2] - robot_pose.position.z + height,
                ]
                optical_quaternion = _quat_multiply(
                    _quaternion_from_yaw(optical_x_yaw - yaw),
                    (1.0, 0.0, 0.0, 0.0))
            target = self._tool_target_from_optical_target(
                optical_target_in_footprint, optical_quaternion)
            if target is None:
                return False
            position, tool_quaternion = target
            self.get_logger().info(
                "OBSERVATION_TARGET:%s ur_tool0=(%.3f,%.3f,%.3f) range=%.3fm azimuth=%+.3f"
                % (label, position[0], position[1], position[2], distance,
                   float(azimuth_offset)))
            self.moveit.move_to_pose(
                position=position,
                quat_xyzw=tool_quaternion,
                frame_id="ur_base_link",
                # The SRDF arm chain ends at ur_tool0.  Asking its KDL IK
                # solver for gripper_tcp (outside that chain) made every
                # otherwise reachable goal unsampleable.  Convert the desired
                # optical/TCP pose to ur_tool0 below, then solve for the chain
                # tip that MoveIt actually owns.
                target_link="ur_tool0",
                tolerance_position=0.015,
                tolerance_orientation=0.08,
                cartesian=False,
            )
            if self.moveit.wait_until_executed(timeout_sec=timeout):
                # Allow image/depth and TF to become a same-pose frame pair.
                if not self._wait_for_arm_stationary():
                    self.get_logger().warning(
                        "Wrist did not settle for %s at clearance %.2fm" %
                        (label, height))
                    continue
                if not self._state_is_valid(None, "fine_result:%s" % label):
                    return False
                if not verify_above or self._camera_is_above_truth(label):
                    return True
                self.get_logger().warning(
                    "Executed wrist pose for %s but the geometric range/centre gate rejected it"
                    % label)
                continue
            self.get_logger().warning(
                "MoveIt observation pose rejected for %s at clearance %.2fm"
                % (label, height))
        return False

    def _candidate_view_heights(self, label):
        """Choose camera clearance from object extent and live intrinsics."""
        bounds = OBJECT_LOCAL_BOUNDS[label]
        size_x, size_y = bounds[1] - bounds[0], bounds[3] - bounds[2]
        long_side, short_side = max(size_x, size_y), min(size_x, size_y)
        required = 0.0
        info = self.wrist_camera_info
        if info is not None:
            fraction = float(self.get_parameter("target_extent_fraction").value)
            # The long side is deliberately aligned to image width in
            # _aim_wrist_at_truth.  Pinhole projection gives a conservative
            # optical distance that leaves a surrounding background margin.
            required = max(
                float(info.k[0]) * long_side / max(1.0, float(info.width) * fraction),
                float(info.k[4]) * short_side / max(1.0, float(info.height) * fraction),
            )
        # view_height is measured from model origin, while required is a
        # distance above the highest visible surface.
        dynamic = required + max(0.0, bounds[5]) + 0.02
        dynamic = max(0.22, min(
            dynamic,
            float(self.get_parameter("max_dynamic_camera_height_m").value)))
        candidates = [dynamic, *[float(value) for value in
                                 self.get_parameter("camera_view_heights_m").value]]
        result = []
        for value in candidates:
            if value >= 0.15 and all(abs(value - old) > 0.015 for old in result):
                result.append(value)
        return result

    def _camera_is_above_truth(self, label):
        """Verify the live camera centre is vertically above the target in world.

        Gazebo truth selects the observation pose only.  This check does not
        alter or publish the visual estimate; it prevents a failed/stale arm
        motion from opening the benchmark window.
        """
        target = self.model_poses.get(LABEL_TO_MODEL[label])
        robot = self.model_poses.get("lab_cobot")
        if target is None or robot is None:
            return False
        try:
            base_from_camera = self.tf_buffer.lookup_transform(
                "base_footprint", "wrist_camera_optical_frame", Time(),
                timeout=Duration(seconds=0.5))
        except Exception as exc:
            self.get_logger().warning(
                f"Cannot verify wrist camera world position: {exc}",
                throttle_duration_sec=2.0)
            return False
        camera_base = _transform_point((0.0, 0.0, 0.0), base_from_camera)
        robot_q = (
            robot.orientation.x, robot.orientation.y,
            robot.orientation.z, robot.orientation.w)
        camera_offset_world = _rotate_point(camera_base, robot_q)
        camera_world = (
            robot.position.x + camera_offset_world[0],
            robot.position.y + camera_offset_world[1],
            robot.position.z + camera_offset_world[2],
        )
        dx = camera_world[0] - target.position.x
        dy = camera_world[1] - target.position.y
        dz = camera_world[2] - target.position.z
        xy_error = math.hypot(dx, dy)
        height = dz
        view_distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        self._status(
            "CAMERA_ABOVE:%s:xy_error=%.4f:height=%.4f:view_distance=%.4f" %
            (label, xy_error, height, view_distance))
        # Oblique inspection deliberately has a non-zero XY separation.  The
        # projected bounds and measured YOLO box are the visibility/centring
        # authority; this geometric gate only rejects a stale or remote pose.
        position_ok = (
            height > 0.05
            and float(self.get_parameter(
                "fine_camera_min_view_distance_m").value) <= view_distance
            <= float(
                self.get_parameter("fine_camera_max_view_distance_m").value)
        )
        projection = self._project_truth_bounds(label)
        if projection is None:
            return position_ok
        self._status(
            "TRUTH_PROJECTED:%s:center_error_px=%.1f:center_inside=%s:fully_inside=%s" %
            (label, projection["center_error_pixels"],
             projection["center_inside_image"], projection["inside_image"]))
        # Gazebo bounds are approximate and some imported meshes include
        # invisible collision padding.  Requiring every projected truth corner
        # to remain inside the image rejected a real, centred YOLO target.  The
        # truth projection now only guards against aiming at a remote/off-frame
        # model.  Synchronized YOLO evidence below remains the authority for
        # actual visibility, edge clipping and centring.
        return position_ok and projection["center_inside_image"]

    def _project_truth_bounds(self, label):
        """Project the known model bounds to validate an observation view.

        Gazebo truth is used only for camera motion and view acceptance.  The
        returned pixels never enter the YOLO or RGB-D pose result.
        """
        info = self.wrist_camera_info
        target = self.model_poses.get(LABEL_TO_MODEL[label])
        robot = self.model_poses.get("lab_cobot")
        if info is None or target is None or robot is None:
            return None
        try:
            camera_from_base = self.tf_buffer.lookup_transform(
                "wrist_camera_optical_frame", "base_footprint", Time(),
                timeout=Duration(seconds=0.5))
        except Exception as exc:
            self.get_logger().warning(
                f"Cannot project truth into wrist image: {exc}",
                throttle_duration_sec=2.0)
            return None
        xmin, xmax, ymin, ymax, zmin, zmax = OBJECT_LOCAL_BOUNDS[label]
        object_q = (target.orientation.x, target.orientation.y,
                    target.orientation.z, target.orientation.w)
        pixels = []
        for x in (xmin, xmax):
            for y in (ymin, ymax):
                for z in (zmin, zmax):
                    offset = _rotate_point((x, y, z), object_q)
                    world_point = (
                        target.position.x + offset[0],
                        target.position.y + offset[1],
                        target.position.z + offset[2],
                    )
                    base_point = _inverse_pose_point(world_point, robot)
                    camera_point = _transform_point(base_point, camera_from_base)
                    if camera_point[2] <= 0.02:
                        return None
                    pixels.append((
                        float(info.k[0]) * camera_point[0] / camera_point[2] + float(info.k[2]),
                        float(info.k[4]) * camera_point[1] / camera_point[2] + float(info.k[5]),
                    ))
        left, right = min(p[0] for p in pixels), max(p[0] for p in pixels)
        top, bottom = min(p[1] for p in pixels), max(p[1] for p in pixels)
        center_x, center_y = (left + right) / 2.0, (top + bottom) / 2.0
        image_center_x, image_center_y = info.width / 2.0, info.height / 2.0
        center_error = math.hypot(center_x - image_center_x, center_y - image_center_y)
        margin = float(self.get_parameter("target_edge_margin_fraction").value)
        margin_x, margin_y = info.width * margin, info.height * margin
        area_fraction = max(0.0, right - left) * max(0.0, bottom - top) / float(
            info.width * info.height)
        inside_image = (
            left >= margin_x and right <= info.width - margin_x
            and top >= margin_y and bottom <= info.height - margin_y
            and area_fraction <= float(
                self.get_parameter("max_target_area_fraction").value)
        )
        center_inside_image = (
            margin_x <= center_x <= info.width - margin_x
            and margin_y <= center_y <= info.height - margin_y
        )
        centered = center_error <= float(
            self.get_parameter("target_center_tolerance_pixels").value)
        return {
            "projected_box_pixels": [left, top, right, bottom],
            "projected_center_pixels": [center_x, center_y],
            "center_error_pixels": center_error,
            "projected_area_fraction": area_fraction,
            "inside_image": inside_image,
            "center_inside_image": center_inside_image,
            "centered": centered,
            # Kept for manifest compatibility with earlier evidence files.
            "fully_visible": inside_image and centered,
        }

    def _tool_target_from_optical_target(self, optical_target_in_footprint, optical_quaternion):
        """Convert a desired camera-centre point into a valid UR-tool IK goal.

        The previous implementation sent a mobile-base coordinate in a
        nonexistent ``base_link`` frame.  Compose the *live* transforms
        instead, so mounting changes and the 0.375 m UR-base height are not
        hidden in a hard-coded Z offset.
        """
        try:
            ur_from_footprint = self.tf_buffer.lookup_transform(
                "ur_base_link", "base_footprint", Time(),
                timeout=Duration(seconds=0.2))
            tcp_from_optical = self.tf_buffer.lookup_transform(
                "gripper_tcp", "wrist_camera_optical_frame", Time(),
                timeout=Duration(seconds=0.2))
            tool_from_tcp = self.tf_buffer.lookup_transform(
                "ur_tool0", "gripper_tcp", Time(),
                timeout=Duration(seconds=0.2))
        except Exception as exc:
            self.get_logger().warning(
                f"Cannot form wrist observation TF chain: {exc}",
                throttle_duration_sec=2.0)
            return None
        optical_in_ur = _transform_point(optical_target_in_footprint, ur_from_footprint)
        camera_offset_in_tcp = tcp_from_optical.transform.translation
        optical_q_in_tcp = tcp_from_optical.transform.rotation
        # T_ur_optical is specified by the observation policy.  Recover
        # T_ur_tcp = T_ur_optical * inverse(T_tcp_optical), instead of
        # incorrectly treating the two frames as parallel.
        tcp_quaternion = _quat_multiply(
            optical_quaternion, _quat_inverse((
                optical_q_in_tcp.x, optical_q_in_tcp.y,
                optical_q_in_tcp.z, optical_q_in_tcp.w)))
        offset_in_ur = _rotate_point(
            (camera_offset_in_tcp.x, camera_offset_in_tcp.y, camera_offset_in_tcp.z),
            tcp_quaternion)
        tcp_in_ur = [
            optical_in_ur[index] - offset_in_ur[index]
            for index in range(3)
        ]
        tcp_in_tool = tool_from_tcp.transform.translation
        tcp_q_in_tool = tool_from_tcp.transform.rotation
        tool_quaternion = _quat_multiply(
            tcp_quaternion, _quat_inverse((
                tcp_q_in_tool.x, tcp_q_in_tool.y,
                tcp_q_in_tool.z, tcp_q_in_tool.w)))
        tool_offset_in_ur = _rotate_point(
            (tcp_in_tool.x, tcp_in_tool.y, tcp_in_tool.z), tool_quaternion)
        tool_in_ur = [
            tcp_in_ur[index] - tool_offset_in_ur[index]
            for index in range(3)
        ]
        return tool_in_ur, list(tool_quaternion)

    def _capture_observation_view(self, station, label, stage):
        """Save one wrist RGB frame and describe its target-box visibility.

        These images are deliberately saved without invented YOLO labels.
        They are candidates for manual annotation using the actual wrist
        viewpoint; a detector's missing or wrong box must not become a false
        training label.
        """
        directory_text = str(self.get_parameter("observation_capture_dir").value).strip()
        image_msg = self.latest_wrist_image
        if image_msg is None:
            return {"saved": False, "centered": False, "reason": "no_image"}
        try:
            image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warning(f"Cannot save wrist evidence frame: {exc}")
            return {"saved": False, "centered": False, "reason": "image_conversion_failed"}
        stamp = image_msg.header.stamp.sec + image_msg.header.stamp.nanosec * 1e-9
        image_path = None
        saved = False
        directory = None
        if directory_text:
            directory = Path(directory_text).expanduser()
            images_dir = directory / "images"
            images_dir.mkdir(parents=True, exist_ok=True)
            self.capture_index += 1
            filename = "%03d_%s_%s_%s_%.3f.png" % (
                self.capture_index, station, label, stage, stamp)
            image_path = images_dir / filename
            if not cv2.imwrite(str(image_path), image):
                return {"saved": False, "centered": False,
                        "reason": "image_write_failed"}
            saved = True

        # The detector runs on CPU and normally finishes after the arm has
        # settled.  Wait for a result whose source frame is no older than this
        # settled wrist image; accepting merely "close" timestamps can pair a
        # stationary capture with the final frame from arm motion.
        fresh_deadline = time.monotonic() + float(
            self.get_parameter("fresh_detection_wait_seconds").value)
        while rclpy.ok() and time.monotonic() < fresh_deadline:
            candidate = self.latest_detection_payload or {}
            try:
                candidate_stamp = float(candidate.get("timestamp", float("nan")))
            except (TypeError, ValueError):
                candidate_stamp = float("nan")
            if math.isfinite(candidate_stamp) and candidate_stamp >= stamp - 0.05:
                break
            time.sleep(0.05)
        payload = self.latest_detection_payload or {}
        try:
            detection_stamp = float(payload.get("timestamp", float("nan")))
        except (TypeError, ValueError):
            detection_stamp = float("nan")
        # Do not assign a delayed YOLO result to another wrist frame.
        synchronized = (
            math.isfinite(detection_stamp)
            and abs(detection_stamp - stamp) <= float(
                self.get_parameter("detection_sync_tolerance_seconds").value)
        )
        detections = payload.get("detections", []) if synchronized else []
        matching = [item for item in detections
                    if isinstance(item, dict) and str(item.get("label", "")) == label]
        box = max(matching, key=lambda item: float(item.get("confidence", 0.0)), default=None)
        height, width = image.shape[:2]
        details = {
            "image": str(image_path) if image_path is not None else None,
            "station": station, "label": label,
            "stage": stage, "image_stamp_sec": stamp,
            "image_size": [int(width), int(height)],
            "detection_synchronized": synchronized,
            "target_boxes": matching,
            "visible_by_yolo": box is not None,
            "manual_occlusion_review": "required",
        }
        truth_projection = self._project_truth_bounds(label)
        if truth_projection is not None:
            details["truth_projection"] = truth_projection
        if box is not None:
            x1, y1, x2, y2 = [int(value) for value in box.get("box", [0, 0, 0, 0])]
            x1, x2 = max(0, min(width, x1)), max(0, min(width, x2))
            y1, y2 = max(0, min(height, y1)), max(0, min(height, y2))
            box_width, box_height = max(0, x2 - x1), max(0, y2 - y1)
            center_x, center_y = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            box_area_fraction = (box_width * box_height) / float(width * height)
            edge_margin = float(self.get_parameter(
                "target_edge_margin_fraction").value)
            box_inside = (
                x1 >= width * edge_margin
                and y1 >= height * edge_margin
                and x2 <= width * (1.0 - edge_margin)
                and y2 <= height * (1.0 - edge_margin)
                and box_area_fraction <= float(self.get_parameter(
                    "max_target_area_fraction").value)
            )
            yolo_center_error = math.hypot(
                center_x - width / 2.0, center_y - height / 2.0)
            centered_by_yolo = (
                yolo_center_error <= float(self.get_parameter(
                    "target_center_tolerance_pixels").value)
                and box_inside
            )
            centered_by_truth_projection = bool(
                self.get_parameter(
                    "allow_truth_centered_view_acceptance").value
                and truth_projection is not None
                and truth_projection.get("centered", False)
                and truth_projection.get("inside_image", False)
            )
            centered_for_statistics = bool(
                box_inside
                and (centered_by_yolo or centered_by_truth_projection)
            )
            details.update({
                "box_pixels": [x1, y1, x2, y2],
                "box_size_pixels": [box_width, box_height],
                "box_area_fraction": box_area_fraction,
                "best_confidence": float(box.get("confidence", 0.0)),
                "box_touches_image_edge": x1 <= 2 or y1 <= 2 or x2 >= width - 2 or y2 >= height - 2,
                "yolo_center_error_pixels": yolo_center_error,
                "box_fully_inside": box_inside,
                "centered_by_yolo": centered_by_yolo,
                "centered_by_truth_projection": centered_by_truth_projection,
                "centered_for_statistics": centered_for_statistics,
                "centering_basis": (
                    "yolo_box" if centered_by_yolo
                    else "projected_model_origin"
                    if centered_by_truth_projection and box_inside
                    else "none"),
            })
        else:
            details["centered_by_yolo"] = False
            details["centered_by_truth_projection"] = False
            details["centered_for_statistics"] = False
            details["centering_basis"] = "none"
        if directory is not None:
            manifest = directory / "manifest.jsonl"
            with manifest.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(details, ensure_ascii=False) + "\n")
        self.get_logger().info(
            "OBSERVATION_FRAME:%s visible=%s centered=%s basis=%s saved=%s path=%s" % (
                label, details["visible_by_yolo"],
                details["centered_for_statistics"], details["centering_basis"],
                saved, image_path))
        return {"saved": saved, "centered": details["centered_for_statistics"],
                "details": details}

    def _acquire_centered_fine_view(
            self, station, label, view_index=0, azimuth_offset=0.0):
        """Search safe heights until geometry and the actual YOLO box agree."""
        last_view = {"saved": False, "centered": False,
                     "reason": "no_reachable_observation_pose"}
        aimed_once = False
        for attempt, height in enumerate(self._candidate_view_heights(label)):
            # Apply a measured pixel correction at the *same* height once.
            # The former loop immediately changed height after calculating an
            # XY correction; this confounded two variables and could move an
            # already detected thin tool out of the image.
            for refinement in range(2):
                self._status(
                    "FINE_VIEW_ATTEMPT:%s:%s:view=%d:azimuth=%+.3f:height=%.3f:refine=%d" %
                    (station, label, view_index, azimuth_offset,
                     height, refinement))
                if not self._aim_wrist_at_truth(
                        label, [height], azimuth_offset=azimuth_offset):
                    break
                aimed_once = True
                time.sleep(float(
                    self.get_parameter(
                        "observation_capture_settle_seconds").value))
                view = self._capture_observation_view(
                    station, label, "wrist_aimed_v%d_a%+.2f_%.2fm_r%d" %
                    (view_index, azimuth_offset, height, refinement))
                if view.get("saved"):
                    last_view = view
                if view.get("centered", False):
                    self._status(
                        "FINE_VIEW_CONFIRMED:%s:%s:view=%d:height=%.3f" %
                        (station, label, view_index, height))
                    return True, view
                details = (view.get("details", {})
                           if isinstance(view, dict) else {})
                adjusted = self._update_wrist_centering_offset(
                    label, height, details)
                if not adjusted and not details.get("box_pixels"):
                    adjusted = self._advance_wrist_search_offset(
                        label, attempt)
                self._status(
                    "FINE_VIEW_RETRY:%s:%s:view=%d:height=%.3f:adjusted=%s" %
                    (station, label, view_index, height, adjusted))
                if not adjusted:
                    break
        return aimed_once, last_view

    def _observe_station(self, station):
        expected = set(STATION_LABELS.get(station, set()))
        requested = {
            str(value).strip() for value in
            self.get_parameter("target_labels").value if str(value).strip()
        }
        if requested:
            expected &= requested
        observed = set()
        coarse_label = STATION_COARSE_LABEL.get(station)
        if coarse_label in expected:
            self.detected_labels.clear()
            self.detection_counts.clear()
            self._counted_detection_timestamps.clear()
            self._status(f"COARSE_POSITIONING:{station}:{coarse_label}")
            if (self._stow_arm_before_base_motion(
                    f"{station}:coarse:{coarse_label}")
                    and self._move_base_for_observation(coarse_label)):
                if self._raise_arm_for_coarse_observation(
                        f"{station}:{coarse_label}"):
                    self._status(f"COARSE_OBSERVING:{station}:{coarse_label}")
                    time.sleep(float(
                        self.get_parameter("coarse_observe_seconds").value))
                    self._capture_observation_view(
                        station, coarse_label, "coarse_raised")
                    coarse_detected = sorted(expected & self.detected_labels)
                    self._status(
                        "COARSE_RESULT:%s:detected=%s" %
                        (station, ",".join(coarse_detected) if coarse_detected else "none"))
                else:
                    self._status(f"COARSE_ARM_AIM_FAILED:{station}:{coarse_label}")
        for label in sorted(expected):
            self.detected_labels.clear()
            self.detection_counts.clear()
            self._counted_detection_timestamps.clear()
            self.pipeline_counts.pop(label, None)
            self._counted_pipeline_events = {
                event for event in self._counted_pipeline_events
                if event[0] != label}
            if not self._stow_arm_before_base_motion(f"{station}:fine:{label}"):
                self._status(f"OBSERVATION_STOW_FAILED:{station}:{label}")
                continue
            if not self._move_base_for_observation(label):
                continue
            # Capture the actual wrist view at the base observation pose even
            # if MoveIt later rejects its IK target.  This is the evidence
            # needed to repair training data and camera orientation.
            time.sleep(float(self.get_parameter("observation_capture_settle_seconds").value))
            self._capture_observation_view(station, label, "base_view")
            confirmed_views = 0
            view_offsets = [float(value) for value in self.get_parameter(
                "fine_view_azimuth_offsets_rad").value]
            minimum_views = max(1, int(self.get_parameter(
                "minimum_confirmed_views").value))
            for view_index, azimuth_offset in enumerate(view_offsets):
                # Each view starts from the geometric target.  A pixel
                # correction measured at another azimuth is not transferable.
                self.wrist_centering_offsets.pop(label, None)
                self.wrist_centering_state.pop(label, None)
                aimed, view = self._acquire_centered_fine_view(
                    station, label, view_index=view_index,
                    azimuth_offset=azimuth_offset)
                if not aimed:
                    self._status(
                        f"OBSERVATION_ARM_AIM_FAILED:{station}:{label}:view={view_index}")
                    continue
                if (bool(self.get_parameter(
                        "require_centered_yolo_before_statistics").value)
                        and not view.get("centered", False)):
                    self._status(
                        f"OBSERVATION_NOT_CENTERED:{station}:{label}:view={view_index}")
                    continue
                # Only detections whose source image was acquired after this
                # exact settled pose may satisfy the stable-frame condition.
                self.detection_counts.clear()
                self._counted_detection_timestamps.clear()
                self._status(
                    "OBSERVING_FINE:%s:%s:%s:view=%d:azimuth=%+.3f" % (
                        station, label, LABEL_TO_MODEL.get(label, ""),
                        view_index, azimuth_offset))
                deadline = time.monotonic() + max(
                    self.dwell_seconds, float(self.get_parameter(
                        "per_object_observe_seconds").value))
                view_confirmed = False
                while rclpy.ok() and time.monotonic() < deadline:
                    time.sleep(0.05)
                    if self.detection_counts.get(label, 0) >= int(
                            self.get_parameter(
                                "min_stable_detection_frames").value):
                        # A stable 2-D box is necessary but not sufficient:
                        # wait until the temporal RGB-D filter has actually
                        # produced a support-referenced pose for this view.
                        pose_deadline = time.monotonic() + max(
                            0.0, float(self.get_parameter(
                                "post_view_rgbd_settle_seconds").value))
                        while (rclpy.ok()
                               and time.monotonic() < pose_deadline
                               and self.pipeline_counts.get(label, {}).get(
                                   "pose_ready", 0) < 1):
                            time.sleep(0.05)
                        counts = self.pipeline_counts.get(label, {})
                        pose_ready = counts.get("pose_ready", 0) >= 1
                        self._status(
                            "PIPELINE_COUNTS:%s:%s:view=%d:yolo_box=%d:valid_depth=%d:world_transform=%d:pose_ready=%d:station_accepted=%d" % (
                                station, label, view_index,
                                counts.get("yolo_box", 0),
                                counts.get("valid_depth", 0),
                                counts.get("world_transform", 0),
                                counts.get("pose_ready", 0),
                                1 if pose_ready else 0))
                        if pose_ready:
                            view_confirmed = True
                            confirmed_views += 1
                        break
                if not view_confirmed:
                    self._status(
                        f"FINE_VIEW_NO_DETECTION:{station}:{label}:view={view_index}")
            if confirmed_views >= minimum_views:
                observed.add(label)
                self._status(
                    f"MULTIVIEW_CONFIRMED:{station}:{label}:views={confirmed_views}")
            else:
                self._status(
                    f"MULTIVIEW_INSUFFICIENT:{station}:{label}:views={confirmed_views}:required={minimum_views}")
        if not self._stow_arm_before_base_motion(f"{station}:departure"):
            self._status(f"FAILED:observation_stow:{station}")
            return
        if not self._return_base_after_observation(station):
            self._status(f"FAILED:observation_return:{station}")
            return
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


def _quaternion_from_yaw(yaw):
    return (0.0, 0.0, math.sin(float(yaw) / 2.0), math.cos(float(yaw) / 2.0))


def _transform_point(point, transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    rotated = _rotate_point(point, (rotation.x, rotation.y, rotation.z, rotation.w))
    return (
        rotated[0] + translation.x,
        rotated[1] + translation.y,
        rotated[2] + translation.z,
    )


def _rotate_point(point, quaternion):
    """Rotate a point by an xyzw quaternion without a new math dependency."""
    x, y, z, w = (float(value) for value in quaternion)
    px, py, pz = (float(value) for value in point)
    # q * p * q^-1, expanded for a vector p.
    ix = w * px + y * pz - z * py
    iy = w * py + z * px - x * pz
    iz = w * pz + x * py - y * px
    iw = -x * px - y * py - z * pz
    return (
        ix * w + iw * -x + iy * -z - iz * -y,
        iy * w + iw * -y + iz * -x - ix * -z,
        iz * w + iw * -z + ix * -y - iy * -x,
    )


def _inverse_pose_point(point, pose):
    """Express a world point in the local coordinates of a Gazebo pose."""
    relative = (
        float(point[0]) - float(pose.position.x),
        float(point[1]) - float(pose.position.y),
        float(point[2]) - float(pose.position.z),
    )
    q = (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w)
    return _rotate_point(relative, _quat_inverse(q))


def _quat_inverse(quaternion):
    x, y, z, w = (float(value) for value in quaternion)
    norm_sq = x * x + y * y + z * z + w * w
    if norm_sq <= 1e-12:
        raise ValueError("cannot invert a zero quaternion")
    return (-x / norm_sq, -y / norm_sq, -z / norm_sq, w / norm_sq)


def _quat_multiply(left, right):
    """Return the xyzw Hamilton product ``left * right``."""
    lx, ly, lz, lw = (float(value) for value in left)
    rx, ry, rz, rw = (float(value) for value in right)
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _look_at_optical_quaternion(view_vector, image_x_hint):
    """Return parent<-optical quaternion with optical +Z at the target."""
    z_axis = _normalize_vector(view_vector)
    hint = _normalize_vector(image_x_hint)
    projection = sum(hint[i] * z_axis[i] for i in range(3))
    x_candidate = tuple(
        hint[i] - projection * z_axis[i] for i in range(3))
    if math.sqrt(sum(value * value for value in x_candidate)) < 1e-6:
        fallback = (1.0, 0.0, 0.0)
        if abs(z_axis[0]) > 0.9:
            fallback = (0.0, 1.0, 0.0)
        projection = sum(fallback[i] * z_axis[i] for i in range(3))
        x_candidate = tuple(
            fallback[i] - projection * z_axis[i] for i in range(3))
    x_axis = _normalize_vector(x_candidate)
    y_axis = _normalize_vector((
        z_axis[1] * x_axis[2] - z_axis[2] * x_axis[1],
        z_axis[2] * x_axis[0] - z_axis[0] * x_axis[2],
        z_axis[0] * x_axis[1] - z_axis[1] * x_axis[0],
    ))
    # Rotation matrix columns are the optical axes expressed in the parent.
    matrix = (
        (x_axis[0], y_axis[0], z_axis[0]),
        (x_axis[1], y_axis[1], z_axis[1]),
        (x_axis[2], y_axis[2], z_axis[2]),
    )
    return _quaternion_from_rotation_matrix(matrix)


def _normalize_vector(vector):
    values = tuple(float(value) for value in vector)
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-9:
        raise ValueError("cannot normalize a zero vector")
    return tuple(value / norm for value in values)


def _quaternion_from_rotation_matrix(matrix):
    """Convert a right-handed 3x3 rotation matrix to normalized xyzw."""
    m00, m01, m02 = matrix[0]
    m10, m11, m12 = matrix[1]
    m20, m21, m22 = matrix[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = (
            (m21 - m12) / scale,
            (m02 - m20) / scale,
            (m10 - m01) / scale,
            0.25 * scale,
        )
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(max(1e-12, 1.0 + m00 - m11 - m22)) * 2.0
        quaternion = (
            0.25 * scale,
            (m01 + m10) / scale,
            (m02 + m20) / scale,
            (m21 - m12) / scale,
        )
    elif m11 > m22:
        scale = math.sqrt(max(1e-12, 1.0 + m11 - m00 - m22)) * 2.0
        quaternion = (
            (m01 + m10) / scale,
            0.25 * scale,
            (m12 + m21) / scale,
            (m02 - m20) / scale,
        )
    else:
        scale = math.sqrt(max(1e-12, 1.0 + m22 - m00 - m11)) * 2.0
        quaternion = (
            (m02 + m20) / scale,
            (m12 + m21) / scale,
            0.25 * scale,
            (m10 - m01) / scale,
        )
    return _normalize_vector(quaternion)


def _wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _wait_for_future(node, future, timeout_seconds):
    deadline = time.monotonic() + timeout_seconds
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        # The station cruise is driven by the executor started in ``main``;
        # do not nest spin_once here or action callbacks can starve MoveIt.
        time.sleep(0.1)
    return future.done()


def _spin_executor(executor):
    """Treat ROS context shutdown during Ctrl-C as a normal executor exit."""
    try:
        executor.spin()
    except Exception:
        if rclpy.ok():
            raise


def main(args=None):
    rclpy.init(args=args)
    cruise = None
    executor = None
    spin_thread = None
    try:
        cruise = StationCruise()
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(cruise)
        spin_thread = Thread(target=_spin_executor, args=(executor,), daemon=True)
        spin_thread.start()
        cruise.run()
    except KeyboardInterrupt:
        if cruise is not None and rclpy.ok():
            cruise._status("CANCELLED")
    finally:
        if executor is not None:
            executor.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)
        if cruise is not None:
            cruise.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
