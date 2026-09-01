"""
Integrated launch for the cross-station pick-and-place stack.

    ros2 launch lab_cobot_bringup lab_cobot.launch.py
启动顺序: Gazebo+控制器 -> Nav2 -> MoveIt+ArUco -> 场景+DL -> mission。
含 WSLg 稳定渲染环境变量(源自 robot_lab_demo 经验)。
发指令触发: ros2 topic pub --once /task/instruction std_msgs/msg/String "{data: '把样件从A送到B'}"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from lab_cobot_bringup.grasp_target_config import GRASP_TARGETS
from lab_cobot_bringup.tube_insert_config import TUBE_INSERT_VALIDATION_BASE_POSE
from lab_cobot_manipulation.yellow_cube_slot_config import (
    YELLOW_CUBE_SLOT_VALIDATION_BASE_POSE,
)


def _launch_bool(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() == "true"


def _validation_spawn_value(axis, default_value, context):
    if _launch_bool(context, "launch_yellow_cube_slot_validation"):
        return "%.6f" % YELLOW_CUBE_SLOT_VALIDATION_BASE_POSE[axis]
    if (
        not _launch_bool(context, "launch_grasp_validation")
        and not _launch_bool(context, "launch_tube_insert_validation")
        and not _launch_bool(context, "launch_tube_insert_feasibility")
        and not _launch_bool(context, "launch_yellow_cube_slot_validation")
    ):
        return default_value
    if (
        _launch_bool(context, "launch_tube_insert_validation")
        or _launch_bool(context, "launch_tube_insert_feasibility")
    ):
        return "%.6f" % TUBE_INSERT_VALIDATION_BASE_POSE[axis]
    target = LaunchConfiguration("validation_target").perform(context)
    return "%.6f" % _validation_spawn_pose(target)[axis]


def _world_launch_actions(context, gz, gui, lighting_profile, enable_actor,
                          require_finger_contact, world_use_refine_detect,
                          world_use_wrist_detect, enable_contact_force):
    robot_spawn_x = _validation_spawn_value("x", "4.50", context)
    robot_spawn_y = _validation_spawn_value("y", "-4.20", context)
    robot_spawn_yaw = _validation_spawn_value("yaw", "0.0", context)
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz, "launch", "world.launch.py")),
        launch_arguments={
            "gui": gui,
            "lighting_profile": lighting_profile,
            "enable_actor": enable_actor,
            "require_finger_contact": require_finger_contact,
            "enable_contact_force": enable_contact_force,
            "use_refine_detect": world_use_refine_detect,
            "use_wrist_detect": world_use_wrist_detect,
            "robot_spawn_x": robot_spawn_x,
            "robot_spawn_y": robot_spawn_y,
            "robot_spawn_yaw": robot_spawn_yaw,
            "tube_insert_fixed_base": PythonExpression([
                "'true' if ('",
                LaunchConfiguration("launch_tube_insert_validation"),
                "' == 'true' or '",
                LaunchConfiguration("launch_tube_insert_feasibility"),
                "' == 'true') else 'false'",
            ]),
        }.items(),
    )]


def _validation_spawn_pose(target):
    try:
        return GRASP_TARGETS[str(target)]["validation_base_pose"]
    except KeyError as exc:
        supported = ", ".join(sorted(GRASP_TARGETS))
        raise ValueError(
            f"unsupported validation_target={target!r}; supported: {supported}"
        ) from exc


def _validate_grasp_validation_target(context):
    enabled = LaunchConfiguration("launch_grasp_validation").perform(context)
    if str(enabled).strip().lower() == "true":
        _validation_spawn_pose(
            LaunchConfiguration("validation_target").perform(context)
        )
    return []


def _validate_isolated_modes(context):
    grasp_enabled = LaunchConfiguration("launch_grasp_validation").perform(context)
    tube_enabled = LaunchConfiguration("launch_tube_insert_validation").perform(context)
    tube_feasibility_enabled = LaunchConfiguration(
        "launch_tube_insert_feasibility"
    ).perform(context)
    yellow_cube_slot_enabled = LaunchConfiguration(
        "launch_yellow_cube_slot_validation"
    ).perform(context)
    enabled = [
        str(grasp_enabled).strip().lower() == "true",
        str(tube_enabled).strip().lower() == "true",
        str(tube_feasibility_enabled).strip().lower() == "true",
        str(yellow_cube_slot_enabled).strip().lower() == "true",
    ]
    if sum(1 for value in enabled if value) > 1:
        raise ValueError(
            "validation modes are mutually exclusive"
        )
    return []


def _validation_spawn_expression(axis, default_value):
    entries = ", ".join(
        "'%s': '%.6f'" % (name, config["validation_base_pose"][axis])
        for name, config in GRASP_TARGETS.items()
    )
    return PythonExpression([
        "'%.6f' if '" % YELLOW_CUBE_SLOT_VALIDATION_BASE_POSE[axis],
        LaunchConfiguration("launch_yellow_cube_slot_validation"),
        "' == 'true' else '",
        default_value,
        "' if ('",
        LaunchConfiguration("launch_grasp_validation"),
        "' != 'true' and '",
        LaunchConfiguration("launch_tube_insert_validation"),
        "' != 'true' and '",
        LaunchConfiguration("launch_tube_insert_feasibility"),
        "' != 'true') else '%.6f' if ('"
        % TUBE_INSERT_VALIDATION_BASE_POSE[axis],
        LaunchConfiguration("launch_tube_insert_validation"),
        "' == 'true' or '",
        LaunchConfiguration("launch_tube_insert_feasibility"),
        "' == 'true') else {",
        entries,
        "}['",
        LaunchConfiguration("validation_target"),
        "']",
    ])


def _disable_in_validation_expression(config):
    return PythonExpression([
        "'false' if ('",
        LaunchConfiguration("launch_grasp_validation"),
        "' == 'true' or '",
        LaunchConfiguration("launch_tube_insert_validation"),
        "' == 'true' or '",
        LaunchConfiguration("launch_tube_insert_feasibility"),
        "' == 'true' or '",
        LaunchConfiguration("launch_yellow_cube_slot_validation"),
        "' == 'true') else '",
        config,
        "'",
    ])


def _validation_mode_expression():
    return PythonExpression([
        "'true' if ('",
        LaunchConfiguration("launch_grasp_validation"),
        "' == 'true' or '",
        LaunchConfiguration("launch_tube_insert_validation"),
        "' == 'true' or '",
        LaunchConfiguration("launch_tube_insert_feasibility"),
        "' == 'true' or '",
        LaunchConfiguration("launch_yellow_cube_slot_validation"),
        "' == 'true') else 'false'",
    ])


def _validation_scene_kind_expression():
    return PythonExpression([
        "'tube_insert' if '",
        LaunchConfiguration("launch_tube_insert_validation"),
        "' == 'true' or '",
        LaunchConfiguration("launch_tube_insert_feasibility"),
        "' == 'true' else 'aging_rack_insert' if '",
        LaunchConfiguration("launch_yellow_cube_slot_validation"),
        "' == 'true' else 'tooling'",
    ])


def generate_launch_description():
    gz = get_package_share_directory("lab_cobot_gazebo")
    moveit = get_package_share_directory("lab_cobot_moveit")
    nav = get_package_share_directory("lab_cobot_navigation")
    image = get_package_share_directory("image_pkg")

    gui = LaunchConfiguration("gui")
    use_rviz = LaunchConfiguration("use_rviz")
    map_yaml = LaunchConfiguration("map")
    use_truth_pose = LaunchConfiguration("use_truth_pose")
    use_sim_attach = LaunchConfiguration("use_sim_attach")
    use_dl_perception = LaunchConfiguration("use_dl_perception")
    vision_backend = LaunchConfiguration("vision_backend")
    dl_device = LaunchConfiguration("dl_device")
    dl_imgsz = LaunchConfiguration("dl_imgsz")
    eight_class_model_path = LaunchConfiguration("eight_class_model_path")
    eight_class_imgsz = LaunchConfiguration("eight_class_imgsz")
    target_object = LaunchConfiguration("target_object")
    require_finger_contact = LaunchConfiguration("require_finger_contact")
    enable_contact_force = LaunchConfiguration("enable_contact_force")
    use_tactile_grasp = LaunchConfiguration("use_tactile_grasp")
    use_refine_detect = LaunchConfiguration("use_refine_detect")
    use_wrist_detect = LaunchConfiguration("use_wrist_detect")
    use_planning_scene_obstacles = LaunchConfiguration(
        "use_planning_scene_obstacles"
    )
    launch_g4g5_results = LaunchConfiguration("launch_g4g5_results")
    g4_force_duration = LaunchConfiguration("g4_force_duration")
    g4_output_dir = LaunchConfiguration("g4_output_dir")
    g4_stem = LaunchConfiguration("g4_stem")
    use_wrist_camera = PythonExpression([
        "'true' if ('",
        use_refine_detect,
        "' == 'true' or '",
        use_wrist_detect,
        "' == 'true') else 'false'",
    ])
    lighting_profile = LaunchConfiguration("lighting_profile")
    enable_actor = LaunchConfiguration("enable_actor")
    launch_navigation = LaunchConfiguration("launch_navigation")
    launch_moveit = LaunchConfiguration("launch_moveit")
    launch_perception = LaunchConfiguration("launch_perception")
    launch_voice = LaunchConfiguration("launch_voice")
    launch_grasp_validation = LaunchConfiguration("launch_grasp_validation")
    launch_tube_insert_validation = LaunchConfiguration(
        "launch_tube_insert_validation"
    )
    launch_tube_insert_feasibility = LaunchConfiguration(
        "launch_tube_insert_feasibility"
    )
    launch_yellow_cube_slot_validation = LaunchConfiguration(
        "launch_yellow_cube_slot_validation"
    )
    auto_prepare_tube_arm = LaunchConfiguration("auto_prepare_tube_arm")
    validation_target = LaunchConfiguration("validation_target")
    validation_enable_force_gate = LaunchConfiguration("validation_enable_force_gate")
    validation_enable_force_control = LaunchConfiguration(
        "validation_enable_force_control"
    )
    validation_force_target_n = LaunchConfiguration("validation_force_target_n")
    validation_force_deadband_n = LaunchConfiguration("validation_force_deadband_n")
    validation_force_kp = LaunchConfiguration("validation_force_kp")
    validation_force_max_close_step = LaunchConfiguration(
        "validation_force_max_close_step"
    )
    validation_force_max_open_step = LaunchConfiguration(
        "validation_force_max_open_step"
    )
    validation_force_safety_limit_n = LaunchConfiguration(
        "validation_force_safety_limit_n"
    )
    validation_force_safety_frames = LaunchConfiguration(
        "validation_force_safety_frames"
    )
    validation_force_balance_limit_n = LaunchConfiguration(
        "validation_force_balance_limit_n"
    )
    validation_force_balance_frames = LaunchConfiguration(
        "validation_force_balance_frames"
    )
    validation_force_settle_frames = LaunchConfiguration(
        "validation_force_settle_frames"
    )
    validation_force_filter_window = LaunchConfiguration(
        "validation_force_filter_window"
    )
    material_spare_descend_mode = LaunchConfiguration("material_spare_descend_mode")
    fixture_grasp_point_y_override = LaunchConfiguration(
        "fixture_grasp_point_y_override"
    )
    voice_audio_file = LaunchConfiguration("voice_audio_file")
    nav_only = LaunchConfiguration("nav_only")
    skip_visual_dock = LaunchConfiguration("skip_visual_dock")
    world_use_refine_detect = _disable_in_validation_expression(use_refine_detect)
    world_use_wrist_detect = _disable_in_validation_expression(use_wrist_detect)
    run_navigation = PythonExpression([
        "'true' if ('",
        launch_navigation,
        "' == 'true' and '",
        launch_grasp_validation,
        "' != 'true' and '",
        launch_tube_insert_validation,
        "' != 'true' and '",
        launch_tube_insert_feasibility,
        "' != 'true') else 'true' if '",
        launch_yellow_cube_slot_validation,
        "' == 'true' else 'false'",
    ])
    run_perception = PythonExpression([
        "'true' if ('",
        launch_perception,
        "' == 'true' and '",
        launch_grasp_validation,
        "' != 'true' and '",
        launch_tube_insert_validation,
        "' != 'true' and '",
        launch_tube_insert_feasibility,
        "' != 'true' and '",
        launch_yellow_cube_slot_validation,
        "' != 'true') else 'false'",
    ])
    run_mission = PythonExpression([
        "'true' if ('",
        LaunchConfiguration("launch_mission"),
        "' == 'true' and '",
        launch_grasp_validation,
        "' != 'true' and '",
        launch_tube_insert_validation,
        "' != 'true' and '",
        launch_tube_insert_feasibility,
        "' != 'true' and '",
        launch_yellow_cube_slot_validation,
        "' != 'true') else 'false'",
    ])
    run_voice = PythonExpression([
        "'true' if ('",
        launch_voice,
        "' == 'true' and '",
        launch_grasp_validation,
        "' != 'true' and '",
        launch_tube_insert_validation,
        "' != 'true' and '",
        launch_tube_insert_feasibility,
        "' != 'true' and '",
        launch_yellow_cube_slot_validation,
        "' != 'true') else 'false'",
    ])
    run_g4g5_results = PythonExpression([
        "'true' if ('",
        launch_g4g5_results,
        "' == 'true' and '",
        launch_grasp_validation,
        "' != 'true' and '",
        launch_tube_insert_validation,
        "' != 'true' and '",
        launch_tube_insert_feasibility,
        "' != 'true' and '",
        launch_yellow_cube_slot_validation,
        "' != 'true') else 'false'",
    ])

    world = OpaqueFunction(
        function=lambda context: _world_launch_actions(
            context,
            gz,
            gui,
            lighting_profile,
            enable_actor,
            require_finger_contact,
            world_use_refine_detect,
            world_use_wrist_detect,
            enable_contact_force,
        )
    )
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit, "launch", "move_group.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
        condition=IfCondition(launch_moveit),
    )
    table_scene_initializer = Node(
        package="lab_cobot_moveit",
        executable="table_scene_initializer",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "world_frame": "map",
            "validation_mode": ParameterValue(
                _validation_mode_expression(), value_type=bool
            ),
            "validation_scene_kind": _validation_scene_kind_expression(),
            "robot_spawn_x": _validation_spawn_expression("x", "4.50"),
            "robot_spawn_y": _validation_spawn_expression("y", "-4.20"),
            "robot_spawn_yaw": _validation_spawn_expression("yaw", "0.0"),
        }],
        condition=IfCondition(launch_moveit),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav, "launch", "navigation.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "map": map_yaml,
            "params_file": os.path.join(nav, "config", "nav2_params.yaml"),
            "amcl_initial_pose_x": _validation_spawn_expression("x", "4.50"),
            "amcl_initial_pose_y": _validation_spawn_expression("y", "-4.20"),
            "amcl_initial_pose_yaw": _validation_spawn_expression("yaw", "0.0"),
            "use_rviz": use_rviz,
        }.items(),
        condition=IfCondition(run_navigation),
    )
    aruco = Node(
        package="lab_cobot_perception",
        executable="aruco_detector",
        name="aruco_detector",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "use_gazebo_model_pose": use_truth_pose,
            "gazebo_model_name": "aruco_sample",
            "gazebo_reference_frame": "odom",
            "rgb_topic": "/bench_camera/image_raw",
            "depth_topic": "/bench_camera/depth/image_raw",
            "info_topic": "/bench_camera/camera_info",
            "optical_frame": "camera_optical_frame",
            "target_frame": "base_link",
            "marker_size_m": 0.07 * (240.0 / 312.0),
            "marker_to_object_center_m": 0.03,
        }],
        condition=IfCondition(run_perception),
    )
    wrist_aruco = Node(
        package="lab_cobot_perception",
        executable="aruco_detector",
        name="wrist_aruco_detector",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "topic_namespace": "/perception/wrist",
            "publish_tf": False,
            "rgb_topic": "/wrist_camera/image_raw",
            "depth_topic": "/wrist_camera/depth/image_raw",
            "info_topic": "/wrist_camera/camera_info",
            "optical_frame": "wrist_camera_optical_frame",
            "target_frame": "base_link",
            "marker_size_m": 0.07 * (240.0 / 312.0),
            "marker_to_object_center_m": 0.03,
            "process_period_sec": 0.05,
        }],
        condition=IfCondition(PythonExpression([
            "'true' if ('",
            run_perception,
            "' == 'true' and '",
            use_wrist_camera,
            "' == 'true') else 'false'",
        ])),
    )
    object_detector = Node(
        package="lab_cobot_perception",
        executable="object_detector",
        name="object_detector",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "device": dl_device,
            "imgsz": dl_imgsz,
            "rgb_topic": "/bench_camera/image_raw",
            "depth_topic": "/bench_camera/depth/image_raw",
            "info_topic": "/bench_camera/camera_info",
            "optical_frame": "camera_optical_frame",
            "target_frame": "base_link",
        }],
        condition=IfCondition(PythonExpression([
            "'true' if ('",
            run_perception,
            "' == 'true' and '",
            use_dl_perception,
            "' == 'true' and '",
            vision_backend,
            "' == 'diagnostic') else 'false'",
        ])),
    )
    eight_class_perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(image, "launch", "pose_estimation.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "model_path": eight_class_model_path,
            "device": dl_device,
            "imgsz": eight_class_imgsz,
            "image_topic": "/wrist_camera/image_raw",
            "depth_topic": "/wrist_camera/depth/image_raw",
            "camera_info_topic": "/wrist_camera/camera_info",
            "optical_frame": "wrist_camera_optical_frame",
            "pointcloud_topic": "/image_pkg/wrist_camera_points",
            "target_frame": "base_link",
            "target_label": target_object,
        }.items(),
        condition=IfCondition(PythonExpression([
            "'true' if ('",
            run_perception,
            "' == 'true' and '",
            use_dl_perception,
            "' == 'true' and '",
            vision_backend,
            "' == 'eight_class') else 'false'",
        ])),
    )
    mecanum_wheel_visualizer = Node(
        package="lab_cobot_bringup",
        executable="mecanum_wheel_visualizer",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "publish_odom": False,
        }],
        condition=IfCondition(PythonExpression([
            "'true' if '",
            launch_tube_insert_validation,
            "' != 'true' and '",
            launch_tube_insert_feasibility,
            "' != 'true' else 'false'",
        ])),
    )
    gripper_attach_bridge = Node(
        package="lab_cobot_bringup",
        executable="gripper_attach_bridge",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "tf_reference_frame": "odom",
        }],
        condition=IfCondition(use_sim_attach),
    )
    joint_state_qos_relay = Node(
        package="lab_cobot_manipulation",
        executable="joint_state_qos_relay",
        output="screen",
    )
    g5_dynamic_obstacle = Node(
        package="lab_cobot_manipulation",
        executable="dynamic_arm_obstacle_node",
        output="screen",
        condition=IfCondition(run_g4g5_results),
    )
    g4_contact_force_recorder = Node(
        package="lab_cobot_manipulation",
        executable="contact_force_recorder",
        output="screen",
        arguments=[
            "--duration", g4_force_duration,
            "--target-object", target_object,
            "--output-dir", g4_output_dir,
            "--stem", g4_stem,
        ],
        condition=IfCondition(run_g4g5_results),
    )
    g4g5_result = Node(
        package="lab_cobot_bringup",
        executable="g4g5_result_node",
        output="screen",
        parameters=[{"target_object": target_object}],
        condition=IfCondition(run_g4g5_results),
    )
    mission = Node(
        package="lab_cobot_bringup",
        executable="mission_node",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "llm_enabled": ParameterValue(
                LaunchConfiguration("llm_enabled"), value_type=bool
            ),
            "target_object": target_object,
            "use_tactile_grasp": ParameterValue(
                use_tactile_grasp, value_type=bool
            ),
            "use_refine_detect": ParameterValue(
                use_refine_detect, value_type=bool
            ),
            "use_wrist_detect": ParameterValue(
                use_wrist_detect, value_type=bool
            ),
            "use_planning_scene_obstacles": ParameterValue(
                use_planning_scene_obstacles, value_type=bool
            ),
            "skip_visual_dock": ParameterValue(
                skip_visual_dock, value_type=bool
            ),
            "nav_only": ParameterValue(
                nav_only, value_type=bool
            ),
        }],
        condition=IfCondition(run_mission),
    )
    grasp_validation = Node(
        package="lab_cobot_bringup",
        executable="grasp_validation_node",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "validation_target": validation_target,
            "use_tactile_grasp": ParameterValue(
                use_tactile_grasp, value_type=bool
            ),
            "use_planning_scene_obstacles": ParameterValue(
                use_planning_scene_obstacles, value_type=bool
            ),
            "enable_force_gate": ParameterValue(
                validation_enable_force_gate, value_type=bool
            ),
            "enable_force_control": ParameterValue(
                validation_enable_force_control, value_type=bool
            ),
            "force_target_n": ParameterValue(
                validation_force_target_n, value_type=float
            ),
            "force_deadband_n": ParameterValue(
                validation_force_deadband_n, value_type=float
            ),
            "force_kp": ParameterValue(validation_force_kp, value_type=float),
            "force_max_close_step": ParameterValue(
                validation_force_max_close_step, value_type=float
            ),
            "force_max_open_step": ParameterValue(
                validation_force_max_open_step, value_type=float
            ),
            "force_safety_limit_n": ParameterValue(
                validation_force_safety_limit_n, value_type=float
            ),
            "force_safety_frames": ParameterValue(
                validation_force_safety_frames, value_type=int
            ),
            "force_balance_limit_n": ParameterValue(
                validation_force_balance_limit_n, value_type=float
            ),
            "force_balance_frames": ParameterValue(
                validation_force_balance_frames, value_type=int
            ),
            "force_settle_frames": ParameterValue(
                validation_force_settle_frames, value_type=int
            ),
            "force_filter_window": ParameterValue(
                validation_force_filter_window, value_type=int
            ),
            "material_spare_descend_mode": material_spare_descend_mode,
            "fixture_grasp_point_y_override": ParameterValue(
                fixture_grasp_point_y_override, value_type=float
            ),
        }],
        condition=IfCondition(launch_grasp_validation),
    )
    tube_insert_validation = Node(
        package="lab_cobot_bringup",
        executable="tube_insert_validation_node",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "auto_prepare_tube_arm": ParameterValue(
                auto_prepare_tube_arm, value_type=bool
            ),
        }],
        condition=IfCondition(launch_tube_insert_validation),
    )
    tube_insert_feasibility = Node(
        package="lab_cobot_bringup",
        executable="tube_insert_base_feasibility",
        output="screen",
        arguments=["--current-base-only"],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(launch_tube_insert_feasibility),
    )
    yellow_cube_slot_validation = Node(
        package="lab_cobot_bringup",
        executable="yellow_cube_slot_validation_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(launch_yellow_cube_slot_validation),
    )
    voice = Node(
        package="lab_cobot_bringup",
        executable="voice_node",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "audio_file": voice_audio_file,
        }],
        condition=IfCondition(run_voice),
    )

    # 生命周期 service 在 CPU 推理与 MoveIt 同时冷启动时会偶发丢响应。
    # 先给 Nav2 定位链独占启动窗口,再逐级引入其余运行负载。
    stage2 = TimerAction(period=10.0, actions=[navigation, joint_state_qos_relay])
    stage3 = TimerAction(
        period=15.0,
        actions=[
            move_group,
            aruco,
            wrist_aruco,
            g5_dynamic_obstacle,
            g4_contact_force_recorder,
        ],
    )
    stage4 = TimerAction(period=25.0, actions=[table_scene_initializer])
    stage5 = TimerAction(
        period=30.0,
        actions=[object_detector, eight_class_perception],
    )
    stage6 = TimerAction(
        period=60.0,
        actions=[
            mission,
            voice,
            g4g5_result,
            grasp_validation,
            tube_insert_validation,
            tube_insert_feasibility,
            yellow_cube_slot_validation,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="Gazebo GUI"),
        DeclareLaunchArgument("lighting_profile", default_value="normal"),
        DeclareLaunchArgument("enable_actor", default_value="false"),
        DeclareLaunchArgument("launch_navigation", default_value="true"),
        DeclareLaunchArgument("launch_moveit", default_value="true"),
        DeclareLaunchArgument("launch_perception", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false", description="Nav2 RViz"),
        DeclareLaunchArgument("launch_mission", default_value="true"),
        DeclareLaunchArgument(
            "llm_enabled",
            default_value="false",
            description="true=LLM instruction planning (needs LLM_API_KEY env)",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(nav, "maps", "map.yaml"),
            description="Navigation map YAML",
        ),
        DeclareLaunchArgument(
            "use_truth_pose",
            default_value="false",
            description="true=stable Gazebo model pose fallback, false=RGB-D ArUco detection",
        ),
        DeclareLaunchArgument(
            "use_sim_attach",
            default_value="false",
            description="true=debug SetEntityState attach bridge, false=physical/contact grasp",
        ),
        DeclareLaunchArgument(
            "use_dl_perception",
            default_value="true",
            description="true=launch the selected DL perception backend",
        ),
        DeclareLaunchArgument(
            "vision_backend",
            default_value="diagnostic",
            choices=["diagnostic", "eight_class"],
            description="DL backend publishing /perception/objects",
        ),
        DeclareLaunchArgument(
            "dl_device",
            default_value="auto",
            description="YOLO-World device: auto, cpu, or CUDA device id",
        ),
        DeclareLaunchArgument(
            "dl_imgsz",
            default_value="1280",
            description="YOLO-World inference image size",
        ),
        DeclareLaunchArgument(
            "eight_class_model_path",
            default_value="~/lab_cobot_models/lab_cobot_eight_class.pt",
            description="External eight-class YOLO weight path",
        ),
        DeclareLaunchArgument(
            "eight_class_imgsz",
            default_value="640",
            description="Eight-class YOLO inference image size",
        ),
        DeclareLaunchArgument("target_object", default_value="aruco_sample"),
        DeclareLaunchArgument("require_finger_contact", default_value="true"),
        DeclareLaunchArgument("enable_contact_force", default_value="false"),
        DeclareLaunchArgument("use_tactile_grasp", default_value="true"),
        DeclareLaunchArgument("use_refine_detect", default_value="true"),
        DeclareLaunchArgument("use_wrist_detect", default_value="true"),
        DeclareLaunchArgument(
            "use_planning_scene_obstacles", default_value="true"
        ),
        DeclareLaunchArgument("launch_g4g5_results", default_value="true"),
        DeclareLaunchArgument("g4_force_duration", default_value="900.0"),
        DeclareLaunchArgument("g4_output_dir", default_value="g4_artifacts"),
        DeclareLaunchArgument("g4_stem", default_value="g4_lab_ab_contact_force"),
        DeclareLaunchArgument("launch_voice", default_value="false"),
        DeclareLaunchArgument("launch_grasp_validation", default_value="false"),
        DeclareLaunchArgument("launch_tube_insert_validation", default_value="false"),
        DeclareLaunchArgument("launch_tube_insert_feasibility", default_value="false"),
        DeclareLaunchArgument(
            "launch_yellow_cube_slot_validation",
            default_value="false",
        ),
        DeclareLaunchArgument("auto_prepare_tube_arm", default_value="true"),
        DeclareLaunchArgument(
            "validation_target",
            default_value="material_spare_igbt",
            choices=sorted(GRASP_TARGETS),
        ),
        DeclareLaunchArgument("validation_enable_force_gate", default_value="false"),
        DeclareLaunchArgument("validation_enable_force_control", default_value="false"),
        DeclareLaunchArgument("validation_force_target_n", default_value="8.0"),
        DeclareLaunchArgument("validation_force_deadband_n", default_value="1.0"),
        DeclareLaunchArgument("validation_force_kp", default_value="0.00020"),
        DeclareLaunchArgument("validation_force_max_close_step", default_value="0.00030"),
        DeclareLaunchArgument("validation_force_max_open_step", default_value="0.00008"),
        DeclareLaunchArgument("validation_force_safety_limit_n", default_value="18.0"),
        DeclareLaunchArgument("validation_force_safety_frames", default_value="3"),
        DeclareLaunchArgument("validation_force_balance_limit_n", default_value="12.0"),
        DeclareLaunchArgument("validation_force_balance_frames", default_value="3"),
        DeclareLaunchArgument("validation_force_settle_frames", default_value="3"),
        DeclareLaunchArgument("validation_force_filter_window", default_value="3"),
        DeclareLaunchArgument(
            "material_spare_descend_mode",
            default_value="horizontal_insert",
            choices=("horizontal_insert", "cartesian"),
        ),
        DeclareLaunchArgument(
            "fixture_grasp_point_y_override",
            default_value="nan",
        ),
        DeclareLaunchArgument("skip_visual_dock", default_value="false"),
        DeclareLaunchArgument("nav_only", default_value="false"),
        DeclareLaunchArgument("voice_audio_file", default_value=""),
        OpaqueFunction(function=_validate_grasp_validation_target),
        OpaqueFunction(function=_validate_isolated_modes),
        SetEnvironmentVariable("GALLIUM_DRIVER", "d3d12"),
        SetEnvironmentVariable("MESA_D3D12_DEFAULT_ADAPTER_NAME", "NVIDIA"),
        SetEnvironmentVariable("QT_X11_NO_MITSHM", "1"),
        SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
        world,
        mecanum_wheel_visualizer,
        gripper_attach_bridge,
        stage2,
        stage3,
        stage4,
        stage5,
        stage6,
    ])
