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
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    voice_audio_file = LaunchConfiguration("voice_audio_file")
    nav_only = LaunchConfiguration("nav_only")
    skip_visual_dock = LaunchConfiguration("skip_visual_dock")

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz, "launch", "world.launch.py")),
        launch_arguments={
            "gui": gui,
            "lighting_profile": lighting_profile,
            "enable_actor": enable_actor,
            "require_finger_contact": require_finger_contact,
            "use_refine_detect": use_refine_detect,
            "use_wrist_detect": use_wrist_detect,
        }.items(),
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
            "use_rviz": use_rviz,
        }.items(),
        condition=IfCondition(launch_navigation),
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
        condition=IfCondition(launch_perception),
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
            launch_perception,
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
            launch_perception,
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
            launch_perception,
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
        condition=IfCondition(launch_g4g5_results),
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
        condition=IfCondition(launch_g4g5_results),
    )
    g4g5_result = Node(
        package="lab_cobot_bringup",
        executable="g4g5_result_node",
        output="screen",
        parameters=[{"target_object": target_object}],
        condition=IfCondition(launch_g4g5_results),
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
        condition=IfCondition(LaunchConfiguration("launch_mission")),
    )
    voice = Node(
        package="lab_cobot_bringup",
        executable="voice_node",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "audio_file": voice_audio_file,
        }],
        condition=IfCondition(launch_voice),
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
    stage6 = TimerAction(period=60.0, actions=[mission, voice, g4g5_result])

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
        DeclareLaunchArgument("skip_visual_dock", default_value="false"),
        DeclareLaunchArgument("nav_only", default_value="false"),
        DeclareLaunchArgument("voice_audio_file", default_value=""),
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
