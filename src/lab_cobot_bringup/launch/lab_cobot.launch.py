"""
Integrated launch for the cross-station pick-and-place stack.

    ros2 launch lab_cobot_bringup lab_cobot.launch.py
启动顺序:Gazebo+机器人+控制器 → (延迟)move_group + Nav2 + 感知 → (再延迟)mission。
含 WSLg 稳定渲染环境变量(源自 robot_lab_demo 经验)。
发指令触发:ros2 topic pub --once /task/instruction std_msgs/msg/String "{data: '把样件从A送到B'}"
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

    gui = LaunchConfiguration("gui")
    use_rviz = LaunchConfiguration("use_rviz")
    map_yaml = LaunchConfiguration("map")
    robot_x = LaunchConfiguration("robot_x")
    robot_y = LaunchConfiguration("robot_y")
    robot_yaw = LaunchConfiguration("robot_yaw")
    use_truth_pose = LaunchConfiguration("use_truth_pose")
    use_sim_attach = LaunchConfiguration("use_sim_attach")
    use_dl_perception = LaunchConfiguration("use_dl_perception")
    dl_device = LaunchConfiguration("dl_device")
    dl_imgsz = LaunchConfiguration("dl_imgsz")
    target_object = LaunchConfiguration("target_object")
    require_finger_contact = LaunchConfiguration("require_finger_contact")
    enable_contact_force = LaunchConfiguration("enable_contact_force")
    enable_lab_sensors = LaunchConfiguration("enable_lab_sensors")
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
    launch_voice = LaunchConfiguration("launch_voice")
    voice_audio_file = LaunchConfiguration("voice_audio_file")

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz, "launch", "world.launch.py")),
        launch_arguments={
            "gui": gui,
            "robot_x": robot_x,
            "robot_y": robot_y,
            "robot_yaw": robot_yaw,
            "require_finger_contact": require_finger_contact,
            "enable_contact_force": enable_contact_force,
            "enable_lab_sensors": enable_lab_sensors,
            "use_refine_detect": use_refine_detect,
            "use_wrist_detect": use_wrist_detect,
        }.items(),
    )
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit, "launch", "move_group.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
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
        }],
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
            "process_period_sec": 0.05,
        }],
        condition=IfCondition(use_wrist_camera),
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
        condition=IfCondition(use_dl_perception),
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
        parameters=[{
            "target_object": target_object,
        }],
        condition=IfCondition(launch_g4g5_results),
    )
    mission = Node(
        package="lab_cobot_bringup",
        executable="mission_node",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            # LLM 任务拆解开关:默认 false 保证 E2E/CI 离线;
            # 演示时 llm_enabled:=true 并 export LLM_API_KEY 打开
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
            # 规划场景障碍注入(台面盒+持物样件附着盒),默认开启。
            "use_planning_scene_obstacles": ParameterValue(
                use_planning_scene_obstacles, value_type=bool
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

    # 等 Gazebo + spawn + 控制器起来后再起规划/导航/感知
    stage2 = TimerAction(
        period=10.0,
        actions=[
            move_group,
            joint_state_qos_relay,
            navigation,
            aruco,
            wrist_aruco,
            object_detector,
            g5_dynamic_obstacle,
            g4_contact_force_recorder,
        ],
    )
    # 再等编排依赖就绪。navigation.launch.py 内部还会延迟 15s 启动
    # Nav2 lifecycle manager；stage2=10s,所以 manager 约 25s 才启动。
    # WSL/Gazebo 负载下 Nav2 激活还会再花十几秒,mission 晚起更稳。
    stage3 = TimerAction(period=60.0, actions=[mission, voice, g4g5_result])

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="Gazebo GUI"),
        DeclareLaunchArgument(
            "robot_x",
            default_value="4.50",
            description="Gazebo world robot x",
        ),
        DeclareLaunchArgument(
            "robot_y",
            default_value="-4.20",
            description="Gazebo world robot y",
        ),
        DeclareLaunchArgument(
            "robot_yaw",
            default_value="0.0",
            description="Gazebo world robot yaw",
        ),
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
            description="true=debug Gazebo model pose, false=RGB-D ArUco detection",
        ),
        DeclareLaunchArgument(
            "use_sim_attach",
            default_value="false",
            description="true=debug SetEntityState attach bridge, false=physical/contact grasp",
        ),
        DeclareLaunchArgument(
            "use_dl_perception",
            default_value="true",
            description="true=launch YOLO-World point cloud object detector",
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
        DeclareLaunchArgument("target_object", default_value="aruco_sample"),
        DeclareLaunchArgument(
            "enable_contact_force",
            default_value="false",
            description="G4 tactile probe contact force enable",
        ),
        DeclareLaunchArgument(
            "enable_lab_sensors",
            default_value="true",
            description="Gazebo lab sensors enable",
        ),
        # 2026-07-10 T-5 翻默认:触觉步进闭合+双指接触门控为默认抓取路径。
        # 两开关必须一起翻:只开门控不开触觉时固定闭合 0.009 永不接触,正常抓取全失败。
        # 回退路径(靠近即焊):require_finger_contact:=false use_tactile_grasp:=false
        DeclareLaunchArgument("require_finger_contact", default_value="true"),
        DeclareLaunchArgument("use_tactile_grasp", default_value="true"),
        # 两段式精修总开关:xacro 相机、检测节点、mission 三处同源。
        # 2026-07-14 用户裁决:腕相机链为主路径默认常开,bench 为降级路径。
        DeclareLaunchArgument("use_refine_detect", default_value="true"),
        # 标准 eye-in-hand DETECT 开关；相机/检测节点与精修开关做 OR。
        DeclareLaunchArgument("use_wrist_detect", default_value="true"),
        # 机械臂规划场景障碍注入:台面盒+持物样件附着盒(mission 透传)。
        DeclareLaunchArgument(
            "use_planning_scene_obstacles", default_value="true"
        ),
        DeclareLaunchArgument(
            "launch_g4g5_results",
            default_value="true",
            description="true=在正式 A->B 流程中启动 G4/G5 结果显示与采集旁路",
        ),
        DeclareLaunchArgument(
            "g4_force_duration",
            default_value="900.0",
            description="G4 接触力曲线采集秒数；节点退出时写 CSV/PNG",
        ),
        DeclareLaunchArgument(
            "g4_output_dir",
            default_value="g4_artifacts",
            description="G4 接触力 CSV/PNG 输出目录",
        ),
        DeclareLaunchArgument(
            "g4_stem",
            default_value="g4_lab_ab_contact_force",
            description="G4 接触力 CSV/PNG 文件名前缀",
        ),
        DeclareLaunchArgument("launch_voice", default_value="false"),
        DeclareLaunchArgument("voice_audio_file", default_value=""),
        # WSLg 稳定渲染(源自 robot_lab_demo 验证经验)
        SetEnvironmentVariable("GALLIUM_DRIVER", "d3d12"),
        SetEnvironmentVariable("MESA_D3D12_DEFAULT_ADAPTER_NAME", "NVIDIA"),
        SetEnvironmentVariable("QT_X11_NO_MITSHM", "1"),
        SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
        world,
        mecanum_wheel_visualizer,
        gripper_attach_bridge,
        stage2,
        stage3,
    ])
