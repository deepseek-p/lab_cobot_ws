"""
Integrated launch for the cross-station pick-and-place stack.

    ros2 launch lab_cobot_bringup lab_cobot.launch.py
启动顺序: Gazebo+机器人+控制器 -> (延迟) move_group + Nav2 + 感知 -> (再延迟) mission。
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

    gui = LaunchConfiguration("gui")
    use_rviz = LaunchConfiguration("use_rviz")
    map_yaml = LaunchConfiguration("map")
    use_truth_pose = LaunchConfiguration("use_truth_pose")
    use_sim_attach = LaunchConfiguration("use_sim_attach")
    use_dl_perception = LaunchConfiguration("use_dl_perception")
    dl_device = LaunchConfiguration("dl_device")
    dl_imgsz = LaunchConfiguration("dl_imgsz")
    target_object = LaunchConfiguration("target_object")
    require_finger_contact = LaunchConfiguration("require_finger_contact")
    use_tactile_grasp = LaunchConfiguration("use_tactile_grasp")
    use_refine_detect = LaunchConfiguration("use_refine_detect")
    use_wrist_detect = LaunchConfiguration("use_wrist_detect")
    use_planning_scene_obstacles = LaunchConfiguration(
        "use_planning_scene_obstacles"
    )
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
            "' == 'true') else 'false'",
        ])),
    )
    rover_twist_relay = Node(
        package="lab_cobot_bringup",
        executable="rover_twist_relay",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "rover": "mecanum3",
            "mecanum3.wheel_radius": 0.07,
            "mecanum3.wheel_separation_width": 0.36,
            "mecanum3.wheel_separation_length": 0.263,
            "max_vx": 0.5,
            "max_vy": 0.3,
            "max_wz": 1.2,
            "max_accel_xy": 0.5,
            "max_accel_wz": 1.5,
            "command_timeout": 0.25,
        }],
    )
    passive_mecanum_joint_states = Node(
        package="lab_cobot_bringup",
        executable="passive_mecanum_joint_states",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    gripper_attach_bridge = Node(
        package="lab_cobot_bringup",
        executable="gripper_attach_bridge",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "tf_reference_frame": "odom",
        }],
        condition=IfCondition(PythonExpression([
            "'true' if ('",
            use_sim_attach,
            "' == 'true' or '",
            use_tactile_grasp,
            "' == 'true') else 'false'",
        ])),
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

    stage2 = TimerAction(
        period=10.0,
        actions=[
            move_group,
            table_scene_initializer,
            navigation,
            aruco,
            wrist_aruco,
            object_detector,
        ],
    )
    stage3 = TimerAction(period=15.0, actions=[mission, voice])

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
            default_value="true",
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
        DeclareLaunchArgument("require_finger_contact", default_value="true"),
        DeclareLaunchArgument("use_tactile_grasp", default_value="true"),
        DeclareLaunchArgument("use_refine_detect", default_value="true"),
        DeclareLaunchArgument("use_wrist_detect", default_value="true"),
        DeclareLaunchArgument(
            "use_planning_scene_obstacles", default_value="true"
        ),
        DeclareLaunchArgument("launch_voice", default_value="false"),
        DeclareLaunchArgument("voice_audio_file", default_value=""),
        SetEnvironmentVariable("GALLIUM_DRIVER", "d3d12"),
        SetEnvironmentVariable("MESA_D3D12_DEFAULT_ADAPTER_NAME", "NVIDIA"),
        SetEnvironmentVariable("QT_X11_NO_MITSHM", "1"),
        SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
        world,
        rover_twist_relay,
        passive_mecanum_joint_states,
        gripper_attach_bridge,
        stage2,
        stage3,
    ])
