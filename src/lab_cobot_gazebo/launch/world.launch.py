"""
Launch the Gazebo Classic lab world and spawn the mobile manipulator.

需 GUI/运行时(用户收尾验证):
    ros2 launch lab_cobot_gazebo world.launch.py
headless(仅 gzserver,验证物理/话题):
    ros2 launch lab_cobot_gazebo world.launch.py gui:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    AppendEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    desc_pkg = get_package_share_directory("lab_cobot_description")
    gz_pkg = get_package_share_directory("lab_cobot_gazebo")
    gazebo_ros = get_package_share_directory("gazebo_ros")

    world = LaunchConfiguration(
        "world", default=os.path.join(gz_pkg, "worlds", "lab.world")
    )
    urdf_xacro = os.path.join(desc_pkg, "urdf", "lab_cobot.urdf.xacro")
    require_finger_contact = LaunchConfiguration("require_finger_contact")
    enable_contact_force = LaunchConfiguration("enable_contact_force")
    enable_lab_sensors = LaunchConfiguration("enable_lab_sensors")
    use_refine_detect = LaunchConfiguration("use_refine_detect")
    use_wrist_detect = LaunchConfiguration("use_wrist_detect")
    use_wrist_camera = PythonExpression([
        "'true' if ('",
        use_refine_detect,
        "' == 'true' or '",
        use_wrist_detect,
        "' == 'true') else 'false'",
    ])
    robot_description = {
        "robot_description": Command([
            "xacro ",
            urdf_xacro,
            " require_finger_contact:=",
            require_finger_contact,
            " gazebo_tactile_probe:=true",
            " enable_contact_force:=",
            enable_contact_force,
            " enable_lab_sensors:=",
            enable_lab_sensors,
            " wrist_refine_camera:=",
            use_wrist_camera,
        ])
    }
    plugin_path = os.path.join(os.path.dirname(os.path.dirname(gz_pkg)), "lib")

    gui = LaunchConfiguration("gui")
    robot_x = LaunchConfiguration("robot_x", default="0.0")
    robot_y = LaunchConfiguration("robot_y", default="0.0")
    robot_yaw = LaunchConfiguration("robot_yaw", default="0.0")

    # 让 Gazebo 能解析 world 里的 model://aruco_sample
    model_path = AppendEnvironmentVariable(
        "GAZEBO_MODEL_PATH", os.path.join(gz_pkg, "models")
    )
    gazebo_plugin_path = AppendEnvironmentVariable(
        "GAZEBO_PLUGIN_PATH", plugin_path
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world, "verbose": "true"}.items(),
    )
    gzclient = ExecuteProcess(
        cmd=["gzclient", "--gui-client-plugin=libgazebo_ros_eol_gui.so"],
        output="screen",
        additional_env={
            "GAZEBO_MODEL_PATH": os.path.join(gz_pkg, "models"),
            "GAZEBO_MODEL_DATABASE_URI": "",
        },
        condition=IfCondition(gui),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "lab_cobot",
            "-timeout", "120",
            "-x", robot_x, "-y", robot_y, "-z", "0.0", "-Y", robot_yaw,
            # 注:Gazebo Classic 的 spawn_entity.py 不支持 -J 设初始关节;
            # 臂初始姿态由 URDF ros2_control 的 initial_value(=home 收拢)
            # 经 gazebo_ros2_control 设置,见 config/initial_positions.yaml
        ],
        output="screen",
    )

    controller_bootstrap = Node(
        package="lab_cobot_gazebo",
        executable="controller_bootstrap",
        output="screen",
    )
    delay_controller_bootstrap = RegisterEventHandler(
        OnProcessExit(target_action=spawn_entity, on_exit=[controller_bootstrap])
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="是否显示 Gazebo GUI"),
        DeclareLaunchArgument(
            "world", default_value=os.path.join(gz_pkg, "worlds", "lab.world"),
            description="Gazebo world 文件",
        ),
        DeclareLaunchArgument(
            "robot_x", default_value="0.0", description="机器人初始世界 x 坐标"
        ),
        DeclareLaunchArgument(
            "robot_y", default_value="0.0", description="机器人初始世界 y 坐标"
        ),
        DeclareLaunchArgument(
            "robot_yaw", default_value="0.0", description="机器人初始世界偏航角"
        ),
        # 2026-07-10 T-5 翻默认:与 bringup 一致,单独起 world 调试时同样门控 attach。
        DeclareLaunchArgument("require_finger_contact", default_value="true"),
        DeclareLaunchArgument(
            "enable_contact_force",
            default_value="false",
            description="仅 G4 力曲线采集时启用 tactile probe 的物理接触力",
        ),
        DeclareLaunchArgument(
            "enable_lab_sensors",
            default_value="true",
            description=(
                "是否启用底盘 LiDAR/IMU/bench RGB-D；抓取专项固定坐标验证可设为 false 降低 Gazebo 负载"
            ),
        ),
        DeclareLaunchArgument("use_refine_detect", default_value="false"),
        DeclareLaunchArgument("use_wrist_detect", default_value="false"),
        model_path,
        gazebo_plugin_path,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
        delay_controller_bootstrap,
    ])
