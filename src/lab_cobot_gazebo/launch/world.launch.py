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

    world = os.path.join(gz_pkg, "worlds", "lab.world")
    urdf_xacro = os.path.join(desc_pkg, "urdf", "lab_cobot.urdf.xacro")
    require_finger_contact = LaunchConfiguration("require_finger_contact")
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
            " wrist_refine_camera:=",
            use_wrist_camera,
        ])
    }
    plugin_path = os.path.join(os.path.dirname(os.path.dirname(gz_pkg)), "lib")

    gui = LaunchConfiguration("gui")

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
            "-x", "0.0", "-y", "0.0", "-z", "0.0",
            # 注:Gazebo Classic 的 spawn_entity.py 不支持 -J 设初始关节;
            # 臂初始姿态由 URDF ros2_control 的 initial_value(=home 收拢)
            # 经 gazebo_ros2_control 设置,见 config/initial_positions.yaml
        ],
        output="screen",
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )
    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )
    gripper_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_position_controller", "-c", "/controller_manager"],
    )
    wheel_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wheel_velocity_controller", "-c", "/controller_manager"],
    )

    # spawn 完成后顺序激活控制器
    delay_jsb = RegisterEventHandler(
        OnProcessExit(target_action=spawn_entity, on_exit=[joint_state_broadcaster])
    )
    delay_jtc = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster, on_exit=[joint_trajectory_controller]
        )
    )
    delay_gripper = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_trajectory_controller,
            on_exit=[gripper_position_controller],
        )
    )
    delay_wheel_velocity = RegisterEventHandler(
        OnProcessExit(
            target_action=gripper_position_controller,
            on_exit=[wheel_velocity_controller],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="是否显示 Gazebo GUI"),
        # 2026-07-10 T-5 翻默认:与 bringup 一致,单独起 world 调试时同样门控 attach。
        DeclareLaunchArgument("require_finger_contact", default_value="true"),
        DeclareLaunchArgument("use_refine_detect", default_value="false"),
        DeclareLaunchArgument("use_wrist_detect", default_value="false"),
        model_path,
        gazebo_plugin_path,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
        delay_jsb,
        delay_jtc,
        delay_gripper,
        delay_wheel_velocity,
    ])
