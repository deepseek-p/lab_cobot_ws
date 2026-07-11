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
    EmitEvent,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def _continue_on_success(event, next_actions, controller_name):
    if event.returncode == 0:
        return next_actions
    return [EmitEvent(event=Shutdown(
        reason=f"controller {controller_name} failed with code {event.returncode}"
    ))]


def generate_launch_description():
    desc_pkg = get_package_share_directory("lab_cobot_description")
    gz_pkg = get_package_share_directory("lab_cobot_gazebo")
    gazebo_ros = get_package_share_directory("gazebo_ros")

    world = os.path.join(gz_pkg, "worlds", "lab.world")
    urdf_xacro = os.path.join(desc_pkg, "urdf", "lab_cobot.urdf.xacro")
    require_finger_contact = LaunchConfiguration("require_finger_contact")
    use_refine_detect = LaunchConfiguration("use_refine_detect")
    robot_description = {
        "robot_description": Command([
            "xacro ",
            urdf_xacro,
            " require_finger_contact:=",
            require_finger_contact,
            " gazebo_tactile_probe:=true",
            " wrist_refine_camera:=",
            use_refine_detect,
        ])
    }
    plugin_path = os.path.join(os.path.dirname(os.path.dirname(gz_pkg)), "lib")

    gui = LaunchConfiguration("gui")

    gazebo_resources = AppendEnvironmentVariable(
        "GAZEBO_RESOURCE_PATH", "/usr/share/gazebo-11"
    )
    gazebo_builtin_models = AppendEnvironmentVariable(
        "GAZEBO_MODEL_PATH", "/usr/share/gazebo-11/models"
    )
    gazebo_offline = SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", "")

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
    mecanum_kinematic_drive = Node(
        package="lab_cobot_gazebo",
        executable="mecanum_gazebo_kinematic_drive",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "model_name": "lab_cobot",
            "model_states_topic": "/gazebo/model_states",
            "service_name": "/gazebo/set_entity_state",
        }],
    )
    gazebo_odom_bridge = Node(
        package="lab_cobot_gazebo",
        executable="gazebo_odom_bridge",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "link_states_topic": "/gazebo/link_states",
            "odom_topic": "/odom",
            "target_link_name": "lab_cobot::base_footprint",
            "fallback_link_name": "lab_cobot::base_link",
            "odom_frame": "odom",
            "base_frame": "base_footprint",
        }],
    )

    # spawn 完成后顺序激活控制器
    delay_jsb = RegisterEventHandler(
        OnProcessExit(target_action=spawn_entity, on_exit=[joint_state_broadcaster])
    )
    delay_jtc = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=lambda event, _context: _continue_on_success(
                event, [joint_trajectory_controller], "joint_state_broadcaster"
            ),
        )
    )
    delay_gripper = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_trajectory_controller,
            on_exit=lambda event, _context: _continue_on_success(
                event, [gripper_position_controller], "joint_trajectory_controller"
            ),
        )
    )
    delay_wheel_velocity = RegisterEventHandler(
        OnProcessExit(
            target_action=gripper_position_controller,
            on_exit=lambda event, _context: _continue_on_success(
                event, [wheel_velocity_controller], "gripper_position_controller"
            ),
        )
    )
    delay_mecanum_runtime = RegisterEventHandler(
        OnProcessExit(
            target_action=wheel_velocity_controller,
            on_exit=lambda event, _context: _continue_on_success(
                event,
                [mecanum_kinematic_drive, gazebo_odom_bridge],
                "wheel_velocity_controller",
            ),
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="是否显示 Gazebo GUI"),
        # 2026-07-10 T-5 翻默认:与 bringup 一致,单独起 world 调试时同样门控 attach。
        DeclareLaunchArgument("require_finger_contact", default_value="true"),
        DeclareLaunchArgument("use_refine_detect", default_value="false"),
        gazebo_resources,
        gazebo_builtin_models,
        gazebo_offline,
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
        delay_mecanum_runtime,
    ])
