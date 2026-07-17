"""
Launch the Gazebo Classic lab world and spawn the mobile manipulator.

运行示例:
    ros2 launch lab_cobot_gazebo world.launch.py
    ros2 launch lab_cobot_gazebo world.launch.py gui:=false
    ros2 launch lab_cobot_gazebo world.launch.py lighting_profile:=dark enable_actor:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _continue_on_success(event, next_actions, controller_name):
    if event.returncode == 0:
        return next_actions
    return [EmitEvent(event=Shutdown(
        reason=f"controller {controller_name} failed with code {event.returncode}"
    ))]


def _world_filename_from_profile(lighting_profile: str, enable_actor: bool) -> str:
    profile = str(lighting_profile).strip().lower()
    actor_suffix = "_actor" if enable_actor else ""
    mapping = {
        "normal": f"lab{actor_suffix}.world",
        "dark": f"lab_dark{actor_suffix}.world",
        "reflective": f"lab_reflective{actor_suffix}.world",
    }
    if profile not in mapping:
        supported = ", ".join(sorted(mapping))
        raise ValueError(
            f"unsupported lighting_profile={lighting_profile!r}; "
            f"supported: {supported}"
        )
    return mapping[profile]


def _gzserver_action(context, gazebo_ros, gz_pkg):
    lighting_profile = LaunchConfiguration("lighting_profile").perform(context)
    enable_actor = LaunchConfiguration("enable_actor").perform(context).lower() == "true"
    world = os.path.join(
        gz_pkg,
        "worlds",
        _world_filename_from_profile(lighting_profile, enable_actor),
    )
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world, "verbose": "true"}.items(),
    )
    return [gzserver]


def generate_launch_description():
    desc_pkg = get_package_share_directory("lab_cobot_description")
    gz_pkg = get_package_share_directory("lab_cobot_gazebo")
    gazebo_ros = get_package_share_directory("gazebo_ros")

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

    gazebo_resources = AppendEnvironmentVariable(
        "GAZEBO_RESOURCE_PATH", "/usr/share/gazebo-11"
    )
    gazebo_builtin_models = AppendEnvironmentVariable(
        "GAZEBO_MODEL_PATH", "/usr/share/gazebo-11/models"
    )
    description_package_models = AppendEnvironmentVariable(
        "GAZEBO_MODEL_PATH", os.path.dirname(desc_pkg)
    )
    gazebo_offline = SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", "")

    model_path = AppendEnvironmentVariable(
        "GAZEBO_MODEL_PATH", os.path.join(gz_pkg, "models")
    )
    gazebo_plugin_path = AppendEnvironmentVariable(
        "GAZEBO_PLUGIN_PATH", plugin_path
    )

    gzserver = OpaqueFunction(
        function=lambda context: _gzserver_action(context, gazebo_ros, gz_pkg)
    )
    gzclient = ExecuteProcess(
        cmd=["gzclient", "--gui-client-plugin=libgazebo_ros_eol_gui.so"],
        output="screen",
        additional_env={
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
            "-x", "2.25", "-y", "-2.10", "-z", "0.0",
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
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="是否显示 Gazebo GUI"),
        DeclareLaunchArgument("lighting_profile", default_value="normal"),
        DeclareLaunchArgument("enable_actor", default_value="false"),
        DeclareLaunchArgument("require_finger_contact", default_value="true"),
        DeclareLaunchArgument("use_refine_detect", default_value="false"),
        DeclareLaunchArgument("use_wrist_detect", default_value="false"),
        gazebo_resources,
        gazebo_builtin_models,
        description_package_models,
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
    ])
