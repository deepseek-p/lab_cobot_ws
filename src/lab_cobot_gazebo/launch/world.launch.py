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
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


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
    enable_contact_force = LaunchConfiguration("enable_contact_force")
    enable_lab_sensors = LaunchConfiguration("enable_lab_sensors")
    use_refine_detect = LaunchConfiguration("use_refine_detect")
    use_wrist_detect = LaunchConfiguration("use_wrist_detect")
    robot_spawn_x = LaunchConfiguration("robot_spawn_x")
    robot_spawn_y = LaunchConfiguration("robot_spawn_y")
    robot_spawn_yaw = LaunchConfiguration("robot_spawn_yaw")
    tube_insert_fixed_base = LaunchConfiguration("tube_insert_fixed_base")
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
            " tube_insert_fixed_base:=",
            tube_insert_fixed_base,
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
            "-x", robot_spawn_x,
            "-y", robot_spawn_y,
            "-z", "0.0",
            "-Y", robot_spawn_yaw,
        ],
        output="screen",
    )
    final_spawn_x_log = LogInfo(msg=["FINAL_ROBOT_SPAWN_X=", robot_spawn_x])
    final_spawn_y_log = LogInfo(msg=["FINAL_ROBOT_SPAWN_Y=", robot_spawn_y])
    final_spawn_yaw_log = LogInfo(
        msg=["FINAL_ROBOT_SPAWN_YAW=", robot_spawn_yaw]
    )

    controller_bootstrap = Node(
        package="lab_cobot_gazebo",
        executable="controller_bootstrap",
        output="screen",
    )
    delay_controller_bootstrap = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[controller_bootstrap],
        )
    )

    actor_collision_shadow = Node(
        package="lab_cobot_gazebo",
        executable="actor_collision_shadow",
        name="actor_collision_shadow",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("enable_actor")),
    )
    obstacle_avoidance_metrics = Node(
        package="lab_cobot_gazebo",
        executable="obstacle_avoidance_metrics",
        name="obstacle_avoidance_metrics",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("enable_actor")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="是否显示 Gazebo GUI"),
        DeclareLaunchArgument("lighting_profile", default_value="normal"),
        DeclareLaunchArgument("enable_actor", default_value="false"),
        DeclareLaunchArgument("require_finger_contact", default_value="true"),
        DeclareLaunchArgument("enable_contact_force", default_value="false"),
        DeclareLaunchArgument("enable_lab_sensors", default_value="true"),
        DeclareLaunchArgument("use_refine_detect", default_value="false"),
        DeclareLaunchArgument("use_wrist_detect", default_value="false"),
        DeclareLaunchArgument("robot_spawn_x", default_value="4.50"),
        DeclareLaunchArgument("robot_spawn_y", default_value="-4.20"),
        DeclareLaunchArgument("robot_spawn_yaw", default_value="0.0"),
        DeclareLaunchArgument("tube_insert_fixed_base", default_value="false"),
        gazebo_resources,
        gazebo_builtin_models,
        description_package_models,
        gazebo_offline,
        model_path,
        gazebo_plugin_path,
        gzserver,
        gzclient,
        robot_state_publisher,
        final_spawn_x_log,
        final_spawn_y_log,
        final_spawn_yaw_log,
        spawn_entity,
        actor_collision_shadow,
        obstacle_avoidance_metrics,
        delay_controller_bootstrap,
    ])
