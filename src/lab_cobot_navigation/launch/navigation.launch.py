"""
Nav2 导航栈:map_server + AMCL + EKF + 规划/控制.

需运行时(配合 Gazebo + 机器人 spawn):
    ros2 launch lab_cobot_navigation navigation.launch.py
默认用 maps/map.yaml,该图由 slam_toolbox 在 lab.world 中保存生成。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    nav_pkg = get_package_share_directory("lab_cobot_navigation")
    nav2_bringup = get_package_share_directory("nav2_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")
    params = LaunchConfiguration("params_file")
    nav2_log_level = "info"
    nav2_autostart = True
    nav2_lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "velocity_smoother",
    ]
    nav2_remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    nav2_params = ParameterFile(
        RewrittenYaml(
            source_file=params,
            param_rewrites={
                "use_sim_time": use_sim_time,
                "autostart": "true",
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    declared = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "map", default_value=os.path.join(nav_pkg, "maps", "map.yaml")
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(nav_pkg, "config", "nav2_params.yaml"),
        ),
        DeclareLaunchArgument("use_rviz", default_value="true"),
    ]

    # EKF: 融合 /odom(底盘插件位姿积分模型) + /imu/data,发布 odom->base_footprint TF
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(nav_pkg, "config", "ekf.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    # 定位:map_server + AMCL
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, "launch", "localization_launch.py")
        ),
        launch_arguments={
            "map": map_yaml,
            "use_sim_time": use_sim_time,
            "params_file": params,
        }.items(),
    )

    # 导航:planner + controller + bt_navigator + behaviors.
    # 本任务不使用 FollowWaypoints; 不启动 waypoint_follower,避免无关生命周期
    # 服务竞态拖垮整个导航栈。
    controller = Node(
        package="nav2_controller",
        executable="controller_server",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[nav2_params],
        arguments=["--ros-args", "--log-level", nav2_log_level],
        remappings=nav2_remappings + [("cmd_vel", "cmd_vel_nav")],
    )
    smoother = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[nav2_params],
        arguments=["--ros-args", "--log-level", nav2_log_level],
        remappings=nav2_remappings,
    )
    planner = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[nav2_params],
        arguments=["--ros-args", "--log-level", nav2_log_level],
        remappings=nav2_remappings,
    )
    behavior = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[nav2_params],
        arguments=["--ros-args", "--log-level", nav2_log_level],
        remappings=nav2_remappings,
    )
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[nav2_params],
        arguments=["--ros-args", "--log-level", nav2_log_level],
        remappings=nav2_remappings,
    )
    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[nav2_params],
        arguments=["--ros-args", "--log-level", nav2_log_level],
        remappings=nav2_remappings
        + [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
    )
    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        # 2026-07-10 GUI 演示实测:gzclient 高负载下 bond 默认 4s 超时,
        # lifecycle_manager 编排失败即挂死整栈(bt_navigator 停在 unconfigured)。
        # 放宽 bond 超时并开启 respawn 重连,节点侧 respawn=True 配套自愈。
        respawn=True,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", nav2_log_level],
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": nav2_autostart},
            {"node_names": nav2_lifecycle_nodes},
            {"bond_timeout": 10.0},
            {"attempt_respawn_reconnection": True},
            {"bond_respawn_max_duration": 30.0},
        ],
    )
    stdout_linebuf = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM",
        "1",
    )

    # 2026-07-10 GUI 实测根因:manager 在 launch 后 ~1s 即发起 configure,
    # 此时 gzserver 仍在加载(gzclient 抢资源),首次 change_state 服务调用 20ms 内
    # 失败,nav2 humble 的 manager 对此零重试直接弃栈(bt_navigator 永远 unconfigured)。
    # 延后 manager 启动,等全部 lifecycle 节点服务稳定后再编排,消灭竞态窗口。
    lifecycle_delayed = TimerAction(period=15.0, actions=[lifecycle])

    navigation = [
        stdout_linebuf,
        controller,
        smoother,
        planner,
        behavior,
        bt_navigator,
        velocity_smoother,
        lifecycle_delayed,
    ]

    # 导航可视化(nav2 默认视图:map/costmap/path/robot + 2D Goal Pose 工具)
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_navigation",
        output="screen",
        arguments=["-d", os.path.join(nav2_bringup, "rviz", "nav2_default_view.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(declared + [ekf, localization] + navigation + [rviz])
