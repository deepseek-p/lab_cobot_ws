"""
Nav2 navigation launch stack.

需运行时(配合 Gazebo + 机器人 spawn):
    ros2 launch lab_cobot_navigation navigation.launch.py
默认用占位地图 maps/map.yaml(待自建实验室 world 后用 slam_toolbox 重建)。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg = get_package_share_directory("lab_cobot_navigation")
    nav2_bringup = get_package_share_directory("nav2_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")
    params = LaunchConfiguration("params_file")

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

    # EKF: 融合 /odom(planar_move) + /imu/data,发布 odom->base_footprint TF
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

    # 导航:planner + controller + bt_navigator + behaviors
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": params,
        }.items(),
    )

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

    return LaunchDescription(declared + [ekf, localization, navigation, rviz])
