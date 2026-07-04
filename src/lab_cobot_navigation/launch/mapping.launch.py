r"""
SLAM 建图(slam_toolbox async + EKF),用于在 lab.world 重建实验室地图.

配合 Gazebo 使用(四个终端):
    终端1: ros2 launch lab_cobot_gazebo world.launch.py
    终端2: ros2 launch lab_cobot_navigation mapping.launch.py
    终端3: ros2 run teleop_twist_keyboard teleop_twist_keyboard
           (遥控机器人在场景里转一圈,把四周墙/工位扫全)
    终端4(建好后保存,覆盖占位地图):
           ros2 run nav2_map_server map_saver_cli -f \\
             ~/projects/lab_cobot_ws/src/lab_cobot_navigation/maps/map

TF 链: map --(slam_toolbox)--> odom --(EKF)--> base_footprint --> base_link ...
默认带 RViz(Fixed Frame=map, 显示 /map + /scan + 机器人),实时看地图成形。
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg = get_package_share_directory("lab_cobot_navigation")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = os.path.join(nav_pkg, "config", "mapping.rviz")

    # EKF: /odom(planar_move) + /imu/data -> odom->base_footprint TF
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

    # slam_toolbox: /scan + odom -> map->odom TF + /map
    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            os.path.join(nav_pkg, "config", "mapping.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_mapping",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        ekf,
        slam,
        rviz,
    ])
