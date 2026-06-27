"""一键启动跨工位抓取全栈(集成层)。

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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gz = get_package_share_directory("lab_cobot_gazebo")
    moveit = get_package_share_directory("lab_cobot_moveit")
    nav = get_package_share_directory("lab_cobot_navigation")

    gui = LaunchConfiguration("gui")
    launch_mission = LaunchConfiguration("launch_mission")

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz, "launch", "world.launch.py")),
        launch_arguments={"gui": gui}.items(),
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
        launch_arguments={"use_sim_time": "true"}.items(),
    )
    aruco = Node(
        package="lab_cobot_perception",
        executable="aruco_detector",
        name="aruco_detector",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    mission = Node(
        package="lab_cobot_bringup",
        executable="mission_node",
        name="mission_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # 等 Gazebo + spawn + 控制器起来后再起规划/导航/感知
    stage2 = TimerAction(period=10.0, actions=[move_group, navigation, aruco])
    # 再等编排依赖就绪
    stage3 = TimerAction(period=15.0, actions=[mission])

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="Gazebo GUI"),
        DeclareLaunchArgument("launch_mission", default_value="true"),
        # WSLg 稳定渲染(源自 robot_lab_demo 验证经验)
        SetEnvironmentVariable("GALLIUM_DRIVER", "d3d12"),
        SetEnvironmentVariable("MESA_D3D12_DEFAULT_ADAPTER_NAME", "NVIDIA"),
        SetEnvironmentVariable("QT_X11_NO_MITSHM", "1"),
        world,
        stage2,
        stage3,
    ])
