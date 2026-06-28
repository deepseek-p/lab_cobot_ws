"""启动 Gazebo Classic 实验室场景并 spawn 一体化麦轮机器人 + 控制器。

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
    IncludeLaunchDescription,
    RegisterEventHandler,
    AppendEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    desc_pkg = get_package_share_directory("lab_cobot_description")
    gz_pkg = get_package_share_directory("lab_cobot_gazebo")
    gazebo_ros = get_package_share_directory("gazebo_ros")

    world = os.path.join(gz_pkg, "worlds", "lab.world")
    urdf_xacro = os.path.join(desc_pkg, "urdf", "lab_cobot.urdf.xacro")
    robot_description = {"robot_description": Command(["xacro ", urdf_xacro])}

    gui = LaunchConfiguration("gui")

    # 让 Gazebo 能解析 world 里的 model://aruco_sample
    model_path = AppendEnvironmentVariable(
        "GAZEBO_MODEL_PATH", os.path.join(gz_pkg, "models")
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": world, "verbose": "true", "gui": gui}.items(),
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
            "-x", "0.0", "-y", "0.0", "-z", "0.12",
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

    # spawn 完成后顺序激活控制器
    delay_jsb = RegisterEventHandler(
        OnProcessExit(target_action=spawn_entity, on_exit=[joint_state_broadcaster])
    )
    delay_jtc = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster, on_exit=[joint_trajectory_controller]
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="是否显示 Gazebo GUI"),
        model_path,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        delay_jsb,
        delay_jtc,
    ])
