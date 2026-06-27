"""查看一体化机器人模型(RViz + joint_state_publisher_gui)。

用法(需 GUI):
    ros2 launch lab_cobot_description view_robot.launch.py
打开后在 RViz 添加 RobotModel display(Description Topic 选 /robot_description)、
TF、Grid 即可看到麦轮底盘 + 立柱 + UR5e + 吸盘 + 传感器。
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("lab_cobot_description")
    urdf_xacro = os.path.join(pkg, "urdf", "lab_cobot.urdf.xacro")
    robot_description = {"robot_description": Command(["xacro ", urdf_xacro])}

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description, {"use_sim_time": False}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
        ),
    ])
