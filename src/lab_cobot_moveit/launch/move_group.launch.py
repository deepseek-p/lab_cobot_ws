"""
Launch MoveIt2 move_group for the lab_cobot UR5e planning group.

需运行时(配合 Gazebo + joint_trajectory_controller):
    ros2 launch lab_cobot_moveit move_group.launch.py
URDF/SRDF 取自 lab_cobot_description;controller 走 follow_joint_trajectory(ur_ 前缀)。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    desc_share = get_package_share_directory("lab_cobot_description")

    moveit_config = (
        MoveItConfigsBuilder("lab_cobot", package_name="lab_cobot_moveit")
        .robot_description(
            file_path=os.path.join(desc_share, "urdf", "lab_cobot.urdf.xacro")
        )
        .robot_description_semantic(
            file_path=os.path.join(desc_share, "srdf", "lab_cobot.srdf")
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        move_group_node,
    ])
