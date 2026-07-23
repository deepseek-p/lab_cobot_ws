import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("image_pkg")
    config = os.path.join(package_share, "config", "pose_estimation.yaml")
    rviz_config = os.path.join(package_share, "config", "camera_visualization.rviz")
    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Launch RViz with the camera RGB-D point cloud display.",
        ),
        Node(
            package="image_pkg",
            executable="rgbd_pointcloud_node",
            name="rgbd_pointcloud",
            output="screen",
            parameters=[config],
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package="image_pkg",
            executable="yolo_world_node",
            name="yolo_world_detection",
            output="screen",
            parameters=[config],
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package="image_pkg",
            executable="yolo_pointcloud_pose_node",
            name="yolo_pointcloud_pose",
            output="screen",
            parameters=[config],
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="image_visualization_rviz",
            arguments=["-d", rviz_config],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="screen",
        ),
    ])
