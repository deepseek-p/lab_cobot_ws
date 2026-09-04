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
        DeclareLaunchArgument(
            "indicator_state",
            default_value="false",
            description="Enable colour-based device indicator recognition.",
        ),
        DeclareLaunchArgument(
            "wrist_aruco",
            default_value="true",
            description="Run calibrated wrist ArUco PnP refinement.",
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
            package="lab_cobot_perception",
            executable="aruco_detector",
            name="image_pkg_wrist_aruco_detector",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "topic_namespace": "/perception/wrist",
                "publish_tf": False,
                "rgb_topic": "/wrist_camera/image_raw",
                "depth_topic": "/wrist_camera/depth/image_raw",
                "info_topic": "/wrist_camera/camera_info",
                "optical_frame": "wrist_camera_optical_frame",
                "target_frame": "base_link",
                # The 312 px texture contains a 240 px coded marker inside a
                # 70 mm face, so solvePnP uses the coded 53.846 mm square.
                "marker_size_m": 0.07 * (240.0 / 312.0),
                "marker_to_object_center_m": 0.036,
                "translation_source": "pnp",
                "process_period_sec": 0.05,
            }],
            condition=IfCondition(LaunchConfiguration("wrist_aruco")),
        ),
        Node(
            package="image_pkg",
            executable="rgbd_pointcloud_node",
            name="bench_rgbd_pointcloud",
            output="screen",
            parameters=[config],
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package="image_pkg",
            executable="indicator_state",
            name="indicator_state",
            output="screen",
            parameters=[config],
            condition=IfCondition(LaunchConfiguration("indicator_state")),
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
            executable="yolo_world_node",
            name="bench_yolo_world_detection",
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
            package="image_pkg",
            executable="yolo_pointcloud_pose_node",
            name="bench_yolo_pointcloud_pose",
            output="screen",
            parameters=[config],
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package="image_pkg",
            executable="dual_camera_fusion",
            name="dual_camera_fusion",
            output="screen",
            parameters=[config],
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
