"""Launch the eight-class YOLO and organized RGB-D perception backend."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Return the standalone eight-class perception launch description."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    model_path = LaunchConfiguration("model_path")
    device = LaunchConfiguration("device")
    imgsz = LaunchConfiguration("imgsz")
    image_topic = LaunchConfiguration("image_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    optical_frame = LaunchConfiguration("optical_frame")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    target_frame = LaunchConfiguration("target_frame")
    target_label = LaunchConfiguration("target_label")

    common_time = ParameterValue(use_sim_time, value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "model_path",
            default_value="~/lab_cobot_models/lab_cobot_eight_class.pt",
        ),
        DeclareLaunchArgument("device", default_value="auto"),
        DeclareLaunchArgument("imgsz", default_value="640"),
        DeclareLaunchArgument(
            "image_topic", default_value="/wrist_camera/image_raw"
        ),
        DeclareLaunchArgument(
            "depth_topic", default_value="/wrist_camera/depth/image_raw"
        ),
        DeclareLaunchArgument(
            "camera_info_topic", default_value="/wrist_camera/camera_info"
        ),
        DeclareLaunchArgument(
            "optical_frame", default_value="wrist_camera_optical_frame"
        ),
        DeclareLaunchArgument(
            "pointcloud_topic",
            default_value="/image_pkg/wrist_camera_points",
        ),
        DeclareLaunchArgument("target_frame", default_value="base_link"),
        DeclareLaunchArgument("target_label", default_value="aruco_sample"),
        Node(
            package="image_pkg",
            executable="rgbd_pointcloud_node",
            name="eight_class_rgbd_pointcloud",
            output="screen",
            parameters=[{
                "use_sim_time": common_time,
                "rgb_topic": image_topic,
                "depth_topic": depth_topic,
                "camera_info_topic": camera_info_topic,
                "pointcloud_topic": pointcloud_topic,
            }],
        ),
        Node(
            package="image_pkg",
            executable="yolo_world_node",
            name="eight_class_yolo_detector",
            output="screen",
            parameters=[{
                "use_sim_time": common_time,
                "image_topic": image_topic,
                "model_path": model_path,
                "device": device,
                "inference_imgsz": ParameterValue(imgsz, value_type=int),
            }],
        ),
        Node(
            package="image_pkg",
            executable="yolo_pointcloud_pose_node",
            name="eight_class_pose_estimator",
            output="screen",
            parameters=[{
                "use_sim_time": common_time,
                "pointcloud_topic": pointcloud_topic,
                "target_frame": target_frame,
                "target_label": target_label,
                "optical_frame": optical_frame,
            }],
        ),
    ])
