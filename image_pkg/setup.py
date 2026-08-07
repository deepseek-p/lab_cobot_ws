from setuptools import find_packages, setup


package_name = "image_pkg"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/pose_estimation.launch.py"]),
        ("share/" + package_name + "/config", [
            "config/pose_estimation.yaml",
            "config/camera_visualization.rviz",
        ]),
        ("share/" + package_name + "/models", ["models/best.pt"]),
    ],
    install_requires=["setuptools", "ultralytics", "opencv-python", "numpy"],
    zip_safe=True,
    maintainer="xqq",
    maintainer_email="xqq@todo.todo",
    description="Trained YOLO and RGB-D pose estimation",
    license="TODO",
    entry_points={
        "console_scripts": [
            "yolo_world_node = image_pkg.yolo_world_node:main",
            "yolo_pointcloud_pose_node = image_pkg.yolo_pointcloud_pose_node:main",
            "lighting_benchmark = image_pkg.lighting_benchmark_node:main",
            "station_cruise = image_pkg.station_cruise_node:main",
            "indicator_state = image_pkg.indicator_state_node:main",
            "rgbd_pointcloud_node = image_pkg.rgbd_pointcloud_node:main",
            "tcp_precision_benchmark = image_pkg.tcp_precision_benchmark_node:main",
            "camera_base_residual_calibrator = image_pkg.camera_base_residual_calibrator_node:main",
            "eye_to_hand_checkerboard_calibrator = image_pkg.eye_to_hand_checkerboard_calibrator_node:main",
            "wrist_observation = image_pkg.wrist_observation_node:main",
            "truth_guided_wrist_observer = image_pkg.truth_guided_wrist_observer:main",
        ],
    },
)
