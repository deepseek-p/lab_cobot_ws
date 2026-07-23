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
    ],
    install_requires=["setuptools", "ultralytics", "opencv-python", "numpy"],
    zip_safe=True,
    maintainer="xqq",
    maintainer_email="xqq@todo.todo",
    description="YOLO-World and RGB-D pose estimation",
    license="TODO",
    entry_points={
        "console_scripts": [
            "yolo_world_node = image_pkg.yolo_world_node:main",
            "yolo_pointcloud_pose_node = image_pkg.yolo_pointcloud_pose_node:main",
            "lighting_benchmark = image_pkg.lighting_benchmark_node:main",
            "rgbd_pointcloud_node = image_pkg.rgbd_pointcloud_node:main",
        ],
    },
)
