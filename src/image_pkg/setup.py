from setuptools import find_packages, setup


package_name = "image_pkg"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/pose_estimation.launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="THW",
    maintainer_email="2188630464@qq.com",
    description="Eight-class YOLO and organized RGB-D object localization.",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "rgbd_pointcloud_node = image_pkg.rgbd_pointcloud_node:main",
            "yolo_world_node = image_pkg.yolo_world_node:main",
            (
                "yolo_pointcloud_pose_node = "
                "image_pkg.yolo_pointcloud_pose_node:main"
            ),
        ],
    },
)
