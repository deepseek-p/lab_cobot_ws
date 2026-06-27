from setuptools import find_packages, setup

package_name = 'lab_cobot_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='THW',
    maintainer_email='2188630464@qq.com',
    description='感知:ArUco 检测与 6D 位姿估计(针孔反投影,复用 pose_math)',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'aruco_detector = lab_cobot_perception.aruco_detector:main',
        ],
    },
)
