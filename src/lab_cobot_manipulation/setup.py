from setuptools import find_packages, setup

package_name = 'lab_cobot_manipulation'

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
    description='机械臂抓放执行(pymoveit2 运动规划 + 平行夹爪后端)',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'pick_place_node = lab_cobot_manipulation.pick_place_node:main',
        ],
    },
)
