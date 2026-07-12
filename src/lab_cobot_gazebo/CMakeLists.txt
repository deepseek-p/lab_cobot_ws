cmake_minimum_required(VERSION 3.8)
project(lab_cobot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(include)

add_library(lab_cobot_grasp_fix SHARED
  src/lab_cobot_grasp_fix.cpp
)
target_link_libraries(lab_cobot_grasp_fix
  ${GAZEBO_LIBRARIES}
)
ament_target_dependencies(lab_cobot_grasp_fix
  gazebo_ros
  rclcpp
  std_msgs
)

add_library(lab_cobot_mecanum_drive SHARED
  src/lab_cobot_mecanum_drive.cpp
)

add_library(lab_cobot_planar_drive SHARED
  src/lab_cobot_planar_drive.cpp
)
ament_target_dependencies(lab_cobot_planar_drive
  gazebo_ros
  rclcpp
  std_msgs
)
target_link_libraries(lab_cobot_planar_drive
  ${GAZEBO_LIBRARIES}
)
ament_target_dependencies(lab_cobot_mecanum_drive
  gazebo_ros
  nav_msgs
  rclcpp
  std_msgs
)
target_link_libraries(lab_cobot_mecanum_drive
  ${GAZEBO_LIBRARIES}
)

add_executable(mecanum_gazebo_kinematic_drive src/mecanum_gazebo_kinematic_drive.cpp)
ament_target_dependencies(mecanum_gazebo_kinematic_drive
  gazebo_msgs geometry_msgs rclcpp std_msgs tf2)

add_executable(gazebo_odom_bridge src/gazebo_odom_bridge.cpp)
ament_target_dependencies(gazebo_odom_bridge
  gazebo_msgs geometry_msgs nav_msgs rclcpp tf2 tf2_ros)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  # C++ 插件核心逻辑的行为级单元测试(纯函数头文件,无需起 Gazebo)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mecanum_kinematics test/test_mecanum_kinematics.cpp)
  target_include_directories(test_mecanum_kinematics PRIVATE include)
  ament_add_gtest(test_grasp_envelope test/test_grasp_envelope.cpp)
  target_include_directories(test_grasp_envelope PRIVATE include
    ${GAZEBO_INCLUDE_DIRS})
  ament_add_gtest(test_finger_contact_gate test/test_finger_contact_gate.cpp)
  target_include_directories(test_finger_contact_gate PRIVATE include)
  ament_add_gtest(test_runtime_motion test/test_runtime_motion.cpp)
  target_include_directories(test_runtime_motion PRIVATE include)
  # 本机 ~/.local 的 anyio pytest 插件与系统 pytest 不兼容(装 DL 依赖后
  # 版本漂移引爆),所有 pytest 注册必须带 -p no:anyio 防护(bringup 范本)
  ament_add_gtest(test_pose_differentiator test/test_pose_differentiator.cpp)
  target_include_directories(test_pose_differentiator PRIVATE include)
  ament_add_pytest_test(world_launch test/test_world_launch.py
    ENV "PYTEST_ADDOPTS=-p no:anyio")
  ament_add_pytest_test(aruco_sample_model test/test_aruco_sample_model.py
    ENV "PYTEST_ADDOPTS=-p no:anyio")
  ament_add_pytest_test(dl_perception_models test/test_dl_perception_models.py
    ENV "PYTEST_ADDOPTS=-p no:anyio")
  ament_add_pytest_test(grasp_fix_plugin test/test_grasp_fix_plugin.py
    ENV "PYTEST_ADDOPTS=-p no:anyio")
  ament_add_pytest_test(mecanum_drive_plugin test/test_mecanum_drive_plugin.py
    ENV "PYTEST_ADDOPTS=-p no:anyio")
  ament_add_pytest_test(mecanum_runtime_contract test/test_mecanum_runtime_contract.py
    ENV "PYTEST_ADDOPTS=-p no:anyio")
endif()

install(DIRECTORY worlds launch models
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  lab_cobot_grasp_fix
  lab_cobot_mecanum_drive
  lab_cobot_planar_drive
  LIBRARY DESTINATION lib
)

install(TARGETS
  mecanum_gazebo_kinematic_drive
  gazebo_odom_bridge
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(
  lab_cobot_grasp_fix
  lab_cobot_mecanum_drive
  lab_cobot_planar_drive
)

ament_package()
