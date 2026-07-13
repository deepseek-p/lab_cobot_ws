cmake_minimum_required(VERSION 3.8)
project(lab_cobot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

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
ament_target_dependencies(lab_cobot_mecanum_drive
  gazebo_ros
  nav_msgs
  rclcpp
  std_msgs
)
target_link_libraries(lab_cobot_mecanum_drive
  ${GAZEBO_LIBRARIES}
)

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
  ament_add_gtest(
    test_attached_object_collision
    test/test_attached_object_collision.cpp)
  target_include_directories(test_attached_object_collision PRIVATE include)
  ament_add_gtest(
    test_post_attach_velocity_clamp
    test/test_post_attach_velocity_clamp.cpp)
  target_include_directories(test_post_attach_velocity_clamp PRIVATE include)
  # 本机 ~/.local 的 anyio pytest 插件与系统 pytest 不兼容(装 DL 依赖后
  # 版本漂移引爆),所有 pytest 注册必须带 -p no:anyio 防护(bringup 范本)
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
endif()

install(DIRECTORY worlds launch models
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  lab_cobot_grasp_fix
  lab_cobot_mecanum_drive
  LIBRARY DESTINATION lib
)

ament_export_libraries(lab_cobot_grasp_fix lab_cobot_mecanum_drive)

ament_package()
