cmake_minimum_required(VERSION 3.8)
project(lab_cobot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/mission_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mission_node
)

install(PROGRAMS
  ${PROJECT_NAME}/wheel_joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wheel_joint_state_publisher
)

install(PROGRAMS
  ${PROJECT_NAME}/mecanum_wheel_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mecanum_wheel_visualizer
)

install(PROGRAMS
  ${PROJECT_NAME}/gripper_attach_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gripper_attach_bridge
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
