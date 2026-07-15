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
  find_package(ament_cmake_pytest REQUIRED)
  # 常规单测与诚实 E2E 分开注册:E2E 自身预算约 500s(任务超时 420s +
  # 服务等待 80s),混在一起时 360s 总超时装不下,失败原因会失真。
  ament_add_pytest_test(${PROJECT_NAME}_pytest test/
    ENV "PYTEST_ADDOPTS=-p no:anyio --ignore=${CMAKE_CURRENT_SOURCE_DIR}/test/test_honest_e2e_launch.py"
    TIMEOUT 120
  )
  ament_add_pytest_test(${PROJECT_NAME}_honest_e2e
    test/test_honest_e2e_launch.py
    ENV "PYTEST_ADDOPTS=-p no:anyio"
    TIMEOUT 600
  )
endif()

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/mission_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mission_node
)

install(PROGRAMS
  ${PROJECT_NAME}/mecanum_wheel_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mecanum_wheel_visualizer
)

install(PROGRAMS
  ${PROJECT_NAME}/rover_twist_relay.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rover_twist_relay
)

install(PROGRAMS
  ${PROJECT_NAME}/passive_mecanum_joint_states.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME passive_mecanum_joint_states
)

install(PROGRAMS
  ${PROJECT_NAME}/gripper_attach_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gripper_attach_bridge
)

install(PROGRAMS
  ${PROJECT_NAME}/voice_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME voice_node
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
