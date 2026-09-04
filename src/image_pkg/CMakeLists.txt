cmake_minimum_required(VERSION 3.8)
project(image_pkg)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS image_pkg/yolo_world_node.py
  DESTINATION lib/${PROJECT_NAME} RENAME yolo_world_node)
install(PROGRAMS image_pkg/yolo_pointcloud_pose_node.py
  DESTINATION lib/${PROJECT_NAME} RENAME yolo_pointcloud_pose_node)
install(PROGRAMS image_pkg/dual_camera_fusion_node.py
  DESTINATION lib/${PROJECT_NAME} RENAME dual_camera_fusion)
install(PROGRAMS image_pkg/lighting_benchmark_node.py
  DESTINATION lib/${PROJECT_NAME} RENAME lighting_benchmark)
install(PROGRAMS image_pkg/station_cruise_node.py
  DESTINATION lib/${PROJECT_NAME} RENAME station_cruise)
install(PROGRAMS image_pkg/indicator_state_node.py
  DESTINATION lib/${PROJECT_NAME} RENAME indicator_state)
install(PROGRAMS image_pkg/rgbd_pointcloud_node.py
  DESTINATION lib/${PROJECT_NAME} RENAME rgbd_pointcloud_node)
install(PROGRAMS image_pkg/tcp_precision_benchmark_node.py
  DESTINATION lib/${PROJECT_NAME} RENAME tcp_precision_benchmark)
install(PROGRAMS image_pkg/camera_base_residual_calibrator_node.py
  DESTINATION lib/${PROJECT_NAME} RENAME camera_base_residual_calibrator)
install(PROGRAMS image_pkg/eye_to_hand_checkerboard_calibrator_node.py
  DESTINATION lib/${PROJECT_NAME} RENAME eye_to_hand_checkerboard_calibrator)
install(PROGRAMS image_pkg/wrist_hand_eye_validator_node.py
  DESTINATION lib/${PROJECT_NAME} RENAME wrist_hand_eye_validator)
install(PROGRAMS image_pkg/wrist_observation_node.py
  DESTINATION lib/${PROJECT_NAME} RENAME wrist_observation)
install(PROGRAMS image_pkg/truth_guided_wrist_observer.py
  DESTINATION lib/${PROJECT_NAME} RENAME truth_guided_wrist_observer)

install(DIRECTORY launch config models DESTINATION share/${PROJECT_NAME})

ament_package()
