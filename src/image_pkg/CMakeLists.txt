cmake_minimum_required(VERSION 3.8)
project(image_pkg)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS image_pkg/yolo_world_node.py
  DESTINATION lib/${PROJECT_NAME} RENAME yolo_world_node)
install(PROGRAMS image_pkg/yolo_pointcloud_pose_node.py
  DESTINATION lib/${PROJECT_NAME} RENAME yolo_pointcloud_pose_node)
install(PROGRAMS image_pkg/lighting_benchmark_node.py
  DESTINATION lib/${PROJECT_NAME} RENAME lighting_benchmark)
install(PROGRAMS image_pkg/rgbd_pointcloud_node.py
  DESTINATION lib/${PROJECT_NAME} RENAME rgbd_pointcloud_node)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
