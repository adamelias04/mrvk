cmake_minimum_required(VERSION 3.8)
project(path_detection)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedPath.msg"
)

install(PROGRAMS
  scripts/path_detection_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
