cmake_minimum_required(VERSION 3.8)
project(imu_quat_to_eul)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EulRot.msg"
  DEPENDENCIES builtin_interfaces
)

# Python nodes
install(PROGRAMS
  scripts/imu_quat_to_eul_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
