cmake_minimum_required(VERSION 3.8)
project(detected_path_layer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(path_detection REQUIRED)

add_library(detected_path_layer SHARED
  src/detected_path_layer.cpp
)
target_include_directories(detected_path_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(detected_path_layer
  rclcpp
  nav2_costmap_2d
  pluginlib
  path_detection
)

pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

install(TARGETS detected_path_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
install(FILES costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(detected_path_layer)
ament_package()
