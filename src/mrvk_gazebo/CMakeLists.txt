cmake_minimum_required(VERSION 3.8)
project(mrvk_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)

set(node_deps
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
)

# ---- executables ----
add_executable(dynamic_obstacle_predictor src/dynamic_obstacle_predictor.cpp)
ament_target_dependencies(dynamic_obstacle_predictor ${node_deps})

add_executable(robot_motion_predictor src/robot_motion_predictor.cpp)
ament_target_dependencies(robot_motion_predictor ${node_deps})

add_executable(path_recorder src/path_recorder.cpp)
ament_target_dependencies(path_recorder ${node_deps})

add_executable(waypoint_tour src/waypoint_tour.cpp)
ament_target_dependencies(waypoint_tour ${node_deps})

add_executable(goal_pingpong src/goal_pingpong.cpp)
ament_target_dependencies(goal_pingpong ${node_deps})

install(TARGETS
  dynamic_obstacle_predictor
  robot_motion_predictor
  path_recorder
  waypoint_tour
  goal_pingpong
  DESTINATION lib/${PROJECT_NAME}
)

# ---- Nav2 costmap layer plugin ----
add_library(predicted_path_obstacle_layer SHARED
  src/predicted_path_obstacle_layer.cpp
)
target_include_directories(predicted_path_obstacle_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(predicted_path_obstacle_layer
  rclcpp
  nav_msgs
  nav2_costmap_2d
  pluginlib
)
pluginlib_export_plugin_description_file(nav2_costmap_2d predicted_path_obstacle_layer.xml)

install(TARGETS predicted_path_obstacle_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

# ---- python metric scripts ----
install(PROGRAMS
  scripts/nav_metrics.py
  scripts/trajectory_recorder.py
  scripts/compare_trajectories.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---- data ----
install(DIRECTORY config launch worlds maps
  DESTINATION share/${PROJECT_NAME}
)
install(FILES predicted_path_obstacle_layer.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(predicted_path_obstacle_layer)
ament_package()
