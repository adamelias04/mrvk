cmake_minimum_required(VERSION 2.8.3)
project(mrvk_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  gazebo_msgs
  visualization_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  costmap_2d
  pluginlib
  actionlib
  move_base_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES predicted_path_obstacle_layer
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    nav_msgs
    gazebo_msgs
    visualization_msgs
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    costmap_2d
    pluginlib
    actionlib
    move_base_msgs
    std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(dynamic_obstacle_predictor
  src/dynamic_obstacle_predictor.cpp
)

add_executable(robot_motion_predictor
  src/robot_motion_predictor.cpp
)

add_executable(path_recorder
  src/path_recorder.cpp
)

add_executable(waypoint_tour
  src/waypoint_tour.cpp
)

add_executable(goal_pingpong
  src/goal_pingpong.cpp
)

target_link_libraries(dynamic_obstacle_predictor
  ${catkin_LIBRARIES}
)

target_link_libraries(robot_motion_predictor
  ${catkin_LIBRARIES}
)


target_link_libraries(path_recorder
  ${catkin_LIBRARIES}
)

target_link_libraries(waypoint_tour
  ${catkin_LIBRARIES}
)

target_link_libraries(goal_pingpong
  ${catkin_LIBRARIES}
)

add_dependencies(dynamic_obstacle_predictor
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(robot_motion_predictor
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)


add_dependencies(path_recorder
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(waypoint_tour
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(goal_pingpong
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_library(predicted_path_obstacle_layer
  src/predicted_path_obstacle_layer.cpp
)

target_link_libraries(predicted_path_obstacle_layer
  ${catkin_LIBRARIES}
)

add_dependencies(predicted_path_obstacle_layer
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS
  dynamic_obstacle_predictor
  robot_motion_predictor
  path_recorder
  waypoint_tour
  goal_pingpong
  predicted_path_obstacle_layer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES
  predicted_path_obstacle_layer.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/nav_metrics.py
  scripts/trajectory_recorder.py
  scripts/compare_trajectories.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
