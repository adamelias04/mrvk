cmake_minimum_required(VERSION 3.8)
project(gazebo_nav_goal_gui)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo REQUIRED)

# Gazebo Classic GUI plugins use Qt5.
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
set(CMAKE_AUTOMOC ON)

link_directories(${GAZEBO_LIBRARY_DIRS})

# X/Y/Yaw form panel overlay.
add_library(gazebo_nav_goal_gui SHARED src/gazebo_nav_goal_gui.cpp)
target_include_directories(gazebo_nav_goal_gui PUBLIC
  ${GAZEBO_INCLUDE_DIRS}
)
target_link_libraries(gazebo_nav_goal_gui
  ${GAZEBO_LIBRARIES}
  Qt5::Core
  Qt5::Widgets
)
ament_target_dependencies(gazebo_nav_goal_gui
  rclcpp
  geometry_msgs
  tf2
)

# Point-and-click "2D Nav Goal" tool (click/click-drag on the 3D view).
add_library(gazebo_goal_tool SHARED src/gazebo_goal_tool.cc)
target_include_directories(gazebo_goal_tool PUBLIC
  include
  ${GAZEBO_INCLUDE_DIRS}
)
target_link_libraries(gazebo_goal_tool
  ${GAZEBO_LIBRARIES}
  Qt5::Core
  Qt5::Widgets
)
ament_target_dependencies(gazebo_goal_tool
  rclcpp
  geometry_msgs
)

install(TARGETS gazebo_nav_goal_gui gazebo_goal_tool
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
