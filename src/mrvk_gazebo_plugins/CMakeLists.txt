cmake_minimum_required(VERSION 3.8)
project(mrvk_gazebo_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(oscillate_box_plugin SHARED src/oscillate_box_plugin.cpp)
target_include_directories(oscillate_box_plugin PUBLIC ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(oscillate_box_plugin ${GAZEBO_LIBRARIES})

add_library(path_trail_visual_plugin SHARED src/path_trail_visual_plugin.cpp)
target_include_directories(path_trail_visual_plugin PUBLIC ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(path_trail_visual_plugin ${GAZEBO_LIBRARIES})

install(TARGETS
  oscillate_box_plugin
  path_trail_visual_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
