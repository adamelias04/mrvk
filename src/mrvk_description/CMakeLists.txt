cmake_minimum_required(VERSION 3.8)
project(mrvk_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes launch
  DESTINATION share/${PROJECT_NAME}
)

# Expose the meshes as a self-contained Gazebo model so that
# model://mrvk_description/meshes/... (converted from package://) resolves,
# while GAZEBO_MODEL_PATH points only at this clean models/ dir (avoids the
# "Missing model.config" scan errors from the ROS share directory).
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME}/models/mrvk_description)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
