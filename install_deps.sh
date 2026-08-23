#!/usr/bin/env bash
# System dependencies for the MRVK ROS2 (Humble) port.
# Run this once. Requires sudo.
set -e

sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui \
  ros-humble-rqt-robot-steering ros-humble-robot-localization \
  ros-humble-tf2-geometry-msgs ros-humble-rviz2 \
  ros-humble-nav2-map-server \
  ros-humble-libg2o \
  python3-opencv python3-numpy python3-psutil

# tf_transformations (used by imu_quat_to_eul python node)
sudo apt install -y ros-humble-tf-transformations || pip3 install transforms3d

echo "System dependencies installed."
