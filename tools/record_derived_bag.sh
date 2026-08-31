#!/usr/bin/env bash
# Record a compact derived bag holding only what dynamic_obstacle_predictor
# needs: the ground-segmented obstacle cloud plus the TF/odom/clock needed to
# transform it into the odom frame. Replaying this is far faster than the
# 5.4 GB raw bag, so predictor changes can be iterated on quickly.
set -e

SRC_BAG=${1:-/home/adam/ws/rosbags/jk0009}
OUT_BAG=${2:-/home/adam/ws/rosbags/derived_jk0009}
DURATION=${3:-320}

source /opt/ros/humble/setup.bash
source /home/adam/ws/install/setup.bash

rm -rf "$OUT_BAG"

ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p use_sim_time:=true \
  -p robot_description:="$(xacro /home/adam/ws/src/mrvk_description/urdf/robot.urdf.xacro)" \
  > /tmp/rec_rsp.log 2>&1 &
RSP_PID=$!

ros2 run ground_segmentation_ros2 ground_segmentation_ros2_node --ros-args \
  --params-file /home/adam/ws/src/mrvk_gazebo/config/ground_segmentation/ground_segmentation_params.yaml \
  -p use_sim_time:=true \
  -r /ground_segmentation/input_pointcloud:=/cloud_registered_body \
  > /tmp/rec_gseg.log 2>&1 &
GSEG_PID=$!

sleep 3

ros2 bag record -o "$OUT_BAG" \
  /ground_segmentation/obstacle_points /ground_segmentation/ground_points \
  /tf /tf_static /odom /clock \
  > /tmp/rec_bagrec.log 2>&1 &
REC_PID=$!

sleep 2

ros2 bag play "$SRC_BAG" --clock --disable-keyboard-controls \
  --qos-profile-overrides-path "$SRC_BAG/qos_overrides.yaml" \
  > /tmp/rec_bagplay.log 2>&1 &
PLAY_PID=$!

# Playback of this bag is bursty; give it generous wall-clock headroom.
SECONDS=0
while kill -0 $PLAY_PID 2>/dev/null && [ $SECONDS -lt "$DURATION" ]; do
  sleep 5
done

kill -INT $PLAY_PID 2>/dev/null || true
sleep 3
kill -INT $REC_PID 2>/dev/null || true
sleep 3
kill -INT $GSEG_PID $RSP_PID 2>/dev/null || true
sleep 2
kill -9 $PLAY_PID $REC_PID $GSEG_PID $RSP_PID 2>/dev/null || true

echo "recorded to $OUT_BAG"
