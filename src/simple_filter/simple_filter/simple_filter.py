#!/usr/bin/env python3
# ROS2 port of the naive complementary filter that averages IMU and
# odometry orientation quaternions and republishes on /odom_filtered.
# NOTE: the ROS1 original had attribute typos (self.odom_filtered_pub and
# data.orientation on an Odometry msg); those are corrected here so the
# node actually runs.

import copy

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class SimpleFilter(Node):
    def __init__(self):
        super().__init__('simple_filter')
        self.imu_data = None
        self.odom_filtered_pub = self.create_publisher(Odometry, '/odom_filtered', 10)
        self.create_subscription(Imu, '/imu_data', self.imu_callback, 10)
        self.create_subscription(Odometry, '/shoddy/odom', self.odometry_callback, 10)

    def imu_callback(self, data):
        self.imu_data = data

    def odometry_callback(self, data):
        if self.imu_data is None:
            return
        filtered = copy.deepcopy(data)
        o = filtered.pose.pose.orientation
        io = self.imu_data.orientation
        o.x = (data.pose.pose.orientation.x + io.x) / 2.0
        o.y = (data.pose.pose.orientation.y + io.y) / 2.0
        o.z = (data.pose.pose.orientation.z + io.z) / 2.0
        o.w = (data.pose.pose.orientation.w + io.w) / 2.0
        self.odom_filtered_pub.publish(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
