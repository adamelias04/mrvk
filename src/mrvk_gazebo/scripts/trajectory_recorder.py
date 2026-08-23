#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 port. Records the robot trajectory (from /shoddy/odom) to a CSV file.
One run = one CSV. The file is written on node shutdown (Ctrl+C / SIGTERM).

CSV format: t,x,y,yaw   (t in seconds from start of recording)

Params:
  odom_topic   (str,  "/shoddy/odom")
  output_path  (str,  "~/ws/trajectories/trajectory.csv")
  min_distance (float, 0.02)   min move between recorded points [m]
  tour_only    (bool, False)   if True, record only while /nav_tour/active
"""
import csv
import math
import os

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')
        self.odom_topic = self.declare_parameter('odom_topic', '/shoddy/odom').value
        default_out = os.path.join(os.path.expanduser('~'), 'ws', 'trajectories', 'trajectory.csv')
        self.output_path = os.path.expanduser(
            self.declare_parameter('output_path', default_out).value)
        self.min_distance = float(self.declare_parameter('min_distance', 0.02).value)
        self.tour_only = bool(self.declare_parameter('tour_only', False).value)

        self.points = []
        self.t0 = None
        self.last_xy = None
        self.tour_active = not self.tour_only

        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 50)
        if self.tour_only:
            self.create_subscription(Bool, '/nav_tour/active', self.tour_cb, 1)

        self.get_logger().info(
            '[trajectory_recorder] recording {} -> {} (min_dist={:.3f} m, tour_only={})'.format(
                self.odom_topic, self.output_path, self.min_distance, self.tour_only))

    def tour_cb(self, msg):
        self.tour_active = bool(msg.data)

    def odom_cb(self, msg):
        if not self.tour_active:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.last_xy is not None:
            dx = x - self.last_xy[0]
            dy = y - self.last_xy[1]
            if dx * dx + dy * dy < self.min_distance * self.min_distance:
                return
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.points.append((t - self.t0, x, y, yaw))
        self.last_xy = (x, y)

    def save(self):
        if not self.points:
            self.get_logger().warn('[trajectory_recorder] no points, not writing.')
            return
        os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
        with open(self.output_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y', 'yaw'])
            for row in self.points:
                w.writerow(['{:.3f}'.format(row[0]), '{:.4f}'.format(row[1]),
                            '{:.4f}'.format(row[2]), '{:.4f}'.format(row[3])])
        self.get_logger().info('[trajectory_recorder] saved {} points -> {}'.format(
            len(self.points), self.output_path))


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
