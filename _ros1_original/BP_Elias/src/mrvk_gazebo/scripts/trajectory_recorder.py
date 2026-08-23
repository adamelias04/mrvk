#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Zapisuje trajektoriu robota (z /shoddy/odom) do CSV suboru.
Jeden beh = jeden CSV. Subor sa ulozi pri vypnuti uzla (Ctrl+C / SIGTERM).

CSV format: t,x,y,yaw   (t v sekundach od startu zaznamu)

Parametre (~):
  ~odom_topic   (str,  "/shoddy/odom")
  ~output_path  (str,  "/home/adam/ws/trajectories/trajectory.csv")
  ~min_distance (float, 0.02)   min. posun medzi zaznamenanymi bodmi [m]
  ~tour_only    (bool, False)   ak True, zaznamenava iba pocas
                                /nav_tour/active == true
"""
import csv
import math
import os
import rospy

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class TrajectoryRecorder:
    def __init__(self):
        self.odom_topic   = rospy.get_param("~odom_topic", "/shoddy/odom")
        self.output_path  = rospy.get_param(
            "~output_path", "/home/adam/ws/trajectories/trajectory.csv")
        self.min_distance = rospy.get_param("~min_distance", 0.02)
        self.tour_only    = rospy.get_param("~tour_only", False)

        self.points = []     # list of (t, x, y, yaw)
        self.t0 = None
        self.last_xy = None
        self.tour_active = not self.tour_only  # ak nie tour_only, vzdy zapisuj

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=50)
        if self.tour_only:
            rospy.Subscriber("/nav_tour/active", Bool, self.tour_cb,
                             queue_size=1)

        rospy.on_shutdown(self.save)

        rospy.loginfo("[trajectory_recorder] zaznamenavam %s -> %s "
                      "(min_dist=%.3f m, tour_only=%s)",
                      self.odom_topic, self.output_path,
                      self.min_distance, self.tour_only)

    def tour_cb(self, msg):
        self.tour_active = bool(msg.data)

    def odom_cb(self, msg):
        if not self.tour_active:
            return
        t = msg.header.stamp.to_sec()
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
            rospy.logwarn("[trajectory_recorder] ziadne body, nezapisujem.")
            return
        os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
        with open(self.output_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "x", "y", "yaw"])
            for row in self.points:
                w.writerow(["{:.3f}".format(row[0]),
                            "{:.4f}".format(row[1]),
                            "{:.4f}".format(row[2]),
                            "{:.4f}".format(row[3])])
        rospy.loginfo("[trajectory_recorder] ulozenych %d bodov -> %s",
                      len(self.points), self.output_path)


if __name__ == "__main__":
    rospy.init_node("trajectory_recorder")
    TrajectoryRecorder()
    rospy.spin()
