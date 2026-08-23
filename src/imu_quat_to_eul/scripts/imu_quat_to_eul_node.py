#!/usr/bin/env python3
# ROS2 port: prevadza kvaternion orientacie z IMU a odometrie na Eulerove
# uhly (roll/pitch/yaw) a publikuje ako imu_quat_to_eul/EulRot. Sucasne
# loguje do ~/Documents/{imuRot.txt,odomRot.txt} (rovnako ako ROS1 verzia).

import os

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from imu_quat_to_eul.msg import EulRot

from tf_transformations import euler_from_quaternion


class Transformer(Node):
    def __init__(self):
        super().__init__('imu_quat_to_eul_node')

        docs = os.path.join(os.path.expanduser('~'), 'Documents')
        os.makedirs(docs, exist_ok=True)
        self.odom_full_path = os.path.join(docs, 'odomRot.txt')
        self.imu_full_path = os.path.join(docs, 'imuRot.txt')
        open(self.odom_full_path, 'w').close()
        open(self.imu_full_path, 'w').close()

        self.pub_imu = self.create_publisher(EulRot, '/imu_quat_to_eul_top', 10)
        self.pub_odom = self.create_publisher(EulRot, '/odom_quat_to_eul_top', 10)

        self.create_subscription(Imu, '/imu_data', self.transform_callback, 10)
        self.create_subscription(Odometry, '/shoddy/odom', self.odometry_callback, 10)

        self.get_logger().info('imu_quat_to_eul_node started')

    def write_to_file(self, stamp, x, y, z, fullpath):
        with open(fullpath, 'a') as f:
            f.write('{} {} {:.4f} {:.4f} {:.4f}  \n'.format(
                stamp.sec, stamp.nanosec, x, y, z))

    def transform_callback(self, data):
        euler = euler_from_quaternion([data.orientation.x,
                                       data.orientation.y,
                                       data.orientation.z,
                                       data.orientation.w])
        msg = EulRot()
        msg.time = data.header.stamp
        msg.x = euler[0]
        msg.y = euler[1]
        msg.z = euler[2]
        self.pub_imu.publish(msg)
        self.write_to_file(data.header.stamp, euler[0], euler[1], euler[2],
                           self.imu_full_path)

    def odometry_callback(self, data):
        q = data.pose.pose.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
        msg = EulRot()
        msg.time = data.header.stamp
        msg.x = euler[0]
        msg.y = euler[1]
        msg.z = euler[2]
        self.pub_odom.publish(msg)
        self.write_to_file(data.header.stamp, euler[0], euler[1], euler[2],
                           self.odom_full_path)


def main(args=None):
    rclpy.init(args=args)
    node = Transformer()
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
