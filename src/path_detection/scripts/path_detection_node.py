#!/usr/bin/env python3
# ROS2 port of the camera-based walkable path detector. Subscribes to
# /camera/image_raw, segments the walkable surface by dominant colour and
# publishes a binary mask as path_detection/DetectedPath on
# /shoddy/detected_path (consumed by detected_path_layer costmap plugin).

import math

import numpy
import cv2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from path_detection.msg import DetectedPath


class PathDetector:
    def __init__(self, node, pub):
        self.node = node
        self.pub = pub
        self.initialized = 0

        self.thresh_lower_hsv = []  # lower [H,S,V] boundaries of accepted pathway color
        self.thresh_upper_hsv = []  # upper [H,S,V] boundaries of accepted pathway color

        # parameters for tuning
        self.K = 2  # number of dominant colors to extract in initialization
        self.offset_lower = [15, 25, 55]  # [H,S,V] negative offset
        self.offset_upper = [15, 25, 55]  # [H,S,V] positive offset
        self.field_of_vision = None
        self.operating_area = None
        self.controller = self.Controller(node, pub)

    class FieldOfVision:
        def __init__(self, shape, lower_width, upper_width, height, x0=0.0, y0=0.0, type='Origin'):
            if type == 'Centralized':
                x0 = (shape[1] - lower_width) / 2.0 - 0.5 * (1.0 + shape[1] % 2)

            self.mask = self.create_field_of_vision_mask(
                shape, lower_width, upper_width, height, x0, y0)
            self.control_zone = self.create_control_zone(shape, upper_width, height)

            _, thresh = cv2.threshold(self.mask, 0, 255, 0)
            self.contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            self.shape = shape
            self.lower_width = lower_width
            self.upper_width = upper_width
            self.height = height
            self.x0 = x0
            self.y0 = y0
            self.type = type

        def create_field_of_vision_mask(self, shape, lower_width, upper_width, height, x0, y0):
            def sign(u):
                return 2.0 * (u > 0) + 1.0 * (u == 0) - 1.0

            def y(x, x0, y0, k):
                return k * (x - x0) + y0

            ROWS = shape[0]
            COLS = shape[1]
            mask = numpy.zeros(shape, dtype='uint8')

            x01 = lower_width + x0
            x02 = x0
            y0 = ROWS - 1 - y0

            width = lower_width - upper_width
            sgn = sign(width)

            if sgn == 0:
                for j in range(ROWS):
                    for i in range(COLS):
                        mask[j][i] = ((i <= x01) and (i >= x02) and (j >= y0 - height) and (j <= y0))
                return mask

            k1 = 2.0 * height / width
            k2 = -2.0 * height / width

            for j in range(ROWS):
                for i in range(COLS):
                    mask[j][i] = ((sgn * (j - y(i, x01, y0, k1)) >= 0) and
                                  (sgn * (j - y(i, x02, y0, k2)) >= 0) and
                                  (j >= y0 - height) and (j <= y0))
            return mask

        def create_control_zone(self, shape, upper_width, height):
            return numpy.array([shape[0] - 1.0 * height,
                                shape[0] - 0.7 * height,
                                shape[1] / 2 - 0.25 * upper_width,
                                shape[1] / 2 + 0.25 * upper_width], dtype='uint64')

    class Controller:
        def __init__(self, node, pub):
            self.node = node
            self.pub = pub
            self.v = 0.0
            self.w = 0.0
            self.passability = 0.0
            self.thresh_MODE0to1 = 0.2
            self.thresh_MODE1to0 = 0.6
            self.currMODE = 0

        def calcActuatingSig(self, x, y):
            def saturation(u, lower_limit, upper_limit):
                return max(lower_limit, min(u, upper_limit))

            def rate_limiter(yv, u, falling_slew_rate, rising_slew_rate):
                rate = u - yv
                if rate < falling_slew_rate:
                    return yv + falling_slew_rate
                if rate > rising_slew_rate:
                    return yv + rising_slew_rate
                return u

            if len(x) < 4 or len(y) < 4:
                self.node.get_logger().warn('Insufficient path points for control computation.')
                return

            dx = x[-3] - x[0]
            dy = -y[-3] + y[0]
            ang = math.atan2(dy, dx)

            if self.passability <= self.thresh_MODE0to1:
                self.currMODE = 1

            if (self.currMODE == 1) and (self.passability >= self.thresh_MODE1to0):
                self.currMODE = 0

            if self.currMODE == 0:
                v = 3 * self.passability
                v = saturation(v, -1, 1)
                v = rate_limiter(self.v, v, -0.1, 0.1)

                w = -math.pi / 2 + ang
                w = saturation(5 * w, -0.5, 0.5)
                w = rate_limiter(self.w, w, -0.1, 0.1)
            else:
                v = 0.0
                v = rate_limiter(self.v, v, -0.1, 0.1)
                w = -0.5
                w = rate_limiter(self.w, w, -0.1, 0.1)

            self.v = v
            self.w = w

            msg = Twist()
            msg.linear.x = float(v)
            msg.angular.z = float(w)
            self.pub.publish(msg)

    def initialize(self, img):
        if self.field_of_vision is None:
            shape = img.shape[0:2]
            height = shape[0] / 1.5
            lower_width = shape[1]
            upper_width = shape[1]
            self.field_of_vision = self.FieldOfVision(shape, lower_width, upper_width, height, type='Centralized')

        if self.operating_area is None:
            shape = img.shape[0:2]
            height = 250.0
            lower_width = shape[1]
            upper_width = shape[1]
            self.operating_area = self.FieldOfVision(shape, lower_width, upper_width, height, type='Centralized')

        [sizeY, sizeX, _] = img.shape
        centerImg = img[(sizeY - int(sizeY / 9)): sizeY,
                        int(sizeX / 2 - sizeX / 4.5): int(sizeX / 2 + sizeX / 4.5), :]

        height, width, _ = centerImg.shape
        centerIm = numpy.float32(centerImg.reshape(height * width, 3))

        _, _, centers = cv2.kmeans(
            K=self.K,
            flags=cv2.KMEANS_RANDOM_CENTERS,
            attempts=2,
            bestLabels=None,
            data=centerIm,
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0))

        dom_col_rgb = numpy.uint8(centers)
        dom_col_hsv = numpy.empty([numpy.size(dom_col_rgb, 0), numpy.size(dom_col_rgb, 1)])
        self.thresh_lower_hsv = numpy.empty([numpy.size(dom_col_rgb, 0), numpy.size(dom_col_rgb, 1)])
        self.thresh_upper_hsv = numpy.empty([numpy.size(dom_col_rgb, 0), numpy.size(dom_col_rgb, 1)])

        for i in range(numpy.size(dom_col_rgb, 0)):
            dom_col_hsv[i, :] = cv2.cvtColor(numpy.array([[dom_col_rgb[i, :]]]), cv2.COLOR_BGR2HSV).flatten()
            self.thresh_lower_hsv[i, :] = numpy.clip(numpy.subtract(dom_col_hsv[i, :], self.offset_lower), 0, 255)
            self.thresh_upper_hsv[i, :] = numpy.clip(numpy.add(dom_col_hsv[i, :], self.offset_upper), 0, 255)

    def detectPath(self, img):
        mask_comb = self.create_mask(img)
        cv2.bitwise_or(mask_comb, 255 * self.operating_area.mask, mask_comb)

        dp = DetectedPath()
        dp.height = mask_comb.shape[0]
        dp.width = mask_comb.shape[1]
        dp.frame = list(mask_comb.flatten())
        self.pub.publish(dp)

        return img, mask_comb

    def create_mask(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask_array = []
        for i in range(numpy.size(self.thresh_lower_hsv, 0)):
            mask_array.append(cv2.inRange(hsv, self.thresh_lower_hsv[i, :], self.thresh_upper_hsv[i, :]))

        mask_comb = mask_array[0]
        for i in range(1, numpy.size(self.thresh_lower_hsv, 0)):
            mask_comb = cv2.bitwise_or(mask_comb, mask_array[i])

        kernel = numpy.ones((3, 3), numpy.uint8)
        mask_comb = cv2.morphologyEx(mask_comb, cv2.MORPH_CLOSE, kernel)

        mask_comb = mask_comb * self.field_of_vision.mask

        cnts, _ = cv2.findContours(mask_comb, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        try:
            cnt = max(cnts, key=cv2.contourArea)
        except ValueError:
            self.node.get_logger().warn('Unable to find clear path')
            return mask_comb

        out = numpy.zeros(mask_comb.shape, numpy.uint8)
        cv2.drawContours(out, [cnt], -1, 255, cv2.FILLED)
        mask_comb = cv2.bitwise_and(mask_comb, out)

        kernel = numpy.ones((11, 11), numpy.uint8)
        mask_comb = cv2.morphologyEx(mask_comb, cv2.MORPH_CLOSE, kernel)

        cnts, _ = cv2.findContours(mask_comb, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        try:
            cnt2 = max(cnts, key=cv2.contourArea)
        except ValueError:
            self.node.get_logger().warn('Unable to find clear path')
            return mask_comb
        cv2.fillPoly(mask_comb, pts=[cnt2], color=(255, 255, 255))

        return mask_comb

    def check_passability(self, arr):
        return arr.sum() / arr.size

    def process_img(self, frame):
        img = numpy.frombuffer(frame.data, numpy.uint8)
        img = img.reshape(frame.height, frame.width, 3)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        img = cv2.GaussianBlur(img, (3, 3), 0)

        if self.initialized == 0:
            self.initialize(img)
        self.initialized = 1

        imgDet, mask_comb = self.detectPath(img)
        cv2.drawContours(imgDet, self.field_of_vision.contours, -1, (0, 0, 255), 3)
        cv2.drawContours(imgDet, self.operating_area.contours, -1, (0, 255, 255), 3)

        cv2.imshow('mask', mask_comb)
        cv2.imshow('path', imgDet)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('path_detection')
    pub = node.create_publisher(DetectedPath, '/shoddy/detected_path', 10)
    path_det = PathDetector(node, pub)
    node.create_subscription(Image, '/camera/image_raw', path_det.process_img, 10)
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
