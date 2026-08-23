#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 port. Collects metrics for comparing local planners (TEB / DWB / TR).

Run = from a new goal to its result. In tour_mode the whole tour (delimited by
/nav_tour/active) is one run, which is what waypoint_tour / goal_pingpong drive.
Per-goal mode starts on /goal_pose and finalizes when the next goal arrives.

Metrics: success, time_to_goal, path_length, avg_speed, min/mean clearance,
rms_jerk, ang_osc_per_s, ctrl_dt_ms mean/std, cpu mean/max (controller_server).

Params:
  csv_path, txt_path, planner, scan_topic, odom_topic, cmd_topic, goal_topic,
  osc_deadband, tour_mode, tour_topic, cost_value
"""
import csv
import math
import os

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool

try:
    import psutil
    HAVE_PSUTIL = True
except ImportError:
    HAVE_PSUTIL = False


class NavMetrics(Node):
    def __init__(self):
        super().__init__('nav_metrics')

        home_ws = os.path.join(os.path.expanduser('~'), 'ws')
        self.csv_path = os.path.expanduser(
            self.declare_parameter('csv_path', os.path.join(home_ws, 'nav_metrics.csv')).value)
        self.txt_path = os.path.expanduser(
            self.declare_parameter('txt_path', os.path.join(home_ws, 'nav_metrics.txt')).value)
        self.planner = self.declare_parameter('planner', 'unknown').value
        self.osc_deadband = float(self.declare_parameter('osc_deadband', 0.05).value)
        self.cost_value = int(self.declare_parameter('cost_value', -1).value)

        self.tour_mode = bool(self.declare_parameter('tour_mode', False).value)
        tour_topic = self.declare_parameter('tour_topic', '/nav_tour/active').value

        scan_topic = self.declare_parameter('scan_topic', '/scan').value
        odom_topic = self.declare_parameter('odom_topic', '/shoddy/odom').value
        cmd_topic = self.declare_parameter('cmd_topic', '/cmd_vel_raw').value
        goal_topic = self.declare_parameter('goal_topic', '/goal_pose').value

        self.active = False
        self.run_id = 0
        self._reset()

        self.mb_proc = self._find_controller() if HAVE_PSUTIL else None
        if self.mb_proc:
            self.mb_proc.cpu_percent(None)

        self.create_subscription(LaserScan, scan_topic, self.scan_cb, 5)
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 20)
        self.create_subscription(Twist, cmd_topic, self.cmd_cb, 20)
        self.create_subscription(PoseStamped, goal_topic, self.goal_cb, 2)
        self.create_subscription(Bool, tour_topic, self.tour_cb, 2)
        self.create_timer(0.2, self.cpu_timer)

        self._ensure_csv_header()
        self.get_logger().info('[nav_metrics] planner={} psutil={} tour_mode={}'.format(
            self.planner, HAVE_PSUTIL, self.tour_mode))
        self.get_logger().info('[nav_metrics] CSV: {}'.format(self.csv_path))
        self.get_logger().info('[nav_metrics] TXT: {}'.format(self.txt_path))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _find_controller(self):
        for p in psutil.process_iter(['name', 'cmdline']):
            try:
                cmd = ' '.join(p.info['cmdline'] or [])
                if 'controller_server' in (p.info['name'] or '') or 'controller_server' in cmd:
                    return p
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        self.get_logger().warn('[nav_metrics] controller_server process not found -> CPU skipped')
        return None

    def _reset(self):
        self.t_start = None
        self.last_pos = None
        self.path_len = 0.0
        self.min_clear = float('inf')
        self.clear_sum = 0.0
        self.clear_n = 0
        self.cmd_samples = []
        self.odom_vel = []
        self.cpu_samples = []

    def _ensure_csv_header(self):
        if not os.path.exists(self.csv_path):
            os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
            with open(self.csv_path, 'w', newline='') as f:
                csv.writer(f).writerow([
                    'planner', 'cost_value', 'run_id', 'success', 'time_to_goal',
                    'path_length', 'avg_speed', 'min_clearance', 'mean_clearance',
                    'rms_jerk', 'ang_osc_per_s', 'ctrl_dt_ms_mean', 'ctrl_dt_ms_std',
                    'cpu_mean', 'cpu_max'])

    def tour_cb(self, msg):
        if not self.tour_mode:
            return
        if msg.data and not self.active:
            self._reset()
            self.active = True
            self.t_start = self._now()
            self.run_id += 1
            self.get_logger().info('[nav_metrics] === TOUR {} START (planner={}) ==='.format(
                self.run_id, self.planner))
        elif (not msg.data) and self.active:
            self.active = False
            self._finalize(True)

    def goal_cb(self, _msg):
        if self.tour_mode:
            return
        if self.active:
            # finalize previous run (approximate: goal reached / superseded)
            self.active = False
            self._finalize(True)
        self._reset()
        self.active = True
        self.t_start = self._now()
        self.run_id += 1
        self.get_logger().info('[nav_metrics] === RUN {} START (planner={}) ==='.format(
            self.run_id, self.planner))

    def scan_cb(self, msg):
        if not self.active:
            return
        rmin, rmax = msg.range_min, msg.range_max
        local_min = float('inf')
        for r in msg.ranges:
            if math.isfinite(r) and rmin <= r <= rmax and r < local_min:
                local_min = r
        if math.isfinite(local_min):
            self.min_clear = min(self.min_clear, local_min)
            self.clear_sum += local_min
            self.clear_n += 1

    def odom_cb(self, msg):
        if not self.active:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.last_pos is not None:
            self.path_len += math.hypot(x - self.last_pos[0], y - self.last_pos[1])
        self.last_pos = (x, y)
        self.odom_vel.append((self._now(), msg.twist.twist.linear.x))

    def cmd_cb(self, msg):
        if not self.active:
            return
        self.cmd_samples.append((self._now(), msg.linear.x, msg.angular.z))

    def cpu_timer(self):
        if self.active and self.mb_proc is not None:
            try:
                self.cpu_samples.append(self.mb_proc.cpu_percent(None))
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass

    def _finalize(self, success):
        t = (self._now() - self.t_start) if self.t_start else 0.0
        length = self.path_len
        avg_v = length / t if t > 1e-3 else 0.0
        min_c = self.min_clear if math.isfinite(self.min_clear) else -1.0
        mean_c = (self.clear_sum / self.clear_n) if self.clear_n else -1.0

        rms_jerk = self._compute_rms_jerk()
        ang_osc = self._compute_ang_osc(t)
        dt_mean, dt_std = self._compute_ctrl_dt()
        cpu_mean = (sum(self.cpu_samples) / len(self.cpu_samples)) if self.cpu_samples else -1.0
        cpu_max = max(self.cpu_samples) if self.cpu_samples else -1.0
        cost_v = self.cost_value

        row = [self.planner, cost_v, self.run_id, int(success),
               round(t, 3), round(length, 3), round(avg_v, 3),
               round(min_c, 3), round(mean_c, 3), round(rms_jerk, 4),
               round(ang_osc, 3), round(dt_mean, 2), round(dt_std, 2),
               round(cpu_mean, 1), round(cpu_max, 1)]
        with open(self.csv_path, 'a', newline='') as f:
            csv.writer(f).writerow(row)

        summary = (
            '========================================================\n'
            ' RUN {}   planner={}   cost_value={}   success={}\n'
            '--------------------------------------------------------\n'
            '  time to goal        : {:.2f} s\n'
            '  path length         : {:.2f} m\n'
            '  avg speed           : {:.2f} m/s\n'
            '  min clearance       : {:.2f} m\n'
            '  mean clearance      : {:.2f} m\n'
            '  RMS jerk            : {:.3f} m/s^3\n'
            '  ang oscillations    : {:.2f} /s\n'
            '  ctrl period         : {:.1f} +- {:.1f} ms\n'
            '  CPU controller      : {:.1f} %% (max {:.1f} %%)\n'
            '========================================================\n'.format(
                self.run_id, self.planner, cost_v, success, t, length, avg_v,
                min_c, mean_c, rms_jerk, ang_osc, dt_mean, dt_std, cpu_mean, cpu_max))

        with open(self.txt_path, 'a') as f:
            f.write(summary)

        for line in summary.rstrip('\n').split('\n'):
            self.get_logger().info('[nav_metrics] {}'.format(line))

    def _compute_rms_jerk(self):
        s = self.odom_vel
        if len(s) < 9:
            return 0.0
        t = [p[0] for p in s]
        v = [p[1] for p in s]
        w = 2
        vs = []
        for i in range(len(v)):
            lo = max(0, i - w)
            hi = min(len(v), i + w + 1)
            vs.append(sum(v[lo:hi]) / (hi - lo))
        acc = []
        for i in range(1, len(vs) - 1):
            dt = t[i + 1] - t[i - 1]
            if dt > 1e-4:
                acc.append((t[i], (vs[i + 1] - vs[i - 1]) / dt))
        if len(acc) < 3:
            return 0.0
        js = []
        for i in range(1, len(acc) - 1):
            dt = acc[i + 1][0] - acc[i - 1][0]
            if dt > 1e-4:
                js.append((acc[i + 1][1] - acc[i - 1][1]) / dt)
        if not js:
            return 0.0
        return math.sqrt(sum(j * j for j in js) / len(js))

    def _compute_ang_osc(self, t):
        s = self.cmd_samples
        if len(s) < 2 or t <= 1e-3:
            return 0.0
        sign = 0
        changes = 0
        for (_, _, w) in s:
            if abs(w) < self.osc_deadband:
                continue
            cur = 1 if w > 0 else -1
            if sign != 0 and cur != sign:
                changes += 1
            sign = cur
        return changes / t

    def _compute_ctrl_dt(self):
        s = self.cmd_samples
        if len(s) < 3:
            return 0.0, 0.0
        dts = [(s[i][0] - s[i - 1][0]) * 1000.0 for i in range(1, len(s))]
        mean = sum(dts) / len(dts)
        var = sum((d - mean) ** 2 for d in dts) / len(dts)
        return mean, math.sqrt(var)


def main(args=None):
    rclpy.init(args=args)
    node = NavMetrics()
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
