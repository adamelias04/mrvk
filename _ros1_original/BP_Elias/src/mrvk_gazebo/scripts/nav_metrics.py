#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Zbiera metriky pre porovnanie lokalnych planovacov (TEB / DWA / TR).

Beh = od prijatia noveho goalu (/move_base_simple/goal) po vysledok
(/move_base/result). Po kazdom behu vypise zhrnutie a prida riadok do CSV.

Metriky:
  success            dosiahol ciel (result status == 3 SUCCEEDED)
  time_to_goal [s]   trvanie behu
  path_length [m]    integral /shoddy/odom
  avg_speed [m/s]    path_length / time
  min_clearance [m]  min /scan range cez beh
  mean_clearance [m] priemer min-per-scan
  rms_jerk [m/s^3]   2x num. derivacia linear.x z /cmd_vel_raw, RMS
  ang_osc_per_s      zmeny znamienka angular.z / s (deadband)
  ctrl_dt_ms_mean    priem. perioda medzi /cmd_vel_raw (proxy vypoct. zataze)
  ctrl_dt_ms_std     jitter periody
  cpu_mean / cpu_max CPU % procesu move_base (psutil)

Parametre (~):
  ~csv_path     (str,  "/home/adam/ws/nav_metrics.csv")
  ~txt_path     (str,  "/home/adam/ws/nav_metrics.txt")
  ~planner      (str,  "auto" -> rosparam /move_base/base_local_planner)
  ~scan_topic   (str,  "/scan")
  ~odom_topic   (str,  "/shoddy/odom")
  ~cmd_topic    (str,  "/cmd_vel_raw")
  ~goal_topic   (str,  "/move_base_simple/goal")
  ~osc_deadband (float, 0.05)
"""
import csv
import math
import os
import rospy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import Bool

try:
    import psutil
    HAVE_PSUTIL = True
except ImportError:
    HAVE_PSUTIL = False


class NavMetrics:
    def __init__(self):
        rospy.init_node("nav_metrics")

        self.csv_path = os.path.expanduser(
            rospy.get_param("~csv_path", "/home/adam/ws/nav_metrics.csv"))
        self.txt_path = os.path.expanduser(
            rospy.get_param("~txt_path", "/home/adam/ws/nav_metrics.txt"))
        self.planner = rospy.get_param("~planner", "auto")
        if self.planner == "auto":
            self.planner = rospy.get_param(
                "/move_base/base_local_planner", "unknown").split("/")[-1]
        self.osc_deadband = float(rospy.get_param("~osc_deadband", 0.05))
        # cesta k cost_value predikcnej vrstvy — zaznamena sa do CSV/TXT
        # aby bolo per-planner nastavenie self-dokumentovane (chyti omyl
        # ked sa zabudne synchronizovat cost_value s planovacom).
        self.cost_param = rospy.get_param(
            "~cost_param",
            "/move_base/local_costmap/predicted_path_obstacle_layer/cost_value")

        # tour_mode: cely waypoint_tour = jeden beh (delimitovany
        # /nav_tour/active), namiesto per-goal merania.
        self.tour_mode = bool(rospy.get_param("~tour_mode", False))
        tour_topic = rospy.get_param("~tour_topic", "/nav_tour/active")

        scan_topic = rospy.get_param("~scan_topic", "/scan")
        odom_topic = rospy.get_param("~odom_topic", "/shoddy/odom")
        cmd_topic  = rospy.get_param("~cmd_topic",  "/cmd_vel_raw")
        goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")

        self.active = False
        self.run_id = 0
        self._reset()

        self.mb_proc = self._find_move_base() if HAVE_PSUTIL else None
        if self.mb_proc:
            self.mb_proc.cpu_percent(None)  # prime psutil

        rospy.Subscriber(scan_topic, LaserScan, self.scan_cb, queue_size=5)
        rospy.Subscriber(odom_topic, Odometry,  self.odom_cb, queue_size=20)
        rospy.Subscriber(cmd_topic,  Twist,     self.cmd_cb,  queue_size=20)
        rospy.Subscriber(goal_topic, PoseStamped, self.goal_cb, queue_size=2)
        rospy.Subscriber("/move_base/result", MoveBaseActionResult,
                         self.result_cb, queue_size=2)
        rospy.Subscriber(tour_topic, Bool, self.tour_cb, queue_size=2)
        rospy.Timer(rospy.Duration(0.2), self.cpu_timer)

        self._ensure_csv_header()
        rospy.loginfo("[nav_metrics] planner=%s psutil=%s tour_mode=%s",
                      self.planner, HAVE_PSUTIL, self.tour_mode)
        rospy.loginfo("[nav_metrics] CSV: %s", self.csv_path)
        rospy.loginfo("[nav_metrics] TXT: %s", self.txt_path)
        rospy.loginfo("[nav_metrics] cakam na goal na %s ...", goal_topic)

    # ---- helpers ----------------------------------------------------
    def _find_move_base(self):
        for p in psutil.process_iter(["name", "cmdline"]):
            try:
                if "move_base" in (p.info["name"] or ""):
                    return p
                if any("move_base" in c for c in (p.info["cmdline"] or [])):
                    return p
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        rospy.logwarn("[nav_metrics] proces move_base nenajdeny -> CPU sa preskoci")
        return None

    def _reset(self):
        self.t_start = None
        self.last_pos = None
        self.path_len = 0.0
        self.min_clear = float("inf")
        self.clear_sum = 0.0
        self.clear_n = 0
        self.cmd_samples = []   # (t, v, w) z /cmd_vel_raw — osc + ctrl_dt
        self.odom_vel = []      # (t, vx) z /shoddy/odom — jerk
        self.cpu_samples = []

    def _ensure_csv_header(self):
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, "w", newline="") as f:
                csv.writer(f).writerow([
                    "planner", "cost_value", "run_id", "success", "time_to_goal",
                    "path_length", "avg_speed", "min_clearance",
                    "mean_clearance", "rms_jerk", "ang_osc_per_s",
                    "ctrl_dt_ms_mean", "ctrl_dt_ms_std",
                    "cpu_mean", "cpu_max"])

    # ---- callbacks --------------------------------------------------
    def tour_cb(self, msg):
        if not self.tour_mode:
            return
        if msg.data and not self.active:
            self._reset()
            self.active = True
            self.t_start = rospy.Time.now()
            self.run_id += 1
            rospy.loginfo("[nav_metrics] === TURA %d START (planner=%s) ===",
                          self.run_id, self.planner)
        elif (not msg.data) and self.active:
            self.active = False
            self._finalize(True)  # tura dobehla po vsetky wp

    def goal_cb(self, _msg):
        if self.tour_mode:
            return  # v tour_mode delimituje beh /nav_tour/active
        self._reset()
        self.active = True
        self.t_start = rospy.Time.now()
        self.run_id += 1
        rospy.loginfo("[nav_metrics] === BEH %d START (planner=%s) ===",
                      self.run_id, self.planner)

    def scan_cb(self, msg):
        if not self.active:
            return
        rmin = msg.range_min
        rmax = msg.range_max
        local_min = float("inf")
        for r in msg.ranges:
            if math.isfinite(r) and rmin <= r <= rmax:
                if r < local_min:
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
            self.path_len += math.hypot(x - self.last_pos[0],
                                        y - self.last_pos[1])
        self.last_pos = (x, y)
        self.odom_vel.append(
            (rospy.Time.now().to_sec(), msg.twist.twist.linear.x))

    def cmd_cb(self, msg):
        if not self.active:
            return
        self.cmd_samples.append(
            (rospy.Time.now().to_sec(), msg.linear.x, msg.angular.z))

    def cpu_timer(self, _evt):
        if self.active and self.mb_proc is not None:
            try:
                self.cpu_samples.append(self.mb_proc.cpu_percent(None))
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass

    def result_cb(self, msg):
        if self.tour_mode:
            return  # v tour_mode delimituje beh /nav_tour/active
        if not self.active:
            return
        self.active = False
        success = (msg.status.status == 3)  # SUCCEEDED
        self._finalize(success)

    # ---- finalize ---------------------------------------------------
    def _finalize(self, success):
        t = (rospy.Time.now() - self.t_start).to_sec() if self.t_start else 0.0
        length = self.path_len
        avg_v = length / t if t > 1e-3 else 0.0
        min_c = self.min_clear if math.isfinite(self.min_clear) else -1.0
        mean_c = (self.clear_sum / self.clear_n) if self.clear_n else -1.0

        rms_jerk = self._compute_rms_jerk()
        ang_osc = self._compute_ang_osc(t)
        dt_mean, dt_std = self._compute_ctrl_dt()
        cpu_mean = (sum(self.cpu_samples) / len(self.cpu_samples)
                    if self.cpu_samples else -1.0)
        cpu_max = max(self.cpu_samples) if self.cpu_samples else -1.0
        cost_v = rospy.get_param(self.cost_param, -1)

        row = [self.planner, cost_v, self.run_id, int(success),
               round(t, 3), round(length, 3), round(avg_v, 3),
               round(min_c, 3), round(mean_c, 3), round(rms_jerk, 4),
               round(ang_osc, 3), round(dt_mean, 2), round(dt_std, 2),
               round(cpu_mean, 1), round(cpu_max, 1)]
        with open(self.csv_path, "a", newline="") as f:
            csv.writer(f).writerow(row)

        summary = (
            "========================================================\n"
            " BEH %d   planner=%s   cost_value=%s   success=%s\n"
            "--------------------------------------------------------\n"
            "  cas do ciela        : %.2f s\n"
            "  dlzka drahy         : %.2f m\n"
            "  priem. rychlost     : %.2f m/s\n"
            "  min. odstup prekazka: %.2f m\n"
            "  priem. odstup       : %.2f m\n"
            "  RMS jerk            : %.3f m/s^3\n"
            "  oscilacie (ang)     : %.2f /s\n"
            "  ctrl perioda        : %.1f +- %.1f ms\n"
            "  CPU move_base       : %.1f %% (max %.1f %%)\n"
            "========================================================\n"
            % (self.run_id, self.planner, cost_v, success, t, length, avg_v,
               min_c, mean_c, rms_jerk, ang_osc, dt_mean, dt_std,
               cpu_mean, cpu_max))

        with open(self.txt_path, "a") as f:
            f.write(summary)

        for line in summary.rstrip("\n").split("\n"):
            rospy.loginfo("[nav_metrics] %s", line)
        rospy.loginfo("[nav_metrics] -> CSV: %s", self.csv_path)
        rospy.loginfo("[nav_metrics] -> TXT: %s", self.txt_path)

    def _compute_rms_jerk(self):
        # Jerk z realizovanej rychlosti (/shoddy/odom), nie zo skokoveho
        # /cmd_vel_raw. Postup: vyhladenie kĺzavým priemerom (okno 5) ->
        # centralne diferencie na zrychlenie -> centralne diferencie na
        # jerk -> RMS. Centralne diferencie + smoothing potlacia
        # kvantizacny sum diskretneho 20 Hz signalu.
        s = self.odom_vel
        if len(s) < 9:
            return 0.0

        t = [p[0] for p in s]
        v = [p[1] for p in s]

        # kĺzavý priemer, okno 5 (centrovane)
        w = 2
        vs = []
        for i in range(len(v)):
            lo = max(0, i - w)
            hi = min(len(v), i + w + 1)
            vs.append(sum(v[lo:hi]) / (hi - lo))

        # zrychlenie centralnou diferenciou
        acc = []
        for i in range(1, len(vs) - 1):
            dt = t[i + 1] - t[i - 1]
            if dt > 1e-4:
                acc.append((t[i], (vs[i + 1] - vs[i - 1]) / dt))
        if len(acc) < 3:
            return 0.0

        # jerk centralnou diferenciou zrychlenia
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


if __name__ == "__main__":
    try:
        NavMetrics()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
