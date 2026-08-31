#!/usr/bin/env python3
"""Objectively score dynamic_obstacle_predictor false positives on a bag.

Ground truth is a vacancy test on the raw cloud, independent of anything
the predictor believes. A thing that genuinely moved has LEFT, so the
ground it occupied is empty a couple of seconds later; a fence or a hedge
is still exactly where it was, however convincingly its cluster centroid
appeared to slide as the robot drove past.

The test is anchored on the cluster's OWN measured points rather than on
any derived position. A derived position wanders into the gaps of a big
sprawling cluster and would read as "vacated" while the structure is
plainly still there -- the measured points cannot lie that way. So for a
sample at time T we remember the segment's points (in odom) and at
T+delta ask what fraction of them are still occupied:

  mostly still occupied -> it never left           -> FALSE POSITIVE
  mostly vacated        -> it really did move away -> TRUE POSITIVE

Points that have gone out of sensor range by T+delta are dropped from the
fraction, since absence there proves nothing. The headline number is
FP_FRAMES: how many predictor cycles a bogus prediction was live, i.e. how
much the costmap actually got polluted.

Usage: eval_predictor.py [duration_s] [out_json]
"""
import json
import math
import sys
import time
from collections import deque

import numpy as np
import rclpy
import rclpy.duration
import rclpy.time
import tf2_ros
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import MarkerArray

VACANCY_DELAY = 2.0        # seconds before asking "is it still there?"
POINT_RADIUS = 0.35        # an anchor counts as still occupied within this
MAX_QUERY_RANGE = 12.0     # beyond this, absence just means out of range
MIN_USABLE_POINTS = 6      # need this many in-range anchors for a verdict
STATIC_OCC_FRACTION = 0.5  # >= this fraction still occupied == never moved
MOVED_OCC_FRACTION = 0.25  # <= this fraction still occupied == really moved
ANCHORS_PER_SAMPLE = 25    # subsample of the cluster used as anchors
CLOUD_BUFFER_S = 6.0
SAMPLE_PERIOD = 0.5        # don't re-probe the same track more often


class Evaluator(Node):
    def __init__(self):
        super().__init__('eval_predictor')
        self.sim_time = None
        self.pos_hist = {}      # id -> [(t, x, y)] centroid of matched segment
        self.shape = {}         # id -> [(n_pts, extent)]
        self.novelty = {}       # id -> [novelty], read back from marker colour
        self.dyn_frames = {}    # id -> cycles with a live prediction
        self.dyn_ids = set()    # ids that ever published a prediction
        self.pending = []       # queued vacancy queries
        self.verdicts = {}      # id -> [fraction_still_occupied, ...]
        self.last_sampled = {}  # id -> sim time of last queued query
        self.clouds = deque()
        self.n_marker_msgs = 0
        self.n_seg_msgs = 0

        self.tf_buffer = tf2_ros.Buffer(
            cache_time=rclpy.duration.Duration(seconds=30))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Clock, '/clock', self._clock_cb, best_effort)
        self.create_subscription(MarkerArray,
                                 '/predicted_obstacle_debug_segment',
                                 self._seg_cb, 20)
        self.create_subscription(MarkerArray, '/predicted_obstacle_markers',
                                 self._marker_cb, 20)
        self.create_subscription(PointCloud2,
                                 '/ground_segmentation/obstacle_points',
                                 self._cloud_cb, best_effort)

    def _clock_cb(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _cloud_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        raw = list(point_cloud2.read_points(msg, field_names=('x', 'y'),
                                            skip_nans=True))
        pts = (np.array([[p[0], p[1]] for p in raw], dtype=np.float64)
               if raw else np.zeros((0, 2)))
        self.clouds.append((t, msg.header.frame_id, pts))
        while self.clouds and t - self.clouds[0][0] > CLOUD_BUFFER_S:
            self.clouds.popleft()
        self._service_pending(t, msg.header.frame_id, pts)

    def _seg_cb(self, msg):
        if self.sim_time is None:
            return
        self.n_seg_msgs += 1
        for m in msg.markers:
            if not m.points:
                continue
            pts = np.array([[p.x, p.y] for p in m.points])
            self.pos_hist.setdefault(m.id, []).append(
                (self.sim_time, float(pts[:, 0].mean()), float(pts[:, 1].mean())))
            span = pts.max(axis=0) - pts.min(axis=0)
            self.shape.setdefault(m.id, []).append(
                (len(m.points), float(math.hypot(span[0], span[1]))))
            # the predictor encodes its own decision statistics in the
            # marker colour: r=novelty, g=straightness, b=net_disp/2
            self.novelty.setdefault(m.id, []).append(
                (float(m.color.r), float(m.color.g), float(m.color.b)))

            # Anchor a vacancy query on this cluster's real points. Probing
            # every track, not only the ones currently called dynamic, gives
            # ground truth for the whole candidate set so a threshold can be
            # chosen from the full distribution rather than guessed.
            if True:
                last = self.last_sampled.get(m.id, -1e9)
                if self.sim_time - last >= SAMPLE_PERIOD:
                    self.last_sampled[m.id] = self.sim_time
                    idx = np.linspace(0, len(pts) - 1,
                                      min(ANCHORS_PER_SAMPLE,
                                          len(pts))).astype(int)
                    self.pending.append((self.sim_time + VACANCY_DELAY,
                                         m.id, pts[idx]))

    def _marker_cb(self, msg):
        if self.sim_time is None:
            return
        self.n_marker_msgs += 1
        for m in msg.markers:
            if m.points:
                self.dyn_frames[m.id] = self.dyn_frames.get(m.id, 0) + 1
                self.dyn_ids.add(m.id)

    # ---- vacancy test ------------------------------------------------

    def _service_pending(self, now_t, frame, cloud_pts):
        if not self.pending:
            return
        due = [q for q in self.pending if q[0] <= now_t]
        if not due:
            return
        self.pending = [q for q in self.pending if q[0] > now_t]

        try:
            tf = self.tf_buffer.lookup_transform(
                frame, 'odom',
                rclpy.time.Time(seconds=int(now_t),
                                nanoseconds=int((now_t % 1) * 1e9)),
                timeout=rclpy.duration.Duration(seconds=0))
        except Exception:
            return

        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        tx, ty = tf.transform.translation.x, tf.transform.translation.y

        for _, tid, anchors in due:
            lx = c * anchors[:, 0] - s * anchors[:, 1] + tx
            ly = s * anchors[:, 0] + c * anchors[:, 1] + ty
            in_range = np.hypot(lx, ly) <= MAX_QUERY_RANGE
            if int(in_range.sum()) < MIN_USABLE_POINTS:
                continue
            lx, ly = lx[in_range], ly[in_range]

            if cloud_pts.shape[0] == 0:
                self.verdicts.setdefault(tid, []).append(0.0)
                continue

            d2 = ((cloud_pts[:, 0][None, :] - lx[:, None]) ** 2 +
                  (cloud_pts[:, 1][None, :] - ly[:, None]) ** 2)
            still = (d2.min(axis=1) <= POINT_RADIUS ** 2)
            self.verdicts.setdefault(tid, []).append(float(still.mean()))


def summarize(ev):
    rows = []
    for tid, hist in ev.pos_hist.items():
        if len(hist) < 2:
            continue
        net = math.hypot(hist[-1][1] - hist[0][1], hist[-1][2] - hist[0][2])
        path = sum(math.hypot(hist[i][1] - hist[i - 1][1],
                              hist[i][2] - hist[i - 1][2])
                   for i in range(1, len(hist)))
        shapes = ev.shape.get(tid, [])
        v = ev.verdicts.get(tid, [])
        nov = ev.novelty.get(tid, [])
        rows.append({
            'id': tid,
            'samples': len(hist),
            'lifetime_s': round(hist[-1][0] - hist[0][0], 2),
            'net_disp_m': round(net, 2),
            'path_len_m': round(path, 2),
            'straightness': round(net / path, 3) if path > 1e-6 else 0.0,
            'mean_pts': (round(sum(s[0] for s in shapes) / len(shapes), 1)
                         if shapes else 0),
            'max_extent_m': round(max((s[1] for s in shapes), default=0.0), 2),
            'dyn_frames': ev.dyn_frames.get(tid, 0),
            'n_probes': len(v),
            'occ_frac': round(float(np.mean(v)), 3) if v else None,
            'novelty_mean': round(float(np.mean([n[0] for n in nov])), 3) if nov else None,
            'novelty_max': round(float(np.max([n[0] for n in nov])), 3) if nov else None,
            'w_straight_mean': round(float(np.mean([n[1] for n in nov])), 3) if nov else None,
            'w_straight_p75': round(float(np.percentile([n[1] for n in nov], 75)), 3) if nov else None,
            'w_netdisp_mean': round(float(np.mean([n[2] for n in nov])) * 2.0, 3) if nov else None,
            'w_netdisp_p75': round(float(np.percentile([n[2] for n in nov], 75)) * 2.0, 3) if nov else None,
        })

    dyn = [r for r in rows if r['dyn_frames'] > 0]
    judged = [r for r in dyn if r['n_probes'] >= 2]
    fp = [r for r in judged if r['occ_frac'] >= STATIC_OCC_FRACTION]
    tp = [r for r in judged if r['occ_frac'] <= MOVED_OCC_FRACTION]
    mid = [r for r in judged
           if MOVED_OCC_FRACTION < r['occ_frac'] < STATIC_OCC_FRACTION]
    unjudged = [r for r in dyn if r['n_probes'] < 2]

    fpf = sum(r['dyn_frames'] for r in fp)
    tpf = sum(r['dyn_frames'] for r in tp)
    return {
        'n_tracks_total': len(rows),
        'n_dynamic_tracks': len(dyn),
        'n_judged_tracks': len(judged),
        'n_false_positive_tracks': len(fp),
        'n_true_positive_tracks': len(tp),
        'n_middle_tracks': len(mid),
        'n_unjudged_tracks': len(unjudged),
        'fp_frames': fpf,
        'tp_frames': tpf,
        'mid_frames': sum(r['dyn_frames'] for r in mid),
        'unjudged_frames': sum(r['dyn_frames'] for r in unjudged),
        'fp_rate': round(fpf / max(1, fpf + tpf), 3),
        'total_dyn_frames': sum(r['dyn_frames'] for r in dyn),
        'seg_msgs': ev.n_seg_msgs,
        'marker_msgs': ev.n_marker_msgs,
        'false_positives': sorted(fp, key=lambda r: -r['dyn_frames'])[:15],
        'true_positives': sorted(tp, key=lambda r: -r['dyn_frames'])[:15],
        'middle': sorted(mid, key=lambda r: -r['dyn_frames'])[:10],
        'all_tracks': sorted(rows, key=lambda r: -r['dyn_frames']),
    }


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
    out_path = sys.argv[2] if len(sys.argv) > 2 else None

    rclpy.init()
    ev = Evaluator()

    start = time.time()
    while time.time() - start < duration:
        rclpy.spin_once(ev, timeout_sec=0.2)

    result = summarize(ev)
    printable = {k: v for k, v in result.items() if not isinstance(v, list)}
    print(json.dumps(printable, indent=2))
    if out_path:
        with open(out_path, 'w') as fh:
            json.dump(result, fh, indent=2)

    ev.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
