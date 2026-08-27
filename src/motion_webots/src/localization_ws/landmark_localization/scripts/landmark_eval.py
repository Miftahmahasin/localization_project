#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro — Apache-2.0
"""TAHAP 4/8 — live convergence evaluator for the no-odom geometric stack.

Two modes:

  LIVE (default): subscribe the running stack and log a CSV per tick —
    GT   /ground_truth/odom   (Webots truth)
    EKF  /odometry/filtered   (fused estimate — the thing under test)
    FIX  /landmark_pose        (raw geometric fixes, before fusion)
  Uses the GT header.stamp as the time base (Webots sim-time), so no
  use_sim_time plumbing is needed (same convention as localization_evaluator.py).
  On exit it prints: EKF position/yaw RMSE, time-to-converge (first time the EKF
  error drops below --conv_m and STAYS there for --conv_hold s), the fix rate
  (full/single/none per second), and the longest fix-less gap (the down-gaze
  coasting window the no-odom design must survive).

  COMPARE: ``--compare a.csv b.csv`` overlays two runs' EKF error(t) and prints
  their RMSE / time-to-converge side by side — built to decide single-corner
  option (a) partial-update vs (b) coast on the SAME walked trajectory.

Examples:
  # run A (single-corner partial), while the sim + v15 stack are running:
  python3 landmark_eval.py --out runA_partial.csv --dur 180
  # run B (coast): relaunch v15 with single_corner_mode:=coast, then:
  python3 landmark_eval.py --out runB_coast.csv --dur 180
  # compare:
  python3 landmark_eval.py --compare runA_partial.csv runB_coast.csv
"""
import argparse
import csv
import math
import sys


def _yaw(q):
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def _wrap_deg(d):
    while d > 180:
        d -= 360
    while d < -180:
        d += 360
    return abs(d)


# ── analysis shared by live-exit summary and --compare ───────────────────────
def summarize(rows, conv_m, conv_hold, label='', center_deadband=0.5):
    """rows: list of dict with t, gt_x, gt_y, gt_yaw, ekf_x, ekf_y, ekf_yaw,
    ekf_err, ekf_yaw_err (ekf_* may be '' before first EKF msg)."""
    ts, errs, yerr = [], [], []
    for r in rows:
        if r['ekf_err'] in ('', None):
            continue
        ts.append(float(r['t']))
        errs.append(float(r['ekf_err']))
        yerr.append(float(r['ekf_yaw_err']))
    if not errs:
        print('  %s: no EKF samples' % label)
        return None
    n = len(errs)
    rmse = math.sqrt(sum(e * e for e in errs) / n)
    yrmse = math.sqrt(sum(e * e for e in yerr) / n)
    t0 = ts[0]
    # time-to-converge: first t after which error stays < conv_m for conv_hold s
    conv = None
    for i in range(n):
        if errs[i] < conv_m:
            j = i
            while j < n and ts[j] - ts[i] < conv_hold:
                if errs[j] >= conv_m:
                    break
                j += 1
            if j < n and ts[j] - ts[i] >= conv_hold:
                conv = ts[i] - t0
                break
            if j >= n and ts[-1] - ts[i] >= conv_hold * 0.5:
                conv = ts[i] - t0
                break
    med = sorted(errs)[n // 2]
    p95 = sorted(errs)[min(n - 1, int(0.95 * n))]
    print('  %s: n=%d  pos RMSE=%.3f m  median=%.3f  p95=%.3f  yaw RMSE=%.2f deg'
          % (label, n, rmse, med, p95, yrmse))
    print('       time-to-converge(<%.2fm for %.0fs)=%s'
          % (conv_m, conv_hold, '%.1f s' % conv if conv is not None
             else 'NOT REACHED'))
    # RAW geometric fix (/landmark_pose) vs GT — the DIAGNOSTIC split: if this is
    # small while the EKF RMSE above is large, the fix is right and the EKF is
    # trapped (fusion); if this is ALSO large, the wrong pose is upstream.
    ferrs = []
    for r in rows:
        fe = r.get('fix_err', '')
        if fe not in ('', None):
            try:
                ferrs.append(float(fe))
            except ValueError:
                pass
    if ferrs:
        fr = math.sqrt(sum(e * e for e in ferrs) / len(ferrs))
        fmed = sorted(ferrs)[len(ferrs) // 2]
        fp95 = sorted(ferrs)[min(len(ferrs) - 1, int(0.95 * len(ferrs)))]
        print('       RAW fix vs GT: n=%d  RMSE=%.3f m  median=%.3f  p95=%.3f'
              % (len(ferrs), fr, fmed, fp95))
    # TAHAP 5 — 180-degree mirror diagnostics: which side is the EKF on, and how
    # many times did it flip? (GATE 5 target: MIRROR ~0%, flips=0.)
    # A DEADBAND around the field centre is applied: within ~0.5 m of (0,0) the
    # true pose and its 180-degree mirror nearly COINCIDE, so "which side" is
    # genuinely undefined there (the symmetry singularity) and the raw label
    # flickers on sub-decimetre noise — those are metric artifacts, not real
    # localization flips. Samples inside the deadband are excluded and counted.
    sides = []
    dead = 0
    for r in rows:
        if r['ekf_err'] in ('', None):
            continue
        try:
            gx, gy = float(r['gt_x']), float(r['gt_y'])
            ex, ey = float(r['ekf_x']), float(r['ekf_y'])
        except (KeyError, ValueError):
            sides = []
            break
        if math.hypot(gx, gy) < center_deadband:
            dead += 1
            continue
        d_true = math.hypot(ex - gx, ey - gy)
        d_mir = math.hypot(ex + gx, ey + gy)
        sides.append(1 if d_mir < d_true else 0)     # 1 = on the mirror side
    mir_pct = flips = None
    if sides:
        mir = sum(sides)
        flips = sum(1 for i in range(1, len(sides)) if sides[i] != sides[i - 1])
        mir_pct = 100.0 * mir / len(sides)
        print('       side (|GT|>%.1fm): TRUE %.0f%%  MIRROR %.0f%%  flips=%d '
              '(%d near-centre samples excluded)'
              % (center_deadband, 100.0 - mir_pct, mir_pct, flips, dead))
    return {'ts': ts, 'errs': errs, 'n': n, 'rmse': rmse, 'yrmse': yrmse,
            'median': med, 'p95': p95, 'conv': conv, 'mirror_pct': mir_pct,
            'flips': flips}


def compare(files, conv_m, conv_hold, out_png):
    series = []
    for f in files:
        rows = list(csv.DictReader(open(f)))
        print('\n== %s ==' % f)
        r = summarize(rows, conv_m, conv_hold, label=f.split('/')[-1])
        if r:
            series.append((f.split('/')[-1], r['ts'], r['errs']))
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        plt.figure(figsize=(11, 5))
        for name, ts, errs in series:
            t0 = ts[0]
            plt.plot([t - t0 for t in ts], errs, lw=1.2, label=name)
        plt.axhline(conv_m, ls='--', c='k', alpha=0.5,
                    label='converge thr %.2f m' % conv_m)
        plt.xlabel('time since start (s)'); plt.ylabel('EKF position error (m)')
        plt.title('No-odom EKF convergence — single-corner (a) partial vs (b) coast')
        plt.legend(); plt.grid(alpha=0.3); plt.ylim(bottom=0)
        plt.tight_layout(); plt.savefig(out_png, dpi=110)
        print('\nsaved %s' % out_png)
    except Exception as e:
        print('plot skipped: %r' % e)


# ── live logging ─────────────────────────────────────────────────────────────
def live(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseWithCovarianceStamped

    class Ev(Node):
        def __init__(self):
            super().__init__('landmark_eval')
            self.gt = None
            self.ekf = None
            self.rows = []
            self.fix = dict(full=0, single=0, last_t=None, gaps=[], last_fix_t=None)
            self.fixpose = None          # latest RAW /landmark_pose (x, y, yaw)
            self.t0 = None
            self.create_subscription(Odometry, args.gt_topic, self._gt,
                                     qos_profile_sensor_data)
            self.create_subscription(Odometry, args.ekf_topic, self._ekf, 20)
            self.create_subscription(PoseWithCovarianceStamped, args.fix_topic,
                                     self._fix, 20)
            self.f = open(args.out, 'w', newline='')
            self.w = csv.writer(self.f)
            self.w.writerow(['t', 'gt_x', 'gt_y', 'gt_yaw', 'ekf_x', 'ekf_y',
                             'ekf_yaw', 'ekf_err', 'ekf_yaw_err',
                             'fix_x', 'fix_y', 'fix_yaw', 'fix_err'])
            self.create_timer(1.0 / args.rate, self._tick)
            self.get_logger().info('logging -> %s (dur=%.0fs)'
                                   % (args.out, args.dur))

        def _stamp(self, msg):
            return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        def _gt(self, m):
            self.gt = (self._stamp(m), m.pose.pose.position.x,
                       m.pose.pose.position.y, _yaw(m.pose.pose.orientation))

        def _ekf(self, m):
            self.ekf = (m.pose.pose.position.x, m.pose.pose.position.y,
                        _yaw(m.pose.pose.orientation))

        def _fix(self, m):
            t = self._stamp(m)
            yaw_var = m.pose.covariance[35]
            kind = 'single' if yaw_var > 1e3 else 'full'
            self.fix[kind] += 1
            if self.fix['last_fix_t'] is not None:
                self.fix['gaps'].append(t - self.fix['last_fix_t'])
            self.fix['last_fix_t'] = t
            self.fixpose = (m.pose.pose.position.x, m.pose.pose.position.y,
                            _yaw(m.pose.pose.orientation))

        def _tick(self):
            if self.gt is None:
                return
            t = self.gt[0]
            if self.t0 is None:
                self.t0 = t
            if t - self.t0 > args.dur:
                self._finish()
                return
            # raw-fix columns (may be present even before the EKF is up)
            if self.fixpose is not None:
                fx, fy, fyaw = self.fixpose
                ferr = '%.4f' % math.hypot(fx - self.gt[1], fy - self.gt[2])
                fcol = [fx, fy, fyaw, ferr]
            else:
                fcol = ['', '', '', '']
            if self.ekf is None:
                self.w.writerow([t, self.gt[1], self.gt[2], self.gt[3],
                                 '', '', '', '', ''] + fcol)
                return
            err = math.hypot(self.ekf[0] - self.gt[1], self.ekf[1] - self.gt[2])
            yerr = _wrap_deg(self.ekf[2] - self.gt[3])
            self.w.writerow([t, self.gt[1], self.gt[2], self.gt[3],
                             self.ekf[0], self.ekf[1], self.ekf[2],
                             '%.4f' % err, '%.3f' % yerr] + fcol)

        def _finish(self):
            self.f.close()
            rows = list(csv.DictReader(open(args.out)))
            print('\n=== live run summary ===')
            summarize(rows, args.conv_m, args.conv_hold, label=args.out)
            tot = self.fix['full'] + self.fix['single']
            gap = max(self.fix['gaps']) if self.fix['gaps'] else 0.0
            dur = max(rows[-1]['t'] and (float(rows[-1]['t']) - self.t0), 1e-9)
            print('  fixes: full=%d single=%d  (%.1f/s)  longest fix-less gap=%.2f s'
                  % (self.fix['full'], self.fix['single'], tot / dur, gap))
            raise KeyboardInterrupt

    rclpy.init()
    node = Ev()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.f.closed:
            node.f.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='landmark_eval.csv')
    ap.add_argument('--dur', type=float, default=180.0)
    ap.add_argument('--rate', type=float, default=10.0)
    ap.add_argument('--gt_topic', default='/ground_truth/odom')
    ap.add_argument('--ekf_topic', default='/odometry/filtered')
    ap.add_argument('--fix_topic', default='/landmark_pose')
    ap.add_argument('--conv_m', type=float, default=0.30)
    ap.add_argument('--conv_hold', type=float, default=3.0)
    ap.add_argument('--compare', nargs='+', metavar='CSV',
                    help='compare 2+ CSVs instead of running live')
    ap.add_argument('--compare_png', default='landmark_eval_compare.png')
    args = ap.parse_args()
    if args.compare:
        compare(args.compare, args.conv_m, args.conv_hold, args.compare_png)
        return
    live(args)


if __name__ == '__main__':
    main()
