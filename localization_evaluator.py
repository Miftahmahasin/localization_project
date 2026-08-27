#!/usr/bin/env python3
"""
localization_evaluator.py — Ground Truth vs Lokalisasi Evaluator
=================================================================

ARSITEKTUR (v2 — Fase 1A):
  Webots → /ground_truth/odom (nav_msgs/Odometry)
  legged_odometry_kf_node → /odom  (raw FK odom, belum masuk EKF)
  robot_localization EKF  → /odometry/filtered
  nav2_amcl               → /amcl_pose

  Yang dievaluasi:
    GT   : /ground_truth/odom     ← posisi real robot dari Webots
    ODOM : /odom                  ← odom mentah (Fase 1A: apakah EKF suppress ini?)
    EKF  : /odometry/filtered     ← output fusion odom+AMCL
    AMCL : /amcl_pose             ← particle filter saja

CARA PAKAI:
  ros2 launch soccer_object_localization localization_v14.launch.py
  python3 localization_evaluator.py --log-csv pose_eval_$(date +%s).csv
  python3 localization_evaluator.py --no-csv

CATATAN t_s: kolom t_s di CSV menggunakan header.stamp dari /ground_truth/odom
  (sim-time Webots), bukan wall-clock. Tidak perlu --ros-args use_sim_time.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import math, time, csv, argparse
from collections import deque


# ─────────────────────────────────────────────────────────────────────────────
# HELPERS
# ─────────────────────────────────────────────────────────────────────────────

def quat_to_yaw_deg(q) -> float:
    s = 2.0 * (q.w * q.z + q.x * q.y)
    c = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(s, c))

def pos_error(a, b) -> float:
    return math.hypot(a['x'] - b['x'], a['y'] - b['y'])

def yaw_err(a, b) -> float:
    d = a - b
    while d >  180: d -= 360
    while d < -180: d += 360
    return abs(d)

def rmse(h) -> float:
    return math.sqrt(sum(e**2 for e in h) / len(h)) if h else 0.0

def empty():
    return {'x': None, 'y': None, 'yaw': None, 't': None}

A = {
    'G': '\033[92m', 'Y': '\033[93m', 'R': '\033[91m',
    'C': '\033[96m', 'B': '\033[1m',  'Z': '\033[0m',
}
def C(txt, c): return f"{A.get(c,'')}{txt}{A['Z']}"
def ec(e):     return 'G' if e < 0.05 else ('Y' if e < 0.15 else 'R')


# ─────────────────────────────────────────────────────────────────────────────
# NODE
# ─────────────────────────────────────────────────────────────────────────────

class LocalizationEvaluator(Node):
    def __init__(self, log_csv: str):
        super().__init__('localization_evaluator')

        # Pose storage
        self.gt   = empty()   # /ground_truth/odom  — posisi real Webots
        self.odom = empty()   # /odom               — raw legged odom (Fase 1A)
        self.ekf  = empty()   # /odometry/filtered  — estimasi terbaik
        self.amcl = empty()   # /amcl_pose          — AMCL saja
        self.cox  = empty()   # /cox_pose           — Cox direct correction

        # Error history (max 500 sample)
        self.h_ekf  = deque(maxlen=500)
        self.h_amcl = deque(maxlen=500)
        self.h_cox  = deque(maxlen=500)

        # Odom origin for displacement tracking (set on first odom message)
        self._odom_x0 = None
        self._odom_y0 = None

        self._iter      = 0
        self._start     = time.time()   # wall-clock, untuk display elapsed saja
        self._start_sim = None          # sim-time origin (dari header.stamp GT), untuk CSV
        self._n_logged  = 0

        # CSV logger
        self._f = self._w = None
        if log_csv:
            self._f = open(log_csv, 'w', newline='')
            self._w = csv.writer(self._f)
            self._w.writerow([
                't_s',
                'gt_x', 'gt_y', 'gt_yaw_deg',
                'odom_x', 'odom_y', 'odom_yaw_deg',
                'ekf_x', 'ekf_y', 'ekf_yaw_deg',
                'amcl_x', 'amcl_y', 'amcl_yaw_deg',
                'cox_x', 'cox_y', 'cox_yaw_deg',
                'err_ekf_pos_m', 'err_ekf_yaw_deg',
                'err_amcl_pos_m', 'err_amcl_yaw_deg',
                'err_cox_pos_m', 'err_cox_yaw_deg',
            ])
            self.get_logger().info(f'CSV log: {log_csv}')

        # ── Subscriptions ────────────────────────────────────────────────────
        qos_be   = qos_profile_sensor_data
        qos_r    = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        # Nav2 AMCL publishes with RELIABLE/TRANSIENT_LOCAL — must match exactly
        qos_amcl = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Ground truth langsung dari Webots
        self.create_subscription(
            Odometry, '/ground_truth/odom',
            lambda m: self._from_odom(m, self.gt),
            qos_r)

        # Raw legged odom (Fase 1A — apakah EKF suppress ini?)
        self.create_subscription(
            Odometry, '/odom',
            self._from_odom_raw,
            qos_be)

        # Estimasi lokalisasi
        self.create_subscription(
            Odometry, '/odometry/filtered',
            lambda m: self._from_odom(m, self.ekf),
            qos_be)

        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            self._from_pose_cov,
            qos_amcl)

        self.create_subscription(
            PoseWithCovarianceStamped, '/cox_pose',
            lambda m: self._from_pose_cov_target(m, self.cox),
            qos_r)

        # 5Hz untuk resolusi lebih baik di CSV (bukan 2Hz sebelumnya)
        self.timer = self.create_timer(0.2, self._display)
        print("\033[2J\033[H", end='')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _from_odom(self, msg, target: dict):
        p = msg.pose.pose
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        target.update({
            'x':     p.position.x,
            'y':     p.position.y,
            'yaw':   quat_to_yaw_deg(p.orientation),
            't':     time.time(),   # wall-clock, untuk age display
            'stamp': stamp,         # sim-time dari Webots header
        })

    def _from_odom_raw(self, msg):
        p = msg.pose.pose
        x = p.position.x
        y = p.position.y
        if self._odom_x0 is None:
            self._odom_x0 = x
            self._odom_y0 = y
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.odom.update({
            'x':     x,
            'y':     y,
            'yaw':   quat_to_yaw_deg(p.orientation),
            't':     time.time(),
            'stamp': stamp,
        })

    def _from_pose_cov(self, msg):
        self._from_pose_cov_target(msg, self.amcl)

    def _from_pose_cov_target(self, msg, target: dict):
        p = msg.pose.pose
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        target.update({
            'x':     p.position.x,
            'y':     p.position.y,
            'yaw':   quat_to_yaw_deg(p.orientation),
            't':     time.time(),
            'stamp': stamp,
        })

    # ── Display ───────────────────────────────────────────────────────────────

    def _display(self):
        self._iter += 1
        now = time.time()
        t   = now - self._start   # wall-clock elapsed (hanya untuk display header)

        # Sim-time untuk CSV: ambil dari header.stamp GT (di-set Webots, selalu sim-time)
        # Tidak bergantung use_sim_time — timer tetap wall-clock, hanya t_csv yang sim-time
        gt_stamp = self.gt.get('stamp')
        if gt_stamp is not None and gt_stamp > 0:
            if self._start_sim is None:
                self._start_sim = gt_stamp
            t_csv = gt_stamp - self._start_sim
        else:
            t_csv = t   # fallback ke wall-clock sebelum GT pertama tiba

        if self._iter > 1:
            print("\033[25A", end='')

        W = 68
        L = []

        # Header
        L.append(C("═" * W, 'B'))
        L.append(C(f"  LOCALIZATION EVALUATOR  OP3  "
                   f"| t={t:6.1f}s | iter={self._iter}", 'B'))
        L.append(C("═" * W, 'B'))

        # ── Ground Truth ──────────────────────────────────────────────────────
        gt = self.gt
        L.append(C("\n  🌍 GROUND TRUTH  (/ground_truth/odom — Webots)", 'C'))
        if gt['x'] is not None:
            age = (now - gt['t']) * 1000
            L.append(
                f"     x={gt['x']:+8.4f} m   "
                f"y={gt['y']:+8.4f} m   "
                f"yaw={gt['yaw']:+7.2f}°   [{age:.0f}ms]")
        else:
            L.append(C(
                "     ⏳ Belum ada data — pastikan Webots sudah berjalan\n"
                "        dan robot sudah di-spawn di lapangan", 'Y'))

        L.append("")

        # ── Raw Odom ──────────────────────────────────────────────────────────
        L.append(C("  RAW ODOM  /odom  (legged_odometry_kf, belum masuk EKF)", 'C'))
        if self.odom['x'] is not None:
            age = (now - self.odom['t']) * 1000
            L.append(
                f"     x={self.odom['x']:+8.4f} m   "
                f"y={self.odom['y']:+8.4f} m   "
                f"yaw={self.odom['yaw']:+7.2f}   [{age:.0f}ms]")
        else:
            L.append(C("     Belum ada — jalankan legged_odometry_kf_node", 'Y'))

        L.append("")
        L.append(C("  ESTIMASI LOKALISASI:", 'B'))
        L.append(C("  " + "─" * (W - 2), 'B'))

        # ── EKF ──────────────────────────────────────────────────────────────
        L.append(C("  EKF  /odometry/filtered  (fusi odom + AMCL)", 'B'))
        if self.ekf['x'] is not None:
            age = (now - self.ekf['t']) * 1000
            L.append(
                f"     x={self.ekf['x']:+8.4f} m   "
                f"y={self.ekf['y']:+8.4f} m   "
                f"yaw={self.ekf['yaw']:+7.2f}°   [{age:.0f}ms]")
            if gt['x'] is not None:
                ep  = pos_error(gt, self.ekf)
                ey  = yaw_err(gt['yaw'], self.ekf['yaw'])
                dx  = self.ekf['x'] - gt['x']
                dy  = self.ekf['y'] - gt['y']
                self.h_ekf.append(ep)
                rm  = rmse(self.h_ekf)
                L.append(C(
                    f"     Δpos={ep:.4f}m   Δyaw={ey:.2f}°   "
                    f"RMSE={rm:.4f}m   (Δx={dx:+.4f}m  Δy={dy:+.4f}m)",
                    ec(ep)))
                self._log_csv(t_csv)
            else:
                L.append("     (menunggu data GT...)")
        else:
            L.append("     ⏳ Belum ada data dari /odometry/filtered")
            L.append("     Pastikan ekf_node berjalan di launch file")

        L.append("")

        # ── AMCL ─────────────────────────────────────────────────────────────
        L.append(C("  AMCL /amcl_pose  (Particle filter saja)", 'B'))
        if self.amcl['x'] is not None:
            age = (now - self.amcl['t']) * 1000
            L.append(
                f"     x={self.amcl['x']:+8.4f} m   "
                f"y={self.amcl['y']:+8.4f} m   "
                f"yaw={self.amcl['yaw']:+7.2f}°   [{age:.0f}ms]")
            if gt['x'] is not None:
                ep = pos_error(gt, self.amcl)
                ey = yaw_err(gt['yaw'], self.amcl['yaw'])
                dx = self.amcl['x'] - gt['x']
                dy = self.amcl['y'] - gt['y']
                self.h_amcl.append(ep)
                rm = rmse(self.h_amcl)
                L.append(C(
                    f"     Δpos={ep:.4f}m   Δyaw={ey:.2f}°   "
                    f"RMSE={rm:.4f}m   (Δx={dx:+.4f}m  Δy={dy:+.4f}m)",
                    ec(ep)))
            else:
                L.append("     (menunggu data GT...)")
        else:
            L.append("     ⏳ Belum ada data dari /amcl_pose")
            L.append("     Pastikan AMCL aktif dan map sudah di-load")

        # ── Tabel perbandingan absolut ────────────────────────────────────────
        L.append("")
        L.append(C("  📈 PERBANDINGAN ABSOLUT vs GROUND TRUTH:", 'B'))

        if gt['x'] is not None and (self.ekf['x'] is not None
                                    or self.amcl['x'] is not None):
            hdr = (f"     {'Source':<8} {'X est':>9} {'X gt':>9} {'Δx':>8} "
                   f"{'Y est':>9} {'Y gt':>9} {'Δy':>8} {'Δpos':>9} {'Δyaw':>7}")
            L.append(C(hdr, 'B'))
            L.append("     " + "─" * 66)

            for lbl, src, hist in [('EKF', self.ekf, None),
                                    ('AMCL', self.amcl, None),
                                    ('Cox', self.cox, self.h_cox)]:
                if src['x'] is None:
                    continue
                ep  = pos_error(gt, src)
                ey  = yaw_err(gt['yaw'], src['yaw'])
                dx  = src['x'] - gt['x']
                dy  = src['y'] - gt['y']
                if hist is not None:
                    hist.append(ep)
                row = (f"     {lbl:<8} {src['x']:>+9.4f} {gt['x']:>+9.4f} "
                       f"{dx:>+8.4f} {src['y']:>+9.4f} {gt['y']:>+9.4f} "
                       f"{dy:>+8.4f} ")
                L.append(row + C(f"{ep:>8.4f}m", ec(ep)) + f"  {ey:>5.2f}°")

            # RMSE + interpretasi
            L.append("")
            rm_e = rmse(self.h_ekf)
            rm_a = rmse(self.h_amcl)
            rm_c = rmse(self.h_cox)
            n    = len(self.h_ekf)
            cox_str = C(f"Cox={rm_c:.4f}m", ec(rm_c)) if self.h_cox else "Cox=---"
            L.append(
                f"     RMSE  "
                + C(f"EKF={rm_e:.4f}m", ec(rm_e))
                + "   "
                + C(f"AMCL={rm_a:.4f}m", ec(rm_a))
                + "   "
                + cox_str
                + f"   (N={n})")

            qual = ("🟢 BAGUS  (< 5cm)"  if rm_e < 0.05 else
                    "🟡 CUKUP  (< 15cm)" if rm_e < 0.15 else
                    "🔴 PERLU PERBAIKAN (> 15cm)")
            L.append(f"     Kualitas EKF: {qual}")
        else:
            L.append(C("     (menunggu semua data aktif...)", 'Y'))

        L.append(C("\n" + "═" * W, 'B'))
        print('\n'.join(L))

    # ── CSV logger ────────────────────────────────────────────────────────────

    def _log_csv(self, t: float):
        if not self._w:
            return
        gt = self.gt; ekf = self.ekf; amcl = self.amcl; odom = self.odom; cox = self.cox
        if None in (gt['x'], ekf['x']):
            return

        def _f(d, k): return d[k] if d[k] is not None else float('nan')
        amcl_x   = _f(amcl, 'x');  amcl_y = _f(amcl, 'y');  amcl_yaw = _f(amcl, 'yaw')
        cox_x    = _f(cox,  'x');  cox_y  = _f(cox,  'y');  cox_yaw  = _f(cox,  'yaw')
        odom_x   = _f(odom, 'x');  odom_y = _f(odom, 'y');  odom_yaw = _f(odom, 'yaw')

        self._w.writerow([
            f"{t:.3f}",
            f"{gt['x']:.6f}",    f"{gt['y']:.6f}",    f"{gt['yaw']:.4f}",
            f"{odom_x:.6f}",     f"{odom_y:.6f}",     f"{odom_yaw:.4f}",
            f"{ekf['x']:.6f}",   f"{ekf['y']:.6f}",   f"{ekf['yaw']:.4f}",
            f"{amcl_x:.6f}",     f"{amcl_y:.6f}",     f"{amcl_yaw:.4f}",
            f"{cox_x:.6f}",      f"{cox_y:.6f}",       f"{cox_yaw:.4f}",
            f"{pos_error(gt, ekf):.6f}",
            f"{yaw_err(gt['yaw'], ekf['yaw']):.4f}",
            f"{pos_error(gt, amcl) if amcl['x'] is not None else float('nan'):.6f}",
            f"{yaw_err(gt['yaw'], amcl_yaw) if amcl['x'] is not None else float('nan'):.4f}",
            f"{pos_error(gt, cox) if cox['x'] is not None else float('nan'):.6f}",
            f"{yaw_err(gt['yaw'], cox_yaw) if cox['x'] is not None else float('nan'):.4f}",
        ])
        self._f.flush()
        self._n_logged += 1

    # ── Shutdown ──────────────────────────────────────────────────────────────

    def shutdown(self):
        if self._f:
            self._f.close()
            print(f"\n✅ CSV disimpan: {self._f.name}  ({self._n_logged} baris)")

        print("\n" + "=" * 60)
        print("  HASIL EVALUASI AKHIR")
        print("=" * 60)
        for lbl, h in [("EKF  /odometry/filtered", self.h_ekf),
                        ("AMCL /amcl_pose",         self.h_amcl),
                        ("Cox  /cox_pose",           self.h_cox)]:
            if h:
                rm = rmse(h)
                mn = sum(h) / len(h)
                mx = max(h)
                print(f"  {lbl}")
                print(f"    RMSE={rm:.4f}m   Mean={mn:.4f}m   "
                      f"Max={mx:.4f}m   N={len(h)}")
            else:
                print(f"  {lbl}: tidak ada data")
        print()
        print("  Interpretasi RMSE:")
        print("    < 0.05m  ( 5cm)  → Bagus untuk soccer robot")
        print("    < 0.15m (15cm)   → Cukup untuk navigasi kasar")
        print("    > 0.15m (15cm)   → Perlu peningkatan lokalisasi")


# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Evaluator: Ground Truth vs Lokalisasi OP3')
    parser.add_argument(
        '--log-csv',
        default=f'pose_eval_{int(time.time())}.csv',
        help='Path file CSV log (default: pose_eval_<timestamp>.csv)')
    parser.add_argument(
        '--no-csv', action='store_true',
        help='Jangan buat file CSV')
    args = parser.parse_args()

    rclpy.init()
    node = LocalizationEvaluator(None if args.no_csv else args.log_csv)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()