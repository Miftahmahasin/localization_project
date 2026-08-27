#!/usr/bin/env python3
"""
global_loc_test.py — Diagnostik Struktur Ambiguitas AMCL (Langkah A)
=====================================================================
Jalankan dengan robot DIAM di posisi spawn. Secara otomatis:
  1. Panggil reinitialize_global_localization (sebar partikel ke seluruh lapangan)
  2. Tunggu konvergensi (default 60s)
  3. Catat posisi konvergensi AMCL
  4. Ulangi N kali

Usage:
  ros2 run soccer_object_localization global_loc_test
  # atau langsung:
  python3 global_loc_test.py

Param:
  n_trials       (int, default 12) — jumlah percobaan
  wait_s         (float, default 60.0) — tunggu konvergensi per trial
  output_csv     (str, default global_loc_results.csv)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import csv, math, time, os


class GlobalLocTest(Node):
    def __init__(self):
        super().__init__('global_loc_test')
        self.declare_parameter('n_trials',      12)
        self.declare_parameter('wait_s',       180.0)   # v2: 180s timeout per trial
        self.declare_parameter('conv_cov_thr',   1.0)   # v2: konvergen jika cov_xx < 1.0m²
        self.declare_parameter('output_csv', 'global_loc_results.csv')
        self.declare_parameter('gt_x',  -0.363)         # referensi statis (diabaikan jika use_gt_odom)
        self.declare_parameter('gt_y',   0.0)
        self.declare_parameter('use_nomotion', False)    # v5: False=robot berjalan (Pendekatan 2)
        self.declare_parameter('use_gt_odom',  True)    # v5: True=GT dari /ground_truth/odom (dinamis)

        self.n_trials     = self.get_parameter('n_trials').value
        self.wait_s       = self.get_parameter('wait_s').value
        self.conv_cov_thr = self.get_parameter('conv_cov_thr').value
        self.output_csv   = self.get_parameter('output_csv').value
        self.gt_x         = self.get_parameter('gt_x').value
        self.gt_y         = self.get_parameter('gt_y').value
        self.use_nomotion = self.get_parameter('use_nomotion').value
        self.use_gt_odom  = self.get_parameter('use_gt_odom').value

        qos_amcl = QoSProfile(depth=1,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self._amcl_x = None
        self._amcl_y = None
        self._amcl_yaw = None
        self._amcl_cov_xx = None
        self._gt_x_live = None   # v5: GT dari /ground_truth/odom (Pendekatan 2)
        self._gt_y_live = None

        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                  self._amcl_cb, qos_amcl)

        # v5: GT dinamis dari ground truth odom (Pendekatan 2)
        if self.use_gt_odom:
            self.create_subscription(Odometry, '/ground_truth/odom', self._gt_cb, 10)

        self._reinit_cli = self.create_client(Empty, '/reinitialize_global_localization')
        self._nomotion_cli = self.create_client(Empty, '/request_nomotion_update')

        mode_str = 'DINAMIS (robot berjalan)' if not self.use_nomotion else 'STATIS (nomotion update)'
        gt_str   = 'ground_truth/odom' if self.use_gt_odom else f'({self.gt_x},{self.gt_y})'
        self.get_logger().info(
            f'GlobalLocTest v5: {self.n_trials} trials x {self.wait_s}s | '
            f'Mode={mode_str} | GT={gt_str} | Output={self.output_csv}')

        self._results = []
        self._timer = self.create_timer(0.5, self._run_once_bootstrap)
        # v6: nomotion timer selalu dibuat — dipakai bootstrap pertama + static mode
        self._nomotion_timer = self.create_timer(0.5, self._force_nomotion_update)
        self._trial = 0
        self._phase = 'WAIT_SERVICE'
        self._phase_start = time.monotonic()
        self._last_pose_before_reinit = None  # v2: deteksi apakah pose berubah setelah reinit
        self._nomotion_active = False          # aktif saat: (a) bootstrap 3s pertama, (b) static mode
        self._nomotion_bootstrap = False       # v6: True hanya selama 3s bootstrap setelah reinit
        self._bootstrap_end_t = 0.0
        self._cov_spiked = False               # v4: cov_xx pernah > 3.0 → reinit berhasil

    def _gt_cb(self, msg: Odometry):
        """v5: simpan GT posisi terkini dari /ground_truth/odom (Pendekatan 2 dinamis)."""
        self._gt_x_live = msg.pose.pose.position.x
        self._gt_y_live = msg.pose.pose.position.y

    def _force_nomotion_update(self):
        """v6: paksa AMCL proses scan.
        Bootstrap (3s pertama setelah reinit): selalu aktif — trigger publish pertama.
        Static mode: aktif terus (nomotion tiap 0.5s).
        Dynamic mode: hanya aktif 3s bootstrap, lalu berhenti (gerak robot cukup).
        """
        now = time.monotonic()
        # Bootstrap selesai jika sudah 3s DAN AMCL sudah publish
        if self._nomotion_bootstrap and now > self._bootstrap_end_t:
            self._nomotion_bootstrap = False
            if not self.use_nomotion:
                self._nomotion_active = False  # dynamic: matikan setelah bootstrap
        if self._nomotion_active and self._nomotion_cli.service_is_ready():
            self._nomotion_cli.call_async(Empty.Request())

    def _amcl_cb(self, msg):
        from math import atan2
        q = msg.pose.pose.orientation
        yaw = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self._amcl_x   = msg.pose.pose.position.x
        self._amcl_y   = msg.pose.pose.position.y
        self._amcl_yaw = math.degrees(yaw)
        self._amcl_cov_xx = msg.pose.covariance[0]

    def _run_once_bootstrap(self):
        now = time.monotonic()
        elapsed = now - self._phase_start

        if self._phase == 'WAIT_SERVICE':
            if self._reinit_cli.wait_for_service(timeout_sec=0.1):
                self.get_logger().info('Service /reinitialize_global_localization ready. '
                                       'Waiting for first AMCL pose ...')
                self._phase = 'WAIT_AMCL'
                self._phase_start = now
            elif elapsed > 30.0:
                self.get_logger().error('Service not available after 30s. Check AMCL is running.')
                raise SystemExit(1)
            return

        # v6: tunggu sampai /amcl_pose diterima setidaknya sekali sebelum reinit
        if self._phase == 'WAIT_AMCL':
            if self._amcl_x is not None:
                self.get_logger().info(
                    f'AMCL pose received: ({self._amcl_x:.3f},{self._amcl_y:.3f}) '
                    f'cov_xx={self._amcl_cov_xx:.3f}. Mulai trials.')
                self._phase = 'TRIGGER'
                self._phase_start = now
            elif elapsed > 60.0:
                self.get_logger().error(
                    'AMCL tidak publish dalam 60s. Pastikan stack berjalan dan AMCL ACTIVE.')
                raise SystemExit(1)
            else:
                # Bootstrap nomotion untuk memancing AMCL publish jika belum ada retained msg
                self._nomotion_active = True
                if int(elapsed) % 10 == 0 and elapsed > 1.0:
                    self.get_logger().info(
                        f'  Menunggu AMCL pose... ({elapsed:.0f}s)')
            return

        if self._phase == 'TRIGGER':
            self._trial += 1
            # v2: simpan pose SEBELUM reinit
            self._last_pose_before_reinit = (self._amcl_x, self._amcl_y, self._amcl_cov_xx)
            self._cov_spiked = False  # v4: reset spike detector
            self.get_logger().info(
                f'--- Trial {self._trial}/{self.n_trials}: calling reinitialize_global_localization ...')
            self._reinit_cli.call_async(Empty.Request())
            # v6: bootstrap nomotion 3s pertama — trigger publish pertama setelah reinit
            # (diperlukan bahkan dalam dynamic mode: AMCL diam sampai ada update pertama)
            self._nomotion_active = True
            self._nomotion_bootstrap = True
            self._bootstrap_end_t = now + 3.0
            self._phase = 'CONVERGE'
            self._phase_start = now
            return

        if self._phase == 'CONVERGE':
            remaining = self.wait_s - elapsed
            cov = self._amcl_cov_xx if self._amcl_cov_xx is not None else 999.0
            if cov > 3.0:
                self._cov_spiked = True
            # v6: static mode → nomotion aktif terus setelah bootstrap selesai
            if self.use_nomotion and not self._nomotion_bootstrap:
                self._nomotion_active = True
            converged = cov < self.conv_cov_thr

            if remaining > 0 and not converged:
                if int(elapsed) % 15 == 0 and int(elapsed) > 0:
                    cur = f'({self._amcl_x:.3f},{self._amcl_y:.3f})' if self._amcl_x is not None else 'N/A'
                    self.get_logger().info(
                        f'  t={elapsed:.0f}s: AMCL={cur}  cov_xx={cov:.3f}  '
                        f'(conv_thr={self.conv_cov_thr})  remaining={remaining:.0f}s')
                return

            if not converged and remaining <= 0:
                self.get_logger().warn(
                    f'Trial {self._trial}: TIMEOUT {self.wait_s}s, cov_xx={cov:.2f} belum konvergen '
                    f'(thr={self.conv_cov_thr}). Catat sebagai NOT_CONVERGED.')

            # v2: cek apakah pose BENAR-BENAR berubah setelah reinit
            pose_changed = True
            if self._last_pose_before_reinit is not None and self._amcl_x is not None:
                px, py, pcov = self._last_pose_before_reinit
                if px == self._amcl_x and py == self._amcl_y:
                    pose_changed = False
                    self.get_logger().warn(
                        f'Trial {self._trial}: pose IDENTIK dengan sebelum reinit '
                        f'({self._amcl_x:.4f},{self._amcl_y:.4f}) — reinit mungkin tidak efektif!')

            # Konvergensi selesai — catat
            if self._amcl_x is None:
                self.get_logger().warn(f'Trial {self._trial}: no AMCL data! Skipping.')
            else:
                # v5: gunakan GT live (dinamis) jika tersedia, fallback ke param statis
                gt_x_ref = self._gt_x_live if self._gt_x_live is not None else self.gt_x
                gt_y_ref = self._gt_y_live if self._gt_y_live is not None else self.gt_y
                err = math.sqrt((self._amcl_x - gt_x_ref)**2 +
                                (self._amcl_y - gt_y_ref)**2)
                cov = self._amcl_cov_xx if self._amcl_cov_xx is not None else 999.0
                if not converged:
                    verdict = 'NOT_CONVERGED'
                elif not pose_changed and not self._cov_spiked:
                    # v4: pose sama DAN cov tidak pernah spike → reinit tidak efektif
                    verdict = 'NO_REINIT'
                elif err < 0.3:
                    verdict = 'BENAR'
                else:
                    verdict = 'FALSE_MIN'
                self._results.append({
                    'trial':        self._trial,
                    'amcl_x':      self._amcl_x,
                    'amcl_y':      self._amcl_y,
                    'amcl_yaw':    self._amcl_yaw,
                    'cov_xx':      cov,
                    'conv_t_s':    elapsed,
                    'pose_changed': pose_changed,
                    'gt_x':        gt_x_ref,
                    'gt_y':        gt_y_ref,
                    'err_m':       err,
                    'verdict':     verdict,
                })
                self.get_logger().info(
                    f'Trial {self._trial}: AMCL=({self._amcl_x:.3f},{self._amcl_y:.3f}) '
                    f'err={err:.3f}m [{verdict}]')

            self._nomotion_active = False  # v3: matikan saat phase selesai

            if self._trial >= self.n_trials:
                self._save_and_shutdown()
                return

            self._phase = 'TRIGGER'
            self._phase_start = now
            return

    def _save_and_shutdown(self):
        path = self.output_csv
        with open(path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=[
                'trial','amcl_x','amcl_y','amcl_yaw','cov_xx','conv_t_s',
                'pose_changed','gt_x','gt_y','err_m','verdict'])
            w.writeheader()
            w.writerows(self._results)
        self.get_logger().info(f'Saved {len(self._results)} results to {path}')

        # Ringkasan inline
        good  = [r for r in self._results if r['verdict'] == 'BENAR']
        false = [r for r in self._results if r['verdict'] == 'FALSE_MIN']
        self.get_logger().info(
            f'\n{"="*50}\n'
            f'HASIL: {len(good)}/{len(self._results)} konvergen BENAR\n'
            f'       {len(false)}/{len(self._results)} FALSE MINIMUM\n'
            f'Jalankan: python3 analyze_global_loc.py {path}\n'
            f'{"="*50}')
        raise SystemExit(0)


def main():
    rclpy.init()
    node = GlobalLocTest()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
