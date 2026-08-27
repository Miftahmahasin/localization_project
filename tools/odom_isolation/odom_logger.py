#!/usr/bin/env python3
"""
tools/odom_isolation/odom_logger.py
------------------------------------
Fase 0 — Log /odom mentah vs /ground_truth/odom ke CSV.

Jalankan setelah source workspace:
  python3 tools/odom_isolation/odom_logger.py --output exp1.csv --label exp1 --speed 1.0

Ctrl+C untuk berhenti. Baris header ditulis sekali di awal.

QoS otomatis:
  /odom            → BEST_EFFORT (sesuai legged_odometry_kf_node)
  /ground_truth/odom → RELIABLE  (sesuai op3_extern_controller.cpp)
"""
import argparse
import csv
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from nav_msgs.msg import Odometry


def quat_to_yaw(q) -> float:
    return 2.0 * math.atan2(q.z, q.w)


def _make_qos(reliable: bool, depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=(
            QoSReliabilityPolicy.RELIABLE
            if reliable
            else QoSReliabilityPolicy.BEST_EFFORT
        ),
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


class OdomLogger(Node):
    def __init__(self, output_csv: str, exp_label: str, speed_cmd: float):
        super().__init__('odom_logger')

        self.exp_label = exp_label
        self.speed_cmd = speed_cmd

        # state
        self._odom_msg   = None
        self._gt_msg     = None
        self._odom_stamp = 0.0
        self._gt_stamp   = 0.0
        self._t0         = None
        self._row_count  = 0
        self._diag_count = 0

        # CSV setup
        new_file = not os.path.exists(output_csv)
        self._f   = open(output_csv, 'a', newline='')
        self._csv = csv.writer(self._f)
        if new_file:
            self._csv.writerow([
                't_s',
                'odom_x', 'odom_y', 'odom_yaw_rad',
                'gt_x',   'gt_y',   'gt_yaw_rad',
                'exp_label', 'speed_cmd',
            ])
            self._f.flush()

        # QoS — match publisher QoS exactly
        qos_best_effort = _make_qos(reliable=False, depth=1)   # for /odom
        qos_reliable    = _make_qos(reliable=True,  depth=10)  # for /ground_truth/odom

        self.create_subscription(Odometry, '/odom',              self._cb_odom, qos_best_effort)
        self.create_subscription(Odometry, '/ground_truth/odom', self._cb_gt,   qos_reliable)

        # 20 Hz log timer
        self.create_timer(0.05, self._log)

        # Diagnostic timer — every 5s, report status until data arrives
        self.create_timer(5.0, self._diagnostics)

        self.get_logger().info("=" * 60)
        self.get_logger().info(f"OdomLogger: output={output_csv}  label={exp_label}  speed={speed_cmd}")
        self.get_logger().info("  /odom            ← QoS: BEST_EFFORT depth=1")
        self.get_logger().info("  /ground_truth/odom ← QoS: RELIABLE depth=10")
        self.get_logger().info("Menunggu data dari kedua topik ...")
        self.get_logger().info("=" * 60)

    # ------------------------------------------------------------------ #
    def _cb_odom(self, msg: Odometry):
        self._odom_msg   = msg
        self._odom_stamp = time.monotonic()

    def _cb_gt(self, msg: Odometry):
        self._gt_msg   = msg
        self._gt_stamp = time.monotonic()

    # ------------------------------------------------------------------ #
    def _log(self):
        now = time.monotonic()

        # Need both messages; reject if stale >0.15s
        if self._odom_msg is None or self._gt_msg is None:
            return
        if (now - self._odom_stamp) > 0.15 or (now - self._gt_stamp) > 0.15:
            return

        if self._t0 is None:
            self._t0 = now
            self.get_logger().info(
                ">>> Data diterima dari KEDUA topik — mulai logging. <<<"
            )

        t_s = now - self._t0

        op = self._odom_msg.pose.pose
        gp = self._gt_msg.pose.pose

        row = [
            f'{t_s:.4f}',
            f'{op.position.x:.6f}', f'{op.position.y:.6f}',
            f'{quat_to_yaw(op.orientation):.6f}',
            f'{gp.position.x:.6f}', f'{gp.position.y:.6f}',
            f'{quat_to_yaw(gp.orientation):.6f}',
            self.exp_label,
            f'{self.speed_cmd:.3f}',
        ]
        self._csv.writerow(row)
        self._f.flush()

        self._row_count += 1
        if self._row_count % 100 == 0:
            self.get_logger().info(
                f"  {self._row_count} baris | t={t_s:.1f}s | "
                f"odom=({op.position.x:.3f},{op.position.y:.3f}) "
                f"gt=({gp.position.x:.3f},{gp.position.y:.3f})"
            )

    # ------------------------------------------------------------------ #
    def _diagnostics(self):
        """Cetak status topik setiap 5s sampai data masuk."""
        if self._t0 is not None:
            return  # sudah logging, tidak perlu diagnostic lagi

        self._diag_count += 1
        now = time.monotonic()

        odom_ok = self._odom_msg is not None and (now - self._odom_stamp) < 1.0
        gt_ok   = self._gt_msg   is not None and (now - self._gt_stamp)   < 1.0

        odom_sym = "✓" if odom_ok else "✗"
        gt_sym   = "✓" if gt_ok   else "✗"

        self.get_logger().warn(
            f"[diag #{self._diag_count}] "
            f"/odom={odom_sym}  /ground_truth/odom={gt_sym}"
        )

        if not odom_ok and not gt_ok:
            self.get_logger().warn(
                "  Kedua topik belum ada. Pastikan robot stack berjalan:"
            )
            self.get_logger().warn(
                "    ros2 topic list | grep odom"
            )
        elif not odom_ok:
            self.get_logger().warn(
                "  /odom belum ada. Cek: legged_odometry_kf_node aktif?"
            )
        elif not gt_ok:
            self.get_logger().warn(
                "  /ground_truth/odom belum ada. Cek: op3_extern_controller aktif?"
            )

        # Tampilkan semua topik /odom* yang tersedia (best effort dari publisher info)
        try:
            pubs_odom = self.get_publishers_info_by_topic('/odom')
            pubs_gt   = self.get_publishers_info_by_topic('/ground_truth/odom')

            if pubs_odom:
                for p in pubs_odom:
                    rel = p.qos_profile.reliability
                    self.get_logger().info(
                        f"  Publisher /odom ditemukan: node={p.node_name}  "
                        f"reliability={rel.name}"
                    )
            else:
                self.get_logger().warn("  Tidak ada publisher pada /odom")

            if pubs_gt:
                for p in pubs_gt:
                    rel = p.qos_profile.reliability
                    self.get_logger().info(
                        f"  Publisher /ground_truth/odom ditemukan: node={p.node_name}  "
                        f"reliability={rel.name}"
                    )
            else:
                self.get_logger().warn("  Tidak ada publisher pada /ground_truth/odom")

        except Exception as e:
            self.get_logger().warn(f"  get_publishers_info error: {e}")

    # ------------------------------------------------------------------ #
    def shutdown(self):
        self._f.close()
        self.get_logger().info(f"Selesai. Total {self._row_count} baris ditulis.")


def main():
    parser = argparse.ArgumentParser(description='Fase 0: Log /odom vs /ground_truth/odom')
    parser.add_argument('--output', default='odom_log.csv',
                        help='Path file CSV output (default: odom_log.csv)')
    parser.add_argument('--label',  default='exp1',
                        help='Label eksperimen, ditulis ke kolom exp_label (default: exp1)')
    parser.add_argument('--speed',  type=float, default=0.0,
                        help='Kecepatan perintah robot (manual, untuk analisis; default: 0.0)')

    # Pisahkan argparse dari ROS args
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = OdomLogger(args.output, args.label, args.speed)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
