#!/usr/bin/env python3
"""
diagnosa_kf_odom.py

Membandingkan output KF odometry kita vs ground truth Webots.
Jalankan saat robot sedang berjalan.

Usage:
  python3 diagnosa_kf_odom.py
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry


class KFDiagNode(Node):
    def __init__(self):
        super().__init__('kf_diag')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        self.kf_pose  = None
        self.gt_pose  = None
        self.count    = 0

        self.create_subscription(Odometry, '/odom',              self.kf_cb, qos)
        self.create_subscription(Odometry, '/ground_truth/odom', self.gt_cb, qos)
        self.create_timer(1.0, self.report)

        self.get_logger().info("=" * 60)
        self.get_logger().info("Diagnosa KF Odometry vs Ground Truth")
        self.get_logger().info("Jalankan robot maju, amati error posisi")
        self.get_logger().info("=" * 60)

    def kf_cb(self, msg):
        self.kf_pose = msg.pose.pose

    def gt_cb(self, msg):
        self.gt_pose = msg.pose.pose

    def report(self):
        self.count += 1

        if self.kf_pose is None:
            self.get_logger().warn(f"[{self.count:3d}] ⚠️  /odom belum ada data — KF belum publish")
            return

        kx = self.kf_pose.position.x
        ky = self.kf_pose.position.y
        kf_yaw = 2.0 * math.atan2(self.kf_pose.orientation.z,
                                   self.kf_pose.orientation.w)

        if self.gt_pose is not None:
            gx = self.gt_pose.position.x
            gy = self.gt_pose.position.y
            gt_yaw = 2.0 * math.atan2(self.gt_pose.orientation.z,
                                       self.gt_pose.orientation.w)
            err_x   = kx - gx
            err_y   = ky - gy
            err_pos = math.sqrt(err_x**2 + err_y**2)
            err_yaw = math.degrees(kf_yaw - gt_yaw)

            status = "✅" if err_pos < 0.1 else ("⚠️ " if err_pos < 0.3 else "❌")
            self.get_logger().info(
                f"[{self.count:3d}] {status} "
                f"KF=({kx:+.3f},{ky:+.3f}) GT=({gx:+.3f},{gy:+.3f}) "
                f"err={err_pos:.3f}m yaw_err={err_yaw:+.1f}°")
        else:
            self.get_logger().info(
                f"[{self.count:3d}] KF=({kx:+.3f},{ky:+.3f}) "
                f"yaw={math.degrees(kf_yaw):+.1f}° | GT: tidak tersedia")


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(KFDiagNode())
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()