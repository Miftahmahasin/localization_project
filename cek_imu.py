#!/usr/bin/env python3
"""
cek_imu.py — Inspeksi nilai IMU dari Webots

Menampilkan nilai akselerasi, gyro, dan orientasi dari /robotis_op3/imu
untuk diagnosa apakah gravitasi sudah di-cancel atau belum.

Usage:
  python3 cek_imu.py
  (robot boleh diam, amati nilai |a| saat diam)

Interpretasi:
  |a| saat diam ~9.8  → gravitasi BELUM di-cancel (raw accelerometer)
  |a| saat diam ~0.0  → gravitasi SUDAH di-cancel
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu


class CekIMU(Node):
    def __init__(self):
        super().__init__('cek_imu')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Imu, '/robotis_op3/imu', self.cb, qos)
        self.count = 0
        self.get_logger().info("Menunggu data IMU dari /robotis_op3/imu ...")
        self.get_logger().info("Biarkan robot DIAM, amati nilai |a|")
        self.get_logger().info("-" * 65)

    def cb(self, msg):
        self.count += 1
        # Print setiap 10 sample (~0.1 detik pada 100Hz)
        if self.count % 10 != 1:
            return

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        mag_a = math.sqrt(ax**2 + ay**2 + az**2)

        # Quaternion → Euler
        q = msg.orientation
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x**2 + q.y**2)
        roll = math.degrees(math.atan2(sinr, cosr))

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.degrees(math.asin(sinp))

        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y**2 + q.z**2)
        yaw = math.degrees(math.atan2(siny, cosy))

        # Flag gravitasi
        if mag_a > 8.0:
            grav_flag = "⚠️  GRAVITASI BELUM CANCEL (raw)"
        elif mag_a < 1.0:
            grav_flag = "✅ gravitasi sudah di-cancel"
        else:
            grav_flag = "❓ tidak jelas"

        print(
            f"[{self.count:4d}] "
            f"a=({ax:+6.2f},{ay:+6.2f},{az:+6.2f}) |a|={mag_a:5.2f} {grav_flag}\n"
            f"       "
            f"w=({gx:+5.3f},{gy:+5.3f},{gz:+5.3f}) "
            f"euler=({roll:+6.1f},{pitch:+6.1f},{yaw:+6.1f})°"
        )

        if self.count >= 200:
            self.get_logger().info("Selesai 200 sample. Ctrl+C untuk keluar.")


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(CekIMU())
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()