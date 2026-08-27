#!/usr/bin/env python3
"""
surrogate_odom_node.py — Surrogate Odometry untuk Fase 1B

Tujuan: menyediakan /odom yang BENAR-BENAR BEKERJA dari ground truth,
dengan k=1/1.042 systematic scale error (odom under-report 4%), sebagai
pengganti sementara legged_odometry_kf_node yang gagal menangkap FK displacement
di gait full-stack online_walking_module.

Ini BUKAN pengganti legged_odometry; ini alat diagnostik untuk membuktikan:
  "Jika odom bekerja, apakah EKF bisa tracking? (Gate G1)"

PAKAI: ganti legged_odometry_kf_node di launch localization_v14.launch.py
       dengan node ini. Setelah G1 dikonfirmasi, kembali ke legged_odometry.

Parameter:
  k_scale   (float, default 1.042): scale factor GT→odom. odom = GT/k_scale.
  noise_pos (float, default 0.003): sigma posisi Gaussian per step (m)
  noise_yaw (float, default 0.005): sigma yaw Gaussian per step (rad)
  noise_vel (float, default 0.002): sigma velocity per step (m/s)
  zero_tf_yaw (bool, default True): publikasikan yaw=0 di TF (sama dgn legged_kf)
"""

import math, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def quat_from_yaw(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


class SurrogateOdomNode(Node):
    def __init__(self):
        super().__init__('surrogate_odom_node')

        self.declare_parameter('k_scale',    1.042)
        self.declare_parameter('noise_pos',  0.003)
        self.declare_parameter('noise_yaw',  0.005)
        self.declare_parameter('noise_vel',  0.002)
        self.declare_parameter('zero_tf_yaw', True)

        self.k       = self.get_parameter('k_scale').value
        self.sig_pos = self.get_parameter('noise_pos').value
        self.sig_yaw = self.get_parameter('noise_yaw').value
        self.sig_vel = self.get_parameter('noise_vel').value
        self.zero_tf_yaw = self.get_parameter('zero_tf_yaw').value

        self.get_logger().info(
            f"[SurrogateOdom] k={self.k:.4f}  noise_pos={self.sig_pos*1000:.1f}mm "
            f"noise_yaw={math.degrees(self.sig_yaw):.2f}°  zero_tf_yaw={self.zero_tf_yaw}")

        # Akumulasi odom dari delta GT
        self.odom_x   = 0.0
        self.odom_y   = 0.0
        self.odom_yaw = 0.0

        # GT pose sebelumnya (untuk menghitung delta)
        self.gt_x_prev   = None
        self.gt_y_prev   = None
        self.gt_yaw_prev = None

        self.rng = np.random.default_rng(seed=42)

        qos_best = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        qos_rel = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        self.sub_gt  = self.create_subscription(
            Odometry, '/ground_truth/odom', self._gt_cb, qos_best)
        self.pub_odom = self.create_publisher(Odometry, '/odom', qos_rel)
        self.tf_br    = TransformBroadcaster(self)

        self.get_logger().info("[SurrogateOdom] Ready — waiting for /ground_truth/odom")

    # ─────────────────────────────────────────────────────────────
    def _gt_cb(self, msg: Odometry):
        now = msg.header.stamp

        # Ekstrak GT pose
        gx = msg.pose.pose.position.x
        gy = msg.pose.pose.position.y
        q  = msg.pose.pose.orientation
        gt_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

        if self.gt_x_prev is None:
            self.gt_x_prev   = gx
            self.gt_y_prev   = gy
            self.gt_yaw_prev = gt_yaw
            return

        # Delta GT
        dx   = gx - self.gt_x_prev
        dy   = gy - self.gt_y_prev
        dyaw = self._angle_diff(gt_yaw, self.gt_yaw_prev)

        self.gt_x_prev   = gx
        self.gt_y_prev   = gy
        self.gt_yaw_prev = gt_yaw

        # Terapkan scale error (odom under-report)
        dx   /= self.k
        dy   /= self.k
        # yaw tidak discale (hanya pose x,y yang under-report)

        # Tambah noise Gaussian
        if self.sig_pos > 0:
            dx   += self.rng.normal(0, self.sig_pos * abs(dx + 1e-9) / 0.022)
            dy   += self.rng.normal(0, self.sig_pos * abs(dy + 1e-9) / 0.022)
        if self.sig_yaw > 0:
            dyaw += self.rng.normal(0, self.sig_yaw * abs(dyaw + 1e-9) / 0.01)

        # Akumulasi
        self.odom_x   += dx
        self.odom_y   += dy
        self.odom_yaw += dyaw

        # Ekstrak GT velocity (untuk twist)
        vx_gt = msg.twist.twist.linear.x
        vy_gt = msg.twist.twist.linear.y
        # Scale velocity sama dengan posisi
        vx = vx_gt / self.k + self.rng.normal(0, self.sig_vel)
        vy = vy_gt / self.k + self.rng.normal(0, self.sig_vel)
        wz = msg.twist.twist.angular.z

        self._publish(now, vx, vy, wz)

    # ─────────────────────────────────────────────────────────────
    def _publish(self, stamp, vx, vy, wz):
        yaw_out  = 0.0 if self.zero_tf_yaw else self.odom_yaw
        qx, qy, qz, qw = quat_from_yaw(yaw_out)

        odom           = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'

        odom.pose.pose.position.x  = self.odom_x
        odom.pose.pose.position.y  = self.odom_y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Covariance pose (sama dengan legged_odometry default)
        cov_p = [0.0] * 36
        cov_p[0]  = 0.01   # x
        cov_p[7]  = 0.01   # y
        cov_p[35] = 0.05   # yaw
        odom.pose.covariance = cov_p

        odom.twist.twist.linear.x  = vx
        odom.twist.twist.linear.y  = vy
        odom.twist.twist.angular.z = wz

        cov_t = [0.0] * 36
        cov_t[0]  = 0.01
        cov_t[7]  = 0.01
        cov_t[35] = 0.1
        odom.twist.covariance = cov_t

        self.pub_odom.publish(odom)

        # TF odom→base_link
        tf = TransformStamped()
        tf.header.stamp    = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id  = 'base_link'
        tf.transform.translation.x = self.odom_x
        tf.transform.translation.y = self.odom_y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_br.sendTransform(tf)

    @staticmethod
    def _angle_diff(a, b):
        d = a - b
        while d >  math.pi: d -= 2 * math.pi
        while d < -math.pi: d += 2 * math.pi
        return d


def main(args=None):
    rclpy.init(args=args)
    node = SurrogateOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
