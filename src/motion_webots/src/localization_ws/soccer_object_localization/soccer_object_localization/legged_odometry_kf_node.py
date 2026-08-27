#!/usr/bin/env python3
"""
legged_odometry_kf_node.py  — v3.2 (FK-Contact Odometry)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
v3.1 FIX: Double-support stance detection
  Bug: stance detection hanya membandingkan tinggi kaki (fz_L vs fz_R).
  Saat berdiri atau fase double-support, kedua kaki sama tinggi → |diff|=0
  → kondisi TIDAK PERNAH terpenuhi → stance=.. selalu → KF tidak update posisi
  → /odom stuck di (0,0) → Cox registration anchor di center → AMCL stuck center.
  Fix: jika |diff| < margin, tandai KEDUA kaki sebagai stance (double-support).

v3.2 FIX: Absolute-height stance detection untuk Webots simulation
  Bug v3.1: Di Webots, kaki kiri dan kanan FK-Z selalu hampir sama (|diff|<1cm)
  karena gaya berjalan simetris → double-support sepanjang waktu → anchor tidak
  pernah diperbarui → posisi tetap (0,0) selamanya.
  Fix: mode simulasi menggunakan perbandingan tiap kaki vs minimum historisnya
  sendiri (absolute stance). Kaki di tanah = fz ≈ minimum; kaki terangkat = fz
  lebih tinggi dari minimum. Menggunakan leaky minimum agar tidak drift selamanya.


Prinsip inti (dari paper):
  • Saat kaki TOUCHDOWN → catat W_p_foot = Wp + W_R_B @ Bp_i
  • Saat kaki CONTACT   → KF update: Wp = W_p_foot - W_R_B @ Bp_i
  • Yaw langsung dari IMU (tidak diestimasi KF)
  • Prediksi TANPA integrasi akselerasi (menghindari drift)

State: x = [Wp_x, Wp_y, Wv_x, Wv_y]  (4D, bukan 12D)
Kenapa 4D: flat_ground → z=0, W_p_foot disimpan terpisah (bukan state),
           H matrix jadi linear sederhana, tidak diverge.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

# Panjang segmen kaki efektif untuk FK sagittal 2D
# Nilai fisik URDF: L_THIGH=0.11015, L_SHANK=0.110, L_FOOT=0.0305
# Nilai efektif = nilai_URDF × 0.917 karena FK sagittal 2D tidak memodelkan:
#   - Hip roll (kaki tidak persis di bidang sagittal)
#   - Servo compliance (joint tidak mencapai nilai yang dicommand tepat)
#   - Foot deformation saat menumpu berat badan
# Faktor 0.917 = 1/1.090 dikalibrasi dari eksperimen GT vs KF
L_THIGH = 0.1010   # 0.11015 × 0.917
L_SHANK = 0.1009   # 0.110   × 0.917
L_FOOT  = 0.0280   # 0.0305  × 0.917

JOINT_L_HIP  = 'l_hip_pitch'
JOINT_L_KNEE = 'l_knee'
JOINT_L_ANK  = 'l_ank_pitch'
JOINT_R_HIP  = 'r_hip_pitch'
JOINT_R_KNEE = 'r_knee'
JOINT_R_ANK  = 'r_ank_pitch'

# Hip roll (untuk FK 3D — opsional, saat ini tidak dipakai)
JOINT_L_HIP_R = 'l_hip_roll'
JOINT_R_HIP_R = 'r_hip_roll'

REQUIRED_JOINTS = [JOINT_L_HIP, JOINT_L_KNEE, JOINT_L_ANK,
                   JOINT_R_HIP, JOINT_R_KNEE, JOINT_R_ANK]


def euler_to_rot(roll, pitch, yaw):
    cr,sr = math.cos(roll),math.sin(roll)
    cp,sp = math.cos(pitch),math.sin(pitch)
    cy,sy = math.cos(yaw),math.sin(yaw)
    Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    Rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    return Rz@Ry@Rx


def yaw_to_quat(yaw):
    q = Quaternion()
    q.w = math.cos(yaw/2.0)
    q.z = math.sin(yaw/2.0)
    q.x = 0.0; q.y = 0.0
    return q


class LeggedOdometryKF(Node):
    def __init__(self):
        super().__init__('legged_odometry_kf')

        self.declare_parameter('base_frame',        'base_link')
        self.declare_parameter('odom_frame',        'odom')
        self.declare_parameter('publish_rate',      100.0)
        self.declare_parameter('simulation_mode',   True)
        self.declare_parameter('imu_topic',         '')
        self.declare_parameter('joint_topic',       '')
        self.declare_parameter('stance_knee_thresh', 0.3)
        self.declare_parameter('q_pos',  0.005)
        self.declare_parameter('q_vel',  0.05)
        self.declare_parameter('r_pos',  0.002)
        # zero_tf_yaw: if True, publish yaw=0 in odom→base_link TF (and /odom).
        # Prevents drifting Webots IMU yaw from corrupting AMCL motion model.
        # EKF gets yaw only from AMCL (pose0) when this is True.
        self.declare_parameter('zero_tf_yaw', False)
        # lift_thresh (sim_mode): hysteresis margin (m) for relative-foot-height stance detection.
        # Foot A is in stance when fz_A < fz_B - lift_thresh (A clearly lower than B).
        # 3mm default: wide enough to reject FK noise, narrow enough to catch 6-10mm gait diff.
        self.declare_parameter('lift_thresh', 0.003)

        self.base_frame  = self.get_parameter('base_frame').value
        self.odom_frame  = self.get_parameter('odom_frame').value
        self.pub_rate    = self.get_parameter('publish_rate').value
        self.knee_thresh = self.get_parameter('stance_knee_thresh').value
        self.q_pos       = self.get_parameter('q_pos').value
        self.q_vel       = self.get_parameter('q_vel').value
        self.r_pos       = self.get_parameter('r_pos').value
        self.zero_tf_yaw  = self.get_parameter('zero_tf_yaw').value
        self.lift_thresh  = self.get_parameter('lift_thresh').value

        sim_mode       = self.get_parameter('simulation_mode').value
        self.sim_mode  = sim_mode
        imu_override   = self.get_parameter('imu_topic').value
        joint_override = self.get_parameter('joint_topic').value
        imu_topic   = imu_override   or ('/robotis_op3/imu'          if sim_mode else '/robotis/open_cr/imu')
        joint_topic = joint_override or ('/robotis_op3/joint_states' if sim_mode else '/robotis/present_joint_states')

        # KF State: [Wp_x, Wp_y, Wv_x, Wv_y]
        self.x = np.zeros(4)
        self.P = np.eye(4) * 0.1

        # Referensi posisi kaki di world frame (anchor saat touchdown)
        self.W_p_foot_R = None
        self.W_p_foot_L = None

        self.prev_r_stance = False
        self.prev_l_stance = False

        self.W_R_B   = np.eye(3)
        self.imu_yaw = 0.0
        self.imu_yaw0 = None
        self.has_imu  = False

        self.joint_pos   = {}
        self.joint_found = False
        self.Bp_R = np.zeros(3)
        self.Bp_L = np.zeros(3)
        self.r_stance = True   # start in DS (robot standing)
        self.l_stance = True
        self.last_time = None

        self.tf_br = TransformBroadcaster(self)
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(Imu,        imu_topic,   self.imu_cb,   qos)
        self.create_subscription(JointState, joint_topic, self.joint_cb, qos)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)
        self.create_timer(1.0/self.pub_rate, self.kf_step)

        self.get_logger().info("=" * 60)
        self.get_logger().info("✅ Legged Odometry KF v3.2 (FK-Contact, 4D state)")
        self.get_logger().info(f"   Mode: {'SIMULASI (absolute stance)' if sim_mode else 'ROBOT FISIK (diff stance)'}")
        self.get_logger().info(f"   IMU: {imu_topic} | Joint: {joint_topic}")
        self.get_logger().info(f"   TF: {self.odom_frame}→{self.base_frame}")
        self.get_logger().info("=" * 60)

    def imu_cb(self, msg):
        q = msg.orientation
        sinr = 2.0*(q.w*q.x + q.y*q.z);  cosr = 1.0-2.0*(q.x**2+q.y**2)
        roll = math.atan2(sinr, cosr)
        sinp = max(-1.0,min(1.0, 2.0*(q.w*q.y - q.z*q.x)))
        pitch = math.asin(sinp)
        siny = 2.0*(q.w*q.z + q.x*q.y);  cosy = 1.0-2.0*(q.y**2+q.z**2)
        yaw = math.atan2(siny, cosy)

        if self.imu_yaw0 is None:
            self.imu_yaw0 = yaw
            self.get_logger().info(
                f"[IMU] init: r={math.degrees(roll):.1f}° "
                f"p={math.degrees(pitch):.1f}° y={math.degrees(yaw):.1f}°")

        self.imu_yaw = yaw - self.imu_yaw0
        self.W_R_B   = euler_to_rot(roll, pitch, self.imu_yaw)
        self.has_imu = True

    def joint_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.joint_pos[name] = pos
        if not self.joint_found:
            if all(j in self.joint_pos for j in REQUIRED_JOINTS):
                self.joint_found = True
                self.get_logger().info("✅ Semua joint ditemukan")
            else:
                return
        j = self.joint_pos
        # FK dengan konvensi axis dari URDF (xacro + proto):
        # KAKI KIRI:  l_hip_pitch +Y, l_knee +Y, l_ank_pitch -Y
        #   → hip, knee langsung; ankle DI-NEGASI
        # KAKI KANAN: r_hip_pitch -Y, r_knee -Y, r_ank_pitch +Y
        #   → hip, knee DI-NEGASI; ankle langsung (setelah negasi hip/knee, arah konsisten)
        self.Bp_L = self._fk( j[JOINT_L_HIP],  j[JOINT_L_KNEE], -j[JOINT_L_ANK])
        self.Bp_R = self._fk(-j[JOINT_R_HIP], -j[JOINT_R_KNEE],  j[JOINT_R_ANK])

        fz_L = self.Bp_L[2]
        fz_R = self.Bp_R[2]

        if self.sim_mode:
            # v3.3: Relative-height hysteresis stance detection untuk Webots.
            # fz(L) - fz(R): negatif = L lebih rendah = L stance; positif = R lebih rendah = R stance.
            # Data nyata walking OP3: |diff| ≈ 6-10mm, bergantian tanda setiap half-step.
            # Hysteresis (lift_thresh=3mm) mencegah flip-flop akibat noise FK.
            # Tidak memerlukan minimum historis — murni perbandingan antar kaki saat ini.
            diff = fz_L - fz_R  # negative = L lower = L in stance
            m = self.lift_thresh
            if self.r_stance and not self.l_stance:   # sedang R-only: tetap R kecuali L jelas lebih rendah
                if diff < -m:
                    self.r_stance = False
                    self.l_stance = True
            elif self.l_stance and not self.r_stance:  # sedang L-only: tetap L kecuali R jelas lebih rendah
                if diff > m:
                    self.r_stance = True
                    self.l_stance = False
            else:                                      # DS atau awal: tentukan dari perbedaan saat ini
                if diff > m:
                    self.r_stance = True
                    self.l_stance = False
                elif diff < -m:
                    self.r_stance = False
                    self.l_stance = True
                # else: stay DS
        else:
            # Real robot: bandingkan kiri vs kanan dengan margin 1cm
            margin = 0.01
            diff = fz_L - fz_R
            if abs(diff) < margin:
                self.l_stance = True
                self.r_stance = True
            else:
                self.l_stance = diff < -margin
                self.r_stance = diff >  margin

    def _fk(self, hip, knee, ank):
        # Axis x body OP3 menghadap ke belakang, sehingga fx dinegasi
        # agar displacement maju = +x world
        fx = -(L_THIGH*math.sin(hip) + L_SHANK*math.sin(hip+knee)
             + L_FOOT*math.sin(hip+knee+ank))
        fz = -(L_THIGH*math.cos(hip) + L_SHANK*math.cos(hip+knee)
             + L_FOOT*math.cos(hip+knee+ank))
        return np.array([fx, 0.0, fz])

    def _init_foot_positions(self):
        Wp = self.x[0:2]
        self.W_p_foot_R = Wp + (self.W_R_B @ self.Bp_R)[0:2]
        self.W_p_foot_L = Wp + (self.W_R_B @ self.Bp_L)[0:2]

    def kf_step(self):
        if not self.has_imu or not self.joint_found:
            return

        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            self._init_foot_positions()
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0 or dt > 0.5:
            return

        # 1. Touchdown detection → update foot anchor
        Wp = self.x[0:2]
        if self.r_stance and not self.prev_r_stance:
            self.W_p_foot_R = Wp + (self.W_R_B @ self.Bp_R)[0:2]
        if self.l_stance and not self.prev_l_stance:
            self.W_p_foot_L = Wp + (self.W_R_B @ self.Bp_L)[0:2]

        # 2. KF Predict (tanpa akselerasi)
        A = np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]])
        self.x = A @ self.x
        Q = np.diag([self.q_pos, self.q_pos, self.q_vel, self.q_vel])
        self.P = A @ self.P @ A.T + Q

        # 3. KF Update dari constraint kontak
        H = np.array([[1,0,0,0],[0,1,0,0]])
        R = np.eye(2) * self.r_pos
        z_list = []

        if self.r_stance and self.W_p_foot_R is not None:
            W_offset_R = (self.W_R_B @ self.Bp_R)[0:2]
            z_list.append(self.W_p_foot_R - W_offset_R - self.x[0:2])

        if self.l_stance and self.W_p_foot_L is not None:
            W_offset_L = (self.W_R_B @ self.Bp_L)[0:2]
            z_list.append(self.W_p_foot_L - W_offset_L - self.x[0:2])

        if z_list:
            # Rata-rata dari semua constraint aktif
            z = np.mean(z_list, axis=0)
            S = H @ self.P @ H.T + R
            K = self.P @ H.T @ np.linalg.inv(S)
            self.x = self.x + K @ z
            self.P = (np.eye(4) - K @ H) @ self.P
            self.P = 0.5*(self.P + self.P.T)
            np.fill_diagonal(self.P, np.clip(np.diag(self.P), 0.0, 100.0))

        # 4. Safety
        if not np.all(np.isfinite(self.x)):
            self.get_logger().warn("[KF] NaN terdeteksi, reset")
            self.x = np.zeros(4)
            self.P = np.eye(4) * 0.1
            self._init_foot_positions()
        else:
            self._publish(now)

        self.prev_r_stance = self.r_stance
        self.prev_l_stance = self.l_stance

    def _publish(self, now):
        now_msg = now.to_msg()
        yaw = self.imu_yaw

        tf_yaw = 0.0 if self.zero_tf_yaw else yaw
        tf = TransformStamped()
        tf.header.stamp    = now_msg
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id  = self.base_frame
        tf.transform.translation.x = float(self.x[0])
        tf.transform.translation.y = float(self.x[1])
        tf.transform.translation.z = 0.0
        tf.transform.rotation = yaw_to_quat(tf_yaw)
        self.tf_br.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp    = now_msg
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame
        odom.pose.pose.position.x  = float(self.x[0])
        odom.pose.pose.position.y  = float(self.x[1])
        odom.pose.pose.position.z  = 0.0
        odom.pose.pose.orientation = yaw_to_quat(tf_yaw)
        odom.twist.twist.linear.x  = float(self.x[2])
        odom.twist.twist.linear.y  = float(self.x[3])
        odom.pose.covariance[0]  = float(self.P[0,0])
        odom.pose.covariance[7]  = float(self.P[1,1])
        odom.pose.covariance[35] = 0.01
        odom.twist.covariance[0]  = float(self.P[2,2])
        odom.twist.covariance[7]  = float(self.P[3,3])
        odom.twist.covariance[35] = 0.01
        self.odom_pub.publish(odom)

        if int(now.nanoseconds/2e9) != getattr(self,'_last_log',-1):
            self._last_log = int(now.nanoseconds/2e9)
            s = ('R' if self.r_stance else '.')+('L' if self.l_stance else '.') + (' DS' if self.r_stance and self.l_stance else '')
            diff_mm = (self.Bp_L[2] - self.Bp_R[2]) * 1000
            self.get_logger().info(
                f"[KF] pos=({self.x[0]:+.3f},{self.x[1]:+.3f}) "
                f"vel=({self.x[2]:+.3f},{self.x[3]:+.3f}) "
                f"stance={s} yaw={math.degrees(yaw):+.1f}° "
                f"fzDiff(L-R)={diff_mm:+.1f}mm thresh=±{self.lift_thresh*1000:.1f}mm")


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(LeggedOdometryKF())
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()