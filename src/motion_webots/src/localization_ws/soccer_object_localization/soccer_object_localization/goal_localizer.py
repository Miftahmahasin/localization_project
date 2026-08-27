#!/usr/bin/env python3
"""
goal_localizer.py — Goal-Width Based Localization  [v1.1]
==========================================================
Referensi: Paper 2 (Schulz & Behnke, 2012) + Geometri kamera pinhole

MOTIVASI:
  Di area dekat gawang (x_robot > 2.5m), semua sensor lain gagal:
  - AMCL: scan laser mendeteksi jaring → particle collapse
  - cox_registration: terlalu sedikit garis terlihat
  - crossing_detector: T-crossing penalty box keluar FOV

  Namun: GAWANG SENDIRI adalah landmark paling jelas di area ini!
  Tiang gawang putih, terstruktur, dan lebar gawang = 2.6m (fixed).

PRINSIP GEOMETRI:
  Lebar gawang di pixel: W_px
  Focal length: f = 793.3px
  Lebar gawang real: W_m = 2.6m

  Z = f * W_m / W_px  (jarak kamera ke bidang gawang, sepanjang optical axis)

  Posisi absolut robot di map (dengan koreksi yaw):
    near right goal (gawang di x=+fhl, y=0):
      est_x = fhl - Z * cos(yaw)
      est_y = -Z * sin(yaw) - offset_y_m / cos(yaw)

    near left goal (gawang di x=-fhl, y=0):
      est_x = -fhl - Z * cos(yaw)   [= -fhl + Z ketika yaw≈π]
      est_y = -Z * sin(yaw) - offset_y_m / cos(yaw)

  offset_y_m = -(goal_center_px - img_center_px) * Z / focal

AKTIVASI:
  Hanya aktif ketika:
  a. Robot estimasi dekat gawang: |robot_x| > activation_x_m
  b. Robot menghadap ke arah gawang: |cos(yaw)| > cos(max_yaw_deg)
     (near_right_goal: cos(yaw) > 0.5 → |yaw| < 60°)
     (near_left_goal:  cos(yaw) < -0.5 → |yaw - π| < 60°)
  c. Gawang terdeteksi dengan confidence cukup
  d. Cooldown sejak koreksi terakhir

CHANGELOG:
  v1.1 — Tiga perbaikan:
    FIX1: Tambah yaw guard — sebelumnya aktif hanya dari robot_x tanpa cek
          arah robot, menyebabkan false pose injection jika robot membelakangi
          gawang. Sekarang hanya aktif jika robot menghadap gawang (±60°).
    FIX2: Perbaiki formula est_x dan est_y:
          - est_x lama: fhl - Z (tanpa faktor cos(yaw))
          - est_x baru: fhl - Z*cos(yaw)
          - est_y lama: robot_y + offset_y_m*cos(yaw)  (salah: bergantung odom)
          - est_y baru: -Z*sin(yaw) - offset_y_m/cos(yaw)  (absolut dari geometri)
    FIX3: Naikkan covariance (cov_x: 0.06→0.10, cov_y: 0.12→0.20) agar
          koreksi goal_localizer tidak terlalu agresif menimpa AMCL.
  v1.0 — Implementasi awal

TOPIC:
  Sub: /robotis_op3/camera/image_rect  (gambar rektifikasi)
  Sub: /odometry/filtered               (pose robot saat ini)
  Pub: /initialpose                     (koreksi pose ke EKF/AMCL)
  Pub: /goal_localizer_debug            (debug image)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from typing import Optional, Tuple


class GoalLocalizer(Node):
    def __init__(self):
        super().__init__('goal_localizer')

        # ── Parameter ────────────────────────────────────────────────────────
        self.declare_parameter('image_width',         1280)
        self.declare_parameter('image_height',         720)
        self.declare_parameter('focal_length',         793.3)

        # Lapangan
        self.declare_parameter('field_half_len',        4.5)   # m
        self.declare_parameter('goal_width_m',          2.6)   # m — lebar gawang
        self.declare_parameter('goal_height_m',         1.2)   # m — tinggi gawang

        # Aktivasi: hanya aktif jika robot diperkirakan dekat gawang DAN menghadap gawang
        self.declare_parameter('activation_x_m',        2.0)   # aktif jika |robot_x| > ini
        self.declare_parameter('max_yaw_deg',           60.0)  # [FIX1] max sudut dari arah gawang
        self.declare_parameter('min_goal_width_px',     80)    # min lebar gawang terdeteksi
        self.declare_parameter('max_goal_dist_m',        3.0)  # max jarak ke gawang (sanity)

        # Deteksi tiang gawang
        self.declare_parameter('white_threshold',       200)   # threshold BGR
        self.declare_parameter('post_min_height_ratio', 0.08)  # min tinggi tiang (fraksi h)
        self.declare_parameter('post_max_width_px',     40)    # max lebar tiang (px)
        self.declare_parameter('post_roi_top',          0.02)  # ROI atas untuk tiang
        self.declare_parameter('post_roi_bottom',       0.45)  # ROI bawah untuk tiang

        # Covariance koreksi — [FIX3] dinaikkan agar tidak terlalu agresif
        self.declare_parameter('cov_x',                0.10)  # m² — x (dinaikkan 0.06→0.10)
        self.declare_parameter('cov_y',                0.20)  # m² — y (dinaikkan 0.12→0.20)
        self.declare_parameter('cov_yaw',              0.10)  # rad²
        self.declare_parameter('cooldown_sec',          1.5)
        self.declare_parameter('publish_debug',         True)

        # ── Load ─────────────────────────────────────────────────────────────
        p = self.get_parameter
        self.img_w          = p('image_width').value
        self.img_h          = p('image_height').value
        self.focal          = p('focal_length').value
        self.fhl            = p('field_half_len').value
        self.goal_w_m       = p('goal_width_m').value
        self.goal_h_m       = p('goal_height_m').value
        self.act_x          = p('activation_x_m').value
        self.max_yaw_cos    = math.cos(math.radians(p('max_yaw_deg').value))  # [FIX1]
        self.min_goal_px    = p('min_goal_width_px').value
        self.max_goal_dist  = p('max_goal_dist_m').value
        self.white_thr      = p('white_threshold').value
        self.post_min_h     = p('post_min_height_ratio').value
        self.post_max_w     = p('post_max_width_px').value
        self.roi_top        = p('post_roi_top').value
        self.roi_bot        = p('post_roi_bottom').value
        self.cov_x          = p('cov_x').value
        self.cov_y          = p('cov_y').value
        self.cov_yaw        = p('cov_yaw').value
        self.cooldown       = p('cooldown_sec').value
        self.pub_debug      = p('publish_debug').value

        # ── State ─────────────────────────────────────────────────────────────
        self.robot_x   = 0.0
        self.robot_y   = 0.0
        self.robot_yaw = 0.0
        self.last_corr = 0.0
        self.n_corr    = 0
        self.bridge    = CvBridge()

        # ── Sub / Pub ─────────────────────────────────────────────────────────
        self.sub_img = self.create_subscription(
            Image, '/robotis_op3/camera/image_rect',
            self._cb_image, qos_profile_sensor_data)
        self.sub_odom = self.create_subscription(
            Odometry, '/odometry/filtered',
            self._cb_odom, qos_profile_sensor_data)

        qos_latch = QoSProfile(depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_pose  = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', qos_latch)
        self.pub_debug_img = self.create_publisher(Image, '/goal_localizer_debug', 10)

        self.get_logger().info(
            f'GoalLocalizer v1.0 started  '
            f'goal_w={self.goal_w_m}m  activation_x={self.act_x}m')

    # ─────────────────────────────────────────────────────────────────────────
    def _cb_odom(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2.0*(q.w*q.z + q.x*q.y),
            1.0 - 2.0*(q.y*q.y + q.z*q.z))

    # ─────────────────────────────────────────────────────────────────────────
    def _cb_image(self, msg: Image):
        # [FIX1] Hanya aktif jika robot dekat gawang DAN menghadap ke arahnya
        # near_right_goal: robot_x > act_x DAN cos(yaw) > cos(max_yaw_deg) → |yaw| < max_yaw_deg
        # near_left_goal:  robot_x < -act_x DAN cos(yaw) < -cos(max_yaw_deg) → |yaw-π| < max_yaw_deg
        cos_yaw = math.cos(self.robot_yaw)
        near_right_goal = self.robot_x >  self.act_x and cos_yaw >  self.max_yaw_cos
        near_left_goal  = self.robot_x < -self.act_x and cos_yaw < -self.max_yaw_cos
        if not (near_right_goal or near_left_goal):
            return

        # Cooldown check
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_corr < self.cooldown:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return

        # Deteksi tiang gawang
        result = self._detect_goal_posts(frame)
        if result is None:
            return

        left_x, right_x, post_top_y = result
        goal_width_px = right_x - left_x
        if goal_width_px < self.min_goal_px:
            return

        # ── Hitung jarak ke gawang (pinhole) ─────────────────────────────────
        # Z = focal * goal_width_m / goal_width_px
        Z = self.focal * self.goal_w_m / goal_width_px
        if Z > self.max_goal_dist or Z < 0.3:
            return

        # ── Hitung offset Y dari center gawang ───────────────────────────────
        goal_center_px = (left_x + right_x) / 2.0
        img_center_px  = self.img_w / 2.0
        # Positive offset_y → gawang sedikit ke kiri dari tengah frame
        # → robot sedikit ke kanan dari center gawang
        offset_px  = goal_center_px - img_center_px
        offset_y_m = -offset_px * Z / self.focal  # negatif: kanan frame = kecil y

        # ── Hitung posisi robot di map frame ─────────────────────────────────
        # [FIX2] Formula geometri pinhole yang benar dengan koreksi yaw.
        # Z adalah jarak sepanjang optical axis kamera ke bidang gawang.
        # Derivasi: goal_map = robot_map + Z*forward_robot + offset_y_m*left_robot
        #   forward_robot = [cos(yaw), sin(yaw)]
        #   left_robot    = [-sin(yaw), cos(yaw)]
        #   goal center   = (±fhl, 0) → selesaikan untuk robot_map
        #
        # near right goal (goal di (fhl, 0)):
        #   est_x = fhl - Z*cos(yaw)
        #   est_y = -Z*sin(yaw) - offset_y_m/cos(yaw)
        # near left goal (goal di (-fhl, 0)):
        #   est_x = -fhl - Z*cos(yaw)   [= -fhl + Z saat yaw=π karena cos(π)=-1]
        #   est_y = -Z*sin(yaw) - offset_y_m/cos(yaw)  [sama]
        #
        # Catatan: cos_yaw dijamin != 0 oleh yaw guard di atas (|cos_yaw| > max_yaw_cos)
        sin_yaw = math.sin(self.robot_yaw)
        if near_right_goal:
            est_x = self.fhl - Z * cos_yaw
        else:
            est_x = -self.fhl - Z * cos_yaw
        est_y = -Z * sin_yaw - offset_y_m / cos_yaw

        # Sanity check: estimasi harus masuk akal
        if abs(est_x) > self.fhl + 0.5:
            return
        if abs(est_y) > 3.5:
            return

        # Confidence berdasarkan kualitas deteksi
        conf = min(1.0, goal_width_px / (self.min_goal_px * 3))

        # ── Publish /initialpose ──────────────────────────────────────────────
        pw = PoseWithCovarianceStamped()
        pw.header.frame_id = 'map'
        pw.header.stamp    = self.get_clock().now().to_msg()
        pw.pose.pose.position.x = est_x
        pw.pose.pose.position.y = est_y
        pw.pose.pose.position.z = 0.0

        cy = math.cos(self.robot_yaw / 2.0)
        sy = math.sin(self.robot_yaw / 2.0)
        pw.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=sy, w=cy)

        cov = [0.0] * 36
        # X: lebih pasti (Z dihitung dari lebar gawang, akurat)
        cov[0]  = self.cov_x / max(0.1, conf)
        # Y: kurang pasti (offset lateral dari center gawang)
        cov[7]  = self.cov_y / max(0.1, conf)
        cov[14] = 0.01
        cov[21] = 0.01
        cov[28] = 0.01
        cov[35] = self.cov_yaw
        pw.pose.covariance = cov

        self.pub_pose.publish(pw)
        self.last_corr  = now
        self.n_corr    += 1

        self.get_logger().info(
            f'[GOAL_LOC #{self.n_corr}] '
            f'goal_w={goal_width_px:.0f}px  Z={Z:.2f}m  '
            f'est=({est_x:.2f},{est_y:.2f})  conf={conf:.2f}  '
            f'robot_x={self.robot_x:.2f}',
            throttle_duration_sec=0.5)

        if self.pub_debug:
            self._publish_debug(frame, left_x, right_x, post_top_y,
                                est_x, est_y, Z, msg.header.stamp)

    # ─────────────────────────────────────────────────────────────────────────
    def _detect_goal_posts(self, frame: np.ndarray) -> Optional[Tuple[int, int, int]]:
        """
        Deteksi dua tiang gawang di frame.
        Return: (left_x, right_x, top_y) dalam pixel, atau None.

        Algoritma:
        1. Threshold putih di ROI atas frame (area gawang)
        2. Cari komponen connected vertikal yang tinggi dan sempit
        3. Pilih 2 yang paling kiri dan paling kanan
        """
        h, w = frame.shape[:2]
        roi_y1 = int(h * self.roi_top)
        roi_y2 = int(h * self.roi_bot)
        roi = frame[roi_y1:roi_y2, :]

        # White mask: semua channel > threshold
        white = (
            (roi[:, :, 0] > self.white_thr) &
            (roi[:, :, 1] > self.white_thr) &
            (roi[:, :, 2] > self.white_thr)
        ).astype(np.uint8) * 255

        # Morphological: close untuk isi celah kecil di tiang
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 5))
        white = cv2.morphologyEx(white, cv2.MORPH_CLOSE, kernel)

        # Connected components
        n, labels, stats, _ = cv2.connectedComponentsWithStats(white, 8)

        min_h_px = int((roi_y2 - roi_y1) * self.post_min_h)
        posts = []

        for i in range(1, n):
            x  = stats[i, cv2.CC_STAT_LEFT]
            y  = stats[i, cv2.CC_STAT_TOP]
            cw = stats[i, cv2.CC_STAT_WIDTH]
            ch = stats[i, cv2.CC_STAT_HEIGHT]
            area = stats[i, cv2.CC_STAT_AREA]

            # Tiang: tinggi > min, sempit, aspect ratio vertikal
            if ch < min_h_px:
                continue
            if cw > self.post_max_w:
                continue
            if ch < cw * 2:  # aspect ratio: harus lebih tinggi dari lebar
                continue
            # Density cukup (bukan noise)
            if area < cw * ch * 0.3:
                continue

            cx = x + cw // 2
            top_y = y + roi_y1
            posts.append((cx, top_y, ch))

        if len(posts) < 2:
            return None

        # Urutkan berdasarkan x, ambil paling kiri dan kanan
        posts.sort(key=lambda p: p[0])
        left_post  = posts[0]
        right_post = posts[-1]

        # Validasi: jarak antar tiang masuk akal
        gap = right_post[0] - left_post[0]
        if gap < self.min_goal_px:
            return None
        # Tidak terlalu lebar (jika lebih dari 80% frame, kemungkinan FP)
        if gap > self.img_w * 0.85:
            return None

        top_y = min(left_post[1], right_post[1])
        return (left_post[0], right_post[0], top_y)

    # ─────────────────────────────────────────────────────────────────────────
    def _publish_debug(self, frame, left_x, right_x, top_y,
                       est_x, est_y, Z, stamp):
        debug = frame.copy()
        h = debug.shape[0]
        roi_y1 = int(h * self.roi_top)
        roi_y2 = int(h * self.roi_bot)

        # ROI box
        cv2.rectangle(debug, (0, roi_y1), (frame.shape[1]-1, roi_y2),
                      (80, 80, 80), 1)
        # Tiang kiri dan kanan
        cv2.line(debug, (left_x,  roi_y1), (left_x,  roi_y2), (0, 255, 255), 2)
        cv2.line(debug, (right_x, roi_y1), (right_x, roi_y2), (0, 255, 255), 2)
        # Garis lebar gawang
        mid_y = (roi_y1 + top_y) // 2
        cv2.line(debug, (left_x, mid_y), (right_x, mid_y), (0, 200, 255), 1)
        cv2.putText(debug, f'{right_x-left_x}px',
                    ((left_x+right_x)//2 - 20, mid_y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 255), 1)
        # Info
        cv2.putText(debug, f'GOAL_LOC  Z={Z:.2f}m  est=({est_x:.2f},{est_y:.2f})',
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        try:
            dbg_msg = self.bridge.cv2_to_imgmsg(debug, 'bgr8')
            dbg_msg.header.stamp = stamp
            self.pub_debug_img.publish(dbg_msg)
        except Exception:
            pass


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = GoalLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()