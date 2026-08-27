#!/usr/bin/env python3
"""
localization_v14_odom_only.launch.py  [DIAGNOSTIK — Fase 0 Diagnosa 1]
=======================================================================
EKF hanya menerima odom0 velocity (TIDAK pose0/AMCL, TIDAK pose1/Cox).

TUJUAN: Membuktikan apakah SELURUH error 3m berasal dari AMCL/Cox loop,
        atau ada sisa error dari dead-reckoning odom itu sendiri.

Hipotesis (harus dibuktikan dengan run):
  EKF RMSE ~0.12m,  x-capture ~100%
  → membuktikan odom viable sebagai sumber pose tanpa AMCL/Cox

Jika hipotesis BENAR: error 3m sepenuhnya akibat feedback tangle EKF↔AMCL↔Cox.
Jika hipotesis SALAH: ada masalah mendasar pada odom (IMU drift, slip, dsb.)

Perbedaan dari localization_v14_walkparam.launch.py:
  - ekf_config → ekf_odom_only.yaml  (pose0=off, pose1=off)
  - TIDAK jalan: AMCL, Cox, perception stack, scan_gate, dll.
  - HANYA: static_tf, walking_param_odom, odom_throttle, map_server, ekf

Cara run:
  1. Jalankan Webots + op3_full_bringup seperti biasa
  2. ros2 launch soccer_object_localization localization_v14_odom_only.launch.py
  3. Beri perintah walking dari GUI (x_amplitude=0.015m, Apply, Start)
  4. Jalankan localization_evaluator.py >=180s
  5. Laporkan: x-capture EKF, RMSE, plot EKF vs GT vs odom
"""
import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_loc    = get_package_share_directory("soccer_object_localization")
    map_file   = os.path.join(pkg_loc, "maps", "soccer_field.yaml")
    ekf_config = os.path.join(pkg_loc, "config", "ekf_odom_only.yaml")

    static_tf_publisher = Node(
        package="op3_utra_bridge", executable="op3_static_transforms.py",
        name="static_tf_publisher", output="screen"
    )

    walking_param_odom_node = Node(
        package="soccer_object_localization", executable="walking_param_odom_node",
        name="walking_param_odom", output="screen",
        parameters=[{
            # STOPGAP: 0.976/0.636=1.536. Menyerap mismatch formula vx=2*x_amp/period
            # vs gerak aktual Webots (GT/formula=1.576x, period=0.650s, x_amp=0.015m).
            # RAPUH: hanya valid untuk x_amp=0.015m operasi saat ini.
            # Untuk x_amp variabel (ball_follower 0.005-0.040m), model slip nonlinear
            # dibutuhkan — ditangani koreksi absolut Fase 2, BUKAN di sini.
            "slip_ratio":    1.536,
            "zero_tf_yaw":   True,
            "imu_topic":    "/robotis_op3/imu",
            "param_timeout": 300.0,
        }]
    )

    odom_throttle_node = Node(
        package="topic_tools", executable="throttle", name="odom_throttle",
        output="screen",
        arguments=["messages", "/odom", "20.0", "/odom_throttled"],
    )

    map_server_node = Node(
        package="nav2_map_server", executable="map_server", name="map_server",
        output="screen",
        parameters=[{"yaml_filename": map_file, "topic_name": "map", "frame_id": "map"}]
    )

    ekf_node = Node(
        package="robot_localization", executable="ekf_node",
        name="ekf_filter_node", output="screen",
        parameters=[ekf_config],
        remappings=[("odometry/filtered", "/odometry/filtered"),
                    ("/set_pose", "/initialpose")]
    )

    lifecycle_manager_map = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_map", output="screen",
        parameters=[{"autostart": True, "node_names": ["map_server"]}]
    )

    return LaunchDescription([
        static_tf_publisher,
        walking_param_odom_node,
        odom_throttle_node,
        map_server_node,
        ekf_node,
        TimerAction(period=1.0, actions=[lifecycle_manager_map]),
    ])
