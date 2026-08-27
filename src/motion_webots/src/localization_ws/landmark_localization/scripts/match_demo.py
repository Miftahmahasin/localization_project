#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro — Apache-2.0
"""Match-play demo — drives the robot through a sequence of realistic gameplay scenes
across the whole field (kickoff, attack/dribble, throw-in, defend, corner) so the
localization can be watched live in ``live_viz.py`` as if the robot were in a match.

Each scene = a PLACEMENT (teleport) + a RE-SEED at the robot's ACTUAL pose (the
manual-reseed SOP; seeding the measured GT pose makes the demo robust to the flaky
set_pose-theta) + a short WALK sequence (forward + gentle turns). Narrated to stdout.

Run (localization stack up; watch live_viz.py in another window):
  python3 match_demo.py [--loop] [--ball-head]
--ball-head also runs ball_head_sim during walks (deep down-gaze) for a harder, more
realistic head-busy-with-the-ball condition.
"""
import argparse
import math
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry

SCR = "/home/miftah/basbot/src/motion_webots/src/localization_ws/landmark_localization/scripts"

# scene = (narration, place_x, place_y, place_yaw_deg, [ (x_step, angle_deg, dur_s), ... ])
SCENES = [
    ("KICKOFF — di paruh sendiri, hadap gawang lawan; maju ke bola",
     -0.9, 0.0, 0, [(0.012, 0, 7), (0.011, 6, 4), (0.012, 0, 5)]),
    ("SERANG — dribble menembus tengah ke paruh lawan",
     0.4, 0.4, -10, [(0.012, 0, 6), (0.011, -8, 4), (0.012, 0, 6)]),
    ("LEMPAR-KE-DALAM — sayap kanan atas, arahkan ke dalam lapangan",
     1.6, 2.2, -110, [(0.012, 0, 6), (0.010, 8, 5)]),
    ("BERTAHAN — kembali ke area sendiri, hadap gawang sendiri",
     -2.6, 0.6, 175, [(0.012, 0, 6), (0.011, -6, 4), (0.012, 0, 5)]),
    ("TENDANGAN SUDUT lawan — pojok kanan bawah",
     3.0, -1.6, 120, [(0.011, 0, 5), (0.010, 10, 4)]),
]


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))


class Demo:
    def __init__(self, args):
        self.args = args
        rclpy.init()
        self.n = rclpy.create_node('match_demo')
        self.gt = None
        self.n.create_subscription(Odometry, '/ground_truth/odom', self._gt, 10)
        self.pub = self.n.create_publisher(Pose2D, '/robotis_op3/set_pose', 5)

    def _gt(self, m):
        p = m.pose.pose
        self.gt = (p.position.x, p.position.y, yaw_of(p.orientation))

    def spin(self, s):
        t0 = time.time()
        while time.time() - t0 < s:
            rclpy.spin_once(self.n, timeout_sec=0.05)

    def place(self, x, y, yaw_deg):
        """Teleport, wait until GT confirms position (set_pose is flaky), return the
        robot's ACTUAL pose for a truthful re-seed."""
        th = math.radians(yaw_deg)
        for _ in range(12):
            self.pub.publish(Pose2D(x=float(x), y=float(y), theta=float(th)))
            self.spin(1.0)
            if self.gt and math.hypot(self.gt[0] - x, self.gt[1] - y) < 0.3:
                break
        self.spin(1.5)                                   # settle
        return self.gt if self.gt else (x, y, th)

    def seed(self, pose):
        subprocess.run([sys.executable, f"{SCR}/seed_side.py", "--x", "%.3f" % pose[0],
                        "--y", "%.3f" % pose[1], "--yaw", "%.1f" % math.degrees(pose[2])],
                       capture_output=True)
        self.spin(2.0)

    def walk(self, segs):
        head = None
        if self.args.ball_head:
            head = subprocess.Popen(
                [sys.executable, f"{SCR}/ball_head_sim.py", "--duration",
                 str(int(sum(s[2] for s in segs)) + 2), "--enable-module"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for (xs, ang, dur) in segs:
            subprocess.run([sys.executable, f"{SCR}/walk_op3.py", "--x", str(xs),
                            "--angle", str(ang), "--duration", str(dur)],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if head:
            try:
                head.wait(timeout=3)
            except Exception:
                head.terminate()

    def run(self):
        self.spin(1.0)
        loop = 0
        try:
            while True:
                loop += 1
                for i, (name, x, y, yd, segs) in enumerate(SCENES, 1):
                    print("\n[SCENE %d/%d] %s" % (i, len(SCENES), name))
                    print("   place -> (%.1f, %.1f, %d deg), re-seed, walk ..." % (x, y, yd))
                    pose = self.place(x, y, yd)
                    print("   robot at GT (%.2f, %.2f, %.0f deg) -> seeding there"
                          % (pose[0], pose[1], math.degrees(pose[2])))
                    self.seed(pose)
                    self.walk(segs)
                    self.spin(1.5)
                if not self.args.loop:
                    break
                print("\n===== match loop %d done; repeating =====" % loop)
        except KeyboardInterrupt:
            pass
        finally:
            subprocess.run(["bash", "-lc",
                            "ros2 topic pub --once /robotis/walking/command "
                            "std_msgs/msg/String \"{data: 'stop'}\""],
                           capture_output=True)
            print("\nmatch_demo done.")
            rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--loop', action='store_true', help='repeat the match forever')
    ap.add_argument('--ball-head', action='store_true',
                    help='also run ball_head_sim (deep down-gaze) during walks')
    Demo(ap.parse_args()).run()


if __name__ == '__main__':
    main()
