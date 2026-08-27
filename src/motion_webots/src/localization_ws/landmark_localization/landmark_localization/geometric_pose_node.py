#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro — Apache-2.0
"""TAHAP 4.2 — geometric pose backend node: /landmark_array -> /landmark_pose.

Wraps the ROS-free ``LandmarkBackend``. Each ``soccer_msgs/LandmarkArray`` frame
is turned into an absolute pose measurement in ``map`` and published as
``geometry_msgs/PoseWithCovarianceStamped`` on ``/landmark_pose``, which the EKF
consumes as ``pose2`` (see config/ekf_soccer_landmark.yaml). NO body odometry is
used anywhere on this path (the plan's no-odom baseline); the EKF's own output
``/odometry/filtered`` is the prior that gates association and breaks the
180-degree field mirror.

Only ``valid_range`` landmarks are used, and the observation stamp is preserved
from the image (``LandmarkArray.header.stamp``) so the EKF fuses it at the right
time. A ``full`` fix carries (x, y, yaw); a ``single``-corner partial fix carries
(x, y) with yaw left unobserved (huge variance); ``none`` publishes nothing and
the EKF coasts.
"""
import csv

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from soccer_msgs.msg import LandmarkArray
from std_msgs.msg import String
from std_msgs.msg import Bool

from landmark_localization.association import AObs, cov_from_flat
from landmark_localization.landmark_backend import LandmarkBackend
from landmark_localization.mirror_mode import MirrorModeTracker
from landmark_localization.mhl import GeometricLocalizer, Obs as MhlObs


def _yaw_from_quat(q) -> float:
    return float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


class GeometricPoseNode(Node):
    def __init__(self):
        super().__init__('geometric_pose_node')
        p = self.declare_parameter
        self.in_topic = str(p('landmark_topic', '/landmark_array').value)
        self.prior_topic = str(p('prior_topic', '/odometry/filtered').value)
        self.out_topic = str(p('output_topic', '/landmark_pose').value)
        self.map_frame = str(p('map_frame', 'map').value)
        # Output-stamp clock domain. The EKF fuses in ITS clock; if the image
        # carries Webots sim-time stamps (small seconds-since-start) but /clock is
        # not published, the EKF runs on wall time and rejects every image-stamped
        # measurement as ancient -> never publishes. 'now' (default) stamps with
        # the node clock so fusion works on wall time (matching how the AMCL/Cox
        # pose sources stamp). Use 'image' only in a proper sim-time setup
        # (Webots publishing /clock + use_sim_time:=true everywhere).
        self.output_stamp = str(p('output_stamp', 'now').value)  # 'now'|'image'
        mode = str(p('single_corner_mode', 'partial').value)   # 'partial'|'coast'
        no_teleport_m = float(p('no_teleport_m', 1.0).value)
        # B3.1 chi-square (Mahalanobis) fix-vs-prior gate. STRUCTURALLY CLOSED — kept
        # only as a mechanical off-switch (default 0.0 = OFF), NOT a "pending
        # calibration". Why closed for good: the EKF prior IS BUILT FROM the fixes, so
        # gating a fix against that prior is a POSITIVE-FEEDBACK / self-confirmation
        # trap — it rejects exactly the fixes that would correct a drifting belief and
        # entrenches the current one. Live A/B confirmed the harm: with the reliable
        # seed the gate OFF gave 5/5 TRUE, mirror 0%; the gate ON (16.27) made it WORSE
        # (mirror 16.8%, flips mean 5.8, one run 25 flips / 73% mirror). The real fix
        # for gross outliers is the authoritative side seed + no_teleport (hard, prior-
        # independent) + C4 cond-inflation (down-WEIGHT, never reject). Do not re-enable.
        chi2_gate = float(p('chi2_gate', 0.0).value)
        prior_timeout = float(p('prior_timeout_sec', 1.0).value)
        # prior covariance to assume before the first EKF message arrives (large
        # -> association leans on RANSAC/global recovery, no false no-teleport gate)
        self.startup_pos_var = float(p('startup_pos_var', 100.0).value)
        self.startup_yaw_var = float(p('startup_yaw_var', 9.87).value)  # (pi)^2

        # C5 — per-profile association group (which classes may cross-match). SIM
        # default {L,T}=[0,1] (X is distinctive -> by-type only, kills the sparse-view
        # X<->T false match); HARDWARE would set [0,1,2] to absorb a confused X.
        agn = list(p('assoc_agnostic_group', [0, 1]).value)
        # C4 — condition-number cov inflation; 0.0 = disabled (default, no regression).
        cond_at = float(p('cond_inflate_at', 0.0).value)
        cond_cap = float(p('cond_inflate_cap', 10.0).value)
        self.backend = LandmarkBackend(single_corner_mode=mode,
                                       no_teleport_m=no_teleport_m,
                                       chi2_gate=chi2_gate,
                                       agnostic_group=agn,
                                       cond_inflate_at=cond_at,
                                       cond_inflate_cap=cond_cap)
        self.prior_timeout = prior_timeout
        self._prior = None            # (t, x, y, yaw, P3x3)

        # TAHAP 5 — 180-degree mirror-mode hold. The tracker deterministically
        # locks the published pose to the committed field-side (from start_x_sign
        # or an /initialpose seed), so transients / blackouts can no longer flip
        # the EKF. GeometricLocalizer re-localizes (prior-free MHL) on a confirmed
        # kidnap ('lost'); the tracker then picks the committed side of that fix.
        start_x_sign = float(p('start_x_sign', -1.0).value)   # -1=own half x<0
        self.commit_from_initialpose = bool(
            p('commit_from_initialpose', True).value)
        self.reloc_pos_var = float(p('reloc_pos_var', 1.0).value)
        self.reloc_yaw_var = float(p('reloc_yaw_var', 0.25).value)
        self.mirror = MirrorModeTracker(
            start_x_sign=(start_x_sign if start_x_sign != 0.0 else None),
            kidnap_resid_m=float(p('kidnap_resid_m', 2.0).value),
            kidnap_frames=int(p('kidnap_frames', 8).value),
            # C2 — only a geometrically-clean fix (WLS residual <= this) updates the
            # side-belief; a crouch/fall fix is still published but can't poison ref.
            blend_resid_max=float(p('blend_resid_max', 0.30).value))
        self.reloc = GeometricLocalizer()
        self._expect_self_init = False        # guard for our own /initialpose reset

        # EKF-trap watchdog (TAHAP 8 finding): the EKF can lock onto a wrong pose
        # from a transient bad startup fix, then REJECT every subsequent correct
        # fix as an outlier (pose2_pose_rejection_threshold=3.0) — a self-
        # reinforcing trap the mirror 'lost' path misses (it compares fix vs its
        # own ref, not vs the EKF). If a CONFIDENT full fix persistently disagrees
        # with a FRESH, CONFIDENT EKF prior, hard-reset the EKF to the (trusted)
        # fix. Gated on EKF confidence so it never fires while legitimately
        # converging from uniform.
        self.diag_assoc = bool(p('diag_assoc', False).value)   # TAHAP 8 diagnostic
        # Mirror-tracker PERSISTENT per-frame diagnostic: writes ref/kind/contra/
        # raw-fix/chosen/reloc/prior each frame to a CSV so the exact mirror-flip
        # mechanism can be read from disk (the recurring drag: c_run3/c_run5/
        # kr_run3). '' disables. Self-contained so no scrollback is needed.
        self.mirror_diag_csv = str(p('mirror_diag_csv', '').value)
        self._mdiag = None
        self._mdiag_f = None
        if self.mirror_diag_csv:
            self._mdiag_f = open(self.mirror_diag_csv, 'w', newline='')
            self._mdiag = csv.writer(self._mdiag_f)
            self._mdiag.writerow([
                't', 'res_kind', 'mkind', 'ref_b_x', 'ref_b_y', 'ref_b_yaw',
                'contra_b', 'raw_x', 'raw_y', 'raw_yaw', 'chosen_x', 'chosen_y',
                'chosen_yaw', 'ref_a_x', 'ref_a_y', 'ref_a_yaw', 'contra_a',
                'resid_to_ref', 'reloc_x', 'reloc_y', 'reloc_yaw',
                'prior_x', 'prior_y', 'prior_yaw', 'resid_m'])
            self.get_logger().info('mirror diag -> %s' % self.mirror_diag_csv)
        self.ekf_trap_watchdog = bool(p('ekf_trap_watchdog', True).value)
        self.trap_resid_m = float(p('trap_resid_m', 0.6).value)
        self.trap_yaw_rad = np.radians(float(p('trap_yaw_deg', 30.0).value))
        self.trap_frames = int(p('trap_frames', 15).value)
        self.trap_ekf_var_max = float(p('trap_ekf_var_max', 0.3).value)
        self.trap_cooldown_s = float(p('trap_cooldown_s', 3.0).value)
        self._trap_count = 0
        self._last_reset_t = -1.0e9

        # TAHAP C3 — vision-only ZUPT (zero-velocity pseudo-measurement).
        # The no-odom EKF infers velocity from pose deltas; on any frame we do NOT
        # publish a fresh fix (none / hold / lost-noreloc), it gets no correction and
        # COASTS on the last inferred velocity. After a kidnap teleport that velocity
        # is a huge phantom (kr_run2: fix died at t=13 s -> mirror 'hold' -> nothing
        # published -> EKF ran -2.4 m to +16 m). Pure-vision remedy: on every coast
        # frame publish a ZERO twist so the EKF pins velocity to ~0 and HOLDS the last
        # pose (bounded ~fix-error) instead of running away. This lets the velocity
        # process noise be restored to a responsive value (ekf yaml) without the
        # runaway returning. NOT IMU/odom — the trigger is fix-validity + the gait
        # command, both vision-side signals. On 'ok'/'commit'/'reloc' frames we do NOT
        # ZUPT (the robot may be moving; the published fix drives velocity normally).
        self.zupt_enable = bool(p('zupt_enable', True).value)
        self.zupt_topic = str(p('zupt_topic', '/zupt').value)
        self.base_frame = str(p('base_link_frame', 'base_link').value)
        # Pin covariance: tight enough to null a phantom velocity within a frame or
        # two, loose enough that a real fix still drives position (twist only touches
        # the velocity states). std ~0.05 m/s, ~0.1 rad/s.
        self.zupt_v_var = float(p('zupt_v_var', 0.0025).value)
        self.zupt_w_var = float(p('zupt_w_var', 0.01).value)
        # Gait stationarity: latched from /robotis/walking/command ('start'/'stop').
        # Default True (assume moving) so we NEVER pin a walking robot on an unknown
        # state; an explicit 'stop' adds an idle ZUPT on top of the coast trigger.
        self._walking = True

        self.sub_prior = self.create_subscription(
            Odometry, self.prior_topic, self._cb_prior, 20)
        self.sub_lm = self.create_subscription(
            LandmarkArray, self.in_topic, self._cb_landmarks, 10)
        if self.commit_from_initialpose:
            # Deep queue: a re-seed arrives as a short burst while the executor is
            # saturated by the ~13 Hz landmark flow; with depth 5 the /initialpose
            # messages were dropped before servicing (mdiag: re-seed reached the EKF
            # but NOT mirror.ref -> ref stayed corrupt -> mirror lock). Depth 20 so
            # a burst survives the flood.
            self.sub_init = self.create_subscription(
                PoseWithCovarianceStamped, '/initialpose', self._cb_initialpose, 20)
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, self.out_topic, 10)
        # Kidnap recovery hard-resets the EKF: a reloc fix jumps metres from the
        # coasting prior, which the EKF's pose2_pose_rejection_threshold (3.0)
        # would REJECT as an outlier. Publishing to /initialpose (the EKF's
        # remapped set_pose) resets the filter state so recovery actually takes.
        self.pub_reset = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 5)

        # S2 — fall/recovery event contract. The behavior layer (a SEPARATE node,
        # IMU-based where the IMU works) publishes a Bool on /fall: True = the robot
        # is down, False = it is upright again (get-up / re-stand done). Localization
        # stays PURE VISION — it never subscribes to the IMU, only this abstract event.
        # A fall/get-up barely shifts (x, y) but the heading can land anywhere, and the
        # existing kidnap recovery is POSITION-triggered (kidnap_resid_m), so it is
        # BLIND to a heading-only disturbance (P1 diag: stuck at 179 deg for 30 s).
        # On /fall True we FREEZE (suppress publish + ZUPT-coast) so garbage down-view
        # fixes never reach the EKF; on the True->False (recovery) edge we CLEAR the
        # stale side-belief and force ONE prior-free reloc so the mirror-tracker
        # re-commits by committed SIDE (x-sign) and recovers the true heading.
        self.fall_enable = bool(p('fall_enable', True).value)
        self.fall_topic = str(p('fall_topic', '/fall').value)
        self._fallen = False
        if self.fall_enable:
            self.sub_fall = self.create_subscription(
                Bool, self.fall_topic, self._fall_cb, 10)

        # C3 — ZUPT publisher + gait-state subscription (both vision-only).
        if self.zupt_enable:
            self.pub_zupt = self.create_publisher(
                TwistWithCovarianceStamped, self.zupt_topic, 10)
            self.sub_cmd = self.create_subscription(
                String, '/robotis/walking/command', self._cmd_cb, 10)

        self._n = dict(full=0, single=0, none=0, hold=0, reloc=0, lost=0,
                       trap=0, chi2gate=0)
        self.create_timer(5.0, self._report)
        self.get_logger().info(
            'geometric_pose_node: %s + prior %s -> %s (single_corner=%s, '
            'no_teleport=%.2f m)' % (self.in_topic, self.prior_topic,
                                     self.out_topic, mode, no_teleport_m))

    def _cb_prior(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        c = np.array(msg.pose.covariance, float).reshape(6, 6)
        P = np.array([[c[0, 0], c[0, 1], c[0, 5]],
                      [c[1, 0], c[1, 1], c[1, 5]],
                      [c[5, 0], c[5, 1], c[5, 5]]])
        self._prior = (t, x, y, yaw, P)

    def _current_prior(self, t_img):
        if self._prior is None:
            return (0.0, 0.0, 0.0), np.diag(
                [self.startup_pos_var, self.startup_pos_var, self.startup_yaw_var])
        t, x, y, yaw, P = self._prior
        if abs(t_img - t) > self.prior_timeout:
            # stale prior: widen it so we don't false-gate on an old pose
            P = P + np.diag([self.startup_pos_var, self.startup_pos_var,
                             self.startup_yaw_var])
        return (x, y, yaw), P

    def _cb_initialpose(self, msg: PoseWithCovarianceStamped):
        # A /initialpose message is ALWAYS authoritative for the mirror side: set
        # the ref unconditionally. Previously our own reloc echo (_reset_ekf) was
        # skipped via _expect_self_init — but that guard also SWALLOWED a real
        # user seed that happened to arrive right after a startup self-reloc,
        # leaving ref on a wrong/mirror side (TAHAP 8: seeded still mirror-locked
        # ~2/3). Re-setting from our own echo is a harmless no-op (the echo pose
        # already equals ref), so just always commit; the label only affects log.
        src = 'self-reset echo' if self._expect_self_init else '/initialpose seed'
        self._expect_self_init = False
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        self.mirror.set_ref((msg.pose.pose.position.x,
                             msg.pose.pose.position.y, yaw))
        self.get_logger().info(
            'mirror ref set from %s: (%.2f, %.2f, %.1f deg)' %
            (src, msg.pose.pose.position.x, msg.pose.pose.position.y,
             np.degrees(yaw)))

    def _reset_ekf(self, pose):
        """Hard-reset the EKF to a recovered pose (via its remapped set_pose)."""
        self._expect_self_init = True
        m = PoseWithCovarianceStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.map_frame
        m.pose.pose.position.x = float(pose[0])
        m.pose.pose.position.y = float(pose[1])
        half = 0.5 * float(pose[2])
        m.pose.pose.orientation.z = float(np.sin(half))
        m.pose.pose.orientation.w = float(np.cos(half))
        c = np.zeros((6, 6))
        c[0, 0] = c[1, 1] = self.reloc_pos_var
        c[5, 5] = self.reloc_yaw_var
        c[2, 2] = c[3, 3] = c[4, 4] = 1.0e6
        m.pose.covariance = c.flatten().tolist()
        self.pub_reset.publish(m)

    def _cmd_cb(self, msg: String):
        cmd = msg.data.lower().strip()
        if cmd == 'start':
            self._walking = True
        elif cmd == 'stop':
            self._walking = False

    def _fall_cb(self, msg: Bool):
        """S2 behavior-event contract (vision-only in localization; IMU lives in the
        separate /fall producer).

        Rising edge -> FREEZE: suppress fix ingest so garbage down-view fixes never
        reach the EKF while the robot is down (a brief no-heading-change stumble then
        resumes cleanly on the preserved pre-fall state). Falling (recovery) edge ->
        just UNFREEZE. A fall that CHANGES heading is recovered by a RE-SEED
        (/initialpose) that the behavior layer / operator provides with the known
        re-entry pose (OPSI 1, instant + robust: 0.4 deg). Autonomous heading-only
        reloc (OPSI 2) is NOT done here: it reduces to global relocalization (the
        backend associates against the stale prior VALUE, so a ~180 deg error is
        self-consistently wrong; inflating yaw variance cannot fix it) = the deferred
        8a problem. See S2_PRAREGISTRASI_GETUP_SUBSTITUSI.md."""
        fell = bool(msg.data)
        if fell and not self._fallen:
            self._fallen = True
            self.get_logger().warn(
                'FALL: localization frozen (coast, no fix ingest) until recovery')
        elif (not fell) and self._fallen:
            self._fallen = False
            self.get_logger().warn(
                'RECOVERY: localization unfrozen; a heading-changing fall needs a '
                're-seed (/initialpose) with the known re-entry pose')

    def _zupt(self, stamp):
        """C3: publish a zero-velocity pseudo-measurement so the no-odom EKF pins
        velocity to ~0 and holds pose, instead of coasting away on a phantom
        velocity when no fresh fix is published this frame."""
        if not self.zupt_enable:
            return
        m = TwistWithCovarianceStamped()
        m.header.stamp = stamp if self.output_stamp == 'image' \
            else self.get_clock().now().to_msg()
        m.header.frame_id = self.base_frame
        # velocities already zero-initialised; only fill the fused variances.
        c = [0.0] * 36
        c[0] = self.zupt_v_var       # vx
        c[7] = self.zupt_v_var       # vy
        c[35] = self.zupt_w_var      # vyaw
        m.twist.covariance = c
        self.pub_zupt.publish(m)

    def _cb_landmarks(self, msg: LandmarkArray):
        if self._fallen:                       # S2: frozen while down -> coast, no ingest
            self._n['none'] += 1
            self._zupt(msg.header.stamp)
            return
        obs = []
        for lm in msg.landmarks:
            if not lm.valid_range:
                continue
            cov = cov_from_flat(lm.covariance_2x2)
            obs.append(AObs(int(lm.class_id),
                            np.array([lm.p_base.x, lm.p_base.y]), cov,
                            float(lm.confidence)))
        if not obs:
            self._n['none'] += 1
            self._zupt(msg.header.stamp)          # coast: nothing seen -> hold pose
            return
        t_img = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        prior_pose, P = self._current_prior(t_img)
        res = self.backend.estimate(obs, prior_pose, P)
        # snapshot the mirror belief BEFORE resolve mutates it (for the diag)
        ref_b = None if self.mirror.ref is None else self.mirror.ref.copy()
        contra_b = self.mirror._contra

        if self.diag_assoc:
            _CN = {0: 'L', 1: 'T', 2: 'X', 3: 'post', 4: 'circ'}
            counts = {}
            for o in obs:
                counts[o.class_id] = counts.get(o.class_id, 0) + 1
            seen = ' '.join('%s:%d' % (_CN.get(k, k), v)
                            for k, v in sorted(counts.items()))
            cond = float(np.linalg.cond(res.cov[:2, :2])) \
                if res.kind != 'none' else -1.0
            mlines = []
            for oi, mj in getattr(res, 'matches', []):
                mm = self.backend.map[mj]
                mlines.append('%s@(%.1f,%.1f)->%s(%s @%.1f,%.1f)' % (
                    _CN.get(obs[oi].class_id, obs[oi].class_id),
                    obs[oi].b[0], obs[oi].b[1], mm.label,
                    _CN.get(mm.class_id, mm.class_id), mm.w[0], mm.w[1]))
            self.get_logger().info(
                'DIAG obs[%s] prior(%.2f,%.2f,%.0f) -> %s pose(%.2f,%.2f,%.0f) '
                'n=%d ransac=%s agn=%s cond=%.0f | %s' % (
                    seen, prior_pose[0], prior_pose[1], np.degrees(prior_pose[2]),
                    res.kind, res.pose[0], res.pose[1], np.degrees(res.pose[2]),
                    res.n_matched, res.used_ransac, res.mode_agnostic, cond,
                    ' ; '.join(mlines)),
                throttle_duration_sec=2.0)

        if getattr(res, 'gated_chi2', False):
            # B3.1: fix rejected as an outlier vs a confident prior. Counted
            # separately from 'none' (nothing-to-see) so the report shows how often
            # the gate bites; the EKF coasts on its (protected) prior this frame.
            self._n['chi2gate'] += 1
            self.get_logger().warn(
                'chi2-gate: fix rejected (d^2=%.1f > %.1f) — outlier vs prior '
                '(%.2f, %.2f, %.0f deg); EKF coasts' % (
                    res.chi2, self.backend.chi2_gate, prior_pose[0],
                    prior_pose[1], np.degrees(prior_pose[2])),
                throttle_duration_sec=2.0)
            self._md('gated', '-', ref_b, contra_b, res.pose, None, None,
                     prior_pose, resid_m=res.resid_m)
            self._zupt(msg.header.stamp)          # coast: fix gated -> hold pose
            return
        if res.kind == 'none':
            self._n['none'] += 1
            self._md('none', '-', ref_b, contra_b, None, None, None, prior_pose,
                     resid_m=res.resid_m)
            self._zupt(msg.header.stamp)          # coast: no fix -> hold pose
            return
        self._n[res.kind] = self._n.get(res.kind, 0) + 1

        # TAHAP 5 — lock to the committed field-side (kills mirror flips).
        reloc_pose = None
        pose_out, kind = self.mirror.resolve(res.pose, resid_m=res.resid_m)  # C2
        cov = res.cov
        if kind == 'hold':                         # fix disagrees with belief:
            self._n['hold'] += 1                   # suppress fix; PIN velocity so the
            self._md(res.kind, 'hold', ref_b, contra_b, res.pose, pose_out,
                     None, prior_pose, resid_m=res.resid_m)
            self._zupt(msg.header.stamp)           # EKF holds pose (not coast->runaway)
            return
        if kind == 'lost':                         # sustained disagreement:
            self._n['lost'] += 1                   # prior-free global recovery
            reloc = self._relocalize(obs)
            if reloc is None:
                self._md(res.kind, 'lost-noreloc', ref_b, contra_b, res.pose,
                         pose_out, None, prior_pose, resid_m=res.resid_m)
                self._zupt(msg.header.stamp)       # coast: no recovery -> hold pose
                return
            reloc_pose = np.array(reloc)
            pose_out, _ = self.mirror.resolve(reloc, is_reloc=True)
            self._reset_ekf(pose_out)          # hard-reset (EKF gate rejects the jump)
            cov = np.diag([self.reloc_pos_var, self.reloc_pos_var,
                           self.reloc_yaw_var])
            self._n['reloc'] += 1
            self.get_logger().warn(
                'KIDNAP recovery: relocalized to (%.2f, %.2f, %.1f deg)' %
                (pose_out[0], pose_out[1], np.degrees(pose_out[2])))

        # EKF-trap watchdog: confident full fix vs a fresh, confident EKF prior.
        if self.ekf_trap_watchdog and res.kind == 'full' \
                and kind in ('ok', 'commit') and self._prior is not None:
            tp, px, py, pyaw, P = self._prior
            fresh = abs(t_img - tp) <= self.prior_timeout
            dpos = float(np.hypot(pose_out[0] - px, pose_out[1] - py))
            dyaw = abs(float(np.arctan2(np.sin(pose_out[2] - pyaw),
                                        np.cos(pose_out[2] - pyaw))))
            # NO EKF-covariance gate: a trapped EKF REJECTS fixes, so its cov
            # INFLATES — gating on "EKF confident" would switch the watchdog off
            # exactly when it is stuck. Persistence is the discriminator instead:
            # while legitimately converging the EKF ACCEPTS the fix and closes the
            # gap within a few frames, so a >trap_frames sustained disagreement
            # only happens in a reject-trap.
            if fresh and (dpos > self.trap_resid_m or dyaw > self.trap_yaw_rad):
                self._trap_count += 1
            else:
                self._trap_count = 0
            now = self.get_clock().now().nanoseconds * 1e-9
            if self._trap_count >= self.trap_frames \
                    and now - self._last_reset_t > self.trap_cooldown_s:
                self.mirror.set_ref((float(pose_out[0]), float(pose_out[1]),
                                     float(pose_out[2])))
                self._reset_ekf(pose_out)
                self._last_reset_t = now
                self._trap_count = 0
                self._n['trap'] += 1
                self.get_logger().warn(
                    'EKF-trap: EKF stuck %.2fm/%.0fdeg off a confident fix -> '
                    'reset to (%.2f, %.2f, %.1f deg)' %
                    (dpos, np.degrees(dyaw), pose_out[0], pose_out[1],
                     np.degrees(pose_out[2])))

        # published paths: 'ok' / 'commit' / 'reloc' (mkind carries the reloc case)
        self._md(res.kind, kind, ref_b, contra_b, res.pose, pose_out,
                 reloc_pose, prior_pose, resid_m=res.resid_m)
        # Idle ZUPT: if the gait is explicitly stopped, pin velocity even with a fresh
        # fix (the robot is stationary; the fix drives position, the ZUPT nulls any
        # phantom velocity). Never fires while walking (default state).
        if not self._walking:
            self._zupt(msg.header.stamp)
        self._publish(msg, pose_out, cov)

    def _md(self, res_kind, mkind, ref_b, contra_b, raw, chosen, reloc, prior,
            resid_m=None):
        """Write one mirror-tracker diagnostic row (if enabled).

        ``resid_m`` = backend WLS mean residual (the C2 ref-blend gate input): on an
        'ok' frame, ref is blended only when resid_m <= blend_resid_max, so a row with
        resid_m > blend_resid_max whose ref_a == ref_b is a C2 gate firing (a
        high-residual crouch/fall/FP fix published but denied the side-belief)."""
        if self._mdiag is None:
            return

        def xyz(p):
            return ['', '', ''] if p is None else [
                '%.4f' % float(p[0]), '%.4f' % float(p[1]), '%.4f' % float(p[2])]
        t = self.get_clock().now().nanoseconds * 1e-9
        ref_a = self.mirror.ref
        resid = ''
        if chosen is not None and ref_b is not None:
            resid = '%.4f' % float(np.hypot(chosen[0] - ref_b[0],
                                            chosen[1] - ref_b[1]))
        self._mdiag.writerow(
            ['%.3f' % t, res_kind, mkind, *xyz(ref_b),
             ('' if contra_b is None else contra_b), *xyz(raw), *xyz(chosen),
             *xyz(ref_a), self.mirror._contra, resid, *xyz(reloc), *xyz(prior),
             ('' if resid_m is None else '%.4f' % float(resid_m))])
        self._mdiag_f.flush()

    def _relocalize(self, obs):
        """Prior-free MHL global recovery -> a pose (still mirror-ambiguous)."""
        r = self.reloc.localize(
            [MhlObs(o.class_id, np.asarray(o.b)) for o in obs])
        if r is None:
            return None
        hyp = r[0]
        return np.array([hyp.x, hyp.y, hyp.yaw])

    def _publish(self, msg, pose, cov):
        out = PoseWithCovarianceStamped()
        out.header.stamp = (msg.header.stamp if self.output_stamp == 'image'
                            else self.get_clock().now().to_msg())
        out.header.frame_id = self.map_frame
        out.pose.pose.position.x = float(pose[0])
        out.pose.pose.position.y = float(pose[1])
        half = 0.5 * float(pose[2])
        out.pose.pose.orientation.z = float(np.sin(half))
        out.pose.pose.orientation.w = float(np.cos(half))
        c = np.zeros((6, 6))
        c[0, 0], c[0, 1], c[1, 0], c[1, 1] = (cov[0, 0], cov[0, 1],
                                              cov[1, 0], cov[1, 1])
        c[0, 5] = c[5, 0] = cov[0, 2]
        c[1, 5] = c[5, 1] = cov[1, 2]
        c[5, 5] = cov[2, 2]
        c[2, 2] = c[3, 3] = c[4, 4] = 1.0e6        # z/roll/pitch unused
        out.pose.covariance = c.flatten().tolist()
        self.pub.publish(out)

    def _report(self):
        tot = self._n['full'] + self._n['single'] + self._n['none']
        if tot == 0:
            return
        self.get_logger().info(
            'fixes: full=%d (%.0f%%) single=%d (%.0f%%) none=%d (%.0f%%) | '
            'mirror: hold=%d reloc=%d lost=%d trap=%d chi2gate=%d committed=%s' %
            (self._n['full'], 100.0 * self._n['full'] / tot,
             self._n['single'], 100.0 * self._n['single'] / tot,
             self._n['none'], 100.0 * self._n['none'] / tot,
             self._n['hold'], self._n['reloc'], self._n['lost'],
             self._n['trap'], self._n['chi2gate'], self.mirror.committed))


def main(args=None):
    rclpy.init(args=args)
    node = GeometricPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
