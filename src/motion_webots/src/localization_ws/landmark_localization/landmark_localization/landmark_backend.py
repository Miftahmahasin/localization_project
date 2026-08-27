#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro — Apache-2.0
"""TAHAP 4.2 — landmark observations -> an absolute pose measurement for the EKF.

The ROS-free core of the geometric backend. Given one frame's landmark
observations (class id + ground point ``b`` in base_link + 2x2 covariance) and the
EKF prior pose (x, y, yaw + 3x3 covariance), produce a single pose measurement to
feed the EKF as ``pose2`` — WITHOUT any body odometry (the plan's no-odom
baseline). Everything here is prior-gated, so the 180-degree field mirror is
broken by the prior, not by odom.

Three outcomes, matching the plan's opportunistic (never mandatory) update:
  * ``full``   — >=2 landmarks associate: a Gauss-Newton WLS pose fit over all of
                 them gives (x, y, yaw) AND an honest 3x3 covariance from the
                 information matrix (``pose_solve.solve_pose_wls``).
  * ``single`` — exactly 1 landmark associates: a *partial* fix. A single corner
                 cannot pin yaw, but with the prior's yaw it pins POSITION:
                 p = w - R(yaw_prior) b. We publish x, y with a covariance that
                 propagates BOTH the measurement noise and the prior-yaw
                 uncertainty, and leave yaw effectively unobserved (huge variance)
                 so the EKF takes only the position. Enabled by
                 ``single_corner_mode='partial'``; ``'coast'`` returns ``none``.
  * ``none``   — nothing usable this frame; the EKF coasts on its process model.

A **no-teleport** gate rejects a fix that jumps farther than physically possible
from a CONFIDENT prior (mis-association / residual mirror), while letting a fix
through when the prior is uncertain or the associator used its prior-free RANSAC
consensus (global recovery — that jump is intended, TAHAP 5).
"""
import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence

import numpy as np

from landmark_localization.association import DataAssociator, AObs
from landmark_localization.pose_solve import solve_pose_wls
from landmark_localization.mhl import MapLandmark, build_map


def _R(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def _wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class BackendResult:
    kind: str                                   # 'full' | 'single' | 'none'
    pose: np.ndarray = field(default_factory=lambda: np.zeros(3))
    cov: np.ndarray = field(default_factory=lambda: np.eye(3))
    n_matched: int = 0
    used_ransac: bool = False
    mode_agnostic: bool = False
    matches: list = field(default_factory=list)   # [(obs_idx, map_idx)] diagnostic
    gated_chi2: bool = False                       # True if chi-square-gate rejected
    chi2: float = -1.0                             # last fix-vs-prior Mahalanobis d^2
    resid_m: float = -1.0                          # WLS mean bearing residual (C2);
    #                                                -1 = unknown (single-corner/none)


class LandmarkBackend:
    def __init__(self,
                 field_map: Optional[List[MapLandmark]] = None,
                 single_corner_mode: str = 'partial',   # 'partial' | 'coast'
                 no_teleport_m: float = 1.0,
                 no_teleport_prior_pos_var: float = 0.25,   # (0.5 m)^2
                 single_yaw_var: float = 1.0e4,
                 chi2_gate: float = 16.27,              # 99.9% chi-square, 3 DOF
                 agnostic_group: Optional[Sequence[int]] = None,
                 cond_inflate_at: float = 0.0,          # 0 = disabled (C4)
                 cond_inflate_cap: float = 10.0,
                 associator: Optional[DataAssociator] = None):
        self.map = field_map if field_map is not None else build_map()
        self.map_w = np.array([m.w for m in self.map])
        # C5 — per-profile association group. Which detector classes may CROSS-match
        # (type-agnostic) to absorb L<->T<->X confusion. SIM = {L,T} (0,1): X is a
        # distinctive/reliable class, so it matches strictly by-type — this KILLS the
        # sparse-view X<->T false match (TAHAP 8) at ~no recall cost. HARDWARE = {L,T,X}
        # (0,1,2): the real detector confuses X too, so it must be absorbed. Risk is
        # ASYMMETRIC — excluding X on sim gains a little (fewer false matches) and loses
        # ~nothing (X is distinct); including X on a reliable sim detector risks the
        # false minima. None -> the DataAssociator default (JUNCTION_CLASSES {L,T,X}).
        if associator is not None:
            self.assoc = associator
        elif agnostic_group is not None:
            self.assoc = DataAssociator(
                field_map=self.map, agnostic_group=frozenset(int(c) for c in agnostic_group))
        else:
            self.assoc = DataAssociator(field_map=self.map)
        self.single_corner_mode = str(single_corner_mode)
        self.no_teleport_m = float(no_teleport_m)
        self.no_teleport_prior_pos_var = float(no_teleport_prior_pos_var)
        self.single_yaw_var = float(single_yaw_var)
        # B3.1 chi-square (Mahalanobis) fix-vs-prior gate: reject a fix whose innovation
        # vs the EKF prior is too large for S = fix_cov + prior_cov. STRUCTURALLY CLOSED
        # (default <=0 = OFF; kept only as a mechanical switch). It is a POSITIVE-FEEDBACK
        # trap: the prior is built FROM the fixes, so gating a fix against it rejects the
        # very fixes that would correct a drifting belief (live A/B: gate ON made mirror
        # 16.8% / flips 5.8 vs 0% OFF). Gross outliers are handled prior-INDEPENDENTLY by
        # no_teleport (hard position cap) and by C4 cond-inflation (down-weight, never
        # reject). Do not re-enable. <=0 disables.
        self.chi2_gate = float(chi2_gate)
        # C4 — condition-number covariance inflation (NOT a gate; no accept/reject, no
        # feedback loop). When the WLS information matrix is ill-conditioned (two nearly
        # collinear bearings -> a degenerate poor view), the fit is geometrically
        # suspect. cov = info^-1 ALREADY blows up in the weak eigendirection, so the EKF
        # down-weights that axis on its own; this adds an OPTIONAL isotropic floor so a
        # high-cond fix is also not over-trusted in its "strong" axis. It only touches
        # fits with cond > cond_inflate_at, so well-conditioned fits (normal 3+ landmark
        # tracking) are byte-for-byte unchanged -> no 8b/8c regression. DEFAULT DISABLED
        # (0.0): the seeded baseline shows no false minima, and info^-1 handles the weak
        # axis; enable + measure the cond distribution on a poor-view eval before relying
        # on it (plan C4: "bila tak ada perbaikan terukur -> tutup & catat").
        self.cond_inflate_at = float(cond_inflate_at)
        self.cond_inflate_cap = float(cond_inflate_cap)

    def _cond_inflate(self, cov: np.ndarray, cond: float) -> np.ndarray:
        """Scale cov up when the fit is ill-conditioned. Identity when disabled or
        the fit is well-conditioned (cond <= threshold)."""
        if self.cond_inflate_at <= 0.0 or cond <= self.cond_inflate_at:
            return cov
        factor = min(cond / self.cond_inflate_at, self.cond_inflate_cap)
        return cov * factor

    def estimate(self, obs: Sequence[AObs], prior_pose: Sequence[float],
                 prior_cov: np.ndarray) -> BackendResult:
        prior_pose = np.array([float(prior_pose[0]), float(prior_pose[1]),
                               float(prior_pose[2])])
        P = np.asarray(prior_cov, float).reshape(3, 3)
        res = self.assoc.associate(list(obs), prior_pose, P)
        matched = [(a.obs_idx, a.map_idx) for a in res.assocs if a.map_idx >= 0]

        if len(matched) >= 2:
            bs = [obs[i].b for i, _ in matched]
            covs = [obs[i].cov for i, _ in matched]
            ws = [self.map_w[j] for _, j in matched]
            fit = solve_pose_wls(bs, covs, ws, res.pose_used)
            if fit is None:
                return BackendResult('none')
            if self._teleport(fit.pose, prior_pose, P, res.used_ransac):
                return BackendResult('none')
            cov = self._cond_inflate(fit.cov, fit.cond)   # C4 (identity if disabled)
            d2 = self._chi2(fit.pose, cov, prior_pose, P)
            if self.chi2_gate > 0.0 and not res.used_ransac and d2 > self.chi2_gate:
                return BackendResult('none', gated_chi2=True, chi2=d2)
            return BackendResult('full', fit.pose, cov, len(matched),
                                 res.used_ransac, res.mode_agnostic, matched,
                                 chi2=d2, resid_m=float(fit.mean_resid_m))

        if len(matched) == 1 and self.single_corner_mode == 'partial':
            i, j = matched[0]
            b = np.asarray(obs[i].b, float)
            w = self.map_w[j]
            yaw = prior_pose[2]
            c, s = math.cos(yaw), math.sin(yaw)
            R = np.array([[c, -s], [s, c]])
            p_xy = w - R @ b                        # position given prior yaw
            # covariance: rotate the measurement cov into map, then add the prior
            # yaw uncertainty propagated through dp/dyaw = -(dR/dyaw) b.
            dR = np.array([[-s, -c], [c, -s]])
            jyaw = -(dR @ b)
            cov_xy = R @ np.asarray(obs[i].cov, float) @ R.T \
                + float(P[2, 2]) * np.outer(jyaw, jyaw)
            pose = np.array([p_xy[0], p_xy[1], yaw])
            if self._teleport(pose, prior_pose, P, res.used_ransac):
                return BackendResult('none')
            cov = np.zeros((3, 3))
            cov[:2, :2] = cov_xy
            cov[2, 2] = self.single_yaw_var        # yaw not observed by 1 corner
            d2 = self._chi2(pose, cov, prior_pose, P)   # yaw term ~0 (huge cov[2,2])
            if self.chi2_gate > 0.0 and not res.used_ransac and d2 > self.chi2_gate:
                return BackendResult('none', gated_chi2=True, chi2=d2)
            return BackendResult('single', pose, cov, 1, res.used_ransac,
                                 res.mode_agnostic, chi2=d2)

        return BackendResult('none')

    def _chi2(self, pose: np.ndarray, cov: np.ndarray,
              prior_pose: np.ndarray, P: np.ndarray) -> float:
        """Mahalanobis d^2 of the fix innovation vs the prior, S = cov + P.

        nu = fix - prior (yaw wrapped); d^2 = nu^T S^-1 nu. Treating S as the sum
        of the two covariances is the standard (slightly conservative, since fix &
        prior are positively correlated) innovation model. Returns 0.0 if S is not
        invertible (never gate on a numerically degenerate S)."""
        nu = np.array([pose[0] - prior_pose[0], pose[1] - prior_pose[1],
                       _wrap_pi(pose[2] - prior_pose[2])])
        S = np.asarray(cov, float) + np.asarray(P, float)
        try:
            return float(nu @ np.linalg.solve(S, nu))
        except np.linalg.LinAlgError:
            return 0.0

    def _teleport(self, pose: np.ndarray, prior_pose: np.ndarray,
                  P: np.ndarray, used_ransac: bool) -> bool:
        """Reject an implausible jump from a CONFIDENT prior (not during recovery)."""
        if used_ransac:
            return False                            # global recovery may jump
        if float(P[0, 0]) > self.no_teleport_prior_pos_var:
            return False                            # prior too uncertain to gate
        d = float(np.hypot(pose[0] - prior_pose[0], pose[1] - prior_pose[1]))
        return d > self.no_teleport_m
