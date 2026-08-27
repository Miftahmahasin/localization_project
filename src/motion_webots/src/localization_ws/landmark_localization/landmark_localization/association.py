#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro — Apache-2.0
"""TAHAP 3 — data association: projected landmarks -> map landmarks.

Given the per-frame observations (``soccer_msgs/LandmarkArray``, each a class id +
ground point ``b`` in base_link + anisotropic 2x2 covariance from the TAHAP 2
error model) and the EKF prior pose (x, y, yaw with a 3x3 covariance), decide
which MAP landmark each observation corresponds to — the step that must be right
before any geometric pose can be trusted, and the step where a false landmark
does its damage (prompt rule #6, precision > recall).

Design (prompt TAHAP 3):
  * **Mahalanobis gating against the EKF prior.** Each map landmark is predicted
    into the base frame through the prior pose; the innovation covariance folds in
    BOTH the measurement covariance and the prior-pose covariance (2x3 Jacobian),
    so a shaky prior widens the gate exactly as much as it should. Only candidates
    inside the chi-square gate (2 DOF) may match.
  * **by-type ∥ type-agnostic, in parallel.** by-type only matches same-class map
    landmarks (cheap, safe when the detector's class is trusted); type-agnostic
    matches any class (recovers L<->T<->X confusion). Both are run and the
    assignment with the smaller total gated cost wins — so class confusion is
    tolerated without ever *requiring* it.
  * **≤ N landmarks/frame** (default 7): keep the nearest (lowest-variance) ones.
  * **RANSAC fallback** when there are many observations and the gated residual is
    large (prior likely wrong / kidnapped): recover a consensus pose from the
    observations alone via the shared CLAP/MHL localizer and re-gate against THAT
    pose. Association never silently trusts a bad prior.

Pure Python/NumPy, ROS-free — swept offline against the GT sidecar
(scripts/assoc_eval.py) and later wrapped by the runtime backend node (TAHAP 4/5).
"""
import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

from landmark_localization.mhl import (
    GeometricLocalizer, MapLandmark, Obs, build_map)

# chi-square 0.99 quantile, 2 DOF (innovation is 2-D).
CHI2_2DOF_99 = 9.210
CHI2_2DOF_95 = 5.991

# Type-agnostic matching exists to absorb the detector's L<->T<->X confusion
# (junctions look alike). It must NOT match ACROSS visually-distinct classes:
# TAHAP 8 diagnosis showed a uniform-prior agnostic pass mis-matching the UNIQUE
# center_circle (and goalposts) to L/T map landmarks, locking a self-consistent
# WRONG global pose. So cross-class is allowed ONLY inside the junction group;
# center_circle (4) and goalpost (3) may only match their own class.
JUNCTION_CLASSES = frozenset({0, 1, 2})   # L, T, X


@dataclass
class AObs:
    """One landmark observation in base_link, with its measurement covariance."""
    class_id: int
    b: np.ndarray                 # (2,) ground point in base_link [m]
    cov: np.ndarray               # (2,2) measurement covariance [m^2]
    conf: float = 1.0


@dataclass
class Assoc:
    """Result of associating one observation to the map."""
    obs_idx: int
    map_idx: int                  # index into field_map, or -1 if gated out
    d2: float                     # Mahalanobis^2 of the accepted match (inf if none)
    type_agnostic: bool = False   # True if matched across class (mode selected)


@dataclass
class AssocResult:
    assocs: List[Assoc] = field(default_factory=list)
    mode_agnostic: bool = False   # which parallel mode won
    used_ransac: bool = False
    pose_used: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    n_inliers: int = 0
    total_cost: float = 0.0


def _R(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def predict_obs(pose: Sequence[float], w: np.ndarray) -> np.ndarray:
    """Predict where map landmark ``w`` should appear in base_link under ``pose``."""
    px, py, yaw = pose
    return _R(-yaw) @ (np.asarray(w, float) - np.array([px, py]))


def _pred_jacobian(pose: Sequence[float], w: np.ndarray
                   ) -> Tuple[np.ndarray, np.ndarray]:
    """z_pred and its 2x3 Jacobian d z_pred / d (px, py, yaw)."""
    px, py, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    Rinv = np.array([[c, s], [-s, c]])          # R(-yaw)
    dx, dy = float(w[0] - px), float(w[1] - py)
    z = Rinv @ np.array([dx, dy])
    # d R(-yaw)/d yaw = [[-s, c], [-c, -s]]
    dR = np.array([[-s, c], [-c, -s]])
    H = np.empty((2, 3))
    H[:, 0] = -Rinv[:, 0]                        # d/d px
    H[:, 1] = -Rinv[:, 1]                        # d/d py
    H[:, 2] = dR @ np.array([dx, dy])            # d/d yaw
    return z, H


class DataAssociator:
    def __init__(self,
                 field_map: Optional[List[MapLandmark]] = None,
                 gate_chi2: float = CHI2_2DOF_99,
                 max_obs: int = 7,
                 ransac_min_obs: int = 5,
                 ransac_resid_m: float = 0.50,
                 ransac_inlier_frac: float = 0.60,
                 gate_pos_cap_m: float = 0.50,
                 gate_yaw_cap_deg: float = 15.0,
                 agnostic_group=JUNCTION_CLASSES):
        # Classes allowed to cross-match in type-agnostic mode (detector class
        # confusion). Default {L,T,X}. Pass {0,1} to make X DISTINCTIVE (X only
        # matches X, like circle/goalpost) — B3.2 measures this trade-off.
        self.agnostic_group = frozenset(agnostic_group)
        self.map = field_map if field_map is not None else build_map()
        self.map_w = np.array([m.w for m in self.map])          # (M,2)
        self.map_cls = np.array([m.class_id for m in self.map])  # (M,)
        self.gate = float(gate_chi2)
        self.max_obs = int(max_obs)
        self.ransac_min_obs = int(ransac_min_obs)
        self.ransac_resid_m = float(ransac_resid_m)
        self.ransac_inlier_frac = float(ransac_inlier_frac)
        # The prior's contribution to the gate is CAPPED: a hopelessly wrong prior
        # must not widen the gate so far that any map landmark "matches" (that is
        # how a kidnapped robot locks onto a self-consistent WRONG labelling). A
        # bounded gate instead rejects the true landmarks -> high gated-out ->
        # RANSAC/MHL global recovery fires. Gate width tracks the prior only up to
        # this cap.
        self._gate_pos_cap = float(gate_pos_cap_m) ** 2
        self._gate_yaw_cap = math.radians(float(gate_yaw_cap_deg)) ** 2
        # a shared CLAP/MHL localizer for the prior-free RANSAC fallback
        self._mhl = GeometricLocalizer(field_map=self.map)

    # ── frame cap ───────────────────────────────────────────────────────────
    def _cap(self, obs: Sequence[AObs]) -> List[int]:
        """Indices of the <=max_obs observations kept (nearest = lowest variance)."""
        idx = list(range(len(obs)))
        if len(idx) <= self.max_obs:
            return idx
        # keep smallest trace(cov) — the most informative measurements
        idx.sort(key=lambda i: float(np.trace(obs[i].cov)))
        return sorted(idx[:self.max_obs])

    # ── one-mode gated nearest-neighbour association ─────────────────────────
    def _associate_mode(self, obs: Sequence[AObs], keep: Sequence[int],
                        pose: Sequence[float], P: np.ndarray,
                        type_agnostic: bool) -> Tuple[List[Assoc], float]:
        # collect all (d2, obs_i, map_j) candidate matches within the gate
        cands: List[Tuple[float, int, int]] = []
        for i in keep:
            o = obs[i]
            # candidate map landmarks
            if type_agnostic and o.class_id in self.agnostic_group:
                # cross-class only within the configured confusable group
                mj = np.nonzero(np.isin(self.map_cls,
                                        list(self.agnostic_group)))[0]
            else:
                # same-class only (by-type mode, OR agnostic for a distinct class
                # like center_circle / goalpost which must never cross-match)
                mj = np.nonzero(self.map_cls == o.class_id)[0]
            for j in mj:
                z, H = _pred_jacobian(pose, self.map_w[j])
                S = o.cov + H @ P @ H.T
                nu = o.b - z
                try:
                    d2 = float(nu @ np.linalg.solve(S, nu))
                except np.linalg.LinAlgError:
                    continue
                if d2 <= self.gate:
                    cands.append((d2, i, int(j)))
        cands.sort(key=lambda t: t[0])
        # greedy unique assignment: ascending d2, each obs and each map used once
        used_obs, used_map = set(), set()
        assocs = {i: Assoc(i, -1, math.inf, type_agnostic) for i in keep}
        total = 0.0
        for d2, i, j in cands:
            if i in used_obs or j in used_map:
                continue
            used_obs.add(i)
            used_map.add(j)
            assocs[i] = Assoc(i, j, d2, type_agnostic)
            total += d2
        # gated-out obs pay a fixed penalty so "more inliers" beats "fewer"
        total += self.gate * sum(1 for i in keep if assocs[i].map_idx < 0)
        return [assocs[i] for i in keep], total

    # ── public entry: by-type ∥ type-agnostic (+ RANSAC fallback) ────────────
    def associate(self, obs: Sequence[AObs], pose: Sequence[float],
                  P: np.ndarray) -> AssocResult:
        keep = self._cap(obs)
        P = self._cap_gate_cov(np.asarray(P, float).reshape(3, 3))

        a_typed, cost_typed = self._associate_mode(obs, keep, pose, P, False)
        a_agno, cost_agno = self._associate_mode(obs, keep, pose, P, True)
        if cost_agno < cost_typed:
            assocs, mode_agno = a_agno, True
        else:
            assocs, mode_agno = a_typed, False

        pose_used = tuple(float(v) for v in pose)
        used_ransac = False
        n_in = sum(1 for a in assocs if a.map_idx >= 0)

        # RANSAC fallback: many obs but the gated fit is poor -> prior is suspect.
        if len(keep) > self.ransac_min_obs and \
                self._poor(obs, assocs, pose, len(keep)):
            rp = self._ransac_pose(obs, keep)
            if rp is not None:
                a2_t, c2_t = self._associate_mode(obs, keep, rp, P, False)
                a2_a, c2_a = self._associate_mode(obs, keep, rp, P, True)
                a2, m2 = (a2_a, True) if c2_a < c2_t else (a2_t, False)
                n2 = sum(1 for a in a2 if a.map_idx >= 0)
                if n2 > n_in:                    # only adopt if it explains more
                    assocs, mode_agno = a2, m2
                    pose_used, used_ransac, n_in = rp, True, n2

        total = sum(a.d2 for a in assocs if a.map_idx >= 0)
        return AssocResult(assocs=assocs, mode_agnostic=mode_agno,
                           used_ransac=used_ransac, pose_used=pose_used,
                           n_inliers=n_in, total_cost=float(total))

    # ── RANSAC helpers ───────────────────────────────────────────────────────
    def _cap_gate_cov(self, P: np.ndarray) -> np.ndarray:
        """Bound the prior covariance used for gating (see __init__ note)."""
        Pc = P.copy()
        Pc[0, 0] = min(Pc[0, 0], self._gate_pos_cap)
        Pc[1, 1] = min(Pc[1, 1], self._gate_pos_cap)
        Pc[2, 2] = min(Pc[2, 2], self._gate_yaw_cap)
        return Pc

    def _poor(self, obs: Sequence[AObs], assocs: Sequence[Assoc],
              pose: Sequence[float], n_keep: int) -> bool:
        """Prior is suspect: matched residuals large OR too few landmarks matched.

        The second clause is what catches a kidnapped robot — a badly wrong prior
        leaves most observations OUTSIDE the (bounded) gate, so few inliers is the
        tell, even when the handful that did match have small residuals.
        """
        res = []
        for a in assocs:
            if a.map_idx < 0:
                continue
            z = predict_obs(pose, self.map_w[a.map_idx])
            res.append(float(np.linalg.norm(obs[a.obs_idx].b - z)))
        if len(res) < self.ransac_inlier_frac * n_keep:
            return True
        return float(np.mean(res)) > self.ransac_resid_m

    def _ransac_pose(self, obs: Sequence[AObs], keep: Sequence[int]
                     ) -> Optional[Tuple[float, float, float]]:
        """Prior-free consensus pose from the observations (shared CLAP/MHL)."""
        mhl_obs = [Obs(obs[i].class_id, obs[i].b) for i in keep]
        out = self._mhl.localize(mhl_obs)
        if out is None:
            return None
        pose, _votes, _n = out
        return (pose.x, pose.y, pose.yaw)


def cov_from_flat(flat: Sequence[float]) -> np.ndarray:
    """Build a 2x2 covariance from the msg's row-major [xx, xy, yx, yy]."""
    f = list(flat)
    return np.array([[f[0], f[1]], [f[2], f[3]]], float)
