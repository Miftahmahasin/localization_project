#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro — Apache-2.0
"""TAHAP 4 — weighted least-squares pose refine + covariance from associations.

``mhl.GeometricLocalizer`` recovers a pose from landmark PAIRS (CLAP Eq.5) by
voting/clustering, but returns no covariance and no conditioning measure. The EKF
``pose2`` update needs an honest 3x3 covariance, and the plan's TAHAP 4.3 defines
a *valid* fix by its conditioning — so we refine the CLAP seed with a Gauss-Newton
weighted least-squares fit over ALL matched landmarks and read the information
matrix straight out of it.

Given matched triples (b_k in base_link, 2x2 measurement cov C_k, w_k on the map),
minimise  sum_k || b_k - z(pose, w_k) ||^2_{C_k^-1}  where
    z(pose, w) = R(-yaw) (w - [x, y])           (mhl / association.predict_obs)
Gauss-Newton step, reusing the exact 2x3 Jacobian H = dz/d(x,y,yaw) already
derived and finite-difference-checked in association._pred_jacobian:
    (sum H^T W H) dp = sum H^T W nu ,   nu = b - z ,  W = C^-1
The information matrix  Λ = sum H^T W H  gives:
  * covariance      P = Λ^-1                        (-> pose2 covariance)
  * conditioning    cond(Λ)                         (-> "well-conditioned?" gate)
A pair of landmarks at nearly the same bearing leaves yaw/range weakly observed:
Λ becomes near-singular, cond blows up — exactly the ill-conditioned "counts as
>=2 but useless" case the plan warns about.

Pure NumPy, ROS-free (swept offline in scripts/fix_rate_eval.py).
"""
import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

from landmark_localization.association import _pred_jacobian, predict_obs


@dataclass
class PoseFit:
    pose: np.ndarray          # (3,) [x, y, yaw]
    cov: np.ndarray           # (3,3) covariance = information^-1
    info: np.ndarray          # (3,3) information matrix (sum H^T W H)
    cond: float               # condition number of the information matrix
    mean_resid_m: float       # mean Euclidean residual over matched landmarks [m]
    n: int                    # number of matched landmarks used


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def solve_pose_wls(bs: Sequence[np.ndarray],
                   covs: Sequence[np.ndarray],
                   ws: Sequence[np.ndarray],
                   pose0: Sequence[float],
                   iters: int = 12,
                   tol: float = 1e-9) -> Optional[PoseFit]:
    """Gauss-Newton WLS pose fit from matched landmarks. None if degenerate.

    ``bs``/``covs``/``ws`` are aligned: observation k in base_link, its 2x2
    measurement covariance, and the map point it was associated to. ``pose0`` is
    a seed (e.g. the CLAP cluster centroid).
    """
    n = len(bs)
    if n < 2:
        return None
    pose = np.array([float(pose0[0]), float(pose0[1]), float(pose0[2])])
    Wk = [np.linalg.inv(np.asarray(c, float)) for c in covs]
    info = np.eye(3)
    for _ in range(iters):
        info = np.zeros((3, 3))
        g = np.zeros(3)
        for b, w, W in zip(bs, ws, Wk):
            z, H = _pred_jacobian(pose, np.asarray(w, float))
            nu = np.asarray(b, float) - z
            info += H.T @ W @ H
            g += H.T @ W @ nu
        try:
            dp = np.linalg.solve(info, g)
        except np.linalg.LinAlgError:
            return None
        pose = pose + dp
        pose[2] = _wrap(pose[2])
        if float(np.linalg.norm(dp)) < tol:
            break
    try:
        cov = np.linalg.inv(info)
    except np.linalg.LinAlgError:
        return None
    resid = float(np.mean([
        float(np.linalg.norm(np.asarray(b, float) - predict_obs(pose, w)))
        for b, w in zip(bs, ws)]))
    cond = float(np.linalg.cond(info))
    return PoseFit(pose=pose, cov=cov, info=info, cond=cond,
                   mean_resid_m=resid, n=n)


def mirror_pose(pose: Sequence[float]) -> np.ndarray:
    """The 180-degree field-symmetry twin: (x,y,yaw) -> (-x,-y,yaw+pi).

    The only proper rigid self-map of the field; a pose and its mirror are
    indistinguishable from landmark geometry alone (broken only by a prior /
    no-teleport, TAHAP 5). Used to *detect* the two-fold ambiguity per frame.
    """
    return np.array([-float(pose[0]), -float(pose[1]), _wrap(float(pose[2]) + math.pi)])
