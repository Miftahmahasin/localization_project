#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro — Apache-2.0
"""TAHAP 5 — 180-degree mirror-mode hold.

The field has a perfect 180-degree rotational self-map (both goals identical), so
a landmark-only pose fix is fundamentally TWO-FOLD ambiguous: ``pose`` and its
mirror ``(-x,-y,yaw+pi)`` explain every frame — and every *motion* — equally well.
This is NOT solvable by the sensor (the plan says so); it is broken only by an
EXTERNAL side prior (RoboCup: the robot enters from a known half) plus temporal
holding.

``MirrorModeTracker`` is the pure, ROS-free layer that sits between the geometric
backend and the EKF publish. Each frame the backend hands it one of the two twins
(whichever association happened to land on); the tracker recovers BOTH twins and
picks the one consistent with its OWN running belief ``ref`` — a belief updated
only from committed fixes and held across gaps, deliberately DECOUPLED from the
EKF covariance. That decoupling is the whole point: the observed flips came from
(1) transients near symmetric configs and (2) long fix-less blackouts inflating
the EKF covariance so the backend's no-teleport gate auto-disabled and re-acquired
the side 50/50. Because the twins are ``2*|p|`` apart in position (metres) and
``pi`` apart in yaw, the correct twin stays unambiguously the closest-to-``ref``
even after the belief coasts through a multi-second gap — so the side-lock is
deterministic and cannot be flipped by a transient or a blackout.

Kidnap is the only thing allowed to switch sides, and only on *sustained* contrary
evidence (``kidnap_frames`` well-conditioned fixes all far from ``ref``), never on
one bad frame. A same-side kidnap is recovered by re-localization (MHL) picking the
committed side; a kidnap ACROSS the mirror line is fundamentally ambiguous and is
reported as such, not faked.
"""
import math
from typing import Optional, Sequence, Tuple

import numpy as np

from landmark_localization.pose_solve import mirror_pose


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class MirrorModeTracker:
    """Deterministic side-lock for the 180-degree field-mirror ambiguity.

    Parameters
    ----------
    start_x_sign : Optional[float]
        External side prior used to COMMIT the mode before any belief exists
        (RoboCup known-entry half). ``-1`` = own half at x<0, ``+1`` = x>0,
        ``None``/``0`` = require a seed (``set_ref``) instead. Overridden by a
        later ``set_ref`` (an ``/initialpose`` seed).
    w_yaw : float
        Weight on the yaw term of the twin-distance. The twins differ by pi in
        yaw, so even at the field centre (where the position twins coincide) the
        yaw term keeps the pick well-separated.
    kidnap_resid_m : float
        A committed-side fix farther than this from ``ref`` is 'contrary'.
    kidnap_frames : int
        Consecutive contrary fixes required to declare 'lost' (hysteresis; a
        single transient never switches the side).
    ref_blend : float
        Low-pass factor for updating ``ref`` from a good fix (0..1).
    blend_resid_max : float
        C2 (vision-only ref-blend robustness) — the ``ref`` belief is only updated
        from fixes whose WLS mean bearing residual is <= this (a geometric-quality
        proxy). The mirror-drag failure was a CROUCHED / fallen robot: its wrong
        camera height makes the fix land NEAR ``ref`` in position (so it passes the
        kidnap gate) yet with a garbage yaw that gradually POISONS ``ref``. Such a fix
        is geometrically inconsistent -> HIGH residual, so this gate keeps it from
        dragging the belief. The fix is still PUBLISHED (the EKF weights it by its own
        covariance); only the side-belief update is withheld. No IMU. A residual of -1
        (single-corner / unknown) is treated as trusted (backward-compatible).
    """

    def __init__(self, start_x_sign: Optional[float] = -1.0,
                 w_yaw: float = 1.0, kidnap_resid_m: float = 2.0,
                 kidnap_frames: int = 8, ref_blend: float = 0.5,
                 blend_resid_max: float = 0.30):
        self.ref: Optional[np.ndarray] = None      # committed belief [x,y,yaw]
        self.start_x_sign = (float(np.sign(start_x_sign))
                             if start_x_sign else 0.0)
        self.w_yaw = float(w_yaw)
        self.kidnap_resid = float(kidnap_resid_m)
        self.kidnap_frames = int(kidnap_frames)
        self.ref_blend = float(ref_blend)
        self.blend_resid_max = float(blend_resid_max)
        self._contra = 0

    # ── external commit ───────────────────────────────────────────────────────
    def set_ref(self, pose: Sequence[float]) -> None:
        """Commit / re-commit the belief from an external seed (/initialpose)."""
        self.ref = np.array([float(pose[0]), float(pose[1]),
                             _wrap(float(pose[2]))])
        self._contra = 0

    @property
    def committed(self) -> bool:
        return self.ref is not None

    # ── twin selection ────────────────────────────────────────────────────────
    def _dist(self, c: np.ndarray, r: np.ndarray) -> float:
        dyaw = _wrap(c[2] - r[2])
        return (c[0] - r[0]) ** 2 + (c[1] - r[1]) ** 2 + self.w_yaw * dyaw * dyaw

    def _choose(self, cands) -> np.ndarray:
        if self.ref is None:
            if self.start_x_sign != 0.0:
                for c in cands:                       # commit on the prior side
                    if c[0] * self.start_x_sign >= 0.0:
                        return c
            return cands[0]
        return min(cands, key=lambda c: self._dist(c, self.ref))

    def _blend_ref(self, chosen: np.ndarray) -> None:
        a = self.ref_blend
        self.ref[0] += a * (chosen[0] - self.ref[0])
        self.ref[1] += a * (chosen[1] - self.ref[1])
        self.ref[2] = _wrap(self.ref[2] + a * _wrap(chosen[2] - self.ref[2]))

    # ── main ──────────────────────────────────────────────────────────────────
    def _trusted(self, resid_m: float) -> bool:
        """C2: is the fix geometrically clean enough to update the side-belief?
        resid_m < 0 = unknown (single-corner) -> trusted (backward-compatible)."""
        return resid_m < 0.0 or resid_m <= self.blend_resid_max

    def resolve(self, pose: Sequence[float],
                is_reloc: bool = False,
                resid_m: float = -1.0) -> Tuple[np.ndarray, str]:
        """Map a raw (mirror-ambiguous) fix to the committed side.

        Returns ``(pose_out, kind)`` where kind is:
          'commit' — first fix, side taken from the external prior
          'reloc'  — a global re-localization result (kidnap recovery)
          'ok'     — normal committed-side fix (publish it)
          'hold'   — fix disagrees with belief; SUPPRESS publish (EKF coasts)
                     while evidence accumulates
          'lost'   — sustained disagreement => caller must run recovery
        """
        p = np.array([float(pose[0]), float(pose[1]), _wrap(float(pose[2]))])
        cands = [p, mirror_pose(p)]
        committing = self.ref is None
        chosen = self._choose(cands)

        if committing or is_reloc:
            self.ref = chosen.copy()
            self._contra = 0
            return chosen, ('commit' if committing else 'reloc')

        resid = math.hypot(chosen[0] - self.ref[0], chosen[1] - self.ref[1])
        if resid <= self.kidnap_resid:
            if self._trusted(resid_m):     # C2: only a CLEAN fix updates the belief;
                self._blend_ref(chosen)    # a high-residual (crouch/fall) fix is still
            self._contra = 0               # published but does NOT poison ref
            return chosen, 'ok'

        self._contra += 1
        if self._contra >= self.kidnap_frames:
            self._contra = 0
            return chosen, 'lost'
        return chosen, 'hold'
