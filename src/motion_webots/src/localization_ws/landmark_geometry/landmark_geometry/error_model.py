#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro — Apache-2.0
"""Distance-dependent measurement covariance for a projected ground landmark.

Fitted from ``scripts/covariance_model.py`` (TAHAP 2.2), which propagates the
input uncertainties (pixel sigma=3 px, head angle sigma=0.5 deg, base height
sigma=2 cm) through the exact pinhole+ground geometry over the val set. The error
is strongly ANISOTROPIC: radial (range) error grows ~quadratically toward the
horizon while tangential (cross-range) error grows slowly — so the covariance
must be oriented along the camera->landmark bearing, NOT isotropic.

Measured medians (val, all ground classes ~identical):
    d[m]:      0     1     2     3     4     5     6      (goalpost to 9)
    sig_rng: 0.04  0.08  0.17  0.29  0.43  0.63  0.86
    sig_crs: 0.008 0.014 0.023 0.031 0.039 0.048 0.056
Parametric fit used below (within ~10% of the table):
    sigma_range(d) = SR0 + SR2 * d^2
    sigma_cross(d) = SC0 + SC1 * d
Re-run the study with measured detector pixel sigma to refine the constants.
"""
import math
from typing import Tuple

# fitted constants (metres)
SR0, SR2 = 0.03, 0.0225        # radial: 0.03 + 0.0225 d^2
SC0, SC1 = 0.008, 0.0080       # tangential: 0.008 + 0.008 d
_SIGMA_FLOOR = 0.01            # never report a degenerate zero-variance fix

# hard cull range per class [m] (sigma_range ~1 m -> no longer a position fix).
# The covariance already down-weights far landmarks; this only drops the clearly
# unreliable ones. Goalposts (tall, detectable farther) get a looser bound.
_MAX_RANGE = {0: 6.0, 1: 6.0, 2: 6.0, 3: 8.0, 4: 6.0}


def ground_sigmas(d: float) -> Tuple[float, float]:
    """(sigma_range, sigma_cross) in metres at camera->landmark distance d."""
    sr = max(SR0 + SR2 * d * d, _SIGMA_FLOOR)
    sc = max(SC0 + SC1 * d, _SIGMA_FLOOR)
    return sr, sc


def max_range(class_id: int) -> float:
    return _MAX_RANGE.get(int(class_id), 6.0)


def cov_2x2(px: float, py: float):
    """2x2 base-frame covariance [xx, xy, yx, yy] for a landmark at (px, py).

    Built as diag(sigma_range^2, sigma_cross^2) in the radial/tangential frame
    of the camera->landmark bearing, then rotated into base_link xy.
    """
    d = math.hypot(px, py)
    sr, sc = ground_sigmas(d)
    phi = math.atan2(py, px)
    c, s = math.cos(phi), math.sin(phi)
    vr, vt = sr * sr, sc * sc
    # R diag(vr,vt) R^T  with R = [[c,-s],[s,c]]
    xx = c * c * vr + s * s * vt
    yy = s * s * vr + c * c * vt
    xy = c * s * (vr - vt)
    return [xx, xy, xy, yy]
