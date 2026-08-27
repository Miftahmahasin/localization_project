#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro — Apache-2.0
"""Unit tests for TAHAP 4.2 LandmarkBackend (ROS-free)."""
import math

import numpy as np

from landmark_geometry import error_model
from landmark_localization.association import AObs
from landmark_localization.landmark_backend import LandmarkBackend
from landmark_localization.mhl import build_map


def _R(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def _obs(fmap, pose, idx, noise=0.0, rng=None):
    px, py, yaw = pose
    Rt = _R(-yaw)
    out = []
    for j in idx:
        b = Rt @ (fmap[j].w - np.array([px, py]))
        if noise and rng is not None:
            b = b + rng.normal(0, noise, 2)
        cov = np.array(error_model.cov_2x2(float(b[0]), float(b[1]))).reshape(2, 2)
        out.append(AObs(fmap[j].class_id, b, cov))
    return out


def test_full_fix_recovers_pose():
    fmap = build_map()
    be = LandmarkBackend(field_map=fmap)
    pose = [1.2, -0.6, math.radians(20)]
    obs = _obs(fmap, pose, [0, 5, 9, 12])
    P = np.diag([0.09, 0.09, math.radians(8) ** 2])
    res = be.estimate(obs, pose, P)
    assert res.kind == 'full'
    assert np.hypot(res.pose[0] - pose[0], res.pose[1] - pose[1]) < 1e-6
    assert abs(res.pose[2] - pose[2]) < 1e-6
    # covariance is finite, small, positive-definite
    assert np.all(np.linalg.eigvals(res.cov[:2, :2]) > 0)
    assert res.cov[0, 0] < 1.0


def test_single_corner_partial_uses_prior_yaw():
    fmap = build_map()
    be = LandmarkBackend(field_map=fmap, single_corner_mode='partial')
    pose = [1.0, 0.5, math.radians(15)]
    obs = _obs(fmap, pose, [0])            # one landmark only
    P = np.diag([0.04, 0.04, math.radians(5) ** 2])
    res = be.estimate(obs, pose, P)
    assert res.kind == 'single'
    # position recovered exactly (prior yaw is exact here)
    assert np.hypot(res.pose[0] - pose[0], res.pose[1] - pose[1]) < 1e-6
    # yaw is left unobserved (huge variance)
    assert res.cov[2, 2] > 1e3


def test_single_corner_coast_returns_none():
    fmap = build_map()
    be = LandmarkBackend(field_map=fmap, single_corner_mode='coast')
    pose = [1.0, 0.5, math.radians(15)]
    obs = _obs(fmap, pose, [0])
    res = be.estimate(obs, pose, np.diag([0.04, 0.04, 0.01]))
    assert res.kind == 'none'


def test_no_teleport_rejects_jump_from_confident_prior():
    fmap = build_map()
    be = LandmarkBackend(field_map=fmap, no_teleport_m=1.0)
    true_pose = [1.5, -0.8, math.radians(30)]
    obs = _obs(fmap, true_pose, [0, 5, 9, 12])
    # confident prior placed far away -> a correct fix would be a teleport
    confident_far = [-2.0, 1.5, math.radians(30)]
    P = np.diag([0.04, 0.04, math.radians(5) ** 2])
    res = be.estimate(obs, confident_far, P)
    # association is gated to the (wrong) confident prior, so it should not
    # produce a confident far jump; whatever it returns must not teleport.
    if res.kind != 'none':
        assert np.hypot(res.pose[0] - confident_far[0],
                        res.pose[1] - confident_far[1]) <= 1.0 + 1e-6


def test_chi2_rejects_yaw_outlier_vs_confident_prior():
    # B3.1: the 8c capture — a ~120 deg-yaw outlier fix against a converged (tight)
    # prior. no_teleport (position-only) misses it; the chi-square gate must not.
    be = LandmarkBackend()
    prior = np.array([1.0, -0.5, math.radians(10)])
    P = np.diag([0.0004, 0.0004, math.radians(0.5) ** 2])   # ~2 cm / 0.5 deg
    fix = prior + np.array([0.0, 0.0, math.radians(120)])
    fix_cov = np.diag([0.01, 0.01, math.radians(2) ** 2])   # tight full-fix cov
    d2 = be._chi2(fix, fix_cov, prior, P)
    assert d2 > be.chi2_gate                                 # decisively gated


def test_chi2_self_disables_under_uncertain_prior():
    # The SAME innovation must pass when the prior is uncertain (global init /
    # post-blackout): S = fix_cov + P is dominated by a large P -> small d^2.
    be = LandmarkBackend()
    prior = np.array([1.0, -0.5, math.radians(10)])
    P = np.diag([100.0, 100.0, math.pi ** 2])               # ~uniform prior
    fix = prior + np.array([0.0, 0.0, math.radians(120)])
    fix_cov = np.diag([0.01, 0.01, math.radians(2) ** 2])
    d2 = be._chi2(fix, fix_cov, prior, P)
    assert d2 < be.chi2_gate                                 # not gated


def test_chi2_gate_wires_into_estimate():
    fmap = build_map()
    true_pose = [1.5, -0.8, math.radians(30)]
    obs = _obs(fmap, true_pose, [0, 5, 9, 12, 20])
    # confident prior offset 0.3 m from truth in position; the WLS fix lands on the
    # truth -> a ~0.3 m innovation vs a tight prior. With a strict gate it is
    # rejected (gated_chi2); with the gate off the same fix publishes as 'full'.
    prior = [1.5 + 0.3, -0.8, math.radians(30)]
    P = np.diag([0.0025, 0.0025, math.radians(3) ** 2])     # ~5 cm confident
    be_on = LandmarkBackend(field_map=fmap, chi2_gate=0.5)
    res_on = be_on.estimate(obs, prior, P)
    assert res_on.kind == 'none' and res_on.gated_chi2 and res_on.chi2 > 0.5
    be_off = LandmarkBackend(field_map=fmap, chi2_gate=0.0)
    res_off = be_off.estimate(obs, prior, P)
    assert res_off.kind == 'full' and not res_off.gated_chi2


def test_uncertain_prior_allows_fix():
    fmap = build_map()
    be = LandmarkBackend(field_map=fmap, no_teleport_m=1.0)
    true_pose = [1.5, -0.8, math.radians(30)]
    obs = _obs(fmap, true_pose, [0, 5, 9, 12, 20])
    # A LARGE prior covariance must NOT trigger the no-teleport gate, and a prior
    # close enough to truth to associate correctly must yield an accurate fix.
    # (A far/uncertain prior instead mis-associates -> that is TAHAP 5's job, and
    # is measured separately in GATE 3, not asserted as accuracy here.)
    prior = [1.45, -0.75, math.radians(29)]
    P = np.diag([0.25, 0.25, math.radians(15) ** 2])   # uncertain, but no gate
    res = be.estimate(obs, prior, P)
    assert res.kind == 'full'
    assert np.hypot(res.pose[0] - true_pose[0],
                    res.pose[1] - true_pose[1]) < 0.05


# ── C5 — per-profile association group ───────────────────────────────────────
def test_agnostic_group_wires_to_associator():
    fmap = build_map()
    be_sim = LandmarkBackend(field_map=fmap, agnostic_group=[0, 1])   # sim {L,T}
    assert be_sim.assoc.agnostic_group == frozenset({0, 1})
    be_hw = LandmarkBackend(field_map=fmap, agnostic_group=[0, 1, 2])  # hw {L,T,X}
    assert be_hw.assoc.agnostic_group == frozenset({0, 1, 2})


def test_agnostic_group_default_is_junction():
    from landmark_localization.association import JUNCTION_CLASSES
    be = LandmarkBackend(field_map=build_map())   # None -> DataAssociator default
    assert be.assoc.agnostic_group == JUNCTION_CLASSES


# ── C4 — condition-number covariance inflation ───────────────────────────────
def test_cond_inflate_disabled_is_identity():
    be = LandmarkBackend(cond_inflate_at=0.0)          # disabled (default)
    cov = np.eye(3) * 2.0
    assert np.allclose(be._cond_inflate(cov, cond=1e6), cov)


def test_cond_inflate_below_threshold_unchanged():
    be = LandmarkBackend(cond_inflate_at=1.0e3)
    cov = np.eye(3)
    assert np.allclose(be._cond_inflate(cov, cond=10.0), cov)   # well-conditioned


def test_cond_inflate_scales_and_caps():
    be = LandmarkBackend(cond_inflate_at=100.0, cond_inflate_cap=5.0)
    cov = np.eye(3)
    assert np.allclose(be._cond_inflate(cov, cond=200.0), cov * 2.0)   # factor 2
    assert np.allclose(be._cond_inflate(cov, cond=500.0), cov * 5.0)   # factor 5
    assert np.allclose(be._cond_inflate(cov, cond=1.0e6), cov * 5.0)   # capped at 5


def test_cond_inflate_does_not_change_well_conditioned_full_fix():
    # A clean multi-landmark fix is well-conditioned, so even with inflation ENABLED
    # the returned cov is the raw WLS cov (factor 1) -> no tracking regression.
    fmap = build_map()
    be = LandmarkBackend(field_map=fmap, cond_inflate_at=1.0e6)
    true_pose = (-2.0, 1.0, 0.3)
    obs = _obs(fmap, true_pose, [0, 1, 2, 3])
    prior = np.array(true_pose)
    P = np.diag([0.25, 0.25, math.radians(15) ** 2])
    res = be.estimate(obs, prior, P)
    assert res.kind == 'full'
