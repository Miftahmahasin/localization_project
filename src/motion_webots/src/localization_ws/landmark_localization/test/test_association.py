#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro — Apache-2.0
"""Unit tests for TAHAP 3 data association (ROS-free)."""
import math

import numpy as np

from landmark_geometry import error_model
from landmark_localization.association import (
    DataAssociator, AObs, predict_obs, _pred_jacobian)
from landmark_localization.mhl import build_map


def _R(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def _obs_from_truth(fmap, pose, map_indices, noise=0.0, rng=None):
    """Perfect (or noisy) observations of the given map landmarks under pose."""
    px, py, yaw = pose
    Rt = _R(-yaw)
    obs, truth = [], []
    for j in map_indices:
        w = fmap[j].w
        b = Rt @ (w - np.array([px, py]))
        if noise and rng is not None:
            b = b + rng.normal(0, noise, 2)
        cov = np.array(error_model.cov_2x2(float(b[0]), float(b[1]))).reshape(2, 2)
        obs.append(AObs(fmap[j].class_id, b, cov))
        truth.append(j)
    return obs, truth


def test_jacobian_matches_finite_difference():
    pose = [0.7, -1.2, math.radians(25)]
    w = np.array([2.5, 1.0])
    z, H = _pred_jacobian(pose, w)
    assert np.allclose(z, predict_obs(pose, w))
    eps = 1e-6
    Hn = np.zeros((2, 3))
    for k in range(3):
        pp = list(pose); pp[k] += eps
        pm = list(pose); pm[k] -= eps
        Hn[:, k] = (predict_obs(pp, w) - predict_obs(pm, w)) / (2 * eps)
    assert np.allclose(H, Hn, atol=1e-5)


def test_perfect_prior_zero_misassoc():
    fmap = build_map()
    da = DataAssociator(field_map=fmap)
    pose = [1.0, 0.5, math.radians(10)]
    # a handful of nearby landmarks
    idx = [0, 5, 9, 12, 20]
    obs, truth = _obs_from_truth(fmap, pose, idx)
    res = da.associate(obs, pose, np.diag([0.09, 0.09, math.radians(8) ** 2]))
    got = {a.obs_idx: a.map_idx for a in res.assocs}
    for k, j in enumerate(truth):
        assert got[k] == j, f'obs {k} -> {got[k]}, expected {j}'
    assert res.n_inliers == len(idx)


def test_frame_cap_seven():
    fmap = build_map()
    da = DataAssociator(field_map=fmap, max_obs=7)
    pose = [0.0, 0.0, 0.0]
    idx = list(range(15))
    obs, _ = _obs_from_truth(fmap, pose, idx)
    res = da.associate(obs, pose, np.diag([0.09, 0.09, 0.02]))
    assert len(res.assocs) <= 7


def test_false_positive_is_gated_out():
    fmap = build_map()
    da = DataAssociator(field_map=fmap)
    pose = [1.0, 0.0, 0.0]
    obs, truth = _obs_from_truth(fmap, pose, [0, 5, 9])
    # inject a false positive far from any same-class map landmark
    b = np.array([0.4, 0.0])
    cov = np.array(error_model.cov_2x2(0.4, 0.0)).reshape(2, 2)
    obs.append(AObs(3, b, cov))     # a 'goalpost' sitting 0.4 m ahead: impossible
    res = da.associate(obs, pose, np.diag([0.04, 0.04, 0.01]))
    fp = [a for a in res.assocs if a.obs_idx == 3][0]
    assert fp.map_idx < 0, 'implausible FP should be gated out'


def test_ransac_recovers_from_bad_prior():
    fmap = build_map()
    da = DataAssociator(field_map=fmap)
    true_pose = [1.5, -0.8, math.radians(30)]
    idx = [0, 5, 9, 12, 20, 3]
    obs, truth = _obs_from_truth(fmap, true_pose, idx)
    # prior is badly wrong (kidnapped); loose P
    bad_prior = [-1.0, 1.5, math.radians(-40)]
    P = np.diag([1.0, 1.0, math.radians(30) ** 2])
    res = da.associate(obs, bad_prior, P)
    got = {a.obs_idx: a.map_idx for a in res.assocs}
    correct = sum(1 for k, j in enumerate(truth) if got.get(k) == j)
    assert res.used_ransac, 'should have triggered RANSAC fallback'
    assert correct >= len(idx) - 1, f'only {correct}/{len(idx)} recovered'
