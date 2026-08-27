#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro — Apache-2.0
"""TAHAP 5 — MirrorModeTracker unit tests (pure, no ROS)."""
import math

import numpy as np

from landmark_localization.mirror_mode import MirrorModeTracker
from landmark_localization.pose_solve import mirror_pose


def test_commit_from_side_prior():
    t = MirrorModeTracker(start_x_sign=-1.0)
    out, kind = t.resolve(np.array([-2.5, 0.3, 0.1]))
    assert kind == 'commit'
    assert out[0] < 0                       # committed to own half x<0
    assert t.committed


def test_commit_picks_prior_side_even_from_mirror_twin():
    # backend hands the x>0 twin, but the side prior is x<0 -> must flip it
    t = MirrorModeTracker(start_x_sign=-1.0)
    out, kind = t.resolve(np.array([2.5, 0.0, math.pi]))
    assert kind == 'commit'
    assert out[0] < 0                       # picked the x<0 mirror twin
    assert abs(out[0] - (-2.5)) < 1e-6


def test_side_lock_recovers_committed_twin():
    t = MirrorModeTracker(start_x_sign=-1.0)
    t.resolve(np.array([-2.5, 0.0, 0.0]))               # commit at own half
    # association flips: backend now hands the mirror twin (2.5,0,pi)
    out, kind = t.resolve(np.array([2.5, 0.0, math.pi]))
    assert kind == 'ok'
    assert out[0] < 0 and abs(out[0] - (-2.5)) < 0.3    # locked back to x<0


def test_transient_holds_not_lost():
    t = MirrorModeTracker(start_x_sign=-1.0, kidnap_resid_m=2.0, kidnap_frames=8)
    t.resolve(np.array([-2.5, 0.0, 0.0]))
    # one far, off-belief fix -> 'hold' (suppress), never 'lost' on a single frame
    _, kind = t.resolve(np.array([0.5, 0.0, 0.0]))
    assert kind == 'hold'
    # a good fix immediately after resets the contra counter
    _, kind = t.resolve(np.array([-2.45, 0.02, 0.0]))
    assert kind == 'ok'
    assert t._contra == 0


def test_sustained_kidnap_declares_lost():
    t = MirrorModeTracker(start_x_sign=-1.0, kidnap_resid_m=2.0, kidnap_frames=8)
    t.resolve(np.array([-2.5, 0.0, 0.0]))
    # nearest twin sits ~3 m from belief, sustained -> 'lost' at kidnap_frames
    kidnap = np.array([2.5, -3.0, math.pi])   # its mirror (-2.5,3.0,0) ~3m off
    kinds = [t.resolve(kidnap)[1] for _ in range(8)]
    assert kinds[:7] == ['hold'] * 7
    assert kinds[7] == 'lost'


def test_seed_overrides_and_holds_through_blackout():
    # no side prior; commit via an /initialpose-style seed, then feed mirror
    # twins repeatedly (as a blackout re-acquire would) -> never flips side
    t = MirrorModeTracker(start_x_sign=None)
    t.set_ref((-2.0, 1.0, 0.5))
    for _ in range(20):
        good = np.array([-2.0, 1.0, 0.5])
        out, kind = t.resolve(mirror_pose(good))       # hand the wrong twin
        assert kind == 'ok'
        assert out[0] < 0                               # stays on committed side


# ── C2 — ref-blend robustness (quality-gated belief update) ──────────────────
def test_c2_high_residual_fix_does_not_poison_ref():
    # A crouch/fall fix lands NEAR ref in position (passes kidnap gate) but has a
    # garbage yaw AND a high WLS residual -> it must be PUBLISHED ('ok') yet must NOT
    # drag the side-belief ref.
    t = MirrorModeTracker(start_x_sign=-1.0, blend_resid_max=0.30)
    t.set_ref([-2.0, 0.0, 0.0])
    ref_before = t.ref.copy()
    out, kind = t.resolve(np.array([-2.0, 0.0, math.radians(140)]),
                          resid_m=0.9)          # near in xy, garbage yaw, HIGH resid
    assert kind == 'ok'                          # still published
    assert np.allclose(t.ref, ref_before)        # ref NOT moved (protected)


def test_c2_clean_fix_updates_ref():
    t = MirrorModeTracker(start_x_sign=-1.0, blend_resid_max=0.30)
    t.set_ref([-2.0, 0.0, 0.0])
    out, kind = t.resolve(np.array([-1.8, 0.1, math.radians(3)]),
                          resid_m=0.05)          # clean, low residual
    assert kind == 'ok'
    assert not np.allclose(t.ref, [-2.0, 0.0, 0.0])   # ref DID move toward the fix
    assert t.ref[0] > -2.0                              # blended toward x=-1.8


def test_c2_unknown_residual_is_trusted():
    # single-corner / unknown residual (-1) stays backward-compatible: it blends.
    t = MirrorModeTracker(start_x_sign=-1.0, blend_resid_max=0.30)
    t.set_ref([-2.0, 0.0, 0.0])
    out, kind = t.resolve(np.array([-1.8, 0.0, 0.0]), resid_m=-1.0)
    assert kind == 'ok'
    assert t.ref[0] > -2.0                              # blended (trusted)
