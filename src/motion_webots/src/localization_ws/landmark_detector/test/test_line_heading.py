#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro — Apache-2.0
"""TAHAP 6B unit tests — line-heading math (no ROS/OpenCV)."""
import math

from landmark_detector.line_heading import (
    dominant_line_mod90, resolve_yaw, estimate_heading, wrap_pi, HALF_PI)


def _seg(x0, y0, ang, length):
    return ((x0, y0),
            (x0 + length * math.cos(ang), y0 + length * math.sin(ang)))


def test_dominant_direction_agrees():
    # two parallel segments at ~20deg -> dominant ~20deg, high strength
    segs = [_seg(0, 0, math.radians(20), 1.0),
            _seg(1, 1, math.radians(21), 1.2)]
    theta, strength, n, total = dominant_line_mod90(segs)
    assert n == 2
    assert strength > 0.95
    assert abs(theta - math.radians(20)) < math.radians(3)


def test_perpendicular_collapses_mod90():
    # a segment at 20deg and one at 110deg (perpendicular) are the SAME line
    # family modulo 90 -> still one tight cluster.
    segs = [_seg(0, 0, math.radians(20), 1.0),
            _seg(0, 0, math.radians(110), 1.0)]
    theta, strength, n, _ = dominant_line_mod90(segs)
    assert strength > 0.95
    assert abs(theta - math.radians(20)) < math.radians(3)


def test_short_segments_rejected():
    assert dominant_line_mod90([_seg(0, 0, 0.0, 0.05)], min_len_m=0.15) is None


def test_resolve_picks_nearest_quadrant():
    # yaw grid {10, 100, -170, -80} deg; prior near 95deg -> pick 100deg
    yaw_mod90 = math.radians(10)
    yaw, delta = resolve_yaw(yaw_mod90, math.radians(95))
    assert abs(wrap_pi(yaw - math.radians(100))) < 1e-6
    assert delta < math.radians(6)


def test_estimate_recovers_known_heading():
    # robot yaw truth = 100deg; a field line along the field x-axis appears in
    # base_link at theta_base = -yaw (mod 90). Build segments at that bearing and
    # check we recover ~100deg given a rough prior.
    truth = math.radians(100)
    theta_base = (-truth) % HALF_PI
    segs = [_seg(0, 0, theta_base, 1.0), _seg(0.5, 0.3, theta_base, 1.0)]
    res = estimate_heading(segs, prior_yaw=math.radians(92))
    assert res is not None
    yaw, strength, n = res
    assert abs(wrap_pi(yaw - truth)) < math.radians(3)
    assert n == 2


def test_no_prior_suppresses():
    segs = [_seg(0, 0, 0.0, 1.0), _seg(0, 0, 0.0, 1.0)]
    assert estimate_heading(segs, prior_yaw=None) is None


def test_ambiguous_prior_suppresses():
    # prior sits at 45deg between two grid candidates -> must suppress.
    segs = [_seg(0, 0, 0.0, 1.0), _seg(0, 0, 0.0, 1.0)]     # yaw_mod90 = 0
    assert estimate_heading(segs, prior_yaw=math.radians(45),
                            accept_margin_rad=math.radians(30)) is None


def test_disagreeing_segments_suppressed():
    # two segments 45deg apart in mod-90 -> strength low -> suppress
    segs = [_seg(0, 0, math.radians(0), 1.0),
            _seg(0, 0, math.radians(45), 1.0)]
    assert estimate_heading(segs, prior_yaw=0.0, min_strength=0.6) is None
