#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro — Apache-2.0
"""Geometric multi-hypothesis localization from typed landmarks (CLAP/ILM).

No odom prior, no particle filter. Given a set of landmark observations in the
robot base frame (each = a class id + a ground point ``b`` in base_link) and the
known field map, recover the robot pose in ``map`` directly:

  * For every ordered assignment of a detection PAIR to a compatible pair of map
    landmarks, a single rigid transform is determined in closed form
    (CLAP/ILM Eq.5):
        theta = atan2(wB - wA) - atan2(bj - bi)
        p     = wA - R(theta) @ bi
    The correct pose is voted for by MANY consistent pairs; wrong assignments
    scatter. A baseline-length gate (|wB-wA| ~= |bj-bi|) prunes almost all wrong
    assignments before they are ever scored.
  * All surviving hypotheses are clustered in (x, y, yaw); the densest cluster's
    (vote-weighted) centroid is the estimate. This is what lets the robot
    global-relocalize from a uniform prior — exactly what the line-scan AMCL
    could not do.

This module is pure Python/NumPy and ROS-free so it can be swept offline against
the GT sidecar (TAHAP 1) and later wrapped by the runtime backend node (TAHAP 5).
Field-map geometry comes from the single shared ``landmark_geometry`` package.
"""
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from landmark_geometry.field_landmarks import (
    build_line_intersections, build_goalposts, build_center_circle)

# dataset class ids
CLS_L, CLS_T, CLS_X, CLS_GOALPOST, CLS_CIRCLE = 0, 1, 2, 3, 4


@dataclass
class Obs:
    """One landmark observation in the robot base frame."""
    class_id: int
    b: np.ndarray          # (2,) ground point in base_link [m]


@dataclass
class MapLandmark:
    class_id: int
    w: np.ndarray          # (2,) world position [m]
    label: str


@dataclass
class PoseHyp:
    x: float
    y: float
    yaw: float


def build_map() -> List[MapLandmark]:
    """The complete typed field map (25 junctions + 4 posts + 1 circle)."""
    out: List[MapLandmark] = []
    for j in build_line_intersections():
        out.append(MapLandmark(j.yolo_class, np.array([j.x, j.y]), j.label))
    for p in build_goalposts():
        out.append(MapLandmark(p.yolo_class, np.array([p.x, p.y]), p.label))
    for c in build_center_circle():
        out.append(MapLandmark(c.yolo_class, np.array([c.cx, c.cy]), c.label))
    return out


def _R(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class GeometricLocalizer:
    def __init__(self, field_map: Optional[List[MapLandmark]] = None,
                 baseline_tol_m: float = 0.30,
                 baseline_tol_frac: float = 0.06,
                 pos_cluster_m: float = 0.40,
                 yaw_cluster_rad: float = math.radians(15.0),
                 field_half_len: float = 4.6,
                 field_half_wid: float = 3.1,
                 type_agnostic: bool = False):
        self.map = field_map if field_map is not None else build_map()
        self.baseline_tol_m = baseline_tol_m
        self.baseline_tol_frac = baseline_tol_frac
        self.pos_cluster_m = pos_cluster_m
        self.yaw_cluster_rad = yaw_cluster_rad
        self.fhl = field_half_len
        self.fhw = field_half_wid
        self.type_agnostic = type_agnostic
        # index map landmarks by class for fast candidate lookup
        self._by_class = {}
        for m in self.map:
            self._by_class.setdefault(m.class_id, []).append(m)

    def _candidates(self, class_id: int) -> List[MapLandmark]:
        if self.type_agnostic:
            return self.map
        return self._by_class.get(class_id, [])

    def _hypotheses(self, obs: List[Obs]) -> List[PoseHyp]:
        hyps: List[PoseHyp] = []
        n = len(obs)
        for i in range(n):
            for j in range(i + 1, n):
                bi, bj = obs[i].b, obs[j].b
                bl_body = float(np.linalg.norm(bj - bi))
                if bl_body < 0.2:            # too close → bearing ill-conditioned
                    continue
                ang_body = math.atan2(bj[1] - bi[1], bj[0] - bi[0])
                cand_i = self._candidates(obs[i].class_id)
                cand_j = self._candidates(obs[j].class_id)
                tol = self.baseline_tol_m + self.baseline_tol_frac * bl_body
                for A in cand_i:
                    for B in cand_j:
                        if A is B:
                            continue
                        bl_world = float(np.linalg.norm(B.w - A.w))
                        if abs(bl_world - bl_body) > tol:
                            continue         # baseline-length gate (key pruner)
                        ang_world = math.atan2(B.w[1] - A.w[1], B.w[0] - A.w[0])
                        theta = _wrap(ang_world - ang_body)
                        p = A.w - _R(theta) @ bi
                        if abs(p[0]) > self.fhl or abs(p[1]) > self.fhw:
                            continue         # outside the field → impossible
                        hyps.append(PoseHyp(float(p[0]), float(p[1]), theta))
        return hyps

    def _best_cluster(self, hyps: List[PoseHyp]
                      ) -> Optional[Tuple[PoseHyp, int]]:
        if not hyps:
            return None
        used = [False] * len(hyps)
        best = None
        p2 = self.pos_cluster_m ** 2
        for i in range(len(hyps)):
            if used[i]:
                continue
            members = [i]
            for j in range(i + 1, len(hyps)):
                if used[j]:
                    continue
                dx = hyps[i].x - hyps[j].x
                dy = hyps[i].y - hyps[j].y
                if dx * dx + dy * dy <= p2 and \
                        abs(_wrap(hyps[i].yaw - hyps[j].yaw)) <= self.yaw_cluster_rad:
                    members.append(j)
                    used[j] = True
            used[i] = True
            if best is None or len(members) > best[1]:
                # vote-weighted centroid (circular mean for yaw)
                xs = np.mean([hyps[k].x for k in members])
                ys = np.mean([hyps[k].y for k in members])
                sy = np.mean([math.sin(hyps[k].yaw) for k in members])
                cy = np.mean([math.cos(hyps[k].yaw) for k in members])
                yaw = math.atan2(sy, cy)
                best = (PoseHyp(float(xs), float(ys), float(yaw)), len(members))
        return best

    def localize(self, obs: List[Obs]) -> Optional[Tuple[PoseHyp, int, int]]:
        """Return (pose, votes, n_hypotheses) or None if unresolvable.

        Needs >=2 observations; a single landmark cannot fix a full pose (handled
        by odom bridging / single-corner EKF update in the runtime stack).
        """
        if len(obs) < 2:
            return None
        hyps = self._hypotheses(obs)
        if not hyps:
            return None
        best = self._best_cluster(hyps)
        if best is None:
            return None
        return best[0], best[1], len(hyps)
