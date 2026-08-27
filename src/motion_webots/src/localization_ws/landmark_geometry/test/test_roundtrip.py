# Copyright 2026 Bascorro — Apache-2.0
"""Round-trip test for the single shared camera model (TAHAP 0.1 gate).

For any ground point (x, y, 0) that projects in front of the camera, the pinhole
forward projection followed by ``unproject_to_ground`` must return the ORIGINAL
world point to < 1e-6 m. This is the contract that guarantees the geometry used
to LABEL the dataset is the exact geometry used at INFERENCE — a divergence here
would never show up in mAP, only as unexplained localization error.

Cases exercise non-trivial rotation: base yaw, head pan, head tilt, and a
directly-injected camera pose with roll != 0 (a classic axis-convention trap
that only surfaces when roll is non-zero).
"""
import math

import numpy as np

from landmark_geometry.projection import Projector
from landmark_geometry.field_landmarks import (
    build_line_intersections, build_goalposts, build_center_circle)

# canonical Webots intrinsics: FOV 1.3613 @ 1920x1080 -> fx=fy~1184.8, cx=960 cy=540
_W, _H = 1920, 1080
_FX = _W / (2.0 * math.tan(1.3613 / 2.0))
_K = np.array([[_FX, 0.0, _W / 2.0],
               [0.0, _FX, _H / 2.0],
               [0.0, 0.0, 1.0]])

_TOL = 1e-6


def _proj():
    return Projector(_K, _W, _H, max_range_m=30.0, ground_max_range_m=30.0)


def _forward_uv(p, world_pt):
    """Pinhole u,v for a single world point using the projector's own transform.
    Returns (u, v, Zc); Zc>0 means in front of the camera."""
    cam = p._to_cam(np.asarray([world_pt], dtype=np.float64))[0]
    zc = cam[2]
    u = p.fx * cam[0] / zc + p.cx
    v = p.fy * cam[1] / zc + p.cy
    return u, v, zc


def _check_ground_point(p, x, y):
    u, v, zc = _forward_uv(p, (x, y, 0.0))
    assert zc > 1e-3, 'test point must be in front of camera'
    w = p.unproject_to_ground(u, v)
    assert w is not None, 'ground ray must intersect z=0'
    assert abs(w[0] - x) < _TOL and abs(w[1] - y) < _TOL and abs(w[2]) < _TOL, \
        'round-trip world(%.3f,%.3f)->px->world(%.6f,%.6f,%.6f)' % (
            x, y, w[0], w[1], w[2])


def test_roundtrip_chain_poses():
    """Ground points at 0.5..9 m across a spread of realistic head/base poses."""
    p = _proj()
    poses = [
        # base_x, base_y, base_z, yaw, head_pan, head_tilt
        (-0.363, 0.0, 0.30, 0.0, 0.0, math.radians(20)),
        (1.20, -0.40, 0.30, math.radians(35), math.radians(-15), math.radians(28)),
        (-2.0, 1.5, 0.30, math.radians(-110), math.radians(20), math.radians(12)),
        (0.0, 0.0, 0.30, math.radians(175), math.radians(-25), math.radians(35)),
    ]
    for pose in poses:
        p.set_pose(*pose)
        # sweep ground points ahead of the camera at several ranges/bearings
        for rng in (0.5, 1.0, 3.0, 5.0, 9.0):
            for bearing in (-0.5, 0.0, 0.5):  # rad, relative to camera heading
                cam_yaw = pose[3] + pose[4]  # base yaw + head pan
                ang = cam_yaw + bearing
                x = pose[0] + rng * math.cos(ang)
                y = pose[1] + rng * math.sin(ang)
                u, v, zc = _forward_uv(p, (x, y, 0.0))
                if zc <= 1e-3 or not (0 <= u < _W and 0 <= v < _H):
                    continue  # only assert on points that actually image
                _check_ground_point(p, x, y)


def test_roundtrip_with_roll():
    """Inject a camera pose with roll != 0 directly and round-trip through it."""
    p = _proj()
    # build an arbitrary map<-cam transform with roll, pitch, yaw all non-zero
    roll, pitch, yaw = math.radians(8), math.radians(-25), math.radians(40)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    # optical frame (Z fwd, X right, Y down) mounted looking forward+down
    R_opt = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])
    R = Rz @ Ry @ Rx @ R_opt
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [0.5, -0.3, 0.45]
    p._T_map_cam = T
    p._T_cam_map = np.linalg.inv(T)
    p._cam_pos = T[:3, 3].copy()
    for x, y in [(2.0, 0.0), (3.0, 1.0), (1.5, -0.8), (5.0, 0.5)]:
        u, v, zc = _forward_uv(p, (x, y, 0.0))
        if zc <= 1e-3:
            continue
        _check_ground_point(p, x, y)


def test_field_map_counts():
    """The shared field map is the complete 26 junction + 4 post + 1 circle set."""
    lj = build_line_intersections()
    assert len(lj) == 25, len(lj)  # 12 L + 10 T + 3 X
    n = {2: 0, 3: 0, 4: 0}
    for j in lj:
        n[int(j.jtype)] += 1
    assert n[2] == 12 and n[3] == 10 and n[4] == 3, n  # 12 L, 10 T, 3 X
    assert len(build_goalposts()) == 4
    assert len(build_center_circle()) == 1
