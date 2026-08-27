# -*- coding: utf-8 -*-
"""TAHAP 3 — project ↔ unproject round-trip, no renderer, no Webots.

Catches axis-convention and intrinsics bugs purely in code. A ground point taken
world→pixel→world must return to itself (< 1e-6 m); a pixel taken pixel→world→
pixel must return to itself (< 1e-4 px). Tested at 0.5/1/3/5/9 m, near the frame
corners, and — importantly — at camera poses with ROLL and YAW != 0, since an
axis-convention bug often only shows up when roll != 0.
"""
import math
import os
import sys

import numpy as np
import pytest

# make the package importable when pytest is run from the package root
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from landmark_dataset_gen.projection import Projector   # noqa: E402

W, H = 1920, 1080
FOV = 1.3613
FX = W / (2.0 * math.tan(FOV * 0.5))
K = np.array([[FX, 0, W / 2.0], [0, FX, H / 2.0], [0, 0, 1.0]])
CAM_H = 0.45
DISTS = [0.5, 1.0, 3.0, 5.0, 9.0]
# (yaw_rad, roll_deg) — roll is applied about the optical (viewing) axis, so the
# frustum still points at the ground; only the image rotates.
POSES = [(0.0, 0.0), (0.7, 0.0), (-1.2, 0.0),
         (0.0, 15.0), (0.6, -20.0), (-1.5, 25.0)]


def _optical_roll(T, roll_deg):
    """Rotate the optical frame about its own +Z (roll) — keeps view forward."""
    a = math.radians(roll_deg)
    Rz = np.eye(4)
    Rz[0, 0], Rz[0, 1] = math.cos(a), -math.sin(a)
    Rz[1, 0], Rz[1, 1] = math.sin(a), math.cos(a)
    return T @ Rz


def make_projector(yaw, roll_deg, tilt_deg=-25.0):
    P = Projector(K, W, H, max_range_m=20.0)
    P.set_pose(0.0, 0.0, CAM_H, yaw, 0.0, math.radians(tilt_deg))
    if roll_deg:
        P._T_map_cam = _optical_roll(P._T_map_cam, roll_deg)
        P._T_cam_map = np.linalg.inv(P._T_map_cam)
        P._cam_pos = P._T_map_cam[:3, 3].copy()
    return P


@pytest.mark.parametrize('yaw,roll', POSES)
def test_world_pixel_world(yaw, roll):
    """Ground points at fixed distances ahead → pixel → back to the same point."""
    P = make_projector(yaw, roll)
    checked = 0
    for d in DISTS:
        gp = np.array([d * math.cos(yaw), d * math.sin(yaw), 0.0])
        uv, valid = P._project(gp[None, :])
        assert valid[0], (yaw, roll, d)          # must be in front / in range
        back = P.unproject_to_ground(uv[0, 0], uv[0, 1])
        assert back is not None
        assert np.linalg.norm(back - gp) < 1e-6, (yaw, roll, d,
                                                  np.linalg.norm(back - gp))
        checked += 1
    assert checked == len(DISTS)


@pytest.mark.parametrize('yaw,roll', POSES)
def test_pixel_world_pixel(yaw, roll):
    """Pixels across the frame (incl. corners) → ground → back to same pixel."""
    P = make_projector(yaw, roll)
    us = [30, W // 2, W - 30]
    vs = [int(0.58 * H), int(0.78 * H), H - 20]   # lower frame = ground ahead
    tested = 0
    for u in us:
        for v in vs:
            gp = P.unproject_to_ground(u, v)
            if gp is None:
                continue
            uv, valid = P._project(gp[None, :])
            if not valid[0]:
                continue
            assert abs(uv[0, 0] - u) < 1e-4 and abs(uv[0, 1] - v) < 1e-4, \
                (yaw, roll, u, v, uv[0])
            tested += 1
    assert tested >= 4                            # corners + interior covered


def test_unproject_rejects_sky():
    """A pixel above the horizon (ray points up) has no ground hit.

    With a horizontal camera (tilt 0) the top of the frame is ~24.5 deg above
    the horizon, so its ray points up and never meets the z=0 plane in front.
    """
    P = make_projector(0.0, 0.0, tilt_deg=0.0)
    assert P.unproject_to_ground(W // 2, 5) is None


if __name__ == '__main__':
    sys.exit(pytest.main([os.path.abspath(__file__), '-v']))
