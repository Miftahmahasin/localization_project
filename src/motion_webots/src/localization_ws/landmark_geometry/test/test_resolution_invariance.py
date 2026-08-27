# Copyright 2026 Bascorro — Apache-2.0
"""TAHAP A1.1 — resolution-invariance gate for the shared camera model.

The deployment concern: hardware cameras run at a different resolution/FOV than
the Webots 1920x1080 the system was tuned on. The `Projector` must be driven by
the intrinsics `K` that MATCH the capture resolution, and its geometry must be
purely `K`-scaled — no pixel constant frozen to 1080p. This test proves that: for
the SAME lens (FOV) and SAME pose, projecting the same world points at 1920x1080,
960x540 and 640x360 gives IDENTICAL results once normalized to relative image
coordinates (u/W, v/H), and the world->px->world round-trip stays < 1e-6 m at
every resolution.

If someone reintroduces a hardcoded pixel constant (a fixed fx, or a cx tied to
960), this test fails. It is the CI guard behind "the pipeline is resolution
invariant as long as a resolution-matched CameraInfo is supplied".
"""
import math

import numpy as np

from landmark_geometry.projection import Projector

_FOV = 1.3613                      # Webots OP3 horizontal FOV [rad] (canonical)
_RESOLUTIONS = [(1920, 1080), (960, 540), (640, 360)]
_POSE = (-0.363, 0.0, 0.30, 0.0, 0.0, math.radians(20))   # base xyz, yaw, pan, tilt


def _k_from_fov(w: int, h: int, fov: float) -> np.ndarray:
    """Derive a pinhole K from FOV + resolution (fx = (W/2)/tan(fov/2))."""
    fx = (w / 2.0) / math.tan(fov / 2.0)
    return np.array([[fx, 0.0, w / 2.0],
                     [0.0, fx, h / 2.0],
                     [0.0, 0.0, 1.0]])


def _projector(w: int, h: int) -> Projector:
    p = Projector(_k_from_fov(w, h, _FOV), w, h,
                  max_range_m=30.0, ground_max_range_m=30.0)
    p.set_pose(*_POSE)
    return p


def _uv(p: Projector, world_pt) -> np.ndarray:
    cam = p._to_cam(np.asarray([world_pt], dtype=np.float64))[0]
    return np.array([p.fx * cam[0] / cam[2] + p.cx,
                     p.fy * cam[1] / cam[2] + p.cy]), cam[2]


# a spread of ground points ahead of the camera (all image in front)
_GROUND_PTS = [(x, y, 0.0)
               for x in (0.5, 1.0, 3.0, 5.0, 9.0)
               for y in (-1.5, 0.0, 1.5)]


def test_normalized_projection_is_resolution_invariant():
    """Same FOV+pose at 3 resolutions -> identical normalized (u/W, v/H)."""
    ref = _projector(*_RESOLUTIONS[0])
    for w, h in _RESOLUTIONS[1:]:
        p = _projector(w, h)
        for wp in _GROUND_PTS:
            (uv_r, zc_r) = _uv(ref, wp)
            (uv_p, zc_p) = _uv(p, wp)
            if zc_r <= 1e-3:
                continue                    # only compare imaged points
            nr = np.array([uv_r[0] / ref.W, uv_r[1] / ref.H])
            npv = np.array([uv_p[0] / w, uv_p[1] / h])
            assert np.allclose(nr, npv, atol=1e-9), (
                'resolution-dependent projection at %dx%d for %s: %s vs %s'
                % (w, h, wp, nr, npv))


def test_roundtrip_holds_at_every_resolution():
    """world->px->world < 1e-6 m at each resolution (no frozen pixel constant)."""
    for w, h in _RESOLUTIONS:
        p = _projector(w, h)
        for wp in _GROUND_PTS:
            (uv, zc) = _uv(p, wp)
            if zc <= 1e-3 or not (0 <= uv[0] < w and 0 <= uv[1] < h):
                continue
            back = p.unproject_to_ground(float(uv[0]), float(uv[1]))
            assert back is not None
            assert abs(back[0] - wp[0]) < 1e-6 and abs(back[1] - wp[1]) < 1e-6 \
                and abs(back[2]) < 1e-6, \
                'roundtrip broke at %dx%d for %s -> %s' % (w, h, wp, back)
