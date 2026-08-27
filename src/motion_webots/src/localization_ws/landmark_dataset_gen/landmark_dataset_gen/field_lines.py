# -*- coding: utf-8 -*-
"""Full painted-line model of the field, reconstructed from the same constants
used to place the junctions (field_landmarks.py).

field_landmarks.py only stores junction POINTS + their meeting directions; it has
no explicit list of the continuous painted segments. TAHAP 1 needs those segments
to project the WHOLE line model over a rendered frame and check, pixel-for-pixel,
that the camera model puts the lines where they actually appear. This module is
that segment list plus a dense (2 cm) ground-point sampler. Geometry only — no
ROS, no Webots.

Segments (all on z=0):
  * outer boundary  : rectangle x=±4.5, y=±3.0  (the two goal lines are its ends)
  * halfway line    : x=0, y∈[-3,+3]
  * goal areas      : both ends, front line x=±3.5 (y=±1.5) + two depth lines
  * penalty areas   : both ends, front line x=±2.5 (y=±2.5) + two depth lines
  * center circle   : radius 0.75 at origin
"""
import math

import numpy as np

from .field_landmarks import (
    FIELD_HALF_LEN, FIELD_HALF_WID,
    GOALAREA_DEPTH, GOALAREA_HALF_WID,
    PENALTY_DEPTH, PENALTY_HALF_WID,
    CENTER_CIRCLE_R,
)


def build_field_segments():
    """Return the painted straight lines as [((x1,y1),(x2,y2)), ...] (z=0)."""
    A, B = FIELD_HALF_LEN, FIELD_HALF_WID
    segs = []

    # outer boundary (sidelines + the two goal/end lines)
    segs += [((-A, -B), (A, -B)), ((-A, B), (A, B))]          # sidelines
    segs += [((-A, -B), (-A, B)), ((A, -B), (A, B))]          # goal/end lines

    # halfway line
    segs.append(((0.0, -B), (0.0, B)))

    # goal & penalty areas at both ends (sx = -1 near goal, +1 far goal)
    for sx in (-1.0, 1.0):
        gx = sx * A                       # goal line x
        # goal area
        gaf = sx * (A - GOALAREA_DEPTH)   # goal-area front x (±3.5)
        segs.append(((gaf, -GOALAREA_HALF_WID), (gaf, GOALAREA_HALF_WID)))   # front
        segs.append(((gx, -GOALAREA_HALF_WID), (gaf, -GOALAREA_HALF_WID)))   # depth
        segs.append(((gx, GOALAREA_HALF_WID), (gaf, GOALAREA_HALF_WID)))     # depth
        # penalty area
        paf = sx * (A - PENALTY_DEPTH)    # penalty-area front x (±2.5)
        segs.append(((paf, -PENALTY_HALF_WID), (paf, PENALTY_HALF_WID)))     # front
        segs.append(((gx, -PENALTY_HALF_WID), (paf, -PENALTY_HALF_WID)))     # depth
        segs.append(((gx, PENALTY_HALF_WID), (paf, PENALTY_HALF_WID)))       # depth

    return segs


def _sample_segment(p1, p2, spacing):
    (x1, y1), (x2, y2) = p1, p2
    length = math.hypot(x2 - x1, y2 - y1)
    n = max(2, int(math.ceil(length / spacing)) + 1)
    t = np.linspace(0.0, 1.0, n)
    xs = x1 + (x2 - x1) * t
    ys = y1 + (y2 - y1) * t
    return np.stack([xs, ys, np.zeros_like(xs)], axis=1)


def build_field_line_points(spacing=0.02, include_circle=True):
    """Dense ground points (Nx3, z=0) sampled along every painted line."""
    chunks = [_sample_segment(a, b, spacing) for (a, b) in build_field_segments()]
    if include_circle:
        circ = 2.0 * math.pi * CENTER_CIRCLE_R
        n = max(64, int(math.ceil(circ / spacing)))
        ang = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
        chunks.append(np.stack([CENTER_CIRCLE_R * np.cos(ang),
                                CENTER_CIRCLE_R * np.sin(ang),
                                np.zeros(n)], axis=1))
    return np.concatenate(chunks, axis=0)


if __name__ == '__main__':
    segs = build_field_segments()
    pts = build_field_line_points()
    print('segments:', len(segs), ' sampled points @2cm:', pts.shape[0])
