#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro — Apache-2.0
"""TAHAP 6B — heading (yaw) from field lines, modulo 90°.

Pure geometry, no ROS/OpenCV, so it is unit-testable in isolation. The node
``line_heading_node`` feeds it ground-plane line SEGMENTS (already projected to
``base_link`` by the SINGLE shared ``landmark_geometry.Projector`` — no second
projection, prompt rule #2) and the current EKF prior yaw, and it returns an
ABSOLUTE heading measurement for the EKF (yaw only; never AMCL, rule #3).

Why modulo 90°. Every RoboCup field line is parallel or perpendicular to the two
field axes, so in the WORLD a line's direction is always ≡ 0 (mod 90°). If a
segment's direction measured in ``base_link`` is θ_base (an axial quantity, mod
180°), then the robot yaw ψ satisfies ψ + θ_base ≡ 0 (mod 90°), i.e.
ψ ≡ −θ_base (mod 90°). Lines therefore pin heading to a 90° grid but CANNOT say
which of the four quadrants — that is fundamentally the field's symmetry, exactly
like the 180° mirror the typed-landmark path faces. We resolve the quadrant with
the EKF prior only (``resolve_yaw``) and, honouring precision>recall (rule #6),
SUPPRESS the measurement whenever the prior does not select one quadrant
cleanly. So line-heading is a heading HOLD/refinement — it shines during the
down-gaze landmark blackout (near-field lines still visible) — not a global
initialiser.

Axial averaging: a segment direction is mod 180°, and the mod-90° collapse of the
perpendicular families is another doubling, so we average exp(i·4θ) (period π/2 in
θ) weighted by segment length; the resultant length is the confidence.
"""
import math
from typing import List, Optional, Sequence, Tuple

HALF_PI = math.pi / 2.0

Point = Sequence[float]           # (x, y) in base_link ground plane [m]
Segment = Tuple[Point, Point]     # (p0, p1)


def wrap_pi(a: float) -> float:
    """Wrap an angle to (-π, π]."""
    a = math.fmod(a, 2.0 * math.pi)
    if a <= -math.pi:
        a += 2.0 * math.pi
    elif a > math.pi:
        a -= 2.0 * math.pi
    return a


def _seg_len_ang(seg: Segment) -> Tuple[float, float]:
    (x0, y0), (x1, y1) = seg
    dx, dy = (x1 - x0), (y1 - y0)
    return math.hypot(dx, dy), math.atan2(dy, dx)   # length, direction (mod π)


def dominant_line_mod90(segments: Sequence[Segment],
                        min_len_m: float = 0.15
                        ) -> Optional[Tuple[float, float, int, float]]:
    """Length-weighted dominant line direction in base_link, collapsed to
    [0, π/2). Returns ``(theta_base_mod90, strength, n_used, total_len)`` or
    ``None`` if no segment is long enough. ``strength`` is the resultant length
    (0..1); ~1 means the kept segments agree on one axial direction.
    """
    sx = sy = 0.0
    total = 0.0
    n = 0
    for seg in segments:
        length, ang = _seg_len_ang(seg)
        if length < min_len_m:
            continue
        sx += length * math.cos(4.0 * ang)
        sy += length * math.sin(4.0 * ang)
        total += length
        n += 1
    if n == 0 or total <= 0.0:
        return None
    strength = math.hypot(sx, sy) / total
    mean_ang = math.atan2(sy, sx) / 4.0          # in [-π/4, π/4]
    theta_mod90 = mean_ang % HALF_PI             # in [0, π/2)
    return theta_mod90, strength, n, total


def resolve_yaw(yaw_mod90: float, prior_yaw: float) -> Tuple[float, float]:
    """Pick the absolute heading on the 90° grid ``yaw_mod90 + k·π/2`` nearest to
    ``prior_yaw``. Returns ``(yaw, delta)`` where ``delta`` is the circular
    distance [rad] to that nearest candidate — small ⇒ the prior selects the
    quadrant confidently; near π/4 ⇒ ambiguous (caller should suppress).
    """
    best_yaw = prior_yaw
    best_delta = math.pi
    for k in range(4):
        cand = wrap_pi(yaw_mod90 + k * HALF_PI)
        d = abs(wrap_pi(cand - prior_yaw))
        if d < best_delta:
            best_delta = d
            best_yaw = cand
    return best_yaw, best_delta


def estimate_heading(segments: Sequence[Segment],
                     prior_yaw: Optional[float],
                     *,
                     min_len_m: float = 0.15,
                     min_strength: float = 0.6,
                     min_segments: int = 2,
                     accept_margin_rad: float = math.radians(30.0)
                     ) -> Optional[Tuple[float, float, int]]:
    """Full line→heading pipeline. Returns ``(yaw, strength, n_used)`` when a
    confident, unambiguous heading is available, else ``None``.

    Suppresses (returns None) when: no prior yaw (cannot break the 90° grid); too
    few / too short segments; the segments disagree (strength below threshold); or
    the prior does not select one quadrant cleanly (delta ≥ accept_margin). This
    is the precision>recall gate — a wrong 90° snap injects a wrong heading, far
    worse than staying silent.
    """
    if prior_yaw is None:
        return None
    dom = dominant_line_mod90(segments, min_len_m=min_len_m)
    if dom is None:
        return None
    theta_mod90, strength, n, _ = dom
    if n < min_segments or strength < min_strength:
        return None
    yaw_mod90 = (-theta_mod90) % HALF_PI
    yaw, delta = resolve_yaw(yaw_mod90, prior_yaw)
    if delta >= accept_margin_rad:
        return None                               # ambiguous quadrant → suppress
    return yaw, strength, n
