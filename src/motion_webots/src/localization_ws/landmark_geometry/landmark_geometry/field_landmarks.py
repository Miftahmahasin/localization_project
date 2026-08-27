#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Authoritative soccer-field landmark model (RoboCup Humanoid KidSize).

Two SEPARATE, authoritative lists in the field-centered world/map frame:

  * ``line_intersections`` — L / T / X junctions derived ONLY from the painted
    field-line markings. This is a geometrically complete superset of the
    reduced 9-crossing AMCL list in
    ``soccer_object_localization/crossing_amcl_constraint.py::build_field_map``,
    reusing its frame convention and type codes (L=2, T=3, X=4).

  * ``goalposts`` — goal-post geometry (base, top, diameter) as its OWN class,
    NOT derived from lines. The point where a post touches the goal line is a
    goalpost, never a junction — junctions come exclusively from
    ``line_intersections``. Where a real line junction coincides with a post
    base (world (±4.5, ±1.3)), BOTH are emitted with their own distinct class.

Frame convention (identical to the localization map + Webots world):
  origin at field center (0,0); +x toward the positive goal line (x=+4.5);
  +y to the left; meters; REP-103. Robot ground-truth is published directly in
  this frame as ``map`` by the Webots controller.

NOTE ON DIMENSIONS: the constants below mirror the values already used across
the localization nodes so this generator stays in the SAME coordinate frame the
localization was tuned against. They are validated against the actually-rendered
``RobocupSoccerField "kid"`` at the verification gate (projected boxes must land
on the painted lines); adjust the constants here if the render disagrees.
"""

from dataclasses import dataclass, field
from enum import IntEnum
from typing import List, Tuple
import math


# ─────────────────────────────────────────────────────────────────────────────
# FIELD DIMENSIONS (meters) — EXACT values of the rendered Webots field
# ``RobocupSoccerField "kid"`` + ``RobocupGoal`` (RoboCup 2021 Humanoid KidSize).
# Verified against the cyberbotics R2025a protos.
#
#   IMPORTANT: the KidSize field has TWO nested boxes per goal — do NOT confuse
#   them with the single simplified 3.1×1.3 "penalty box" used inside the
#   localization AMCL code (crossing_amcl_constraint.py); that value does NOT
#   match what Webots renders and previously mis-placed the L/T junctions.
#     * goal area  : E=1.0 m deep × F=3.0 m wide  → front line x=±3.5, y=±1.5
#     * penalty area: J=2.0 m deep × K=5.0 m wide → front line x=±2.5, y=±2.5
#     * penalty mark: G=1.5 m from goal line       → (±3.0, 0)
# ─────────────────────────────────────────────────────────────────────────────
FIELD_HALF_LEN = 4.5    # center → goal(end) line   (x = ±4.5)   [A=9]
FIELD_HALF_WID = 3.0    # center → sideline         (y = ±3.0)   [B=6]

GOALAREA_DEPTH = 1.0    # E: goal area depth from goal line  → front x=±3.5
GOALAREA_HALF_WID = 1.5  # F/2: goal area half-width          → y=±1.5
PENALTY_DEPTH = 2.0     # J: penalty area depth from goal line → front x=±2.5
PENALTY_HALF_WID = 2.5  # K/2: penalty area half-width         → y=±2.5
PENALTY_MARK_DIST = 1.5  # G: penalty mark distance from goal line → x=±3.0

CENTER_CIRCLE_R = 0.75  # H=1.5 diameter → radius 0.75

GOAL_WIDTH = 2.6        # D: goal mouth width  → posts at y = ±1.3
GOAL_HALF_WID = GOAL_WIDTH / 2.0
# NOTE: the RobocupGoal proto text says post height 1.25 m, but the field as
# actually RENDERED by Webots shows a ~1.5 m post (measured against the known
# 2.6 m goal width, and confirmed numerically: the observed crossbar matches
# top_z≈1.5). We label against what is rendered, so use 1.5.
GOAL_HEIGHT = 1.5       # crossbar height (top of post in frame) — render-matched
POST_DIAMETER = 0.10    # goal-post diameter (post_radius=0.05)

LINE_WIDTH = 0.05       # painted line width (5 cm) — used to give junction
                        # boxes a real on-ground thickness.
JUNCTION_ARM = 0.15     # RADIUS (m) of the compact symmetric ground square used
                        # to size a line-junction box, centred on the junction
                        # point (see projection._junction_points). ~0.30 m box.
MARK_ARM = 0.10         # RADIUS (m) for the small cross MARKS (center mark +
                        # penalty marks) — tighter box than line junctions.


# ─────────────────────────────────────────────────────────────────────────────
# TYPES
# ─────────────────────────────────────────────────────────────────────────────
class JunctionType(IntEnum):
    """Junction type codes — SAME values as crossing_amcl_constraint.py."""
    L = 2
    T = 3
    X = 4


# YOLO class ids (documented in config/classes.txt + data.yaml).
YOLO_CLASS = {
    JunctionType.L: 0,   # L
    JunctionType.T: 1,   # T
    JunctionType.X: 2,   # X
    # goalpost = 3, center_circle = 4 (handled separately below)
}
YOLO_GOALPOST_CLASS = 3
YOLO_CENTERCIRCLE_CLASS = 4

# Penalty marks are treated as class X (cross marks), same as the center mark —
# see the X section of build_line_intersections(). There is deliberately NO
# separate 'penalty_mark' class.
CLASS_NAMES = {0: 'L', 1: 'T', 2: 'X', 3: 'goalpost', 4: 'center_circle'}


Vec2 = Tuple[float, float]


@dataclass
class LineJunction:
    """An L/T/X intersection of painted field lines, in the world frame."""
    x: float
    y: float
    jtype: JunctionType
    label: str
    arm: float = JUNCTION_ARM   # stub half-length used to size the bbox
    # Unit direction(s) (world xy) of the line segments that meet here. Each
    # direction is drawn out from (x,y) by ±JUNCTION_ARM to build the bbox.
    line_dirs: List[Vec2] = field(default_factory=list)

    @property
    def yolo_class(self) -> int:
        return YOLO_CLASS[self.jtype]


@dataclass
class Goalpost:
    """A vertical goal post, modeled as a thin cylinder from base to top."""
    x: float                 # world x of the post base
    y: float                 # world y of the post base
    top_z: float             # height of the post top (crossbar) [m]
    diameter: float          # post diameter [m]
    label: str

    yolo_class: int = YOLO_GOALPOST_CLASS


@dataclass
class CenterCircle:
    """The center circle, a flat ring on the ground at (cx,cy) radius r."""
    cx: float
    cy: float
    radius: float
    label: str

    yolo_class: int = YOLO_CENTERCIRCLE_CLASS


def _unit(dx: float, dy: float) -> Vec2:
    n = math.hypot(dx, dy)
    return (dx / n, dy / n) if n > 1e-9 else (0.0, 0.0)


# ─────────────────────────────────────────────────────────────────────────────
# BUILDERS
# ─────────────────────────────────────────────────────────────────────────────
def build_line_intersections(
        fhl: float = FIELD_HALF_LEN,
        fhw: float = FIELD_HALF_WID,
        ga_depth: float = GOALAREA_DEPTH,
        ga_hw: float = GOALAREA_HALF_WID,
        pa_depth: float = PENALTY_DEPTH,
        pa_hw: float = PENALTY_HALF_WID) -> List[LineJunction]:
    """Complete list of L/T/X junctions from the KidSize field-line markings.

    Painted lines present (per RobocupSoccerField "kid"):
      * end/goal lines at x=±fhl spanning y∈[-fhw,fhw]
      * sidelines at y=±fhw spanning x∈[-fhl,fhl]
      * halfway line at x=0 spanning y∈[-fhw,fhw]
      * GOAL AREA per goal: front line x=±(fhl-ga_depth) over y∈[-ga_hw,ga_hw]
        + side lines from (±(fhl-ga_depth),±ga_hw) to (±fhl,±ga_hw)
      * PENALTY AREA per goal: front line x=±(fhl-pa_depth) over y∈[-pa_hw,pa_hw]
        + side lines from (±(fhl-pa_depth),±pa_hw) to (±fhl,±pa_hw)
    """
    out: List[LineJunction] = []
    ga_x = fhl - ga_depth   # goal-area front line x (=3.5)
    pa_x = fhl - pa_depth   # penalty-area front line x (=2.5)

    # ── L: field corners (end line ⟂ sideline), 4 ────────────────────────────
    # End line (runs along y) and sideline (runs along x) both extend inward:
    #   end line → (0,-sy);  sideline → (-sx,0)
    for sx in (-1, 1):
        for sy in (-1, 1):
            out.append(LineJunction(
                x=sx * fhl, y=sy * fhw, jtype=JunctionType.L,
                label=f'corner_{"R" if sx > 0 else "L"}_{"F" if sy > 0 else "B"}',
                line_dirs=[_unit(-sx, 0.0), _unit(0.0, -sy)]))

    # ── L: goal-area front corners, 4 ────────────────────────────────────────
    # front line along y → (0,-sy);  side line toward goal → (+sx,0)
    for sx in (-1, 1):
        for sy in (-1, 1):
            out.append(LineJunction(
                x=sx * ga_x, y=sy * ga_hw, jtype=JunctionType.L,
                label=f'goalarea_{"R" if sx > 0 else "L"}_'
                      f'{"F" if sy > 0 else "B"}',
                line_dirs=[_unit(0.0, -sy), _unit(sx, 0.0)]))

    # ── L: penalty-area front corners, 4 ─────────────────────────────────────
    for sx in (-1, 1):
        for sy in (-1, 1):
            out.append(LineJunction(
                x=sx * pa_x, y=sy * pa_hw, jtype=JunctionType.L,
                label=f'penalty_{"R" if sx > 0 else "L"}_'
                      f'{"F" if sy > 0 else "B"}',
                line_dirs=[_unit(0.0, -sy), _unit(sx, 0.0)]))

    # ── T: goal-area side line meets the goal line, 4 ────────────────────────
    # Goal line CONTINUES through (±y); goal-area side line meets from inside(-sx)
    for sx in (-1, 1):
        for sy in (-1, 1):
            out.append(LineJunction(
                x=sx * fhl, y=sy * ga_hw, jtype=JunctionType.T,
                label=f'goalareaT_{"R" if sx > 0 else "L"}_'
                      f'{"F" if sy > 0 else "B"}',
                line_dirs=[_unit(0.0, 1.0), _unit(0.0, -1.0), _unit(-sx, 0.0)]))

    # ── T: penalty side line meets the goal line, 4 ──────────────────────────
    for sx in (-1, 1):
        for sy in (-1, 1):
            out.append(LineJunction(
                x=sx * fhl, y=sy * pa_hw, jtype=JunctionType.T,
                label=f'penaltyT_{"R" if sx > 0 else "L"}_'
                      f'{"F" if sy > 0 else "B"}',
                line_dirs=[_unit(0.0, 1.0), _unit(0.0, -1.0), _unit(-sx, 0.0)]))

    # ── T: halfway line meets the sidelines, 2 ───────────────────────────────
    # Sideline CONTINUES through (±x); halfway line meets from inside (-sy).
    for sy in (-1, 1):
        out.append(LineJunction(
            x=0.0, y=sy * fhw, jtype=JunctionType.T,
            label=f'centerT_{"F" if sy > 0 else "B"}',
            line_dirs=[_unit(1.0, 0.0), _unit(-1.0, 0.0), _unit(0.0, -sy)]))

    # ── X: cross MARKS (center mark + penalty marks) ─────────────────────────
    # Small "+" marks painted on the field. Treated as class X (cross marks).
    # Use the smaller MARK_ARM so their box hugs the actual small cross.
    cross = [_unit(0.0, 1.0), _unit(0.0, -1.0),
             _unit(1.0, 0.0), _unit(-1.0, 0.0)]
    out.append(LineJunction(0.0, 0.0, JunctionType.X, 'center_mark',
                            arm=MARK_ARM, line_dirs=list(cross)))
    for sx in (-1, 1):   # penalty marks G=1.5 m from each goal line → x=±3.0
        out.append(LineJunction(
            sx * (fhl - PENALTY_MARK_DIST), 0.0, JunctionType.X,
            f'penalty_mark_{"R" if sx > 0 else "L"}',
            arm=MARK_ARM, line_dirs=list(cross)))

    return out


def build_center_circle(radius: float = CENTER_CIRCLE_R) -> List[CenterCircle]:
    """The single center circle at the field origin."""
    return [CenterCircle(cx=0.0, cy=0.0, radius=radius, label='center_circle')]


def build_goalposts(fhl: float = FIELD_HALF_LEN,
                    ghw: float = GOAL_HALF_WID,
                    top_z: float = GOAL_HEIGHT,
                    diameter: float = POST_DIAMETER) -> List[Goalpost]:
    """The 4 goal posts (2 per goal), base on the goal line at x=±fhl."""
    out: List[Goalpost] = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            out.append(Goalpost(
                x=sx * fhl, y=sy * ghw, top_z=top_z, diameter=diameter,
                label=f'post_{"R" if sx > 0 else "L"}_'
                      f'{"L" if sy > 0 else "R"}'))
    return out


if __name__ == '__main__':
    lj = build_line_intersections()
    gp = build_goalposts()
    n = {JunctionType.L: 0, JunctionType.T: 0, JunctionType.X: 0}
    for j in lj:
        n[j.jtype] += 1
    print(f'line_intersections: {len(lj)}  '
          f'(L={n[JunctionType.L]}, T={n[JunctionType.T]}, X={n[JunctionType.X]})')
    for j in lj:
        print(f'  {j.jtype.name:1s} ({j.x:+.2f},{j.y:+.2f})  {j.label}')
    print(f'goalposts: {len(gp)}')
    for p in gp:
        print(f'  post ({p.x:+.2f},{p.y:+.2f}) top_z={p.top_z}  {p.label}')
    cc = build_center_circle()
    print(f'center_circle: {len(cc)}')
    for c in cc:
        print(f'  circle ({c.cx:+.2f},{c.cy:+.2f}) r={c.radius}  {c.label}')
