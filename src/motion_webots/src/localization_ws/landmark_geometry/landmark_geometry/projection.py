#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""World→pixel projection, visibility, and bounding-box construction.

Builds the exact camera optical pose in the world frame from:
  * robot base pose (x, y, z, yaw) from /ground_truth/odom  (roll/pitch = 0),
  * head_pan / head_tilt from /robotis_op3/joint_states,
  * the exact OP3 URDF head chain (robotis_op3.structure.head.xacro),
then projects with the pinhole K taken from /robotis_op3/camera/camera_info.

URDF chain (base = body_link, base→body_link is identity in the root xacro):
  base → head_pan  : T(-0.001, 0, 0.1365) · Rz(θ_pan)          axis (0,0,1)
       → head_tilt : T( 0.010, 0.019, 0.0285) · Ry(-θ_tilt)    axis (0,-1,0)
       → cam(fixed): T(0.01425,-0.019,0.04975) · Rrpy(-π/2,0,-π/2)
       → cam_link (optical frame: Z forward, X right, Y down)

Small calibration offsets (base_z_offset, pitch_bias_deg, pan_bias_deg) default
to 0 and are meant to absorb any residual Webots-origin-vs-URDF-base mismatch;
they are tuned ONCE at the verification gate so projected boxes land exactly on
the painted lines. Rotation/intrinsic conventions mirror
``soccer_object_localization/camera_pose_provider.py::CameraPose``.
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from .field_landmarks import (LineJunction, Goalpost, CenterCircle,
                              LINE_WIDTH, JUNCTION_ARM, CLASS_NAMES)

# ── URDF head-chain constants (meters / radians) ─────────────────────────────
_PAN_XYZ = (-0.001, 0.0, 0.1365)
_TILT_XYZ = (0.010, 0.019, 0.0285)
_CAM_XYZ = (0.01425, -0.019, 0.04975)
_CAM_RPY = (-math.pi * 0.5, 0.0, -math.pi * 0.5)

# ── label-culling thresholds (TAHAP 5) ───────────────────────────────────────
MAX_CLIP_FRAC = 0.30       # drop a junction box if the frame edge eats >30% of it
POST_OCCL_MARGIN = 0.03    # extra m added to post radius for occlusion ray test


# ── homogeneous transform helpers ────────────────────────────────────────────
def _trans(x: float, y: float, z: float) -> np.ndarray:
    T = np.eye(4)
    T[0, 3], T[1, 3], T[2, 3] = x, y, z
    return T


def _rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])


def _rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])


def _rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])


def _rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    # URDF fixed-axis XYZ: R = Rz(yaw) · Ry(pitch) · Rx(roll)
    return _rot_z(yaw) @ _rot_y(pitch) @ _rot_x(roll)


@dataclass
class Detection:
    """A projected, visible landmark as a pixel-space axis-aligned box."""
    class_id: int
    x1: float
    y1: float
    x2: float
    y2: float
    label: str
    base_clipped: bool = False   # goalpost: base fell outside the frame

    @property
    def class_name(self) -> str:
        return CLASS_NAMES.get(self.class_id, str(self.class_id))


class Projector:
    """Projects world landmarks to image boxes for a given robot/camera pose."""

    def __init__(self, K: np.ndarray, width: int, height: int,
                 max_range_m: float = 9.0,
                 min_box_px: float = 6.0,
                 min_emit_px: float = 12.0,
                 base_z_offset: float = 0.0,
                 pitch_bias_deg: float = 0.0,
                 pan_bias_deg: float = 0.0,
                 ground_max_range_m: Optional[float] = None):
        self.fx = float(K[0, 0])
        self.fy = float(K[1, 1])
        self.cx = float(K[0, 2])
        self.cy = float(K[1, 2])
        self.W = int(width)
        self.H = int(height)
        self.max_range = float(max_range_m)
        # Ground landmarks (junctions, marks, circle) sit near the horizon at
        # long range, where a sub-degree calibration residual maps to a large
        # vertical pixel error (boxes "float" above the line). Gate them at a
        # tighter range than tall goalposts, which stay reliable far out.
        self.ground_max_range = (float(ground_max_range_m)
                                 if ground_max_range_m is not None
                                 else float(max_range_m))
        self.min_box_px = float(min_box_px)
        # Far landmarks project to sub-pixel-thin boxes. Instead of dropping
        # them, pad every emitted box to at least this size (centered) so far
        # junctions/marks still get a usable, trainable box.
        self.min_emit_px = float(min_emit_px)
        self.base_z_offset = float(base_z_offset)
        self.pitch_bias = math.radians(pitch_bias_deg)
        self.pan_bias = math.radians(pan_bias_deg)

        # camera pose state (set via set_pose)
        self._T_map_cam = np.eye(4)
        self._T_cam_map = np.eye(4)
        self._cam_pos = np.zeros(3)

    # ── pose ────────────────────────────────────────────────────────────────
    def set_pose(self, base_x: float, base_y: float, base_z: float,
                 yaw: float, head_pan: float, head_tilt: float) -> None:
        """Recompute the camera↔world transform for the current robot pose."""
        T_map_base = _trans(base_x, base_y, base_z + self.base_z_offset) \
            @ _rot_z(yaw)
        T_base_pan = _trans(*_PAN_XYZ) @ _rot_z(head_pan + self.pan_bias)
        # head_tilt joint axis is (0,-1,0) → rotation of +θ about -Y = Ry(-θ)
        T_pan_tilt = _trans(*_TILT_XYZ) @ _rot_y(-(head_tilt + self.pitch_bias))
        T_tilt_cam = _trans(*_CAM_XYZ) @ _rpy(*_CAM_RPY)

        self._T_map_cam = T_map_base @ T_base_pan @ T_pan_tilt @ T_tilt_cam
        self._T_cam_map = np.linalg.inv(self._T_map_cam)
        self._cam_pos = self._T_map_cam[:3, 3].copy()

    def set_pose_matrix(self, T_map_cam: np.ndarray) -> None:
        """Set the camera-optical pose in the world/map frame directly.

        Bypasses the URDF head chain — use this when the camera pose is known as
        a 4x4 transform (from the GT sidecar's cam_world_pos/quat, or a runtime
        TF lookup at the image stamp). The optical-frame convention must match
        set_pose (Z forward, X right, Y down).
        """
        T = np.asarray(T_map_cam, dtype=np.float64).reshape(4, 4)
        self._T_map_cam = T
        self._T_cam_map = np.linalg.inv(T)
        self._cam_pos = T[:3, 3].copy()

    # ── low-level projection ──────────────────────────────────────────────────
    def _to_cam(self, pts_world: np.ndarray) -> np.ndarray:
        """(N,3) world → (N,3) camera-optical coords (Z fwd, X right, Y down)."""
        n = pts_world.shape[0]
        homog = np.hstack([pts_world, np.ones((n, 1))])
        return (self._T_cam_map @ homog.T).T[:, :3]

    def _project(self, pts_world: np.ndarray,
                 range_limit: Optional[float] = None
                 ) -> Tuple[np.ndarray, np.ndarray]:
        """Return (uv (N,2), valid (N,) bool) for points in front of camera.

        valid = in front (Zc>0) AND within useful range (range_limit, or the
        overall max_range when not given).
        """
        rl = self.max_range if range_limit is None else float(range_limit)
        cam = self._to_cam(pts_world)
        Zc = cam[:, 2]
        in_front = Zc > 1e-3
        # range gate on the euclidean camera→point distance
        dist = np.linalg.norm(pts_world - self._cam_pos[None, :], axis=1)
        valid = in_front & (dist <= rl)

        uv = np.full((pts_world.shape[0], 2), np.nan)
        zc = np.where(in_front, Zc, 1.0)
        uv[:, 0] = self.fx * cam[:, 0] / zc + self.cx
        uv[:, 1] = self.fy * cam[:, 1] / zc + self.cy
        return uv, valid

    def unproject_to_ground(self, u: float, v: float) -> Optional[np.ndarray]:
        """Inverse of _project for a ground point: pixel → camera ray → the point
        where that ray meets the z=0 world plane. Returns the world (x,y,0) point,
        or None if the ray is parallel to / points away from the ground (no hit in
        front of the camera). Exact inverse of the pinhole projection above.
        """
        d_cam = np.array([(u - self.cx) / self.fx,
                          (v - self.cy) / self.fy, 1.0])
        d_world = self._T_map_cam[:3, :3] @ d_cam
        if abs(d_world[2]) < 1e-12:
            return None                      # ray parallel to the ground plane
        t = -self._cam_pos[2] / d_world[2]
        if t <= 0.0:
            return None                      # ground is behind the camera
        return self._cam_pos + t * d_world

    def _bbox_from_points(self, pts_world: np.ndarray,
                          range_limit: Optional[float] = None
                          ) -> Optional[Tuple[float, float, float, float, bool]]:
        """Build an in-frame bbox from a cloud of world points.

        Returns (x1,y1,x2,y2, any_out_of_frame) clipped to the image, or None
        if nothing visible / box too small. any_out_of_frame flags that some
        projected geometry fell outside the frame (used for goalpost base flag).
        """
        uv, valid = self._project(pts_world, range_limit=range_limit)
        if not np.any(valid):
            return None
        vis = uv[valid]

        # Require at least one visible point INSIDE the frame — otherwise the
        # landmark is off-screen (e.g. just past an edge) and must not be
        # emitted as a box clamped to the border.
        inframe = ((vis[:, 0] >= 0) & (vis[:, 0] < self.W) &
                   (vis[:, 1] >= 0) & (vis[:, 1] < self.H))
        if not np.any(inframe):
            return None

        x1, y1 = float(np.min(vis[:, 0])), float(np.min(vis[:, 1]))
        x2, y2 = float(np.max(vis[:, 0])), float(np.max(vis[:, 1]))
        out_of_frame = bool(np.any(~inframe))

        # Pad to a minimum size (centered) so FAR landmarks — which project to
        # sub-pixel-thin boxes — still get a usable, trainable box instead of
        # being dropped.
        cxm = 0.5 * (x1 + x2)
        cym = 0.5 * (y1 + y2)
        half = 0.5 * self.min_emit_px
        x1 = min(x1, cxm - half)
        x2 = max(x2, cxm + half)
        y1 = min(y1, cym - half)
        y2 = max(y2, cym + half)

        # clip to image
        cx1 = max(0.0, min(x1, self.W - 1.0))
        cy1 = max(0.0, min(y1, self.H - 1.0))
        cx2 = max(0.0, min(x2, self.W - 1.0))
        cy2 = max(0.0, min(y2, self.H - 1.0))
        if (cx2 - cx1) < 1.0 or (cy2 - cy1) < 1.0:
            return None
        return (cx1, cy1, cx2, cy2, out_of_frame)

    # ── landmark → detection ──────────────────────────────────────────────────
    def project_junction(self, j: LineJunction) -> Optional[Detection]:
        """Project a junction into a SQUARE box centered on the junction point.

        A junction is a POINT feature. Its box marks a fixed physical
        neighbourhood (±``arm`` m) around the point, sized by the pinhole
        projection of that patch AT THE JUNCTION'S DEPTH:

            s_px = fx * (2*arm) / depth ;  half = clamp(0.5*s_px, MIN, HALF_FRAME)

        This is square by construction (same half in u and v) and centred on the
        junction (``u0,v0`` ± half), so the crossing sits at the box middle at
        every distance. It replaces the old scheme that took INDEPENDENT half_w /
        half_h from the projected ground-patch spread: foreshortening made that
        spread far wider than tall (aspect up to ~7:1) and the min-size floor then
        pinned the height, yielding wide-and-short boxes that hugged the line.
        Downstream (CLAP/ILM) only consumes the box CENTRE, so a consistent
        square carries all the geometry the pipeline needs.
        """
        arm = float(getattr(j, 'arm', JUNCTION_ARM))
        pt = np.array([[j.x, j.y, 0.0]], dtype=np.float64)
        # depth (camera-frame +Z) drives the pinhole size; guaranteed > 0 when
        # the point is in front (checked via the validity gate below).
        depth = float(self._to_cam(pt)[0, 2])
        uv, valid = self._project(pt, range_limit=self.ground_max_range)
        # the junction point itself must be visible, in range, and in-frame
        if not valid[0]:
            return None
        u0, v0 = float(uv[0, 0]), float(uv[0, 1])
        if not (0.0 <= u0 < self.W and 0.0 <= v0 < self.H):
            return None
        s_px = self.fx * (2.0 * arm) / max(depth, 1e-3)
        half = min(max(0.5 * s_px, 0.5 * self.min_emit_px), 0.5 * self.H)
        x1, y1 = u0 - half, v0 - half
        x2, y2 = u0 + half, v0 + half
        # clip to image; drop if the frame edge eats >30% of the box (a heavily
        # clipped box no longer sits centered on the junction — TAHAP 5).
        cx1 = max(0.0, min(x1, self.W - 1.0))
        cy1 = max(0.0, min(y1, self.H - 1.0))
        cx2 = max(0.0, min(x2, self.W - 1.0))
        cy2 = max(0.0, min(y2, self.H - 1.0))
        if (cx2 - cx1) < 1.0 or (cy2 - cy1) < 1.0:
            return None
        full = (x2 - x1) * (y2 - y1)
        if full > 0.0 and (cx2 - cx1) * (cy2 - cy1) < (1.0 - MAX_CLIP_FRAC) * full:
            return None
        return Detection(j.yolo_class, cx1, cy1, cx2, cy2, j.label)

    def project_goalpost(self, p: Goalpost) -> Optional[Detection]:
        """Project a vertical post into a tall-thin box; flag clipped base."""
        pts, base_pt = self._goalpost_points(p)
        box = self._bbox_from_points(pts)
        if box is None:
            return None
        x1, y1, x2, y2, out_of_frame = box

        # Is the post BASE itself visible & in-frame? bottom of box = base.
        base_uv, base_valid = self._project(base_pt[None, :])
        base_in = bool(base_valid[0]) and \
            (0 <= base_uv[0, 0] < self.W) and (0 <= base_uv[0, 1] < self.H)
        return Detection(p.yolo_class, x1, y1, x2, y2, p.label,
                         base_clipped=not base_in)

    def project_center_circle(self, c: CenterCircle) -> Optional[Detection]:
        """Project the ground circle → compact bbox, or None if ill-defined.

        A bbox is a good label ONLY when the circle appears as a compact,
        fully-visible ellipse (robot far enough, facing it). When the robot is
        near/inside the circle the circumference wraps around and part of it
        falls behind the camera or off-frame, giving a degenerate full-width
        strip. We therefore emit the box ONLY if the *entire* circumference is
        in front of the camera, within range, and inside the frame; otherwise
        we skip (handled later by a dedicated ellipse-fit stage if needed).
        """
        pts = self._circle_points(c)
        cam = self._to_cam(pts)
        Zc = cam[:, 2]
        # whole circle must be in front of the camera (no wrap-around)
        if np.any(Zc <= 0.05):
            return None
        # whole circle within useful range (compact / far, not enveloping us)
        dist = np.linalg.norm(pts - self._cam_pos[None, :], axis=1)
        if np.any(dist > self.ground_max_range):
            return None
        uv = np.empty((pts.shape[0], 2), dtype=np.float64)
        uv[:, 0] = self.fx * cam[:, 0] / Zc + self.cx
        uv[:, 1] = self.fy * cam[:, 1] / Zc + self.cy
        # whole circumference must be inside the frame (compact ellipse)
        if not np.all((uv[:, 0] >= 0) & (uv[:, 0] < self.W) &
                      (uv[:, 1] >= 0) & (uv[:, 1] < self.H)):
            return None
        x1, y1 = float(uv[:, 0].min()), float(uv[:, 1].min())
        x2, y2 = float(uv[:, 0].max()), float(uv[:, 1].max())
        w, h = x2 - x1, y2 - y1
        # Require a minimum extent in BOTH dims: a full-circle-in-frame view is
        # a valid (if foreshortened) ellipse; only reject sub-floor slivers.
        if w < self.min_emit_px or h < self.min_emit_px:
            return None
        return Detection(c.yolo_class, x1, y1, x2, y2, c.label)

    # ── point-cloud builders ──────────────────────────────────────────────────
    def _junction_points(self, j: LineJunction) -> np.ndarray:
        """Junction point + a small SYMMETRIC ±r square around it (ground plane).

        The junction is a POINT feature, so its box is sized from a compact
        symmetric neighbourhood centred on the point — NOT one-directional line
        stubs, which pushed the junction to the box edge and made the box sprawl
        into empty grass. project_junction centres the box on the junction and
        takes its half-size from this spread (floored to min_emit_px), giving a
        compact, centred box that scales with distance. `arm` acts as the radius:
        JUNCTION_ARM for line junctions, the shorter MARK_ARM for cross marks.
        """
        r = float(getattr(j, 'arm', JUNCTION_ARM))
        pts = [(j.x, j.y, 0.0)]
        for dx in (-r, r):
            for dy in (-r, r):
                pts.append((j.x + dx, j.y + dy, 0.0))
        return np.asarray(pts, dtype=np.float64)

    def _goalpost_points(self, p: Goalpost
                         ) -> Tuple[np.ndarray, np.ndarray]:
        """Prism corners+edges around the vertical post (base..top)."""
        r = p.diameter * 0.5
        zs = np.linspace(0.0, p.top_z, 6)
        pts = []
        for z in zs:
            for sx in (-1, 1):
                for sy in (-1, 1):
                    pts.append((p.x + sx * r, p.y + sy * r, z))
        base_pt = np.array([p.x, p.y, 0.0], dtype=np.float64)
        return np.asarray(pts, dtype=np.float64), base_pt

    def _circle_points(self, c: CenterCircle, n: int = 48) -> np.ndarray:
        """Points sampled around the circle circumference on the ground."""
        ang = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
        xs = c.cx + c.radius * np.cos(ang)
        ys = c.cy + c.radius * np.sin(ang)
        zs = np.zeros_like(xs)
        return np.stack([xs, ys, zs], axis=1)

    # ── batch ─────────────────────────────────────────────────────────────────
    def _occluded_by_posts(self, jx: float, jy: float,
                           posts: List[Goalpost]) -> bool:
        """True if a goalpost stands BETWEEN the camera and this ground junction.

        Test in the horizontal plane: the post occludes when the camera→junction
        ray passes within (post_radius + margin) of the post axis AND the closest
        approach happens strictly before the junction (post is nearer). The ray
        z stays between 0 and the camera height (<0.5 m) < post top (1.5 m), so a
        vertical post always spans the crossing height — no z test needed.
        """
        a = self._cam_pos[:2]
        b = np.array([jx, jy])
        ab = b - a
        denom = float(ab @ ab)
        if denom < 1e-9:
            return False
        for p in posts:
            r = p.diameter * 0.5 + POST_OCCL_MARGIN
            pc = np.array([p.x, p.y])
            t = float((pc - a) @ ab) / denom
            if not (0.02 < t < 0.98):      # post must be BETWEEN cam and junction
                continue
            closest = a + t * ab
            if float(np.hypot(*(closest - pc))) < r:
                return True
        return False

    def project_all(self, junctions: List[LineJunction],
                    posts: List[Goalpost],
                    circles: Optional[List[CenterCircle]] = None
                    ) -> List[Detection]:
        dets: List[Detection] = []
        for j in junctions:
            d = self.project_junction(j)
            if d is None:
                continue
            if self._occluded_by_posts(j.x, j.y, posts):
                continue               # a goalpost stands in front of it
            dets.append(d)
        for p in posts:
            d = self.project_goalpost(p)
            if d is not None:
                dets.append(d)
        for c in (circles or []):
            d = self.project_center_circle(c)
            if d is not None:
                dets.append(d)
        return dets
