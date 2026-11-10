"""Layout and scoring for the conduit run, in plain Python and numpy (no simulator).

World frame: x forward (away from the arm bases), y left, z up, metres. The wall panel lies flat on the
bench with its floor edge nearest the arms, so "up the wall" is +x and "along the wall" is y.

A concealed cable may only run in line with an accessory box: in the vertical strip directly above or below
it, or in the horizontal strip level with it, each as wide as the box. Every conduit length laid on the panel
must lie flat and along one of those strips; a run in any other direction or place scores 0.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

TOP = 0.7475                      # station table height (yam_arena TABLE_TOP["station"])

# Wall panel: an 18 mm OSB face, fixed, 36 cm up the wall (x) by 40 cm along it (y).
PANEL_X = (0.11, 0.47)
PANEL_Y = (-0.20, 0.20)
PANEL_T = 0.018
PANEL_TOP = TOP + PANEL_T

# Accessory boxes, fixed to the panel face: 80 mm square, 20 mm proud of it.
BOX = 0.08
BOX_H = 0.02
SOCKET_XY = (0.19, 0.12)          # the existing socket, low on the left
NEW_BOX_XY = (0.38, -0.1175)      # the empty back box, higher and to the right
BOXES = {"socket": SOCKET_XY, "new_box": NEW_BOX_XY}

# Conduit lengths: rigid 25 mm square PVC, 17 cm long. Three on the bench; two make either legal L.
PIPE_L, PIPE_W, PIPE_H = 0.17, 0.025, 0.025
PIPES = ("conduit_a", "conduit_b", "conduit_c")

# Tolerances.
ZONE_MARGIN = 0.010   # each strip is the box width plus 10 mm either side
ALIGN_DEG = 10.0      # a length's axis within 10 degrees of its strip
JOINT = 0.020         # a joint is made when two outlines are within 20 mm
OPEN_JOINT = 0.050    # a run with every joint within 50 mm is laid but not joined up
LAID_Z = 0.030        # a length whose lowest point is within 30 mm of the face counts as laid
FLAT_Z = 0.006        # ... and must rest on the face: lowest point within 6 mm of it
TILT_DEG = 10.0


@dataclass(frozen=True)
class Pose:
    """What the scorer needs from one conduit length: centre, yaw about vertical, tilt, lowest point, still."""

    x: float
    y: float
    yaw: float           # rad, the long axis measured from world +x
    tilt_deg: float = 0.0
    low_z: float = PANEL_TOP
    resting: bool = True


def rect(cx, cy, length, width, yaw):
    """Corners of a rectangle centred on (cx, cy) with its length along yaw."""
    c, s = math.cos(yaw), math.sin(yaw)
    ux, uy = c * length / 2, s * length / 2
    vx, vy = -s * width / 2, c * width / 2
    return np.array([(cx + ux + vx, cy + uy + vy), (cx + ux - vx, cy + uy - vy),
                     (cx - ux - vx, cy - uy - vy), (cx - ux + vx, cy - uy + vy)])


def box_rect(name):
    x, y = BOXES[name]
    return rect(x, y, BOX, BOX, 0.0)


def pipe_rect(p: Pose):
    return rect(p.x, p.y, PIPE_L, PIPE_W, p.yaw)


def _inside(pt, poly):
    """Point in a convex polygon (corners in order), boundary included."""
    sign = 0
    for i in range(4):
        a, b = poly[i], poly[(i + 1) % 4]
        cross = (b[0] - a[0]) * (pt[1] - a[1]) - (b[1] - a[1]) * (pt[0] - a[0])
        if abs(cross) < 1e-12:
            continue
        s = 1 if cross > 0 else -1
        if sign and s != sign:
            return False
        sign = s
    return True


def _seg_dist(p, q, r, s):
    """Shortest distance between segments pq and rs."""
    def point_seg(a, b, c):
        ab = b - a
        t = 0.0 if not ab.any() else max(0.0, min(1.0, float(np.dot(c - a, ab) / np.dot(ab, ab))))
        return float(np.linalg.norm(a + t * ab - c))
    return min(point_seg(p, q, r), point_seg(p, q, s), point_seg(r, s, p), point_seg(r, s, q))


def _cross(p, q, r, s):
    """Segments pq and rs properly cross."""
    def orient(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    return (orient(p, q, r) * orient(p, q, s) < 0) and (orient(r, s, p) * orient(r, s, q) < 0)


def gap(a, b):
    """Gap between two convex outlines in top view (0 if they touch or overlap)."""
    if any(_inside(v, b) for v in a) or any(_inside(v, a) for v in b):
        return 0.0
    if any(_cross(a[i], a[(i + 1) % 4], b[j], b[(j + 1) % 4]) for i in range(4) for j in range(4)):
        return 0.0
    return min(_seg_dist(a[i], a[(i + 1) % 4], b[j], b[(j + 1) % 4]) for i in range(4) for j in range(4))


def on_panel(p: Pose) -> bool:
    """Laid on the wall panel: low enough to be down on it, with any part of its outline over the face."""
    if p.low_z > PANEL_TOP + LAID_Z:
        return False                      # still in the gripper, above the panel
    return any(PANEL_X[0] < cx < PANEL_X[1] and PANEL_Y[0] < cy < PANEL_Y[1] for cx, cy in pipe_rect(p))


def _axis_off_deg(yaw, along_x):
    """Angle between the length's axis and the x axis (or the y axis), 0..90 degrees."""
    d = abs(math.degrees(yaw)) % 180.0
    off_x = min(d, 180.0 - d)
    return off_x if along_x else 90.0 - off_x


def zone(p: Pose):
    """The strip the length lies in ('socket column', 'new_box row', ...), or None if it is in none."""
    half = BOX / 2 + ZONE_MARGIN
    corners = pipe_rect(p)
    for name, (bx, by) in BOXES.items():
        # Column: straight above or below the box, running up the wall (along x).
        if _axis_off_deg(p.yaw, along_x=True) <= ALIGN_DEG and np.all(np.abs(corners[:, 1] - by) <= half):
            return f"{name} column"
        # Row: level with the box, running along the wall (along y).
        if _axis_off_deg(p.yaw, along_x=False) <= ALIGN_DEG and np.all(np.abs(corners[:, 0] - bx) <= half):
            return f"{name} row"
    return None


def faults(poses: dict[str, Pose]) -> list[str]:
    """Reasons the work scores 0: a laid length out of every zone, not flat on the face, or not still."""
    out = []
    for name, p in poses.items():
        if not on_panel(p):
            continue
        if p.tilt_deg > TILT_DEG or p.low_z > PANEL_TOP + FLAT_Z:
            out.append(f"{name} is not lying flat on the panel face")
        elif not p.resting:
            out.append(f"{name} is still moving")
        elif zone(p) is None:
            out.append(f"{name} is outside the zones in line with the boxes")
    return out


def joined(poses: dict[str, Pose], tol: float) -> bool:
    """A chain of laid lengths from the socket to the new box, each joint within tol."""
    laid = {n: pipe_rect(p) for n, p in poses.items() if on_panel(p)}
    reached = {n for n, r in laid.items() if gap(r, box_rect("socket")) <= tol}
    frontier = list(reached)
    while frontier:
        a = frontier.pop()
        for b, r in laid.items():
            if b not in reached and gap(laid[a], r) <= tol:
                reached.add(b)
                frontier.append(b)
    return any(gap(laid[n], box_rect("new_box")) <= tol for n in reached)


def started(poses: dict[str, Pose]) -> bool:
    """At least one laid length is butted (within the joint tolerance) against a box."""
    return any(on_panel(p) and gap(pipe_rect(p), box_rect(b)) <= JOINT for p in poses.values() for b in BOXES)


def grade(poses: dict[str, Pose]) -> float:
    if faults(poses):
        return 0.0
    if joined(poses, JOINT):
        return 1.0
    if joined(poses, OPEN_JOINT):
        return 0.5
    if started(poses):
        return 0.25
    return 0.0


def yaw_tilt(quat_xyzw):
    """Yaw of the length's long (body x) axis and that axis's tilt from horizontal, from a quaternion.

    The section is square, so a length rolled a quarter turn about its own axis still lies flat; only the
    long axis's slope counts as tilt (a length propped on a box end or on another length slopes)."""
    x, y, z, w = (float(v) for v in quat_xyzw)
    r00 = 1 - 2 * (y * y + z * z)
    r10 = 2 * (x * y + z * w)
    r20 = 2 * (x * z - y * w)
    yaw = math.atan2(r10, r00)
    tilt = math.degrees(math.asin(max(-1.0, min(1.0, abs(r20)))))
    return yaw, tilt
