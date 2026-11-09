"""Pose-only, simulator-independent scoring for the draft filter task.

Positions are metres in the table frame.  A state has ``pos`` and
``quat_xyzw`` attributes, as in yam_arena's ObjectState / PartState.
"""

from __future__ import annotations

from collections import deque
from itertools import product
import math

PLAQUE_SIZE = (0.060, 0.060, 0.020)
STACK_XY = (0.300, 0.055)
LIGHTBOX_H = 0.010
FILTER_NAMES = ("amber", "cyan", "violet", "lime")
REQUIRED = frozenset(("amber", "cyan"))
REFERENCE = "reference"

CENTRE_TOL = 0.005       # metres, along each table axis
SUPPORT_GAP = 0.0005     # max positive gap to the support, m (provisional)
SUPPORT_OVERLAP = 0.0005 # max penetration below the support, m (provisional)
MAX_TILT = math.radians(3)
STILL_STEPS = 20          # one second at 20 Hz
STILL_POSITION = 0.002    # max distance from final position over those samples, m
STILL_ROTATION = math.radians(2)
REF_POSITION = 0.003      # displacement from pose recorded after reset, m
REF_ROTATION = math.radians(2)
TOUCH_Z = 0.003           # include misplaced extras touching a credited stack, m
TOUCH_OVERLAP = 0.001     # projected overlap (separating-axis depth), m
EPS = 1e-9


def _pose(state):
    p = tuple(float(v) for v in state.pos)
    q = tuple(float(v) for v in state.quat_xyzw)
    if len(p) != 3 or len(q) != 4 or not all(map(math.isfinite, (*p, *q))):
        raise ValueError("non-finite or malformed pose")
    n = math.sqrt(sum(v * v for v in q))
    if n < 1e-12:
        raise ValueError("zero quaternion")
    return p, tuple(v / n for v in q)


def _distance(p, q):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(p, q)))


def _angle(a, b):
    # q and -q encode the same rotation.
    dot = min(1.0, abs(sum(x * y for x, y in zip(a, b))))
    return 2 * math.acos(dot)


def _rotation(q):
    x, y, z, w = q
    return ((1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)),
            (2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)),
            (2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)))


def _hull(points):
    """Convex hull of projected corners, for rotated and tilted plaques."""
    pts = sorted(set(points))
    if len(pts) < 3:
        return pts

    def half(seq):
        out = []
        for p in seq:
            while len(out) >= 2 and ((out[-1][0] - out[-2][0]) * (p[1] - out[-2][1])
                                     - (out[-1][1] - out[-2][1]) * (p[0] - out[-2][0])) <= 0:
                out.pop()
            out.append(p)
        return out[:-1]
    return half(pts) + half(reversed(pts))


def _geometry(state):
    p, q = _pose(state)
    r = _rotation(q)
    corners = []
    for signs in product((-1, 1), repeat=3):
        c = tuple(0.5 * s * d for s, d in zip(signs, PLAQUE_SIZE))
        corners.append(tuple(p[i] + sum(r[i][j] * c[j] for j in range(3)) for i in range(3)))
    # The sheet is above the rails. An upside-down plaque is not equivalent.
    return {"pos": p, "quat": q, "flat": r[2][2] >= math.cos(MAX_TILT) - EPS,
            "bottom": min(c[2] for c in corners), "top": max(c[2] for c in corners),
            "hull": _hull([(c[0], c[1]) for c in corners])}


def _projected_overlap(a, b):
    """Smallest overlap on all polygon edge normals; negative means apart."""
    depth = math.inf
    for polygon in (a, b):
        for p, q in zip(polygon, polygon[1:] + polygon[:1]):
            dx, dy = q[0] - p[0], q[1] - p[1]
            norm = math.hypot(dx, dy)
            if norm < 1e-12:
                continue
            axis = (-dy / norm, dx / norm)
            pa = [x * axis[0] + y * axis[1] for x, y in a]
            pb = [x * axis[0] + y * axis[1] for x, y in b]
            depth = min(depth, min(max(pa), max(pb)) - max(min(pa), min(pb)))
    return depth


def _touches(a, b):
    return (a["bottom"] <= b["top"] + TOUCH_Z + EPS
            and b["bottom"] <= a["top"] + TOUCH_Z + EPS
            and _projected_overlap(a["hull"], b["hull"]) >= TOUCH_OVERLAP - EPS)


class Trial:
    """Trial-long reference latch and a 20-control-step stillness history.

    ``observe`` must run once per control step, even when score is not
    requested. The reference pose must be recorded after the intended reset
    settling period; the actual embodiment-hook timing remains unverified.
    """

    def __init__(self, table_top):
        self.floor = float(table_top) + LIGHTBOX_H
        self.reset(None)

    def reset(self, reference):
        self.violated = False
        self.initial = None
        self.recent = {name: deque(maxlen=STILL_STEPS) for name in FILTER_NAMES}
        if reference is not None:
            try:
                self.initial = _pose(reference)
            except (ValueError, TypeError, AttributeError):
                self.violated = True

    def observe(self, objects):
        if self.initial is None:
            self.violated = True
            return
        try:
            p, q = _pose(objects[REFERENCE])
            p0, q0 = self.initial
            if _distance(p, p0) > REF_POSITION + EPS or _angle(q, q0) > REF_ROTATION + EPS:
                self.violated = True
            for name in FILTER_NAMES:
                self.recent[name].append(_pose(objects[name]))
        except (KeyError, ValueError, TypeError, AttributeError):
            self.violated = True

    def _still(self, name, pose):
        samples = self.recent[name]
        return (len(samples) == STILL_STEPS
                and all(_distance(p, pose[0]) <= STILL_POSITION + EPS
                        and _angle(q, pose[1]) <= STILL_ROTATION + EPS for p, q in samples))

    def score(self, objects):
        if self.violated or self.initial is None:
            return 0.0
        try:
            # Also fail closed if called with a changed reference before observe().
            p, q = _pose(objects[REFERENCE])
            if _distance(p, self.initial[0]) > REF_POSITION + EPS or _angle(q, self.initial[1]) > REF_ROTATION + EPS:
                self.violated = True
                return 0.0
            geometry = {name: _geometry(objects[name]) for name in FILTER_NAMES}
        except (KeyError, ValueError, TypeError, AttributeError):
            self.violated = True
            return 0.0

        # Construct one supported chain from the lightbox upward. A candidate
        # must be flat, within the outline's centre window, and still for 1 s.
        candidates = []
        for name, g in geometry.items():
            x, y, _ = g["pos"]
            if (g["flat"] and abs(x - STACK_XY[0]) <= CENTRE_TOL + EPS
                    and abs(y - STACK_XY[1]) <= CENTRE_TOL + EPS
                    and self._still(name, (g["pos"], g["quat"]))):
                candidates.append(name)
        candidates.sort(key=lambda name: geometry[name]["bottom"])
        stack = []
        support_top = self.floor
        for name in candidates:
            g = geometry[name]
            gap = g["bottom"] - support_top
            if not -SUPPORT_OVERLAP - EPS <= gap <= SUPPORT_GAP + EPS:
                continue
            stack.append(name)
            support_top = g["top"]

        found = REQUIRED.intersection(stack)
        if not found:
            return 0.0

        # Include crooked, unstable or offset extras in contact with the stack,
        # not merely well-placed ones. Unused, separate plaques are unrestricted.
        connected = set(stack)
        while True:
            attached = {name for name in FILTER_NAMES if name not in connected
                        and any(_touches(geometry[name], geometry[other]) for other in connected)}
            if not attached:
                break
            connected.update(attached)
        return round(0.10 + (0.45 if REQUIRED.issubset(stack) else 0.0)
                     + (0.45 if not (connected - REQUIRED) else 0.0), 2)
