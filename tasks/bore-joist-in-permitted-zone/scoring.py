"""Where the drill guide stands on the joist, and what that is worth. Pure Python (numpy only), so it can be tested
without the simulator; task.py builds the scene from the same numbers.

Frame: metres from the midpoint between the arm bases, x forward, y toward the left arm, z up; TOP is the table top.

The rule scored is the building-control guidance for holes in simply supported solid timber floor and ceiling joists
(LABC Hertfordshire Technical Guidance Note 22, "Notching of joists", May 2011; the same zones as NHBC Standards
6.4.18): holes only on the centre line, between 0.25 and 0.4 of the clear span from either support, no more than 0.25
of the joist's depth across, and adjacent holes at least three diameters apart.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# -- the joist (a full-size 47 x 97 mm C16 ceiling joist, lying flat on the bench for pre-boring) ----------------------
JOIST_X = 0.36  # its centre line (x)
JOIST_DEPTH = 0.097  # the joist's depth, here across the table (x)
JOIST_THICK = 0.047  # its thickness, here vertical; the hole is bored straight down through it
SPAN = 0.80  # clear span between the joist hangers = its length (y), centred on y = 0
END_Y = SPAN / 2  # the two supports are its ends, at y = -0.40 (right) and +0.40 (left)

HOLE_D = 0.022  # the hole to be bored (for a 15 mm pipe); 0.25 x 97 = 24.25 mm is the largest allowed
EXISTING_HOLE = (JOIST_X, 0.14)  # the electrician's 22 mm hole: on the centre line, 260 mm from the left support

ZONE = (0.25 * SPAN, 0.40 * SPAN)  # 200 to 320 mm from the nearer support
MIN_SPACING = 3 * HOLE_D  # 66 mm centre to centre
FULL_OFF_CL = 0.010  # "on the centre line": bore within 10 mm of it (the 22 mm hole then stays 26.5 mm clear of
# both edges)

# -- the drill guide (a hardwood block with a steel bush through its centre; its origin is the block's centre) --------
GUIDE_SIZE = (0.060, 0.060, 0.060)
GUIDE_HALF_H = GUIDE_SIZE[2] / 2

MAX_TILT_DEG = 5.0
SEATED_M = 0.003  # the guide's base within 3 mm of the joist's top face
STILL_LIN = 0.01  # m/s
STILL_ANG = 0.5  # rad/s
SETTLE_STEPS = 20  # let go and left alone: the bore moves at most SETTLE_M over the last 20 control steps (1 s)
SETTLE_M = 0.002
CLEAR_M = 0.05  # let go: no point of either gripper, wrist mount to pad tips, within 5 cm of the guide
GRIPPER_BACK = 0.134  # the gripper reaches this far behind the TCP along its approach (wrist mount)
PAD_TIP = 0.013  # and this far past it (the pad tips)


def joist_face_z(top: float) -> float:
    return top + JOIST_THICK


def rotation(quat_xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def tilt_deg(quat_xyzw) -> float:
    """The angle between the guide's bore (its z axis) and vertical."""
    return math.degrees(math.acos(max(-1.0, min(1.0, rotation(quat_xyzw)[2, 2]))))


def bore_point(pos, quat_xyzw) -> np.ndarray:
    """Where the bore's axis leaves the guide's base: the centre of the hole on the joist face."""
    return np.asarray(pos, float) + rotation(quat_xyzw) @ np.array([0.0, 0.0, -GUIDE_HALF_H])


@dataclass(frozen=True)
class Reading:
    """What a tape measure and a square read off the final state, in metres."""

    standing: bool  # upright and seated on the joist's top face
    from_support: float  # along the joist, from the bore to the nearer end
    off_centre_line: float  # across the joist, from the bore to its centre line
    from_existing_hole: float  # centre to centre
    still: bool

    @property
    def in_zone(self) -> bool:
        return ZONE[0] <= self.from_support <= ZONE[1]

    @property
    def spaced(self) -> bool:
        return self.from_existing_hole >= MIN_SPACING


def read(pos, quat_xyzw, lin_vel, ang_vel, top: float) -> Reading:
    b = bore_point(pos, quat_xyzw)
    on_face = abs(b[0] - JOIST_X) <= JOIST_DEPTH / 2 and abs(b[1]) <= END_Y
    seated = on_face and abs(b[2] - joist_face_z(top)) <= SEATED_M
    return Reading(
        standing=seated and tilt_deg(quat_xyzw) <= MAX_TILT_DEG,
        from_support=END_Y - abs(float(b[1])),
        off_centre_line=abs(float(b[0]) - JOIST_X),
        from_existing_hole=math.hypot(b[0] - EXISTING_HOLE[0], b[1] - EXISTING_HOLE[1]),
        still=float(np.linalg.norm(lin_vel)) < STILL_LIN and float(np.linalg.norm(ang_vel)) < STILL_ANG,
    )


def points(r: Reading) -> float:
    """1.0: standing on the joist, bore 200-320 mm from a support, within 10 mm of the centre line and at least 66 mm
    from the existing hole. Everything else 0: mid-span, the chalk cross, nearer a support than 200 mm, closer than
    66 mm to the existing hole, more than 10 mm off the centre line, tilted, on the table or not placed. There is no
    partial credit: every other spot breaks the guidance (see README)."""
    ok = r.standing and r.in_zone and r.spaced and r.off_centre_line <= FULL_OFF_CL
    return 1.0 if ok else 0.0


def _box_distance(point, pos, quat_xyzw) -> float:
    """Distance from a point to the guide block (0 inside)."""
    local = rotation(quat_xyzw).T @ (np.asarray(point, float) - np.asarray(pos, float))
    excess = np.maximum(np.abs(local) - np.array(GUIDE_SIZE) / 2, 0.0)
    return float(np.linalg.norm(excess))


def gripper_clear(tcp, approach, pos, quat_xyzw, samples: int = 40) -> bool:
    """No point on the gripper's centre line, from its wrist mount to its pad tips, within CLEAR_M of the guide."""
    tcp, approach = np.asarray(tcp, float), np.asarray(approach, float)
    return all(_box_distance(tcp + s * approach, pos, quat_xyzw) >= CLEAR_M
               for s in np.linspace(-GRIPPER_BACK, PAD_TIP, samples))


def settled(bore_history) -> bool:
    """The last SETTLE_STEPS + 1 bore points (oldest first) all lie within SETTLE_M of the latest."""
    if len(bore_history) < SETTLE_STEPS + 1:
        return False
    recent = np.asarray(list(bore_history)[-(SETTLE_STEPS + 1):], float)
    return float(np.max(np.linalg.norm(recent - recent[-1], axis=1))) <= SETTLE_M


def final(r: Reading, grippers_clear: bool | None, bore_history) -> float:
    """The trial's score: the spot's points, only once the guide is let go (both grippers clear; None, if the arms
    can't be read, counts as not let go) and has stayed put for the last second."""
    return points(r) if grippers_clear and settled(bore_history) else 0.0
