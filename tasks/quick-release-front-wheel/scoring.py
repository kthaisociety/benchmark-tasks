"""Scoring for the quick-release front wheel. Pure NumPy: no simulator, so test_scoring.py runs anywhere.

`Trial.step(objects)` is called once per control step and returns the score so far. It reads one state, the wheel's:
`pos`/`quat_xyzw` (the axle's middle and its orientation, world frame, m), `lin_vel`/`ang_vel`, `joints` (rad:
"cam" is minus the lever's closure, "aim" the skewer's turn) and `links["cam"]` (the lever's world pose). The fork is
fixed, so its dropouts are constants from layout.py. The one fault is latched for the trial: nothing a robot does
afterwards clears it. Only `Trial()` (a new trial, at reset) does.
"""

import math
from dataclasses import dataclass, field

import numpy as np

import layout as L
from mechanism import CLOSED

SEAT_Z = 0.003  # axle centre at most this far above its fully seated height at a dropout: seated
SEAT_Y = 0.006  # and this close to the slot's centre line (the slot allows 4 mm)
MOUTH_Z = 0.008  # in a dropout's mouth: axle centre at most this far above the mouth's top, within the mouth's width
FORWARD = 30.0  # a closed lever pointing more than this (deg) from the blade toward the bike's front
CLAMP_MOVE = 0.005  # the axle moving this far while the lever stays closed: the wheel shifted in a closed QR
LIN, ANG = 0.01, 0.2  # at rest: m/s, rad/s


def rot(q):
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array(((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
                     (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
                     (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y))))


def closure(wheel):
    """The lever's closure, deg: 0 fully open, 90 closed."""
    return -math.degrees(float(wheel.joints["cam"]))


def lever_direction(wheel):
    """The lever's direction from its pivot to its paddle, world frame (unit vector)."""
    return rot(wheel.links["cam"][1]) @ np.array((-1.0, 0.0, 0.0))


def lever_aim(wheel):
    """Where the lever points in the fork's plane, deg from straight down the blade (toward the crown): positive
    toward the bike's front (+y), negative toward its rear."""
    d = lever_direction(wheel)
    return math.degrees(math.atan2(d[1], -d[2]))


def at_dropout(wheel, side):
    """The axle's centre line where it crosses dropout `side` (+1 far, -1 near), relative to that dropout's slot
    centre line (y) and the table top (z); None if the axle doesn't reach through that dropout."""
    R, p = rot(wheel.quat_xyzw), np.asarray(wheel.pos, float)
    axis = R[:, 0]
    if abs(axis[0]) < 0.5:
        return None
    plate = L.FORK_XY[0] + side * L.PLATE_X
    t = (plate - p[0]) / axis[0]
    if abs(t) > L.AXLE_HALF - 0.002:
        return None
    q = p + t * axis
    return q[1] - L.FORK_XY[1], q[2] - L.TOP


def seated(wheel, side):
    a = at_dropout(wheel, side)
    return a is not None and abs(a[0]) <= SEAT_Y and a[1] <= L.H_SEAT + SEAT_Z


def in_mouth(wheel, side):
    a = at_dropout(wheel, side)
    return a is not None and abs(a[0]) <= L.MOUTH_HALF and a[1] <= L.MOUTH_TOP + L.AXLE_SIDE / 2 + MOUTH_Z


def still(wheel):
    return bool(np.linalg.norm(wheel.lin_vel) < LIN and np.linalg.norm(wheel.ang_vel) < ANG)


def grade(wheel):
    """(score, what) for the wheel as it is now, before faults."""
    closed = closure(wheel) >= CLOSED
    both = seated(wheel, 1) and seated(wheel, -1)
    if both:
        if not closed:
            return 0.5, "seated, lever not closed"
        if lever_aim(wheel) > FORWARD:
            return 0.75, "seated, lever closed pointing forward"
        return 1.0, "seated, lever closed along the fork"
    if closed:
        return 0.0, "lever closed on an unseated axle"
    if all(seated(wheel, s) or in_mouth(wheel, s) for s in (1, -1)):
        return 0.2, "in the dropouts, not fully seated"
    return 0.0, "not in the dropouts"


@dataclass
class Trial:
    fault: str = ""
    clamp: object = None  # the axle's position when the lever closed, while it stays closed
    last: float = 0.0
    what: str = ""
    history: list = field(default_factory=list)

    def step(self, objects):
        w = objects["wheel"]
        c = closure(w)
        p = np.asarray(w.pos, float)
        if c >= CLOSED:
            if self.clamp is None:
                self.clamp = p.copy()
            elif not self.fault and np.linalg.norm(p - self.clamp) > CLAMP_MOVE:
                self.fault = "the wheel moved while the lever was closed"
        elif c < L.CLOSE_DETENT:  # reopened past the over-centre point
            self.clamp = None
        if self.fault:
            self.last, self.what = 0.0, self.fault
        else:
            self.last, self.what = grade(w)
        return self.last

    def success(self, objects):
        return self.last >= 1.0 and still(objects["wheel"])
