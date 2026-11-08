"""Scoring for the kerb climb. Pure NumPy: no simulator needed, so test_scoring.py runs anywhere.

`Trial.step(objects)` is called once per control step with the scene's states (chair, dummy, left_tcp,
right_tcp; world frame, metres) and returns the score so far. Faults are latched for the trial: nothing a robot
does afterwards clears them. Only `Trial()` (a new trial, at reset) does."""

import math
from dataclasses import dataclass, field

import numpy as np

import chair as C
import layout as L

CONTROL_HZ = 20.0
ON_TOP = 0.01  # a wheel's centre this far past the kerb edge, and resting on the kerb top, is up
REST_GAP = 0.003  # resting: the wheel's lowest point within this of the surface below it
LIFT = 0.005  # every wheel this far above the surface below it at once: the chair was lifted
TIP_BACK = math.radians(25)
TIP_FORWARD = math.radians(15)
ROLL = math.radians(12)
DUMMY_TILT = math.radians(20)
LEVEL = math.radians(3)
HANDS_OFF = 0.05
LIN, ANG, STILL_S = 0.01, 0.2, 1.0


def rot(q):
    x, y, z, w = q
    return np.array(((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
                     (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
                     (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y))))


def frame(objects):
    s = objects["chair"]
    return rot(s.quat_xyzw), np.asarray(s.pos, float)


def pitch(objects):
    """Tip back about the chair's own y axis, radians: positive when the front is up."""
    R, _ = frame(objects)
    return math.atan2(R[2, 0], math.hypot(R[0, 0], R[1, 0]))


def roll(objects):
    R, _ = frame(objects)
    return math.asin(float(np.clip(R[2, 1], -1.0, 1.0)))


WHEELS = {"rear_l": (0.0, C.REAR["y"], C.REAR["r"]), "rear_r": (0.0, -C.REAR["y"], C.REAR["r"]),
          "caster_l": (C.CASTER["x"], C.CASTER["y"], C.CASTER["r"]), "caster_r": (C.CASTER["x"], -C.CASTER["y"], C.CASTER["r"])}


def wheels(objects):
    """Each wheel's centre in the world, and its radius."""
    R, p = frame(objects)
    return {n: (p + R @ np.array((x, y, r)), r) for n, (x, y, r) in WHEELS.items()}


def clearance(centre, r):
    """Gap between a wheel and the ground below it: the table, the kerb's top or the kerb's edge (in x and z)."""
    x, z = centre[0], centre[2]
    if x >= L.KERB_X0:
        d = z - L.KERB_TOP
    else:
        edge = math.hypot(L.KERB_X0 - x, z - L.KERB_TOP) if z > L.KERB_TOP else L.KERB_X0 - x
        d = min(z - L.TOP, edge)
    return d - r


def up(centre, r):
    """On the kerb: centre ON_TOP past its edge, resting on its top."""
    return centre[0] > L.KERB_X0 + ON_TOP and clearance(centre, r) < REST_GAP


def lifted(objects):
    return all(clearance(c, r) > LIFT for c, r in wheels(objects).values())


def dummy_seated(objects):
    """The dummy's centre over the seat, low on it, and upright on it."""
    R, p = frame(objects)
    d = objects["dummy"]
    v = R.T @ (np.asarray(d.pos, float) - p)
    Rd = R.T @ rot(d.quat_xyzw)
    tilt = math.acos(float(np.clip(Rd[2, 2], -1.0, 1.0)))
    return (C.SEAT["x0"] - 0.005 < v[0] < C.SEAT["x1"] and abs(v[1]) < C.SEAT["half_y"]
            and C.SEAT["top"] < v[2] < C.SEAT["top"] + L.DUMMY["h"] / 2 + 0.01 and tilt < DUMMY_TILT)


def hands_off(objects):
    """Both tool centre points at least HANDS_OFF from the chair's outline, judged in the chair's frame."""
    R, p = frame(objects)
    lo, hi = np.array(C.OUTLINE[0]), np.array(C.OUTLINE[1])
    for a in ("left_tcp", "right_tcp"):
        v = R.T @ (np.asarray(objects[a].pos, float) - p)
        if np.linalg.norm(v - np.clip(v, lo, hi)) < HANDS_OFF:
            return False
    return True


def still(objects):
    s = objects["chair"]
    return bool(np.linalg.norm(s.lin_vel) < LIN and np.linalg.norm(s.ang_vel) < ANG)


@dataclass
class Trial:
    fault: str = ""
    still_s: float = 0.0
    history: list = field(default_factory=list)

    def check(self, objects):
        if pitch(objects) > TIP_BACK:
            return "tipped back past 25 degrees"
        if pitch(objects) < -TIP_FORWARD:
            return "tipped forward past 15 degrees"
        if abs(roll(objects)) > ROLL:
            return "tipped sideways past 12 degrees"
        if lifted(objects):
            return "lifted clear of the ground"
        if not dummy_seated(objects):
            return "dummy off the seat"
        return ""

    def step(self, objects):
        if not self.fault:
            self.fault = self.check(objects)
        if self.fault:
            return 0.0
        w = wheels(objects)
        casters = up(*w["caster_l"]) and up(*w["caster_r"])
        rears = up(*w["rear_l"]) and up(*w["rear_r"])
        settled = casters and rears and abs(pitch(objects)) < LEVEL and still(objects) and hands_off(objects)
        self.still_s = self.still_s + 1.0 / CONTROL_HZ if settled else 0.0
        if settled and self.still_s >= STILL_S:
            return 1.0
        if casters and rears:
            return 0.6
        if casters:
            return 0.25
        return 0.0
