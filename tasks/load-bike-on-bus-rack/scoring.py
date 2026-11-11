"""Scoring for loading a bike on the bus's front bike rack. Pure NumPy: no simulator needed, so test_scoring.py runs
anywhere.

`Trial.step(objects)` is called once per control step and returns the score so far. `objects` holds:
- "bike": its body origin's pose (on the ground midway between the axles, world frame) and velocities;
- "rack": the rack frame's pose, with `sliders` = {"outer": hinge angle, "inner": hinge angle} in radians
  (0 stowed, negative swung out);
- "left_tcp", "right_tcp": the grippers' tool centre points.
Heights in world z. Faults are latched for the trial: nothing a robot does afterwards clears them. Only `Trial()` (a
new trial, at reset) does."""

import math
from dataclasses import dataclass

import numpy as np

import bike as B
import layout as L
import rack as K

CONTROL_HZ = 20.0
FELL = math.radians(25)  # the bike leaning further than this has fallen over (latched)
UPRIGHT = math.radians(12)  # secured: within this of upright
RACK_SHIFT, RACK_TURN = 0.01, math.radians(3)  # the rack moved: shoved (latched)
IN_TRAY_X = 0.024  # a wheel's centre this close to a tray's centre line (across it)...
IN_TRAY_Z = 0.010  # ...no more than this above resting on the table...
IN_TRAY_Y = 0.005  # ...and within the tray's length (wheel inside the end stops, with this margin)
LIP_SLACK = 0.012  # hook point within this of the tyre's mid-plane (lips straddle the tyre)
ON_TYRE = (-0.004, 0.008)  # hook point's distance from the wheel centre, minus R: in direct contact with the tyre
NEAR_FORK = 0.040  # hook point at most this far along the bike from the front axle (the fork): 45 deg of arc
HANDS_OFF = 0.04
LIN, ANG, STILL_S = 0.01, 0.2, 1.0
POSITIONS = ("outer", "inner")


def rot(q):
    x, y, z, w = q
    return np.array(((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
                     (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
                     (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y))))


def bike_frame(objects):
    s = objects["bike"]
    return rot(s.quat_xyzw), np.asarray(s.pos, float)


def rack_frame(objects):
    s = objects["rack"]
    return rot(s.quat_xyzw), np.asarray(s.pos, float)


def table_to_world(p):
    return np.asarray(p, float) + np.array((0.0, 0.0, L.TOP))


def lean(objects):
    """How far the bike leans over (rad): its across-the-bike axis out of horizontal."""
    R, _ = bike_frame(objects)
    return math.asin(min(1.0, abs(float(R[2, 0]))))


def wheel(objects, which):
    """A wheel's centre in the world."""
    R, p = bike_frame(objects)
    return p + R @ np.array(B.WHEELS[which])


def in_tray(objects, position, centre):
    """A wheel centre resting in a position's tray (judged in the rack's frame)."""
    R, p = rack_frame(objects)
    v = R.T @ (centre - p) + np.asarray(L.RACK_ORIGIN, float)  # back to table coordinates at the rack's start pose
    t = L.TRAY
    return (abs(v[0] - L.TRAYS[position]) <= IN_TRAY_X and v[2] <= B.R + IN_TRAY_Z
            and t["y0"] + B.R - IN_TRAY_Y <= v[1] <= t["y1"] - B.R + IN_TRAY_Y)


def position(objects):
    """The rack position both wheels rest in, or None."""
    f, r = wheel(objects, "front"), wheel(objects, "rear")
    for name in POSITIONS:
        if in_tray(objects, name, f) and in_tray(objects, name, r):
            return name
    return None


def faces_hook(objects, name):
    """The front wheel is at the end of the position where its hook is."""
    R, _ = rack_frame(objects)
    d = R.T @ (wheel(objects, "front") - wheel(objects, "rear"))
    return d[1] * L.HOOK_END[name] > 0


def hook_world(objects, name):
    """The support arm's hook point (pad bottom centre) in the world, from the rack pose and the hinge angle."""
    q = float(objects["rack"].sliders[name])
    p = K.hook(name, q)  # table coordinates at the rack's start pose
    R, o = rack_frame(objects)
    return o + R @ K.to_rack(p)


def hook_on_front_tyre(objects, name):
    """Where the hook sits relative to the front tyre: "" when it rests on the top of the tyre close to the fork,
    otherwise why not."""
    R, _ = bike_frame(objects)
    v = hook_world(objects, name) - wheel(objects, "front")
    across = float(R[:, 0] @ v)
    if abs(across) > LIP_SLACK:
        return "not over the front tyre"
    vin = v - across * R[:, 0]
    rho = float(np.linalg.norm(vin))
    if not (B.R + ON_TYRE[0] <= rho <= B.R + ON_TYRE[1]):
        return "not touching the front tyre" if rho > B.R else "below the tyre's surface"
    if float(R[:, 2] @ vin) <= 0.0:
        return "not on top of the front tyre"
    if abs(float(R[:, 1] @ vin)) > NEAR_FORK:
        return "too far from the fork"
    return ""


def _outline_gap(objects, point):
    R, p = bike_frame(objects)
    lo, hi = np.array(B.OUTLINE[0]), np.array(B.OUTLINE[1])
    v = R.T @ (np.asarray(point, float) - p)
    return float(np.linalg.norm(v - np.clip(v, lo, hi)))


def hands_off(objects, name):
    """Both tool centre points HANDS_OFF from the bike's outline and from the support arm's hook."""
    h = hook_world(objects, name)
    for a in ("left_tcp", "right_tcp"):
        t = np.asarray(objects[a].pos, float)
        if _outline_gap(objects, t) < HANDS_OFF or np.linalg.norm(t - h) < HANDS_OFF:
            return False
    return True


def still(objects):
    s = objects["bike"]
    return bool(np.linalg.norm(s.lin_vel) < LIN and np.linalg.norm(s.ang_vel) < ANG)


def rack_moved(objects):
    R, p = rack_frame(objects)
    start = table_to_world(L.RACK_ORIGIN)
    turn = math.acos(float(np.clip((np.trace(R) - 1) / 2, -1.0, 1.0)))
    return float(np.linalg.norm(p - start)) > RACK_SHIFT or turn > RACK_TURN


def secured(objects, name):
    """In position `name`, upright, front wheel toward its hook, and its hook on the front tyre near the fork."""
    return lean(objects) <= UPRIGHT and faces_hook(objects, name) and hook_on_front_tyre(objects, name) == ""


@dataclass
class Trial:
    fault: str = ""
    still_s: float = 0.0

    def check(self, objects):
        if lean(objects) > FELL:
            return "bike fell over (leaned past 25 degrees)"
        if rack_moved(objects):
            return "rack moved"
        return ""

    def step(self, objects):
        if not self.fault:
            self.fault = self.check(objects)
        if self.fault:
            self.still_s = 0.0
            return 0.0
        pos = position(objects)
        done = pos is not None and secured(objects, pos)
        let_go = done and hands_off(objects, pos) and still(objects)
        self.still_s = self.still_s + 1.0 / CONTROL_HZ if (pos == "outer" and let_go) else 0.0
        if pos is None:
            return 0.0
        if pos == "outer":
            if let_go and self.still_s >= STILL_S:
                return 1.0
            return 0.6 if done else 0.25
        return 0.5 if let_go else 0.25
