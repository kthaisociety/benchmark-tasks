"""On the station table: a pine plank (40 x 8 x 1.5 cm) rests on a fixed wooden block 10 cm tall and overhangs
it by 18 cm on the left. A glass bottle stands on the overhang. A steel block on the plank's right end keeps it
level; lifting the steel block while the bottle is on the overhang tips the plank. A plastic tray is fixed in front.
"""

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
X = 0.28
PLANK_TOP = TOP + 0.116

SUPPORT = Box("support", pos=(X, -0.02, TOP + 0.05), size=(0.08, 0.10, 0.10), color=(0.55, 0.40, 0.25))
PLANK = Box("plank", pos=(X, 0.01, TOP + 0.1085), size=(0.08, 0.40, 0.015), mass=0.24, color=(0.85, 0.70, 0.45))
BOTTLE = Cylinder(
    "bottle", pos=(X, 0.17, PLANK_TOP + 0.051), radius=0.025, height=0.10, mass=0.20, color=(0.20, 0.55, 0.35)
)
WEIGHT = Box(
    "weight", pos=(X, -0.095, PLANK_TOP + 0.016), size=(0.04, 0.04, 0.03), mass=0.38, color=(0.45, 0.47, 0.50)
)

TRAY_X, TRAY_Y = 0.13, 0.0
TRAY_INSIDE = (0.12, 0.16)
TRAY_FLOOR = TOP + 0.004


def _tray(wall=0.006, height=0.025, color=(0.20, 0.30, 0.60)):
    lx, ly = TRAY_INSIDE
    wz = TOP + height / 2
    return [
        Box("tray_floor", pos=(TRAY_X, TRAY_Y, TOP + 0.002), size=(lx + 2 * wall, ly + 2 * wall, 0.004), color=color),
        Box("tray_front", pos=(TRAY_X - (lx + wall) / 2, TRAY_Y, wz), size=(wall, ly + 2 * wall, height), color=color),
        Box("tray_back", pos=(TRAY_X + (lx + wall) / 2, TRAY_Y, wz), size=(wall, ly + 2 * wall, height), color=color),
        Box("tray_left", pos=(TRAY_X, TRAY_Y + (ly + wall) / 2, wz), size=(lx, wall, height), color=color),
        Box("tray_right", pos=(TRAY_X, TRAY_Y - (ly + wall) / 2, wz), size=(lx, wall, height), color=color),
    ]


OBJECTS = [SUPPORT, PLANK, BOTTLE, WEIGHT, *_tray()]

START_XY = {o.name: np.array(o.pos[:2]) for o in (PLANK, BOTTLE, WEIGHT)}
_unsafe = False


def _tilt_deg(obj):
    x, y, _, _ = obj.quat_xyzw
    return float(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def _moved(obj, name):
    return float(np.linalg.norm(obj.pos[:2] - START_XY[name]))


def _in_tray(obj):
    dx, dy = np.abs(obj.pos[:2] - (TRAY_X, TRAY_Y))
    inside = dx < TRAY_INSIDE[0] / 2 and dy < TRAY_INSIDE[1] / 2
    return bool(inside and obj.low[2] < TRAY_FLOOR + 0.01 and obj.resting())


def _track(objects):
    """Remember, for the whole trial, whether the plank tipped or the bottle fell."""
    global _unsafe
    if all(_moved(objects[n], n) < 0.01 for n in START_XY) and _tilt_deg(objects["plank"]) < 2:
        _unsafe = False
    _unsafe = _unsafe or _tilt_deg(objects["plank"]) > 10 or _tilt_deg(objects["bottle"]) > 45


def score(objects):
    """0 if the plank ever tipped past 10 deg, the bottle ever tilted past 45 deg, or the plank ends off its
    support or tilted past 5 deg. Otherwise 0.5 for the bottle upright in the tray, 0.5 for the steel block in it."""
    _track(objects)
    plank = objects["plank"]
    if _unsafe or _moved(plank, "plank") > 0.03 or _tilt_deg(plank) > 5:
        return 0.0
    bottle = objects["bottle"]
    return 0.5 * (_in_tray(bottle) and _tilt_deg(bottle) < 15) + 0.5 * _in_tray(objects["weight"])


def success(objects):
    return score(objects) == 1.0


def demo():
    """Left arm moves the bottle to the tray first; right arm waits above the steel block, then moves it."""
    travel = TOP + 0.20
    bottle_drop = (TRAY_X, TRAY_Y + 0.04, TRAY_FLOOR + BOTTLE.height / 2 + 0.005)
    weight_pick = (*WEIGHT.pos[:2], WEIGHT.pos[2] + 0.005)
    weight_drop = (TRAY_X, TRAY_Y - 0.04, TRAY_FLOOR + WEIGHT.size[2] / 2 + 0.01)
    above_weight = (*WEIGHT.pos[:2], travel)

    left = pick_place_waypoints(BOTTLE.pos, bottle_drop, travel_z=travel)
    left.append(Waypoint(1.5, (0.20, 0.25, travel), FINGER_OPEN, "clear the tray"))
    right = [
        Waypoint(1.5, above_weight, FINGER_OPEN, "above the weight"),
        Waypoint(10.0, above_weight, FINGER_OPEN, "wait for the bottle"),
        *pick_place_waypoints(weight_pick, weight_drop, travel_z=travel),
    ]
    return {"left": left, "right": right}
