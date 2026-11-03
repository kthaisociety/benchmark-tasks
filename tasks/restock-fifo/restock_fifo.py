"""On the station table: a fixed shelf board with a lane 8 cm wide between two dividers 5 cm tall, running away
from the robot; its front is the end nearest the robot. A red can (older stock, 5.5 cm wide, 10 cm tall) stands at
the back of the lane. A blue can (new stock, the same size) stands on the table to the left of the shelf.
"""

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SHELF_TOP = TOP + 0.02
LANE_Y, LANE_WIDTH = 0.15, 0.08
LANE_X = (0.21, 0.41)
MID_X = sum(LANE_X) / 2
RADIUS, HEIGHT = 0.0275, 0.10
WOOD = (0.55, 0.40, 0.25)

SHELF = Box("shelf", pos=(0.31, LANE_Y, TOP + 0.01), size=(0.22, 0.14, 0.02), color=WOOD)
DIVIDERS = [
    Box(f"divider_{side}", pos=(MID_X, LANE_Y + s * (LANE_WIDTH / 2 + 0.005), SHELF_TOP + 0.025),
        size=(0.20, 0.01, 0.05), color=WOOD)
    for side, s in (("left", 1), ("right", -1))
]
RED = Cylinder("red_can", pos=(0.35, LANE_Y, SHELF_TOP + HEIGHT / 2 + 0.001), radius=RADIUS, height=HEIGHT,
               mass=0.25, color=(0.80, 0.12, 0.12))
BLUE = Cylinder("blue_can", pos=(0.20, 0.28, TOP + HEIGHT / 2 + 0.001), radius=RADIUS, height=HEIGHT,
                mass=0.25, color=(0.15, 0.30, 0.85))
OBJECTS = [SHELF, *DIVIDERS, RED, BLUE]

START_XY = {c.name: np.array(c.pos[:2]) for c in (RED, BLUE)}
_fell = False


def _tilt_deg(obj):
    x, y, _, _ = obj.quat_xyzw
    return float(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def _in_lane(obj):
    x, y = obj.pos[:2]
    inside = abs(y - LANE_Y) < LANE_WIDTH / 2 and LANE_X[0] < x < LANE_X[1]
    return bool(inside and obj.low[2] < SHELF_TOP + 0.01 and obj.resting() and _tilt_deg(obj) < 15)


def _track(objects):
    """Remember, for the whole trial, whether a can fell."""
    global _fell
    if all(np.linalg.norm(objects[n].pos[:2] - xy) < 0.005 and _tilt_deg(objects[n]) < 2 for n, xy in START_XY.items()):
        _fell = False
    _fell = _fell or any(_tilt_deg(objects[n]) > 45 for n in START_XY)


def score(objects):
    """0 if a can ever tilted past 45 deg. Red can upright in the front half of the lane and blue can upright in the
    back half: 1.0. Red can in the front half, blue can not in the lane: 0.5. Anything else, including the blue can
    in front of the red one: 0."""
    _track(objects)
    if _fell:
        return 0.0
    red, blue = objects["red_can"], objects["blue_can"]
    red_front = _in_lane(red) and red.pos[0] < MID_X
    if red_front and _in_lane(blue) and blue.pos[0] > MID_X:
        return 1.0
    return 0.5 if red_front and not _in_lane(blue) else 0.0


def success(objects):
    return score(objects) == 1.0


def demo():
    """Left arm, fingers closing front to back: slides the red can forward along the lane, then sets the blue can
    down behind it."""
    y = LANE_Y
    travel = TOP + 0.17
    grip_red = SHELF_TOP + HEIGHT - 0.025
    grip_blue = TOP + HEIGHT - 0.025
    bx, by = BLUE.pos[:2]
    front_x, back_x = 0.26, RED.pos[0]
    steps = [
        (1.5, (back_x, y, travel), FINGER_OPEN, "above the red can"),
        (1.5, (back_x, y, grip_red), FINGER_OPEN, "descend"),
        (0.8, (back_x, y, grip_red), FINGER_CLOSED, "close"),
        (0.6, (back_x, y, grip_red + 0.01), FINGER_CLOSED, "lift"),
        (1.5, (front_x, y, grip_red + 0.01), FINGER_CLOSED, "slide to the front"),
        (0.6, (front_x, y, grip_red + 0.003), FINGER_CLOSED, "set down"),
        (0.6, (front_x, y, grip_red + 0.003), FINGER_OPEN, "release"),
        (1.0, (front_x, y, travel), FINGER_OPEN, "up"),
        (1.5, (bx, by, travel), FINGER_OPEN, "above the blue can"),
        (1.5, (bx, by, grip_blue), FINGER_OPEN, "descend"),
        (0.8, (bx, by, grip_blue), FINGER_CLOSED, "close"),
        (1.2, (bx, by, travel), FINGER_CLOSED, "lift"),
        (1.8, (back_x, y, travel), FINGER_CLOSED, "carry"),
        (1.2, (back_x, y, grip_red + 0.005), FINGER_CLOSED, "lower"),
        (0.6, (back_x, y, grip_red + 0.005), FINGER_OPEN, "release"),
        (1.0, (back_x, y, travel), FINGER_OPEN, "retreat"),
    ]
    return {"left": [Waypoint(d, p, f, label, -np.pi / 2) for d, p, f, label in steps]}
