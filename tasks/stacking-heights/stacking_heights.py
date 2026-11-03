"""On the station table: a fixed grey pad (6 x 6 cm) with a wooden post behind it carrying an orange band 11 cm
above the pad. In front, four wooden spacer blocks with a 4 x 4 cm footprint, 6, 4, 3 and 2 cm tall.
"""

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
PAD_X = 0.26
PAD_TOP = TOP + 0.003
TARGET = 0.11
WOOD = (0.80, 0.65, 0.40)

PAD = Box("pad", pos=(PAD_X, 0.0, TOP + 0.0015), size=(0.06, 0.06, 0.003), color=(0.35, 0.35, 0.38))
POST = Box("post", pos=(0.33, 0.0, TOP + 0.075), size=(0.02, 0.02, 0.15), color=(0.55, 0.40, 0.25))
BAND = Box("band", pos=(0.33, 0.0, PAD_TOP + TARGET), size=(0.024, 0.024, 0.005), color=(1.0, 0.50, 0.05))


def _spacer(height, y):
    return Box(f"spacer_{round(height * 100)}", pos=(0.16, y, TOP + height / 2), size=(0.04, 0.04, height),
               mass=0.0096 * height * 100, color=WOOD)


SPACERS = [_spacer(0.06, -0.12), _spacer(0.04, -0.04), _spacer(0.03, 0.04), _spacer(0.02, 0.12)]
S6, S4, S3, S2 = SPACERS
OBJECTS = [PAD, POST, BAND, *SPACERS]


def _tilt_deg(obj):
    x, y, _, _ = obj.quat_xyzw
    return float(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def stack_height(objects):
    """Height above the pad of the stack's top: spacers resting on the pad, each sitting on the one below."""
    on_pad = [
        o for o in (objects[s.name] for s in SPACERS)
        if abs(o.pos[0] - PAD_X) < 0.03 and abs(o.pos[1]) < 0.03 and o.resting() and _tilt_deg(o) < 10
    ]
    level = PAD_TOP
    for o in sorted(on_pad, key=lambda o: o.low[2]):
        if abs(o.low[2] - level) > 0.005:
            break
        level = o.high[2]
    return level - PAD_TOP


def score(objects):
    """1.0 if the stack's top is within 0.5 cm of the band (11 cm), 0.25 within 1.5 cm, otherwise 0."""
    off = abs(stack_height(objects) - TARGET)
    return 1.0 if off <= 0.005 else 0.25 if off <= 0.015 else 0.0


def success(objects):
    return score(objects) == 1.0


def demo():
    """Right arm sets the 6 cm spacer on the pad and moves away; left arm adds the 3 cm, then the 2 cm spacer."""
    travel = TOP + 0.16

    def grip(s):
        return (*s.pos[:2], TOP + max(s.size[2] / 2, 0.015))

    def onto(stack, s):
        return (PAD_X, 0.0, PAD_TOP + stack + max(s.size[2] / 2, 0.015) + 0.005)

    right = [
        *pick_place_waypoints(grip(S6), onto(0.0, S6), travel_z=travel),
        Waypoint(1.5, (0.16, -0.20, travel), FINGER_OPEN, "clear the pad"),
    ]
    left = [
        Waypoint(1.5, (0.16, 0.20, travel), FINGER_OPEN, "wait clear of the right arm"),
        Waypoint(9.0, (0.16, 0.20, travel), FINGER_OPEN, "wait"),
        *pick_place_waypoints(grip(S3), onto(0.06, S3), travel_z=travel),
        *pick_place_waypoints(grip(S2), onto(0.09, S2), travel_z=travel),
    ]
    return {"right": right, "left": left}
