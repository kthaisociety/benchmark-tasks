"""On the station table: a tall cream windbreak screen (fixed) stands upright along x, a little left of centre. The
station's lamp hangs high on the robot's right and acts as the sun: the screen casts a long, slanted shadow onto the
table on its left (y positive). A blue cool box (a 6 cm cube) starts in full sun at the back left. A flat black
solar charger starts lying in the screen's shadow. The cool box must end up entirely in the shade, lid included
(direct sun warms vaccines), and the charger entirely in the sun (it charges only in light). Everything sits within
the left arm's reach; the right arm is not needed.
"""

import math
import os

import numpy as np
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# The station's own shadow-casting sphere lamp (yam_arena.station.LAMP_POS), world frame, m. Task files can't add
# lights; this one is fixed, high on the robot's right, so shadows fall to the robot's left (+y) and slightly away.
LAMP = np.array([0.04, -0.80, 2.6875])

# -- geometry (m) ----------------------------------------------------------------------------------------------
SCREEN_X, SCREEN_Y, SCREEN_L, SCREEN_T, SCREEN_H = 0.40, 0.0, 0.30, 0.02, 0.40
BOX_SIZE = (0.06, 0.06, 0.06)
BOX_START = (0.50, 0.30)
PANEL_SIZE = (0.12, 0.045, 0.02)   # flat folding solar charger, long side along x
PANEL_START = (0.36, 0.12)

SCREEN = Box("screen", pos=(SCREEN_X, SCREEN_Y, TOP + SCREEN_H / 2), size=(SCREEN_L, SCREEN_T, SCREEN_H),
             color=(0.88, 0.86, 0.80))
COOLBOX = Box("coolbox", pos=(*BOX_START, TOP + BOX_SIZE[2] / 2 + 0.001), size=BOX_SIZE, mass=0.3,
              color=(0.15, 0.40, 0.85))
CHARGER = Box("charger", pos=(*PANEL_START, TOP + PANEL_SIZE[2] / 2 + 0.001), size=PANEL_SIZE, mass=0.15,
              color=(0.10, 0.11, 0.16))
OBJECTS = [SCREEN, COOLBOX, CHARGER]

# -- scoring ---------------------------------------------------------------------------------------------------
ON_TABLE = 0.010     # m, an object's lowest point within this of the table top = resting on the table
MAX_TILT = math.radians(20)


def shade_polygon(h):
    """The screen's shadow on the horizontal plane TOP + h: the quadrilateral between the screen's shaded face and
    its top edge projected from the lamp onto that plane (counter-clockwise, top view)."""
    y0 = SCREEN_Y + SCREEN_T / 2
    far = []
    for x in (SCREEN_X - SCREEN_L / 2, SCREEN_X + SCREEN_L / 2):
        p = np.array([x, y0, TOP + SCREEN_H])
        t = (LAMP[2] - (TOP + h)) / (LAMP[2] - p[2])
        far.append((LAMP + (p - LAMP) * t)[:2])
    return [np.array([SCREEN_X - SCREEN_L / 2, y0]), np.array([SCREEN_X + SCREEN_L / 2, y0]), far[1], far[0]]


def _inside(poly, q):
    s = [(b[0] - a[0]) * (q[1] - a[1]) - (b[1] - a[1]) * (q[0] - a[0])
         for a, b in zip(poly, poly[1:] + poly[:1])]
    return all(v >= 0 for v in s) or all(v <= 0 for v in s)


def shaded_corners(obj):
    """How many of the object's four top corners (top view of its bounding box, at its top height) are in shade.
    The top is the highest, so the least shaded, part of an upright box: all four shaded = the whole box is."""
    lo, hi = obj.low, obj.high
    poly = shade_polygon(float(hi[2]) - TOP)
    return sum(_inside(poly, (x, y)) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]))


def _settled_flat(obj):
    x, y, z, w = (float(v) for v in obj.quat_xyzw)
    up_z = 1.0 - 2.0 * (x * x + y * y)   # world z of the object's own z axis
    return float(obj.low[2]) - TOP < ON_TABLE and up_z > math.cos(MAX_TILT)


def score(objects):
    """0.5 for the cool box resting upright on the table with all four top corners in the screen's shadow, plus
    0.5 for the charger lying flat on the table with none of its top corners in the shadow."""
    box, charger = objects["coolbox"], objects["charger"]
    s = 0.0
    if _settled_flat(box) and shaded_corners(box) == 4:
        s += 0.5
    if _settled_flat(charger) and shaded_corners(charger) == 0:
        s += 0.5
    if os.environ.get("SHADE_DEBUG"):
        _N[0] += 1
        if _N[0] % 40 == 0:
            print(f"[dbg {_N[0]}] box {np.round(box.pos, 3)} shaded {shaded_corners(box)}/4 flat "
                  f"{_settled_flat(box)} | charger {np.round(charger.pos, 3)} shaded {shaded_corners(charger)}/4 "
                  f"flat {_settled_flat(charger)} | score {s}", flush=True)
    return s


_N = [0]

# -- scripted demo ---------------------------------------------------------------------------------------------
BOX_GOAL = (0.42, 0.10)          # all four top corners >= 5 cm inside the lid-height shade
CHARGER_GOAL = (0.36, 0.28)      # in full sun, clear of the shadow's far edge (y 0.21) by ~5 cm
BOX_SCREEN_END = (0.19, 0.10)    # shortcut: right beside the screen, but off its near end, in the sun
BOX_EDGE = (0.42, 0.20)          # shortcut: base in the ground shadow, lid in the sun


def _charger_moves(goal=CHARGER_GOAL):
    z = TOP + PANEL_SIZE[2] / 2 + 0.008
    return pick_place_waypoints((*PANEL_START, z), (*goal, z - 0.003), travel_z=TOP + 0.18, yaw=0.0, speed=1.3)


def _box_moves(goal=BOX_GOAL, release=True):
    z = TOP + BOX_SIZE[2] / 2 + 0.008
    steps = pick_place_waypoints((*BOX_START, z), (*goal, z - 0.004), travel_z=TOP + 0.20, yaw=math.pi / 2,
                                 speed=1.3)
    steps.insert(6, Waypoint(0.6, (*goal, z - 0.004), FINGER_CLOSED, "settle", math.pi / 2))
    if not release:   # keep holding the box 3 cm above its spot to the end
        up = (*goal, z + 0.03)
        steps = steps[:6] + [Waypoint(1.0, up, FINGER_CLOSED, "lift", math.pi / 2),
                             Waypoint(30.0, up, FINGER_CLOSED, "hold", math.pi / 2)]
    return steps


CLEAR = Waypoint(1.0, (0.15, 0.42, TOP + 0.25), FINGER_OPEN, "clear of the camera", 0.0)


def demo():
    """SHADE_DEMO selects the plan: correct (default), nothing, screen_end, edge, charger_only, box_only, hold."""
    mode = os.environ.get("SHADE_DEMO", "correct")
    plans = {
        "correct": _charger_moves() + _box_moves(),
        "nothing": [Waypoint(2.0, (0.15, 0.42, TOP + 0.25), FINGER_OPEN, "stay clear", 0.0)],
        "screen_end": _box_moves(BOX_SCREEN_END),
        "edge": _box_moves(BOX_EDGE),
        "charger_only": _charger_moves(),
        "box_only": _box_moves((0.48, 0.08)),   # beside the charger still lying in the shade
        "hold": _charger_moves() + _box_moves(release=False),
    }
    steps = plans[mode]
    if mode != "hold":
        steps = steps + [CLEAR]
    return {"left": steps}
