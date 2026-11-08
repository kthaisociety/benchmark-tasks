"""On the station table, a weather observing site: a brick building wall runs along the far side of a lawn plot, with a
compass rose on the grass showing which way is north (toward the arms' left). A copper rain gauge stands hard against
the wall, where the wall would shelter it, and a white instrument screen stands beside it with its dark door facing
south. A rain gauge must stand at least twice as far from an obstacle as the obstacle is tall, and a screen's door
must face north so the sun never falls on the thermometers when it is opened.
"""

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from partkit import part  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

LAWN, BRICK, WHITE, COPPER, DARK, SLAB, RED = (0.33, 0.55, 0.27), (0.72, 0.42, 0.32), (0.96, 0.96, 0.94), \
    (0.72, 0.45, 0.25), (0.10, 0.10, 0.12), (0.62, 0.62, 0.60), (0.80, 0.12, 0.10)

WALL_X, WALL_T, WALL_H = 0.55, 0.02, 0.06
FACE_X = WALL_X - WALL_T / 2
GAUGE_H = 0.07
CLEAR = 2 * WALL_H
NORTH = (0.0, 1.0)
GAUGE_XY, GAUGE_FROM = (0.30, 0.20), (0.46, 0.20)
SCREEN_XY, SCREEN_Y = (0.34, -0.20), -0.20

SITE = [
    Box("lawn", pos=(0.33, 0.0, TOP + 0.0003), size=(0.46, 0.84, 0.0006), color=LAWN),
    Box("building_wall", pos=(WALL_X, 0.0, TOP + WALL_H / 2), size=(WALL_T, 0.84, WALL_H), color=BRICK),
    *[Box(f"path_slab_{i}", pos=(0.20, -0.30 + i * 0.12, TOP + 0.0009), size=(0.10, 0.11, 0.0006), color=SLAB)
      for i in range(6)],
    Box("compass_arrow", pos=(0.50, 0.0, TOP + 0.0009), size=(0.012, 0.14, 0.0006), color=RED),
    Box("compass_cross", pos=(0.50, 0.0, TOP + 0.0009), size=(0.07, 0.012, 0.0006), color=RED),
    Box("compass_head_left", pos=(0.50 + 0.012, 0.075, TOP + 0.0009), size=(0.012, 0.03, 0.0006), color=RED),
    Box("compass_n_stroke_a", pos=(0.50, 0.095, TOP + 0.0009), size=(0.025, 0.004, 0.0006), color=DARK),
    Box("compass_n_stroke_b", pos=(0.50 - 0.011, 0.11, TOP + 0.0009), size=(0.004, 0.022, 0.0006), color=DARK),
    Box("compass_n_stroke_c", pos=(0.50 + 0.011, 0.11, TOP + 0.0009), size=(0.004, 0.022, 0.0006), color=DARK),
]

GAUGE = part("rain_gauge", pos=(*GAUGE_FROM, TOP + 0.001), mass=0.15, color=COPPER, friction=1.0,
             boxes=[((0, 0, 0.004), (0.07, 0.07, 0.008), DARK)], cylinders=[((0, 0, 0.008 + GAUGE_H / 2), 0.024, GAUGE_H, COPPER)])
BODY, BAR_Z, BAR = (0.09, 0.08, 0.06), 0.09, (0.016, 0.10, 0.016)
DOOR = (0.004, 0.07, 0.054)
SCREEN = part("instrument_screen", pos=(*SCREEN_XY, TOP + 0.001), mass=0.3, color=WHITE, friction=1.0, yaw=-math.pi / 2,
              boxes=[
    ((0, 0, BODY[2] / 2), BODY, WHITE),
    *[((BODY[0] / 2 + 0.002, 0, 0.008 + k * 0.011), (0.004, 0.07, 0.004), WHITE) for k in range(4)],
    ((BODY[0] / 2 + 0.002, 0, BODY[2] / 2), DOOR, DARK),
    *[((0, s * 0.04, BODY[2] + 0.015), (0.012, 0.012, 0.03), WHITE) for s in (-1, 1)],
    ((0, 0, BAR_Z), BAR, WHITE)])
DRESSING = [
    Cylinder("rain_bucket", pos=(0.20, 0.34, TOP + 0.03), radius=0.03, height=0.06, color=(0.30, 0.45, 0.70)),
    Box("clipboard", pos=(0.22, -0.34, TOP + 0.004), size=(0.07, 0.05, 0.008), color=(0.55, 0.40, 0.25)),
    Cylinder("anemometer_base", pos=(0.46, -0.30, TOP + 0.02), radius=0.022, height=0.04, color=(0.75, 0.75, 0.78)),
    Box("anemometer_arm", pos=(0.46, -0.30, TOP + 0.046), size=(0.07, 0.006, 0.006), color=(0.75, 0.75, 0.78)),
    Cylinder("thermometer_case", pos=(0.28, 0.34, TOP + 0.015), radius=0.012, height=0.03, color=(0.95, 0.95, 0.95)),
]

OBJECTS = [*SITE, *DRESSING, GAUGE, SCREEN]
DEBUG = os.environ.get("WX_DEBUG") == "1"
_K = {"k": 0}


def _tilt_deg(state):
    return float(np.degrees(np.arccos(np.clip(state.rotation()[2, 2], -1.0, 1.0))))


def _at_rest_upright(state):
    return _tilt_deg(state) < 10 and np.linalg.norm(state.lin_vel) < 0.02 and abs(state.pos[2] - TOP) < 0.015


def _gauge_sited(g):
    """At rest upright and at least twice the wall's height out from the wall (12 cm), measured to the gauge's axis
    less its 2.4 cm radius."""
    return bool(_at_rest_upright(g) and (FACE_X - g.pos[0]) - 0.024 >= CLEAR)


def _door_north(s):
    """At rest upright with its door (the part's local +x) pointing north (+y) within 45 degrees."""
    door = s.rotation()[:, 0]
    return bool(_at_rest_upright(s) and door[1] / max(np.linalg.norm(door[:2]), 1e-6) > math.cos(math.radians(45)))


def score(objects):
    """0.5 for the rain gauge standing at least twice the wall's height clear of it; 0.5 for the instrument screen
    standing upright with its door facing north."""
    total = 0.5 * _gauge_sited(objects["rain_gauge"]) + 0.5 * _door_north(objects["instrument_screen"])
    _K["k"] += 1
    if DEBUG and _K["k"] % 40 == 0:
        g, s = objects["rain_gauge"], objects["instrument_screen"]
        door = s.rotation()[:, 0]
        print(f"WX step {_K['k']} score {total} gauge {np.round(g.pos, 3)} clear {FACE_X - g.pos[0] - 0.024:.3f} "
              f"screen {np.round(s.pos, 3)} door ({door[0]:.2f},{door[1]:.2f}) tilt {_tilt_deg(s):.1f}", flush=True)
    return float(total)


TRAVEL = TOP + 0.22
MODE = os.environ.get("WX_MODE", "demo")


def _carry(to_xy):
    grip = TOP + 0.001 + 0.008 + GAUGE_H / 2 + 0.008
    return [(1.5, (*GAUGE_FROM, TRAVEL), FINGER_OPEN, "above the rain gauge", 0.0),
            (1.5, (*GAUGE_FROM, grip), FINGER_OPEN, "descend", 0.0), (0.6, (*GAUGE_FROM, grip), FINGER_OPEN, "settle", 0.0),
            (0.8, (*GAUGE_FROM, grip), FINGER_CLOSED, "grip", 0.0),
            (1.0, (*GAUGE_FROM, grip + 0.05), FINGER_CLOSED, "lift", 0.0),
            (2.0, (*to_xy, grip + 0.05), FINGER_CLOSED, "carry it out from the wall", 0.0),
            (1.0, (*to_xy, grip + 0.002), FINGER_CLOSED, "set down", 0.0),
            (0.6, (*to_xy, grip + 0.002), FINGER_OPEN, "let go", 0.0), (1.0, (*to_xy, TRAVEL), FINGER_OPEN, "up", 0.0)]


def _quarter(yaw0, yaw1):
    """Grip the handle (the bar runs along the gripper yaw 0 when it lies along x, yaw -90 degrees when along y)."""
    x, y = SCREEN_XY
    grip = TOP + 0.001 + BAR_Z + 0.008
    return [(1.5, (x, y, TRAVEL), FINGER_OPEN, "above the handle", yaw0),
            (1.5, (x, y, grip), FINGER_OPEN, "descend", yaw0), (0.6, (x, y, grip), FINGER_OPEN, "settle", yaw0),
            (0.8, (x, y, grip), FINGER_CLOSED, "grip the handle", yaw0),
            (1.0, (x, y, grip + 0.03), FINGER_CLOSED, "lift", yaw0),
            (2.0, (x, y, grip + 0.03), FINGER_CLOSED, "quarter turn", yaw1),
            (1.0, (x, y, grip + 0.002), FINGER_CLOSED, "set down", yaw1),
            (0.6, (x, y, grip + 0.002), FINGER_OPEN, "let go", yaw1), (1.0, (x, y, TRAVEL), FINGER_OPEN, "up", yaw1)]


def demo():
    """The left arm carries the rain gauge out from the wall to 14.6 cm clear of it; then the right arm turns the
    instrument screen round by its handle in two quarter turns, regripping between them, from facing south to north."""
    if MODE == "none":
        return {"left": [Waypoint(1.0, (0.12, 0.42, TRAVEL), FINGER_OPEN, "idle", 0.0)]}
    left = (_carry(GAUGE_XY) if MODE != "screen_only" else []) + [(1.5, (0.12, 0.42, TRAVEL), FINGER_OPEN, "move clear", 0.0)]
    wait = sum(s[0] for s in left)
    right = [(wait, (0.20, -0.42, TRAVEL), FINGER_OPEN, "wait", 0.0)]
    if MODE not in ("gauge_only",):
        turn = _quarter(0.0, math.pi / 2) + _quarter(-math.pi / 2, 0.0)
        if MODE == "wrong_way":
            turn = _quarter(0.0, math.pi / 2)
        right += turn
    right += [(1.5, (0.12, -0.42, TRAVEL), FINGER_OPEN, "move clear", 0.0),
              (2.0, (0.12, -0.42, TRAVEL), FINGER_OPEN, "let everything settle", 0.0)]
    return {"left": [Waypoint(*s) for s in left], "right": [Waypoint(*s) for s in right]}
