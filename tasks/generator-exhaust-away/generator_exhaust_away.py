"""On the station table, a power cut at a house: a portable generator has been left right against the house wall, its
exhaust facing the wall. Carbon monoxide from a generator kills, so it has to stand well clear of the house with its
exhaust pointing away. It has to be carried out from the wall and turned round.
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

RED, DARK, FRAME, BRICK = (0.75, 0.12, 0.10), (0.06, 0.06, 0.07), (0.20, 0.20, 0.22), (0.72, 0.42, 0.32)
LAWN, SLAB = (0.35, 0.55, 0.28), (0.62, 0.62, 0.60)

WALL_X, WALL_T, WALL_H = 0.53, 0.02, 0.14
FACE_X = WALL_X - WALL_T / 2
SAFE_DISTANCE = 0.28
Y, START, MID_X, END_X = 0.20, 0.40, 0.33, 0.235

HOUSE = [
    Box("lawn", pos=(0.31, 0.04, TOP + 0.0003), size=(0.42, 0.74, 0.0006), color=LAWN),
    Box("house_wall", pos=(WALL_X, 0.04, TOP + WALL_H / 2), size=(WALL_T, 0.74, WALL_H), color=BRICK),
    Cylinder("downpipe", pos=(FACE_X - 0.008, -0.14, TOP + 0.065), radius=0.008, height=0.13, color=(0.30, 0.30, 0.32)),
    Box("downpipe_clip", pos=(FACE_X - 0.003, -0.14, TOP + 0.10), size=(0.006, 0.022, 0.008), color=(0.20, 0.20, 0.22)),
    *[Box(f"patio_slab_{i}", pos=(0.46, 0.04 + i * 0.12 - 0.30, TOP + 0.0009), size=(0.10, 0.11, 0.0006), color=SLAB)
      for i in range(6)],
]

BODY = (0.09, 0.07, 0.05)
BAR_Z, BAR, POST_H = 0.08, (0.016, 0.10, 0.016), 0.03
PIPE = (0.036, 0.024, 0.024)
GENERATOR = part("generator", pos=(START, Y, TOP + 0.002), mass=0.4, color=RED, friction=1.0, boxes=[
    ((0, 0, BODY[2] / 2), BODY, RED),
    *[((0, s * 0.04, BODY[2] + POST_H / 2), (0.012, 0.012, POST_H), FRAME) for s in (-1, 1)],
    ((0, 0, BAR_Z), BAR, FRAME),
    ((BODY[0] / 2 + PIPE[0] / 2, 0, 0.028), PIPE, DARK)])

DRESSING = [
    Cylinder("flower_pot", pos=(0.20, -0.34, TOP + 0.03), radius=0.028, height=0.06, color=(0.75, 0.40, 0.25)),
    Cylinder("flower_pot_soil", pos=(0.20, -0.34, TOP + 0.0605), radius=0.024, height=0.001, color=(0.25, 0.17, 0.10)),
    Box("garden_table", pos=(0.60, -0.30, TOP + 0.015), size=(0.10, 0.08, 0.03), color=(0.55, 0.40, 0.25)),
    Cylinder("cable_reel", pos=(0.20, -0.20, TOP + 0.015), radius=0.03, height=0.03, color=(0.95, 0.75, 0.10)),
    Box("torch", pos=(0.33, -0.28, TOP + 0.012), size=(0.09, 0.024, 0.024), color=(0.20, 0.20, 0.22)),
]

OBJECTS = [*HOUSE, *DRESSING, GENERATOR]
DEBUG = os.environ.get("GEN_DEBUG") == "1"
_K = {"k": 0}


def _tilt_deg(state):
    return float(np.degrees(np.arccos(np.clip(state.rotation()[2, 2], -1.0, 1.0))))


def _yaw_deg(state):
    return math.degrees(math.atan2(state.rotation()[1, 0], state.rotation()[0, 0]))


def score(objects):
    """0.5 for the generator standing at rest at least 28 cm from the house wall; 0.5 more if, standing there, its
    exhaust points away from the house within 45 degrees."""
    g = objects["generator"]
    standing = _tilt_deg(g) < 10 and np.linalg.norm(g.lin_vel) < 0.02 and abs(g.pos[2] - TOP) < 0.012
    clear = standing and FACE_X - g.pos[0] >= SAFE_DISTANCE
    outlet = g.rotation()[:, 0]
    away = -outlet[0] / max(np.linalg.norm(outlet[:2]), 1e-6) > math.cos(math.radians(45))
    total = 0.5 * clear + 0.5 * (clear and away)
    _K["k"] += 1
    if DEBUG and _K["k"] % 40 == 0:
        print(f"GEN step {_K['k']} score {total} pos {np.round(g.pos, 3)} yaw {_yaw_deg(g):.1f} tilt {_tilt_deg(g):.1f} "
              f"dist {FACE_X - g.pos[0]:.3f}", flush=True)
    return float(total)


TRAVEL = TOP + 0.22
MODE = os.environ.get("GEN_MODE", "demo")


def _quarter(x, to_x, yaw0, yaw1):
    grip = TOP + 0.002 + BAR_Z + 0.008
    lift = grip + 0.03
    return [(1.5, (x, Y, TRAVEL), FINGER_OPEN, "above the handle", yaw0),
            (1.5, (x, Y, grip), FINGER_OPEN, "descend", yaw0), (0.6, (x, Y, grip), FINGER_OPEN, "settle", yaw0),
            (0.8, (x, Y, grip), FINGER_CLOSED, "grip the handle", yaw0),
            (1.0, (x, Y, lift), FINGER_CLOSED, "lift", yaw0),
            (2.5, (to_x, Y, lift), FINGER_CLOSED, "carry", yaw1),
            (1.0, (to_x, Y, grip + 0.002), FINGER_CLOSED, "set down", yaw1),
            (0.6, (to_x, Y, grip + 0.002), FINGER_OPEN, "let go", yaw1),
            (1.0, (to_x, Y, TRAVEL), FINGER_OPEN, "up", yaw1)]


def demo():
    """The left arm carries the generator away from the wall in two moves, turning it a quarter turn in each and
    regripping the handle between them, so it ends well clear with its exhaust pointing away from the house."""
    if MODE == "none":
        steps = [(1.0, (0.12, 0.42, TRAVEL), FINGER_OPEN, "idle", 0.0)]
    elif MODE == "moved":
        steps = _quarter(START, MID_X, -math.pi / 2, -math.pi / 2) + _quarter(MID_X, END_X, -math.pi / 2, -math.pi / 2)
    elif MODE == "spot":
        steps = _quarter(START, START, -math.pi / 2, 0.0) + _quarter(START, START, 0.0, math.pi / 2)
    else:
        steps = _quarter(START, MID_X, -math.pi / 2, 0.0) + _quarter(MID_X, END_X, 0.0, math.pi / 2)
    steps += [(1.5, (0.12, 0.42, TRAVEL), FINGER_OPEN, "move clear", 0.0),
              (2.0, (0.12, 0.42, TRAVEL), FINGER_OPEN, "let everything settle", 0.0)]
    return {"left": [Waypoint(*s) for s in steps]}
