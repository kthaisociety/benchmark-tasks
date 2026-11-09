"""On the station table, a harbour chart model: two channels run in from open sea to harbours at opposite ends, one of
them with a bend. Each marked gate has two mooring pads, one either side of the channel. Red and green lateral buoys
wait on a workboat. Under IALA Region A, a ship entering from sea keeps red marks to port and green marks to starboard,
so which pad takes which colour depends on the direction of travel along the channel at that gate.
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
FLOOR_T, PAD_T, DECK_T, LAND_T = 0.002, 0.001, 0.004, 0.006
BASE = TOP + FLOOR_T
PAD_TOP = BASE + PAD_T
DECK_TOP = BASE + DECK_T

SEA, CHANNEL, LAND, QUAY, PAD, DECK, DARK, RED, GREEN, WHITE = (0.07, 0.22, 0.42), (0.30, 0.56, 0.74), (0.52, 0.47, 0.38), \
    (0.72, 0.72, 0.70), (0.25, 0.25, 0.27), (0.60, 0.60, 0.62), (0.12, 0.12, 0.14), (0.82, 0.10, 0.10), (0.10, 0.62, 0.22), (0.92, 0.92, 0.90)

PADS = {"a_port": (0.38, 0.19), "a_starboard": (0.38, 0.33), "b_port": (0.335, -0.17), "b_starboard": (0.335, -0.31),
        "c_port": (0.33, -0.06), "c_starboard": (0.47, -0.06)}
REQUIRED = {"a_port": "red", "a_starboard": "green", "b_port": "red", "b_starboard": "green",
            "c_port": "red", "c_starboard": "green"}
STARTS = {"green_0": 0.15, "red_0": 0.09, "green_2": 0.03, "red_2": -0.03, "green_1": -0.09, "red_1": -0.15}
START_X = 0.24

FOOT, FOOT_H, BODY_W = 0.05, 0.008, 0.036
GRIP_Z = 0.034


def _boxes(colour):
    foot = ((0, 0, FOOT_H / 2), (FOOT, FOOT, FOOT_H), DARK)
    if colour == "red":
        return [foot, ((0, 0, FOOT_H + 0.03), (BODY_W, BODY_W, 0.06), RED)]
    return [foot, ((0, 0, FOOT_H + 0.0225), (BODY_W, BODY_W, 0.045), GREEN),
            ((0, 0, FOOT_H + 0.051), (0.026, 0.026, 0.012), GREEN), ((0, 0, FOOT_H + 0.062), (0.014, 0.014, 0.01), GREEN)]


BUOYS = {n: part(n, pos=(START_X, y, DECK_TOP + 0.001), mass=0.08, color=DARK, friction=1.0, boxes=_boxes(n.split("_")[0]))
         for n, y in STARTS.items()}

_Z = BASE + 0.0003
CHART = [
    Box("sea", pos=(0.33, 0.0, TOP + FLOOR_T / 2), size=(0.46, 0.84, FLOOR_T), color=SEA),
    Box("channel_a", pos=(0.375, 0.26, _Z), size=(0.37, 0.08, 0.0006), color=CHANNEL),
    Box("channel_b_outer", pos=(0.27, -0.24, _Z), size=(0.34, 0.08, 0.0006), color=CHANNEL),
    Box("channel_b_inner", pos=(0.40, -0.145, _Z), size=(0.08, 0.27, 0.0006), color=CHANNEL),
    Box("harbour_a", pos=(0.145, 0.26, BASE + LAND_T / 2), size=(0.09, 0.32, LAND_T), color=LAND),
    Box("harbour_b", pos=(0.445, 0.065, BASE + LAND_T / 2), size=(0.23, 0.15, LAND_T), color=LAND),
    Box("quay_a", pos=(0.186, 0.26, BASE + LAND_T + 0.001), size=(0.008, 0.16, 0.002), color=QUAY),
    Box("quay_b", pos=(0.40, -0.006, BASE + LAND_T + 0.001), size=(0.13, 0.008, 0.002), color=QUAY),
    *[Box(f"pad_{n}", pos=(*xy, BASE + PAD_T / 2), size=(0.06, 0.06, PAD_T), color=PAD) for n, xy in PADS.items()],
    Box("workboat", pos=(START_X, 0.0, BASE + DECK_T / 2), size=(0.08, 0.36, DECK_T), color=DECK),
]
DRESSING = [
    *[Box(f"cottage_{i}", pos=(0.145, y, BASE + LAND_T + 0.012), size=(0.04, 0.03, 0.024), color=c)
      for i, (y, c) in enumerate(((0.16, WHITE), (0.36, (0.75, 0.55, 0.35))))],
    *[Box(f"store_{i}", pos=(x, 0.10, BASE + LAND_T + 0.012), size=(0.05, 0.05, 0.024), color=c)
      for i, (x, c) in enumerate(((0.40, WHITE), (0.50, (0.70, 0.40, 0.30))))],
    Cylinder("lighthouse", pos=(0.53, -0.0, BASE + LAND_T + 0.035), radius=0.012, height=0.07, color=WHITE),
    Box("moored_boat_a", pos=(0.215, 0.26, BASE + 0.006), size=(0.05, 0.022, 0.012), color=WHITE),
    Box("moored_boat_b", pos=(0.40, -0.04, BASE + 0.006), size=(0.022, 0.05, 0.012), color=(0.85, 0.75, 0.2)),
]
OBJECTS = [*CHART, *DRESSING, *BUOYS.values()]

DEBUG = os.environ.get("BUOY_DEBUG") == "1"
MODE = os.environ.get("BUOY_MODE", "demo")
_K = {"k": 0}
TOLERANCE, MAX_TILT, MAX_SPEED = 0.03, 10.0, 0.02


def _tilt_deg(state):
    return float(np.degrees(np.arccos(np.clip(state.rotation()[2, 2], -1.0, 1.0))))


def _occupied(objects, xy, colour):
    for n, b in BUOYS.items():
        s = objects[b.name]
        if (n.startswith(colour) and np.linalg.norm(s.pos[:2] - np.array(xy)) <= TOLERANCE
                and abs(s.pos[2] - PAD_TOP) < 0.012 and _tilt_deg(s) < MAX_TILT and np.linalg.norm(s.lin_vel) < MAX_SPEED):
            return True
    return False


def score(objects):
    """One sixth for each mooring pad holding a buoy of the colour Region A requires there, upright and at rest within
    3 cm of the pad's centre."""
    hits = {p: _occupied(objects, xy, REQUIRED[p]) for p, xy in PADS.items()}
    total = sum(hits.values()) / 6
    _K["k"] += 1
    if DEBUG and _K["k"] % 40 == 0:
        print(f"BUOY step {_K['k']} score {total:.3f} " + " ".join(f"{p}:{int(h)}" for p, h in hits.items()) + " " + " ".join(
            f"{n} {np.round(objects[b.name].pos, 3)} t{_tilt_deg(objects[b.name]):.0f}" for n, b in BUOYS.items()), flush=True)
    return float(total)


TRAVEL = TOP + 0.24
YAW = -math.pi / 2


def _move(name, pad):
    x0, y0 = START_X, STARTS[name]
    tx, ty = PADS[pad]
    grip = DECK_TOP + 0.001 + GRIP_Z + 0.008
    rest = PAD_TOP + GRIP_Z + 0.008 + 0.003
    return [(1.3, (x0, y0, TRAVEL), FINGER_OPEN, "above the buoy", YAW), (1.1, (x0, y0, grip), FINGER_OPEN, "descend", YAW),
            (0.4, (x0, y0, grip), FINGER_OPEN, "settle", YAW), (0.6, (x0, y0, grip), FINGER_CLOSED, "grip", YAW),
            (0.9, (x0, y0, TRAVEL), FINGER_CLOSED, "lift", YAW), (1.8, (tx, ty, TRAVEL), FINGER_CLOSED, "carry", YAW),
            (1.0, (tx, ty, rest + 0.02), FINGER_CLOSED, "lower", YAW), (0.5, (tx, ty, rest), FINGER_CLOSED, "set down", YAW),
            (0.5, (tx, ty, rest), FINGER_OPEN, "let go", YAW), (0.9, (tx, ty, TRAVEL), FINGER_OPEN, "up", YAW)]


def _plan():
    side = {"port": "starboard", "starboard": "port"}
    left = [("green_0", "a_starboard"), ("red_0", "a_port")]
    right = [("red_1", "b_port"), ("green_1", "b_starboard"), ("red_2", "c_port"), ("green_2", "c_starboard")]
    def flip(p):
        g, s = p.split("_")
        return f"{g}_{side[s]}"
    if MODE == "reversed":
        left, right = [(n, flip(p)) for n, p in left], [(n, flip(p)) for n, p in right]
    if MODE == "habit":
        left = [(n, flip(p)) for n, p in left]
    if MODE == "one":
        right = []
    return left, right


def demo():
    """The left arm moors the two buoys of the straight channel, the right arm the four of the channel with the bend."""
    if MODE == "none":
        return {"left": [Waypoint(1.0, (0.12, 0.42, TRAVEL), FINGER_OPEN, "idle", YAW)]}
    plan_l, plan_r = _plan()
    left = [s for n, p in plan_l for s in _move(n, p)] + [(1.5, (0.12, 0.42, TRAVEL), FINGER_OPEN, "move clear", YAW)]
    right = ([s for n, p in plan_r for s in _move(n, p)] or [(1.0, (0.12, -0.42, TRAVEL), FINGER_OPEN, "idle", YAW)]) + [
        (1.5, (0.12, -0.42, TRAVEL), FINGER_OPEN, "move clear", YAW), (2.0, (0.12, -0.42, TRAVEL), FINGER_OPEN, "settle", YAW)]
    return {"left": [Waypoint(*s) for s in left], "right": [Waypoint(*s) for s in right]}
