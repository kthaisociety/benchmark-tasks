"""Five rigid product boxes and an open tote fixed to the table.

The tote's walls stand on the table; inside it is 28 cm front to back and 36 cm side to side, with 6 cm
tall, 1 cm thick walls, centred 30 cm in front of the arm bases on the midline. All five boxes are 6.5 cm
tall and lie flat beside the tote: three long ones (24 x 6, 22 x 7 and 18 x 6 cm) on the left, two short ones
(11 x 6 and 9 x 7 cm) on the right. They only all fit flat on the tote floor in one arrangement.
"""

import math
import sys
import os

import numpy as np
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Put all five boxes in the tote, each lying flat on the tote floor."

H = 0.065                                   # every box is 6.5 cm tall
WALL_H, WALL_T = 0.06, 0.01
X0, X1, Y0, Y1 = 0.16, 0.44, -0.18, 0.18     # tote inner faces
WALL_COLOR = (0.30, 0.32, 0.36)

# name: (length, width, colour, mass, start centre, start lies along x?)
BOXES = {
    "red_box": (0.24, 0.06, (0.80, 0.25, 0.20), 0.15, (0.44, 0.430), True),
    "blue_box": (0.22, 0.07, (0.20, 0.42, 0.80), 0.13, (0.42, 0.335), True),
    "yellow_box": (0.18, 0.06, (0.95, 0.75, 0.15), 0.12, (0.42, 0.235), True),
    "green_box": (0.11, 0.06, (0.25, 0.62, 0.32), 0.10, (0.33, -0.290), False),
    "purple_box": (0.09, 0.07, (0.58, 0.33, 0.70), 0.10, (0.43, -0.275), False),
}
NAMES = tuple(BOXES)


def _box(name):
    length, width, color, mass, (x, y), along_x = BOXES[name]
    size = (length, width, H) if along_x else (width, length, H)
    return Box(name, pos=(x, y, TOP + H / 2 + 0.001), size=size, color=color, mass=mass)


WALLS = [
    Box("tote_near", pos=(X0 - WALL_T / 2, 0.0, TOP + WALL_H / 2), size=(WALL_T, Y1 - Y0 + 2 * WALL_T, WALL_H), color=WALL_COLOR),
    Box("tote_far", pos=(X1 + WALL_T / 2, 0.0, TOP + WALL_H / 2), size=(WALL_T, Y1 - Y0 + 2 * WALL_T, WALL_H), color=WALL_COLOR),
    Box("tote_left", pos=((X0 + X1) / 2, Y1 + WALL_T / 2, TOP + WALL_H / 2), size=(X1 - X0, WALL_T, WALL_H), color=WALL_COLOR),
    Box("tote_right", pos=((X0 + X1) / 2, Y0 - WALL_T / 2, TOP + WALL_H / 2), size=(X1 - X0, WALL_T, WALL_H), color=WALL_COLOR),
]
OBJECTS = WALLS + [_box(n) for n in NAMES]

SUCCESS_HOLD_S = 1.0
FLOOR_TOL = 0.005       # lowest point within 5 mm of the floor (the table)
TILT_MAX = 10.0         # degrees from flat
EPS = 1e-4


def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _corners(name, o):
    length, width, *_ , along_x = BOXES[name]
    sx, sy = (length, width) if along_x else (width, length)
    half = np.array([sx, sy, H]) / 2
    signs = np.array([[a, b, c] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)], dtype=float)
    return np.asarray(o.pos, dtype=float) + (signs * half) @ _rot(o.quat_xyzw).T


def _tilt_deg(o):
    return math.degrees(math.acos(max(-1.0, min(1.0, _rot(o.quat_xyzw)[2, 2]))))


def packed(name, o):
    """Flat on the tote floor, wholly inside the inner walls, at rest."""
    c = _corners(name, o)
    return bool(
        c[:, 2].min() - TOP <= FLOOR_TOL
        and _tilt_deg(o) <= TILT_MAX
        and c[:, 0].min() >= X0 - EPS and c[:, 0].max() <= X1 + EPS
        and c[:, 1].min() >= Y0 - EPS and c[:, 1].max() <= Y1 + EPS
        and o.resting()
    )


def outside(name, o):
    """On the table, clear of the tote's outer outline."""
    c = _corners(name, o)
    clear = (c[:, 0].max() <= X0 - WALL_T or c[:, 0].min() >= X1 + WALL_T
             or c[:, 1].max() <= Y0 - WALL_T or c[:, 1].min() >= Y1 + WALL_T)
    return bool(clear and c[:, 2].min() - TOP <= FLOOR_TOL)


_DEBUG = {"step": 0}


def _debug(objects):
    _DEBUG["step"] += 1
    if _DEBUG["step"] % 40:
        return
    rows = []
    for n in NAMES:
        o, c = objects[n], _corners(n, objects[n])
        yaw = math.degrees(math.atan2(_rot(o.quat_xyzw)[1, 0], _rot(o.quat_xyzw)[0, 0]))
        rows.append(f"{n[:-4]} ({o.pos[0]:.3f},{o.pos[1]:.3f}) low {100*(c[:,2].min()-TOP):.1f} tilt {_tilt_deg(o):.1f} "
                    f"yaw {yaw:+.0f} x[{c[:,0].min():.3f},{c[:,0].max():.3f}] y[{c[:,1].min():.3f},{c[:,1].max():.3f}] "
                    f"rest {int(o.resting())} P{int(packed(n, o))}")
    print(f"[fit] step {_DEBUG['step']}: " + " | ".join(rows), file=sys.stderr, flush=True)


def score(objects):
    if os.environ.get("FIT_BOXES_DEBUG"):
        _debug(objects)
    inside = [n for n in NAMES if packed(n, objects[n])]
    if len(inside) == 5:
        return 1.0
    if len(inside) == 4:
        (rest,) = [n for n in NAMES if n not in inside]
        return 0.25 if outside(rest, objects[rest]) else 0.0
    return 0.0


def success(objects):
    return all(packed(n, objects[n]) for n in NAMES)


# ---------------------------------------------------------------- scripted demo

# Intended places (centre x, centre y, long side along x?). Boxes beside the right wall sit 5 mm inboard.
PLACE = {
    "red_box": (0.385, 0.040, False),
    "blue_box": (0.220, 0.050, False),
    "yellow_box": (0.305, 0.070, False),
    "green_box": (0.350, -0.125, True),
    "purple_box": (0.230, -0.110, True),
}
GRIP_Z = PAD_TIP_PAST_TCP + 0.015            # pad tips 1.5 cm above the surface the box stands on
TRAVEL = TOP + 0.15
ABOVE = TOP + 0.115                          # box bottom 1.2 cm above the box tops / 1.9 cm above the walls


def _close(name):
    return -(BOXES[name][1] - 0.012) / 2     # 12 mm narrower than the box: firm, centring grip


def _yaw(along_x):
    # yaw 0 closes the fingers along world y, so a box lying along x is gripped at yaw 0.
    return 0.0 if along_x else -math.pi / 2


# Where along its length each box is gripped, from its centre (m, along the box as it lies at the start).
# The red box is gripped 4 cm off centre so that its far, inboard place stays inside the left arm's reach.
GRIP_OFFSET = {"red_box": -0.04}


def move(name, place=None):
    _, _, _, _, (cx, cy), start_x = BOXES[name]
    qx, qy, end_x = place or PLACE[name]
    a, b = _yaw(start_x), _yaw(end_x)
    off = GRIP_OFFSET.get(name, 0.0)
    ox, oy = (off, 0.0) if start_x else (0.0, off)
    px, py = cx + ox, cy + oy
    turn = b - a                                  # the box turns with the gripper
    qx += ox * math.cos(turn) - oy * math.sin(turn)
    qy += ox * math.sin(turn) + oy * math.cos(turn)
    grip = _close(name)
    wide = -(BOXES[name][1] + 0.02) / 2           # pads 1 cm clear of the box on each side
    release = -(BOXES[name][1] + 0.012) / 2       # 6 mm clear: lets go without reaching the neighbours
    zp, zq = TOP + GRIP_Z, TOP + GRIP_Z + 0.004
    return [
        Waypoint(1.5, (px, py, TRAVEL), wide, f"{name}: above", a),
        Waypoint(1.5, (px, py, zp), wide, "descend", a),
        Waypoint(0.4, (px, py, zp), wide, "settle", a),
        Waypoint(0.8, (px, py, zp), grip, "close", a),
        Waypoint(2.0, (px, py, TRAVEL), grip, "lift", a),
        Waypoint(3.0, (qx, qy, TRAVEL), grip, "carry and turn", b),
        Waypoint(1.0, (qx, qy, ABOVE), grip, "above the gap", b),
        Waypoint(0.8, (qx, qy, ABOVE), grip, "settle", b),
        Waypoint(2.0, (qx, qy, zq), grip, "lower", b),
        Waypoint(0.4, (qx, qy, zq), grip, "settle", b),
        Waypoint(0.6, (qx, qy, zq), release, "release", b),
        Waypoint(1.0, (qx, qy, TRAVEL), release, "retreat", b),
    ]


def _wait(seconds, arm):
    home = (0.25, 0.25 if arm == "left" else -0.25, TRAVEL)
    return [Waypoint(seconds, home, FINGER_OPEN, "wait", 0.0)]


def _park(arm):
    return [Waypoint(1.5, (0.20, 0.30 if arm == "left" else -0.30, TRAVEL), FINGER_OPEN, "park", 0.0)]


MOVE_S = sum(w.duration for w in move("red_box"))


def demo():
    variant = os.environ.get("FIT_BOXES_VARIANT", "full")
    if variant == "full":
        # Left: far row, middle row, near row. Right: the green box alongside the yellow one, then the purple
        # box once the blue one is in (both go by the near wall, too close to place at the same time).
        left = move("red_box") + move("yellow_box") + move("blue_box") + _park("left")
        right = _wait(MOVE_S, "right") + move("green_box") + _wait(MOVE_S, "right") + move("purple_box") + _park("right")
    elif variant == "four":
        # Everything correct except the purple box, left on its mat: 0.25.
        left = move("red_box") + move("yellow_box") + move("blue_box") + _park("left")
        right = _wait(MOVE_S, "right") + move("green_box") + _park("right")
    elif variant == "stack":
        # Four correct, the purple box set on top of the blue box: 0.
        left = move("red_box") + move("yellow_box") + move("blue_box") + _park("left")
        stack = (0.220, 0.020, True)
        right = _wait(3 * MOVE_S, "right") + move("green_box") + _stack_move("purple_box", stack) + _park("right")
    elif variant == "nothing":
        left, right = _park("left"), _park("right")
    else:
        raise ValueError(f"unknown FIT_BOXES_VARIANT {variant!r}")
    return {"left": left, "right": right}


def _stack_move(name, place):
    """Like move(), but released on top of a packed box (one box height higher)."""
    out = []
    for w in move(name, place):
        if w.label in ("lower", "settle", "release") and w.pos[2] < ABOVE:
            w = Waypoint(w.duration, (w.pos[0], w.pos[1], w.pos[2] + H), w.finger, w.label, w.yaw)
        out.append(w)
    return out
