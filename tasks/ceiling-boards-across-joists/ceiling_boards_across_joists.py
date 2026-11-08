"""On the station table, a dryliner's job: a ceiling bay, built upside down so it can be boarded from above, with three
timber joists running away from the arms between two wall plates. Two plasterboard sheets lie on board dollies beside
the arms, their long edges running the same way as the joists. A ceiling board goes up with its long edges at right
angles to the joists and both ends landing on a joist, so every edge is supported; so each sheet must be turned a
quarter turn before it is laid.
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

TIMBER, PLATE_C, BOARD_C, PAPER_C = (0.72, 0.55, 0.36), (0.60, 0.44, 0.28), (0.88, 0.88, 0.86), (0.80, 0.82, 0.86)
DOLLY_C, TOOL_C = (0.25, 0.27, 0.30), (0.12, 0.35, 0.70)

JOIST_H, JOIST_W = 0.04, 0.02
JOIST_X = (0.26, 0.46)
JOIST_YS = (-0.10, 0.0, 0.10)
BAY = [
    *[Box(f"joist_{i}", pos=((JOIST_X[0] + JOIST_X[1]) / 2, y, TOP + JOIST_H / 2),
          size=(JOIST_X[1] - JOIST_X[0], JOIST_W, JOIST_H), color=TIMBER) for i, y in enumerate(JOIST_YS)],
    *[Box(f"wall_plate_{i}", pos=(x, 0.0, TOP + JOIST_H / 2), size=(0.02, 0.26, JOIST_H), color=PLATE_C)
      for i, x in enumerate((JOIST_X[0] - 0.01, JOIST_X[1] + 0.01))],
]

BOARD = (0.22, 0.06, 0.010)
DOLLY_H = 0.008
START = {"left": (0.26, 0.26), "right": (0.26, -0.26)}
SHEETS = {arm: part(f"board_{arm}", pos=(*xy, TOP + DOLLY_H + BOARD[2] / 2 + 0.001), mass=0.12, color=BOARD_C,
                    friction=1.0, boxes=[((0, 0, 0), BOARD, BOARD_C),
                                         ((0, 0, BOARD[2] / 2 + 0.0003), (BOARD[0] - 0.01, 0.003, 0.0006), PAPER_C)])
          for arm, xy in START.items()}
DOLLIES = [Box(f"board_dolly_{arm}", pos=(*xy, TOP + DOLLY_H / 2), size=(0.24, 0.07, DOLLY_H), color=DOLLY_C)
           for arm, xy in START.items()]

DRESSING = [
    Box("screw_gun_case", pos=(0.60, -0.26, TOP + 0.02), size=(0.10, 0.07, 0.04), color=TOOL_C),
    Cylinder("drywall_screws", pos=(0.60, -0.12, TOP + 0.025), radius=0.022, height=0.05, color=(0.30, 0.30, 0.32)),
    Box("drywall_square_blade", pos=(0.60, 0.20, TOP + 0.002), size=(0.20, 0.012, 0.004), color=(0.80, 0.80, 0.82)),
    Box("drywall_square_head", pos=(0.51, 0.20, TOP + 0.004), size=(0.012, 0.08, 0.008), color=(0.80, 0.80, 0.82)),
    Box("utility_knife", pos=(0.62, 0.05, TOP + 0.006), size=(0.08, 0.018, 0.012), color=(0.95, 0.75, 0.10)),
]

OBJECTS = [*BAY, *DOLLIES, *DRESSING, *SHEETS.values()]

DEBUG = os.environ.get("CEIL_DEBUG") == "1"
_K = {"k": 0}
TOLERANCE = math.radians(15)


def _laid(state):
    """At rest on the joists, long edges within 15 degrees of square to them, and both ends over a joist."""
    axis = state.rotation()[:, 0]
    square = abs(math.atan2(abs(axis[0]), abs(axis[1]))) < TOLERANCE
    on_joists = abs(state.pos[2] - BOARD[2] / 2 - (TOP + JOIST_H)) < 0.003
    within_bay = JOIST_X[0] < state.pos[0] < JOIST_X[1]
    ends = [state.pos + s * BOARD[0] / 2 * axis for s in (-1, 1)]
    ends_supported = all(min(abs(e[1] - y) for y in JOIST_YS) < JOIST_W / 2 + 0.005 for e in ends)
    still = np.linalg.norm(state.lin_vel) < 0.02
    return bool(square and on_joists and within_bay and ends_supported and still)


def score(objects):
    """0.5 for each board laid across the joists with both ends landing on a joist."""
    boards = [objects[s.name] for s in SHEETS.values()]
    total = 0.5 * sum(_laid(b) for b in boards)
    _K["k"] += 1
    if DEBUG and _K["k"] % 40 == 0:
        def yaw(b):
            return math.degrees(math.atan2(b.rotation()[1, 0], b.rotation()[0, 0]))
        print(f"CEIL step {_K['k']} score {total} " + " ".join(
            f"{s.name} {np.round(objects[s.name].pos, 3)} yaw {yaw(objects[s.name]):.1f} laid {_laid(objects[s.name])}"
            for s in SHEETS.values()), flush=True)
    return total


TRAVEL = TOP + 0.20
TURN = {"left": math.pi / 2, "right": -math.pi / 2}
GRIP_SKEW = {"left": -0.10, "right": 0.23}
PLACE_X = {"left": 0.40, "right": 0.32}
MODE = os.environ.get("CEIL_MODE", "turn")


def _lay(arm, turn):
    sx, sy = START[arm]
    grip = TOP + DOLLY_H + BOARD[2] / 2 + 0.008
    place = TOP + JOIST_H + BOARD[2] / 2 + 0.008 + 0.004
    x, y = PLACE_X[arm], float(os.environ.get("CEIL_Y", "0.0"))
    skew = GRIP_SKEW[arm] if turn else 0.0
    yaw = TURN[arm] if turn else 0.0
    return [(1.5, (sx, sy, TRAVEL), FINGER_OPEN, "above the board", 0.0),
            (1.5, (sx, sy, grip), FINGER_OPEN, "descend", skew), (0.6, (sx, sy, grip), FINGER_OPEN, "settle", skew),
            (0.8, (sx, sy, grip), FINGER_CLOSED, "grip", skew),
            (1.5, (sx, sy, grip + 0.10), FINGER_CLOSED, "lift", skew),
            (2.5, (x, y + (0.06 if arm == "left" else -0.06), place + 0.08), FINGER_CLOSED, "carry and turn", yaw),
            (1.5, (x, y, place + 0.03), FINGER_CLOSED, "over the joists", yaw),
            (1.2, (x, y, place), FINGER_CLOSED, "lower onto the joists", yaw),
            (0.8, (x, y, place), FINGER_OPEN, "let go", yaw),
            (1.2, (x, y, TRAVEL), FINGER_OPEN, "up", yaw)]


def demo():
    """The left arm turns its board a quarter turn and lays it across the joists, both ends on the outer joists;
    then the right arm does the same with the other board, nearer the arms."""
    if MODE == "none":
        return {"left": [Waypoint(1.0, (0.12, 0.42, TRAVEL), FINGER_OPEN, "idle", 0.0)]}
    turn_left, turn_right = {"turn": (True, True), "parallel": (False, False), "one": (True, False)}[MODE]
    left = _lay("left", turn_left) + [(2.0, (0.12, 0.42, TRAVEL), FINGER_OPEN, "move clear", 0.0)]
    wait = sum(s[0] for s in left)
    right = [(wait, (0.25, -0.30, TRAVEL), FINGER_OPEN, "wait", 0.0), *_lay("right", turn_right),
             (2.0, (0.12, -0.42, TRAVEL), FINGER_OPEN, "move clear", 0.0),
             (2.0, (0.12, -0.42, TRAVEL), FINGER_OPEN, "let everything settle", 0.0)]
    return {"left": [Waypoint(*s) for s in left], "right": [Waypoint(*s) for s in right]}
