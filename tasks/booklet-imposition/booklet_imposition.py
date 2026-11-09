"""On the station table, a print shop's layout bench: eight numbered page blocks and four boards, each board one side
of a sheet with a left and a right page position either side of its fold. The sheets are folded, nested and stitched
through the fold, so each side must carry two pages that are far apart in reading order.
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
FLOOR_T, BOARD_T = 0.002, 0.004
BASE = TOP + FLOOR_T
BOARD_TOP = BASE + BOARD_T

BENCH, BOARD, SLOT, INK, PAPER, TIN = (0.46, 0.38, 0.30), (0.20, 0.27, 0.36), (0.80, 0.82, 0.78), (0.05, 0.05, 0.06), \
    (0.96, 0.95, 0.90), (0.62, 0.20, 0.16)

BLOCK, BLOCK_H, SEG_T, SEG_Z = 0.05, 0.05, 0.006, 0.0005
DIGIT_H, DIGIT_W = 0.042, 0.028
BOARDS_XY, SLOT_OFFSET = ((0.455, 0.13), (0.35, 0.13), (0.455, -0.13), (0.35, -0.13)), 0.045

STARTS = {7: (0.22, 0.30), 1: (0.22, 0.215), 5: (0.22, 0.13), 2: (0.22, 0.045), 3: (0.22, -0.045), 8: (0.22, -0.13),
          6: (0.22, -0.215), 4: (0.22, -0.30)}

SEGMENTS = {1: "bc", 2: "abged", 3: "abgcd", 4: "fgbc", 5: "afgcd", 6: "afgedc", 7: "abc", 8: "abcdefg"}
_H, _W, _Z = DIGIT_H / 2, DIGIT_W / 2, BLOCK_H + SEG_Z / 2
_SEG = {"a": ((_H, 0), (SEG_T, DIGIT_W)), "g": ((0, 0), (SEG_T, DIGIT_W)), "d": ((-_H, 0), (SEG_T, DIGIT_W)),
        "f": ((_H / 2, _W), (_H, SEG_T)), "b": ((_H / 2, -_W), (_H, SEG_T)),
        "e": ((-_H / 2, _W), (_H, SEG_T)), "c": ((-_H / 2, -_W), (_H, SEG_T))}


def _page(n):
    numeral = [((*_SEG[s][0], _Z), (*_SEG[s][1], SEG_Z), INK) for s in SEGMENTS[n]]
    return part(f"page_{n}", pos=(*STARTS[n], BASE + 0.001), mass=0.1, color=PAPER, friction=1.0,
                boxes=[((0, 0, BLOCK_H / 2), (BLOCK, BLOCK, BLOCK_H), PAPER), *numeral])


def _board(i, xy):
    x, y = xy
    z = BOARD_TOP + 0.0003
    return [Box(f"board_{i}", pos=(x, y, BASE + BOARD_T / 2), size=(0.07, 0.20, BOARD_T), color=BOARD),
            Box(f"slot_{i}_left", pos=(x, y + SLOT_OFFSET, z), size=(0.056, 0.056, 0.0006), color=SLOT),
            Box(f"slot_{i}_right", pos=(x, y - SLOT_OFFSET, z), size=(0.056, 0.056, 0.0006), color=SLOT),
            Box(f"fold_{i}", pos=(x, y, z), size=(0.066, 0.003, 0.0006), color=INK),
            Box(f"head_{i}", pos=(x + 0.032, y, z), size=(0.004, 0.19, 0.0006), color=INK)]


BENCH_TOP = [Box("bench", pos=(0.33, 0.0, TOP + FLOOR_T / 2), size=(0.46, 0.84, FLOOR_T), color=BENCH)]
BOARDS = [b for i, xy in enumerate(BOARDS_XY) for b in _board(i, xy)]
DRESSING = [
    *[Box(f"ream_{i}", pos=(0.63, 0.22, TOP + 0.015 + i * 0.03), size=(0.15, 0.10, 0.03),
          color=(0.95, 0.95, 0.92) if i % 2 else (0.82, 0.70, 0.45)) for i in range(3)],
    Cylinder("ink_tin_red", pos=(0.62, -0.14, TOP + 0.03), radius=0.03, height=0.06, color=TIN),
    Cylinder("ink_tin_blue", pos=(0.62, -0.23, TOP + 0.03), radius=0.03, height=0.06, color=(0.14, 0.24, 0.55)),
    Box("proof_sheet", pos=(0.62, 0.02, TOP + 0.001), size=(0.14, 0.20, 0.002), color=PAPER),
    Box("proof_rule", pos=(0.62, 0.02, TOP + 0.0023), size=(0.11, 0.004, 0.0006), color=INK),
]
PAGES = {n: _page(n) for n in STARTS}
OBJECTS = [*BENCH_TOP, *BOARDS, *DRESSING, *PAGES.values()]

DEBUG = os.environ.get("IMPOSE_DEBUG") == "1"
_K = {"k": 0}
TOLERANCE, MAX_TILT, MAX_SPEED = 0.025, 10.0, 0.02


def _tilt_deg(state):
    return float(np.degrees(np.arccos(np.clip(state.rotation()[2, 2], -1.0, 1.0))))


def _occupant(objects, xy):
    for n, p in PAGES.items():
        s = objects[p.name]
        if (np.linalg.norm(s.pos[:2] - xy) <= TOLERANCE and abs(s.pos[2] - BOARD_TOP) < 0.012
                and _tilt_deg(s) < MAX_TILT and np.linalg.norm(s.lin_vel) < MAX_SPEED):
            return n
    return None


def _sides(objects, xy):
    x, y = xy
    return (_occupant(objects, np.array([x, y + SLOT_OFFSET])), _occupant(objects, np.array([x, y - SLOT_OFFSET])))


SHEETS = ({1, 2, 7, 8}, {3, 4, 5, 6})


def _row_score(objects, boards):
    sides = [_sides(objects, BOARDS_XY[b]) for b in boards]
    pages = {p for pair in sides for p in pair}
    if None in pages or pages not in SHEETS:
        return 0.0, sides
    return sum(0.125 + (0.125 if left % 2 == 0 else 0.0) for left, right in sides if left + right == 9), sides


def score(objects):
    """Each side of a sheet earns 0.125 for two pages at rest in its positions whose numbers add up to nine, and 0.125
    more if the even page is on the left, but only when the two sides of its sheet carry pages 1, 2, 7 and 8 or 3, 4, 5
    and 6."""
    rows = [_row_score(objects, (0, 1)), _row_score(objects, (2, 3))]
    total = sum(r[0] for r in rows)
    _K["k"] += 1
    if DEBUG and _K["k"] % 40 == 0:
        print(f"IMPOSE step {_K['k']} score {total} " + " ".join(f"{l}|{r}" for row in rows for l, r in row[1]) + " pos " + " ".join(
            f"{n}:{np.round(objects[p.name].pos[:2], 3).tolist()}/{_tilt_deg(objects[p.name]):.0f}" for n, p in PAGES.items()), flush=True)
    return float(total)


TRAVEL = TOP + 0.24
YAW = -math.pi / 2


MODE = os.environ.get("IMPOSE_MODE", "demo")
LAYOUTS = {
    "demo": ((8, 1), (2, 7), (6, 3), (4, 5)), "flipped": ((1, 8), (7, 2), (3, 6), (5, 4)),
    "naive": ((1, 2), (7, 8), (3, 4), (5, 6)), "wrong_sheet": ((8, 1), (4, 5), (6, 3), (2, 7)), "one": ((8, 1), (2, 7), (6, 3), (4, 5)),
}


def _target(n):
    for (x, y), (left, right) in zip(BOARDS_XY, LAYOUTS[MODE]):
        if n == left:
            return x, y + SLOT_OFFSET
        if n == right:
            return x, y - SLOT_OFFSET
    raise ValueError(n)


def _move(n):
    x0, y0 = STARTS[n]
    tx, ty = _target(n)
    grip = BASE + 0.001 + 0.034 + 0.008
    rest = BOARD_TOP + 0.034 + 0.008 + 0.003
    return [(1.3, (x0, y0, TRAVEL), FINGER_OPEN, "above the page", YAW), (1.1, (x0, y0, grip), FINGER_OPEN, "descend", YAW),
            (0.4, (x0, y0, grip), FINGER_OPEN, "settle", YAW), (0.6, (x0, y0, grip), FINGER_CLOSED, "grip", YAW),
            (0.9, (x0, y0, TRAVEL), FINGER_CLOSED, "lift", YAW), (1.8, (tx, ty, TRAVEL), FINGER_CLOSED, "carry", YAW),
            (1.0, (tx, ty, rest + 0.02), FINGER_CLOSED, "lower", YAW), (0.5, (tx, ty, rest), FINGER_CLOSED, "set down", YAW),
            (0.5, (tx, ty, rest), FINGER_OPEN, "let go", YAW), (0.9, (tx, ty, TRAVEL), FINGER_OPEN, "up", YAW)]


def demo():
    """The left arm lays pages 8, 1, 2 and 7, reaching across for page 8; the right arm then lays 6, 3, 4 and 5."""
    if MODE == "none":
        return {"left": [Waypoint(1.0, (0.12, 0.42, TRAVEL), FINGER_OPEN, "idle", YAW)]}
    left = [s for n in (8, 1, 2, 7) for s in _move(n)] + [(1.5, (0.12, 0.42, TRAVEL), FINGER_OPEN, "move clear", YAW)]
    right = [(sum(w[0] for w in left), (0.12, -0.42, TRAVEL), FINGER_OPEN, "wait", YAW)]
    if MODE != "one":
        right += [s for n in (6, 3, 4, 5) for s in _move(n)]
    right += [(1.5, (0.12, -0.42, TRAVEL), FINGER_OPEN, "move clear", YAW), (2.0, (0.12, -0.42, TRAVEL), FINGER_OPEN, "settle", YAW)]
    return {"left": [Waypoint(*s) for s in left], "right": [Waypoint(*s) for s in right]}
