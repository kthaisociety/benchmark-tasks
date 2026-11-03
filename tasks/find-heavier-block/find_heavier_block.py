"""On the station table: a balance made of a pine plank (36 x 8 x 1.2 cm) resting on a fixed wooden ridge 3 cm
tall, running left to right. In front of it, three grey 4 cm blocks that look the same: one weighs 0.20 kg, the
others 0.10 kg. A plastic bin is fixed to the left. Equal loads on the two ends of the plank leave it level; an
unequal pair tips it toward the heavier block.
"""

import numpy as np

from yam_arena.demo import pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
BEAM_X, END_Y = 0.28, 0.14
RIDGE_TOP = TOP + 0.03
PLANK_TOP = RIDGE_TOP + 0.013
CUBE = 0.04
GREY = (0.55, 0.55, 0.58)

RIDGE = Box("ridge", pos=(BEAM_X, 0.0, TOP + 0.015), size=(0.06, 0.02, 0.03), color=(0.55, 0.40, 0.25))
PLANK = Box("plank", pos=(BEAM_X, 0.0, RIDGE_TOP + 0.007), size=(0.08, 0.36, 0.012), mass=0.15,
            color=(0.85, 0.70, 0.45))
BLOCKS = [
    Box("block_a", pos=(0.16, -0.10, TOP + CUBE / 2), size=(CUBE,) * 3, mass=0.10, color=GREY),
    Box("block_c", pos=(0.16, 0.00, TOP + CUBE / 2), size=(CUBE,) * 3, mass=0.20, color=GREY),
    Box("block_b", pos=(0.16, 0.10, TOP + CUBE / 2), size=(CUBE,) * 3, mass=0.10, color=GREY),
]
HEAVY = "block_c"
A, C, B = BLOCKS

BIN_X, BIN_Y = 0.20, 0.30
BIN_INSIDE = 0.10
BIN_FLOOR = TOP + 0.004


def _bin(wall=0.006, height=0.02, color=(0.20, 0.30, 0.60)):
    w = BIN_INSIDE
    wz = TOP + height / 2
    return [
        Box("bin_floor", pos=(BIN_X, BIN_Y, TOP + 0.002), size=(w + 2 * wall, w + 2 * wall, 0.004), color=color),
        Box("bin_front", pos=(BIN_X - (w + wall) / 2, BIN_Y, wz), size=(wall, w + 2 * wall, height), color=color),
        Box("bin_back", pos=(BIN_X + (w + wall) / 2, BIN_Y, wz), size=(wall, w + 2 * wall, height), color=color),
        Box("bin_left", pos=(BIN_X, BIN_Y + (w + wall) / 2, wz), size=(w, wall, height), color=color),
        Box("bin_right", pos=(BIN_X, BIN_Y - (w + wall) / 2, wz), size=(w, wall, height), color=color),
    ]


OBJECTS = [RIDGE, PLANK, *BLOCKS, *_bin()]

START_XY = {b.name: np.array(b.pos[:2]) for b in BLOCKS}
_weighed = False


def _tilt_deg(obj):
    x, y, _, _ = obj.quat_xyzw
    return float(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def _on_plank_end(block, plank):
    """Resting on one end of the plank (sign of y), or None."""
    x, y = block.pos[:2]
    on = abs(x - BEAM_X) < 0.04 and 0.03 < abs(y) < 0.18 and block.low[2] < plank.high[2] + 0.005
    return np.sign(y) if on and block.low[2] > TOP + 0.01 and block.resting() else None


def _in_bin(block):
    dx, dy = np.abs(block.pos[:2] - (BIN_X, BIN_Y))
    return bool(dx < BIN_INSIDE / 2 and dy < BIN_INSIDE / 2 and block.low[2] < BIN_FLOOR + 0.01 and block.resting())


def _track(objects):
    """Remember, for the whole trial, whether two blocks were weighed against each other."""
    global _weighed
    plank = objects["plank"]
    if all(np.linalg.norm(objects[n].pos[:2] - xy) < 0.005 for n, xy in START_XY.items()) and _tilt_deg(plank) < 1:
        _weighed = False
    sides = {_on_plank_end(objects[n], plank) for n in START_XY}
    _weighed = _weighed or ({1.0, -1.0} <= sides and plank.resting())


def score(objects):
    """0 unless two blocks were weighed against each other at some point. Then 1.0 if the bin holds only the
    heavy block, otherwise 0.25."""
    _track(objects)
    if not _weighed:
        return 0.0
    in_bin = [n for n in START_XY if _in_bin(objects[n])]
    return 1.0 if in_bin == [HEAVY] else 0.25


def success(objects):
    return score(objects) == 1.0


def demo():
    """Both arms weigh A against B at the same time; the plank stays level, so the left arm bins C."""
    travel = TOP + 0.16
    on_end = PLANK_TOP + CUBE / 2 + 0.005
    in_bin = BIN_FLOOR + CUBE / 2 + 0.005
    return {
        "right": pick_place_waypoints(A.pos, (BEAM_X, -END_Y, on_end), travel_z=travel),
        "left": [
            *pick_place_waypoints(B.pos, (BEAM_X, END_Y, on_end), travel_z=travel),
            *pick_place_waypoints(C.pos, (BIN_X, BIN_Y, in_bin), travel_z=travel),
        ],
    }
