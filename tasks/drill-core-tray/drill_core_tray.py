"""On the station table, a core shed bench: a run of drill core lies on a V-rack in the order it came out of the hole,
top of hole at the rack's left end, with a gap where one interval of core was lost. Core is boxed like reading a book:
each tray row from left to right, rows from the top of the tray down, every piece with its uphole end to the left, and
a core-loss block of the lost interval's length in the tray where the core is missing, so every later piece keeps its
depth.
"""

import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from partkit import part  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
BOARD_T, RIDGE_H, CORE = 0.004, 0.004, 0.03
FLOOR = TOP + BOARD_T

PINE, TRAY, RIDGE, STEEL, RED, YELLOW, WOOD = (0.76, 0.62, 0.42), (0.16, 0.17, 0.18), (0.10, 0.10, 0.11), \
    (0.50, 0.51, 0.54), (0.80, 0.15, 0.12), (0.92, 0.72, 0.10), (0.85, 0.74, 0.55)
ROCK = {"granite": (0.66, 0.62, 0.60), "schist": (0.36, 0.40, 0.38), "quartz": (0.90, 0.90, 0.88),
        "sulphide": (0.55, 0.48, 0.25), "basalt": (0.20, 0.21, 0.22)}

PIECES = (("core_1", 0.10, "granite"), ("core_2", 0.08, "schist"), ("core_3", 0.13, "quartz"),
          ("core_4", 0.09, "sulphide"), ("core_5", 0.10, "basalt"))
LOST = 0.06
RACK_X, RACK_TOP_Y, BUTT = 0.22, 0.286, 0.003
ROWS_X, ROW_W, ROW_START, PIECE_GAP = (0.475, 0.39, 0.305), 0.085, 0.1025, 0.015
SPARES = {"loss_block_9": (0.09, (0.31, 0.25)), "loss_block_3": (0.03, (0.39, 0.25)), "loss_block_6": (0.06, (0.47, 0.25))}
LOSS = "loss_block_6"
LAYOUT = (("core_1", "core_2"), (LOSS, "core_3"), ("core_4", "core_5"))


def _rack_starts():
    y, out = RACK_TOP_Y, {}
    for name, length, _ in PIECES:
        out[name] = (RACK_X, y - length / 2)
        y -= length + BUTT
        if name == "core_2":
            y -= LOST
    return out


def _lengths():
    return {**{n: length for n, length, _ in PIECES}, **{n: length for n, (length, _) in SPARES.items()}}


def _targets(layout=LAYOUT):
    lengths, out = _lengths(), {}
    for x, row in zip(ROWS_X, layout):
        y = ROW_START
        for name in row:
            out[name] = (x, y - lengths[name] / 2)
            y -= lengths[name] + PIECE_GAP
    return out


STARTS = {**_rack_starts(), **{n: xy for n, (_, xy) in SPARES.items()}}
TARGETS = _targets()


def _core(name, length, rock):
    cap = 0.006
    return part(name, pos=(*STARTS[name], FLOOR + 0.001), mass=0.08, color=ROCK[rock], friction=1.0, boxes=[
        ((0, -cap / 2, CORE / 2), (CORE, length - cap, CORE), ROCK[rock]),
        ((0, length / 2 - cap / 2, CORE / 2), (CORE, cap, CORE), RED)])


def _spare(name, length):
    return part(name, pos=(*STARTS[name], FLOOR + 0.001), mass=0.03, color=WOOD, friction=1.0,
                boxes=[((0, 0, CORE / 2), (CORE, length, CORE), WOOD)])


def _bench():
    rack_len = 2 * RACK_TOP_Y + 0.02
    tray_x0, tray_x1 = ROWS_X[-1] - ROW_W / 2, ROWS_X[0] + ROW_W / 2
    tray_len = 2 * ROW_START + 0.03
    out = [Box("rack_board", pos=(RACK_X, 0.0, TOP + BOARD_T / 2), size=(0.07, rack_len, BOARD_T), color=STEEL),
           Box("rack_top_tag", pos=(RACK_X, RACK_TOP_Y + 0.022, FLOOR + 0.002), size=(0.05, 0.016, 0.004), color=RED),
           Box("tray_floor", pos=((tray_x0 + tray_x1) / 2, 0.0, TOP + BOARD_T / 2),
               size=(tray_x1 - tray_x0, tray_len, BOARD_T), color=TRAY),
           Box("tray_label", pos=(tray_x1 + 0.012, ROW_START - 0.02, TOP + 0.002), size=(0.018, 0.05, 0.004), color=YELLOW),
           Box("spares_board", pos=(0.39, 0.25, TOP + BOARD_T / 2), size=(0.22, 0.12, BOARD_T), color=PINE)]
    for i, x in enumerate((tray_x0, *[(a + b) / 2 for a, b in zip(ROWS_X, ROWS_X[1:])], tray_x1)):
        out.append(Box(f"tray_ridge_{i}", pos=(x, 0.0, FLOOR + RIDGE_H / 2), size=(0.004, tray_len, RIDGE_H), color=RIDGE))
    for i, y in enumerate((tray_len / 2, -tray_len / 2)):
        out.append(Box(f"tray_end_{i}", pos=((tray_x0 + tray_x1) / 2, y, FLOOR + RIDGE_H / 2),
                       size=(tray_x1 - tray_x0, 0.004, RIDGE_H), color=RIDGE))
    return out


CORES = {n: _core(n, length, rock) for n, length, rock in PIECES}
LOSS_BLOCKS = {n: _spare(n, length) for n, (length, _) in SPARES.items()}
OBJECTS = [*_bench(), *CORES.values(), *LOSS_BLOCKS.values()]

DEBUG = os.environ.get("CORE_DEBUG") == "1"
MODE = os.environ.get("CORE_MODE", "demo")
GRIP_YAW = float(os.environ.get("CORE_YAW", "1.5708"))
SEAT_X, SEAT_Y, MAX_TILT, MAX_SPEED = 0.02, 0.02, 15.0, 0.02
_K = {"k": 0}


def _tilt_deg(s):
    return float(np.degrees(np.arccos(np.clip(s.rotation()[2, 2], -1.0, 1.0))))


def _in_tray(s):
    return ROWS_X[-1] - ROW_W / 2 <= s.pos[0] <= ROWS_X[0] + ROW_W / 2 and abs(s.pos[1]) <= ROW_START + 0.015


def _placed(s, xy, uphole_left):
    resting = _tilt_deg(s) < MAX_TILT and np.linalg.norm(s.lin_vel) < MAX_SPEED and abs(s.pos[2] - FLOOR) < 0.012
    seated = abs(s.pos[0] - xy[0]) <= SEAT_X and abs(s.pos[1] - xy[1]) <= SEAT_Y
    return resting and seated and (not uphole_left or s.rotation()[1, 1] > 0.7)


def _results(objects):
    out = {n: _placed(objects[n], TARGETS[n], True) for n in CORES}
    wrong_spare = any(_in_tray(objects[n]) for n in LOSS_BLOCKS if n != LOSS)
    out[LOSS] = _placed(objects[LOSS], TARGETS[LOSS], False) and not wrong_spare
    return out


def score(objects):
    """1/6 for each of the five core pieces and the 6 cm core-loss block resting in its tray row within 2 cm of its
    place in reading order, each core piece with its red uphole end to the left; the loss block earns nothing while a
    spare block of another length lies in the tray."""
    results = _results(objects)
    total = sum(results.values()) / 6
    _K["k"] += 1
    if DEBUG and _K["k"] % 40 == 0:
        print(f"CORE step {_K['k']} score {total:.3f} " + " ".join(
            f"{n[-1]}:{int(v)}:{np.round(objects[n].pos[:2], 3).tolist()}/{_tilt_deg(objects[n]):.0f}"
            for n, v in results.items()), flush=True)
    return float(total)


TRAVEL = TOP + 0.22
AIM = {"core_1": (0.01, 0.0)}


def _move(name, xy):
    x0, y0 = STARTS[name]
    ax, ay = AIM.get(name, (0.0, 0.0))
    tx, ty = xy[0] + ax, xy[1] + ay
    grip = FLOOR + 0.001 + CORE / 2 + 0.008
    rest = FLOOR + CORE / 2 + 0.008 + 0.003
    yaw = GRIP_YAW
    return [(1.3, (x0, y0, TRAVEL), FINGER_OPEN, "above the piece", yaw), (1.1, (x0, y0, grip), FINGER_OPEN, "descend", yaw),
            (0.4, (x0, y0, grip), FINGER_OPEN, "settle", yaw), (0.6, (x0, y0, grip), FINGER_CLOSED, "grip", yaw),
            (1.0, (x0, y0, TRAVEL), FINGER_CLOSED, "lift", yaw), (2.4, (tx, ty, TRAVEL), FINGER_CLOSED, "carry", yaw),
            (1.2, (tx, ty, rest + 0.02), FINGER_CLOSED, "lower into the row", yaw),
            (0.6, (tx, ty, rest), FINGER_CLOSED, "set down", yaw), (0.5, (tx, ty, rest), FINGER_OPEN, "let go", yaw),
            (0.9, (tx, ty, TRAVEL), FINGER_OPEN, "up", yaw)]


def _plan():
    targets = TARGETS
    if MODE == "closed":
        targets = _targets((("core_1", "core_2"), ("core_3", "core_4"), ("core_5",)))
    if MODE == "snake":
        targets = _targets((("core_1", "core_2"), ("core_3", LOSS), ("core_4", "core_5")))
    if MODE == "wrong_block":
        targets = {**_targets((("core_1", "core_2"), ("loss_block_9", "core_3"), ("core_4", "core_5")))}
    order_l = ["core_1", LOSS, "loss_block_9", "core_4", "core_5"]
    order_r = ["core_2", "core_3", "core_5"]
    left = [(n, targets[n]) for n in order_l if n in targets and n not in order_r]
    right = [(n, targets[n]) for n in order_r if n in targets]
    if MODE == "one":
        left, right = left[:1], []
    return left, right


def demo():
    """The left arm boxes core 1 at the start of the top row, the 6 cm loss block at the start of the middle row and core 4
    at the start of the bottom row; then the right arm boxes core 2, core 3 and core 5 after them."""
    if MODE == "none":
        return {"left": [Waypoint(1.0, (0.12, 0.42, TRAVEL), FINGER_OPEN, "idle", 0.0)]}
    plan_l, plan_r = _plan()
    left = [s for n, xy in plan_l for s in _move(n, xy)] + [(1.5, (0.12, 0.42, TRAVEL), FINGER_OPEN, "move clear", 0.0)]
    right = [(sum(w[0] for w in left), (0.12, -0.42, TRAVEL), FINGER_OPEN, "wait", 0.0)]
    right += [s for n, xy in plan_r for s in _move(n, xy)]
    right += [(1.5, (0.12, -0.42, TRAVEL), FINGER_OPEN, "move clear", 0.0), (2.0, (0.12, -0.42, TRAVEL), FINGER_OPEN, "settle", 0.0)]
    return {"left": [Waypoint(*s) for s in left], "right": [Waypoint(*s) for s in right]}
