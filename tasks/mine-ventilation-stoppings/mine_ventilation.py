"""On the station table, a mine ventilation model: two development districts, each with an intake and a return entry
driven side by side towards a coal face and joined by crosscuts through the pillar row between them. Air takes the
easiest way back to the return, so every crosscut outby the last open crosscut needs a stopping, or the air
short-circuits before it reaches the face. The districts are driven to different depths with different numbers of
crosscuts, and a stopping left standing in an entry blocks that entry.
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
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
FLOOR_T, ENTRY_T = 0.002, 0.0006
BASE = TOP + FLOOR_T

COAL, DUST, PILLAR, FACE, CONCRETE, BLUE, RED, YELLOW, STEEL, DARK = (0.14, 0.14, 0.15), (0.62, 0.62, 0.60), \
    (0.09, 0.09, 0.10), (0.04, 0.04, 0.05), (0.88, 0.88, 0.85), (0.15, 0.35, 0.85), (0.80, 0.15, 0.12), \
    (0.92, 0.72, 0.10), (0.50, 0.51, 0.54), (0.20, 0.20, 0.22)

PORTAL_X, ENTRY_W, ENTRY_OFFSET, PILLAR_W, PILLAR_H, GAP = 0.12, 0.06, 0.055, 0.05, 0.015, 0.07
DISTRICTS = {
    "north": {"y": 0.22, "crosscuts": (0.22, 0.34), "face": 0.40},
    "south": {"y": -0.20, "crosscuts": (0.18, 0.28, 0.38, 0.48), "face": 0.53},
}
STOPPING = (0.04, 0.045, 0.05)
REACH = {"left": (0.004, 0.0), "right": (0.003, 0.003)}
STARTS = {f"stopping_{i}": (x, 0.03) for i, x in enumerate((0.18, 0.26, 0.34, 0.42, 0.50))}


def _arrow(name, x, y, direction, colour):
    z = BASE + ENTRY_T + 0.0003
    return [Box(f"{name}_shaft", pos=(x, y, z), size=(0.03, 0.004, 0.0006), color=colour),
            *[Box(f"{name}_head_{i}", pos=(x + direction * 0.011, y + s * 0.005, z), size=(0.013, 0.004, 0.0006),
                  color=colour, yaw=direction * s * -math.pi / 4) for i, s in enumerate((-1, 1))]]


def _district(name, d):
    y, xs, xf = d["y"], d["crosscuts"], d["face"]
    intake_y, return_y = y + ENTRY_OFFSET, y - ENTRY_OFFSET
    length = xf - PORTAL_X
    out = [Box(f"{name}_{kind}_entry", pos=(PORTAL_X + length / 2, ey, BASE + ENTRY_T / 2), size=(length, ENTRY_W, ENTRY_T),
               color=DUST) for kind, ey in (("intake", intake_y), ("return", return_y))]
    edges = [PORTAL_X, *[e for x in xs for e in (x - GAP / 2, x + GAP / 2)], xf]
    for i in range(0, len(edges), 2):
        a, b = edges[i], edges[i + 1]
        out.append(Box(f"{name}_pillar_{i // 2}", pos=((a + b) / 2, y, BASE + PILLAR_H / 2), size=(b - a, PILLAR_W, PILLAR_H),
                       color=PILLAR))
    out += [Box(f"{name}_face", pos=(xf + 0.01, y, BASE + 0.02), size=(0.02, 0.17, 0.04), color=FACE),
            Box(f"{name}_miner", pos=(xf - 0.03, intake_y, BASE + 0.012), size=(0.045, 0.03, 0.024), color=YELLOW),
            Box(f"{name}_fan", pos=(PORTAL_X - 0.008, intake_y, BASE + 0.02), size=(0.016, 0.06, 0.04), color=STEEL),
            Box(f"{name}_fan_inlet", pos=(PORTAL_X - 0.0165, intake_y, BASE + 0.02), size=(0.001, 0.04, 0.03), color=DARK)]
    for i, x in enumerate((PORTAL_X + 0.03, PORTAL_X + length * 0.6)):
        out += _arrow(f"{name}_in_{i}", x, intake_y, 1, BLUE) + _arrow(f"{name}_out_{i}", x, return_y, -1, RED)
    return out


MINE = [Box("coal_floor", pos=(0.33, 0.0, TOP + FLOOR_T / 2), size=(0.46, 0.84, FLOOR_T), color=COAL),
        Box("supply_track", pos=(0.34, 0.03, BASE + ENTRY_T / 2), size=(0.40, 0.06, ENTRY_T), color=(0.30, 0.26, 0.22))]
for n, d in DISTRICTS.items():
    MINE += _district(n, d)
STOPPINGS = {n: part(n, pos=(*xy, BASE + ENTRY_T + 0.001), mass=0.12, color=CONCRETE, friction=1.0,
                     boxes=[((0, 0, STOPPING[2] / 2), STOPPING, CONCRETE)]) for n, xy in STARTS.items()}
OBJECTS = [*MINE, *STOPPINGS.values()]

DEBUG = os.environ.get("MINE_DEBUG") == "1"
MODE = os.environ.get("MINE_MODE", "demo")
_K = {"k": 0}
SEAT_X, SEAT_Y, CLEAR, MAX_TILT, MAX_SPEED = 0.02, 0.02, 0.03, 10.0, 0.02


def _tilt_deg(state):
    return float(np.degrees(np.arccos(np.clip(state.rotation()[2, 2], -1.0, 1.0))))


def _resting(s):
    return _tilt_deg(s) < MAX_TILT and np.linalg.norm(s.lin_vel) < MAX_SPEED and abs(s.pos[2] - BASE) < 0.012


def _district_ok(objects, d):
    states = [objects[p.name] for p in STOPPINGS.values()]
    sealed = lambda x: any(_resting(s) and abs(s.pos[0] - x) <= SEAT_X and abs(s.pos[1] - d["y"]) <= SEAT_Y for s in states)
    near = lambda x: any(np.linalg.norm(s.pos[:2] - np.array([x, d["y"]])) < CLEAR for s in states)
    in_entry = any(PORTAL_X <= s.pos[0] <= d["face"] and min(abs(s.pos[1] - (d["y"] + o)) for o in (ENTRY_OFFSET, -ENTRY_OFFSET))
                   < ENTRY_W / 2 for s in states)
    *outby, last = d["crosscuts"]
    return all(sealed(x) for x in outby) and not near(last) and not in_entry


def score(objects):
    """0.5 for each district whose crosscuts outby the last one each hold a stopping, standing at rest within 2 cm of
    the crosscut's centre line, while the last crosscut stays open and no stopping stands in either entry."""
    results = {n: _district_ok(objects, d) for n, d in DISTRICTS.items()}
    total = 0.5 * sum(results.values())
    _K["k"] += 1
    if DEBUG and _K["k"] % 40 == 0:
        print(f"MINE step {_K['k']} score {total} " + " ".join(f"{n}:{int(v)}" for n, v in results.items()) + " " + " ".join(
            f"{n[-1]}:{np.round(objects[p.name].pos[:2], 3).tolist()}/{_tilt_deg(objects[p.name]):.0f}" for n, p in STOPPINGS.items()),
            flush=True)
    return float(total)


TRAVEL = TOP + 0.22


def _move(name, xy, arm):
    x0, y0 = STARTS[name]
    tx, ty = xy[0] + REACH[arm][0], xy[1] + REACH[arm][1]
    grip = BASE + ENTRY_T + 0.001 + 0.034 + 0.008
    rest = BASE + 0.034 + 0.008 + 0.003
    return [(1.3, (x0, y0, TRAVEL), FINGER_OPEN, "above the stopping", 0.0), (1.1, (x0, y0, grip), FINGER_OPEN, "descend", 0.0),
            (0.4, (x0, y0, grip), FINGER_OPEN, "settle", 0.0), (0.6, (x0, y0, grip), FINGER_CLOSED, "grip", 0.0),
            (0.9, (x0, y0, TRAVEL), FINGER_CLOSED, "lift", 0.0), (1.8, (tx, ty, TRAVEL), FINGER_CLOSED, "carry", 0.0),
            (1.2, (tx, ty, rest + 0.02), FINGER_CLOSED, "lower into the crosscut", 0.0),
            (0.6, (tx, ty, rest), FINGER_CLOSED, "set down", 0.0), (0.5, (tx, ty, rest), FINGER_OPEN, "let go", 0.0),
            (0.9, (tx, ty, TRAVEL), FINGER_OPEN, "up", 0.0)]


def _plan():
    n, s = DISTRICTS["north"], DISTRICTS["south"]
    left = [("stopping_0", (n["crosscuts"][0], n["y"]))]
    right = [(f"stopping_{i + 1}", (x, s["y"])) for i, x in enumerate(s["crosscuts"][:3])]
    if MODE == "all":
        left.append(("stopping_4", (n["crosscuts"][1], n["y"])))
    if MODE == "nearest":
        right = right[:1]
    if MODE == "entry":
        left.append(("stopping_4", (0.30, n["y"] + ENTRY_OFFSET)))
    if MODE == "one":
        right = []
    return left, right


def demo():
    """The left arm seals the north district's first crosscut, then the right arm seals the south district's first
    three, leaving the last crosscut of each district open."""
    if MODE == "none":
        return {"left": [Waypoint(1.0, (0.12, 0.42, TRAVEL), FINGER_OPEN, "idle", 0.0)]}
    plan_l, plan_r = _plan()
    left = [s for n, xy in plan_l for s in _move(n, xy, "left")] + [(1.5, (0.12, 0.42, TRAVEL), FINGER_OPEN, "move clear", 0.0)]
    right = [(sum(w[0] for w in left), (0.12, -0.42, TRAVEL), FINGER_OPEN, "wait", 0.0)]
    right += [s for n, xy in plan_r for s in _move(n, xy, "right")]
    right += [(1.5, (0.12, -0.42, TRAVEL), FINGER_OPEN, "move clear", 0.0), (2.0, (0.12, -0.42, TRAVEL), FINGER_OPEN, "settle", 0.0)]
    return {"left": [Waypoint(*s) for s in left], "right": [Waypoint(*s) for s in right]}
