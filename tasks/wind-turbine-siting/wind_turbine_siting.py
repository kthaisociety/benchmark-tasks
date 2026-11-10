"""On the station table, a model of two wind farm plots under one prevailing wind, shown by a wind rose and a
windsock. Plot A is long in the direction of the wind and plot B is long across it. A turbine standing in another's
wake loses output, so turbines need about 7 rotor diameters between them along the wind and 3 across it. A row along
plot A's long side puts its turbines in each other's wake; the same habit is right for plot B.
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
FLOOR_T, PLOT_T = 0.002, 0.001
BASE = TOP + FLOOR_T
PLOT_TOP = BASE + PLOT_T

MOOR, PLOT, EDGE, WHITE, GREY, DARK, ORANGE, HEDGE, ROOF = (0.47, 0.40, 0.30), (0.78, 0.72, 0.55), (0.30, 0.26, 0.20), \
    (0.94, 0.94, 0.93), (0.55, 0.56, 0.58), (0.20, 0.20, 0.22), (0.95, 0.45, 0.10), (0.22, 0.32, 0.18), (0.55, 0.20, 0.15)

ROTOR = 0.03
CROSS, DOWN = 3 * ROTOR, 7 * ROTOR
PLOTS = {"a": ((0.22, 0.50), (0.13, 0.37)), "b": ((0.28, 0.44), (-0.40, -0.12))}
STARTS = {f"turbine_{i}": xy for i, xy in enumerate(((0.17, 0.07), (0.26, 0.07), (0.35, 0.07),
                                                     (0.17, -0.07), (0.26, -0.07), (0.35, -0.07)))}
FOOT, TOWER_W, TOWER_H = (0.055, 0.055, 0.012), 0.02, 0.105
HUB_Z, BLADE = FOOT[2] + TOWER_H + 0.006, 0.015
GRIP_Z = 0.035


def _turbine_boxes():
    boxes = [((0, 0, FOOT[2] / 2), FOOT, DARK), ((0, 0, FOOT[2] + TOWER_H / 2), (TOWER_W, TOWER_W, TOWER_H), WHITE),
             ((0, 0, HUB_Z), (0.032, 0.02, 0.018), WHITE), ((0.019, 0, HUB_Z), (0.006, 0.008, 0.008), GREY)]
    for a in (0.0, 2 * math.pi / 3, 4 * math.pi / 3):
        boxes.append(((0.023, -math.sin(a) * BLADE / 2, HUB_Z + math.cos(a) * BLADE / 2), (0.002, 0.004, BLADE), WHITE,
                      (math.cos(a / 2), math.sin(a / 2), 0.0, 0.0)))
    return boxes


def _plot(name, xs, ys):
    (x0, x1), (y0, y1) = xs, ys
    cx, cy, lx, ly = (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0
    z = BASE + PLOT_T + 0.0003
    return [Box(f"plot_{name}", pos=(cx, cy, BASE + PLOT_T / 2), size=(lx, ly, PLOT_T), color=PLOT),
            *[Box(f"plot_{name}_edge_{i}", pos=p, size=s, color=EDGE) for i, (p, s) in enumerate((
                ((cx, y0, z), (lx, 0.004, 0.0006)), ((cx, y1, z), (lx, 0.004, 0.0006)),
                ((x0, cy, z), (0.004, ly, 0.0006)), ((x1, cy, z), (0.004, ly, 0.0006))))]]


def _wind_rose(cx, cy):
    z = BASE + 0.0003
    out = [Cylinder("rose_disc", pos=(cx, cy, BASE + 0.0002), radius=0.052, height=0.0004, color=(0.90, 0.88, 0.80))]
    for i, length in enumerate((0.048, 0.022, 0.014, 0.012, 0.016, 0.012, 0.014, 0.022)):
        a = i * math.pi / 4
        out.append(Box(f"rose_petal_{i}", pos=(cx + math.cos(a) * length / 2, cy + math.sin(a) * length / 2, z + 0.0004),
                       size=(length, 0.008, 0.0006), color=(0.15, 0.30, 0.60), yaw=a))
    return out


WINDSOCK = [Cylinder("windsock_pole", pos=(0.53, 0.0, BASE + 0.05), radius=0.003, height=0.10, color=GREY),
            *[Box(f"windsock_{i}", pos=(0.522 - i * 0.012, 0.0, BASE + 0.095), size=(0.012, w, w),
                  color=ORANGE if i % 2 == 0 else WHITE) for i, w in enumerate((0.016, 0.014, 0.012, 0.010))]]
LANDSCAPE = [Box("moorland", pos=(0.33, 0.0, TOP + FLOOR_T / 2), size=(0.46, 0.84, FLOOR_T), color=MOOR),
             *_plot("a", *PLOTS["a"]), *_plot("b", *PLOTS["b"]), *_wind_rose(0.45, 0.0), *WINDSOCK,
             Box("farmhouse", pos=(0.14, 0.30, BASE + 0.015), size=(0.05, 0.04, 0.03), color=WHITE),
             Box("farmhouse_roof", pos=(0.14, 0.30, BASE + 0.033), size=(0.052, 0.042, 0.006), color=ROOF),
             Box("substation", pos=(0.14, -0.30, BASE + 0.012), size=(0.05, 0.05, 0.024), color=GREY),
             Box("hedge_a", pos=(0.36, 0.395, BASE + 0.008), size=(0.30, 0.012, 0.016), color=HEDGE),
             Box("hedge_b", pos=(0.47, -0.26, BASE + 0.008), size=(0.012, 0.28, 0.016), color=HEDGE)]
TURBINES = {n: part(n, pos=(*xy, BASE + 0.001), mass=0.1, color=WHITE, friction=1.0, boxes=_turbine_boxes())
            for n, xy in STARTS.items()}
OBJECTS = [*LANDSCAPE, *TURBINES.values()]

DEBUG = os.environ.get("WIND_DEBUG") == "1"
MODE = os.environ.get("WIND_MODE", "demo")
_K = {"k": 0}
MAX_TILT, MAX_SPEED = 10.0, 0.02


def _tilt_deg(state):
    return float(np.degrees(np.arccos(np.clip(state.rotation()[2, 2], -1.0, 1.0))))


def _standing(s):
    return _tilt_deg(s) < MAX_TILT and np.linalg.norm(s.lin_vel) < MAX_SPEED and abs(s.pos[2] - PLOT_TOP) < 0.012


def _in_plot(s, plot):
    (x0, x1), (y0, y1) = PLOTS[plot]
    return x0 <= s.pos[0] <= x1 and y0 <= s.pos[1] <= y1


def _waked(j, states):
    """True if another turbine stands upwind of j (the wind blows towards the arms, from larger x) within 7 rotor
    diameters along the wind and 3 across it, or level with j and closer than 3 diameters."""
    xj, yj = states[j].pos[:2]
    for i, s in states.items():
        if i != j and abs(s.pos[1] - yj) < CROSS and -0.01 < s.pos[0] - xj < DOWN:
            return True
    return False


def score(objects):
    """Per plot, 0.5 when three turbines stand upright and at rest inside it with none in another turbine's wake,
    0.25 when two do."""
    states = {n: objects[p.name] for n, p in TURBINES.items()}
    good = {plot: sum(_standing(s) and _in_plot(s, plot) and not _waked(n, states) for n, s in states.items())
            for plot in PLOTS}
    total = sum(0.5 if g >= 3 else 0.25 if g == 2 else 0.0 for g in good.values())
    _K["k"] += 1
    if DEBUG and _K["k"] % 40 == 0:
        print(f"WIND step {_K['k']} score {total} " + " ".join(f"{p}:{g}" for p, g in good.items()) + " " + " ".join(
            f"{n[-1]}:{np.round(s.pos[:2], 3).tolist()}/{_tilt_deg(s):.0f}" for n, s in states.items()), flush=True)
    return float(total)


TRAVEL = TOP + 0.22
TARGETS = {
    "demo": {"a": ((0.36, 0.15), (0.36, 0.255), (0.36, 0.36)), "b": ((0.36, -0.37), (0.36, -0.26), (0.36, -0.15))},
    "long": {"a": ((0.25, 0.255), (0.36, 0.255), (0.47, 0.255)), "b": ((0.36, -0.37), (0.36, -0.26), (0.36, -0.15))},
    "along": {"a": ((0.25, 0.255), (0.36, 0.255), (0.47, 0.255)), "b": ((0.30, -0.26), (0.36, -0.26), (0.42, -0.26))},
    "close": {"a": ((0.36, 0.195), (0.36, 0.255), (0.36, 0.315)), "b": ((0.36, -0.37), (0.36, -0.26), (0.36, -0.15))},
}


def _move(name, xy):
    x0, y0 = STARTS[name]
    tx, ty = xy
    grip = BASE + 0.001 + GRIP_Z + 0.008
    rest = PLOT_TOP + GRIP_Z + 0.008 + 0.003
    return [(1.3, (x0, y0, TRAVEL), FINGER_OPEN, "above the tower", 0.0), (1.2, (x0, y0, grip), FINGER_OPEN, "descend", 0.0),
            (0.4, (x0, y0, grip), FINGER_OPEN, "settle", 0.0), (0.6, (x0, y0, grip), FINGER_CLOSED, "grip the tower", 0.0),
            (1.0, (x0, y0, TRAVEL), FINGER_CLOSED, "lift", 0.0), (2.6, (tx, ty, TRAVEL), FINGER_CLOSED, "carry", 0.0),
            (0.6, (tx, ty, TRAVEL), FINGER_CLOSED, "let the swing settle", 0.0),
            (1.6, (tx, ty, rest + 0.02), FINGER_CLOSED, "lower", 0.0), (0.8, (tx, ty, rest), FINGER_CLOSED, "set down", 0.0),
            (0.5, (tx, ty, rest), FINGER_OPEN, "let go", 0.0), (1.0, (tx, ty, TRAVEL), FINGER_OPEN, "up", 0.0)]


def demo():
    """The left arm sites three turbines in a row across the wind on plot A, then the right arm does the same on plot
    B, where that row runs along the plot's long side."""
    if MODE == "none":
        return {"left": [Waypoint(1.0, (0.12, 0.42, TRAVEL), FINGER_OPEN, "idle", 0.0)]}
    t = TARGETS[MODE if MODE in TARGETS else "demo"]
    left = [s for n, xy in zip(("turbine_2", "turbine_1", "turbine_0"), t["a"]) for s in _move(n, xy)]
    left += [(1.5, (0.12, 0.42, TRAVEL), FINGER_OPEN, "move clear", 0.0)]
    right = [(sum(w[0] for w in left), (0.12, -0.42, TRAVEL), FINGER_OPEN, "wait", 0.0)]
    if MODE != "one":
        right += [s for n, xy in zip(("turbine_3", "turbine_4", "turbine_5"), t["b"]) for s in _move(n, xy)]
    right += [(1.5, (0.12, -0.42, TRAVEL), FINGER_OPEN, "move clear", 0.0), (2.0, (0.12, -0.42, TRAVEL), FINGER_OPEN, "settle", 0.0)]
    return {"left": [Waypoint(*s) for s in left], "right": [Waypoint(*s) for s in right]}
