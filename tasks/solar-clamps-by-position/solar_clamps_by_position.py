"""On the station table, a solar installer's job: two panels lie side by side on two mounting rails over a strip of
roof, waiting to be clamped. A parts tray beside each arm holds two end clamps and one mid clamp. An end clamp has a
single lip and goes only at a row end, over one panel's outer edge; a mid clamp has a lip each side and goes in the
gap between neighbouring panels, holding both. Each of the six clamp positions, three on each rail, needs the right
kind.
"""

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

ALU, BOLT_C, CELL, GRID, ROOF_C, TRAY_C = (0.78, 0.79, 0.81), (0.18, 0.18, 0.20), (0.08, 0.14, 0.32), \
    (0.55, 0.60, 0.70), (0.30, 0.30, 0.32), (0.85, 0.55, 0.15)

ROOF_T, RAIL_H, RAIL_W = 0.004, 0.02, 0.02
BASE = TOP + ROOF_T
RAIL_XS = (0.31, 0.41)
PANEL = (0.18, 0.14, 0.012)
GAP = 0.03
PANEL_YS = (GAP / 2 + PANEL[1] / 2, -GAP / 2 - PANEL[1] / 2)
PANEL_Z = BASE + RAIL_H + PANEL[2] / 2

ARRAY = [
    Box("roof_strip", pos=(0.36, 0.0, TOP + ROOF_T / 2), size=(0.30, 0.48, ROOF_T), color=ROOF_C),
    *[Box(f"rail_{i}", pos=(x, 0.0, BASE + RAIL_H / 2), size=(RAIL_W, 0.42, RAIL_H), color=ALU)
      for i, x in enumerate(RAIL_XS)],
    *[Box(f"panel_{i}", pos=(0.36, y, PANEL_Z), size=PANEL, color=CELL) for i, y in enumerate(PANEL_YS)],
    *[Box(f"panel_{i}_busbar_{j}", pos=(0.36 + dx, y, PANEL_Z + PANEL[2] / 2 + 0.0003), size=(0.002, PANEL[1] - 0.01,
                                                                                            0.0006), color=GRID)
      for i, y in enumerate(PANEL_YS) for j, dx in enumerate((-0.045, 0.0, 0.045))],
]

BODY = (0.03, 0.012, PANEL[2])
BOLT = (0.012, 0.012, 0.03)
LIP_T = 0.004
EDGE = GAP / 2 + PANEL[1]
END_Y = EDGE + 0.006 + BODY[1] / 2


def _clamp(name, xy, kind, lip_toward=0):
    """A clamp body that stands on the rail, a lip over one panel edge (end) or both (mid), and a bolt on top."""
    lip_w, lip_y = (GAP + 0.03, 0.0) if kind == "mid" else (0.03, lip_toward * 0.009)
    return part(name, pos=(*xy, BASE + BODY[2] / 2 + 0.001), mass=0.03, color=ALU, friction=1.0, boxes=[
        ((0, 0, 0), BODY, ALU),
        ((0, lip_y, BODY[2] / 2 + LIP_T / 2), (BODY[0], lip_w, LIP_T), ALU),
        ((0, 0, BODY[2] / 2 + LIP_T + BOLT[2] / 2), BOLT, BOLT_C)])


TRAY_Y = {"left": 0.27, "right": -0.27}
SIDE = {"left": 1, "right": -1}
STARTS = {arm: {"end_a": (0.18, TRAY_Y[arm]), "end_b": (0.25, TRAY_Y[arm]), "mid": (0.32, TRAY_Y[arm])}
          for arm in SIDE}
CLAMPS = {(arm, key): _clamp(f"{key}_clamp_{arm}", xy, "mid" if key == "mid" else "end", -SIDE[arm])
          for arm, starts in STARTS.items() for key, xy in starts.items()}
TRAYS = [Box(f"parts_tray_{arm}", pos=(0.25, y, TOP + 0.002), size=(0.22, 0.08, 0.004), color=TRAY_C)
         for arm, y in TRAY_Y.items()]

DRESSING = [
    Box("inverter", pos=(0.62, -0.20, TOP + 0.03), size=(0.08, 0.10, 0.06), color=(0.85, 0.86, 0.88)),
    Box("inverter_display", pos=(0.62, -0.20, TOP + 0.0603), size=(0.03, 0.05, 0.0006), color=(0.10, 0.30, 0.20)),
    Cylinder("cable_reel_red", pos=(0.62, 0.05, TOP + 0.015), radius=0.03, height=0.03, color=(0.75, 0.12, 0.10)),
    Cylinder("cable_reel_black", pos=(0.62, 0.15, TOP + 0.015), radius=0.03, height=0.03, color=(0.10, 0.10, 0.10)),
    Box("torque_wrench", pos=(0.60, 0.30, TOP + 0.006), size=(0.18, 0.016, 0.012), color=(0.40, 0.42, 0.45)),
]

OBJECTS = [*ARRAY, *TRAYS, *DRESSING, *CLAMPS.values()]

POSITIONS = [(x, y, kind) for x in RAIL_XS for y, kind in ((END_Y, "end"), (0.0, "mid"), (-END_Y, "end"))]
DEBUG = os.environ.get("SOLAR_DEBUG") == "1"
_K = {"k": 0}


def _tilt_deg(state):
    return float(np.degrees(np.arccos(np.clip(state.rotation()[2, 2], -1.0, 1.0))))


def _seated_at(state, x, y):
    """At rest, level, standing on the rail at this position."""
    near = abs(state.pos[0] - x) < 0.012 and abs(state.pos[1] - y) < 0.012
    on_rail = abs(state.pos[2] - (BASE + RAIL_H + BODY[2] / 2)) < 0.004
    return bool(near and on_rail and _tilt_deg(state) < 10 and np.linalg.norm(state.lin_vel) < 0.02)


def _right_clamp(objects, x, y, kind):
    return any(_seated_at(objects[c.name], x, y) for (arm, key), c in CLAMPS.items()
               if (key == "mid") == (kind == "mid"))


def score(objects):
    """A sixth for each of the six clamp positions holding the right kind of clamp, seated on its rail: end clamps at
    the row ends, mid clamps in the gap between the panels."""
    total = sum(1 / 6 for x, y, kind in POSITIONS if _right_clamp(objects, x, y, kind))
    _K["k"] += 1
    if DEBUG and _K["k"] % 40 == 0:
        print(f"SOLAR step {_K['k']} score {total:.3f} " + " ".join(
            f"{c.name} {np.round(objects[c.name].pos, 3)} tilt {_tilt_deg(objects[c.name]):.1f}"
            for c in CLAMPS.values()), flush=True)
    return round(total, 3)


TRAVEL = TOP + 0.22
MODE = os.environ.get("SOLAR_MODE", "right")


def _place(start, target):
    grip = BASE + 0.001 + BODY[2] + LIP_T + BOLT[2] / 2 + 0.008
    seat = BASE + RAIL_H + BODY[2] + LIP_T + BOLT[2] / 2 + 0.008 + 0.003
    return [(1.5, (*start, TRAVEL), FINGER_OPEN, "above the clamp", 0.0),
            (1.5, (*start, grip), FINGER_OPEN, "descend", 0.0), (0.6, (*start, grip), FINGER_OPEN, "settle", 0.0),
            (0.8, (*start, grip), FINGER_CLOSED, "grip the bolt", 0.0),
            (1.0, (*start, TRAVEL), FINGER_CLOSED, "lift", 0.0),
            (2.0, (*target, seat + 0.04), FINGER_CLOSED, "over the rail", 0.0),
            (1.2, (*target, seat), FINGER_CLOSED, "seat it", 0.0), (0.6, (*target, seat), FINGER_OPEN, "let go", 0.0),
            (1.0, (*target, TRAVEL), FINGER_OPEN, "up", 0.0)]


def _arm_plan(arm):
    s, (x_near, x_far) = SIDE[arm], RAIL_XS
    mid_rail = x_near if arm == "left" else x_far
    plan = {"end_a": (x_near, s * END_Y), "end_b": (x_far, s * END_Y), "mid": (mid_rail, 0.0)}
    if MODE == "swapped" and arm == "left":
        plan["mid"], plan["end_a"] = (x_near, s * END_Y), (mid_rail, 0.0)
    return [step for key in ("end_a", "end_b", "mid") for step in _place(STARTS[arm][key], plan[key])]


def demo():
    """The left arm puts its two end clamps at the left row end on both rails and its mid clamp in the gap on the
    near rail; then the right arm does the same on the right, with its mid clamp in the gap on the far rail."""
    if MODE == "none":
        return {"left": [Waypoint(1.0, (0.12, 0.42, TRAVEL), FINGER_OPEN, "idle", 0.0)]}
    left = _arm_plan("left") + [(1.5, (0.12, 0.42, TRAVEL), FINGER_OPEN, "move clear", 0.0)]
    wait = sum(s[0] for s in left)
    right = [(wait, (0.25, -0.32, TRAVEL), FINGER_OPEN, "wait", 0.0)]
    if MODE != "left_only":
        right += _arm_plan("right")
    right += [(1.5, (0.12, -0.42, TRAVEL), FINGER_OPEN, "move clear", 0.0),
              (2.0, (0.12, -0.42, TRAVEL), FINGER_OPEN, "let everything settle", 0.0)]
    return {"left": [Waypoint(*s) for s in left], "right": [Waypoint(*s) for s in right]}
