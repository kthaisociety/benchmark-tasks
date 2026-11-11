"""On the station table, a wildland fire crew's sand table: a grass fire driven away from the robot by the wind, with a
narrow heel near a gravel road, two flanks and a wide head. A hand line is anchored on the road and run up both flanks
close to the fire edge, toward the head; a line across the running head puts the crew in the fire's path.
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
SAND_T, FLAT = 0.002, 0.0006
BASE = TOP + SAND_T

GRASS, DRY, ROAD, BURNT, EMBER, FLAME, SOIL, BLUE = (0.55, 0.58, 0.30), (0.72, 0.66, 0.38), (0.58, 0.56, 0.52), \
    (0.08, 0.07, 0.07), (0.85, 0.30, 0.05), (0.98, 0.55, 0.08), (0.40, 0.28, 0.18), (0.15, 0.35, 0.85)

ROAD_X, ROAD_W = 0.20, 0.04
HEEL_X, HEAD_X, HEEL_W, HEAD_W, CAP = 0.24, 0.44, 0.06, 0.20, 0.03
SEG = (0.075, 0.022, 0.016)
SEG_XS, OFFSET = (0.255, 0.34, 0.425), 0.035
STARTS = {f"line_{side}_{i}": (x, s * 0.23) for side, s in (("left", 1), ("right", -1))
          for i, x in enumerate((0.30, 0.38, 0.46))}


def edge(x):
    """Half-width of the burnt area at x."""
    t = min(max((x - HEEL_X) / (HEAD_X - HEEL_X), 0.0), 1.0)
    return (HEEL_W + t * (HEAD_W - HEEL_W)) / 2


FLANK_YAW = math.atan((HEAD_W - HEEL_W) / 2 / (HEAD_X - HEEL_X))


def _fire():
    out, n = [], 10
    dx = (HEAD_X - HEEL_X) / n
    for i in range(n):
        x = HEEL_X + (i + 0.5) * dx
        out.append(Box(f"burnt_{i}", pos=(x, 0.0, BASE + FLAT / 2), size=(dx + 0.001, 2 * edge(x), FLAT), color=BURNT))
    for i, (w, a) in enumerate(((HEAD_W * 0.9, 0.0), (HEAD_W * 0.6, CAP * 0.6), (HEAD_W * 0.3, CAP))):
        out.append(Box(f"head_burnt_{i}", pos=(HEAD_X + a / 2, 0.0, BASE + FLAT / 2), size=(a + 0.01, w, FLAT), color=BURNT))
    flank_len = math.hypot(HEAD_X - HEEL_X, (HEAD_W - HEEL_W) / 2)
    mid = (HEEL_X + HEAD_X) / 2
    for side, s in (("left", 1), ("right", -1)):
        out.append(Box(f"flank_edge_{side}", pos=(mid, s * edge(mid), BASE + FLAT * 1.5), size=(flank_len, 0.006, FLAT),
                       color=EMBER, yaw=s * FLANK_YAW))
    out.append(Box("heel_edge", pos=(HEEL_X - 0.002, 0.0, BASE + FLAT * 1.5), size=(0.004, HEEL_W, FLAT), color=EMBER))
    for i, (dy, ddx, yaw) in enumerate(((0.0, CAP + 0.003, 0.0), (0.065, CAP * 0.55, -0.6), (-0.065, CAP * 0.55, 0.6))):
        out.append(Box(f"head_flame_{i}", pos=(HEAD_X + ddx, dy, BASE + FLAT * 1.5), size=(0.012, 0.08, FLAT * 2),
                       color=FLAME, yaw=yaw))
    return out


def _wind_arrow():
    x, y, z = 0.508, 0.0, BASE + FLAT
    return [Box("wind_shaft", pos=(x, y, z), size=(0.05, 0.014, FLAT), color=BLUE),
            *[Box(f"wind_head_{i}", pos=(x + 0.019, y + s * 0.012, z), size=(0.04, 0.012, FLAT), color=BLUE,
                  yaw=s * -math.pi / 4) for i, s in enumerate((-1, 1))]]


TABLE = [Box("sand_table", pos=(0.33, 0.0, TOP + SAND_T / 2), size=(0.44, 0.84, SAND_T), color=GRASS),
         Box("dry_grass", pos=(0.42, 0.0, BASE + FLAT / 4), size=(0.22, 0.50, FLAT / 2), color=DRY),
         Box("gravel_road", pos=(ROAD_X, 0.0, BASE + FLAT / 2), size=(ROAD_W, 0.84, FLAT), color=ROAD),
         *_fire(), *_wind_arrow()]
LINES = {n: part(n, pos=(*xy, BASE + 0.001), mass=0.05, color=SOIL, friction=1.0,
                 boxes=[((0, 0, SEG[2] / 2), SEG, SOIL)]) for n, xy in STARTS.items()}
OBJECTS = [*TABLE, *LINES.values()]

DEBUG = os.environ.get("FIRE_DEBUG") == "1"
MODE = os.environ.get("FIRE_MODE", "demo")
ANCHOR_GAP, LINK_GAP, BAND, REACH_HEAD = 0.02, 0.025, (0.015, 0.06), 0.40
MAX_TILT, MAX_SPEED = 15.0, 0.02
_K = {"k": 0}


def _tilt_deg(s):
    return float(np.degrees(np.arccos(np.clip(s.rotation()[2, 2], -1.0, 1.0))))


def _resting(s):
    return _tilt_deg(s) < MAX_TILT and np.linalg.norm(s.lin_vel) < MAX_SPEED and abs(s.pos[2] - BASE) < 0.012


def _ends(s):
    axis = s.rotation()[:2, 0] * SEG[0] / 2
    return s.pos[:2] - axis, s.pos[:2] + axis


def _in_fire_or_ahead(s):
    x, y = s.pos[:2]
    on_fire = ROAD_X + ROAD_W / 2 < x < HEAD_X + CAP and abs(y) < edge(x)
    ahead = x > HEAD_X + CAP and abs(y) < HEAD_W / 2 + 0.04
    return on_fire or ahead


def _flank(states, s):
    """0.5 for a line anchored on the road and linked up the flank to within 4 cm of the head, 0.25 for an anchored
    line that stops short, 0 for an unanchored or broken one."""
    segs = sorted((st for st in states if _resting(st) and s * st.pos[1] > 0
                   and BAND[0] <= s * st.pos[1] - edge(st.pos[0]) <= BAND[1]), key=lambda st: st.pos[0])
    if not segs or min(e[0] for e in _ends(segs[0])) > ROAD_X + ROAD_W / 2 + ANCHOR_GAP:
        return 0.0
    reach = max(e[0] for e in _ends(segs[0]))
    for st in segs[1:]:
        rear, front = sorted(e[0] for e in _ends(st))
        if rear - reach > LINK_GAP:
            break
        reach = max(reach, front)
    return 0.5 if reach >= REACH_HEAD else 0.25


def score(objects):
    """Per flank, 0.5 for hand line resting 1.5 to 6 cm outside the fire edge, anchored within 2 cm of the road and
    linked (gaps under 2.5 cm) up to within 4 cm of the head, 0.25 if it is anchored but stops short. Any line on the
    burnt area or across the head (ahead of it, within 4 cm of its width) scores 0 for the whole task."""
    states = [objects[n] for n in LINES]
    unsafe = any(_in_fire_or_ahead(st) for st in states)
    flanks = {side: _flank(states, s) for side, s in (("left", 1), ("right", -1))}
    total = 0.0 if unsafe else sum(flanks.values())
    _K["k"] += 1
    if DEBUG and _K["k"] % 40 == 0:
        print(f"FIRE step {_K['k']} score {total} unsafe {int(unsafe)} " + " ".join(f"{k}:{v}" for k, v in flanks.items())
              + " " + " ".join(f"{n[5]}{n[-1]}:{np.round(objects[n].pos[:2], 3).tolist()}/{_tilt_deg(objects[n]):.0f}"
                               for n in LINES), flush=True)
    return float(total)


TRAVEL = TOP + 0.22
AIM = {}
GRIP_YAW = 0.0


def _targets():
    out = {}
    for side, s in (("left", 1), ("right", -1)):
        for i, x in enumerate(SEG_XS):
            out[f"line_{side}_{i}"] = ((x, s * (edge(x) + OFFSET)), s * FLANK_YAW)
    if MODE == "head":
        for i, side in enumerate(("left", "right")):
            out[f"line_{side}_2"] = ((HEAD_X + 0.06, (1 if side == "left" else -1) * 0.04), math.pi / 2)
    if MODE == "unanchored":
        out.pop("line_left_0")
    return out


def _move(name, xy, yaw1):
    x0, y0 = STARTS[name]
    ax, ay = AIM.get(name, (0.0, 0.0))
    tx, ty = xy[0] + ax, xy[1] + ay
    grip = BASE + 0.001 + SEG[2] / 2 + 0.008
    rest = BASE + SEG[2] / 2 + 0.008 + 0.003
    g = GRIP_YAW
    return [(1.3, (x0, y0, TRAVEL), FINGER_OPEN, "above the strip", g), (1.1, (x0, y0, grip), FINGER_OPEN, "descend", g),
            (0.4, (x0, y0, grip), FINGER_OPEN, "settle", g), (0.6, (x0, y0, grip), FINGER_CLOSED, "grip", g),
            (1.0, (x0, y0, TRAVEL), FINGER_CLOSED, "lift", g), (2.0, (tx, ty, TRAVEL), FINGER_CLOSED, "carry", g + yaw1),
            (1.2, (tx, ty, rest + 0.02), FINGER_CLOSED, "lower to the flank", g + yaw1),
            (0.6, (tx, ty, rest), FINGER_CLOSED, "set down", g + yaw1), (0.5, (tx, ty, rest), FINGER_OPEN, "let go", g + yaw1),
            (0.9, (tx, ty, TRAVEL), FINGER_OPEN, "up", g + yaw1)]


def demo():
    """Each arm anchors its flank's line on the road, then links two more strips up the flank toward the head."""
    if MODE == "none":
        return {"left": [Waypoint(1.0, (0.12, 0.42, TRAVEL), FINGER_OPEN, "idle", 0.0)]}
    targets, plan = _targets(), {}
    for arm, s in (("left", 1), ("right", -1)):
        steps = [st for i in range(3) if f"line_{arm}_{i}" in targets
                 for st in _move(f"line_{arm}_{i}", *targets[f"line_{arm}_{i}"])]
        if MODE == "one" and arm == "right":
            steps = []
        steps += [(1.5, (0.12, s * 0.42, TRAVEL), FINGER_OPEN, "move clear", 0.0),
                  (2.0, (0.12, s * 0.42, TRAVEL), FINGER_OPEN, "settle", 0.0)]
        plan[arm] = [Waypoint(*st) for st in steps]
    return plan
