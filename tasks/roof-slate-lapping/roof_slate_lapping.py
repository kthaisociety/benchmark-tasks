"""On the station table, a slater's job: a gable roof section between the arms, its ridge running left to right. One
face falls toward the arms to a gutter at the near eaves, the other falls away to a gutter at the far eaves. White
chalk lines mark the tail of each of two slate courses on each face. Two slates lie on a pallet beside each arm. Rain
runs from the ridge down each face to its gutter, so on each face the slate nearer the ridge must lap over the one
below it, which means laying each face from its eaves upward.
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

PITCH = math.radians(25.0)
C, S = math.cos(PITCH), math.sin(PITCH)

SARKING, RAFTER, CHALK, SLATE_C = (0.62, 0.48, 0.32), (0.45, 0.32, 0.20), (0.95, 0.95, 0.92), (0.22, 0.25, 0.30)
RIDGE_C, GUTTER_C, PALLET_C = (0.62, 0.22, 0.15), (0.30, 0.30, 0.32), (0.70, 0.58, 0.40)

SLATE = (0.11, 0.06, 0.010)
GAUGE = 0.055
COURSES = [SLATE[0] / 2, SLATE[0] / 2 + GAUGE]
FACE_LEN, DECK_W, DECK_T, EAVES_H = SLATE[0] + GAUGE + 0.01, 0.10, 0.012, 0.03
ROOF_Y = 0.0
NEAR_EAVES = 0.14
RIDGE_X = NEAR_EAVES + FACE_LEN * C
FAR_EAVES = RIDGE_X + FACE_LEN * C
FACES = {"near": (NEAR_EAVES, 1), "far": (FAR_EAVES, -1)}
RIDGE_H = EAVES_H + FACE_LEN * S


def _surface(face, s, lift=0.0):
    """World (x, z) on a face's top at s metres up the slope from its eaves, raised `lift` along its normal."""
    x0, d = FACES[face]
    return x0 + d * (s * C - lift * S), TOP + EAVES_H + s * S + lift * C


def _on_face(face, s, lift, size, color, y=0.0):
    x, z = _surface(face, s, lift)
    d = FACES[face][1]
    q = (math.cos(-d * PITCH / 2), 0.0, math.sin(-d * PITCH / 2), 0.0)
    return (x - NEAR_EAVES, y, z - TOP), size, color, q


def _post(x, h):
    return (x - NEAR_EAVES, 0.0, (h - DECK_T) / 2), (0.024, DECK_W, h - DECK_T), RAFTER


ROOF = part("roof", pos=(NEAR_EAVES, ROOF_Y, TOP), mass=25.0, color=SARKING, friction=1.0, boxes=[
    *[_on_face(f, FACE_LEN / 2, -DECK_T / 2, (FACE_LEN, DECK_W, DECK_T), SARKING) for f in FACES],
    *[_on_face(f, FACE_LEN / 2, -DECK_T - 0.01, (FACE_LEN, 0.015, 0.02), RAFTER, y)
      for f in FACES for y in (-0.035, 0.035)],
    _post(NEAR_EAVES + 0.012, EAVES_H), _post(FAR_EAVES - 0.012, EAVES_H), _post(RIDGE_X, RIDGE_H),
    ((RIDGE_X - NEAR_EAVES, 0.0, RIDGE_H + 0.004), (0.035, DECK_W, 0.018), RIDGE_C),
    *[_on_face(f, c - SLATE[0] / 2 + 0.002, 0.0003, (0.004, DECK_W - 0.01, 0.0006), CHALK)
      for f in FACES for c in COURSES],
])

GUTTERS = [
    *[Box(f"gutter_{f}", pos=(x0 - d * 0.012, ROOF_Y, TOP + EAVES_H - 0.012), size=(0.02, DECK_W + 0.04, 0.014),
          color=GUTTER_C) for f, (x0, d) in FACES.items()],
    *[Cylinder(f"downpipe_{f}", pos=(x0 - d * 0.012, ROOF_Y - DECK_W / 2 - 0.02, TOP + 0.012), radius=0.008,
               height=0.024, color=GUTTER_C) for f, (x0, d) in FACES.items()],
]

PALLET_H = 0.012
SLATE_START = {"left": [(0.22, 0.17), (0.36, 0.17)], "right": [(0.22, -0.17), (0.36, -0.17)]}
PALLETS = [Box(f"slate_pallet_{arm}", pos=(0.29, y, TOP + PALLET_H / 2), size=(0.30, 0.09, PALLET_H), color=PALLET_C)
           for arm, y in (("left", 0.17), ("right", -0.17))]
SLATES = [part(f"slate_{arm}_{i}", pos=(*xy, TOP + PALLET_H + SLATE[2] / 2 + 0.001), mass=0.10, color=SLATE_C,
               friction=1.0, boxes=[((0, 0, 0), SLATE, SLATE_C)])
          for arm, starts in SLATE_START.items() for i, xy in enumerate(starts)]

DRESSING = [
    Box("slaters_hammer_handle", pos=(0.52, -0.30, TOP + 0.008), size=(0.16, 0.022, 0.016), color=(0.55, 0.38, 0.22)),
    Box("slaters_hammer_head", pos=(0.59, -0.30, TOP + 0.012), size=(0.03, 0.09, 0.024), color=(0.35, 0.36, 0.38)),
    Box("nail_box", pos=(0.62, -0.14, TOP + 0.02), size=(0.07, 0.05, 0.04), color=(0.80, 0.60, 0.15)),
    Box("spare_ridge_tile", pos=(0.62, 0.14, TOP + 0.015), size=(0.12, 0.05, 0.03), color=RIDGE_C),
    *[Box(f"ladder_rail_{i}", pos=(0.62, y, TOP + 0.01), size=(0.30, 0.02, 0.02), color=(0.75, 0.75, 0.78))
      for i, y in enumerate((0.28, 0.38))],
    *[Box(f"ladder_rung_{i}", pos=(0.50 + i * 0.08, 0.33, TOP + 0.01), size=(0.015, 0.10, 0.012),
          color=(0.75, 0.75, 0.78)) for i in range(4)],
]

OBJECTS = [ROOF, *GUTTERS, *PALLETS, *DRESSING, *SLATES]

DEBUG = os.environ.get("SLATE_DEBUG") == "1"
_K = {"k": 0}


def _tilt_deg(state):
    return float(np.degrees(np.arccos(np.clip(state.rotation()[2, 2], -1.0, 1.0))))


def _place(state):
    """The (face, course) a slate is laid on: at rest on a face, long side up the slope, centre on a course line."""
    face = "near" if state.pos[0] < RIDGE_X else "far"
    x0, d = FACES[face]
    s = d * (state.pos[0] - x0) * C + (state.pos[2] - TOP - EAVES_H) * S
    along = abs(state.rotation()[0, 0] * d * C + state.rotation()[2, 0] * S)
    if np.linalg.norm(state.lin_vel) > 0.02 or abs(state.pos[1] - ROOF_Y) > 0.02 or along < math.cos(math.radians(20)):
        return None
    if not 5.0 < _tilt_deg(state) < 45.0:
        return None
    for i, c in enumerate(COURSES):
        if abs(s - c) < 0.02 + (SLATE[2] * S if i else 0.0):
            return face, i
    return None


def _laps_over(lower, upper):
    """The upper slate's tail (its lower end) lies above the lower slate's mid-plane."""
    r = upper.rotation()
    ends = [r @ np.array([sx * SLATE[0] / 2, 0, -SLATE[2] / 2]) + upper.pos for sx in (-1, 1)]
    tail = min(ends, key=lambda p: p[2])
    return float(lower.to_local(tail)[2]) > 0.0


def score(objects):
    """0.5 for each roof face whose two courses each hold a slate, with the upper slate lapping over the lower one;
    0 for a face laid the wrong way round or not fully laid."""
    slates = [objects[s.name] for s in SLATES]
    laid = {}
    for st in slates:
        p = _place(st)
        if p is not None and p not in laid:
            laid[p] = st
    total = sum(0.5 for f in FACES if (f, 0) in laid and (f, 1) in laid and _laps_over(laid[f, 0], laid[f, 1]))
    _K["k"] += 1
    if DEBUG and _K["k"] % 40 == 0:
        print(f"SLATE step {_K['k']} score {total} " + " ".join(
            f"{n.name[6:]} {np.round(st.pos, 3)} tilt {_tilt_deg(st):.1f} at {_place(st)}"
            for n, st in zip(SLATES, slates)), flush=True)
    return total


TRAVEL = TOP + 0.22
LAY_ORDER = {"near": os.environ.get("SLATE_NEAR", "01"), "far": os.environ.get("SLATE_FAR", "01")}


def _lay(start, face, course, over_slate):
    sx, sy = start
    y = ROOF_Y + (0.012 if face == "far" and course == 0 else 0.0)
    grip = TOP + PALLET_H + SLATE[2] / 2 + 0.008
    d = FACES[face][1]
    head_x, head_z = _surface(face, COURSES[course] + SLATE[0] / 2, SLATE[2] if over_slate else 0.0)
    x, z = head_x - d * SLATE[0] / 2, head_z + 0.004 + SLATE[2] / 2 + 0.008
    return [(1.5, (sx, sy, TRAVEL), FINGER_OPEN, "above the slate", 0.0),
            (1.5, (sx, sy, grip), FINGER_OPEN, "descend", 0.0),
            (0.6, (sx, sy, grip), FINGER_OPEN, "settle", 0.0),
            (0.8, (sx, sy, grip), FINGER_CLOSED, "grip", 0.0),
            (1.0, (sx, sy, grip + 0.10), FINGER_CLOSED, "lift", 0.0),
            (2.0, (x, y, z + 0.06), FINGER_CLOSED, f"over the {face} face, course {course + 1}", 0.0),
            (1.2, (x, y, z), FINGER_CLOSED, "rest the head on the roof", 0.0),
            (0.8, (x, y, z), FINGER_OPEN, "let it lie down", 0.0),
            (1.2, (x, y, TRAVEL), FINGER_OPEN, "up", 0.0)]


def _face(arm, face):
    steps, laid = [], set()
    for k, ch in enumerate(LAY_ORDER[face]):
        c = int(ch)
        steps += _lay(SLATE_START[arm][k], face, c, c + 1 in laid)
        laid.add(c)
    return steps


def demo():
    """The left arm lays the near face, eaves course first; then the right arm lays the far face, eaves course (by
    the far gutter) first, so on both faces the slate nearer the ridge laps over the one below."""
    left = _face("left", "near") + [(1.5, (0.12, 0.42, TRAVEL), FINGER_OPEN, "move clear", 0.0)]
    wait = sum(s[0] for s in left)
    right = [(wait, (0.22, -0.30, TRAVEL), FINGER_OPEN, "wait", 0.0), *_face("right", "far"),
             (1.5, (0.12, -0.42, TRAVEL), FINGER_OPEN, "move clear", 0.0)]
    return {"left": [Waypoint(*s) for s in left], "right": [Waypoint(*s) for s in right]}
