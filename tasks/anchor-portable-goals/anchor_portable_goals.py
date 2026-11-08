"""On the station table, a school sports field: two portable football goals stand on the grass, one with its mouth
toward the arms and one with its mouth away. A portable goal tips forward onto whoever hangs from it unless ballast
holds down the rear ground bar, behind the line it would tip over. Two sandbags lie by the arms, one for each goal.
"""

import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from partkit import part  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder, Sphere  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
TURF_T = 0.002
BASE = TOP + TURF_T

GRASS, LINE, WHITE, BAG_C, STRAP_C = (0.25, 0.50, 0.22), (0.95, 0.95, 0.95), (0.96, 0.96, 0.96), (0.72, 0.62, 0.42), \
    (0.25, 0.20, 0.15)

BAR_W, BAR_H, DEPTH, WIDTH, POST, TALL = 0.03, 0.012, 0.09, 0.16, 0.012, 0.075
LEFT_Y, RIGHT_Y, NEAR_X, FAR_X = 0.17, -0.17, 0.27, 0.36
REAR_X = {"left": FAR_X, "right": NEAR_X}
FRONT_X = {"left": NEAR_X, "right": FAR_X}
GOAL_Y = {"left": LEFT_Y, "right": RIGHT_Y}


def _goal(name, y, front_x, rear_x):
    mid_x = (front_x + rear_x) / 2
    return [
        Box(f"{name}_rear_bar", pos=(rear_x, y, BASE + BAR_H / 2), size=(BAR_W, WIDTH, BAR_H), color=WHITE),
        *[Box(f"{name}_side_bar_{i}", pos=(mid_x, y + s * (WIDTH / 2 - POST / 2), BASE + BAR_H / 2),
              size=(abs(rear_x - front_x), POST, BAR_H), color=WHITE) for i, s in enumerate((-1, 1))],
        *[Box(f"{name}_post_{i}", pos=(front_x, y + s * (WIDTH / 2 - POST / 2), BASE + TALL / 2), size=(POST, POST, TALL),
              color=WHITE) for i, s in enumerate((-1, 1))],
        Box(f"{name}_crossbar", pos=(front_x, y, BASE + TALL + POST / 2), size=(POST, WIDTH, POST), color=WHITE),
    ]


def _area(name, y, front_x, toward):
    """Penalty box lines on the grass in front of a goal's mouth, so which way it faces reads from above."""
    far = front_x + toward * 0.075
    return [Box(f"{name}_box_front", pos=(far, y, BASE + 0.0003), size=(0.005, WIDTH + 0.04, 0.0006), color=LINE),
            *[Box(f"{name}_box_side_{i}", pos=((front_x + far) / 2, y + s * (WIDTH / 2 + 0.02), BASE + 0.0003),
                  size=(0.075, 0.005, 0.0006), color=LINE) for i, s in enumerate((-1, 1))],
            Cylinder(f"{name}_spot", pos=(front_x + toward * 0.05, y, BASE + 0.0003), radius=0.006, height=0.0006,
                     color=LINE)]


PITCH = [
    Box("grass", pos=(0.33, 0.0, TOP + TURF_T / 2), size=(0.46, 0.84, TURF_T), color=GRASS),
    *_goal("goal_left", LEFT_Y, NEAR_X, FAR_X), *_goal("goal_right", RIGHT_Y, FAR_X, NEAR_X),
    *_area("goal_left", LEFT_Y, NEAR_X, -1), *_area("goal_right", RIGHT_Y, FAR_X, 1),
]
DRESSING = [
    Sphere("football", pos=(0.58, 0.05, BASE + 0.022), radius=0.022, color=(0.95, 0.95, 0.95)),
    *[Cylinder(f"training_cone_{i}", pos=(0.60, y, BASE + 0.02), radius=0.014, height=0.04, color=(0.95, 0.45, 0.10))
      for i, y in enumerate((-0.32, -0.20, 0.30))],
    Box("water_crate", pos=(0.62, -0.05, BASE + 0.02), size=(0.08, 0.06, 0.04), color=(0.15, 0.30, 0.65)),
]

BODY, KNOB = (0.04, 0.09, 0.02), (0.018, 0.018, 0.03)
START = {"left": (0.18, 0.36), "right": (0.18, -0.36)}
BAGS = {arm: part(f"sandbag_{arm}", pos=(*xy, BASE + BODY[2] / 2 + 0.001), mass=0.15, color=BAG_C, friction=1.0, boxes=[
    ((0, 0, 0), BODY, BAG_C), ((0, 0, BODY[2] / 2 + KNOB[2] / 2), KNOB, STRAP_C)]) for arm, xy in START.items()}

OBJECTS = [*PITCH, *DRESSING, *BAGS.values()]
DEBUG = os.environ.get("GOAL_DEBUG") == "1"
_K = {"k": 0}


def _tilt_deg(state):
    return float(np.degrees(np.arccos(np.clip(state.rotation()[2, 2], -1.0, 1.0))))


def _on_rear_bar(state, goal):
    on_bar = abs(state.pos[0] - REAR_X[goal]) < BAR_W / 2 and abs(state.pos[1] - GOAL_Y[goal]) < WIDTH / 2
    resting = abs(state.pos[2] - (BASE + BAR_H + BODY[2] / 2)) < 0.005 and _tilt_deg(state) < 10
    return bool(on_bar and resting and np.linalg.norm(state.lin_vel) < 0.02)


def score(objects):
    """0.5 for each goal with a sandbag lying at rest on its rear ground bar, behind the line it would tip over."""
    bags = [objects[b.name] for b in BAGS.values()]
    total = 0.5 * sum(any(_on_rear_bar(b, g) for b in bags) for g in REAR_X)
    _K["k"] += 1
    if DEBUG and _K["k"] % 40 == 0:
        print(f"GOAL step {_K['k']} score {total} " + " ".join(
            f"{b.name} {np.round(objects[b.name].pos, 3)} tilt {_tilt_deg(objects[b.name]):.1f}"
            for b in BAGS.values()), flush=True)
    return total


TRAVEL = TOP + 0.22
MODE = os.environ.get("GOAL_MODE", "demo")
AIM_FAR = 0.012


def _lay(start, target, on_bar):
    grip = BASE + 0.001 + BODY[2] + KNOB[2] / 2 + 0.008
    rest = BASE + (BAR_H if on_bar else 0.0) + BODY[2] + KNOB[2] / 2 + 0.008 + 0.003
    return [(1.5, (*start, TRAVEL), FINGER_OPEN, "above the sandbag", 0.0),
            (1.5, (*start, grip), FINGER_OPEN, "descend", 0.0), (0.6, (*start, grip), FINGER_OPEN, "settle", 0.0),
            (0.8, (*start, grip), FINGER_CLOSED, "grip the strap", 0.0),
            (1.0, (*start, TRAVEL), FINGER_CLOSED, "lift", 0.0),
            (2.0, (*target, rest + 0.05), FINGER_CLOSED, "over the rear bar", 0.0),
            (1.2, (*target, rest), FINGER_CLOSED, "lower", 0.0), (0.6, (*target, rest), FINGER_OPEN, "let go", 0.0),
            (1.0, (*target, TRAVEL), FINGER_OPEN, "up", 0.0)]


def _target(arm):
    if MODE == "front":
        return (NEAR_X - 0.05, LEFT_Y) if arm == "left" else (FAR_X + 0.05, RIGHT_Y)
    return (FAR_X + AIM_FAR, LEFT_Y) if arm == "left" else (NEAR_X, RIGHT_Y)


def demo():
    """The left arm lays its sandbag on the rear bar of the goal facing the arms, the far bar; then the right arm lays
    its sandbag on the rear bar of the goal facing away, the near bar."""
    if MODE == "none":
        return {"left": [Waypoint(1.0, (0.12, 0.42, TRAVEL), FINGER_OPEN, "idle", 0.0)]}
    left = _lay(START["left"], _target("left"), MODE != "front")
    left.append((1.5, (0.12, 0.42, TRAVEL), FINGER_OPEN, "move clear", 0.0))
    wait = sum(s[0] for s in left)
    right = [(wait, (0.20, -0.42, TRAVEL), FINGER_OPEN, "wait", 0.0)]
    if MODE != "one":
        right += _lay(START["right"], _target("right"), MODE != "front")
    right += [(1.5, (0.12, -0.42, TRAVEL), FINGER_OPEN, "move clear", 0.0),
              (2.0, (0.12, -0.42, TRAVEL), FINGER_OPEN, "let everything settle", 0.0)]
    return {"left": [Waypoint(*s) for s in left], "right": [Waypoint(*s) for s in right]}
