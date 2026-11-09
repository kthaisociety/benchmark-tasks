"""On the station table, a film studio: two pairs of actors stand on their marks, each pair filmed by a dark camera on
one side of them. A yellow camera stands on the wrong side of each pair, ready for the reverse shot. The camera has to
stay on the same side of the imaginary line through the two actors as the first camera, the 180-degree rule, or the
actors appear to swap sides of the screen when the shots are cut together.
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
from yam_arena.task_file import TABLE_TOP, Box, Cylinder, Sphere  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
FLOOR_T = 0.002
BASE = TOP + FLOOR_T

FLOOR, TAPE, SKIN, DARK, STEEL, YELLOW, LENS = (0.16, 0.16, 0.19), (0.95, 0.85, 0.20), (0.85, 0.68, 0.55), \
    (0.12, 0.12, 0.14), (0.45, 0.46, 0.50), (0.95, 0.75, 0.10), (0.05, 0.05, 0.07)

SETS = {
    "left": {"actors": ((0.30, 0.14), (0.38, 0.26)), "costumes": ((0.15, 0.35, 0.75), (0.75, 0.15, 0.15)),
             "camera_a": ((0.22, 0.13), 0.53), "start": (0.44, 0.16), "target": (0.30, 0.28), "wrong": (0.44, 0.24)},
    "right": {"actors": ((0.28, -0.12), (0.42, -0.22)), "costumes": ((0.20, 0.60, 0.30), (0.55, 0.25, 0.65)),
              "camera_a": ((0.46, -0.12), 3.60), "start": (0.25, -0.25), "target": (0.37, -0.06), "wrong": (0.30, -0.30)},
}

FOOT, MAST, BODY_Z, BODY = (0.07, 0.07, 0.012), (0.014, 0.014, 0.03), 0.0595, (0.07, 0.04, 0.035)
LENS_S = (0.03, 0.022, 0.022)
BAR, BAR_Z, POST_H = (0.07, 0.016, 0.016), 0.107, 0.03


def _camera_boxes(color):
    return [((0, 0, FOOT[2] / 2), FOOT, DARK), ((0, 0, FOOT[2] + MAST[2] / 2), MAST, STEEL), ((0, 0, BODY_Z), BODY, color),
            ((BODY[0] / 2 + LENS_S[0] / 2, 0, BODY_Z), LENS_S, LENS),
            *[((s * 0.03, 0, BODY_Z + BODY[2] / 2 + POST_H / 2), (0.012, 0.012, POST_H), STEEL) for s in (-1, 1)],
            ((0, 0, BAR_Z), BAR, STEEL)]


def _actor(name, xy, costume):
    x, y = xy
    return [Cylinder(f"{name}_body", pos=(x, y, BASE + 0.035), radius=0.016, height=0.07, color=costume),
            Sphere(f"{name}_head", pos=(x, y, BASE + 0.082), radius=0.013, color=SKIN),
            Box(f"{name}_mark_bar", pos=(x, y, BASE + 0.0003), size=(0.05, 0.006, 0.0006), color=TAPE),
            Box(f"{name}_mark_stem", pos=(x, y, BASE + 0.0003), size=(0.006, 0.05, 0.0006), color=TAPE)]


def _fixed_camera(name, xy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [Box(f"{name}_{i}", pos=(xy[0] + c * o[0] - s * o[1], xy[1] + s * o[0] + c * o[1], BASE + o[2]), size=size,
                color=color, yaw=yaw) for i, (o, size, color) in enumerate(_camera_boxes(DARK))]


STUDIO = [Box("studio_floor", pos=(0.33, 0.0, TOP + FLOOR_T / 2), size=(0.46, 0.84, FLOOR_T), color=FLOOR)]
for arm, s in SETS.items():
    STUDIO += _actor(f"actor_{arm}_one", s["actors"][0], s["costumes"][0])
    STUDIO += _actor(f"actor_{arm}_two", s["actors"][1], s["costumes"][1])
    STUDIO += _fixed_camera(f"first_camera_{arm}", *s["camera_a"])

DRESSING = [
    Box("director_chair_seat", pos=(0.60, 0.26, TOP + 0.04), size=(0.07, 0.07, 0.01), color=(0.12, 0.12, 0.14)),
    Box("director_chair_back", pos=(0.635, 0.26, TOP + 0.07), size=(0.008, 0.07, 0.04), color=(0.12, 0.12, 0.14)),
    Cylinder("light_stand", pos=(0.62, -0.28, TOP + 0.06), radius=0.008, height=0.12, color=STEEL),
    Box("light_head", pos=(0.62, -0.28, TOP + 0.13), size=(0.06, 0.06, 0.04), color=(0.90, 0.90, 0.85)),
    Box("clapperboard", pos=(0.60, 0.02, TOP + 0.004), size=(0.09, 0.07, 0.008), color=(0.08, 0.08, 0.09)),
    *[Box(f"clapper_stripe_{i}", pos=(0.565 + i * 0.024, 0.02, TOP + 0.0083), size=(0.012, 0.07, 0.0006), color=(0.95, 0.95, 0.95))
      for i in range(4)],
]

CAMERAS = {arm: part(f"camera_{arm}", pos=(*s["start"], BASE + 0.001), mass=0.25, color=YELLOW, friction=1.0,
                     boxes=_camera_boxes(YELLOW)) for arm, s in SETS.items()}
OBJECTS = [*STUDIO, *DRESSING, *CAMERAS.values()]

DEBUG = os.environ.get("FILM_DEBUG") == "1"
_K = {"k": 0}
MIN_OFF_LINE, MAX_FROM_ACTORS, AIM_DEG = 0.04, 0.18, 30


def _side(xy, actors):
    """Signed distance of a point from the line through the two actors."""
    (px, py), (qx, qy) = actors
    d = np.array([qx - px, qy - py])
    return float((d[0] * (xy[1] - py) - d[1] * (xy[0] - px)) / np.linalg.norm(d))


def _tilt_deg(state):
    return float(np.degrees(np.arccos(np.clip(state.rotation()[2, 2], -1.0, 1.0))))


def _aim_error(state, actors):
    mid = np.mean(actors, axis=0)
    lens, to_mid = state.rotation()[:2, 0], mid - state.pos[:2]
    cosine = lens @ to_mid / max(np.linalg.norm(lens) * np.linalg.norm(to_mid), 1e-6)
    return float(np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0))))


def _set_score(state, s):
    """0.25 for standing at rest on the first camera's side of the line, 0.25 more for aiming at the actors from there."""
    side, first = _side(state.pos[:2], s["actors"]), _side(s["camera_a"][0], s["actors"])
    near = np.linalg.norm(state.pos[:2] - np.mean(s["actors"], axis=0)) <= MAX_FROM_ACTORS
    standing = _tilt_deg(state) < 10 and np.linalg.norm(state.lin_vel) < 0.02 and abs(state.pos[2] - BASE) < 0.012
    if not (standing and near and side * first > 0 and abs(side) >= MIN_OFF_LINE):
        return 0.0
    return 0.5 if _aim_error(state, s["actors"]) <= AIM_DEG else 0.25


def score(objects):
    """For each pair of actors, 0.25 for the yellow camera standing at rest within 18 cm of them, on the first camera's
    side of the line through them and at least 4 cm off it, and 0.25 more if its lens points at them within 30 degrees."""
    total = sum(_set_score(objects[CAMERAS[arm].name], s) for arm, s in SETS.items())
    _K["k"] += 1
    if DEBUG and _K["k"] % 40 == 0:
        print(f"FILM step {_K['k']} score {total} " + " ".join(
            f"{arm} {np.round(objects[CAMERAS[arm].name].pos, 3)} side {_side(objects[CAMERAS[arm].name].pos[:2], s['actors']):.3f} "
            f"aim {_aim_error(objects[CAMERAS[arm].name], s['actors']):.0f} tilt {_tilt_deg(objects[CAMERAS[arm].name]):.1f}"
            for arm, s in SETS.items()), flush=True)
    return float(total)


TRAVEL = TOP + 0.24
MODE = os.environ.get("FILM_MODE", "demo")


def _aim_yaw(target, actors):
    mid = np.mean(actors, axis=0)
    return float(math.atan2(mid[1] - target[1], mid[0] - target[0]))


def _move(arm):
    s = SETS[arm]
    x0, y0 = s["start"]
    wrong = MODE == "wrong"
    tx, ty = s["wrong"] if wrong else s["target"]
    yaw = 0.0 if wrong or MODE == "no_aim" else _aim_yaw((tx, ty), s["actors"])
    grip = BASE + 0.001 + BAR_Z + 0.008
    rest = BASE + BAR_Z + 0.008 + 0.003
    return [(1.5, (x0, y0, TRAVEL), FINGER_OPEN, "above the camera", 0.0),
            (1.5, (x0, y0, grip), FINGER_OPEN, "descend", 0.0), (0.6, (x0, y0, grip), FINGER_OPEN, "settle", 0.0),
            (0.8, (x0, y0, grip), FINGER_CLOSED, "grip the top handle", 0.0),
            (1.2, (x0, y0, TRAVEL), FINGER_CLOSED, "lift", 0.0),
            (2.5, (tx, ty, TRAVEL), FINGER_CLOSED, "carry and aim", yaw),
            (1.5, (tx, ty, rest + 0.02), FINGER_CLOSED, "lower", yaw), (0.8, (tx, ty, rest), FINGER_CLOSED, "set down", yaw),
            (0.6, (tx, ty, rest), FINGER_OPEN, "let go", yaw), (1.2, (tx, ty, TRAVEL), FINGER_OPEN, "up", yaw)]


def demo():
    """Each arm carries its yellow camera across to the first camera's side of the line through its actors and aims it
    at them, the left arm first and then the right."""
    if MODE == "none":
        return {"left": [Waypoint(1.0, (0.12, 0.42, TRAVEL), FINGER_OPEN, "idle", 0.0)]}
    left = _move("left") + [(1.5, (0.12, 0.42, TRAVEL), FINGER_OPEN, "move clear", 0.0)]
    wait = sum(s[0] for s in left)
    right = [(wait, (0.20, -0.42, TRAVEL), FINGER_OPEN, "wait", 0.0)]
    if MODE != "one":
        right += _move("right")
    right += [(1.5, (0.12, -0.42, TRAVEL), FINGER_OPEN, "move clear", 0.0),
              (2.0, (0.12, -0.42, TRAVEL), FINGER_OPEN, "let everything settle", 0.0)]
    return {"left": [Waypoint(*s) for s in left], "right": [Waypoint(*s) for s in right]}
