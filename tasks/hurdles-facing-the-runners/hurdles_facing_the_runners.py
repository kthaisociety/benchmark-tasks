"""On the station table, a track before a hurdles race: two lanes of red track run away from the arms, from a start
line with starting blocks nearest the arms to a chequered finish line at the far side. A white hurdle stands on its
mark in each lane, but both were put out the wrong way round, their L-shaped feet pointing toward the finish. A
hurdle must stand with its feet toward the runners' approach, so one that is clipped tips away from the runner; set
the other way, the feet brace it and the runner falls. Each hurdle must be turned round on its mark.
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

TRACK_C, LINE_C, BLACK, WHITE, BAR_C = (0.70, 0.22, 0.17), (0.96, 0.96, 0.96), (0.10, 0.10, 0.10), \
    (0.97, 0.97, 0.97), (0.92, 0.92, 0.92)
MARK_C = (0.95, 0.80, 0.10)

TRACK_T = 0.002
SURFACE = TOP + TRACK_T
TRACK_X, LANES = (0.12, 0.62), {"left": 0.11, "right": -0.11}
START_X, FINISH_X, MARK_X = 0.16, 0.58, 0.34

TRACK = [
    Box("track", pos=((TRACK_X[0] + TRACK_X[1]) / 2, 0.0, TOP + TRACK_T / 2), size=(TRACK_X[1] - TRACK_X[0], 0.44, TRACK_T),
        color=TRACK_C),
    *[Box(f"lane_line_{i}", pos=((TRACK_X[0] + TRACK_X[1]) / 2, y, SURFACE + 0.0003), size=(TRACK_X[1] - TRACK_X[0], 0.005,
                                                                                             0.0006), color=LINE_C)
      for i, y in enumerate((-0.21, 0.0, 0.21))],
    Box("start_line", pos=(START_X, 0.0, SURFACE + 0.0003), size=(0.008, 0.42, 0.0006), color=LINE_C),
    *[Box(f"finish_check_{i}", pos=(FINISH_X, -0.20 + 0.02 + i * 0.04, SURFACE + 0.0003), size=(0.012, 0.04, 0.0006),
          color=BLACK if i % 2 else WHITE) for i in range(10)],
    *[Box(f"hurdle_mark_{lane}_{i}", pos=(MARK_X, y + s * 0.09, SURFACE + 0.0003), size=(0.02, 0.012, 0.0006),
          color=MARK_C) for lane, y in LANES.items() for i, s in enumerate((-1, 1))],
]

BLOCKS = [
    *[Box(f"starting_block_rail_{lane}", pos=(START_X - 0.035, y, SURFACE + 0.004), size=(0.07, 0.012, 0.008),
          color=(0.55, 0.57, 0.60)) for lane, y in LANES.items()],
    *[Box(f"starting_block_pedal_{lane}_{i}", pos=(START_X - 0.035 + s * 0.015, y + s * 0.02, SURFACE + 0.009),
          size=(0.015, 0.025, 0.012), color=(0.15, 0.30, 0.65)) for lane, y in LANES.items()
      for i, s in enumerate((-1, 1))],
]

DRESSING = [
    Cylinder("cone_far_left", pos=(0.60, 0.26, TOP + 0.03), radius=0.018, height=0.06, color=(0.95, 0.45, 0.10)),
    Cylinder("cone_far_right", pos=(0.60, -0.26, TOP + 0.03), radius=0.018, height=0.06, color=(0.95, 0.45, 0.10)),
    Cylinder("stopwatch", pos=(0.66, 0.10, TOP + 0.006), radius=0.025, height=0.012, color=(0.10, 0.10, 0.12)),
    Box("stopwatch_face", pos=(0.66, 0.10, TOP + 0.0123), size=(0.025, 0.02, 0.0006), color=(0.75, 0.85, 0.70)),
    Box("clipboard", pos=(0.66, -0.10, TOP + 0.004), size=(0.09, 0.065, 0.008), color=(0.55, 0.40, 0.25)),
]

W, H, FOOT, T = 0.14, 0.09, 0.08, 0.012


def _hurdle(name, xy):
    """Uprights at the hurdle's back, the bar across them, L-shaped feet running from the uprights along local +x."""
    return part(name, pos=(*xy, SURFACE + 0.001), mass=0.15, color=WHITE, friction=1.0, boxes=[
        ((0, 0, H - T / 2), (T + 0.004, W, T + 0.004), BAR_C),
        *[((0, 0.03 * s, H - T / 2 + 0.0003), (T + 0.0046, 0.03, T + 0.004), BLACK) for s in (-1, 1)],
        *[((0, s * (W / 2 - T / 2), (H - T) / 2), (T, T, H - T), BLACK) for s in (-1, 1)],
        *[((FOOT / 2 - T / 2, s * (W / 2 - T / 2), T / 2), (FOOT, T, T), BLACK) for s in (-1, 1)]])


HURDLES = {lane: _hurdle(f"hurdle_{lane}", (MARK_X, y)) for lane, y in LANES.items()}
OBJECTS = [*TRACK, *BLOCKS, *DRESSING, *HURDLES.values()]

DEBUG = os.environ.get("HURDLE_DEBUG") == "1"
_K = {"k": 0}


def _tilt_deg(state):
    return float(np.degrees(np.arccos(np.clip(state.rotation()[2, 2], -1.0, 1.0))))


def _set(state, lane):
    """Standing upright and at rest on its mark, with its feet pointing back toward the start line within 30 degrees."""
    feet = state.rotation()[:, 0]
    toward_start = -feet[0] / max(np.linalg.norm(feet[:2]), 1e-6) > math.cos(math.radians(30))
    on_mark = abs(state.pos[0] - MARK_X) < 0.03 and abs(state.pos[1] - LANES[lane]) < 0.03
    upright = _tilt_deg(state) < 10
    still = np.linalg.norm(state.lin_vel) < 0.02
    return bool(toward_start and on_mark and upright and still)


def score(objects):
    """0.5 for each hurdle standing on its mark with its feet toward the start, so it tips away from a runner."""
    total = 0.5 * sum(_set(objects[h.name], lane) for lane, h in HURDLES.items())
    _K["k"] += 1
    if DEBUG and _K["k"] % 40 == 0:
        def yaw(s):
            return math.degrees(math.atan2(s.rotation()[1, 0], s.rotation()[0, 0]))
        print(f"HURDLE step {_K['k']} score {total} " + " ".join(
            f"{lane} {np.round(objects[h.name].pos, 3)} yaw {yaw(objects[h.name]):.1f} "
            f"tilt {_tilt_deg(objects[h.name]):.1f} set {_set(objects[h.name], lane)}"
            for lane, h in HURDLES.items()), flush=True)
    return total


TRAVEL = TOP + 0.22
MODE = os.environ.get("HURDLE_MODE", "both")
SHIFT = float(os.environ.get("HURDLE_SHIFT", "0.0"))


def _quarter(x, y, yaw0, yaw1, to_x=None):
    grip = SURFACE + 0.001 + H - T / 2 + 0.008
    lift = grip + 0.03
    tx = x if to_x is None else to_x
    return [(1.5, (x, y, TRAVEL), FINGER_OPEN, "above the bar", yaw0),
            (1.5, (x, y, grip), FINGER_OPEN, "descend", yaw0), (0.6, (x, y, grip), FINGER_OPEN, "settle", yaw0),
            (0.8, (x, y, grip), FINGER_CLOSED, "grip the bar", yaw0),
            (1.0, (x, y, lift), FINGER_CLOSED, "lift", yaw0),
            (2.0, (tx, y, lift), FINGER_CLOSED, "quarter turn", yaw1),
            (1.0, (tx, y, grip + 0.002), FINGER_CLOSED, "set down", yaw1),
            (0.6, (tx, y, grip + 0.002), FINGER_OPEN, "let go", yaw1),
            (1.0, (tx, y, TRAVEL), FINGER_OPEN, "up", yaw1)]


def _turn_round(lane, half):
    y = LANES[lane]
    steps = _quarter(MARK_X, y, -math.pi / 2, 0.0)
    if not half:
        steps += _quarter(MARK_X, y, 0.0, math.pi / 2, MARK_X + SHIFT)
    return steps


def demo():
    """The left arm turns the left lane's hurdle round on its mark in two quarter turns, regripping the bar between
    them; then the right arm does the same in the right lane."""
    if MODE == "none":
        return {"left": [Waypoint(1.0, (0.12, 0.42, TRAVEL), FINGER_OPEN, "idle", 0.0)]}
    left = _turn_round("left", MODE == "half") + [(1.5, (0.12, 0.42, TRAVEL), FINGER_OPEN, "move clear", 0.0)]
    wait = sum(s[0] for s in left)
    right = [(wait, (0.25, -0.32, TRAVEL), FINGER_OPEN, "wait", 0.0)]
    if MODE != "one":
        right += _turn_round("right", MODE == "half")
    right += [(1.5, (0.12, -0.42, TRAVEL), FINGER_OPEN, "move clear", 0.0),
              (2.0, (0.12, -0.42, TRAVEL), FINGER_OPEN, "let everything settle", 0.0)]
    return {"left": [Waypoint(*s) for s in left], "right": [Waypoint(*s) for s in right]}
