"""On the station table: a model beach at 1:40. The sea is at the near edge, then a band of dark wet sand, then a
line of seaweed and driftwood left by the last high tide, then pale dry sand. A dinghy sits on the wet sand, below
the seaweed line. Left overnight there, the next high tide floats it away; it has to go above the line.
"""

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

Y_HALF = 0.20                      # the beach runs from y = -0.20 to +0.20
SEA_X = (0.10, 0.15)
WET_X = (0.15, 0.30)
DRY_X = (0.30, 0.54)
LINE_X = 0.30                      # centre of the seaweed line
LINE_HALF = 0.012                  # the seaweed lies within 1.2 cm of it
MARGIN = 0.03                      # full credit: stern 3 cm or more above the line's upper edge
SAND_TOP = TOP + 0.001


def _slab(name, x, color):
    return Box(name, pos=((x[0] + x[1]) / 2, 0.0, TOP + 0.0005), size=(x[1] - x[0], 2 * Y_HALF, 0.001), color=color)


def _wrack():
    """Seaweed and driftwood: fixed strips along the line, alternating dark green and dark brown, each within 1.2 cm
    of it. 1.2 cm wide so the line shows from the top camera."""
    off = [0.004, -0.004, 0.005, -0.003, 0.004, -0.005, 0.003, -0.004, 0.005, -0.003]
    out = []
    for i, dx in enumerate(off):
        y = -0.18 + i * 0.04
        color = (0.05, 0.28, 0.08) if i % 2 == 0 else (0.20, 0.10, 0.04)
        out.append(Box(f"wrack_{i}", pos=(LINE_X + dx, y, SAND_TOP + 0.002), size=(0.012, 0.036, 0.004), color=color))
    return out


BOAT = Box("dinghy", pos=(0.21, 0.10, SAND_TOP + 0.026), size=(0.08, 0.05, 0.05), mass=0.15, color=(0.95, 0.95, 0.92))

OBJECTS = [
    _slab("sea", SEA_X, (0.22, 0.42, 0.50)),
    _slab("wet_sand", WET_X, (0.36, 0.26, 0.17)),
    _slab("dry_sand", DRY_X, (0.93, 0.76, 0.38)),
    *_wrack(),
    BOAT,
]

DROP = (0.42, 0.10, BOAT.pos[2] + 0.005)


def _tilt_deg(obj):
    x, y, _, _ = obj.quat_xyzw
    return float(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def score(objects):
    """End state only. 0 unless the dinghy rests upright on the sand. Then 1.0 if its lowest (seaward) point is
    3 cm or more above the seaweed line's upper edge, 0.5 if it is above that edge by less, else 0."""
    boat = objects["dinghy"]
    on_sand = abs(boat.pos[1]) < Y_HALF and boat.pos[0] < DRY_X[1] and boat.low[2] < SAND_TOP + 0.008
    if not (on_sand and boat.resting() and _tilt_deg(boat) < 15):
        return 0.0
    clear = float(boat.low[0]) - (LINE_X + LINE_HALF)      # stern to the line's upper edge
    return 1.0 if clear >= MARGIN else 0.5 if clear >= 0.0 else 0.0


def success(objects):
    return score(objects) == 1.0


def _demo(drop):
    travel = TOP + 0.20
    left = pick_place_waypoints(BOAT.pos, drop, travel_z=travel)
    left.append(Waypoint(1.5, (0.20, 0.30, travel), FINGER_OPEN, "move clear of the camera"))
    left.append(Waypoint(2.0, (0.20, 0.30, travel), FINGER_OPEN, "hold while the dinghy settles"))
    return {"left": left}


def demo():
    return _demo(DROP)
