"""On the station table: a flat cable runs left to right across a marked walkway (a grey band with white edge lines).
A cable cover (a yellow-orange bar, long side along the cable) lies off to one side. Left uncovered, the crossing is a trip hazard; the cover
has to go over the cable where the walkway crosses it.
"""

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

CABLE_Y = 0.0                      # the cable runs front to back (along x) on the midline
CABLE_X = (0.11, 0.51)
WALK_C = 0.36                      # the walkway band crosses it, centred 36 cm from the bases
WALK_HALF = 0.07                   # half-depth of the band, in x
WALK_Y = 0.18                      # half-width of the band, in y
COVER_LEN, COVER_W, COVER_H = 0.18, 0.05, 0.022

CABLE = Box("cable", pos=(sum(CABLE_X) / 2, CABLE_Y, TOP + 0.002), size=(CABLE_X[1] - CABLE_X[0], 0.012, 0.002),
            color=(0.05, 0.05, 0.05))
WALK = Box("walkway", pos=(WALK_C, 0.0, TOP + 0.0005), size=(2 * WALK_HALF, 2 * WALK_Y, 0.001), color=(0.42, 0.42, 0.47))
EDGES = [Box(f"edge_{i}", pos=(WALK_C + s * WALK_HALF, 0.0, TOP + 0.0015), size=(0.006, 2 * WALK_Y, 0.0015),
             color=(0.97, 0.97, 0.97)) for i, s in enumerate((-1, 1))]
COVER = Box("cover", pos=(0.18, -0.15, TOP + 0.001 + COVER_H / 2), size=(COVER_LEN, COVER_W, COVER_H), mass=0.12,
            color=(0.95, 0.55, 0.05))

OBJECTS = [WALK, *EDGES, CABLE, COVER]
DROP = (WALK_C, 0.0, COVER.pos[2] + 0.005)


def _tilt_deg(obj):
    x, y, _, _ = obj.quat_xyzw
    return float(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def _sin_yaw(obj):
    x, y, z, w = obj.quat_xyzw
    return 2 * (w * z + x * y)


def score(objects):
    """End state only. 0 unless the cover rests flat on the table, lying along the cable (within 15 degrees) and over
    it. Then 1.0 if it also spans the whole walkway depth (its centre within 2 cm of the walkway's centre line),
    0.5 if it covers part of the cable elsewhere."""
    c = objects["cover"]
    flat = c.resting() and _tilt_deg(c) < 10 and c.low[2] < TOP + 0.006
    along = abs(_sin_yaw(c)) < 0.26
    dy = abs(c.pos[1] - CABLE_Y)
    if not (flat and along and dy <= 0.019 and CABLE_X[0] <= c.pos[0] <= CABLE_X[1]):
        return 0.0
    return 1.0 if (dy <= 0.015 and abs(c.pos[0] - WALK_C) <= 0.02) else 0.5


def success(objects):
    return score(objects) == 1.0


def _demo(drop):
    travel = TOP + 0.20
    right = pick_place_waypoints(COVER.pos, drop, travel_z=travel)
    right.append(Waypoint(1.5, (0.20, -0.30, travel), FINGER_OPEN, "move clear of the camera"))
    right.append(Waypoint(2.0, (0.20, -0.30, travel), FINGER_OPEN, "hold while the cover settles"))
    return {"right": right}


def demo():
    return _demo(DROP)
