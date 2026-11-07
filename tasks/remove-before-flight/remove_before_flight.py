"""On the station table, an apron: a model light aircraft stands chocked between the arms, nose to the left. Three
remove-before-flight items are still fitted, each with a red streamer: a plug in the engine air intake on the nose, a
cover on the pitot tube on the wing, and a lock pin in the main landing gear. A navy ground-crew kit bag lies in front
of the aircraft. Before the aircraft can fly, every one of them must come off and go in the bag.
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

WHITE, STRIPE, GLASS, GREY = (0.93, 0.93, 0.95), (0.12, 0.30, 0.62), (0.10, 0.14, 0.20), (0.55, 0.57, 0.60)
RED, KNOB, BAG_C, YELLOW = (0.85, 0.08, 0.08), (0.15, 0.15, 0.17), (0.15, 0.20, 0.35), (0.95, 0.78, 0.10)

FUS_X, FUS_Z = 0.36, TOP + 0.05
AIRCRAFT = [
    Box("fuselage", pos=(FUS_X, 0.0, FUS_Z), size=(0.06, 0.40, 0.06), color=WHITE),
    Box("fuselage_stripe", pos=(FUS_X, -0.01, FUS_Z + 0.0303), size=(0.012, 0.34, 0.0006), color=STRIPE),
    Box("windscreen", pos=(FUS_X, 0.105, FUS_Z + 0.0303), size=(0.04, 0.03, 0.0006), color=GLASS),
    Box("wing", pos=(FUS_X, 0.04, FUS_Z + 0.025), size=(0.26, 0.07, 0.01), color=WHITE),
    *[Box(f"wing_tip_{i}", pos=(FUS_X + s * 0.125, 0.04, FUS_Z + 0.0303), size=(0.01, 0.07, 0.0006), color=STRIPE)
      for i, s in enumerate((-1, 1))],
    Box("tailplane", pos=(FUS_X, -0.18, FUS_Z + 0.03), size=(0.12, 0.04, 0.008), color=WHITE),
    Box("fin", pos=(FUS_X + 0.01, -0.17, FUS_Z + 0.08), size=(0.012, 0.07, 0.10), color=WHITE),
    Box("propeller", pos=(FUS_X, 0.205, FUS_Z), size=(0.14, 0.008, 0.012), color=KNOB),
    Box("spinner", pos=(FUS_X, 0.205, FUS_Z), size=(0.025, 0.018, 0.025), color=STRIPE),
    Box("nose_gear", pos=(FUS_X, 0.15, TOP + 0.01), size=(0.012, 0.012, 0.02), color=GREY),
    Box("main_gear_leg", pos=(FUS_X + 0.08, -0.06, TOP + 0.02), size=(0.03, 0.03, 0.04), color=GREY),
    Box("main_gear_leg_far", pos=(FUS_X - 0.08, -0.06, TOP + 0.02), size=(0.03, 0.03, 0.04), color=GREY),
    *[Box(f"chock_{i}", pos=(FUS_X + dx, -0.06 + dy, TOP + 0.008), size=(0.03, 0.02, 0.016), color=YELLOW)
      for i, (dx, dy) in enumerate(((0.08, 0.03), (0.08, -0.03), (-0.08, 0.03), (-0.08, -0.03)))],
]

BASE, KNOB_S, STREAMER = (0.024, 0.024, 0.014), (0.018, 0.018, 0.03), (0.07, 0.012, 0.002)
ITEMS = {
    "intake_plug": ((FUS_X, 0.17), FUS_Z + 0.03),
    "pitot_cover": ((FUS_X - 0.09, 0.04), FUS_Z + 0.03),
    "gear_pin": ((FUS_X + 0.08, -0.06), TOP + 0.04),
}


def _item(name, xy, seat_z):
    """A remove-before-flight item: a base seated in its socket, a grip knob on top, a red streamer laid to one side."""
    kz = BASE[2] / 2 + KNOB_S[2] / 2
    return part(name, pos=(*xy, seat_z + BASE[2] / 2 + 0.001), mass=0.03, color=RED, friction=1.0, boxes=[
        ((0, 0, 0), BASE, RED), ((0, 0, kz), KNOB_S, KNOB),
        ((-(STREAMER[0] / 2 + 0.012), 0, 0.002), STREAMER, RED)])


def _socket(name, xy, seat_z):
    """Four low walls around an item's base, 1.5 mm clear on each side, so it stays seated."""
    g, t, h = BASE[0] / 2 + 0.0015, 0.004, 0.008
    walls = ((g + t / 2, 0, (t, 2 * g + 2 * t, h)), (-g - t / 2, 0, (t, 2 * g + 2 * t, h)),
             (0, g + t / 2, (2 * g, t, h)), (0, -g - t / 2, (2 * g, t, h)))
    return [Box(f"{name}_socket_{i}", pos=(xy[0] + dx, xy[1] + dy, seat_z + h / 2), size=sz, color=GREY)
            for i, (dx, dy, sz) in enumerate(walls)]


PARTS = {n: _item(n, xy, z) for n, (xy, z) in ITEMS.items()}
SOCKETS = [b for n, (xy, z) in ITEMS.items() for b in _socket(n, xy, z)]

BAG_X, BAG_Y, BAG = 0.16, 0.0, (0.07, 0.22, 0.03)
KIT_BAG = [
    Box("kit_bag_floor", pos=(BAG_X, BAG_Y, TOP + 0.002), size=(BAG[0], BAG[1], 0.004), color=BAG_C),
    *[Box(f"kit_bag_wall_{i}", pos=(BAG_X + dx, BAG_Y + dy, TOP + BAG[2] / 2), size=s, color=BAG_C)
      for i, (dx, dy, s) in enumerate(((BAG[0] / 2, 0, (0.004, BAG[1], BAG[2])), (-BAG[0] / 2, 0, (0.004, BAG[1], BAG[2])),
                                        (0, BAG[1] / 2, (BAG[0], 0.004, BAG[2])), (0, -BAG[1] / 2, (BAG[0], 0.004, BAG[2]))))],
]

DRESSING = [
    Cylinder("fire_extinguisher", pos=(0.62, 0.30, TOP + 0.05), radius=0.025, height=0.10, color=RED),
    Cylinder("traffic_cone", pos=(0.62, -0.30, TOP + 0.035), radius=0.022, height=0.07, color=(0.95, 0.45, 0.10)),
    Cylinder("fuel_tester_cup", pos=(0.58, 0.05, TOP + 0.025), radius=0.018, height=0.05, color=(0.75, 0.82, 0.88)),
    Box("checklist_board", pos=(0.60, -0.12, TOP + 0.004), size=(0.09, 0.06, 0.008), color=(0.90, 0.88, 0.80)),
    *[Box(f"checklist_line_{i}", pos=(0.60 - 0.02 + i * 0.015, -0.12, TOP + 0.0083), size=(0.003, 0.045, 0.0006),
          color=GLASS) for i in range(3)],
    Box("headset_band", pos=(0.50, 0.22, TOP + 0.006), size=(0.08, 0.012, 0.012), color=KNOB),
    *[Box(f"headset_cup_{i}", pos=(0.50 + s * 0.04, 0.22, TOP + 0.012), size=(0.03, 0.035, 0.024), color=KNOB)
      for i, s in enumerate((-1, 1))],
]

OBJECTS = [*AIRCRAFT, *SOCKETS, *KIT_BAG, *DRESSING, *PARTS.values()]

DEBUG = os.environ.get("RBF_DEBUG") == "1"
_K = {"k": 0}


def _in_bag(state):
    """At rest inside the kit bag's walls, below their top."""
    dx, dy = state.pos[0] - BAG_X, state.pos[1] - BAG_Y
    inside = abs(dx) < BAG[0] / 2 - 0.004 and abs(dy) < BAG[1] / 2 - 0.004
    return bool(inside and state.pos[2] < TOP + BAG[2] + 0.02 and np.linalg.norm(state.lin_vel) < 0.02)


def score(objects):
    """A third for each remove-before-flight item lying in the kit bag. An item taken off but left anywhere else
    scores nothing: loose on an apron it is foreign-object debris."""
    total = sum(1 / 3 for n in PARTS if _in_bag(objects[n]))
    _K["k"] += 1
    if DEBUG and _K["k"] % 40 == 0:
        print(f"RBF step {_K['k']} score {total:.3f} " + " ".join(
            f"{n} {np.round(objects[n].pos, 3)} bag {_in_bag(objects[n])}" for n in PARTS), flush=True)
    return round(total, 3)


TRAVEL = TOP + 0.24
TAKE = os.environ.get("RBF_TAKE", "intake_plug,pitot_cover,gear_pin").split(",")
DROP_ON_TABLE = os.environ.get("RBF_DROP") == "table"
BAG_SLOT = {"intake_plug": 0.07, "pitot_cover": 0.03, "gear_pin": -0.06}
TABLE_SPOT = {"intake_plug": (0.16, 0.18), "pitot_cover": (0.22, 0.18), "gear_pin": (0.16, -0.18)}


def _take(name):
    (x, y), seat = ITEMS[name]
    grip = seat + BASE[2] + KNOB_S[2] / 2 + 0.001 + 0.008
    dx, dy, dz = (*TABLE_SPOT[name], TOP + 0.05) if DROP_ON_TABLE else (BAG_X, BAG_SLOT[name], TOP + 0.10)
    return [(1.5, (x, y, TRAVEL), FINGER_OPEN, f"above the {name.replace('_', ' ')}", 0.0),
            (1.5, (x, y, grip), FINGER_OPEN, "descend", 0.0),
            (0.6, (x, y, grip), FINGER_OPEN, "settle", 0.0),
            (0.8, (x, y, grip), FINGER_CLOSED, "grip the knob", 0.0),
            (1.2, (x, y, grip + 0.10), FINGER_CLOSED, "lift it out", 0.0),
            (2.0, (dx, dy, dz), FINGER_CLOSED, "over the kit bag", 0.0),
            (0.6, (dx, dy, dz), FINGER_OPEN, "drop it in", 0.0),
            (1.0, (dx, dy, TRAVEL), FINGER_OPEN, "up", 0.0)]


def demo():
    """The left arm takes the intake plug off the nose and the cover off the pitot tube, then the right arm pulls the
    gear lock pin; each goes into the kit bag."""
    left = [s for n in ("intake_plug", "pitot_cover") if n in TAKE for s in _take(n)]
    left.append((1.5, (0.12, 0.42, TRAVEL), FINGER_OPEN, "move clear", 0.0))
    wait = sum(s[0] for s in left)
    right = [(wait, (0.25, -0.30, TRAVEL), FINGER_OPEN, "wait", 0.0),
             *(_take("gear_pin") if "gear_pin" in TAKE else []),
             (1.5, (0.12, -0.42, TRAVEL), FINGER_OPEN, "move clear", 0.0)]
    return {"left": [Waypoint(*s) for s in left], "right": [Waypoint(*s) for s in right]}
