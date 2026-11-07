"""On the station table, packing before a flight: an open red suitcase with a checked-baggage tag on its handle and
folded clothes in one half, and a small green cabin backpack. A power bank, a large bottle of shampoo and a folded sweater lie
between them; a passport, a boarding pass and sunglasses lie to one side. Airline rules send the power bank to the
cabin (a lithium battery fire in the hold cannot be reached) and the large shampoo to the hold (over 100 ml is not
allowed through security in the cabin).
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

CASE_C, PACK_C, STRAP_C, TAG_C = (0.62, 0.10, 0.12), (0.16, 0.36, 0.26), (0.10, 0.12, 0.10), (0.95, 0.55, 0.10)
CLOTHES = [(0.85, 0.85, 0.80), (0.35, 0.45, 0.65), (0.75, 0.60, 0.45)]
WALL_T = 0.006


def _open_box(name, xy, inner, h, color):
    x, y = xy
    walls = ((inner[0] / 2 + WALL_T / 2, 0, (WALL_T, inner[1] + 2 * WALL_T, h)),
             (-inner[0] / 2 - WALL_T / 2, 0, (WALL_T, inner[1] + 2 * WALL_T, h)),
             (0, inner[1] / 2 + WALL_T / 2, (inner[0], WALL_T, h)), (0, -inner[1] / 2 - WALL_T / 2, (inner[0], WALL_T, h)))
    return [Box(f"{name}_floor", pos=(x, y, TOP + 0.003), size=(inner[0], inner[1], 0.006), color=color),
            *[Box(f"{name}_wall_{i}", pos=(x + dx, y + dy, TOP + h / 2), size=s, color=color)
              for i, (dx, dy, s) in enumerate(walls)]]


SUITCASE_XY, SUITCASE_IN, SUITCASE_H = (0.45, 0.0), (0.16, 0.24), 0.06
BACKPACK_XY, BACKPACK_IN, BACKPACK_H = (0.18, 0.0), (0.09, 0.12), 0.05
SX, SY = SUITCASE_XY
SUITCASE = [
    *_open_box("suitcase", SUITCASE_XY, SUITCASE_IN, SUITCASE_H, CASE_C),
    Box("suitcase_lid", pos=(SX + SUITCASE_IN[0] / 2 + 0.05, SY, TOP + 0.006), size=(0.09, SUITCASE_IN[1] + 0.012, 0.012),
        color=CASE_C),
    Box("suitcase_lid_lining", pos=(SX + SUITCASE_IN[0] / 2 + 0.05, SY, TOP + 0.0123), size=(0.08, 0.23, 0.0006),
        color=(0.25, 0.25, 0.28)),
    Box("suitcase_handle", pos=(SX, SY - SUITCASE_IN[1] / 2 - 0.02, TOP + 0.03), size=(0.06, 0.012, 0.012),
        color=STRAP_C),
    Box("checked_baggage_tag", pos=(SX + 0.02, SY - SUITCASE_IN[1] / 2 - 0.035, TOP + 0.002),
        size=(0.05, 0.02, 0.002), color=TAG_C),
    *[Box(f"folded_clothes_{i}", pos=(SX, SY + 0.07, TOP + 0.009 + i * 0.012), size=(0.14, 0.09, 0.012), color=c)
      for i, c in enumerate(CLOTHES)],
]
BACKPACK = [
    *_open_box("backpack", BACKPACK_XY, BACKPACK_IN, BACKPACK_H, PACK_C),
    *[Box(f"backpack_strap_{i}", pos=(BACKPACK_XY[0] - BACKPACK_IN[0] / 2 - 0.02, y, TOP + 0.004),
          size=(0.04, 0.012, 0.008), color=STRAP_C) for i, y in enumerate((-0.035, 0.035))],
]

PB = (0.07, 0.04, 0.046)
POWER_BANK_XY = (0.30, 0.20)
POWER_BANK = part("power_bank", pos=(*POWER_BANK_XY, TOP + PB[2] / 2 + 0.001), mass=0.25, color=(0.08, 0.08, 0.09),
                  friction=1.0, boxes=[((0, 0, 0), PB, (0.08, 0.08, 0.09)),
                                       ((0.02, 0, PB[2] / 2 + 0.0004), (0.02, 0.006, 0.0008), (0.2, 0.6, 1.0)),
                                       ((-PB[0] / 2 - 0.006, 0, -0.016), (0.012, 0.006, 0.006), (0.85, 0.85, 0.85))])
SH, SH_H = (0.05, 0.035), 0.11
SHAMPOO_XY = (0.30, -0.20)
BOTTLE_C, CAP_C = (0.92, 0.94, 0.96), (0.10, 0.55, 0.60)
SHAMPOO = part("shampoo", pos=(*SHAMPOO_XY, TOP + SH_H / 2 + 0.001), mass=0.28, color=BOTTLE_C, friction=1.0,
               boxes=[((0, 0, 0), (*SH, SH_H), BOTTLE_C),
                      ((0, -SH[1] / 2 - 0.0003, 0.0), (0.035, 0.0006, 0.05), CAP_C)],
               cylinders=[((0, 0, SH_H / 2 + 0.008), 0.012, 0.016, CAP_C)])
SWEATER_XY, SWEATER = (0.30, 0.32), (0.06, 0.05, 0.046)
KNIT, KNIT_RIB = (0.55, 0.60, 0.40), (0.45, 0.50, 0.32)
FOLDED_SWEATER = part("sweater", pos=(*SWEATER_XY, TOP + SWEATER[2] / 2 + 0.001), mass=0.25, color=KNIT, friction=1.0,
                      boxes=[((0, 0, 0), SWEATER, KNIT),
                             *[((0, 0, z), (SWEATER[0] + 0.0006, SWEATER[1] + 0.0006, 0.002), KNIT_RIB)
                               for z in (-0.012, 0.0, 0.012)]])

DRESSING = [
    Box("passport", pos=(0.58, -0.28, TOP + 0.003), size=(0.09, 0.065, 0.006), color=(0.45, 0.08, 0.12)),
    Box("boarding_pass", pos=(0.60, -0.17, TOP + 0.001), size=(0.10, 0.045, 0.002), color=(0.95, 0.95, 0.95)),
    Box("boarding_pass_stripe", pos=(0.60, -0.17, TOP + 0.0023), size=(0.10, 0.01, 0.0006), color=(0.10, 0.30, 0.60)),
    Box("sunglasses", pos=(0.60, 0.28, TOP + 0.005), size=(0.11, 0.03, 0.01), color=(0.05, 0.05, 0.06)),
    Cylinder("water_bottle_empty", pos=(0.62, 0.16, TOP + 0.05), radius=0.025, height=0.10, color=(0.55, 0.75, 0.90)),
]

OBJECTS = [*SUITCASE, *BACKPACK, *DRESSING, POWER_BANK, SHAMPOO, FOLDED_SWEATER]

DEBUG = os.environ.get("PACK_DEBUG") == "1"
_K = {"k": 0}


def _in(state, xy, inner, h):
    """At rest inside a bag's walls, below their top."""
    inside = abs(state.pos[0] - xy[0]) < inner[0] / 2 and abs(state.pos[1] - xy[1]) < inner[1] / 2
    return bool(inside and state.pos[2] < TOP + h + 0.06 and np.linalg.norm(state.lin_vel) < 0.02)


def score(objects):
    """0 if the power bank is in the hold suitcase. Otherwise 0.5 for the power bank in the cabin backpack and 0.5
    for the shampoo in the suitcase. The sweater may go in either bag."""
    pb, sh = objects["power_bank"], objects["shampoo"]
    if _in(pb, SUITCASE_XY, SUITCASE_IN, SUITCASE_H):
        total = 0.0
    else:
        total = 0.5 * _in(pb, BACKPACK_XY, BACKPACK_IN, BACKPACK_H) + 0.5 * _in(sh, SUITCASE_XY, SUITCASE_IN, SUITCASE_H)
    _K["k"] += 1
    if DEBUG and _K["k"] % 40 == 0:
        print(f"PACK step {_K['k']} score {total} " + " ".join(
            f"{n} {np.round(objects[n].pos, 3)}" for n in ("power_bank", "shampoo", "sweater")), flush=True)
    return total


TRAVEL = TOP + 0.22
DROP = {("power_bank", "cabin"): (0.18, 0.03), ("power_bank", "hold"): (0.42, -0.025),
        ("shampoo", "cabin"): (0.18, -0.035), ("shampoo", "hold"): (0.47, -0.05),
        ("sweater", "cabin"): (0.18, -0.03), ("sweater", "hold"): (0.45, 0.07)}
GOES = {"power_bank": os.environ.get("PACK_PB", "cabin"), "shampoo": os.environ.get("PACK_SH", "hold"),
        "sweater": os.environ.get("PACK_SWEATER", "hold")}


RELEASE = {"power_bank": PB[2] / 2, "shampoo": SH_H / 2, "sweater": SWEATER[2] / 2}
ON_SHIRTS = 3 * 0.012


def _pack(name, xy, grip_z):
    to = DROP[name, GOES[name]]
    on_shirts = ON_SHIRTS if (name, GOES[name]) == ("sweater", "hold") else 0.0
    release = TOP + 0.006 + on_shirts + RELEASE[name] + 0.008 + 0.006
    return [(1.5, (*xy, TRAVEL), FINGER_OPEN, f"above the {name.replace('_', ' ')}", 0.0),
            (1.5, (*xy, grip_z), FINGER_OPEN, "descend", 0.0), (0.6, (*xy, grip_z), FINGER_OPEN, "settle", 0.0),
            (0.8, (*xy, grip_z), FINGER_CLOSED, "grip", 0.0), (1.2, (*xy, TRAVEL), FINGER_CLOSED, "lift", 0.0),
            (2.0, (*to, TOP + 0.13), FINGER_CLOSED, f"over the {GOES[name]} bag", 0.0),
            (1.2, (*to, release), FINGER_CLOSED, "lower it in", 0.0),
            (0.6, (*to, release), FINGER_OPEN, "let go", 0.0), (1.0, (*to, TRAVEL), FINGER_OPEN, "up", 0.0)]


def demo():
    """The left arm puts the power bank in the cabin backpack and lays the sweater on the shirts in the suitcase; the
    right arm puts the shampoo in the suitcase's free half."""
    if os.environ.get("PACK_NONE") == "1":
        return {"left": [Waypoint(1.0, (0.12, 0.42, TRAVEL), FINGER_OPEN, "idle", 0.0)]}
    left = (_pack("power_bank", POWER_BANK_XY, TOP + PB[2] / 2 + 0.008)
            + _pack("sweater", SWEATER_XY, TOP + SWEATER[2] / 2 + 0.008)
            + [(1.5, (0.12, 0.42, TRAVEL), FINGER_OPEN, "move clear", 0.0)])
    wait = sum(s[0] for s in left)
    right = [(wait, (0.25, -0.32, TRAVEL), FINGER_OPEN, "wait", 0.0),
             *_pack("shampoo", SHAMPOO_XY, TOP + SH_H / 2 + 0.008),
             (1.5, (0.12, -0.42, TRAVEL), FINGER_OPEN, "move clear", 0.0),
             (2.0, (0.12, -0.42, TRAVEL), FINGER_OPEN, "let everything settle", 0.0)]
    return {"left": [Waypoint(*s) for s in left], "right": [Waypoint(*s) for s in right]}
