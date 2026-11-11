"""Blood-pressure cuff sizing for yam_arena.

A child-size training arm (a fixed square beam, 5 x 5 cm, 20 cm round) lies across the table along y: a lighter upper
arm with its shoulder end on the robot's left, and a darker forearm and hand stub on its right. Three blue 3D-printed
cuffs, 4, 8 and 12 cm wide, stand in a row in front of it. A cuff is a rigid arch: two legs 2.5 cm clear either side of
the arm, a bridge over it and a square grip knob on top. All three fit over the arm; they differ only in width, and
the right one has a width of about 40% of the arm's circumference (the 8 cm cuff). The robot has to put a cuff on the
arm.

The scoring and the demo plan live in cuff_scoring.py (no simulator needed, tested by test_task.py). See README.md.
"""

import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cuff_scoring as cs  # noqa: E402
from partkit import part  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
INSTRUCTION = "Put a blood-pressure cuff on the training arm."

BLUE = (0.10, 0.35, 0.75)
KNOB_BLUE = (0.05, 0.22, 0.55)
SKIN = (0.86, 0.66, 0.52)
SKIN_DARK = (0.62, 0.44, 0.34)
CREASE = (0.35, 0.22, 0.17)


def _cuff(name):
    """Width along local y (along the arm), legs either side of the arm along local x."""
    w = cs.WIDTHS[name]
    leg_x = cs.INNER / 2 + cs.LEG_T / 2
    boxes = [
        ((leg_x, 0.0, cs.LEG_H / 2), (cs.LEG_T, w, cs.LEG_H), BLUE),
        ((-leg_x, 0.0, cs.LEG_H / 2), (cs.LEG_T, w, cs.LEG_H), BLUE),
        ((0.0, 0.0, cs.LEG_H + cs.BRIDGE_T / 2), (cs.OUTER, w, cs.BRIDGE_T), BLUE),
        ((0.0, 0.0, cs.LEG_H + cs.BRIDGE_T + cs.KNOB[2] / 2), cs.KNOB, KNOB_BLUE),
    ]
    return part(name, pos=(cs.START_X, cs.START_Y[name], TOP + cs.SPAWN_LIFT), boxes=boxes, mass=cs.cuff_mass(w),
                color=BLUE, friction=1.0)


def _limb():
    z = TOP + cs.LIMB_H / 2
    (f0, f1), (u0, u1) = cs.FOREARM_Y, cs.UPPER_Y
    return [
        Box("forearm", pos=(cs.LIMB_X, (f0 + f1) / 2, z), size=(cs.LIMB_W, f1 - f0, cs.LIMB_H), color=SKIN_DARK),
        Box("upper_arm", pos=(cs.LIMB_X, (u0 + u1) / 2, z), size=(cs.LIMB_W, u1 - u0, cs.LIMB_H), color=SKIN),
        # a thin band marking the elbow crease
        Box("elbow_crease", pos=(cs.LIMB_X, u0, z), size=(cs.LIMB_W + 0.002, 0.006, cs.LIMB_H + 0.002), color=CREASE),
    ]


OBJECTS = [*_limb(), *[_cuff(n) for n in cs.CUFF_NAMES]]


def score(objects):
    """1.0 the 8 cm cuff (width 40% of the arm's circumference) is on the upper arm, with no other cuff on the arm;
    0.5 it is on the arm but misplaced or another cuff is on too; 0.25 it is over the arm but not standing on the
    table (held over it, perched on it); else 0. Read at the end of the run (no success(): a later knock counts)."""
    return cs.evaluate(objects, TOP)


def demo():
    """Right arm: grip the 8 cm cuff by its knob, carry it over the upper arm, lower its legs either side of the arm,
    let go. CUFF_DEMO picks a shortcut variant instead: narrow | wide | elbow | beside | hold | nothing."""
    variant = os.environ.get("CUFF_DEMO", "correct")
    steps = cs.demo_plan(variant, TOP)
    return {"right": [Waypoint(d, p, FINGER_OPEN if f == "open" else f, label, 0.0) for d, p, f, label in steps]}
