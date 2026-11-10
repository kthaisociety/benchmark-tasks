"""Gym spotter for yam_arena.

A scaled-down smith machine stands on the table with a lifter lying on its bench, head toward the robot. The bar rides
a vertical guide (a partkit slider) over the lifter's chest; the lifter's hands and forearms move with it. A scripted
lifter drives the bar through a set: three easy reps, a slower fourth, a grinding fifth, and a sixth that stalls 3.5 cm
off the chest, shakes and sinks back down. The robot is the spotter: hands off while the lifter moves the bar, take hold
when the sixth rep stalls, help it to lockout, where the lifter racks it.

The lifter, the scoring and the demo plans live in spotter.py (no simulator needed, tested by test_task.py).
"""

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import spotter as sp  # noqa: E402
from partkit import Slider, _part_state, on_reset, on_step, part  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
INSTRUCTION = "Spot the bar through this set of bench presses."

STEEL = (0.62, 0.64, 0.67)
FRAME = (0.20, 0.22, 0.25)
PLATE = (0.12, 0.12, 0.13)
COLLAR = (0.80, 0.15, 0.12)
PAD = (0.08, 0.08, 0.09)
SKIN = (0.86, 0.66, 0.52)
SHIRT = (0.15, 0.40, 0.70)
SHORTS = (0.18, 0.18, 0.22)

RATE, DAMPING, BAR_MASS = 500.0, 30.0, 0.25        # the lifter's "muscle": a stiff drive on the bar's guide
SAG = BAR_MASS * 9.81 / RATE                       # added to every target so the bar sits where the lifter wants it


def _ry(deg):
    """(w, x, y, z) rotation about y."""
    a = math.radians(deg) / 2
    return (math.cos(a), 0.0, math.sin(a), 0.0)


def _octagon(cx, cy, cz, width, length, color):
    """A regular octagonal prism along y, `width` across the flats: four boxes turned 0/45/90/135 degrees about y."""
    side = width * math.tan(math.radians(22.5))
    return [((cx, cy, cz), (width, length, side), color, _ry(a)) for a in (0, 45, 90, 135)]


def _bar_boxes():
    z = sp.BAR_HIGH
    boxes = _octagon(0.0, 0.0, z, sp.BAR_W, 2 * sp.BAR_HALF_LEN, STEEL)
    for side in (-1, 1):
        for py in sp.PLATE_Y:
            boxes += _octagon(0.0, side * py, z, sp.PLATE_W, sp.PLATE_T, PLATE)
        boxes.append(((0.0, side * (sp.PLATE_Y[-1] + 0.012), z), (0.030, 0.010, 0.030), COLLAR))
        boxes.append(((0.0, side * sp.POST_Y, z), (0.036, 0.030, 0.030), FRAME))          # carriage on the upright
        boxes.append(((0.0, side * sp.HAND_Y, z), (0.030, 0.026, 0.030), SKIN))           # the lifter's hands
        boxes.append(((0.0, side * sp.HAND_Y, z - sp.BAR_W / 2 - 0.035), (0.018, 0.020, 0.070), SKIN))  # forearms
    return tuple(boxes)


BAR = Slider("bar", rest=(0.0, 0.0, sp.BAR_HIGH), travel=sp.TRAVEL, rate=RATE, damping=DAMPING, mass=BAR_MASS,
             boxes=_bar_boxes())


def _frame_boxes():
    b = []
    for side in (-1, 1):
        y = side * sp.POST_Y
        b.append(((0.0, y, sp.POST_H / 2), (0.030, 0.030, sp.POST_H), FRAME))              # upright
        b.append(((0.02, y, 0.006), (0.16, 0.030, 0.012), FRAME))                          # foot
    b += [
        ((0.095, 0.0, sp.BENCH_TOP / 2), (0.39, 0.100, sp.BENCH_TOP), PAD),                # bench, x -0.10..0.29
        ((0.04, 0.0, (sp.BENCH_TOP + sp.CHEST_TOP) / 2), (0.16, 0.130, sp.CHEST_TOP - sp.BENCH_TOP), SHIRT),  # torso
        ((-0.075, 0.0, sp.BENCH_TOP + 0.018), (0.040, 0.045, 0.036), SKIN),               # head, toward the robot
        ((0.150, 0.0, sp.BENCH_TOP + 0.016), (0.060, 0.110, 0.032), SHORTS),              # hips
    ]
    for side in (-1, 1):
        b.append(((0.235, side * 0.032, sp.BENCH_TOP + 0.012), (0.110, 0.034, 0.024), SKIN))  # legs along the bench
        b.append(((0.000, side * 0.085, sp.BENCH_TOP + 0.012), (0.030, 0.040, 0.024), SKIN))  # upper arms, out to the side
    return b


RACK = part("smith_machine", pos=(sp.BAR_X, 0.0, TOP + 0.001), boxes=_frame_boxes(), mass=15.0, color=FRAME,
            sliders=(BAR,), friction=1.0)

OBJECTS = [RACK]

_TRIAL = sp.Trial()
_START = {"pos": None, "yaw": None}


def _yaw(q_xyzw):
    x, y, z, w = q_xyzw
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def _set_target(environment, bar_h):
    asset = environment.unwrapped.scene[RACK.name]
    target = asset.data.joint_pos_target.clone()
    target[:, list(asset.joint_names).index("bar_slide")] = min(SAG, bar_h - sp.TRAVEL + SAG)
    asset.set_joint_position_target(target)


@on_reset
def _reset(environment=None):
    _TRIAL.reset()
    _START.update(pos=None, yaw=None)


@on_step
def _step(environment=None):
    """Every control step: read the bar, run the lifter, drive the bar to where the lifter wants it."""
    if environment is None:
        return
    st = _part_state(environment, RACK)
    if _START["pos"] is None:
        _START.update(pos=st.pos.copy(), yaw=_yaw(st.quat_xyzw))
    moved = (float(((st.pos[:2] - _START["pos"][:2]) ** 2).sum()) ** 0.5 > 0.01
             or abs(_yaw(st.quat_xyzw) - _START["yaw"]) > math.radians(3))
    bar_h = st.sliders["bar"] + sp.TRAVEL
    _set_target(environment, _TRIAL.observe(bar_h, moved))


def score(objects):
    """0 if the machine was moved, or the bar was held or pushed off the lifter's path while the lifter moved it
    alone (the warm-up hold, any descent, reps 1-4, after the rack). 1.0 the stalled sixth rep caught before the bar
    came back down on the chest and helped to lockout; 0.5 helped to lockout only after it came down on the chest;
    0.25 caught and still held at least 2 cm off the chest at the end; else 0."""
    return _TRIAL.score()


def success(objects):
    """Full score: the lifter has racked the bar."""
    return _TRIAL.success()


def demo():
    """Right arm: wait with open fingers 4 cm over the bar at lockout, lower around the bar when the sixth rep stalls,
    take hold, help it to lockout, let go and park. SPOT_DEMO picks a variant instead:
    nothing | late | rep5 | takeover | halfway | block (see README.md)."""
    variant = os.environ.get("SPOT_DEMO", "correct")
    steps = sp.demo_plan(variant, TOP)
    if not steps:
        return {}
    return {"right": [Waypoint(d, tuple(float(v) for v in p), FINGER_OPEN if f == sp.FINGER_OPEN else float(f),
                               label, float(yaw)) for d, p, f, label, yaw in steps]}
