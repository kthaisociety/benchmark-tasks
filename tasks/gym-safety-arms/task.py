"""Gym safety arms for yam_arena.

A scaled power rack stands on the table with a lifter lying on its bench, head toward the robot, under a bar resting
in the J-hooks. Each upright carries a red safety arm on a vertical slider with eight notches. For a bench press both
arms belong at the notch just below where the bar touches the lifter's chest. The left arm starts at the top notch and
the right arm at the bottom one. Every control step, task.py reads both arms and sets their drive targets so that they
follow wherever the robot moves them and, once let go, click into the nearest notch.

The notch detent, the scoring and the demo plans live in safety.py (no simulator needed, tested by test_task.py).
"""

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import safety as sf  # noqa: E402
from partkit import Slider, _part_state, on_reset, on_step, part  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
INSTRUCTION = "Set the rack's safety arms for this bench press."

STEEL = (0.62, 0.64, 0.67)
FRAME = (0.20, 0.22, 0.25)
SAFETY = (0.80, 0.12, 0.10)
MARK = (0.92, 0.92, 0.88)
PLATE = (0.12, 0.12, 0.13)
PAD = (0.08, 0.08, 0.09)
SKIN = (0.86, 0.66, 0.52)
SHIRT = (0.15, 0.40, 0.70)
SHORTS = (0.18, 0.18, 0.22)

DX = -sf.RACK_X          # world x to the rack's frame


def _ry(deg):
    a = math.radians(deg) / 2
    return (math.cos(a), 0.0, math.sin(a), 0.0)


def _octagon(cx, cy, cz, width, length, color):
    """A regular octagonal prism along y, `width` across the flats: four boxes turned 0/45/90/135 degrees about y."""
    side = width * math.tan(math.radians(22.5))
    return [((cx, cy, cz), (width, length, side), color, _ry(a)) for a in (0, 45, 90, 135)]


def _safety_arm(side):
    """One safety arm at its rest (top) notch, in the rack's frame: the arm along x under the bar, a carriage on the
    upright, and a handle on the near end."""
    y = side * sf.ARM_Y
    z = sf.TOP_NOTCH
    return (
        (((sf.ARM_X0 + sf.ARM_X1) / 2 + DX, y, z - sf.ARM_T / 2), (sf.ARM_X1 - sf.ARM_X0, sf.ARM_W, sf.ARM_T), SAFETY),
        ((0.0, side * 0.150, z - 0.010), (0.036, 0.030, 0.020), SAFETY),                 # 1 cm off the table at the low notch
        ((sf.HANDLE_X + DX, y, z + sf.HANDLE[2] / 2), sf.HANDLE, SAFETY),
    )


LEFT = Slider("left_arm", rest=(0.0, sf.ARM_Y, sf.TOP_NOTCH), travel=sf.TRAVEL, rate=sf.RATE, damping=sf.DAMPING,
              mass=sf.ARM_MASS, boxes=_safety_arm(1))
RIGHT = Slider("right_arm", rest=(0.0, -sf.ARM_Y, sf.TOP_NOTCH), travel=sf.TRAVEL, rate=sf.RATE, damping=sf.DAMPING,
               mass=sf.ARM_MASS, boxes=_safety_arm(-1))


def _rack_boxes():
    b = []
    for side in (-1, 1):
        y = side * sf.POST_Y
        b.append(((0.0, y, sf.POST_H / 2), (0.030, 0.030, sf.POST_H), FRAME))                       # upright
        b.append(((0.05, y, 0.006), (0.14, 0.030, 0.012), FRAME))                                   # foot
        hy = side * (sf.POST_Y - 0.022)
        b.append(((0.0, hy, sf.BAR_Z - sf.BAR_W / 2 - 0.005), (0.030, 0.014, 0.010), FRAME))       # J-hook seat
        b.append(((0.017, hy, sf.BAR_Z - 0.004), (0.005, 0.014, 0.022), FRAME))                    # J-hook lip
        for n in sf.NOTCHES:                                                                        # notch marks
            b.append(((0.0, side * (sf.POST_Y - 0.0155), n), (0.020, 0.002, 0.003), MARK))
    b += _octagon(0.0, 0.0, sf.BAR_Z, sf.BAR_W, 2 * sf.BAR_HALF_LEN - 0.004, STEEL)                  # the bar
    for side in (-1, 1):
        for py in sf.PLATE_Y:
            b += _octagon(0.0, side * py, sf.BAR_Z, sf.PLATE_W, sf.PLATE_T, PLATE)
    b += [
        ((0.095, 0.0, sf.BENCH_TOP / 2), (0.39, 0.100, sf.BENCH_TOP), PAD),                         # bench
        ((0.04, 0.0, (sf.BENCH_TOP + sf.CHEST_TOP) / 2), (0.16, 0.130, sf.CHEST_TOP - sf.BENCH_TOP), SHIRT),  # torso
        ((-0.075, 0.0, sf.BENCH_TOP + 0.018), (0.040, 0.045, 0.036), SKIN),                        # head
        ((0.150, 0.0, sf.BENCH_TOP + 0.016), (0.060, 0.110, 0.032), SHORTS),                       # hips
    ]
    for side in (-1, 1):
        b.append(((0.235, side * 0.032, sf.BENCH_TOP + 0.012), (0.110, 0.034, 0.024), SKIN))       # legs
        b.append(((0.030, side * 0.082, sf.BENCH_TOP + 0.012), (0.090, 0.030, 0.024), SKIN))       # arms at the sides
    return b


RACK = part("power_rack", pos=(sf.RACK_X, 0.0, TOP + 0.001), boxes=_rack_boxes(), mass=20.0, color=FRAME,
            sliders=(LEFT, RIGHT), friction=1.0)

OBJECTS = [RACK]

_TRIAL = sf.Trial()
_START = {"pos": None, "yaw": None}


def _yaw(q_xyzw):
    x, y, z, w = q_xyzw
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def _set_targets(environment, left_h, right_h):
    asset = environment.unwrapped.scene[RACK.name]
    target = asset.data.joint_pos_target.clone()
    names = list(asset.joint_names)
    for name, h in (("left_arm_slide", left_h), ("right_arm_slide", right_h)):
        target[:, names.index(name)] = min(sf.SAG, max(-sf.TRAVEL, sf.h_to_q(h)) + sf.SAG)
    asset.set_joint_position_target(target)


def _place_at_start(environment):
    """Both sliders spawn at their rest (top) notch: put each arm at its start notch (the right one at the bottom)."""
    asset = environment.unwrapped.scene[RACK.name]
    names = list(asset.joint_names)
    pos, vel = asset.data.joint_pos.clone(), asset.data.joint_vel.clone()
    for name, side in (("left_arm_slide", "left"), ("right_arm_slide", "right")):
        pos[:, names.index(name)] = sf.h_to_q(sf.START[side])
        vel[:, names.index(name)] = 0.0
    asset.write_joint_state_to_sim(pos, vel)
    _set_targets(environment, sf.START["left"], sf.START["right"])


@on_reset
def _reset(environment=None):
    _TRIAL.reset()
    _START.update(pos=None, yaw=None)


@on_step
def _step(environment=None):
    """Every control step: read both safety arms and set their drives to follow them toward the nearest notch."""
    if environment is None:
        return
    st = _part_state(environment, RACK)
    if _START["pos"] is None:
        _START.update(pos=st.pos.copy(), yaw=_yaw(st.quat_xyzw))
        _place_at_start(environment)
        st = _part_state(environment, RACK)
    moved = (float(((st.pos[:2] - _START["pos"][:2]) ** 2).sum()) ** 0.5 > 0.01
             or abs(_yaw(st.quat_xyzw) - _START["yaw"]) > math.radians(3))
    left, right = sf.q_to_h(st.sliders["left_arm"]), sf.q_to_h(st.sliders["right_arm"])
    _set_targets(environment, *_TRIAL.observe(left, right, moved))


def score(objects):
    """0 if the rack moved more than 1 cm or turned more than 3 degrees, or either safety arm ends below 7.0 cm (a
    failed rep would come down on the chest first). Otherwise 1.0 both arms between 7.0 and 8.0 cm (just under where the
    bar touches the chest, 8.0 cm); 0.5 one there and the other higher; 0.25 both higher (no full rep possible)."""
    return _TRIAL.score()


def demo():
    """Each arm grips its safety arm's handle, moves it to the notch just below the chest contact, lets go and parks.
    SAFETY_DEMO picks a variant instead (see README.md)."""
    variant = os.environ.get("SAFETY_DEMO", "correct")
    plans = sf.demo_plan(variant, TOP)
    return {side: [Waypoint(d, tuple(float(v) for v in p), FINGER_OPEN if f == sf.FINGER_OPEN else float(f), label,
                            float(yaw)) for d, p, f, label, yaw in plan]
            for side, plan in plans.items() if plan}
