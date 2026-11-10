"""On the station table: a full-size 47 x 97 mm softwood ceiling joist, 800 mm long (cut to fit between joist hangers
800 mm apart, so its clear span is its length), lies flat across the table in front of both arms, its 97 mm face up,
clamped to the bench for pre-boring. A plumber's white chalk cross sits at mid-span, 15 mm in from the near edge. A 22 mm
hole the electrician bored earlier shows as a dark disc on the centre line, 260 mm from the left end. A hardwood drill
guide block (60 mm cube, a steel bush through its centre) stands on the table in front of the joist, opposite the
chalk cross. The robot must stand the guide on the joist where the 22 mm pipe hole should be bored.

Scoring is in scoring.py. The only spot that scores is the right-hand permitted zone (200 to 320 mm from the right end)
on the centre line; the left-hand zone is ruled out by the electrician's hole (holes at least three diameters apart).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Variants of the scripted demo: JHZ_VARIANT=<name> (see VARIANTS).
"""

from __future__ import annotations

import collections
import os
import pathlib
import sys
import warnings

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import scoring as S  # noqa: E402
from partkit import env, on_reset, part  # noqa: E402
from yam_arena.constants import ARM_JOINTS, FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint, pick_place_waypoints  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
FACE_Z = S.joist_face_z(TOP)

INSTRUCTION = ("The plumber has chalked a cross where he wants his 15 mm pipe to pass through this ceiling joist. "
               "Stand the drill guide on the joist where the 22 mm hole for it should be bored, then let go of it.")

SPRUCE = (0.52, 0.56, 0.38)  # pressure-treated (greenish) C16, so the face stands out from the pale bench
BEECH = (0.72, 0.52, 0.32)
STEEL = (0.70, 0.72, 0.75)
BORE = (0.06, 0.06, 0.06)
CHALK = (0.96, 0.96, 0.93)  # strokes 7 mm wide: about 2 px in the 320 x 176 top camera
DECAL_H = 0.0006  # marks stand 0.4 mm proud of the face so they render; the guide can still sit on them

JOIST = Box("joist", pos=(S.JOIST_X, 0.0, TOP + S.JOIST_THICK / 2), size=(S.JOIST_DEPTH, S.SPAN, S.JOIST_THICK),
            color=SPRUCE)  # no mass: fixed (clamped to the bench)
EXISTING = Cylinder("existing_hole", pos=(*S.EXISTING_HOLE, FACE_Z + 0.0001), radius=S.HOLE_D / 2, height=DECAL_H,
                    color=BORE)
CROSS_XY = (S.JOIST_X - S.JOIST_DEPTH / 2 + 0.015, 0.0)  # mid-span, its centre 15 mm in from the near edge
CROSS = [
    Box("chalk_cross_a", pos=(*CROSS_XY, FACE_Z + 0.0001), size=(0.007, 0.050, DECAL_H), color=CHALK),
    Box("chalk_cross_b", pos=(*CROSS_XY, FACE_Z + 0.0002), size=(0.026, 0.007, DECAL_H), color=CHALK),
]

# The guide: a beech block with a steel bush (outer ring) and its 22 mm bore (dark) on the top face, centred.
GUIDE_START = (0.21, 0.0)  # on the table in front of the joist, opposite the chalk cross
GH = S.GUIDE_HALF_H
GUIDE = part("drill_guide", pos=(*GUIDE_START, TOP + GH + 0.001), mass=0.15, color=BEECH, friction=1.0,
             boxes=[((0.0, 0.0, 0.0), S.GUIDE_SIZE, BEECH)],
             cylinders=[((0.0, 0.0, GH + 0.0005), 0.016, 0.001, STEEL),
                        ((0.0, 0.0, GH + 0.0012), S.HOLE_D / 2, 0.0004, BORE)])

OBJECTS = [JOIST, EXISTING, *CROSS, GUIDE]


# -- scoring: the spot (scoring.py), let go and left alone ------------------------------------------------------------
_ARMS = (("left_robot", "left"), ("right_robot", "right"))
_S = {"objects": None, "value": 0.0, "bores": collections.deque(maxlen=S.SETTLE_STEPS + 1), "warned": False}


@on_reset
def _reset(environment=None):
    _S.update(objects=None, value=0.0, warned=False)  # the module lives across trials
    _S["bores"].clear()


def _grippers_clear(g):
    """Both grippers clear of the guide, from the arms' joint positions and the framework's kinematics; None (counts as
    not let go) if the arms can't be read."""
    try:
        from yam_arena import ik, station
        scene = env().unwrapped.scene
        clear = True
        for name, side in _ARMS:
            robot = scene[name]
            ids = robot.find_joints(ARM_JOINTS, preserve_order=True)[0]
            q = robot.data.joint_pos
            q = getattr(q, "torch", q)[0]
            q = np.asarray(q.detach().cpu().numpy() if hasattr(q, "detach") else q, dtype=float)
            tcp, rot = ik.tcp_pose(q[list(ids)], station.BASES[side])
            clear &= S.gripper_clear(tcp, np.asarray(rot)[:, 2], g.pos, g.quat_xyzw)
        return clear
    except Exception as error:  # noqa: BLE001 - an unexpected embodiment: never award full credit
        if not _S["warned"]:
            _S["warned"] = True
            warnings.warn(f"bore-joist: can't read the arms ({type(error).__name__}: {error}); scoring 0",
                          RuntimeWarning, stacklevel=2)
        return None


def _update(objects):
    """Score once per control step (success() and score() get the same state dict)."""
    if _S["objects"] is objects:
        return _S["value"]
    g = objects[GUIDE.name]
    _S["bores"].append(S.bore_point(g.pos, g.quat_xyzw))
    r = S.read(g.pos, g.quat_xyzw, g.lin_vel, g.ang_vel, TOP)
    clear = _grippers_clear(g) if S.points(r) == 1.0 else False
    _S.update(objects=objects, value=S.final(r, clear, _S["bores"]))
    if TRACE:
        with open(TRACE, "a") as f:
            b = _S["bores"][-1]
            f.write(f"{b[0]:.4f} {b[1]:.4f} {b[2] - FACE_Z:+.4f} tilt={S.tilt_deg(g.quat_xyzw):.1f} "
                    f"off_cl={1000 * r.off_centre_line:.1f}mm from_end={1000 * r.from_support:.0f}mm "
                    f"standing={r.standing} points={S.points(r)} clear={clear} score={_S['value']}\n")
    return _S["value"]


def success(objects):
    """The guide in a permitted spot, let go and left alone for the last second; holding this for 1 s ends the
    trial with 1.0."""
    return _update(objects) == 1.0


def score(objects):
    return _update(objects)


def distance(objects):
    """From the bore to the middle of the right-hand zone on the centre line (m)."""
    b = S.bore_point(objects[GUIDE.name].pos, objects[GUIDE.name].quat_xyzw)
    target_y = -(S.END_Y - sum(S.ZONE) / 2)
    return float(((b[0] - S.JOIST_X) ** 2 + (b[1] - target_y) ** 2) ** 0.5)


# -- scripted demo and shortcut variants ------------------------------------------------------------------------------
# Pad tips 25 mm above the guide's base, so the closed fingers stay 25 mm above the joist face as it is set down.
GRIP_Z = TOP + 0.025 + PAD_TIP_PAST_TCP
SET_Z = GRIP_Z + S.JOIST_THICK + 0.001  # the guide's base 1 mm above the joist face
TRAVEL_Z = TOP + 0.12  # WHY: at 0.15 m the arms can't reach mid-span on the joist (IK check); the carried
# guide's base is still 35 mm above the joist face
PARK = {"right": (0.12, -0.42, TOP + 0.20), "left": (0.12, 0.42, TOP + 0.20)}
# WHY: a placement aim offset, measured on the official stack on the first probe (where the guide lands against where
# the TCP was sent). Zero until traced.
PLACE_AIM = (0.0, 0.0)

ZONE_MID = S.END_Y - sum(S.ZONE) / 2  # 0.14: 260 mm from an end
# name: (arm, bore x, bore y, expected score)
VARIANTS = {
    "demo": ("right", S.JOIST_X, -ZONE_MID, 1.0),  # right-hand zone, on the centre line
    "offcentre": ("right", S.JOIST_X + 0.015, -ZONE_MID, 0.0),  # in the zone, 15 mm off the centre line
    "held": ("right", S.JOIST_X, -ZONE_MID, 0.0),  # the right spot, but never let go
    "edge": ("right", S.JOIST_X + 0.030, -ZONE_MID, 0.0),  # in the zone, 30 mm off the centre line
    "cross": ("right", *CROSS_XY, 0.0),  # on the plumber's chalk cross
    "midspan": ("right", S.JOIST_X, -0.03, 0.0),  # mid-span (30 mm right of it, within reach), on the centre line
    "bearing": ("right", S.JOIST_X, -0.33, 0.0),  # 70 mm from the right support
    "on_hole": ("left", *S.EXISTING_HOLE, 0.0),  # over the electrician's hole
    "near_hole": ("left", S.JOIST_X, 0.09, 0.0),  # left zone (310 mm from the end), 50 mm from the existing hole
    "nothing": (None, None, None, 0.0),
}
VARIANT = os.environ.get("JHZ_VARIANT", "demo")
TRACE = os.environ.get("JHZ_TRACE")  # debugging: append the guide's reading every step to this file


def demo():
    """One arm lifts the guide off the table by its sides, carries it over the chosen spot, sets it down 1 mm above
    the joist face, lets go, rises and moves clear of the top camera. The other arm stays parked."""
    arm, x, y, _ = VARIANTS[VARIANT]
    if arm is None:
        return {"right": [Waypoint(2.0, PARK["right"], FINGER_OPEN, "stay clear")]}
    pick = (*GUIDE_START, GRIP_Z)
    place = (x + PLACE_AIM[0], y + PLACE_AIM[1], SET_Z)
    steps = pick_place_waypoints(pick, place, travel_z=TRAVEL_Z)
    if VARIANT == "held":
        return {arm: [*steps[:6], Waypoint(60.0, place, FINGER_CLOSED, "keep holding it")]}
    steps.append(Waypoint(1.5, PARK[arm], FINGER_OPEN, "move clear"))
    return {arm: steps}

