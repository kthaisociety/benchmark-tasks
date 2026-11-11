"""Geometry, scoring and scripted-demo plan for the blood-pressure cuff task.

Pure Python and numpy: no Isaac Sim, yam_arena or GPU, so the scoring and the demo plan are unit-tested offline
(test_task.py). task.py wraps this module for the simulator.

World frame, metres: x forward from the arm bases, y left, z up; heights are given relative to the table top.
The training arm (child size) lies across the table along y; "along" is the y coordinate, "lateral" the x offset from its axis.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# -- the training arm (child size): a fixed square beam lying across the table along y, shoulder end on the left --
LIMB_X = 0.34                      # the arm's axis
LIMB_W = 0.05                      # across the arm (x)
LIMB_H = 0.05                      # height
FOREARM_Y = (-0.20, -0.10)         # forearm and hand stub, darker; hand end at y -0.20
UPPER_Y = (-0.10, 0.08)            # upper arm: elbow crease at y -0.10, shoulder end at y +0.08
LIMB_Y = (FOREARM_Y[0], UPPER_Y[1])
CIRCUMFERENCE = 2 * (LIMB_W + LIMB_H)          # a tape measure round the arm: 0.20 m

# -- the sizing rule: a cuff's width should be about 40% of the arm's circumference ------------------------------------
RATIO_LO, RATIO_HI = 0.37, 0.50
ELBOW_GAP = 0.02                   # the cuff's lower edge at least this far above the elbow crease

# -- a cuff: a rigid arch standing on the table with its legs either side of the arm -------------------------------
# Body origin: the centre of the arch's footprint, on the table. Its width runs along its local y (along the arm),
# its span (legs either side of the arm) along its local x.
CLEAR = 0.025                      # clear gap between each leg and the arm
INNER = LIMB_W + 2 * CLEAR         # 0.10
LEG_T = 0.006
LEG_H = 0.06                       # the bridge's underside clears the 0.05 arm by 1 cm
BRIDGE_T = 0.006
OUTER = INNER + 2 * LEG_T          # 0.112
KNOB = (0.025, 0.025, 0.05)        # square grip knob on the bridge (x, y, z): the grip works across either pair of faces
HEIGHT = LEG_H + BRIDGE_T + KNOB[2]
SPAWN_LIFT = 0.001                 # loose parts start 1 mm above the table to settle

WIDTHS = {"cuff_12": 0.12, "cuff_8": 0.08, "cuff_4": 0.04}
# A row across the table in front of the training arm. The cuff that fits is in the middle, so taking the nearest,
# the farthest, the widest or the narrowest never scores.
START_X = 0.17
START_Y = {"cuff_12": 0.10, "cuff_8": -0.03, "cuff_4": -0.15}
CUFF_NAMES = tuple(WIDTHS)


def cuff_mass(width: float) -> float:
    """A hollow print: 40, 70 and 100 g for the 4, 8 and 12 cm cuffs."""
    return 0.010 + 0.75 * width


def ratio(name: str) -> float:
    return WIDTHS[name] / CIRCUMFERENCE


def right_size() -> list[str]:
    """The cuffs whose width is 37-50% of the arm's circumference."""
    return [n for n in CUFF_NAMES if RATIO_LO <= ratio(n) <= RATIO_HI]


# -- placement tests -------------------------------------------------------------------------------------------------
LATERAL_TOL = CLEAR                # arch centre within this of the arm's axis: standing astride it, legs clear
SEATED_Z = 0.006                   # origin at most this above the table: standing on its legs, not perched on the arm
TILT_MAX_DEG = 8.0
YAW_TOL_DEG = 12.0                 # the cuff's width axis within this of the arm's axis
OVER_LATERAL = INNER / 2           # "over the arm" (the 0.25 tier and extra cuffs): the axis passes between its legs
OVER_TILT_DEG = 30.0
EXTRA_SEATED_Z = 0.012


def axis_z(quat_xyzw) -> float:
    """Vertical component of the cuff's own z axis."""
    x, y, _, _ = (float(v) for v in quat_xyzw)
    return 1.0 - 2.0 * (x * x + y * y)


def tilt_deg(quat_xyzw) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, axis_z(quat_xyzw)))))


def yaw_error_deg(quat_xyzw) -> float:
    """Angle between the cuff's width axis (its local y) and the arm's axis (world y), either way round (0-90)."""
    x, y, z, w = (float(v) for v in quat_xyzw)
    yaw = math.atan2(2.0 * (x * y + w * z), 1.0 - 2.0 * (y * y + z * z))
    return math.degrees(abs((yaw + math.pi / 2) % math.pi - math.pi / 2))


@dataclass(frozen=True)
class Pose:
    lateral: float                 # x offset of the centre from the arm's axis
    along: float                   # y of the centre
    z: float                       # origin height above the table
    tilt: float
    yaw_err: float
    width: float

    @property
    def low(self) -> float:        # the edge toward the hand
        return self.along - self.width / 2

    @property
    def high(self) -> float:       # the edge toward the shoulder
        return self.along + self.width / 2


def pose_of(state, name: str, top: float) -> Pose:
    return Pose(float(state.pos[0]) - LIMB_X, float(state.pos[1]), float(state.pos[2]) - top,
                tilt_deg(state.quat_xyzw), yaw_error_deg(state.quat_xyzw), WIDTHS[name])


def on_arm(p: Pose) -> bool:
    """Standing on the table with its legs either side of the arm, wholly along the arm, upright."""
    return (abs(p.lateral) <= LATERAL_TOL and p.z <= SEATED_Z and p.tilt <= TILT_MAX_DEG
            and p.low >= LIMB_Y[0] and p.high <= LIMB_Y[1])


def over_arm(p: Pose) -> bool:
    """Its centre over the arm in plan view, upright-ish, at any height: held over it, perched on it or seated."""
    return abs(p.lateral) <= OVER_LATERAL and LIMB_Y[0] <= p.along <= LIMB_Y[1] and p.tilt <= OVER_TILT_DEG


def on_arm_loosely(p: Pose) -> bool:
    """Any part of the cuff standing over any part of the arm: what makes an extra cuff count against the score."""
    return (abs(p.lateral) <= OVER_LATERAL and p.z <= EXTRA_SEATED_Z
            and p.high > LIMB_Y[0] and p.low < LIMB_Y[1])


def in_zone(p: Pose) -> bool:
    """Wholly on the upper arm, its lower edge at least 2 cm above the elbow crease."""
    return p.low >= UPPER_Y[0] + ELBOW_GAP and p.high <= UPPER_Y[1]


def evaluate(states, top: float) -> float:
    """1.0 the right-size cuff is on the upper arm, aligned, with no other cuff on the arm; 0.5 it is on the arm but
    in the wrong place (forearm, crease) or turned, or another cuff is on the arm too; 0.25 it is over the arm but not
    standing on the table (held over it, perched on it); otherwise 0. A cuff of the wrong width never scores."""
    fits = right_size()
    if len(fits) != 1:
        raise ValueError(f"exactly one cuff must be the right size, got {fits}")
    right = fits[0]
    poses = {n: pose_of(states[n], n, top) for n in CUFF_NAMES}
    p = poses[right]
    extra = any(on_arm_loosely(poses[n]) for n in CUFF_NAMES if n != right)
    if on_arm(p):
        return 1.0 if (in_zone(p) and p.yaw_err <= YAW_TOL_DEG and not extra) else 0.5
    return 0.25 if over_arm(p) else 0.0


# -- scripted demo plan ----------------------------------------------------------------------------------------------
# Right arm, top-down. The pad tips sit about 8 mm below the TCP, so the TCP is 8 mm above the knob's mid-height.
REACH_M = (0.10, 0.50)             # distance from an arm's base the gripper can work at
RIGHT_BASE = (0.0, -0.305)
LEFT_BASE = (0.0, 0.305)
TCP_ABOVE_PAD = 0.008
KNOB_MID = LEG_H + BRIDGE_T + KNOB[2] / 2
SQUEEZE = 0.016                    # probe try 1: 8 mm let the hanging cuff rock in the grip; build: 16 mm
GRIP_VALUE = -(KNOB[0] - SQUEEZE) / 2         # finger command that closes on the knob's 25 mm
TRAVEL_ABOVE = 0.19                # TCP height while carrying: the arch's feet clear the 5 cm arm by 4 cm
PARK = (0.12, -0.42, 0.20)         # right arm clear of the top camera (x, y, height above the table)
ZONE_CENTRE_Y = (UPPER_Y[0] + ELBOW_GAP + UPPER_Y[1]) / 2       # 0.0: the middle of the 8 cm cuff's allowed range

# variant -> (cuff, target x, target y, release)   release False: the cuff is held over the target at the end
VARIANTS = {
    "correct": ("cuff_8", LIMB_X, ZONE_CENTRE_Y, True),
    "narrow": ("cuff_4", LIMB_X, ZONE_CENTRE_Y, True),
    "wide": ("cuff_12", LIMB_X, ZONE_CENTRE_Y, True),
    "elbow": ("cuff_8", LIMB_X, -0.15, True),           # the right cuff, over the forearm
    "beside": ("cuff_8", 0.24, -0.03, True),            # the right cuff set down in front of the arm, not over it
    "hold": ("cuff_8", LIMB_X, ZONE_CENTRE_Y, False),   # carried over the arm and never lowered
}


def demo_plan(variant: str, top: float):
    """Steps of (seconds, (x, y, z), finger, label); finger is "open" or a finger command. Heights are absolute."""
    if variant == "nothing":
        return [(2.0, (*PARK[:2], top + PARK[2]), "open", "wait")]
    name, tx, ty, release = VARIANTS[variant]
    sx, sy = START_X, START_Y[name]
    grip_z = top + SPAWN_LIFT + KNOB_MID + TCP_ABOVE_PAD
    travel = top + TRAVEL_ABOVE
    # Probe try 1: a 2 s carry swung the hanging cuff ~10 degrees and a leg landed on the arm. Move slowly, let it
    # steady over the target, hover with the feet just above the arm, then set it down.
    steps = [
        (2.0, (sx, sy, travel), "open", "above the cuff"),
        (1.5, (sx, sy, grip_z), "open", "descend"),
        (0.5, (sx, sy, grip_z), "open", "settle"),
        (1.0, (sx, sy, grip_z), GRIP_VALUE, "grip the knob"),
        (2.0, (sx, sy, travel), GRIP_VALUE, "lift"),
        (3.5, (tx, ty, travel), GRIP_VALUE, "carry over the arm"),
        (1.5, (tx, ty, travel), GRIP_VALUE, "steady"),
    ]
    if not release:
        return steps + [(2.0, (tx, ty, travel), GRIP_VALUE, "hold")]
    place_z = grip_z + 0.002                      # let go with the feet 3 mm above the table
    hover_z = place_z + LIMB_H + 0.015            # the feet 1.5 cm above the arm's top
    steps += [
        (1.5, (tx, ty, hover_z), GRIP_VALUE, "hover just above the arm"),
        (1.0, (tx, ty, hover_z), GRIP_VALUE, "steady"),
        (2.0, (tx, ty, place_z), GRIP_VALUE, "lower the legs either side of the arm"),
        (0.5, (tx, ty, place_z), GRIP_VALUE, "settle"),
        (1.0, (tx, ty, place_z), "open", "release"),
        (1.5, (tx, ty, travel), "open", "withdraw"),
        (1.5, (PARK[0], PARK[1], top + PARK[2]), "open", "park clear of the camera"),
    ]
    return steps


def footprints():
    """Plan-view rectangles (name, x0, x1, y0, y1) of everything in the starting scene, for layout checks."""
    out = [("arm", LIMB_X - LIMB_W / 2, LIMB_X + LIMB_W / 2, LIMB_Y[0], LIMB_Y[1])]
    for n in CUFF_NAMES:
        y, w = START_Y[n], WIDTHS[n]
        out.append((n, START_X - OUTER / 2, START_X + OUTER / 2, y - w / 2, y + w / 2))
    return out
