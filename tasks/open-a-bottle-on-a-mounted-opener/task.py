"""On the station table, in front of the right arm, stands a short green glass soda bottle closed with a red crown
cap. Further forward and to the left a steel bottle opener is fixed to the front of a dark post at about the height of
a bar top: a flat steel tooth sticks out toward the robot from the opener's face. A round cork coaster lies on the
table near the middle.

A crown cap is crimped on and cannot be pulled off by hand. To open the bottle on a mounted opener, the bottle is
lifted, its cap is slipped over the opener's tooth so that the tooth sits under the edge of the cap, and the bottle is
pulled down: the tooth holds the cap's edge and the cap pops off. The open bottle is then stood upright on the coaster
and let go.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from partkit import _SPECS, _part_state, on_reset, on_step, part, write_pose  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Open the bottle on the mounted bottle opener, then stand the open bottle on the coaster and let go."

GLASS = (0.16, 0.46, 0.24)
LABEL = (0.93, 0.92, 0.86)
CAP_RED = (0.80, 0.10, 0.08)
STEEL = (0.70, 0.72, 0.75)
POST = (0.18, 0.19, 0.21)
CORK = (0.72, 0.55, 0.36)

# -- the bottle: origin at the middle of its base ----------------------------------------------------------------------
BODY_R, BODY_H = 0.030, 0.090
NECK_R = 0.007
SEAT = 0.115  # top of the neck, where the cap sits (above the base)
BOTTLE_MASS = 0.30
BOTTLE_AT = (0.27, -0.25)

# -- the crown cap: origin at the middle of its underside --------------------------------------------------------------
CAP_R, CAP_H = 0.0145, 0.006
CAP_MASS = 0.006

# -- the opener: a steel face plate on a dark post, a flat tooth sticking out toward the robot -------------------------
ENGAGE = (0.40, -0.20)  # bottle axis with the cap over the tooth
TOOTH_REACH = 0.014  # out from the face plate
TOOTH_W, TOOTH_T = 0.024, 0.003
TIP_IN = 0.0110  # the tooth's tip this far in from the cap's axis at ENGAGE (cap edge at 14.5 mm, neck at 7 mm)
FACE_X = ENGAGE[0] + TIP_IN + TOOTH_REACH  # the face plate's front
TIP_X = FACE_X - TOOTH_REACH
TZ = TOP + 0.148  # the tooth's top face: 12 mm under the cap of the bottle carried 4.5 cm up
PLATE_T, PLATE_W = 0.012, 0.050
PLATE_Z0, PLATE_Z1 = TZ - 0.010, TZ + 0.003
POST_T, POST_W = 0.020, 0.060

# -- the coaster -------------------------------------------------------------------------------------------------------
COASTER = (0.31, -0.04)
COASTER_R, COASTER_H = 0.045, 0.004

# -- scoring -----------------------------------------------------------------------------------------------------------
TIPPED_DEG = 30.0  # the bottle tilted more than this, ever: it spilled
UPRIGHT_DEG = 10.0
ON_COASTER = 0.030  # bottle axis within this of the coaster's centre
POP_LIFT = 0.0010  # the tooth has held the cap's edge while the bottle came down this far: the cap pops
REGLUE = 0.0005  # away from the tooth the crimp holds: any shift of the cap off its seat is undone
LET_GO = 0.06  # each gripper's TCP at least this far from the bottle's grip point


def _cyl(z0, z1, r, color):
    return ((0.0, 0.0, (z0 + z1) / 2), r, z1 - z0, color)


BOTTLE = part(
    "bottle", pos=(BOTTLE_AT[0], BOTTLE_AT[1], TOP + 0.001), mass=BOTTLE_MASS, color=GLASS, friction=1.0,
    boxes=[((0.0, 0.0, 0.045), (0.03, 0.03, 0.06), GLASS)],
    cylinders=[_cyl(0.0, BODY_H, BODY_R, GLASS),
               _cyl(0.030, 0.070, BODY_R + 0.0004, LABEL),
               _cyl(BODY_H, 0.097, 0.020, GLASS),
               _cyl(0.097, 0.105, 0.013, GLASS),
               _cyl(0.105, SEAT, NECK_R, GLASS)])
CAP = part(
    "cap", pos=(BOTTLE_AT[0], BOTTLE_AT[1], TOP + 0.001 + SEAT + 0.0005), mass=CAP_MASS, color=CAP_RED, friction=0.8,
    boxes=[((0.0, 0.0, CAP_H / 2), (0.012, 0.012, CAP_H - 0.001), CAP_RED)],
    cylinders=[_cyl(0.0, CAP_H, CAP_R, CAP_RED)])

OBJECTS = [
    Box("opener_post", pos=(FACE_X + PLATE_T + POST_T / 2, ENGAGE[1], (TOP + PLATE_Z1) / 2),
        size=(POST_T, POST_W, PLATE_Z1 - TOP), color=POST),
    Box("opener_plate", pos=(FACE_X + PLATE_T / 2, ENGAGE[1], (PLATE_Z0 + PLATE_Z1) / 2),
        size=(PLATE_T, PLATE_W, PLATE_Z1 - PLATE_Z0), color=STEEL),
    Box("opener_tooth", pos=(FACE_X - TOOTH_REACH / 2, ENGAGE[1], TZ - TOOTH_T / 2),
        size=(TOOTH_REACH, TOOTH_W, TOOTH_T), color=STEEL),
    Cylinder("coaster", pos=(COASTER[0], COASTER[1], TOP + COASTER_H / 2), radius=COASTER_R, height=COASTER_H,
             color=CORK),
    BOTTLE,
    CAP,
]


# -- the trial record, followed after every control step -----------------------------------------------------------------

def _fresh() -> dict:
    return {"engaged": False, "popped": False, "tipped": False, "steps": 0, "pop_step": None, "tcps": []}


_S = _fresh()
_DEBUG = bool(os.environ.get("BOTTLE_DEBUG"))


def _tilt_deg(state) -> float:
    z = state.rotation()[:, 2]
    return math.degrees(math.acos(max(-1.0, min(1.0, float(z[2])))))


def tooth_under_cap(cap_pos) -> bool:
    """The tooth sits under the cap's overhang (past the neck, inside the cap's edge), the cap on or just above it."""
    x, y, z = (float(v) for v in cap_pos)
    reach = TIP_X - x  # how far in from the cap's axis the tooth's tip is
    return (NECK_R + 0.001 < reach < CAP_R - 0.0005 and abs(y - ENGAGE[1]) < TOOTH_W / 2 + 0.004
            and TZ - 0.0015 < z < TZ + 0.006)


def grippers(environment) -> list[np.ndarray]:
    """World positions of both grippers' TCP (between the finger pads, 13 mm above the pad tips)."""
    base = environment.unwrapped
    origin = base.scene.env_origins[0]
    out = []
    for key in ("left_robot", "right_robot"):
        art = base.scene[key]
        names = list(art.body_names)
        pos = (getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0] - origin).detach().cpu().numpy()
        g = pos[names.index("gripper")].astype(np.float64)
        mid = (pos[names.index("tip_left")] + pos[names.index("tip_right")]).astype(np.float64) / 2
        ax = (mid - g) / (np.linalg.norm(mid - g) + 1e-12)
        out.append(g + ax * 0.134)
    return out


def _seat_cap(environment, bottle) -> None:
    x, y, z, w = (float(v) for v in bottle.quat_xyzw)
    write_pose(environment, CAP.name, tuple(float(v) for v in bottle.to_world((0.0, 0.0, SEAT))), (x, y, z, w),
               lin_vel=tuple(float(v) for v in bottle.lin_vel), ang_vel=tuple(float(v) for v in bottle.ang_vel))


@on_reset
def _reset(environment=None):
    _S.clear()
    _S.update(_fresh())
    if environment is not None:
        _seat_cap(environment, _part_state(environment, _SPECS[BOTTLE.name]))


@on_step
def _follow(environment) -> None:
    """The crimp: until the tooth has popped it, the cap stays on the bottle's neck, except that a cap resting on the
    tooth may lift off the neck (the bottle being pulled down under it)."""
    _S["steps"] += 1
    bottle = _part_state(environment, _SPECS[BOTTLE.name])
    cap = _part_state(environment, _SPECS[CAP.name])
    if _tilt_deg(bottle) > TIPPED_DEG or bottle.pos[2] < TOP - 0.05:
        _S["tipped"] = True
    try:
        _S["tcps"] = grippers(environment)
    except (KeyError, ValueError, AttributeError):
        _S["tcps"] = []
    if not _S["popped"]:
        lift = float(bottle.to_local(cap.pos)[2] - SEAT)
        engaged = tooth_under_cap(cap.pos) and _tilt_deg(bottle) < 20.0
        if engaged:
            _S["engaged"] = True
        if engaged and lift > POP_LIFT:
            _S["popped"], _S["pop_step"] = True, _S["steps"]
            # the crimp lets go with a snap: the cap flips up and off toward the robot
            write_pose(environment, CAP.name, tuple(float(v) for v in cap.pos),
                       tuple(float(v) for v in cap.quat_xyzw), lin_vel=(-0.35, 0.0, 0.25), ang_vel=(0.0, -10.0, 0.0))
        elif not engaged or lift < -REGLUE:
            if abs(lift) > REGLUE or np.linalg.norm(bottle.to_local(cap.pos)[:2]) > REGLUE:
                _seat_cap(environment, bottle)
    if _DEBUG and _S["steps"] % 10 == 0:
        print(f"[bottle] step {_S['steps']} bottle {np.round(bottle.pos, 4)} tilt {_tilt_deg(bottle):.1f} "
              f"cap {np.round(cap.pos, 4)} lift {float(bottle.to_local(cap.pos)[2] - SEAT) * 1000:.1f}mm "
              f"engaged {_S['engaged']} popped {_S['popped']} tcps {[np.round(t, 3).tolist() for t in _S['tcps']]}",
              flush=True)


def on_coaster(bottle) -> bool:
    dx, dy = float(bottle.pos[0] - COASTER[0]), float(bottle.pos[1] - COASTER[1])
    base_z = float(bottle.pos[2])
    return (math.hypot(dx, dy) < ON_COASTER and TOP + COASTER_H - 0.003 < base_z < TOP + COASTER_H + 0.006
            and _tilt_deg(bottle) < UPRIGHT_DEG)


def released(bottle) -> bool:
    grip = bottle.to_world((0.0, 0.0, GRIP_UP))
    return bool(_S["tcps"]) and all(np.linalg.norm(np.asarray(t) - grip) > LET_GO for t in _S["tcps"])


def score(objects) -> float:
    bottle = objects[BOTTLE.name]
    if _S["tipped"]:
        return 0.0
    if not _S["popped"]:
        return 0.25 if _S["engaged"] else 0.0
    if not on_coaster(bottle):
        return 0.5
    if not (released(bottle) and bottle.resting()):
        return 0.75
    return 1.0


def success(objects) -> bool:
    return score(objects) >= 1.0


# -- the scripted demo (right arm) -----------------------------------------------------------------------------------

GRIP_UP = 0.080  # TCP above the bottle's base when gripping it (pads on the top of the body)
LIFT = 0.045  # bottle base above the table while carried to the opener: cap underside 12 mm over the tooth
PULL = 0.025  # how far the bottle is pulled down under the tooth
SAG = (0.002, 0.016, 0.009)  # the loaded arm sags this far (x, y, z) below its commanded pose; the demo aims high


def demo():
    variant = os.environ.get("DEMO_VARIANT", "full")
    shortcut = _shortcut(variant)
    if shortcut is not None:
        return shortcut
    bx, by = BOTTLE_AT
    ex, ey = ENGAGE
    cx, cy = COASTER
    sx, sy, sz = SAG
    ex, ey, cx, cy = ex + sx, ey + sy, cx + sx, cy + sy
    z_grip = TOP + 0.001 + GRIP_UP
    z_carry = TOP + LIFT + GRIP_UP + sz
    z_place = TOP + COASTER_H + 0.002 + GRIP_UP
    hover = TOP + 0.16
    right = [
        Waypoint(2.0, (bx, by, hover), FINGER_OPEN, "above the bottle"),
        Waypoint(1.5, (bx, by, z_grip), FINGER_OPEN, "down round it"),
        Waypoint(0.8, (bx, by, z_grip), FINGER_CLOSED, "grip"),
        Waypoint(1.2, (bx, by, z_carry), FINGER_CLOSED, "lift"),
        Waypoint(1.8, (ex - 0.04, ey, z_carry), FINGER_CLOSED, "in front of the opener"),
    ]
    if variant in ("full", "no_place", "no_release"):
        right += [
            Waypoint(1.5, (ex, ey, z_carry), FINGER_CLOSED, "cap over the tooth"),
            Waypoint(0.5, (ex, ey, z_carry), FINGER_CLOSED, "settle"),
            Waypoint(2.0, (ex, ey, z_carry - PULL), FINGER_CLOSED, "pull down"),
            Waypoint(1.2, (ex - 0.06, ey, z_carry - PULL), FINGER_CLOSED, "back out"),
        ]
    if variant == "no_place":
        right += [Waypoint(3.0, (ex - 0.06, ey, z_carry - PULL), FINGER_CLOSED, "hold it")]
        return {"right": right}
    right += [
        Waypoint(1.8, (cx, cy, z_carry), FINGER_CLOSED, "over the coaster"),
        Waypoint(1.5, (cx, cy, z_place), FINGER_CLOSED, "set down"),
    ]
    if variant == "no_release":
        right += [Waypoint(3.0, (cx, cy, z_place), FINGER_CLOSED, "keep holding")]
        return {"right": right}
    right += [
        Waypoint(0.8, (cx, cy, z_place), FINGER_OPEN, "let go"),
        Waypoint(1.0, (cx, cy, hover), FINGER_OPEN, "up"),
        Waypoint(1.8, (0.12, -0.42, TOP + 0.20), FINGER_OPEN, "clear"),
    ]
    return {"right": right}


def _shortcut(variant: str):
    """Shortcuts that must not score: doing nothing, carrying the closed bottle straight to the coaster, and pinching
    the cap and pulling it straight up."""
    bx, by = BOTTLE_AT
    cx, cy = COASTER[0] + SAG[0], COASTER[1] + SAG[1]
    hover = TOP + 0.16
    if variant == "nothing":
        return {"right": [Waypoint(3.0, (0.25, -0.30, hover), FINGER_OPEN, "wait")]}
    if variant == "skip_opener":
        z_grip, z_carry = TOP + 0.001 + GRIP_UP, TOP + LIFT + GRIP_UP + SAG[2]
        z_place = TOP + COASTER_H + 0.002 + GRIP_UP
        return {"right": [
            Waypoint(2.0, (bx, by, hover), FINGER_OPEN, "above the bottle"),
            Waypoint(1.5, (bx, by, z_grip), FINGER_OPEN, "down round it"),
            Waypoint(0.8, (bx, by, z_grip), FINGER_CLOSED, "grip"),
            Waypoint(1.2, (bx, by, z_carry), FINGER_CLOSED, "lift"),
            Waypoint(2.0, (cx, cy, z_carry), FINGER_CLOSED, "over the coaster"),
            Waypoint(1.5, (cx, cy, z_place), FINGER_CLOSED, "set down"),
            Waypoint(0.8, (cx, cy, z_place), FINGER_OPEN, "let go"),
            Waypoint(1.0, (cx, cy, hover), FINGER_OPEN, "up"),
            Waypoint(1.8, (0.12, -0.42, TOP + 0.20), FINGER_OPEN, "clear"),
        ]}
    if variant == "pull_cap":
        z_cap = TOP + 0.001 + SEAT + CAP_H / 2 + 0.008
        return {"right": [
            Waypoint(2.0, (bx, by, hover), FINGER_OPEN, "above the cap"),
            Waypoint(1.5, (bx, by, z_cap), FINGER_OPEN, "fingers round the cap"),
            Waypoint(0.8, (bx, by, z_cap), FINGER_CLOSED, "pinch it"),
            Waypoint(2.0, (bx, by, z_cap + 0.05), FINGER_CLOSED, "pull it straight up"),
            Waypoint(1.0, (bx, by, z_cap + 0.05), FINGER_CLOSED, "hold"),
            Waypoint(1.0, (bx, by, z_cap + 0.05), FINGER_OPEN, "let go"),
            Waypoint(1.8, (0.12, -0.42, TOP + 0.20), FINGER_OPEN, "clear"),
        ]}
    return None
