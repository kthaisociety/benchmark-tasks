"""On the station table: a tan head board (child mannequin, seen from above) carries two identical, unmarked eye
markers. A chart card lies between the robot and the head with two coloured dots in the same two y-columns as the
head's eyes: a grey dot over the week's flagged (weaker) eye's column, a green dot over the clear (stronger) eye's
column. Two identical foam eye patches sit on a tray. Amblyopia occlusion therapy patches the STRONGER eye, so the
robot must set one patch on the eye in the green column and leave the eye in the grey column bare - the opposite of
patching whichever eye looks "wrong".
"""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import os  # noqa: E402
from partkit import env  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, pick_place_waypoints  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- geometry (m), everything on the robot's right (y negative), within the right arm's reach -------------------
HEAD_X, HEAD_Y = 0.40, -0.10
HEAD_R, HEAD_H = 0.07, 0.03
EYE_SEP = 0.045  # the two eyes' offset from the head's own centre line (9 cm apart, so the two 3.5 cm partial-credit
                 # zones below never touch)
EYE_R, EYE_H = 0.016, 0.004

CHART_X, CHART_Y = 0.20, -0.10
CHART_SIZE = (0.05, 0.085, 0.006)
DOT_R, DOT_H = 0.012, 0.003

PATCH_X = 0.26
# Probed on Brev 2026-11-09: a 1.6 cm tall disc only ever got grazed, never gripped - ONBOARDING.md 5 states objects
# under about 4.5 cm tall slip out of the grip (a disc squeezed from above needs its own height for the fingers to
# close on, unlike a "flat bar" gripped from the side along its length). Patches are now modelled as a rigid foam
# occlusion cup (a real paediatric product, not just adhesive stickers) rather than a flat sticker.
# Second probe: a 3.6 cm diameter cup got shoved sideways, never lifted - likely too wide for the gripper's opening.
# Third probe: narrowed to 2.6 cm (matching pin-fit's pin_a radius) but kept it light (5 g) - the gripper made contact
# but then flung it clean off the table during the lift, consistent with a light object taking a hard closing
# impulse. Fourth probe matched pin-fit's pin_a exactly (26 mm dia, 8 cm tall, 333 g): the grasp and carry finally
# worked cleanly (patch tracked the gripper right to within 1.3 cm of the target by step 160) - but released, the
# tall thin cylinder toppled over and rolled away, ending 20 cm off target by the end. Shortening height only,
# keeping the proven radius and mass fixed, to lower the centre of mass and cut the topple-and-roll risk, did not
# fix it either: even settled, resting patches still slid off their mark after a touchdown with a few mm of residual
# drop (same failure place-ecg-leads-correct-electrodes avoids with its electrode cups). Adding a low raised guide
# ring around each eye position - plausible on a training head board, which often marks equipment placement - so a
# dropped patch can no longer roll away, the same proven fix, instead of chasing sub-mm drop-height precision.
PATCH_R, PATCH_H, PATCH_MASS = 0.013, 0.05, 0.333
# The arm's own tracking error at this reach was ~1.5-1.7 cm steady-state (gripper settled 1.5 cm from the commanded
# target, a known IK/reach-accuracy effect - see tasks/pin-fit's own "aim past it" note), not just object slip, so
# the guide ring needs to be wider than the patch alone to actually catch a touchdown. 6 cm inner, still well short
# of the 9 cm gap between the two eye positions (no overlap with the other ring).
GUIDE_INSIDE, GUIDE_WALL, GUIDE_H = 0.06, 0.004, 0.012

# The grey (flagged/weaker) and green (clear/stronger) dots and the eyes they match sit in the same y-columns.
FLAG_Y = HEAD_Y + EYE_SEP   # -0.055: grey dot's column; this eye must stay bare
CLEAR_Y = HEAD_Y - EYE_SEP  # -0.145: green dot's column; this eye gets the patch

FLAG_XY = (HEAD_X + 0.015, FLAG_Y)
CLEAR_XY = (HEAD_X + 0.015, CLEAR_Y)
HEAD_SURFACE_Z = TOP + HEAD_H

TAN = (0.85, 0.72, 0.58)
NEUTRAL = (0.75, 0.78, 0.80)   # both eye markers on the head use this - no colour hint on the head itself
CREAM = (0.93, 0.90, 0.80)
GREY = (0.55, 0.55, 0.55)
GREEN = (0.25, 0.65, 0.35)
PATCH_COLOR = (0.88, 0.80, 0.68)

HEAD = Cylinder("head", pos=(HEAD_X, HEAD_Y, TOP + HEAD_H / 2), radius=HEAD_R, height=HEAD_H, color=TAN)
EYE_FLAG_MARK = Cylinder("eye_flag_mark", pos=(FLAG_XY[0], FLAG_XY[1], HEAD_SURFACE_Z + EYE_H / 2),
                         radius=EYE_R, height=EYE_H, color=NEUTRAL)
EYE_CLEAR_MARK = Cylinder("eye_clear_mark", pos=(CLEAR_XY[0], CLEAR_XY[1], HEAD_SURFACE_Z + EYE_H / 2),
                          radius=EYE_R, height=EYE_H, color=NEUTRAL)


def _guide_ring(name, cx, cy):
    """Four low walls on the head's surface, square, around (cx, cy): both eye positions get one (symmetric, no
    hint), so a dropped patch can't roll off its mark."""
    half = GUIDE_INSIDE / 2 + GUIDE_WALL / 2
    wz = HEAD_SURFACE_Z + GUIDE_H / 2
    side = GUIDE_INSIDE + 2 * GUIDE_WALL
    return [
        Box(f"{name}_near", pos=(cx - half, cy, wz), size=(GUIDE_WALL, side, GUIDE_H), color=NEUTRAL),
        Box(f"{name}_far", pos=(cx + half, cy, wz), size=(GUIDE_WALL, side, GUIDE_H), color=NEUTRAL),
        Box(f"{name}_left", pos=(cx, cy + half, wz), size=(GUIDE_INSIDE, GUIDE_WALL, GUIDE_H), color=NEUTRAL),
        Box(f"{name}_right", pos=(cx, cy - half, wz), size=(GUIDE_INSIDE, GUIDE_WALL, GUIDE_H), color=NEUTRAL),
    ]


GUIDE_FLAG = _guide_ring("guide_flag", *FLAG_XY)
GUIDE_CLEAR = _guide_ring("guide_clear", *CLEAR_XY)

CHART = Box("chart", pos=(CHART_X, CHART_Y, TOP + CHART_SIZE[2] / 2), size=CHART_SIZE, color=CREAM)
CHART_TOP = TOP + CHART_SIZE[2]
DOT_FLAG = Cylinder("dot_flag", pos=(CHART_X, FLAG_Y, CHART_TOP + DOT_H / 2), radius=DOT_R, height=DOT_H, color=GREY)
DOT_CLEAR = Cylinder("dot_clear", pos=(CHART_X, CLEAR_Y, CHART_TOP + DOT_H / 2), radius=DOT_R, height=DOT_H,
                      color=GREEN)

PATCH_1 = Cylinder("patch_1", pos=(PATCH_X, FLAG_Y, TOP + PATCH_H / 2), radius=PATCH_R, height=PATCH_H,
                    mass=PATCH_MASS, color=PATCH_COLOR)
PATCH_2 = Cylinder("patch_2", pos=(PATCH_X, CLEAR_Y, TOP + PATCH_H / 2), radius=PATCH_R, height=PATCH_H,
                    mass=PATCH_MASS, color=PATCH_COLOR)

OBJECTS = [HEAD, EYE_FLAG_MARK, EYE_CLEAR_MARK, CHART, DOT_FLAG, DOT_CLEAR, *GUIDE_FLAG, *GUIDE_CLEAR, PATCH_1, PATCH_2]

# -- scoring -------------------------------------------------------------------------------------------------------
FULL_TOL, PARTIAL_TOL = 0.015, 0.035   # m, patch centre from the eye marker's centre
REST_TOL = 0.010                        # m, patch's low point from the head's top surface, to count as "on the head"
MOVE_TOL = 0.010                        # m, how far the head or chart can drift and still count as "fixed"
RELEASE_DIST = 0.045                    # m, both grippers' TCP at least this far from a patch = let go


def _dist_xy(obj, xy) -> float:
    return float(np.linalg.norm(np.asarray(obj.pos[:2]) - np.asarray(xy)))


def _on_head(obj) -> bool:
    return abs(float(obj.low[2]) - HEAD_SURFACE_Z) < REST_TOL and obj.resting()


def _fixed_ok(objects) -> bool:
    return _dist_xy(objects["head"], (HEAD_X, HEAD_Y)) < MOVE_TOL and _dist_xy(objects["chart"],
                                                                                (CHART_X, CHART_Y)) < MOVE_TOL


def grippers() -> list[np.ndarray]:
    """World positions of both grippers' TCP (13 mm above the finger-pad tips); [] if unknown (offline)."""
    e = env()
    if e is None:
        return []
    base = e.unwrapped
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


def _released(objects) -> bool:
    pts = [objects["patch_1"].pos, objects["patch_2"].pos]
    grips = grippers()
    if not grips:
        return True
    return all(float(np.linalg.norm(np.asarray(p) - g)) >= RELEASE_DIST for g in grips for p in pts)


def score(objects) -> float:
    """0 if the head or chart moved, a patch rests on the flagged (grey-column) eye, neither patch rests within
    3.5 cm of the clear (green-column) eye, or a gripper still holds a patch. Otherwise 1.0 within 1.5 cm of the
    clear eye marker's centre, 0.5 within 1.5-3.5 cm."""
    if not _fixed_ok(objects):
        return 0.0
    patches = [objects["patch_1"], objects["patch_2"]]
    on_head = [p for p in patches if _on_head(p)]
    if any(_dist_xy(p, FLAG_XY) <= PARTIAL_TOL for p in on_head):
        return 0.0
    if not _released(objects):
        return 0.0
    near_clear = [_dist_xy(p, CLEAR_XY) for p in on_head if _dist_xy(p, CLEAR_XY) <= PARTIAL_TOL]
    if not near_clear:
        return 0.0
    return 1.0 if min(near_clear) <= FULL_TOL else 0.5


def success(objects) -> bool:
    return score(objects) == 1.0


def distance(objects) -> float:
    """How far (m) the nearer patch on the head is from the clear eye marker's centre; +0.10 if a patch sits on the
    flagged eye instead."""
    patches = [objects["patch_1"], objects["patch_2"]]
    on_head = [p for p in patches if _on_head(p)]
    penalty = 0.10 if any(_dist_xy(p, FLAG_XY) <= PARTIAL_TOL for p in on_head) else 0.0
    best = min((_dist_xy(p, CLEAR_XY) for p in on_head), default=PARTIAL_TOL + 0.10)
    return float(best + penalty)


# -- demo + shortcut variants (PATCH_DEMO=correct|wrong|both|nothing|hold; default correct) -----------------------
MODE = os.environ.get("PATCH_DEMO", "correct").strip().lower()
TRAVEL_Z = TOP + 0.20
AWAY = (0.10, -0.30, TRAVEL_Z)


# Probed on Brev 2026-11-09: passing a patch's raw centre as the pick point grazed it instead of gripping it (it
# barely lifted 4 mm then dropped) - the patch is only 1.6 cm tall, so the fingers need to close at its mid-height,
# and the Waypoint TCP sits above the pad tips, not at them. TCP = pad-tip target height + 0.008 (LEAN_WORKFLOW.md).
GRIP_TCP_OFFSET = 0.008


def _pick_xyz(patch):
    return (patch.pos[0], patch.pos[1], patch.pos[2] + GRIP_TCP_OFFSET)


# Probed on Brev 2026-11-09: the guide ring contained the patch (it settled ~2.4 cm from the target instead of
# rolling 10-20 cm away), but the arm's own reach/IK tracking error at this distance is ~1.9 cm in x, 1.3 cm in y -
# the patch always settles short and to one side of the commanded target, not randomly. Aiming past it by that same
# fixed amount (pin-fit's own "aim past it" fix for the same effect) should land it inside the 1.5 cm full-credit
# zone instead of the 1.5-3.5 cm partial one.
PLACE_AIM_CORRECTION = (0.019, 0.013)


def _place_xy(xy):
    cx, cy = xy[0] + PLACE_AIM_CORRECTION[0], xy[1] + PLACE_AIM_CORRECTION[1]
    return (cx, cy, HEAD_SURFACE_Z + PATCH_H / 2 + GRIP_TCP_OFFSET + 0.001)


# Probed on Brev 2026-11-09: with the grasp finally solid (pin-fit-matched radius/mass), the patch landed within
# 1.5 cm of the target around step 160, then was knocked ~12-20 cm away by the time the arm retreated - the
# built-in retreat goes straight up from the release point with no pause, and plank-unload's own demo needed an
# extra "clear the tray" waypoint after its retreat for the same reason. Pausing after release, and cutting the
# drop clearance in _place_xy above, lets the patch settle before anything moves near it again.
def _settled_pick_place(pick, place):
    wps = pick_place_waypoints(pick, place, travel_z=TRAVEL_Z)
    rx, ry, rz = wps[-2].pos  # the built-in "release" waypoint's position (at the place height)
    tx, ty, tz = wps[-1].pos  # the built-in "retreat" waypoint's position (same x, y, at travel_z)
    return wps[:-2] + [
        Waypoint(0.6, (rx, ry, rz), FINGER_OPEN, "release"),
        Waypoint(1.0, (rx, ry, rz), FINGER_OPEN, "settle"),
        Waypoint(1.2, (tx, ty, tz), FINGER_OPEN, "retreat"),
    ]


def demo():
    clear_place = _place_xy(CLEAR_XY)
    flag_place = _place_xy(FLAG_XY)

    if MODE == "nothing":
        return {"right": [Waypoint(1.0, PATCH_1.pos, FINGER_OPEN, "wait", 0.0)]}

    if MODE == "wrong":
        arm = _settled_pick_place(_pick_xyz(PATCH_1), flag_place)
        arm.append(Waypoint(1.5, AWAY, FINGER_OPEN, "clear the camera"))
        return {"right": arm}

    if MODE == "both":
        # Probed on Brev 2026-11-09: placing the clear patch first scored 1.0 - the harness ends the episode the
        # instant score() hits 1.0, so the flag placement never ran at all (patch_1 never left its start position).
        # Covering the flagged eye FIRST means it's already covered by the time the clear placement could trigger
        # that early stop, so the hard-fail check in score() fires before any false success.
        arm = _settled_pick_place(_pick_xyz(PATCH_1), flag_place)
        arm += _settled_pick_place(_pick_xyz(PATCH_2), clear_place)
        arm.append(Waypoint(1.5, AWAY, FINGER_OPEN, "clear the camera"))
        return {"right": arm}

    if MODE == "hold":
        # _settled_pick_place already opens the fingers and retreats; a "keep holding" waypoint appended after it
        # would just close empty fingers up at travel height, a no-op (probed: scored 1.0, a real bug, not a pass).
        # Stop before its release+settle+retreat tail (its "lower" waypoint is wps[-3]) and hold there instead.
        wps = pick_place_waypoints(_pick_xyz(PATCH_2), clear_place, travel_z=TRAVEL_Z)
        arm = wps[:-2] + [Waypoint(2.5, wps[-2].pos, FINGER_CLOSED, "keep holding")]
        return {"right": arm}

    arm = _settled_pick_place(_pick_xyz(PATCH_2), clear_place)
    arm.append(Waypoint(1.5, AWAY, FINGER_OPEN, "clear the camera"))
    return {"right": arm}
