"""Servicing a linear-guide carriage on a gantry or wafer handler: the rail has an end stopper that keeps the carriage
from running off its open end. To take the carriage off, a technician holds the stopper down with one hand and slides
the carriage off past it with the other, because forcing it over the stop face damages the balls and the stopper.
Here: a grey rail lies along x on a low plinth, its open end toward the arms, with a blue carriage (grip post) seated near
the far end. An orange spring stopper stands near the open end, its face blocking the carriage and its 2 cm pad
off to the right of the rail. A blue mat to the left is for the removed carriage.
"""

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from partkit import Slider, env, on_reset, on_step, part  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = ("Hold the orange stopper down and slide the blue carriage along the grey rail off its open end, keeping it "
               "within 1.5 cm of the rail line and no more than 3 cm above it until it is past the stopper. Set it upright with its "
               "centre on the blue mat, then let go of both and keep both fingers at least 3 cm from the post.")

GREY, BLUE, ORANGE = (0.62, 0.64, 0.66), (0.10, 0.30, 0.90), (1.00, 0.50, 0.00)
PLINTH, MAT_H = 0.012, 0.003
RAIL_XY, MAT_XY, MAT_HALF = (0.33, -0.01), (0.30, 0.18), 0.05
HALF_RAIL, HALF_CAR, STOP_T = 97.5, 18.0, 20.0          # mm: rail half length, carriage half length, far end block
CAR_START, SEATED_Z = 56.0, 3.0                         # mm: carriage centre along the rail; origin height on the head
STOPPER_X0, STOPPER_X1, TRAVEL = -76.0, -66.0, 0.007    # mm along the rail: stopper rear / carriage-side face; slide (m)
LIFT_LIMIT, SIDE_LIMIT, FREE_DIST = 0.03, 0.015, 0.03


def mm(x0, x1, y0, y1, z0, z1, color):
    return (((x0 + x1) / 2000, (y0 + y1) / 2000, (z0 + z1) / 2000), ((x1 - x0) / 1000, (y1 - y0) / 1000,
                                                                      (z1 - z0) / 1000), color)


# Rail frame: origin on the rail centre at its base, x along the rail. Neck 8 mm, head 16 mm (top at 14 mm).
# Stopper: a 4.4 mm plate (z 4.6 to 9) under the carriage's side wall (which hangs from z 3), 6 mm face at x -66, joined
# by a bar to a 2 cm pad at y -15 cm; pressing the pad 7 mm drops the plate below the wall.
RAIL = part("rail", pos=(RAIL_XY[0], RAIL_XY[1], TOP + PLINTH), mass=6.0, friction=1.2, color=GREY, boxes=[
    mm(-HALF_RAIL, HALF_RAIL, -4, 4, 0, 8, GREY), mm(-HALF_RAIL, HALF_RAIL, -8, 8, 8, 14, GREY),
    mm(HALF_RAIL, HALF_RAIL + STOP_T, -14, 14, 0, 30, GREY)], sliders=[Slider(
        "stopper", rest=(STOPPER_X1 / 1000, 0.0, 0.0), travel=TRAVEL, rate=200.0, damping=3.0, mass=0.004,
        boxes=(mm(STOPPER_X0, STOPPER_X1, -13, -9.5, 4.6, 9, ORANGE), mm(STOPPER_X0, STOPPER_X1, -150, -13, 4.6, 9, ORANGE),
               mm(-80, -60, -160, -140, 4.6, 14, ORANGE)))])
# Carriage frame: origin on the centre of its footprint at the lips' underside; 36 x 26 mm, 80 g, 30 mm post, 60 mm tall.
CARRIAGE = part("carriage", pos=(RAIL_XY[0] + CAR_START / 1000, RAIL_XY[1], TOP + PLINTH + SEATED_Z / 1000 + 0.0005),
                mass=0.08, friction=1.2, color=BLUE, boxes=[
    mm(-HALF_CAR, HALF_CAR, -13, 13, 11, 15, BLUE), mm(-HALF_CAR, HALF_CAR, 9, 13, 0, 11, BLUE),
    mm(-HALF_CAR, HALF_CAR, -13, -9, 0, 11, BLUE), mm(-HALF_CAR, HALF_CAR, 5, 9, 0, 1.5, BLUE),
    mm(-HALF_CAR, HALF_CAR, -9, -5, 0, 1.5, BLUE)], cylinders=[((0.0, 0.0, 0.045), 0.015, 0.060, BLUE)])
OBJECTS = [
    Box("floor_slab", pos=(0.33, 0.03, TOP + 0.0002), size=(0.40, 0.56, 0.0004), color=(0.80, 0.80, 0.78)),
    Box("rail_plinth", pos=(RAIL_XY[0], RAIL_XY[1], TOP + PLINTH / 2), size=(0.195, 0.012, PLINTH), color=(0.20, 0.25, 0.32)),
    Box("removed_mat", pos=(*MAT_XY, TOP + MAT_H / 2), size=(0.10, 0.10, MAT_H), color=(0.35, 0.55, 0.95)),
    RAIL, CARRIAGE]

_S: dict = {}


@on_reset
def _reset(environment=None):
    _S.update(step=0, scored=None, last=0.0, violated=False, passed=False, hist=[], free=[])


@on_step
def _tick(environment=None):
    _S["step"] += 1


_reset()


def _released(car) -> bool:
    """Both arms: fingers at least 90% open and both pad tips at least 3 cm from the post's axis (read from the arms)."""
    e = env()
    if e is None:
        return False
    scene = e.unwrapped.scene
    a0, a1 = car.to_world((0, 0, 0.015)), car.to_world((0, 0, 0.075))
    for name in ("left_robot", "right_robot"):
        arm = scene[name]
        q = np.asarray(getattr(arm.data.joint_pos, "torch", arm.data.joint_pos)[0].detach().cpu(), float)
        fingers = q[arm.find_joints(["joint7", "joint8"], preserve_order=True)[0]]
        tips = getattr(arm.data.body_pos_w, "torch", arm.data.body_pos_w)[0].detach().cpu().numpy().astype(float)
        tips = tips[arm.find_bodies(["tip_left", "tip_right"], preserve_order=True)[0]] - np.asarray(
            getattr(scene.env_origins, "torch", scene.env_origins)[0].detach().cpu(), float)
        if fingers.mean() / FINGER_OPEN < 0.9:
            return False
        for t in tips:
            u = np.clip(np.dot(t - a0, a1 - a0) / np.dot(a1 - a0, a1 - a0), 0, 1)
            if np.linalg.norm(t - (a0 + u * (a1 - a0))) < FREE_DIST:
                return False
    return True


def _evaluate(objects) -> float:
    if _S["scored"] == _S["step"]:
        return _S["last"]
    _S["scored"] = _S["step"]
    rail, car = objects["rail"], objects["carriage"]
    c = (car.pos - rail.pos) * 1000                       # carriage origin in the rail frame, mm
    lift = (c[2] - SEATED_Z) / 1000
    # Latched: until the carriage is wholly past the stopper, lifted more than 3 cm or more than 1.5 cm off the rail axis.
    if c[0] + HALF_CAR < STOPPER_X0:
        _S["passed"] = True
    if not _S["passed"] and (lift > LIFT_LIMIT or abs(c[1]) > SIDE_LIMIT * 1000):
        _S["violated"] = True
    R = car.rotation()
    upright = bool(R[2, 2] > math.cos(math.radians(3)))
    hist = _S["hist"]
    hist.append((car.pos.copy(), _released(car), rail.sliders.get("stopper", -1.0)))
    del hist[:-20]
    still = len(hist) == 20 and all(np.abs(p - car.pos).max() < 0.001 and r and s > -0.0005 for p, r, s in hist)
    off = bool(c[0] + HALF_CAR < -HALF_RAIL or abs(c[1]) > 40)
    on_mat = bool(abs(car.pos[0] - MAT_XY[0]) <= MAT_HALF and abs(car.pos[1] - MAT_XY[1]) <= MAT_HALF
                  and car.pos[2] < TOP + MAT_H + 0.004)
    _S["last"] = 0.0 if _S["violated"] or not off else 1.0 if (on_mat and upright and still) else 0.5
    return _S["last"]


# No success(): a later action could spoil the result.
def score(objects) -> float:
    """1.0: carriage wholly off the rail's open end, upright (within 3 degrees), centre within the 10 x 10 cm blue mat,
    released (both fingers open and 3 cm from the post), still (under 1 mm) and the stopper back up for the last 1 s.
    0.5: wholly off the rail otherwise. 0: still on the rail, or at any time before it was wholly past the
    stopper it was lifted more than 3 cm or its centre was more than 1.5 cm sideways from the rail axis (latched)."""
    return _evaluate(objects)


# -- scripted demo: right arm presses the stopper pad, left arm pulls the carriage off by its post --------------------
VARIANT = os.environ.get("RCRP_VARIANT", "")      # nothing, left_only, tip_off, right_only, lift_over, lift_restore, elsewhere, hold
PARK = {"left": (0.14, 0.34, TOP + 0.20), "right": (0.12, -0.38, TOP + 0.20)}
CARRY = TOP + 0.16


def demo():
    rx, ry = RAIL_XY
    px, py = rx - 0.070, ry - 0.150                  # stopper pad centre
    pad_top = TOP + PLINTH + 0.014
    press = pad_top - 0.0075 + 0.013 - 0.001        # closed finger tips 13 mm below the TCP, 7.5 mm down
    org = TOP + PLINTH + 0.0035
    grab, slide = org + 0.0525, org + 0.0535
    cx0 = rx + CAR_START / 1000
    cx1 = rx - (HALF_RAIL + HALF_CAR + 12) / 1000
    mat_z = TOP + MAT_H + 0.0525 + 0.002
    park_r, park_l = PARK["right"], PARK["left"]
    R = [Waypoint(1.5, (px, py, pad_top + 0.06), FINGER_CLOSED, "above pad"),
         Waypoint(1.5, (px, py, press), FINGER_CLOSED, "press"), Waypoint(0.6, (px, py, press), FINGER_CLOSED, "settle"),
         Waypoint(11.0, (px, py, press), FINGER_CLOSED, "hold down"),
         Waypoint(1.0, (px, py, pad_top + 0.06), FINGER_CLOSED, "release the stopper"),
         Waypoint(0.6, (px, py, pad_top + 0.06), FINGER_OPEN, "open"), Waypoint(2.0, park_r, FINGER_OPEN, "park")]
    L = [Waypoint(1.5, (rx - 0.04, ry, TOP + 0.115), FINGER_OPEN, "above the rail"),
         Waypoint(1.0, (rx - 0.04, ry, grab), FINGER_OPEN, "descend"),
         Waypoint(2.5, (cx0, ry, grab), FINGER_OPEN, "along the rail to the post"),
         Waypoint(0.6, (cx0, ry, grab), FINGER_OPEN, "settle"), Waypoint(0.8, (cx0, ry, grab), FINGER_CLOSED, "grip")]
    off_rail = [Waypoint(6.5, (cx1, ry, slide), FINGER_CLOSED, "slide off the open end"),
                Waypoint(0.8, (cx1, ry, slide), FINGER_CLOSED, "settle"),
                Waypoint(1.0, (cx1, ry, TOP + 0.0004 + 0.0525 + 0.002), FINGER_CLOSED, "set down"),
                Waypoint(1.2, (cx1, ry, CARRY), FINGER_CLOSED, "lift")]
    to_mat = [Waypoint(2.0, (MAT_XY[0], MAT_XY[1], CARRY), FINGER_CLOSED, "carry"),
              Waypoint(1.2, (MAT_XY[0], MAT_XY[1], mat_z), FINGER_CLOSED, "lower"),
              Waypoint(0.8, (MAT_XY[0], MAT_XY[1], mat_z), FINGER_OPEN, "let go"),
              Waypoint(1.2, (MAT_XY[0], MAT_XY[1], CARRY), FINGER_OPEN, "up"), Waypoint(2.0, park_l, FINGER_OPEN, "park")]
    if VARIANT == "tip_off":       # jam on the stopper face, then tip it sideways off the rail and carry it to the mat
        low = grab + 0.012
        return {"left": L + [Waypoint(3.0, (rx - 0.040, ry, grab + 0.001), FINGER_CLOSED, "slide to the face"),
                             Waypoint(1.0, (rx - 0.040, ry, low), FINGER_CLOSED, "tip up"),
                             Waypoint(2.5, (MAT_XY[0], MAT_XY[1], low), FINGER_CLOSED, "carry sideways"),
                             Waypoint(1.0, (MAT_XY[0], MAT_XY[1], mat_z), FINGER_CLOSED, "lower"),
                             Waypoint(0.8, (MAT_XY[0], MAT_XY[1], mat_z), FINGER_OPEN, "let go"),
                             Waypoint(1.2, (MAT_XY[0], MAT_XY[1], CARRY), FINGER_OPEN, "up"),
                             Waypoint(2.0, park_l, FINGER_OPEN, "park")],
                "right": [Waypoint(1.0, park_r, FINGER_OPEN, "park")]}
    if VARIANT == "hold":          # still gripping the carriage on the mat at the end
        return {"left": L + off_rail + to_mat[:2] + [Waypoint(8.0, (MAT_XY[0], MAT_XY[1], mat_z), FINGER_CLOSED, "keep holding")],
                "right": R}
    if VARIANT == "nothing":
        return {"left": [Waypoint(1.0, park_l, FINGER_OPEN, "park")], "right": [Waypoint(1.0, park_r, FINGER_OPEN, "park")]}
    if VARIANT == "right_only":
        return {"left": [Waypoint(1.0, park_l, FINGER_OPEN, "park")], "right": R}
    if VARIANT == "left_only":     # slide without holding the stopper: must jam
        return {"left": L + off_rail + to_mat, "right": [Waypoint(1.0, park_r, FINGER_OPEN, "park")]}
    if VARIANT == "lift_over":     # lift over the stopper instead of sliding past it
        up = TOP + PLINTH + 0.003 + 0.0525 + 0.05
        Lw = L + [Waypoint(1.5, (cx0, ry, up), FINGER_CLOSED, "lift"),
                  Waypoint(3.0, (cx1, ry, up), FINGER_CLOSED, "over the stopper"),
                  Waypoint(1.5, (cx1, ry, TOP + 0.0004 + 0.0525 + 0.002), FINGER_CLOSED, "set down"),
                  Waypoint(1.2, (cx1, ry, CARRY), FINGER_CLOSED, "lift")]
        return {"left": Lw + to_mat, "right": [Waypoint(1.0, park_r, FINGER_OPEN, "park")]}
    if VARIANT == "lift_restore":  # break the rule, put the carriage back, then do the job properly
        up = TOP + PLINTH + 0.003 + 0.0525 + 0.05
        Lw = L + [Waypoint(1.5, (cx0, ry, up), FINGER_CLOSED, "lift"), Waypoint(1.5, (cx0, ry, grab), FINGER_CLOSED, "restore"),
                  Waypoint(0.8, (cx0, ry, grab), FINGER_OPEN, "release"), Waypoint(1.0, (cx0, ry, CARRY), FINGER_OPEN, "up"),
                  Waypoint(1.0, (cx0, ry, grab), FINGER_OPEN, "down"), Waypoint(0.8, (cx0, ry, grab), FINGER_CLOSED, "regrip")]
        Rw = [Waypoint(18.0, park_r, FINGER_OPEN, "wait")] + R
        return {"left": Lw + [Waypoint(17.0, (cx0, ry, grab), FINGER_CLOSED, "wait")] + off_rail + to_mat, "right": Rw}
    if VARIANT == "elsewhere":
        to_mat = [Waypoint(2.0, (0.34, 0.08, CARRY), FINGER_CLOSED, "carry"), Waypoint(1.2, (0.34, 0.08, mat_z - 0.0035), FINGER_CLOSED, "lower"),
                  Waypoint(0.8, (0.34, 0.08, mat_z), FINGER_OPEN, "let go"), Waypoint(1.2, (0.34, 0.08, CARRY), FINGER_OPEN, "up"),
                  Waypoint(2.0, park_l, FINGER_OPEN, "park")]
    return {"left": L + off_rail + to_mat, "right": R}
