"""On the station table, right of the midline: a wheelchair seen from above (dark seat pan between two side guards,
a backrest on the far side, a wheel outside each guard). Left of it, on the table, lies a contoured pressure-relief
cushion: a blue foam base with a light-grey raised ridge across one end and a dark sunken well at the other, and a
carry handle standing in the middle. It lies backwards: well toward the robot.

The well goes under the sit bones, so it must face the backrest; fitted backwards, the cushion loads the sit bones and
causes the pressure injuries it is meant to prevent. The robot has to turn it round (two quarter turns, regrasping in
between) and lower it between the side guards, flat on the seat.

Run it:  inspect-robots run --embodiment yam_arena -E task=wheelchair_cushion.py --scorer task_score ...
Debug:   TASK_DEBUG=1 prints the cushion's pose and score every 2 s.
"""

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from partkit import _part_state, on_step, part  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DEBUG = bool(os.environ.get("TASK_DEBUG"))

BLUE, RIDGE, WELL, HANDLE = (0.15, 0.35, 0.75), (0.88, 0.88, 0.86), (0.04, 0.05, 0.12), (0.25, 0.25, 0.27)
VINYL, CHROME, TYRE = (0.12, 0.12, 0.13), (0.72, 0.73, 0.76), (0.08, 0.08, 0.08)
BACK = (0.26, 0.30, 0.38)  # backrest upholstery: slate, so it stands apart from the black seat

# The wheelchair (fixed): seat pan 22 x 22 cm with its top 5 cm up, side guards 10 cm tall leaving a 22 cm gap,
# backrest on the far (+x) edge, wheels 22 cm across outside the guards. x away from the robot, y to its left.
SX, SY = 0.41, -0.19
PAN, PAN_H = 0.22, 0.05
SEAT_TOP = TOP + PAN_H
GUARD_T, GUARD_H = 0.015, 0.10
_GY = PAN / 2 + GUARD_T / 2  # guard centre from the seat centre line
WHEELCHAIR = [
    Box("seat_pan", pos=(SX, SY, TOP + PAN_H / 2), size=(PAN, PAN, PAN_H), color=VINYL),
    *[Box(f"side_guard_{n}", pos=(SX, SY + s * _GY, TOP + GUARD_H / 2), size=(PAN, GUARD_T, GUARD_H), color=CHROME)
      for n, s in (("left", 1), ("right", -1))],
    Box("backrest", pos=(SX + PAN / 2 + 0.0125, SY, TOP + 0.12), size=(0.025, PAN, 0.24), color=BACK),
    *[Box(f"back_post_{n}", pos=(SX + PAN / 2 + 0.0125, SY + s * _GY, TOP + 0.13), size=(0.025, GUARD_T, 0.26),
          color=CHROME) for n, s in (("left", 1), ("right", -1))],
    *[Box(f"push_handle_{n}", pos=(SX + PAN / 2 + 0.06, SY + s * _GY, TOP + 0.25), size=(0.07, 0.02, 0.02),
          color=TYRE) for n, s in (("left", 1), ("right", -1))],
    # Wheels seen edge-on from above: long dark bars outside the guards, kept 2 cm clear of the cushion's start.
    *[Box(f"wheel_{n}", pos=(SX + 0.03, SY + s * (_GY + 0.025), TOP + 0.02), size=(0.28, 0.025, 0.04), color=TYRE)
      for n, s in (("left", 1), ("right", -1))],
    # Footrests: two low plates in front of the seat on chrome hangers.
    *[Box(f"footrest_{i}", pos=(SX - PAN / 2 - 0.035, SY + dy, TOP + 0.0075), size=(0.04, 0.05, 0.015), color=TYRE)
      for i, dy in enumerate((0.05, -0.05))],
    *[Box(f"footrest_hanger_{i}", pos=(SX - PAN / 2 - 0.0075, SY + dy, TOP + 0.025), size=(0.015, 0.012, 0.05),
          color=CHROME) for i, dy in enumerate((0.05, -0.05))],
]

# The cushion: local x runs ridge (-x) -> well (+x); origin at the base's centre. The handle stands in the middle,
# 2 x 2 cm, so the fingers can close on it from either side.
BASE = (0.16, 0.18, 0.03)  # front-to-back, side-to-side, thick
RIDGE_H = 0.02
HANDLE_W, HANDLE_H = 0.02, 0.06
START = (0.19, 0.03)
START_YAW = math.pi  # backwards: well toward the robot
CUSHION = part("cushion", pos=(*START, TOP + BASE[2] / 2 + 0.001), yaw=START_YAW, mass=0.25, color=BLUE,
               boxes=[((0, 0, 0), BASE, BLUE),
                      ((-0.055, 0, BASE[2] / 2 + RIDGE_H / 2), (0.05, BASE[1], RIDGE_H), RIDGE),
                      ((0.0425, 0, BASE[2] / 2 + 0.0005), (0.065, 0.12, 0.001), WELL),
                      ((0, 0, BASE[2] / 2 + HANDLE_H / 2), (HANDLE_W, HANDLE_W, HANDLE_H), HANDLE)])
OBJECTS = [*WHEELCHAIR, CUSHION]

# -- scoring --------------------------------------------------------------------------------------------------------

MAX_YAW_DEG, MAX_TILT_DEG = 20.0, 5.0
SEATED_M = 0.005  # bottom within 5 mm of the seat pan (or of the table, for 0.25)
CENTRED_M = 0.04  # front-to-back slack on the 22 cm pan; side-to-side the guards hold it


def _pose(o):
    """Yaw of the well (local +x) from world +x, and tilt from vertical, in degrees."""
    x, y, z, w = (float(v) for v in o.quat_xyzw)
    yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))
    return yaw, tilt


def score(objects):
    """0 unless the well faces the backrest (yaw within 20 deg) and the cushion is at rest. Then 1.0 if it sits flat on
    the seat pan (bottom within 5 mm, tilt <= 5 deg, centre within 4 cm of the pan's centre front-to-back); 0.5 if its
    centre is over the pan but it isn't seated; 0.25 if it lies flat on the table."""
    c = objects[CUSHION.name]
    yaw, tilt = _pose(c)
    if abs(yaw) > MAX_YAW_DEG or not c.resting():
        return 0.0
    x, y, z = (float(v) for v in c.pos)
    bottom = z - BASE[2] / 2
    flat = tilt <= MAX_TILT_DEG
    if abs(x - SX) <= PAN / 2 and abs(y - SY) <= PAN / 2:
        return 1.0 if flat and abs(bottom - SEAT_TOP) <= SEATED_M and abs(x - SX) <= CENTRED_M else 0.5
    return 0.25 if flat and abs(bottom - TOP) <= SEATED_M else 0.0


_S = {}  # debug step counter only: scoring is final-state, nothing to clear on reset


@on_step
def _debug(environment):
    if not DEBUG:
        return
    _S["step"] = _S.get("step", 0) + 1
    if _S["step"] % 40:
        return
    c = _part_state(environment, CUSHION)
    yaw, tilt = _pose(c)
    print(f"TASK t={_S['step'] / 20:.0f}s cushion x={c.pos[0]:.4f} y={c.pos[1]:.4f} "
          f"bottom={(c.pos[2] - BASE[2] / 2 - TOP) * 1e3:+.1f}mm yaw={yaw:+.1f} tilt={tilt:.1f} "
          f"score={score({CUSHION.name: c})}", flush=True)


# -- the scripted demo (right arm) ----------------------------------------------------------------------------------

# "turn": lift the cushion in place, turn it a quarter turn (+90 deg), set it down; "seat": carry it to the seat as it
# is and lower it between the guards. run_checks.sh swaps this line for the variants.
DEMO_PLAN = ("turn", "turn", "seat")

GRIP_ABOVE = BASE[2] + 0.03 + 0.008  # TCP above the cushion's bottom: pad tips 3 cm up the handle, 1 cm over the ridge
OPEN = -0.018  # 36 mm: 8 mm each side of the handle
GRIP = -(HANDLE_W - 0.008) / 2  # close 8 mm narrower than the handle
TURN_LIFT = 0.06  # cushion bottom above the table while it turns
CARRY = GUARD_H + 0.02  # cushion bottom above the table while it is carried: 2 cm over the guards
ENTRY = GUARD_H + 0.005 - PAN_H  # pause with the cushion's bottom 5 mm over the guard tops, then the slow descent
PLACE_AIM = (0.0, 0.0)  # calibration knobs for the probe
QUARTER = math.pi / 2
CLEAR_ABOVE = 0.08  # retreat: TCP this far above the grip height before moving on


def _grab(x, y, bottom, label):
    z = bottom + GRIP_ABOVE
    return [Waypoint(1.5, (x, y, z + CLEAR_ABOVE), OPEN, f"above {label}"),
            Waypoint(0.8, (x, y, z + CLEAR_ABOVE), OPEN, "settle above"),
            Waypoint(1.5, (x, y, z), OPEN, f"down to {label}"),
            Waypoint(0.6, (x, y, z), OPEN, "settle"),
            Waypoint(1.5, (x, y, z), GRIP, "close")]


def _let_go(x, y, bottom, yaw):
    z = bottom + GRIP_ABOVE
    return [Waypoint(0.6, (x, y, z), GRIP, "settle", yaw),
            Waypoint(1.2, (x, y, z), OPEN, "release", yaw),
            Waypoint(1.2, (x, y, z + CLEAR_ABOVE), OPEN, "retreat", yaw)]


def demo():
    """Two quarter turns in place (lift 6 cm, turn, set down, regrasp), so the well ends up toward the backrest; then
    carry the cushion over the seat, pause above it and lower it slowly between the guards. Cushion and seat positions
    come from the top image; the seat's height and guard width are the chair's fixed geometry."""
    x, y = START
    steps = []
    for action in DEMO_PLAN:
        steps += _grab(x, y, TOP, "the handle")
        if action == "turn":
            steps += [Waypoint(1.2, (x, y, TOP + TURN_LIFT + GRIP_ABOVE), GRIP, "lift"),
                      Waypoint(2.0, (x, y, TOP + TURN_LIFT + GRIP_ABOVE), GRIP, "quarter turn", QUARTER),
                      Waypoint(0.6, (x, y, TOP + TURN_LIFT + GRIP_ABOVE), GRIP, "settle", QUARTER),
                      Waypoint(1.5, (x, y, TOP + 0.003 + GRIP_ABOVE), GRIP, "set down", QUARTER)]
            steps += _let_go(x, y, TOP + 0.003, QUARTER)
            continue
        sx, sy = SX + PLACE_AIM[0], SY + PLACE_AIM[1]
        steps += [Waypoint(1.5, (x, y, TOP + CARRY + GRIP_ABOVE), GRIP, "lift"),
                  Waypoint(2.5, (sx, sy, TOP + CARRY + GRIP_ABOVE), GRIP, "carry over the seat"),
                  Waypoint(0.8, (sx, sy, TOP + CARRY + GRIP_ABOVE), GRIP, "settle over the seat"),
                  Waypoint(1.2, (sx, sy, SEAT_TOP + ENTRY + GRIP_ABOVE), GRIP, "lower to the entry"),
                  Waypoint(0.8, (sx, sy, SEAT_TOP + ENTRY + GRIP_ABOVE), GRIP, "settle at the entry"),
                  Waypoint(3.0, (sx, sy, SEAT_TOP + 0.003 + GRIP_ABOVE), GRIP, "set down between the guards")]
        steps += _let_go(sx, sy, SEAT_TOP + 0.003, 0.0)
        x, y = sx, sy
    steps.append(Waypoint(1.5, (0.12, -0.42, TOP + 0.25), FINGER_OPEN, "move clear"))
    return {"right": steps}
