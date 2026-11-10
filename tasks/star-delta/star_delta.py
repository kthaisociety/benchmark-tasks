"""On the station table, in front of the right arm: the open terminal box of a 230/400 V three-phase motor. Its
terminal board has six brass studs in the IEC layout: the line row U1 V1 W1 (far from the robot, where the supply
leads come in) and the winding-end row W2 U2 V2 (near), 4.5 cm apart both ways. Three brass links stand on the studs
in delta: U1-W2, V1-U2 and W1-V2, each across a column. A parking tray stands to the right of the box.

The motor is going onto a 400 V supply. On a 230/400 V nameplate 400 V means star: the delta links come off, two of
them bridge the near row (W2-U2 and U2-V2, stacked at U2) and the third is put away. Left in delta, each winding gets
root 3 times its rated voltage and burns out.

Run it:  inspect-robots run --embodiment yam_arena -E task=star_delta.py --scorer task_score ...
Debug:   TASK_DEBUG=1 prints every link's place and pose every 2 s.
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

GREY, BLACK, BRASS = (0.62, 0.64, 0.66), (0.08, 0.08, 0.08), (0.80, 0.62, 0.25)
BOX_GREY, MOTOR_BLUE, STEEL = (0.50, 0.53, 0.56), (0.12, 0.27, 0.50), (0.72, 0.73, 0.75)

# The terminal box: a 16 x 16 cm board 3 cm thick with the studs, inside 2 cm walls (below the finger pads, 2 cm or
# more from any finger), a cable gland in the left wall. Part frame origin at its centre (top face at z = +0.015).
# x away from the robot, y to the robot's left.
BX, BY = 0.34, -0.12
BOARD_T = 0.03
FLOOR = TOP + BOARD_T
PITCH = 0.045
STUD_D, STUD_H = 0.008, 0.015  # calibration knob: a thinner stud gives more clearance in the link's holes
_ROW = {"1": PITCH / 2, "2": -PITCH / 2}  # line row far, winding-end row near
_COL = {"1": {"U": PITCH, "V": 0.0, "W": -PITCH}, "2": {"W": PITCH, "U": 0.0, "V": -PITCH}}  # W2 under U1, ...
STUD_LOCAL = {f"{p}{r}": (_ROW[r], y) for r in "12" for p, y in _COL[r].items()}
STUD = {s: (BX + x, BY + y) for s, (x, y) in STUD_LOCAL.items()}

_LEAD_X0, _LEAD_X1 = PITCH / 2 + 0.0135, 0.065  # supply leads: from just past the line studs to a gland strip
BOARD = part("terminal_board", pos=(BX, BY, TOP + BOARD_T / 2), mass=20.0, color=GREY,
             boxes=[((0, 0, 0), (0.16, 0.16, BOARD_T), GREY),
                    *[((((_LEAD_X0 + _LEAD_X1) / 2), y, 0.0165), (_LEAD_X1 - _LEAD_X0, 0.006, 0.003), BLACK)
                      for y in _COL["1"].values()],
                    ((_LEAD_X1, 0.015, 0.0165), (0.006, 0.13, 0.003), BLACK),
                    *[((s * 0.077, 0, 0.025), (0.006, 0.16, 0.02), BOX_GREY) for s in (-1, 1)],  # box walls
                    *[((0, s * 0.077, 0.025), (0.148, 0.006, 0.02), BOX_GREY) for s in (-1, 1)],
                    ((_LEAD_X1, 0.082, 0.025), (0.024, 0.012, 0.024), BLACK)],  # cable gland
             cylinders=[((x, y, BOARD_T / 2 + STUD_H / 2), STUD_D / 2, STUD_H, BRASS)
                        for x, y in STUD_LOCAL.values()])

# The links: a brass plate with two square holes (2 mm clearance round a stud) and an upright grip tab between them.
# Local frame: long axis x, origin at the plate's centre.
LINK_L, LINK_W, LINK_T = 0.065, 0.020, 0.004
HOLE = 0.012  # calibration knob: wider holes forgive more aim error
TAB = (0.020, 0.015, 0.050)  # along the link (the fingers close on this), across it, tall
_h, _hh = PITCH / 2, HOLE / 2
_rail, _end = (LINK_W - HOLE) / 2, LINK_L / 2 - PITCH / 2 - HOLE / 2
_LINK_BOXES = [((0, 0, 0), (2 * (_h - _hh), LINK_W, LINK_T), BRASS),
               *[((s * (_h + _hh + _end / 2), 0, 0), (_end, LINK_W, LINK_T), BRASS) for s in (-1, 1)],
               *[((s * _h, t * (_hh + _rail / 2), 0), (HOLE, _rail, LINK_T), BRASS) for s in (-1, 1) for t in (-1, 1)],
               ((0, 0, LINK_T / 2 + TAB[2] / 2), TAB, BLACK)]  # the tab: an insulated handle
DELTA = {"u1w2": ("U1", "W2"), "v1u2": ("V1", "U2"), "w1v2": ("W1", "V2")}


def _mid(a, b):
    return ((STUD[a][0] + STUD[b][0]) / 2, (STUD[a][1] + STUD[b][1]) / 2)


LINKS = {n: part(f"link_{n}", pos=(*_mid(*pair), FLOOR + LINK_T / 2 + 0.001), mass=0.04, color=BRASS,
                 boxes=_LINK_BOXES) for n, pair in DELTA.items()}

# The terminal box's lid, lying upside down right of the box (12 x 12 cm, 1.2 cm rim): the spare link goes in it.
TX, TY = 0.38, -0.28
TRAY_FLOOR = TOP + 0.006
TRAY_IN = 0.055  # inner half-width
TRAY = part("box_lid", pos=(TX, TY, TOP + 0.003), mass=2.0, color=BOX_GREY,
            boxes=[((0, 0, 0), (0.12, 0.12, 0.006), BOX_GREY),
                   *[((s * 0.0575, 0, 0.009), (0.005, 0.12, 0.012), BOX_GREY) for s in (-1, 1)],
                   *[((0, s * 0.0575, 0.009), (0.11, 0.005, 0.012), BOX_GREY) for s in (-1, 1)]])

# The motor, lying behind its terminal box with the shaft to the robot's left: a finned frame, a fan cowl on the
# right, the shaft on the left, a nameplate on top. Fixed decoration.
MX, MLEN, MH = 0.49, 0.20, 0.12
DECOR = [
    Box("motor_frame", pos=(MX, BY, TOP + MH / 2), size=(0.11, MLEN, MH), color=MOTOR_BLUE),
    *[Box(f"motor_fin{i}", pos=(MX + dx, BY - 0.02, TOP + MH + 0.004), size=(0.005, MLEN - 0.04, 0.008),
          color=MOTOR_BLUE) for i, dx in enumerate((-0.045, -0.027, -0.009, 0.009, 0.027, 0.045))],
    Box("fan_cowl", pos=(MX, BY - MLEN / 2 - 0.02, TOP + MH / 2), size=(0.10, 0.04, 0.11), color=BLACK),
    Box("shaft", pos=(MX, BY + MLEN / 2 + 0.025, TOP + MH / 2), size=(0.022, 0.05, 0.022), color=STEEL),
    Box("nameplate", pos=(MX, BY + MLEN / 2 - 0.02, TOP + MH + 0.001), size=(0.06, 0.03, 0.002), color=STEEL),
    Box("supply_cable", pos=(BX + _LEAD_X1, (BY + 0.08 + 0.04) / 2, TOP + 0.006), size=(0.012, 0.04 - BY - 0.08, 0.012),
        color=BLACK),
]
OBJECTS = [BOARD, *LINKS.values(), TRAY, *DECOR]

# -- scoring --------------------------------------------------------------------------------------------------------

STAR = (("W2", "U2"), ("U2", "V2"))
SEAT_M = 0.004  # each hole centre within 4 mm of its stud
PLATE_LOW_M = 0.009  # plate bottom within 9 mm of the board (room for the second link stacked at U2)
MAX_TILT_DEG = 8.0


def _axes(q):
    """The link's long axis (local x) and the vertical part of its local z, from an xyzw quaternion."""
    x, y, z, w = (float(v) for v in q)
    return (1 - 2 * (y * y + z * z), 2 * (x * y + w * z)), 1 - 2 * (x * x + y * y)


def _seated(o, pair):
    (ax, ay), up = _axes(o.quat_xyzw)
    cx, cy, cz = (float(v) for v in o.pos)
    holes = [(cx + s * _h * ax, cy + s * _h * ay) for s in (-1, 1)]
    a, b = STUD[pair[0]], STUD[pair[1]]
    near = lambda h, s: math.dist(h, s) <= SEAT_M  # noqa: E731
    return ((near(holes[0], a) and near(holes[1], b)) or (near(holes[0], b) and near(holes[1], a))) \
        and cz - LINK_T / 2 <= FLOOR + PLATE_LOW_M and math.degrees(math.acos(max(-1.0, min(1.0, up)))) <= MAX_TILT_DEG


def _place(o):
    """'tray', a star pair, or None (anywhere else: still in delta, across the line row, loose, dropped, held)."""
    cx, cy, cz = (float(v) for v in o.pos)
    if abs(cx - TX) <= TRAY_IN and abs(cy - TY) <= TRAY_IN and cz <= TRAY_FLOOR + 0.02:
        return "tray"
    return next((p for p in STAR if _seated(o, p)), None)


def score(objects):
    """Every link must be on a star pair (W2-U2, U2-V2) or in the tray, or 0: a link left across a column is delta
    (or star plus delta, a short), one across the line row shorts the supply, a loose one may short anything. Then
    1.0 for both star bridges, 0.6 for one, 0.3 for none (all links put away: safe, but the motor isn't connected)."""
    places = [_place(objects[o.name]) for o in LINKS.values()]
    if None in places:
        return 0.0
    return (0.3, 0.6, 1.0)[len({p for p in places if p != "tray"})]


def success(objects):
    """Connected in star with nothing else on the studs; held for 1 s, the trial ends."""
    return score(objects) == 1.0


@on_step
def _debug(environment):
    if not DEBUG:
        return
    _S["step"] = _S.get("step", 0) + 1
    if _S["step"] % 40:
        return
    for n, spec in LINKS.items():
        o = _part_state(environment, spec)
        (ax, ay), up = _axes(o.quat_xyzw)
        print(f"TASK t={_S['step'] / 20:.0f}s {n} place={_place(o)} x={o.pos[0]:.4f} y={o.pos[1]:.4f} "
              f"bottom={(o.pos[2] - LINK_T / 2 - FLOOR) * 1e3:+.1f}mm yaw={math.degrees(math.atan2(ay, ax)):+.1f} "
              f"tilt={math.degrees(math.acos(max(-1.0, min(1.0, up)))):.1f}", flush=True)


_S = {}  # debug step counter only; scoring is final-state, so there is no trial record to clear on reset

# -- the scripted demo ----------------------------------------------------------------------------------------------

# (link, target): target is a stud pair "A-B", "tray" or "table". run_checks.sh swaps this line for the variants.
DEMO_PLAN = (("w1v2", "tray"), ("v1u2", "U2-V2"), ("u1w2", "W2-U2"))

GRIP_ABOVE = 0.029  # TCP above the plate's bottom: pad tips 21 mm up the link, 6 mm above the stud tops
TRAVEL_Z = FLOOR + STUD_H + 0.02 + GRIP_ABOVE  # carried: plate bottom 2 cm above the stud tops
ENTRY = STUD_H + 0.004  # pause with the plate bottom 4 mm above the stud tops before the slow set-down
# The fingers close along the link (across the tab's 2 cm), so a brush pushes the link along its length, where the two
# studs hold it. Probes 1-2 closed across the link: a brush rolled it about its long axis (24-32 deg) and jammed it.
TAB_OPEN = -0.022  # 44 mm open: 12 mm each side of the tab; the pads pass 6 mm over the stud tops
RELEASE_OPEN = -0.014  # 28 mm to let go: along a row the first star link's tab is ~3 cm from a fully opened finger
SQUEEZE = 0.008  # close to 8 mm narrower than the tab, ~20 N per finger, as in access-panel-closeout
GRIP = -(TAB[0] - SQUEEZE) / 2
TRAY_SLOTS = (0.0, -0.04, 0.04)  # y offsets for the 1st, 2nd and 3rd link put in the tray
TABLE_XY = (0.20, -0.12)  # in front of the board
# Calibration knobs for the probe: the wrist settles past a commanded yaw, and the held link can sit off-centre.
PLACE_AIM = (0.0, 0.0)
YAW_TRIM = 0.0


def _move(pick, place, pick_yaw, yaw):
    """Top-down pick by the tab at ``pick_yaw`` and place at ``yaw``, paced like access-panel-closeout's: settle before
    every close and release, pause at the entry, slow last descent. 17.0 s."""
    (px, py, pz), (qx, qy, qz) = pick, place
    return [
        Waypoint(1.5, (px, py, TRAVEL_Z), TAB_OPEN, "to the link", pick_yaw),
        Waypoint(0.8, (px, py, TRAVEL_Z), TAB_OPEN, "settle above the link", pick_yaw),
        Waypoint(1.5, (px, py, pz), TAB_OPEN, "descend", pick_yaw),
        Waypoint(0.6, (px, py, pz), TAB_OPEN, "settle", pick_yaw),
        Waypoint(1.5, (px, py, pz), GRIP, "close", pick_yaw),
        Waypoint(1.2, (px, py, TRAVEL_Z), GRIP, "lift off the studs", pick_yaw),
        Waypoint(2.0, (qx, qy, TRAVEL_Z), GRIP, "carry", yaw),
        Waypoint(0.6, (qx, qy, TRAVEL_Z), GRIP, "settle over the place", yaw),
        Waypoint(1.2, (qx, qy, qz + ENTRY), GRIP, "lower to the entry", yaw),
        Waypoint(0.8, (qx, qy, qz + ENTRY), GRIP, "settle at the entry", yaw),
        Waypoint(2.0, (qx, qy, qz), GRIP, "set down", yaw),
        Waypoint(0.8, (qx, qy, qz), GRIP, "settle", yaw),
        Waypoint(1.5, (qx, qy, qz), RELEASE_OPEN, "release", yaw),
        Waypoint(1.0, (qx, qy, TRAVEL_Z), RELEASE_OPEN, "retreat", yaw),
    ]


def demo():
    """The gripper closes along each link: yaw +90 deg for a link lying along x, 0 for one along y. The right arm
    parks W1-V2's link in the tray, then lifts V1-U2's, turns it a quarter turn and drops it over U2-V2, then does the same with U1-W2's over W2-U2, stacking it on the first at U2. Link positions come from the top
    image; the stud positions are the box's fixed geometry. ~50 s."""
    along_x = math.pi / 2  # gripper yaw for a link lying along x
    at = {n: (*_mid(*pair), FLOOR, along_x) for n, pair in DELTA.items()}  # each link's x, y, plate bottom z, grip yaw
    used, parked, steps = set(), 0, []
    for n, target in DEMO_PLAN:
        if target == "tray":
            dest, yaw = (TX, TY + TRAY_SLOTS[parked], TRAY_FLOOR), along_x
            parked += 1
        elif target == "table":
            dest, yaw = (*TABLE_XY, TOP), along_x
        else:
            a, b = target.split("-")
            (mx, my), stack = _mid(a, b), LINK_T / 2 if used & {a, b} else 0.0
            dest = (mx + PLACE_AIM[0], my + PLACE_AIM[1], FLOOR + 0.002 + stack)
            yaw = (0.0 if a[1] == b[1] else along_x) + YAW_TRIM  # along a row (y): a quarter turn from the pick
            used |= {a, b}
        x, y, z, pick_yaw = at[n]
        steps += _move((x, y, z + GRIP_ABOVE), (dest[0], dest[1], dest[2] + GRIP_ABOVE), pick_yaw, yaw)
        at[n] = (*dest, yaw)
    steps.append(Waypoint(1.5, (0.12, -0.42, TOP + 0.20), FINGER_OPEN, "move clear"))
    return {"right": steps}
