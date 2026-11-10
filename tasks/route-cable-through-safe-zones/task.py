"""A section of a factory-built timber-frame wall panel lies flat on the bench, its floor edge (a timber sole
plate) nearest the robot, so up the wall is away from the robot. Its 18 mm OSB face is 36 cm up the wall by 40 cm
along it. Two accessory boxes are fixed to the face, each 80 mm square: an existing socket (white faceplate)
low on the left, and an empty back box (grey, open) 19 cm higher and 23.75 cm to the right. Three rigid grey
PVC conduit lengths, 17 cm long with a 25 mm square section, lie on the bench: two to the left of the panel
and one to the right. The panel will be boarded over, hiding whatever conduit is laid on it.
"""

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

import scoring as S  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
assert abs(TOP - S.TOP) < 1e-9
SCORE_OVER = "end"
INSTRUCTION = ("Run conduit for a new cable from the socket to the empty back box on this wall panel, using the "
               "grey conduit lengths. The panel is for a house in the UK and will be boarded over. Leave any "
               "length you don't need on the bench.")

OSB = (0.78, 0.64, 0.42)
TIMBER = (0.66, 0.50, 0.32)
WHITE = (0.96, 0.96, 0.94)
DARK = (0.12, 0.12, 0.12)
BACKBOX = (0.55, 0.57, 0.60)
CONDUIT = (0.42, 0.44, 0.47)

# Conduit lengths at the start: on the bench beside the panel, lying along x (up the wall).
START = {
    "conduit_a": (0.385, 0.26),
    "conduit_b": (0.385, -0.26),
    "conduit_c": (0.385, 0.32),
}


def _scene():
    px0, px1 = S.PANEL_X
    py0, py1 = S.PANEL_Y
    sx, sy = S.SOCKET_XY
    nx, ny = S.NEW_BOX_XY
    box_z = S.PANEL_TOP + S.BOX_H / 2
    face_z = S.PANEL_TOP + S.BOX_H
    objs = [
        Box("wall_panel", pos=((px0 + px1) / 2, (py0 + py1) / 2, TOP + S.PANEL_T / 2),
            size=(px1 - px0, py1 - py0, S.PANEL_T), color=OSB),
        Box("sole_plate", pos=(px0 - 0.0225, (py0 + py1) / 2, TOP + 0.0225),
            size=(0.045, py1 - py0, 0.045), color=TIMBER),
        Box("socket", pos=(sx, sy, box_z), size=(S.BOX, S.BOX, S.BOX_H), color=WHITE),
        # Two pin slots and an earth slot on the faceplate, and a rocker switch.
        Box("socket_slot_l", pos=(sx - 0.008, sy + 0.012, face_z + 0.0005), size=(0.004, 0.009, 0.001), color=DARK),
        Box("socket_slot_r", pos=(sx - 0.008, sy - 0.012, face_z + 0.0005), size=(0.004, 0.009, 0.001), color=DARK),
        Box("socket_slot_e", pos=(sx + 0.012, sy, face_z + 0.0005), size=(0.009, 0.004, 0.001), color=DARK),
        Box("socket_rocker", pos=(sx - 0.008, sy - 0.03, face_z + 0.002), size=(0.012, 0.008, 0.004), color=WHITE),
        Box("new_box", pos=(nx, ny, box_z), size=(S.BOX, S.BOX, S.BOX_H), color=BACKBOX),
        # The open mouth of the back box: a dark recess 64 mm square.
        Box("new_box_mouth", pos=(nx, ny, face_z + 0.0005), size=(0.064, 0.064, 0.001), color=(0.20, 0.21, 0.23)),
    ]
    for name in S.PIPES:
        x, y = START[name]
        objs.append(Box(name, pos=(x, y, TOP + S.PIPE_H / 2 + 0.001), size=(S.PIPE_L, S.PIPE_W, S.PIPE_H),
                        color=CONDUIT, mass=0.06))
    return objs


OBJECTS = _scene()


def poses(objects):
    out = {}
    for name in S.PIPES:
        o = objects[name]
        yaw, tilt = S.yaw_tilt(o.quat_xyzw)
        out[name] = S.Pose(float(o.pos[0]), float(o.pos[1]), yaw, tilt, float(o.low[2]), o.resting())
    return out


_DEBUG = {"step": 0}


def _debug(p, value):
    _DEBUG["step"] += 1
    if _DEBUG["step"] % 40:
        return
    rows = [f"{n[-1]} ({q.x:.3f},{q.y:.3f}) yaw {math.degrees(q.yaw):+.0f} tilt {q.tilt_deg:.0f} "
            f"low {100 * (q.low_z - S.PANEL_TOP):+.1f}cm on {int(S.on_panel(q))} zone {S.zone(q)}"
            for n, q in p.items()]
    print(f"[csz] step {_DEBUG['step']} score {value}: " + " | ".join(rows) + f" faults {S.faults(p)}",
          file=sys.stderr, flush=True)


def score(objects):
    p = poses(objects)
    value = S.grade(p)
    if os.environ.get("CSZ_DEBUG"):
        _debug(p, value)
    return value


# ---------------------------------------------------------------- scripted demo and shortcut variants

TRAVEL = TOP + 0.10
PAD_ABOVE = 0.008                         # finger-pad tips 8 mm above the surface the length rests on
PICK_Z = TOP + PAD_TIP_PAST_TCP + PAD_ABOVE
PLACE_Z = S.PANEL_TOP + PAD_TIP_PAST_TCP + PAD_ABOVE + 0.003   # released 3 mm above the face
GRIP = -(S.PIPE_W - 0.012) / 2            # 12 mm narrower than the length: a firm, centring grip
WIDE = -(S.PIPE_W + 0.02) / 2             # pads 10 mm clear of each side
RELEASE = -(S.PIPE_W + 0.012) / 2

# Where each length goes (centre x, centre y, yaw of the length). Both L routes use two lengths.
G = 0.0075                                # 7.5 mm joint gaps in the plan
SX, SY = S.SOCKET_XY
NX, NY = S.NEW_BOX_XY
HB, HL, HW = S.BOX / 2, S.PIPE_L / 2, S.PIPE_W / 2
# Route B: along the wall level with the socket, then up the wall in line with the back box.
ROUTE_B = {
    "row": (SX, SY - HB - G - HL, math.pi / 2),            # horizontal leg, socket row
    "column": (NX - HB - G - HL, NY, 0.0),                 # vertical leg, back-box column, past the row
}
# Route A: up the wall in line with the socket, then along the wall level with the back box.
ROUTE_A = {
    "column": (SX + HB + G + HL, SY, 0.0),
    "row": (NX, NY + HB + G + HL, math.pi / 2),
}
DIAGONAL = ((SX + NX) / 2, (SY + NY) / 2, math.atan2(NY - SY, NX - SX))


def move(name, place, arm):
    """Pick a length at its middle from the bench and lay it flat at place = (x, y, yaw of the length)."""
    px, py = START[name]
    qx, qy, yaw = place
    a = 0.0                                    # gripper yaw 0 closes along y: across a length lying along x
    b = yaw                                    # the length turns with the gripper
    if b > 0:
        b -= math.pi                           # same line on the wall; every turn is clockwise, at most 90 deg
    return [
        Waypoint(1.5, (px, py, TRAVEL), WIDE, f"{name}: above", a),
        Waypoint(1.5, (px, py, PICK_Z), WIDE, "descend", a),
        Waypoint(0.4, (px, py, PICK_Z), WIDE, "settle", a),
        Waypoint(0.8, (px, py, PICK_Z), GRIP, "close", a),
        Waypoint(1.5, (px, py, TRAVEL), GRIP, "lift", a),
        Waypoint(2.5, (qx, qy, TRAVEL), GRIP, "carry and turn", b),
        Waypoint(1.5, (qx, qy, PLACE_Z + 0.02), GRIP, "above the spot", b),
        Waypoint(1.0, (qx, qy, PLACE_Z), GRIP, "lower", b),
        Waypoint(0.4, (qx, qy, PLACE_Z), GRIP, "settle", b),
        Waypoint(0.6, (qx, qy, PLACE_Z), RELEASE, "release", b),
        Waypoint(1.0, (qx, qy, TRAVEL), RELEASE, "retreat", b),
    ]


MOVE_S = sum(w.duration for w in move("conduit_a", ROUTE_B["row"], "left"))


def _wait(seconds, arm):
    return [Waypoint(seconds, (0.25, 0.25 if arm == "left" else -0.25, TRAVEL), FINGER_OPEN, "wait", 0.0)]


def _park(arm):
    return [Waypoint(1.5, (0.20, 0.30 if arm == "left" else -0.30, TRAVEL), FINGER_OPEN, "park", 0.0)]


def _shift(place, dx=0.0, dy=0.0):
    return (place[0] + dx, place[1] + dy, place[2])


VARIANTS = {
    # Full score: right arm lays the back-box column, then the left arm the socket row. conduit_c stays on the bench.
    "full": lambda: (_wait(MOVE_S, "left") + move("conduit_a", ROUTE_B["row"], "left") + _park("left"),
                     move("conduit_b", ROUTE_B["column"], "right") + _park("right")),
    # The other legal L: up the wall in line with the socket, then along level with the back box (left arm).
    "route_a": lambda: (move("conduit_a", ROUTE_A["column"], "left") + move("conduit_c", ROUTE_A["row"], "left")
                        + _park("left"), _park("right")),
    # The tempting short cut: one length straight across from box to box. Joined, but out of zone: 0.
    "diagonal": lambda: (_park("left"), move("conduit_b", DIAGONAL, "right") + _park("right")),
    # A correct L, plus the spare length left lying on the panel out of zone: 0.
    "stray": lambda: (_wait(MOVE_S, "left") + move("conduit_a", ROUTE_B["row"], "left")
                      + move("conduit_c", (0.30, -0.03, 0.0), "left") + _park("left"),
                      move("conduit_b", ROUTE_B["column"], "right") + _park("right")),
    # Only the back-box column, butted to the box: 0.25.
    "one_leg": lambda: (_park("left"), move("conduit_b", ROUTE_B["column"], "right") + _park("right")),
    # The L laid in zone but with the column 30 mm short of the back box (a 37.5 mm open joint): 0.5.
    "open_joint": lambda: (_wait(MOVE_S, "left") + move("conduit_a", ROUTE_B["row"], "left") + _park("left"),
                           move("conduit_b", _shift(ROUTE_B["column"], dx=-0.03), "right") + _park("right")),
    "nothing": lambda: (_park("left"), _park("right")),
    # Probe of the risky mechanic: one grip, a quarter turn and a placement beside the socket: 0.25.
    "probe": lambda: (move("conduit_a", ROUTE_B["row"], "left") + _park("left"), _park("right")),
}


def demo():
    variant = os.environ.get("CSZ_VARIANT", "full")
    if variant not in VARIANTS:
        raise ValueError(f"unknown CSZ_VARIANT {variant!r}; one of {sorted(VARIANTS)}")
    left, right = VARIANTS[variant]()
    return {"left": left, "right": right}


EXPECTED = {"full": 1.0, "route_a": 1.0, "diagonal": 0.0, "stray": 0.0, "one_leg": 0.25, "open_joint": 0.5,
            "nothing": 0.0, "probe": 0.25}
