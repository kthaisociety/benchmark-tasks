"""The front of a bus at the far edge of the table, at 1:6 scale: a black bumper and a white front panel with a dark
windscreen. A grey two-position bike rack stands lowered in front of the bumper, its two U-shaped wheel channels
running left-right across the table: one farther from the bus (nearer the robot) and one nearer the bus. Each
position has a black support arm hinged at the rack's bus end, standing up stowed and leaning back toward the bus,
with a black cross-knob part-way up and a yellow hook at its end. The hook of the position farther from the bus is
at its left end; the hook of the position nearer the bus is at its right end. A support arm swings out and down
over a wheel in its position. A blue bicycle stands upright in a low wheel rail on the pavement in front of the
rack, front wheel to the left. The bike has a black front brake on its fork, just above the front tyre.

Run with run.py (it builds the rack as a jointed body): see README.md.
"""

import math
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bike as B  # noqa: E402
import layout as L  # noqa: E402
import rack as K  # noqa: E402
import scoring  # noqa: E402
from partkit import part  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import Box  # noqa: E402

SCENE = "station"
TOP = L.TOP
SCORE_OVER = "end"
INSTRUCTION = "Load the bike onto the bus's bike rack, ready for the bus to drive off."
BUMPER_BLACK, BUS_WHITE, GLASS, LIGHT, AMBER = (0.06, 0.06, 0.07), (0.93, 0.93, 0.90), (0.12, 0.16, 0.20), \
    (0.98, 0.96, 0.85), (0.95, 0.60, 0.10)
RAIL_GREY = (0.55, 0.55, 0.57)

_rail_x = L.RAIL["x"] + B.TYRE_W / 2 + L.RAIL["gap"] + L.RAIL["wall"] / 2
_rail_len = L.RAIL["y1"] - L.RAIL["y0"]
_bus_mid = (L.BUS["x0"] + L.BUS["x1"]) / 2

OBJECTS = [
    part("bike", pos=(L.BIKE0[0], L.BIKE0[1], TOP + 0.001), boxes=B.boxes(), mass=B.MASS, friction=B.FRICTION,
         color=B.FRAME),
    Box("rack", pos=(L.RACK_ORIGIN[0], L.RACK_ORIGIN[1], TOP + L.RACK_ORIGIN[2]), size=(0.01, 0.01, 0.01),
        mass=K.FRAME_MASS),  # replaced by rack.build() in run.py
    *[Box(f"rail_{n}", pos=(L.RAIL["x"] + s * _rail_x, (L.RAIL["y0"] + L.RAIL["y1"]) / 2, TOP + L.RAIL["h"] / 2),
          size=(L.RAIL["wall"], _rail_len, L.RAIL["h"]), color=RAIL_GREY) for n, s in (("l", -1), ("r", 1))],
    Box("bumper", pos=((L.BUMPER["x0"] + L.BUMPER["x1"]) / 2, 0.0, TOP + L.BUMPER["h"] / 2),
        size=(L.BUMPER["x1"] - L.BUMPER["x0"], 2 * L.BUMPER["half_y"], L.BUMPER["h"]), color=BUMPER_BLACK),
    Box("bus_front", pos=(_bus_mid, 0.0, TOP + L.BUS["h"] / 2), size=(L.BUS["x1"] - L.BUS["x0"], 2 * L.BUS["half_y"], L.BUS["h"]),
        color=BUS_WHITE),
    Box("windscreen", pos=(L.BUS["x0"] - 0.002, 0.0, TOP + 0.215), size=(0.004, 2 * L.BUS["half_y"] - 0.03, 0.15), color=GLASS),
    Box("route_sign", pos=(L.BUS["x0"] - 0.0045, 0.0, TOP + 0.27), size=(0.002, 0.14, 0.025), color=AMBER),
    *[Box(f"headlight_{n}", pos=(L.BUS["x0"] - 0.002, s * 0.22, TOP + 0.075), size=(0.004, 0.05, 0.025), color=LIGHT)
      for n, s in (("l", 1), ("r", -1))],
]

_trial = {"t": scoring.Trial(), "last": 0.0}


def reset_trial():
    _trial["t"], _trial["last"] = scoring.Trial(), 0.0


def score(objects):
    _trial["last"] = _trial["t"].step(objects)
    return _trial["last"]


def success(objects):
    return _trial["last"] >= 1.0


def fault():
    return _trial["t"].fault


# -- the scripted demo -------------------------------------------------------------------------------------------------
# Open loop: every waypoint comes from the scene's known start (where the bike, rack and knob stand), nothing a policy
# couldn't see. The right arm lifts the bike by its top tube, sets it in the outer position's channels and holds it
# upright; the left arm straddles the outer support arm's knob with open fingers, swings it out past vertical and
# lowers it onto the front tyre; then the right arm lets go.

VARIANT = os.environ.get("BIKE_VARIANT", "demo")  # demo, or a shortcut for the checks in README.md
RIGHT_YAW = math.pi / 2  # fingers close across the top tube (along x); -pi/2 is out of IK reach here (34 deg off)
LEFT_YAW = math.pi / 2  # open fingers straddle the knob along x
LIFT = 0.032  # wheels clear the 1.5 cm rail and channel walls by 1.7 cm
FORK_Y = 0.080  # the left fingers take the knob this far out from the arm's plane
FORK_OPEN = -0.028  # fingers part-open (about 5.6 cm): fully open, the far finger hits the bus front
TIPS_BELOW_KNOB = 0.018  # deep enough that the knob cannot drop out under the fingers
ARC_STEPS = 7
STOP_SHORT = math.radians(3)  # stop guiding 3 degrees before rest: the arm's weight settles it


def _park(side):
    return (0.30, 0.27 if side == "left" else -0.27, TOP + 0.20)


def _tcp_tips_at(z_table):
    """TCP world height whose pad tips sit at z_table above the worktop."""
    return TOP + z_table + PAD_TIP_PAST_TCP


def _knob_tcp(name, q):
    k = K.knob(name, q)
    return (float(k[0]), float(L.ARM_Y[name] + L.HOOK_END[name] * FORK_Y), _tcp_tips_at(float(k[2]) - TIPS_BELOW_KNOB))


def _right(tray_x, dy, hold_until, release=True):
    """Pick the bike from the rail, set it in the channels at tray_x shifted dy along them, hold until hold_until."""
    gx, gy = L.BIKE0[0], L.BIKE0[1] + B.GRIP_Y
    gz = _tcp_tips_at(0.001 + B.TOP_TUBE["z"])
    o, c, y = FINGER_OPEN, FINGER_CLOSED, RIGHT_YAW
    w = [Waypoint(2.0, (gx, gy, TOP + 0.20), o, "over the top tube", y),
         Waypoint(1.5, (gx, gy, gz), o, "down around the top tube", y),
         Waypoint(1.0, (gx, gy, gz), c, "grip", y),
         Waypoint(1.5, (gx, gy, gz + LIFT), c, "lift out of the rail", y),
         Waypoint(3.0, (tray_x, gy + dy, gz + LIFT), c, "carry over the channels", y),
         Waypoint(1.5, (tray_x, gy + dy, gz), c, "set the wheels in the channels", y)]
    t = sum(p.duration for p in w)
    w.append(Waypoint(max(0.5, hold_until - t), (tray_x, gy + dy, gz), c, "hold the bike upright", y))
    if not release:
        return w
    return w + [Waypoint(1.0, (tray_x, gy + dy, gz), o, "let go", y),
                Waypoint(1.5, (tray_x, gy + dy, gz + 0.05), o, "up", y),
                Waypoint(2.0, _park("right"), o, "clear", y)]


def _left(name, start):
    """Swing a support arm out and over onto the tyre, starting at time `start`."""
    y, o = LEFT_YAW, FORK_OPEN
    k0 = _knob_tcp(name, 0.0)
    w = [Waypoint(2.0, _park("left"), o, "park", y),
         Waypoint(max(0.5, start - 2.0), _park("left"), o, "wait", y),
         Waypoint(2.0, (k0[0], k0[1], k0[2] + 0.03), o, "over the knob", y),
         Waypoint(1.5, k0, o, "fingers either side of the knob", y)]
    q_end = K.q_rest(name) + STOP_SHORT
    for i in range(1, ARC_STEPS + 1):
        w.append(Waypoint(5.0 / ARC_STEPS, _knob_tcp(name, q_end * i / ARC_STEPS), o, f"swing out {i}/{ARC_STEPS}", y))
    end = w[-1].pos
    return w + [Waypoint(1.0, end, o, "let it settle on the tyre", y),
                Waypoint(1.5, (end[0], end[1], end[2] + 0.05), o, "up", y),
                Waypoint(2.0, _park("left"), o, "park", y)]


def demo():
    outer, inner = L.TRAYS["outer"], L.TRAYS["inner"]
    swing_done = 10.5 + 2.0 + 1.5 + 5.0 + 1.0  # the left arm has lowered the support arm by then
    if VARIANT == "nothing":
        return {"left": [Waypoint(2.0, _park("left"), FINGER_OPEN, "park")],
                "right": [Waypoint(2.0, _park("right"), FINGER_OPEN, "park")]}
    if VARIANT == "unsecured":  # set the bike in the right channels and let go, support arm left stowed
        return {"left": [Waypoint(2.0, _park("left"), FINGER_OPEN, "park")], "right": _right(outer, 0.0, 11.0)}
    if VARIANT == "held":  # set the bike in the right channels and keep holding it, support arm left stowed
        return {"left": [Waypoint(2.0, _park("left"), FINGER_OPEN, "park")],
                "right": _right(outer, 0.0, 40.0, release=False)}
    if VARIANT == "probe_swing":  # probe only (about 240 steps): swing the outer arm out with no bike under it
        return {"left": _left("outer", 2.5), "right": [Waypoint(2.0, _park("right"), FINGER_OPEN, "park")]}
    if VARIANT == "probe_place":  # probe only (about 240 steps): lift the bike into the outer channels and hold it
        return {"left": [Waypoint(2.0, _park("left"), FINGER_OPEN, "park")],
                "right": _right(outer, 0.0, 12.0, release=False)}
    if VARIANT == "inner":  # the position nearer the bus, with the outer arm swung over it, then let go
        return {"left": _left("outer", 10.5), "right": _right(inner, 0.0, swing_done + 0.5)}
    if VARIANT == "brake":  # bike slid 2.2 cm toward the hook end: the hook comes down on the brake, not the tyre
        return {"left": _left("outer", 10.5), "right": _right(outer, L.FRONT_OFFSET, swing_done + 0.5)}
    return {"left": _left("outer", 10.5), "right": _right(outer, 0.0, swing_done + 0.5)}
