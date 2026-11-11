"""On the station table: two pipe tails stand up from a grey wall block, a copper one in front of the left arm and a
white plastic (PEX) one in front of the right arm. Two grey push-fit elbows lie in a black tray in front of them, and
a bright brass pipe insert stands on a peg stand to the right.

A push-fit elbow grips the outside of the pipe with a toothed ring. Plastic pipe is soft: without a metal insert in
its end the ring squeezes it and the joint leaks, so the insert goes into the plastic pipe first. Copper needs none.
The task ends well with the insert seated in the plastic pipe and an elbow pushed fully onto each pipe.

Run it:  inspect-robots run --embodiment yam_arena -E task=pipe_insert.py --scorer task_score ...
Debug:   TASK_DEBUG=1 prints the insert's and elbows' poses every 2 s.
"""

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from partkit import _part_state, on_reset, on_step, part  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DEBUG = bool(os.environ.get("TASK_DEBUG"))

COPPER, PEX, GREY, DARK = (0.80, 0.42, 0.20), (0.95, 0.95, 0.93), (0.55, 0.57, 0.60), (0.08, 0.08, 0.08)
BRASS, FIT = (1.00, 0.80, 0.10), (0.72, 0.74, 0.77)

# The pipe tails: round tubes (12-sided, which reads as round at 320 x 176) 4.4 cm across with a 3.4 cm bore, standing
# 8 cm proud of a 4 cm wall block.
BLOCK_H, PIPE_H, PIPE_W, BORE = 0.04, 0.08, 0.044, 0.034
PIPES = {"copper": (0.30, 0.125), "pex": (0.30, -0.125)}
RIM = TOP + BLOCK_H + PIPE_H  # the pipe ends
SIDES = 12


def _ring(r_in, r_out):
    """(centre offset, yaw, size along/across the side) of each side of a 12-sided ring between two apothems."""
    rc, length = (r_in + r_out) / 2, 2 * r_out * math.tan(math.pi / SIDES)
    return [((rc * math.cos(a), rc * math.sin(a)), a, (r_out - r_in, length))
            for a in (2 * math.pi * k / SIDES for k in range(SIDES))]


def _tube(name, xy, color):
    z = TOP + BLOCK_H + PIPE_H / 2
    return [Box(f"{name}_w{i}", pos=(xy[0] + dx, xy[1] + dy, z), size=(t, ln, PIPE_H), color=color, yaw=a)
            for i, ((dx, dy), a, (t, ln)) in enumerate(_ring(BORE / 2, PIPE_W / 2))]


# The insert: a brass top hat, a 3.0 cm shank 3 cm long under a 4.2 cm collar 2.5 cm tall (the collar is what the
# fingers hold, and what shows on the pipe end). Origin at the collar's bottom face. It stands on a peg stand.
SHANK_R, SHANK_L, COLLAR_R, COLLAR_H = 0.015, 0.03, 0.021, 0.025
STAND_XY, STAND_H = (0.24, -0.19), 0.05
INSERT = part("insert", pos=(*STAND_XY, TOP + STAND_H + SHANK_L + 0.001), mass=0.03, color=BRASS,
              cylinders=[((0, 0, -SHANK_L / 2), SHANK_R, SHANK_L, BRASS), ((0, 0, COLLAR_H / 2), COLLAR_R, COLLAR_H, BRASS)],
              boxes=[((0, 0, COLLAR_H / 2), (0.001, 0.001, 0.001), BRASS)])

# The elbows: a round socket (12-sided) 4.9 cm across inside (2.5 mm clearance on the pipe's flats, 1.7 mm at its
# corners if turned) and 4 cm deep, closed by a round cap, with the outlet stub along +x and an upright grip tab on
# the cap. Origin at the cap's underside.
SOCK_IN, SOCK_WALL, SOCK_D, CAP_T = 0.049, 0.006, 0.04, 0.006  # probe 2: 4.7 caught the pipe end at 2.9 mm off
TAB = (0.02, 0.015, 0.045)  # along x (the fingers close on this, gripper turned 90 deg), across, tall
_SOCK_OUT = SOCK_IN / 2 + SOCK_WALL  # outer apothem
_ELBOW_BOXES = [*[((dx, dy, -SOCK_D / 2), (t, ln, SOCK_D), FIT, (math.cos(a / 2), 0.0, 0.0, math.sin(a / 2)))
                  for (dx, dy), a, (t, ln) in _ring(SOCK_IN / 2, _SOCK_OUT)],
                # outlet stub, kept small: partkit spreads the mass by volume, and probe 3's 5 x 5 x 3 cm stub put the
                # centre of mass ~28 mm off the axis, outside the collar it rests on, so the elbow tipped when let go
                ((_SOCK_OUT + 0.015, 0, -0.012), (0.03, 0.04, 0.025), FIT),
                ((0, 0, CAP_T + TAB[2] / 2), TAB, DARK)]
_ELBOW_CYLS = [((0, 0, CAP_T / 2), _SOCK_OUT / math.cos(math.pi / SIDES), CAP_T, FIT)]
TRAY_X, TRAY_T = 0.16, 0.006  # the outlet stubs stay clear of the wall block (x 0.25-0.35)
ELBOWS = {n: part(n, pos=(TRAY_X, y, TOP + TRAY_T + SOCK_D + 0.001), mass=0.05, color=FIT, boxes=_ELBOW_BOXES,
                    cylinders=_ELBOW_CYLS)
          for n, y in (("elbow_left", 0.10), ("elbow_right", -0.10))}

OBJECTS = [
    Box("wall_block", pos=(0.30, 0.0, TOP + BLOCK_H / 2), size=(0.10, 0.40, BLOCK_H), color=GREY),
    *_tube("pipe_copper", PIPES["copper"], COPPER), *_tube("pipe_pex", PIPES["pex"], PEX),
    Box("tray", pos=(TRAY_X, 0.0, TOP + TRAY_T / 2), size=(0.10, 0.36, TRAY_T), color=DARK),
    Cylinder("stand_base", pos=(*STAND_XY, TOP + 0.003), radius=0.03, height=0.006, color=DARK),
    Cylinder("stand_peg", pos=(*STAND_XY, TOP + STAND_H / 2), radius=0.008, height=STAND_H, color=DARK),
    INSERT, *ELBOWS.values(),
]

# -- scoring --------------------------------------------------------------------------------------------------------

AXIS_M = 0.003  # centred on the pipe's axis within 3 mm
SEAT_M = 0.003  # resting within 3 mm of its stop
MAX_TILT_DEG = 5.0


def _tilt_deg(q):
    x, y, _, _ = (float(v) for v in q)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))


def _on_axis(o, pipe):
    return math.dist((float(o.pos[0]), float(o.pos[1])), PIPES[pipe]) <= AXIS_M and _tilt_deg(o.quat_xyzw) <= MAX_TILT_DEG


def _insert_in(objects):
    """The pipe the insert is seated in (collar resting on its end), or None."""
    o = objects[INSERT.name]
    return next((p for p in PIPES if _on_axis(o, p) and abs(float(o.pos[2]) - RIM) <= SEAT_M), None)


def _elbow_on(objects, pipe):
    """Some elbow is pushed fully onto ``pipe``: its cap on the pipe end, or on the insert's collar if one is in it."""
    stop = RIM + (COLLAR_H if _insert_in(objects) == pipe else 0.0)
    return any(_on_axis(objects[e.name], pipe) and abs(float(objects[e.name].pos[2]) - stop) <= SEAT_M
               for e in ELBOWS.values())


_S = {}


@on_reset
def _reset(environment=None):
    _S.update(step=0, pex_without_insert=False)


_reset()


def _update(objects):
    """One control step: latch an elbow ever pushed onto the plastic pipe with no insert in it (the grab ring has
    already crushed the pipe end; taking the elbow off doesn't undo that)."""
    if _elbow_on(objects, "pex") and _insert_in(objects) != "pex":
        _S["pex_without_insert"] = True
    _S["step"] += 1


@on_step
def _step(environment):
    objects = {s.name: _part_state(environment, s) for s in (INSERT, *ELBOWS.values())}
    _update(objects)
    if DEBUG and _S["step"] % 40 == 0:
        for n, o in objects.items():
            print(f"TASK t={_S['step'] / 20:.0f}s {n} x={o.pos[0]:.4f} y={o.pos[1]:.4f} z={(o.pos[2] - RIM) * 1e3:+.1f}mm "
                  f"tilt={_tilt_deg(o.quat_xyzw):.1f}", flush=True)
        print(f"TASK insert_in={_insert_in(objects)} copper={_elbow_on(objects, 'copper')} "
              f"pex={_elbow_on(objects, 'pex')} latched={_S['pex_without_insert']}", flush=True)


def score(objects):
    """0.5 per joint. Copper: an elbow fully on, no insert in it. Plastic: the insert seated, an elbow fully on over
    it, and no elbow ever pushed onto it without the insert."""
    ins = _insert_in(objects)
    copper = 0.5 if _elbow_on(objects, "copper") and ins != "copper" else 0.0
    pex = 0.5 if ins == "pex" and _elbow_on(objects, "pex") and not _S["pex_without_insert"] else 0.0
    return copper + pex


# No success(): the trial runs to the end of the demo and the final state is scored, with the arms clear.

# -- the scripted demo ----------------------------------------------------------------------------------------------

# (arm, part, pipe), done one after the other. run_checks.sh swaps this line for the variants.
DEMO_PLAN = (("right", "insert", "pex"), ("right", "elbow_right", "pex"), ("left", "elbow_left", "copper"))

HIGH = TOP + 0.25  # carry height: an elbow's socket clears the insert's collar on a pipe end by ~1.5 cm
INSERT_GRIP = 0.015  # TCP above the collar's bottom: the pad tips stay a few mm above the pipe end at the release
ELBOW_GRIP = CAP_T + 0.016  # TCP above the elbow cap's underside: the pads low on the tab, close to the elbow
INSERT_OPEN, INSERT_CLOSE = -0.032, -(2 * COLLAR_R - 0.008) / 2  # 64 mm open; close 8 mm narrower than the collar
# The elbow is held with the fingers closing along x, across the tab's 2 cm (yaw +90 right arm, -90 left): its
# outlet stub puts the centre of mass off the tab in x, and probe 1 (fingers along y) let it swing 13 deg in the
# grip, catch on the pipe end and flip off.
TAB_OPEN, TAB_CLOSE = -0.022, -(TAB[0] - 0.008) / 2
ELBOW_YAW = {"left": -math.pi / 2, "right": math.pi / 2}
ENTRY = 0.006  # pause with the part 6 mm above where it first meets the pipe, then the slow push
AIM = {"copper": (0.0025, 0.0), "pex": (0.0025, 0.0)}  # probe 2: both elbows came down 2-3 mm short in x
WAIT = {"left": (0.20, 0.28, TOP + 0.20), "right": (0.20, -0.28, TOP + 0.20)}
CLEAR = {"left": (0.12, 0.42, TOP + 0.20), "right": (0.12, -0.42, TOP + 0.20)}


def _move(pick, place, meet, open_, close, yaw=0.0):
    """Top-down pick and place at yaw 0: settle above, descend, settle, close, lift high, carry, settle, lower to just
    above where the part meets the pipe, settle, push slowly home, settle, let go, retreat high. ``meet`` is the
    place height at which the part first touches the pipe. 20.1 s."""
    (px, py, pz), (qx, qy, qz) = pick, place
    return [
        Waypoint(1.5, (px, py, HIGH), open_, "over the part", yaw),
        Waypoint(0.8, (px, py, HIGH), open_, "settle above", yaw),
        Waypoint(1.5, (px, py, pz), open_, "descend", yaw),
        Waypoint(0.6, (px, py, pz), open_, "settle", yaw),
        Waypoint(1.5, (px, py, pz), close, "close", yaw),
        Waypoint(2.0, (px, py, HIGH), close, "lift", yaw),
        Waypoint(3.0, (qx, qy, HIGH), close, "carry", yaw),
        Waypoint(0.8, (qx, qy, HIGH), close, "settle over the pipe", yaw),
        Waypoint(1.5, (qx, qy, meet + ENTRY), close, "lower to the pipe end", yaw),
        Waypoint(0.8, (qx, qy, meet + ENTRY), close, "settle", yaw),
        Waypoint(2.5, (qx, qy, qz + 0.001), close, "push home", yaw),
        Waypoint(0.6, (qx, qy, qz + 0.001), close, "settle", yaw),
        Waypoint(1.0, (qx, qy, qz + 0.001), open_, "let go", yaw),
        Waypoint(1.5, (qx, qy, HIGH), open_, "retreat high", yaw),
    ]


def demo():
    """The right arm takes the insert from its stand and pushes it into the plastic pipe (it can see the white pipe
    and the brass insert), then pushes the right elbow onto the plastic pipe over it; then the left arm pushes the
    left elbow onto the copper pipe. One move at a time, the idle arm waiting clear; both arms end clear of the
    camera."""
    arms = {"left": [], "right": []}
    t = {"left": 0.0, "right": 0.0}
    stacked = {"copper": 0.0, "pex": 0.0}
    for arm, name, pipe in DEMO_PLAN:
        if t[arm] < max(t.values()):  # wait for the other arm's move to finish
            arms[arm].append(Waypoint(max(t.values()) - t[arm] + 0.01, WAIT[arm], FINGER_OPEN, "wait"))
        x, y = PIPES[pipe][0] + AIM[pipe][0], PIPES[pipe][1] + AIM[pipe][1]
        if name == "insert":
            pick = (*STAND_XY, TOP + STAND_H + SHANK_L + INSERT_GRIP)
            place = (x, y, RIM + INSERT_GRIP)
            steps = _move(pick, place, RIM + SHANK_L + INSERT_GRIP, INSERT_OPEN, INSERT_CLOSE)
            stacked[pipe] = COLLAR_H
        else:
            ex, ey = TRAY_X, ELBOWS[name].pos[1]
            pick = (ex, ey, TOP + TRAY_T + SOCK_D + ELBOW_GRIP)
            place = (x, y, RIM + stacked[pipe] + ELBOW_GRIP)
            steps = _move(pick, place, RIM + stacked[pipe] + SOCK_D + ELBOW_GRIP, TAB_OPEN, TAB_CLOSE, ELBOW_YAW[arm])
        arms[arm] += steps
        t[arm] = max(t.values()) + sum(w.duration for w in steps)
    end = max(t.values())
    for arm in arms:
        if t[arm] < end:
            arms[arm].append(Waypoint(end - t[arm] + 0.01, WAIT[arm], FINGER_OPEN, "wait"))
        arms[arm].append(Waypoint(1.5, CLEAR[arm], FINGER_OPEN, "move clear"))
    return arms
