"""On the station table: a substation's DC distribution panel in front of the right arm, an insulation monitor
(earth-fault meter) left of it, and a battery bank behind, cabled together (meter and battery are fixed decoration
apart from the meter's needle).

The panel feeds four DC circuits, F1 (nearest the robot) to F4 (farthest). Each runs through a fuse carrier standing
in its own socket on the panel top; pulling a carrier out opens that circuit, and pushing one back in closes it. All
four start in. The meter's needle sits in the red zone: there is an earth fault on F3. Whenever F3's socket is empty
the needle swings to the green zone within about a second; otherwise it returns to red.

Operators find the faulty circuit by sectionalising: pull one fuse, watch the meter, and if it stays red, put that fuse
back before trying the next, so protection is never left unpowered on two circuits at once. The task ends well with
only F3's fuse out, the others in, and never two out at the same time.

Run it:  inspect-robots run --embodiment yam_arena -E task=dc_earth_fault.py --scorer task_score ...
Debug:   TASK_DEBUG=1 prints every socket change and the needle once a second.
"""

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from partkit import _part_state, on_reset, on_step, part, write_pose  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
HZ = 20.0  # control steps per second
DEBUG = bool(os.environ.get("TASK_DEBUG"))

FEEDERS = ("f1", "f2", "f3", "f4")
FAULTY = "f3"
RED_DEG, GREEN_DEG = -50.0, 50.0  # needle angle from 12 o'clock: right = earth fault, left = healthy
NEEDLE_TAU_S = 0.4  # the needle follows the insulation reading with this lag

GREY, BLACK, WHITE = (0.78, 0.79, 0.76), (0.08, 0.08, 0.08), (0.95, 0.95, 0.95)
RED, GREEN = (0.80, 0.10, 0.10), (0.10, 0.65, 0.20)

# The DC panel: a 3 cm base with four fuse sockets on top. Part frame origin: the base's centre (top face at z =
# +0.015). x away from the robot, y to the robot's left.
PX, PY = 0.35, -0.12
FEEDER_X = {f: -0.09 + 0.06 * i for i, f in enumerate(FEEDERS)}  # 6 cm apart along x
SOCKET_Y, BUS_Y = -0.03, 0.05
SOCKET, WALL, DEPTH = 0.025, 0.008, 0.030  # square socket, inside (m); its walls; its depth
FLOOR = TOP + 0.03  # socket floor = panel base top
_OUT = SOCKET / 2 + WALL  # socket block half-width


def _socket_walls(f):
    x, y, z, off = FEEDER_X[f], SOCKET_Y, 0.015 + DEPTH / 2, SOCKET / 2 + WALL / 2
    return [((x + off, y, z), (WALL, 2 * _OUT, DEPTH), BLACK), ((x - off, y, z), (WALL, 2 * _OUT, DEPTH), BLACK),
            ((x, y + off, z), (SOCKET, WALL, DEPTH), BLACK), ((x, y - off, z), (SOCKET, WALL, DEPTH), BLACK)]


_BUS_LEN, _OUT_LEN = BUS_Y - (SOCKET_Y + _OUT), (SOCKET_Y - _OUT) + 0.075  # feeder strips: socket -> busbar, outgoing
PANEL = part("panel", pos=(PX, PY, TOP + 0.015), mass=20.0, color=GREY,
             boxes=[((0, 0, 0), (0.26, 0.16, 0.03), GREY),
                    ((0, BUS_Y, 0.0155), (0.22, 0.006, 0.001), BLACK),  # busbar strip
                    *[((FEEDER_X[f], BUS_Y - _BUS_LEN / 2, 0.0155), (0.006, _BUS_LEN, 0.001), BLACK) for f in FEEDERS],
                    *[((FEEDER_X[f], -0.075 + _OUT_LEN / 2, 0.0155), (0.006, _OUT_LEN, 0.001), BLACK) for f in FEEDERS],
                    *[w for f in FEEDERS for w in _socket_walls(f)]])
SOCKET_XY = {f: (PX + FEEDER_X[f], PY + SOCKET_Y) for f in FEEDERS}

# The fuse carriers: black bodies 2 cm square, 7.5 cm tall, with a red handle cap; seated, they stand 4.5 cm proud of
# the sockets. Carriers are interchangeable: a socket is closed by whichever carrier is seated in it.
FUSE_H = 0.075
SEATED_M = 0.010  # a carrier counts as in when its bottom is within 10 mm of the socket floor (the contacts break)
FUSES = {f: part(f"fuse_{f}", pos=(*SOCKET_XY[f], FLOOR + FUSE_H / 2 + 0.001), mass=0.08, color=BLACK,
                 boxes=[((0, 0, 0), (0.02, 0.02, FUSE_H), BLACK),
                        ((0, 0, FUSE_H / 2 + 0.002), (0.02, 0.02, 0.004), RED)]) for f in FEEDERS}

# The insulation monitor: white dial with red (right) and green (left) zone marks, black needle from the centre.
MX, MY = 0.36, 0.06
METER = part("meter", pos=(MX, MY, TOP + 0.03), mass=10.0, color=GREY,
             boxes=[((0, 0, 0), (0.14, 0.14, 0.06), GREY),
                    *[((0.062 * math.cos(math.radians(a)), 0.062 * math.sin(math.radians(a)), 0.0305),
                       (0.016, 0.016, 0.001), c) for a, c in ((RED_DEG, RED), (GREEN_DEG, GREEN))]],
             cylinders=[((0, 0, 0.031), 0.055, 0.002, BLACK), ((0, 0, 0.0315), 0.05, 0.003, WHITE)])
NEEDLE_POS = (MX, MY, TOP + 0.03 + 0.033 + 0.0015)
NEEDLE = part("needle", pos=NEEDLE_POS, yaw=math.radians(RED_DEG), mass=0.02, color=BLACK,
              boxes=[((0.02, 0, 0), (0.06, 0.006, 0.003), BLACK)])  # origin at the dial centre, tip 5 cm out

# Fixed decoration: a battery bank behind the meter, and cables meter -> panel and battery -> meter.
BX, BY = 0.50, 0.08
DECOR = [
    Box("battery_base", pos=(BX, BY, TOP + 0.01), size=(0.10, 0.14, 0.02), color=(0.25, 0.26, 0.27)),
    *[Box(f"battery_cell{i}", pos=(BX, BY - 0.0495 + 0.033 * i, TOP + 0.05), size=(0.07, 0.028, 0.06),
          color=(0.15, 0.30, 0.55)) for i in range(4)],
    Box("cable_meter_panel", pos=(MX, (MY - 0.07 + PY + 0.08) / 2, TOP + 0.005), size=(0.012, MY - 0.07 - PY - 0.08, 0.01),
        color=BLACK),
    Box("cable_battery_meter", pos=((BX - 0.05 + MX + 0.07) / 2, BY, TOP + 0.005), size=(BX - 0.05 - MX - 0.07, 0.012, 0.01),
        color=BLACK),
]
OBJECTS = [PANEL, *FUSES.values(), METER, NEEDLE, *DECOR]

# The trial record. The module lives across trials, so it is cleared on every reset.
_S = {}


@on_reset
def _reset(environment=None):
    _S.update(step=0, open={f: False for f in FEEDERS}, max_open=0, needle=RED_DEG)


_reset()


def _seated_in(f, carriers):
    """Some carrier stands in socket f: its centre over the socket and its bottom within SEATED_M of the floor."""
    sx, sy = SOCKET_XY[f]
    return any(abs(x - sx) <= SOCKET / 2 and abs(y - sy) <= SOCKET / 2 and bottom <= FLOOR + SEATED_M
               for x, y, bottom in carriers.values())


def _advance(carriers):
    """One control step of the DC system, given each carrier's centre x, y and bottom z (m)."""
    for f in FEEDERS:
        is_open = not _seated_in(f, carriers)
        if DEBUG and is_open != _S["open"][f]:
            print(f"TASK t={_S['step'] / HZ:.2f}s {f} {'opened' if is_open else 'closed'}", flush=True)
        _S["open"][f] = is_open
    _S["max_open"] = max(_S["max_open"], sum(_S["open"].values()))
    target = GREEN_DEG if _S["open"][FAULTY] else RED_DEG
    _S["needle"] += (target - _S["needle"]) * (1.0 - math.exp(-1.0 / (HZ * NEEDLE_TAU_S)))
    _S["step"] += 1
    if DEBUG and _S["step"] % 20 == 0:
        print(f"TASK t={_S['step'] / HZ:.1f}s needle={_S['needle']:+.0f} open={[f for f in FEEDERS if _S['open'][f]]} "
              f"max_open={_S['max_open']}", flush=True)
        for f, (x, y, bottom) in carriers.items():  # from the socket centre and floor, mm
            print(f"TASK   {f} dx={(x - SOCKET_XY[f][0]) * 1e3:+.1f} dy={(y - SOCKET_XY[f][1]) * 1e3:+.1f} "
                  f"dz={(bottom - FLOOR) * 1e3:+.1f}", flush=True)


def _yaw_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


@on_step
def _step(environment):
    carriers = {}
    for f, fuse in FUSES.items():
        p = _part_state(environment, fuse).pos
        carriers[f] = (float(p[0]), float(p[1]), float(p[2]) - FUSE_H / 2)  # treats the carrier as upright: a tilted
        # one reads up to a few mm low, well inside the 10 mm SEATED_M, and one lying down is nowhere near a socket
    _advance(carriers)
    write_pose(environment, NEEDLE.name, NEEDLE_POS, _yaw_quat(math.radians(_S["needle"])))


def score(objects):
    """Only F3's socket empty at the end: 1.0 if two sockets were never empty at once, else 0.5. F3's socket empty
    with another one still empty: 0.25. F3's socket filled: 0."""
    if not _S["open"][FAULTY]:
        return 0.0
    if any(_S["open"][f] for f in FEEDERS if f != FAULTY):
        return 0.25
    return 1.0 if _S["max_open"] <= 1 else 0.5


# No success(): it would end the trial 1 s after F3 is pulled, before a policy that puts it back again is seen.


# The scripted demo, as (action, feeder) pairs: "check" pulls a fuse, watches, puts it back; "pull" pulls it and sets
# it down on the table; "insert" takes a set-down fuse back to its socket. run_checks.sh swaps this line for the
# shortcut variants.
DEMO_PLAN = (("check", "f1"), ("check", "f2"), ("pull", "f3"))

GRIP_Z = FLOOR + FUSE_H - 0.020  # TCP 2 cm below a seated carrier's top; the pad tips stay 12 mm above the socket walls
LIFT = DEPTH + 0.020  # pulled: the carrier's bottom 2 cm above the socket's rim, clear of every wall it passes over
ABOVE = 0.06
PARK_Y = -0.27  # set-down spots on the table, straight to the robot's right of each socket, clear of the panel
PARK_GRIP_Z = TOP + FUSE_H - 0.020
INSERT_AIM_X = 0.005  # a carrier taken back from the table arrives 5 mm short in x (variant run 1); aim past it


def demo():
    """Sectionalise like an operator: pull F1's fuse, hold it clear while watching the meter stay red, push it back;
    the same for F2; pull F3's, see the needle swing to green, and set the fuse down beside the panel. The holds stand
    in for watching the meter (it settles in ~1 s)."""

    def grab(xy, z, label):
        return [Waypoint(1.5, (*xy, z + ABOVE), FINGER_OPEN, f"above {label}"),
                Waypoint(1.0, (*xy, z), FINGER_OPEN, f"down to {label}"),
                Waypoint(0.6, (*xy, z), FINGER_CLOSED, f"grip {label}")]

    def put(xy, z, label):
        mouth = z + DEPTH + 0.005  # carrier bottom 5 mm above the rim: settle there, then go in slowly (probe 1: a
        # 1.5 s straight descent let F1's carrier swing 4.6 mm, past the 2.5 mm clearance, onto the rim)
        return [Waypoint(1.0, (*xy, mouth), FINGER_CLOSED, f"over {label}"),
                Waypoint(0.8, (*xy, mouth), FINGER_CLOSED, "settle"),
                Waypoint(2.5, (*xy, z + 0.002), FINGER_CLOSED, f"lower into {label}"),
                Waypoint(0.6, (*xy, z + 0.002), FINGER_OPEN, "let go"),
                Waypoint(0.8, (*xy, z + ABOVE), FINGER_OPEN, "clear")]

    steps = []
    for action, f in DEMO_PLAN:
        xy, park = SOCKET_XY[f], (SOCKET_XY[f][0], PARK_Y)
        if action == "insert":
            steps += grab(park, PARK_GRIP_Z, f"set-down {f}")
            steps += [Waypoint(1.0, (*park, GRIP_Z + LIFT), FINGER_CLOSED, f"lift {f}"),
                      Waypoint(1.5, (xy[0] + INSERT_AIM_X, xy[1], GRIP_Z + LIFT), FINGER_CLOSED, f"over socket {f}")]
            steps += put((xy[0] + INSERT_AIM_X, xy[1]), GRIP_Z, f"socket {f}")
            continue
        steps += grab(xy, GRIP_Z, f"fuse {f}")
        steps += [Waypoint(1.2, (*xy, GRIP_Z + LIFT), FINGER_CLOSED, f"pull {f}"),
                  Waypoint(1.5, (*xy, GRIP_Z + LIFT), FINGER_CLOSED, "watch the meter")]
        if action == "check":  # the meter stayed red: put it back
            steps += put(xy, GRIP_Z, f"socket {f}")
        else:
            steps += [Waypoint(1.5, (*park, GRIP_Z + LIFT), FINGER_CLOSED, f"carry {f} aside")]
            steps += put(park, PARK_GRIP_Z, "the table")
    steps.append(Waypoint(1.5, (0.12, -0.42, TOP + 0.20), FINGER_OPEN, "move clear"))
    return {"right": steps}
