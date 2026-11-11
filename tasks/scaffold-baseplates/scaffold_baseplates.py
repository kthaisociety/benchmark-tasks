"""On the station table: a patch of lawn with two white marks 30 cm apart, where two scaffold tubes are to stand.
In front of the lawn lies a timber sole board; nearer the robot, two steel base plates with a socket collar; behind
the lawn, two scaffold tubes stand on the table.

Scaffolders never stand a tube, or a base plate, straight on soft ground: it punches in and the scaffold leans. The
footing goes sole board first, base plates on the board, then the tubes into the base plates. A software ground model
makes anything set straight on the grass sink 8 mm and lean (10 degrees for a tube, 6 for a plate), latched for the
trial: picking it up again doesn't undo it, as with a real footing that has punched in.

Run it:  inspect-robots run --embodiment yam_arena -E task=scaffold_baseplates.py --scorer task_score ...
Debug:   TASK_DEBUG=1 prints every part's pose every 2 s and each sinking.
"""

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from partkit import _part_state, on_reset, on_step, part, write_pose  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DEBUG = bool(os.environ.get("TASK_DEBUG"))

GRASS, WHITE, TIMBER = (0.22, 0.55, 0.18), (0.95, 0.95, 0.95), (0.72, 0.50, 0.26)
STEEL, SILVER = (0.30, 0.32, 0.36), (0.82, 0.83, 0.85)

# The lawn: a fixed green mat, 14 cm (x) by 50 cm, 1 cm thick. Marks are where the two tubes must stand.
LX, LAWN_HX, LAWN_HY = 0.34, 0.07, 0.25
LAWN = TOP + 0.01  # the grass surface
MARKS = {"left": (LX, 0.15), "right": (LX, -0.15)}
DECOR = [Box("lawn", pos=(LX, 0.0, TOP + 0.005), size=(2 * LAWN_HX, 2 * LAWN_HY, 0.01), color=GRASS),
         *[Box(f"mark_{s}", pos=(x, y, LAWN + 0.0005), size=(0.03, 0.03, 0.001), color=WHITE) for s, (x, y) in MARKS.items()]]

# The sole board: timber 8 x 40 x 3 cm, long side along y. Origin at its centre.
BOARD_W, BOARD_L, BOARD_T = 0.08, 0.40, 0.03
BOARD_START = (0.22, 0.0)
BOARD = part("sole_board", pos=(*BOARD_START, TOP + BOARD_T / 2 + 0.001), mass=0.3, color=TIMBER,
             boxes=[((0, 0, 0), (BOARD_W, BOARD_L, BOARD_T), TIMBER)])

# Base plates: an 8 x 8 cm steel plate 6 mm thick with a square socket collar (4.4 cm inside, 3 cm tall) that takes a
# tube's foot. Origin at the centre of the plate's base.
PLATE, PLATE_T = 0.08, 0.006
COLLAR_IN, COLLAR_WALL, COLLAR_H = 0.044, 0.006, 0.030  # 7 mm play round the tube (probes 1-2: a tube held ~4.5 deg
# off vertical missed a 2 mm-play collar and jammed)
_co = COLLAR_IN / 2 + COLLAR_WALL / 2
_cz = PLATE_T / 2 + COLLAR_H / 2
_PLATE_BOXES = [((0, 0, 0), (PLATE, PLATE, PLATE_T), STEEL),
                ((_co, 0, _cz), (COLLAR_WALL, COLLAR_IN + 2 * COLLAR_WALL, COLLAR_H), STEEL),
                ((-_co, 0, _cz), (COLLAR_WALL, COLLAR_IN + 2 * COLLAR_WALL, COLLAR_H), STEEL),
                ((0, _co, _cz), (COLLAR_IN, COLLAR_WALL, COLLAR_H), STEEL),
                ((0, -_co, _cz), (COLLAR_IN, COLLAR_WALL, COLLAR_H), STEEL)]
PLATE_START = {"left": (0.12, 0.10), "right": (0.12, -0.10)}
PLATES = {s: part(f"plate_{s}", pos=(*xy, TOP + PLATE_T / 2 + 0.001), mass=0.4, color=STEEL, boxes=_PLATE_BOXES)
          for s, xy in PLATE_START.items()}

# Scaffold tubes: 3 cm across, 15 cm tall, standing on the table behind the lawn. Origin at the tube's centre.
TUBE_R, TUBE_H = 0.015, 0.15
TUBE_START = {"left": (0.48, 0.15), "right": (0.48, -0.15)}
TUBES = {s: part(f"tube_{s}", pos=(*xy, TOP + TUBE_H / 2 + 0.001), mass=0.15, color=SILVER,
                 boxes=[((0, 0, 0), (0.001, 0.001, 0.001), SILVER)],
                 cylinders=[((0, 0, 0), TUBE_R, TUBE_H, SILVER), ((0, 0, TUBE_H / 2), TUBE_R * 0.75, 0.001, STEEL)])
         for s, xy in TUBE_START.items()}

OBJECTS = [*DECOR, BOARD, *PLATES.values(), *TUBES.values()]

# -- the ground model and the trial record ------------------------------------------------------------------------

SINK_M = 0.008
LEAN_DEG = {"plate": 6.0, "tube": 10.0}
ON_GRASS_M = 0.004  # a bottom within 4 mm of the grass surface, over the lawn and not over the board, rests on grass
SETTLE_STEPS = 10  # resting that long (under 2 cm/s) before it sinks
_S = {}


@on_reset
def _reset(environment=None):
    _S.update(step=0, still={}, sunk={})


_reset()


def _yaw(q):
    x, y, z, w = (float(v) for v in q)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def _tilt_deg(q):
    x, y, _, _ = (float(v) for v in q)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))


def _over_board(x, y, board):
    """(x, y) inside the board's outline, in the board's own frame."""
    bx, by = float(board.pos[0]), float(board.pos[1])
    c, s = math.cos(_yaw(board.quat_xyzw)), math.sin(_yaw(board.quat_xyzw))
    lx, ly = c * (x - bx) + s * (y - by), -s * (x - bx) + c * (y - by)
    return abs(lx) <= BOARD_W / 2 and abs(ly) <= BOARD_L / 2


def _bottom(name, o):
    return float(o.pos[2]) - (PLATE_T / 2 if name.startswith("plate") else TUBE_H / 2)


def _on_grass(name, o, board):
    x, y = float(o.pos[0]), float(o.pos[1])
    return (abs(x - LX) <= LAWN_HX and abs(y) <= LAWN_HY and not _over_board(x, y, board)
            and _bottom(name, o) <= LAWN + ON_GRASS_M)


def _ground(states):
    """One control step of the ground model, given every part's state by name. Returns the poses to hold."""
    board = states[BOARD.name]
    for name, o in states.items():
        if name == BOARD.name or name in _S["sunk"]:
            continue
        resting = float(sum(v * v for v in o.lin_vel)) ** 0.5 < 0.02
        _S["still"][name] = _S["still"].get(name, 0) + 1 if resting and _on_grass(name, o, board) else 0
        if _S["still"][name] >= SETTLE_STEPS:
            a = math.radians(LEAN_DEG["plate" if name.startswith("plate") else "tube"])
            _S["sunk"][name] = ((float(o.pos[0]), float(o.pos[1]), float(o.pos[2]) - SINK_M),
                                (math.sin(a / 2), 0.0, 0.0, math.cos(a / 2)))
            if DEBUG:
                print(f"TASK t={_S['step'] / 20:.2f}s {name} sank into the grass", flush=True)
    _S["step"] += 1
    return dict(_S["sunk"])


@on_step
def _step(environment):
    states = {spec.name: _part_state(environment, spec) for spec in (BOARD, *PLATES.values(), *TUBES.values())}
    for name, (pos, quat) in _ground(states).items():
        write_pose(environment, name, pos, quat)  # held sunk and leaning for the rest of the trial
    if DEBUG and _S["step"] % 40 == 0:
        for name, o in states.items():
            print(f"TASK t={_S['step'] / 20:.0f}s {name} x={o.pos[0]:.4f} y={o.pos[1]:.4f} "
                  f"bottom={(_bottom(name, o) - TOP) * 1e3 if name != BOARD.name else (o.pos[2] - BOARD_T / 2 - TOP) * 1e3:+.1f}mm "
                  f"yaw={math.degrees(_yaw(o.quat_xyzw)):+.1f} tilt={_tilt_deg(o.quat_xyzw):.1f}", flush=True)
        print(f"TASK t={_S['step'] / 20:.0f}s score={score(states):.2f} sunk={sorted(_S['sunk'])}", flush=True)


# -- scoring ---------------------------------------------------------------------------------------------------------

BOARD_TOL_M, BOARD_YAW_DEG = 0.03, 10.0
PLATE_MARK_M, ON_BOARD_M = 0.03, 0.005
TUBE_AXIS_M, TUBE_SEAT_M, TUBE_TILT_DEG = 0.008, 0.005, 5.0


def _board_ok(b):
    yaw = abs(math.degrees(_yaw(b.quat_xyzw))) % 180.0
    return (abs(float(b.pos[0]) - LX) <= BOARD_TOL_M and abs(float(b.pos[1])) <= BOARD_TOL_M
            and min(yaw, 180.0 - yaw) <= BOARD_YAW_DEG and float(b.pos[2]) - BOARD_T / 2 <= LAWN + 0.005)


def _plate_ok(side, p, board):
    x, y = float(p.pos[0]), float(p.pos[1])
    return (abs(_bottom("plate", p) - (LAWN + BOARD_T)) <= ON_BOARD_M and _over_board(x, y, board)
            and math.dist((x, y), MARKS[side]) <= PLATE_MARK_M)


def _tube_in(t, p):
    return (math.dist((float(t.pos[0]), float(t.pos[1])), (float(p.pos[0]), float(p.pos[1]))) <= TUBE_AXIS_M
            and abs(_bottom("tube", t) - (_bottom("plate", p) + PLATE_T)) <= TUBE_SEAT_M
            and _tilt_deg(t.quat_xyzw) <= TUBE_TILT_DEG)


def score(objects):
    """0.2 for the sole board across both marks; 0.15 per base plate on the board at its mark (not sunk); 0.25 per tube
    standing in a counted base plate (not sunk). Anything that ever sank into the grass earns nothing."""
    board = objects[BOARD.name]
    total = 0.2 if _board_ok(board) else 0.0
    ok_plates = [PLATES[s].name for s in PLATES
                 if PLATES[s].name not in _S["sunk"] and _plate_ok(s, objects[PLATES[s].name], board)]
    total += 0.15 * len(ok_plates)
    for t in TUBES.values():
        if t.name not in _S["sunk"] and any(_tube_in(objects[t.name], objects[p]) for p in ok_plates):
            total += 0.25
    return round(total, 4)


# No success(): the trial runs its full limit and the final state is scored, so the arms move clear first.

# -- the scripted demo ----------------------------------------------------------------------------------------------

# Steps, in order: ("board",) carries the sole board onto the lawn across the marks; ("plates", "board"|"grass") sets
# each base plate at its mark, on the board or straight on the grass; ("tubes", "plates"|"grass") stands each tube in
# its base plate or straight on the grass at its mark. run_checks.sh swaps this line for the variants.
DEMO_PLAN = (("board",), ("plates", "board"), ("tubes", "plates"))

SQUEEZE = 0.008  # close to 8 mm narrower than the part: ~20 N per finger
CLEAR_Z = TOP + 0.24  # retreat height after letting go: above every tube top
BOARD_TCP = 0.023  # TCP above the board's bottom: pad tips at its mid-height
PLATE_TCP = PLATE_T + COLLAR_H / 2 + 0.008  # pad tips at the collar's mid-height
TUBE_TCP = TUBE_H / 2 + 0.008  # pad tips at the tube's mid-height, where its weight is (probe 1: gripped 4 cm below
# the top, the tube swung 7-10 degrees in the grip, its foot hit the collar rim and it fell)
TUBE_SQUEEZE = 0.012  # a firmer close on the tube: ~30 N per finger
COLLAR_TOP = PLATE_T + COLLAR_H
# Calibration knobs for the probe (aim corrections measured from the TASK lines)
TUBE_AIM = {"left": (0.0, 0.0), "right": (0.0, 0.0)}
PLATE_AIM = {"left": (0.0, 0.0), "right": (0.0, 0.0)}


def _move(pick, place, *, width, opening, travel_bottom, entry, yaw=0.0, tcp, squeeze=SQUEEZE, release=None):
    """Pick a part whose bottom is at pick[2], set it with its bottom at place[2]. 19.6 s."""
    (px, py, pz), (qx, qy, qz) = pick, place
    grip, travel = -(width - squeeze) / 2, travel_bottom + tcp
    release = opening if release is None else release
    return [
        Waypoint(1.5, (px, py, travel), opening, "to the part", yaw),
        Waypoint(0.8, (px, py, travel), opening, "settle above", yaw),
        Waypoint(1.5, (px, py, pz + tcp), opening, "descend", yaw),
        Waypoint(0.6, (px, py, pz + tcp), opening, "settle", yaw),
        Waypoint(1.5, (px, py, pz + tcp), grip, "close", yaw),
        Waypoint(1.2, (px, py, travel), grip, "lift", yaw),
        Waypoint(3.0, (qx, qy, travel), grip, "carry", yaw),
        Waypoint(1.5, (qx, qy, travel), grip, "settle over the place", yaw),
        Waypoint(1.2, (qx, qy, qz + entry + tcp), grip, "lower to the entry", yaw),
        Waypoint(1.5, (qx, qy, qz + entry + tcp), grip, "settle at the entry", yaw),
        Waypoint(2.0, (qx, qy, qz + 0.002 + tcp), grip, "set down", yaw),
        Waypoint(0.8, (qx, qy, qz + 0.002 + tcp), grip, "settle", yaw),
        Waypoint(1.5, (qx, qy, qz + 0.002 + tcp), release, "release", yaw),
        Waypoint(1.0, (qx, qy, CLEAR_Z), release, "retreat", yaw),
    ]


def _wait(arm, s):
    return [Waypoint(s, (0.12, 0.42 if arm == "left" else -0.42, TOP + 0.20), FINGER_OPEN, "wait clear")]


def demo():
    """Right arm: the sole board onto the lawn across the marks (gripped across its 8 cm width, yaw +90). Then both
    arms at once, each on its own side: its base plate onto the board at its mark (gripped by the collar), then its
    tube from behind the lawn, carried upright and lowered into the plate's collar. Both arms then move clear."""
    arms = {"left": [], "right": []}
    plate_at = {s: (*PLATE_START[s], TOP) for s in PLATES}  # x, y, bottom z
    for step in DEMO_PLAN:
        if step[0] == "board":
            mv = _move((*BOARD_START, TOP), (LX, 0.0, LAWN), width=BOARD_W, opening=FINGER_OPEN,
                       travel_bottom=TOP + 0.05, entry=0.01, yaw=math.pi / 2, tcp=BOARD_TCP)
            arms["right"] += mv
            arms["left"] += _wait("left", sum(w.duration for w in mv))
        elif step[0] == "plates":
            z = LAWN + (BOARD_T if step[1] == "board" else 0.0)
            for s in PLATES:
                mx, my = MARKS[s][0] + PLATE_AIM[s][0], MARKS[s][1] + PLATE_AIM[s][1]
                arms[s] += _move(plate_at[s], (mx, my, z), width=COLLAR_IN + 2 * COLLAR_WALL, opening=-0.040,
                                 travel_bottom=LAWN + BOARD_T + 0.02, entry=0.01, tcp=PLATE_TCP)
                plate_at[s] = (MARKS[s][0], MARKS[s][1], z)
        elif step[0] == "tubes":
            for s in TUBES:
                if step[1] == "plates":
                    px, py, pz = plate_at[s]
                    dest, entry = (px + TUBE_AIM[s][0], py + TUBE_AIM[s][1], pz + PLATE_T), COLLAR_H + 0.008
                else:
                    dest, entry = (*MARKS[s], LAWN), 0.01
                # pick with a 50 mm opening (probe 4: closing from 94 mm knocked a tube over), release wide (probe 3:
                # a 50 mm release nudged a seated tube to 5.1 deg on the way up)
                arms[s] += _move((*TUBE_START[s], TOP), dest, width=2 * TUBE_R, opening=-0.025, release=FINGER_OPEN,
                                 travel_bottom=LAWN + BOARD_T + COLLAR_TOP + 0.02, entry=entry, tcp=TUBE_TCP,
                                 squeeze=TUBE_SQUEEZE)
    for s in arms:
        arms[s] += _wait(s, 1.5)
    return arms
