"""On the station table: a pipe run on a low bench, 38 cm in front of the arm bases, running left to right with its axis
5.7 cm above the table. From left to right: an existing galvanised steel pipe (grey), a 12 cm gap, a new copper pipe
section (18 cm), a second 12 cm gap and an existing copper pipe. Each gap has a grey saddle plate with two low rails, so
a fitting laid in it lines up with the pipes. Nearer the arms (20 cm in front of the bases) a dark tray holds three
fittings in slots, each lying with its axis pointing at the arms: a galvanised steel coupling (left), a dielectric
union (middle: a brass half and a galvanised half with a black insulating ring and a steel nut between them) and a
copper coupling (right). All three are 9 cm long and 5 cm across. The union's brass end points at the arms.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Variants of the scripted demo: DU_VARIANT=demo|reversed|nearest|union_b|only_a|only_b|held|nothing (see demo()).
"""

import math
import os
import warnings
from collections import deque
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import scoring as S  # noqa: E402
from partkit import env, on_reset, part  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import ARM_JOINTS, FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = ("Join the new copper pipe into the line: seat a fitting from the tray in each of the two gaps, "
               "then let go.")

COPPER = (0.80, 0.47, 0.27)
BRASS = (0.82, 0.64, 0.30)
GALV = (0.64, 0.67, 0.70)
STEEL_DARK = (0.48, 0.51, 0.55)
INSULATOR = (0.06, 0.06, 0.06)
BENCH = (0.36, 0.30, 0.24)
SADDLE = (0.42, 0.44, 0.47)
RAIL = (0.25, 0.26, 0.28)
TRAY = (0.16, 0.19, 0.23)

PIPE_R = 0.020
TRAY_X = 0.20
TRAY_TOP = 0.012
SLOT_Y = {"steel_coupling": 0.10, "union": 0.0, "cu_coupling": -0.10}   # left to right, seen from behind the arms


def _box(x0, x1, y0, y1, z0, z1, color):
    return (((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), color)


def _pipe(y0, y1, color, r=PIPE_R):
    return ((0.0, (y0 + y1) / 2, S.AXIS_Z), r, abs(y1 - y0), color, "Y")


# -- the bench: one heavy body, origin on the table under the pipe line at y = 0 ------------------------------------
_half = S.GAP_LEN / 2
_ya, _yb = S.GAP_Y["A"], S.GAP_Y["B"]
_post_top = S.AXIS_Z - PIPE_R
BENCH_BOXES = [_box(-0.05, 0.05, -0.32, 0.32, 0.0, 0.010, BENCH)]                         # bench board
for _y in (0.26, 0.045, -0.045, -0.26):                                                     # pipe supports
    BENCH_BOXES.append(_box(-0.015, 0.015, _y - 0.015, _y + 0.015, 0.010, _post_top, RAIL))
for _g in (_ya, _yb):                                                                       # saddle under each gap
    BENCH_BOXES.append(_box(-0.035, 0.035, _g - _half, _g + _half, 0.010, S.PLATE_TOP, SADDLE))
    for _s in (1, -1):
        _x0, _x1 = sorted((_s * S.RAIL_IN, _s * (S.RAIL_IN + 0.010)))
        BENCH_BOXES.append(_box(_x0, _x1, _g - _half, _g + _half, S.PLATE_TOP, S.PLATE_TOP + S.RAIL_H, RAIL))
# the fittings tray, nearer the arms: a plate with two rails per slot, running front to back
_tx = TRAY_X - S.PIPE_X
BENCH_BOXES.append(_box(_tx - 0.065, _tx + 0.065, -0.16, 0.16, 0.0, TRAY_TOP, TRAY))
for _y in SLOT_Y.values():
    for _s in (1, -1):
        _y0, _y1 = sorted((_y + _s * S.RAIL_IN, _y + _s * (S.RAIL_IN + 0.010)))
        BENCH_BOXES.append(_box(_tx - 0.06, _tx + 0.06, _y0, _y1, TRAY_TOP, TRAY_TOP + S.RAIL_H, RAIL))

BENCH_PIPES = [
    _pipe(_ya + _half, 0.31, GALV),                 # existing galvanised steel pipe, left
    _pipe(_ya - _half, _yb + _half, COPPER),        # the new copper section, between the gaps
    _pipe(_yb - _half, -0.31, COPPER),              # existing copper pipe, right
]
PIPEWORK = part("pipework", pos=(S.PIPE_X, 0.0, TOP + 0.001), mass=20.0, boxes=BENCH_BOXES, cylinders=BENCH_PIPES,
                color=BENCH, friction=0.8)

# -- the fittings: axis along local y; the union's brass end at local +y ------------------------------------------------
_core = [((0.0, 0.0, 0.0), (0.01, 0.01, 0.01), STEEL_DARK)]   # partkit needs one box; this one sits inside the body
UNION_CYL = [
    ((0.0, 0.0285, 0.0), S.FIT_R, 0.033, BRASS, "Y"),          # brass sweat end (soldered to copper)
    ((0.0, 0.009, 0.0), 0.0262, 0.006, INSULATOR, "Y"),        # insulating ring
    ((0.0, -0.003, 0.0), S.NUT_R, 0.018, STEEL_DARK, "Y"),     # union nut
    ((0.0, -0.0285, 0.0), S.FIT_R, 0.033, GALV, "Y"),          # galvanised threaded end
]
CU_CYL = [((0.0, 0.0, 0.0), S.FIT_R, S.FIT_LEN, COPPER, "Y"),
          ((0.0, 0.0, 0.0), 0.0255, 0.004, (0.66, 0.36, 0.20), "Y")]   # the dimpled stop ring
STEEL_CYL = [((0.0, 0.0, 0.0), S.FIT_R, 0.07, GALV, "Y"),
             ((0.0, 0.040, 0.0), 0.0262, 0.010, GALV, "Y"), ((0.0, -0.040, 0.0), 0.0262, 0.010, GALV, "Y")]
WIDEST = {"union": S.NUT_R, "cu_coupling": 0.0255, "steel_coupling": 0.0262}
START_YAW = math.pi / 2          # local +y (the union's brass end) points at the arms (-x)


def _fitting(name, cylinders, mass):
    z = TOP + TRAY_TOP + S.seat_height(WIDEST[name]) + 0.001
    return part(name, pos=(TRAY_X, SLOT_Y[name], z), yaw=START_YAW, mass=mass, boxes=_core, cylinders=cylinders,
                color=cylinders[0][3], friction=0.8)


UNION = _fitting("union", UNION_CYL, 0.30)
CU_COUPLING = _fitting("cu_coupling", CU_CYL, 0.15)
STEEL_COUPLING = _fitting("steel_coupling", STEEL_CYL, 0.18)
OBJECTS = [PIPEWORK, UNION, CU_COUPLING, STEEL_COUPLING]

# -- scoring -----------------------------------------------------------------------------------------------------------
PIPEWORK_START = S.Pose(pos=(S.PIPE_X, 0.0, 0.001))
_ARMS = (("left_robot", "left"), ("right_robot", "right"))
_S = {"knocked": False, "hist": {}, "last": None, "warned": False}


@on_reset
def _reset(environment=None):
    # the task module lives across trials; clear the record here
    _S.update(knocked=False, hist={name: deque(maxlen=2 * S.SETTLE_STEPS) for name in S.FITTINGS}, last=None)


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    for step in ("detach", "cpu", "numpy"):
        if hasattr(x, step):
            x = getattr(x, step)()
    return np.asarray(x, dtype=float)


def _grippers():
    """Both grippers' (TCP, approach direction), z above the table top, from the arms' joint angles and the framework's
    forward kinematics; None (with one warning) if the arms can't be read."""
    try:
        scene = env().unwrapped.scene
        out = []
        for name, side in _ARMS:
            robot = scene[name]
            names = list(robot.joint_names)
            q = _np(robot.data.joint_pos)[0]
            tcp, rot = ik.tcp_pose(q[[names.index(j) for j in ARM_JOINTS]], station.BASES[side])
            tcp = np.array(tcp, dtype=float)
            tcp[2] -= TOP
            out.append((tcp, np.asarray(rot, dtype=float)[:, 2]))
        return out
    except Exception as error:  # no environment (tests) or an unexpected robot: nothing counts as let go
        if not _S["warned"]:
            _S["warned"] = True
            warnings.warn(f"join-copper-to-galvanised-pipe: can't read the arms ({error!r}); no fitting counts as let go",
                          RuntimeWarning, stacklevel=2)
        return None


def _pose(state) -> S.Pose:
    x, y, z = (float(v) for v in state.pos)
    return S.Pose(pos=(x, y, z - TOP), quat_xyzw=tuple(float(v) for v in state.quat_xyzw),
                  lin_vel=tuple(float(v) for v in state.lin_vel), ang_vel=tuple(float(v) for v in state.ang_vel))


def evaluate(objects):
    """Score this step once (score() and success() both call it with the same state dict)."""
    last = _S["last"]
    if last is not None and last[0] is objects:
        return last[1]
    if not _S["hist"]:
        _reset()
    if S.knocked(_pose(objects["pipework"]), PIPEWORK_START):
        _S["knocked"] = True   # latched for the whole trial
    fittings = {name: _pose(objects[name]) for name in S.FITTINGS}
    grippers = _grippers()
    free = {}
    for name, p in fittings.items():
        _S["hist"][name].append(p.pos)
        free[name] = S.let_go(p, grippers) and S.settled(_S["hist"][name])
    result = {"score": S.score(fittings, _S["knocked"], free), "free": free, "wrong": S.wrong_fittings(fittings),
              "knocked": _S["knocked"], "fittings": fittings, "grippers": grippers}
    _S["last"] = (objects, result)
    _trace(result)
    return result


def _trace(result):
    """DU_TRACE=<file>: append the fittings' poses, the grippers and the score every 10 steps (for building)."""
    path = os.environ.get("DU_TRACE")
    if not path:
        return
    _S["step"] = _S.get("step", 0) + 1
    if _S["step"] % 10:
        return
    parts = [f"step {_S['step']} score {result['score']} knocked {result['knocked']} wrong {result['wrong']}"]
    for name, p in result["fittings"].items():
        a = p.axis()
        parts.append(f"{name} pos ({p.pos[0]:.3f},{p.pos[1]:.3f},{p.pos[2]:.4f}) axis ({a[0]:+.2f},{a[1]:+.2f},"
                     f"{a[2]:+.2f}) free {result['free'][name]}")
    for tcp, _ in result["grippers"] or []:
        parts.append(f"tcp ({tcp[0]:.3f},{tcp[1]:.3f},{tcp[2]:.3f})")
    with open(path, "a") as f:
        f.write(" | ".join(parts) + "\n")


def score(objects):
    return evaluate(objects)["score"]


def success(objects):
    return evaluate(objects)["score"] == 1.0


# -- scripted demo -----------------------------------------------------------------------------------------------------
HOVER = TOP + 0.16
GRIP_DZ = 0.005                  # TCP this far above the fitting's axis: the pads centred on it
RELEASE_DZ = 0.003               # let go with the fitting 3 mm above its seat
PARK = {"left": (0.12, 0.42, TOP + 0.20), "right": (0.12, -0.42, TOP + 0.20)}


def _tray_grip(name):
    return (TRAY_X, SLOT_Y[name], TOP + TRAY_TOP + S.seat_height(WIDEST[name]) + GRIP_DZ)


def _move(name, gap, turn, wait=0.0, arm="left", release=True):
    """Pick ``name`` from its slot (fingers across it), turn it ``turn`` rad about the vertical on the way, and lay it
    in ``gap``'s saddle."""
    gx, gy, gz = _tray_grip(name)
    py = S.GAP_Y[gap]
    place_z = TOP + S.SEAT_Z[name] + GRIP_DZ + RELEASE_DZ
    steps = []
    if wait:
        steps.append(Waypoint(wait, (0.20, 0.25 if arm == "left" else -0.25, HOVER), FINGER_OPEN, "wait clear"))
    steps += [
        Waypoint(1.8, (gx, gy, HOVER), FINGER_OPEN, f"{name}: above"),
        Waypoint(1.5, (gx, gy, gz), FINGER_OPEN, f"{name}: down"),
        Waypoint(0.8, (gx, gy, gz), FINGER_CLOSED, f"{name}: grip"),
        Waypoint(1.2, (gx, gy, HOVER), FINGER_CLOSED, f"{name}: lift"),
        Waypoint(2.5, (S.PIPE_X, py, HOVER), FINGER_CLOSED, f"{name}: carry and turn", turn),
        Waypoint(1.5, (S.PIPE_X, py, place_z), FINGER_CLOSED, f"{name}: lower into gap {gap}", turn),
    ]
    if not release:
        return steps + [Waypoint(30.0, (S.PIPE_X, py, place_z), FINGER_CLOSED, f"{name}: keep holding", turn)]
    return steps + [
        Waypoint(0.8, (S.PIPE_X, py, place_z), FINGER_OPEN, f"{name}: let go", turn),
        Waypoint(1.0, (S.PIPE_X, py, HOVER), FINGER_OPEN, "up", turn),
        Waypoint(1.5, PARK[arm], FINGER_OPEN, "park"),
    ]


def _idle(arm):
    return [Waypoint(2.0, PARK[arm], FINGER_OPEN, "park")]


def demo():
    """The left arm lays the union in gap A, turned a quarter turn counter-clockwise (seen from above) so its brass end
    faces the copper section; then the right arm lays the copper coupling in gap B. ``DU_VARIANT`` plays the shortcuts:
    reversed (union turned clockwise, brass end to the steel pipe), held (both placed right but never let go), nearest (each arm takes the fitting nearest its
    gap: steel coupling in A, copper coupling in B), union_b (the right arm puts the union in B), only_a, only_b and
    nothing."""
    variant = os.environ.get("DU_VARIANT", "demo")
    q = math.pi / 2
    union_a = _move("union", "A", q)
    copper_b = _move("cu_coupling", "B", q, wait=9.0, arm="right")
    plans = {
        "demo": {"left": union_a, "right": copper_b},
        "reversed": {"left": _move("union", "A", -q), "right": copper_b},
        "nearest": {"left": _move("steel_coupling", "A", q), "right": copper_b},
        "union_b": {"left": _idle("left"), "right": _move("union", "B", q, arm="right")},
        "only_a": {"left": union_a, "right": _idle("right")},
        "only_b": {"left": _idle("left"), "right": copper_b},
        "held": {"left": _move("union", "A", q, release=False),
                 "right": _move("cu_coupling", "B", q, wait=9.0, arm="right", release=False)},
        "nothing": {"left": _idle("left"), "right": _idle("right")},
    }
    return plans[variant]


EXPECTED = {"demo": 1.0, "reversed": 0.0, "nearest": 0.0, "union_b": 0.0, "only_a": 0.75, "only_b": 0.25,
            "held": 0.0, "nothing": 0.0}
