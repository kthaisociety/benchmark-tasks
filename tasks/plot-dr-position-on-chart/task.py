"""On the station table: a paper nautical chart, 1:50,000 Mercator, about 296 x 417 mm, lies taped flat in front of
both arms with north away from the robot. It covers open sea from 50°00'N to 50°07'N and from 3°00'W to 2°44'W. Its
border carries the usual scales: latitude down the left and right sides (whole minutes in alternate black bands,
tenths as ticks), longitude along the near and far edges. A red buoy symbol is printed near the near-left corner, and
a pencilled course line runs from it on 045° true across the chart. A brass pair of navigator's dividers lies closed
on the chart near its near-right corner. A yellow position marker (a weighted disc with a black knob) stands on the
table just in front of the chart's near edge, in the middle.

The robot must put the marker on the chart at the boat's dead-reckoning position: 6 nautical miles from the buoy
along the course line. A nautical mile is a minute of latitude, read off the side scales; a minute of longitude is
shorter here, so the top and bottom scales give the wrong distance.

Scoring is in scoring.py, the chart's geometry in chart.py.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Variants of the scripted demo: CDR_VARIANT=<name> (see VARIANTS).
"""

from __future__ import annotations

import collections
import json
import math
import os
import pathlib
import sys
import warnings

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import chart as C  # noqa: E402
import scoring as S  # noqa: E402
from partkit import Hinge, Sheet, env, on_reset, part  # noqa: E402
from yam_arena.constants import ARM_JOINTS, FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint, pick_place_waypoints  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
PAPER_TOP = S.paper_top(TOP)

INSTRUCTION = ("The boat passed the red buoy at the start of the pencilled course line and has since run 6 nautical "
               "miles along that line. Put the position marker on the chart where the boat is now, then let go of it.")

# The finest scale ticks; CDR_TICKS=0.5 prints half-minute ticks only (fewer prims) if the full chart is slow to load.
TICK_STEP = float(os.environ.get("CDR_TICKS", "0.1"))
CHART = Sheet("chart", pos=(*C.PAPER_CENTRE, TOP + C.PAPER_THICK / 2), size=C.PAPER_SIZE, color=C.PAPER,
              decals=tuple(C.decals(TICK_STEP)))  # no mass: taped flat to the table

# The position marker: a yellow disc (25 mm, 6 mm thick) with a black knob (12 mm, 16 mm tall) to lift it by.
YELLOW, BLACK = (0.95, 0.78, 0.10), (0.06, 0.06, 0.07)
MARKER_START = (0.115, 0.0)  # on the table in front of the chart's near edge, on the midline
MARKER = part("marker", pos=(*MARKER_START, TOP + S.MARKER_HALF_H + 0.001), mass=0.03, color=YELLOW, friction=1.0,
              boxes=[((0.0, 0.0, 0.0), (0.012, 0.012, S.DISC_H - 0.001), YELLOW)],
              cylinders=[((0.0, 0.0, 0.0), S.DISC_R, S.DISC_H, YELLOW),
                         ((0.0, 0.0, S.MARKER_HALF_H + S.KNOB_H / 2), S.KNOB_R, S.KNOB_H, BLACK)])

# Navigator's dividers: two brass legs (150 mm from the hinge to the steel point, 8 x 7 mm) on a hinge with a 20 mm
# head, lying on the chart. Part frame: the hinge at the origin, at mid-height of the legs; leg A (the part itself)
# along +x; leg B on the hinge, opened 30 degrees counterclockwise so each leg can be taken on its own (at 8 degrees a
# closing gripper pushed leg B back onto leg A). No spring and a trace of damping: like real dividers, they stay as
# they are left. WHY 0.0005: at 0.02 N m s/rad, swinging leg B dragged leg A round with it on the simulator.
BRASS, STEEL = (0.78, 0.62, 0.25), (0.62, 0.64, 0.68)
LEG_L, LEG_W, LEG_H = 0.150, 0.008, 0.007
DIV_START_DEG = 30.0
_LEG = [((0.072, 0.0, 0.0), (0.136, LEG_W, LEG_H), BRASS),
        ((0.145, 0.0, -0.002), (0.010, 0.003, 0.003), STEEL)]  # the point
DIV_HEAD = (0.175, -0.035)  # on the chart's near-right part, clear of the scales, the course line and the answer
DIV_YAW = -math.pi / 2  # legs toward the robot's right (-y)
DIVIDERS = part("dividers", pos=(*DIV_HEAD, PAPER_TOP + LEG_H / 2 + 0.0005), mass=0.06, yaw=DIV_YAW, color=BRASS,
                friction=1.0, boxes=_LEG, cylinders=[((0.0, 0.0, 0.0025), 0.010, 0.012, BRASS)],
                hinges=[Hinge("leg_b", pivot=(0.0, 0.0, 0.0), boxes=tuple(_LEG), lower_deg=0.0, upper_deg=120.0,
                              start_deg=DIV_START_DEG, damping=0.0005, mass=0.02)])

OBJECTS = [CHART, MARKER, DIVIDERS]


# -- scoring: the mark (scoring.py), let go and left alone ------------------------------------------------------------
_ARMS = (("left_robot", "left"), ("right_robot", "right"))
_S = {"objects": None, "value": 0.0, "marks": collections.deque(maxlen=S.SETTLE_STEPS + 1), "warned": False,
      "step": 0, "trial": 0}


@on_reset
def _reset(environment=None):
    _S.update(objects=None, value=0.0, warned=False, step=0, trial=_S["trial"] + 1)  # the module lives across trials
    _S["marks"].clear()
    if os.environ.get("CDR_DEBUG"):
        _debug_dump()


def _debug_dump():
    """Probe-only: what the simulator built for the chart and the dividers."""
    try:
        from isaaclab.sim.utils.stage import get_current_stage
        scene = env().unwrapped.scene
        d = scene["dividers"]
        print("CDR_DEBUG joints", list(d.joint_names), d.data.joint_pos.tolist(),
              "bodies", list(d.body_names), flush=True)
        for k in ("joint_damping", "joint_stiffness", "joint_friction_coeff", "joint_armature", "joint_vel_limits"):
            v = getattr(d.data, k, None)
            print("CDR_DEBUG", k, None if v is None else getattr(v, "torch", v).tolist(), flush=True)
        stage = get_current_stage()
        for prim in stage.Traverse():
            p = str(prim.GetPath())
            if ("chart" in p and "/d" not in p.rsplit("/", 1)[-1][:2]) or "dividers" in p:
                print("CDR_DEBUG prim", p, prim.GetTypeName(), flush=True)
        n = sum(1 for prim in stage.Traverse() if "/decals/d" in str(prim.GetPath()))
        print("CDR_DEBUG decal prims", n, flush=True)
    except Exception as e:  # noqa: BLE001
        print("CDR_DEBUG failed", repr(e), flush=True)


def _grippers_clear(m):
    """Both grippers clear of the marker, from the arms' joint positions and the framework's kinematics; None (counts
    as not let go) if the arms can't be read."""
    try:
        from yam_arena import ik, station
        scene = env().unwrapped.scene
        clear = True
        for name, side in _ARMS:
            robot = scene[name]
            ids = robot.find_joints(ARM_JOINTS, preserve_order=True)[0]
            q = robot.data.joint_pos
            q = getattr(q, "torch", q)[0]
            q = np.asarray(q.detach().cpu().numpy() if hasattr(q, "detach") else q, dtype=float)
            tcp, rot = ik.tcp_pose(q[list(ids)], station.BASES[side])
            ok = S.gripper_clear(tcp, np.asarray(rot)[:, 2], m.pos, m.quat_xyzw)
            if os.environ.get("CDR_DEBUG") and _S["step"] % 40 == 0:
                print("CDR_DEBUG arm", side, list(robot.joint_names), np.round(q[list(ids)], 3).tolist(),
                      np.round(np.asarray(tcp), 3).tolist(), bool(ok), flush=True)
            clear &= ok
        return clear
    except Exception as error:  # noqa: BLE001 - an unexpected embodiment: never award credit
        if not _S["warned"]:
            _S["warned"] = True
            warnings.warn(f"chart-dr: can't read the arms ({type(error).__name__}: {error}); scoring 0",
                          RuntimeWarning, stacklevel=2)
        return None


def _update(objects):
    """Score once per control step (success() and score() get the same state dict)."""
    if _S["objects"] is objects:
        return _S["value"]
    m = objects[MARKER.name]
    _S["marks"].append(S.mark_point(m.pos, m.quat_xyzw))
    r = S.read(m.pos, m.quat_xyzw, TOP)
    clear = _grippers_clear(m) if S.points(r) > 0 else False
    _S.update(objects=objects, value=S.final(r, clear, _S["marks"]))
    _S["step"] += 1
    if TRACE:
        _trace(objects, m, r, clear)
    return _S["value"]


def _trace(objects, m, r, clear):
    """One JSON line per control step to <CDR_TRACE>/trace-<trial>.jsonl (export_media.py captions the video from it)."""
    p = _S["marks"][-1]
    d = objects[DIVIDERS.name]
    spread = getattr(d, "hinges", {}).get("leg_b")
    row = {"time_s": round(_S["step"] / 20, 2), "mark": [round(float(v), 4) for v in p[:2]],
           "height_mm": round(1000 * (float(p[2]) - PAPER_TOP), 1), "tilt_deg": round(S.tilt_deg(m.quat_xyzw), 1),
           "off_nm": round(float(r.off_nm), 3), "lying": bool(r.lying), "clear": None if clear is None else bool(clear),
           "score": float(_S["value"]),
           "dividers_deg": None if spread is None else round(math.degrees(spread), 1),
           "dividers_head": [round(float(v), 4) for v in d.pos[:2]],
           "leg_a_yaw_deg": round(math.degrees(math.atan2(*(S.rotation(d.quat_xyzw)[[1, 0], 0]))), 1)}
    out = pathlib.Path(TRACE)
    out.mkdir(parents=True, exist_ok=True)
    with (out / f"trace-{_S['trial']}.jsonl").open("a") as f:
        f.write(json.dumps(row) + "\n")


def success(objects):
    """The marker within 0.2 nm of the DR position, let go and left alone for the last second; holding this for 1 s
    ends the trial with 1.0."""
    return _update(objects) == 1.0


def score(objects):
    return _update(objects)


def distance(objects):
    """From the marked point to the DR position (m, on the chart)."""
    m = objects[MARKER.name]
    p = S.mark_point(m.pos, m.quat_xyzw)
    return float(math.dist(p[:2], C.DR_XY))


# -- scripted demo and shortcut variants ------------------------------------------------------------------------------
# Lift the marker by its knob: pad tips 12 mm above the disc's base, mid-knob.
GRIP_TIPS = 0.012
PICK_Z = TOP + 0.001 + GRIP_TIPS + PAD_TIP_PAST_TCP
SET_Z = PAPER_TOP + GRIP_TIPS + PAD_TIP_PAST_TCP + 0.001  # the disc's base 1 mm above the paper
TRAVEL_Z = TOP + 0.10
PARK = {"right": (0.12, -0.42, TOP + 0.20), "left": (0.12, 0.42, TOP + 0.20)}
# WHY: a placement aim offset, to be measured on the official stack on the first probe (where the marker lands against
# where the TCP was sent). Zero until traced.
PLACE_AIM = (0.0, 0.0)

_UX, _UY = math.cos(math.radians(C.COURSE_DEG)), -math.sin(math.radians(C.COURSE_DEG))  # along the course line
_NM_HERE = C.lat_minute(C.DR[0])  # one nautical mile on the chart at the DR position (m)


def _along(nm):
    return C.DR_XY[0] + nm * _NM_HERE * _UX, C.DR_XY[1] + nm * _NM_HERE * _UY


# name: (target x, y on the chart or None, expected score)
VARIANTS = {
    "demo": (*C.DR_XY, 1.0),  # 6 nm off the latitude scale
    "dividers": (*C.DR_XY, 1.0),  # the same, after opening the dividers to the 6 nm span (feasibility probe)
    "close": (*_along(0.15), 1.0),  # 0.15 nm past it
    "half": (*_along(-0.35), 0.5),  # 0.35 nm short
    "lon_scale": (*C.LON_SCALE_XY, 0.0),  # 6 minutes off the longitude scale: 2.1 nm short
    "at_buoy": (*C.BUOY_XY, 0.0),
    "line_end": (*C.LINE_END_XY, 0.0),  # the end of the pencilled line (7 nm)
    "held": (*C.DR_XY, 0.0),  # the right spot, never let go
    "nothing": (None, None, 0.0),
}
VARIANT = os.environ.get("CDR_VARIANT", "demo")
TRACE = os.environ.get("CDR_TRACE")  # a directory: one JSON line per step per trial (see _trace)

# Opening the dividers (variant "dividers"): the right arm takes leg B 130 mm from the hinge and swings it round the
# hinge until the points are 6 nm apart (2 x 150 mm x sin(angle / 2) = 6 nm). Leg A is not held: the hinge turns
# freely enough that leg A stays where it lies on the paper. (Holding it with the left arm crosses the arms.)
DIV_GRIP_R = 0.130  # WHY: nearer the hinge the first grip needs more wrist turn than joint 6 allows (IK check)
DIV_SPAN = C.RUN_NM * C.lat_minute((C.BUOY[0] + C.DR[0]) / 2)
DIV_OPEN_DEG = math.degrees(2 * math.asin(DIV_SPAN / (2 * LEG_L)))
DIV_TIPS = LEG_H / 2 + 0.0005  # grip leg B at mid-height
DIV_Z = PAPER_TOP + DIV_TIPS + PAD_TIP_PAST_TCP


def _leg_b_grip(angle_deg):
    """(x, y, grip yaw) to hold leg B at DIV_GRIP_R when the dividers are open angle_deg."""
    a = DIV_YAW + math.radians(angle_deg)  # leg B's direction
    return DIV_HEAD[0] + DIV_GRIP_R * math.cos(a), DIV_HEAD[1] + DIV_GRIP_R * math.sin(a), a


def _open_dividers():
    x0, y0, a0 = _leg_b_grip(DIV_START_DEG)
    steps = [Waypoint(1.5, (x0, y0, TRAVEL_Z), FINGER_OPEN, "over leg B", a0),
             Waypoint(1.5, (x0, y0, DIV_Z), FINGER_OPEN, "down to leg B", a0),
             Waypoint(0.8, (x0, y0, DIV_Z), FINGER_CLOSED, "take leg B", a0)]
    n = 8
    for i in range(1, n + 1):  # the swing, in short arcs so the TCP follows the circle
        x, y, a = _leg_b_grip(DIV_START_DEG + (DIV_OPEN_DEG - DIV_START_DEG) * i / n)
        steps.append(Waypoint(0.6, (x, y, DIV_Z), FINGER_CLOSED, f"swing {i}/{n}", a))
    x, y, a = _leg_b_grip(DIV_OPEN_DEG)
    steps += [Waypoint(0.6, (x, y, DIV_Z), FINGER_OPEN, "let go of leg B", a),
              Waypoint(1.0, (x, y, TRAVEL_Z), FINGER_OPEN, "up", a)]
    return steps


def demo():
    """One arm lifts the marker by its knob, carries it over the target, sets it down 1 mm above the chart, lets go,
    rises and moves clear of the top camera: the right arm, or the left for a target on the chart's left side. The
    other arm stays parked."""
    x, y, _ = VARIANTS[VARIANT]
    if x is None:
        return {arm: [Waypoint(2.0, PARK[arm], FINGER_OPEN, "stay clear")] for arm in ("left", "right")}
    arm = "left" if y > 0.05 else "right"
    other = "right" if arm == "left" else "left"
    plan = {other: [Waypoint(2.0, PARK[other], FINGER_OPEN, "stay clear")]}
    pick = (*MARKER_START, PICK_Z)
    place = (x + PLACE_AIM[0], y + PLACE_AIM[1], SET_Z)
    steps = pick_place_waypoints(pick, place, travel_z=TRAVEL_Z)
    if VARIANT == "held":
        return {**plan, arm: [*steps[:6], Waypoint(60.0, place, FINGER_CLOSED, "keep holding it")]}
    if VARIANT == "dividers":  # the right arm opens the dividers first, then takes the marker
        steps = _open_dividers() + steps
    steps.append(Waypoint(1.5, PARK[arm], FINGER_OPEN, "move clear"))
    return {**plan, arm: steps}
