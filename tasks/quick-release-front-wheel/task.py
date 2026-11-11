"""On the station table, in front of the right arm, at 1:4 scale: a bicycle's front fork held upside down in a
workshop stand (red, dropouts at the top, a white reflector on the crown facing the bike's front, which is to the
robot's left), and a 20-inch front wheel standing upright in a rubber wheel stand to the right of it, in the fork's
plane. The wheel has a quick-release skewer. Its black lever is on the end of the axle facing the robot and is fully
open, pointing straight out along the axle.

The wheel turns freely on its axle. The lever swings about its cam pivot (open, part-way or closed) and the skewer
turns about the axle while the lever is open. The fork and the wheel stand are fixed.

QR_VARIANT=<name> selects a scripted shortcut for the checks in README.md (see VARIANTS). QR_TRACE_DIR=<dir> writes a
per-step trace (trace-<trial>.jsonl).
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bike  # noqa: E402
import layout as L  # noqa: E402
import parts  # noqa: E402
import scoring as S  # noqa: E402
from mechanism import Lever  # noqa: E402
from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import Box  # noqa: E402

SCENE = "station"
TOP = L.TOP
SCORE_OVER = "end"
INSTRUCTION = "Put the front wheel back in the bike's fork so that the bike is ready to ride."

OBJECTS = [
    bike.FORK,
    bike.WHEEL,
    *[Box(f"stand_{n}", pos=(x, L.STAND_XY[1], TOP + L.STAND_BLOCK[2] / 2), size=L.STAND_BLOCK, color=bike.RUBBER)
      for n, x in zip(("far", "near"), bike.STAND_X)],
]

# -- per trial: the scorer, the lever's mechanism and the trace --------------------------------------------------------
STATE = {"trial": S.Trial(), "lever": Lever(), "n": 0, "t": 0.0, "targets": (0.0, 0.0)}
TRACE_DIR = os.environ.get("QR_TRACE_DIR")


def reset_trial():
    STATE.update(trial=S.Trial(), lever=Lever(), n=STATE["n"] + 1, t=0.0, targets=(0.0, 0.0))


@parts.on_reset
def _reset(sim):
    reset_trial()
    sim.set_target("wheel", "cam", 0.0)
    sim.set_target("wheel", "aim", 0.0)


@parts.on_step
def _mechanism(sim):
    w = sim.states()["wheel"]
    cam, aim = STATE["lever"].targets(float(w.joints["cam"]), float(w.joints["aim"]))
    if (cam, aim) != STATE["targets"]:
        sim.set_target("wheel", "cam", cam)
        sim.set_target("wheel", "aim", aim)
        STATE["targets"] = (cam, aim)
    STATE["t"] = sim.t


def score(objects):
    trial = STATE["trial"]
    value = trial.step(objects)
    if TRACE_DIR:
        w = objects["wheel"]
        r = lambda v: [round(float(a), 4) for a in v]  # noqa: E731
        sides = {s: S.at_dropout(w, k) for s, k in (("near", -1), ("far", 1))}
        row = {"t": round(STATE["t"], 2), "score": value, "what": trial.what, "fault": trial.fault,
               "axle": r(w.pos), "closure_deg": round(S.closure(w), 1),
               "aim_deg": round(math.degrees(float(w.joints["aim"])), 1), "lever_aim_deg": round(S.lever_aim(w), 1),
               "dropouts": {s: (r(a) if a is not None else None) for s, a in sides.items()},
               "targets_deg": [round(math.degrees(v), 1) for v in STATE["targets"]], "still": S.still(w)}
        path = pathlib.Path(TRACE_DIR) / f"trace-{max(STATE['n'], 1)}.jsonl"
        with path.open("a") as f:
            f.write(json.dumps(row) + "\n")
    return value


# No success(): the episode always runs to its step limit and is scored at the end, so a scripted or model run can
# finish by moving the arm clear of the camera (an early success stop cut the demo with the arm over the lever).


# -- scripted demo: the right arm does everything, the left arm parks ---------------------------------------------------
# Reads only what the cameras show: where the wheel stands, where the fork's dropouts are, where the lever is.
VARIANTS = {
    "demo": "fit the wheel, press the lever closed along the arc",
    "nothing": "both arms park",
    "open": "fit the wheel, leave the lever open",
    "half_open": "fit the wheel, press the lever only to about 40 deg",
    "one_side": "fit the wheel cocked (turned 17 deg, 2 cm off), lever open",
}
YAW_TYRE = math.pi / 2  # fingers close along x, across the tyre
GRIP_OPEN = -0.030  # fingers 60 mm apart
GRIP_DEPTH = 0.012  # pads reach this far down the tyre's top
PARK_R, PARK_L = (0.20, -0.30, TOP + 0.18), (0.20, 0.30, TOP + 0.18)
FX, FY = L.FORK_XY
SX, SY = L.STAND_XY
CARRY_AXLE = L.MOUTH_TOP + L.AXLE_SIDE / 2 + 0.012  # the axle ends 12 mm above the dropouts' mouths
RELEASE_AXLE = L.H_SEAT + 0.006  # let go with the axle 6 mm above its seat, inside the slots
PRESS = 0.004  # how far the finger is commanded into the lever's face



def _grip_tcp(axle_z):
    """TCP height gripping the tyre's top with the wheel's axle at axle_z (above the table)."""
    return TOP + axle_z + L.R_OUT - GRIP_DEPTH + L.TIP


def _lever_tcp(pivot, c_deg, press=PRESS, lift=0.0):
    """TCP pressing the lever's paddle at closure c_deg: the closed fingers' front-bottom edge on the paddle's outer
    face (its top when open), `press` into it."""
    c = math.radians(c_deg)
    u = (-math.cos(c), 0.0, -math.sin(c))
    n = (-math.sin(c), 0.0, math.cos(c))
    px, py, pz = pivot
    face = [p + L.PADDLE_R * a + L.LEVER["thick"] / 2 * b for p, a, b in zip((px, py, pz), u, n)]
    return (face[0] - press * n[0] - L.FX_HALF * math.sin(c), face[1], face[2] - press * n[2] + L.TIP + lift)


def _fit(yaw=YAW_TYRE, dy=0.0):
    """Pick the wheel out of its stand by the tyre's top, carry it in its own plane over the fork, lower it into the
    dropouts and let go."""
    start_axle = L.WHEEL_START[2] - TOP
    carry, release = _grip_tcp(CARRY_AXLE), _grip_tcp(RELEASE_AXLE)
    return [Waypoint(2.0, (SX, SY, _grip_tcp(start_axle) + 0.05), GRIP_OPEN, "above the tyre", YAW_TYRE),
            Waypoint(1.2, (SX, SY, _grip_tcp(start_axle)), GRIP_OPEN, "fingers either side of the tyre", YAW_TYRE),
            Waypoint(0.8, (SX, SY, _grip_tcp(start_axle)), FINGER_CLOSED, "grip", YAW_TYRE),
            Waypoint(1.5, (SX, SY, carry), FINGER_CLOSED, "lift out of the stand", YAW_TYRE),
            Waypoint(2.5, (FX, FY + dy, carry), FINGER_CLOSED, "carry over the fork, axle above the dropouts", yaw),
            Waypoint(0.5, (FX, FY + dy, carry), FINGER_CLOSED, "settle", yaw),
            Waypoint(1.8, (FX, FY + dy, release), FINGER_CLOSED, "lower the axle into the dropouts", yaw),
            Waypoint(0.6, (FX, FY + dy, release), GRIP_OPEN, "let go: it drops onto the slot ends", yaw),
            Waypoint(1.0, (FX, FY + dy, carry + 0.012), GRIP_OPEN, "up", yaw)]


def _close(pivot, upto=68.0, label="press the lever closed"):
    """Press the open lever down and round its arc to `upto` deg (past 60 it snaps closed), then back away."""
    steps = [Waypoint(1.5, _lever_tcp(pivot, 0.0, press=0.0, lift=0.012), FINGER_CLOSED, "above the lever's paddle"),
             Waypoint(0.8, _lever_tcp(pivot, 0.0), FINGER_CLOSED, "on the paddle")]
    for c in [c for c in (20.0, 40.0, 55.0) if c < upto] + [upto]:
        steps.append(Waypoint(0.8, _lever_tcp(pivot, c), FINGER_CLOSED, f"{label}: {c:.0f} deg"))
    x, y, z = steps[-1].pos
    steps += [Waypoint(0.8, (x - 0.02, y, z + 0.01), FINGER_CLOSED, "back off"),
              Waypoint(1.0, (x - 0.02, y, z + 0.06), FINGER_CLOSED, "up")]
    return steps


def demo():
    v = os.environ.get("QR_VARIANT", "demo")
    if v not in VARIANTS:
        raise ValueError(f"QR_VARIANT must be one of {sorted(VARIANTS)}, got {v!r}")
    left = [Waypoint(2.0, PARK_L, FINGER_CLOSED, "park")]
    park = [Waypoint(2.0, PARK_R, FINGER_CLOSED, "park clear of the camera")]
    seated_pivot = (FX + L.PIVOT_X, FY, TOP + L.H_SEAT)
    if v == "nothing":
        return {"left": left, "right": park}
    cocked = _fit(yaw=YAW_TYRE - 0.30, dy=-0.020)
    if v == "one_side":
        return {"left": left, "right": cocked + park}
    right = _fit()
    if v == "open":
        return {"left": left, "right": right + park}
    if v == "half_open":
        return {"left": left, "right": right + _close(seated_pivot, upto=40.0, label="press part-way") + park}
    return {"left": left, "right": right + _close(seated_pivot) + park}
