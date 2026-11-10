"""Scoring for the copper-to-galvanised pipe joints. Pure Python (NumPy only), so it can be tested without the simulator.

The pipe run lies along the world y axis (left = +y). Joint A, on the left, joins the existing galvanised steel pipe
(left of it) to the new copper section (right of it): only the dielectric union belongs there, with its brass end toward
the copper. Joint B, on the right, joins the new copper section to the existing copper pipe: only the copper coupling
belongs there. Every fitting is modelled with its axis along its own local y; the union's brass (sweat) end is at its
local +y.

Score, judged on the last step (or at success):
- 0 if the pipework was ever moved 1 cm or turned 2 degrees (latched for the trial);
- 0 if any fitting that does not belong is in either joint at the end (the steel coupling in either joint, the copper
  coupling in joint A, the union in joint B, or the union in joint A with its brass end turned within 60 degrees of the
  steel pipe);
- otherwise 0.75 if the union is seated in joint A with its brass end toward the copper, plus 0.25 if the copper
  coupling is seated in joint B. A fitting only counts as seated once it is let go: no part of either gripper (wrist to
  pad tips) within 5 cm of its surface, and its centre moved at most 2 mm over the last second.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# -- geometry shared with task.py (world frame, metres; z relative to the table top) ---------------------------------
PIPE_X = 0.38                 # the pipe run's axis, forward of the arm bases
PLATE_TOP = 0.030             # top of the saddle plates under the gaps, above the table top
GAP_Y = {"A": 0.15, "B": -0.15}
GAP_LEN = 0.12                # between the pipe ends
FIT_LEN = 0.09                # every fitting's length
FIT_R = 0.025                 # fitting body radius
NUT_R = 0.027                 # the union's nut, its widest part
RAIL_IN, RAIL_H = 0.016, 0.008  # saddle rails: inner edges 16 mm either side of the axis, 8 mm tall


def seat_height(radius: float) -> float:
    """Height of a fitting's axis above the plate top when its widest part rests on the two rails' inner edges."""
    return RAIL_H + math.sqrt(radius * radius - RAIL_IN * RAIL_IN)


AXIS_Z = PLATE_TOP + seat_height(FIT_R)   # the pipe axis (a coupling's axis when seated), above the table top
SEAT_Z = {"union": PLATE_TOP + seat_height(NUT_R), "cu_coupling": AXIS_Z, "steel_coupling": AXIS_Z}

# -- tolerances ---------------------------------------------------------------------------------------------------------
SEAT_DX = 0.010               # centre within 10 mm of the pipe line, front to back
SEAT_DY = 0.015               # centre within 15 mm of the gap's centre along the pipe (1.5 cm of play at each end)
SEAT_DZ = 0.006               # axis height within 6 mm of the seated height (resting in the saddle, not on a pipe end)
SEAT_AXIS_DEG = 15.0          # axis within 15 degrees of the pipe line
IN_DX, IN_DY, IN_ABOVE = 0.05, 0.07, 0.04   # "in the joint": centre within 5 cm / 7 cm, at most 4 cm above the axis
KNOCK_M, KNOCK_DEG = 0.010, 2.0
CLEAR = 0.05                  # no gripper point within 5 cm of a scored fitting's surface
SETTLE_M, SETTLE_STEPS = 0.002, 20   # moved at most 2 mm over the last 20 control steps (1 s at 20 Hz)
GRIPPER_BACK, PAD_TIP = 0.10, 0.013  # gripper points from 10 cm behind the TCP to the pad tips, along its approach
MAX_SPEED, MAX_SPIN = 0.01, 0.5
REVERSED_COS = 0.5            # brass end within 60 degrees of the steel side: the union is in backwards

FITTINGS = ("union", "cu_coupling", "steel_coupling")
WEIGHT = {"A": 0.75, "B": 0.25}
BELONGS = {"A": "union", "B": "cu_coupling"}


@dataclass(frozen=True)
class Pose:
    """One body's state: position (world, m, z above the table top), orientation (x, y, z, w) and velocities."""

    pos: tuple
    quat_xyzw: tuple = (0.0, 0.0, 0.0, 1.0)
    lin_vel: tuple = (0.0, 0.0, 0.0)
    ang_vel: tuple = (0.0, 0.0, 0.0)

    def rotation(self) -> np.ndarray:
        x, y, z, w = (float(v) for v in self.quat_xyzw)
        return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                         [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                         [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])

    def axis(self) -> np.ndarray:
        """The fitting's axis (its local +y; the union's brass end) in the world frame."""
        return self.rotation()[:, 1]

    def still(self) -> bool:
        return bool(np.linalg.norm(self.lin_vel) < MAX_SPEED and np.linalg.norm(self.ang_vel) < MAX_SPIN)


def surface_distance(p: Pose, point) -> float:
    """Distance from a point to the fitting, taken as a cylinder of the union nut's radius and the fitting's length."""
    local = p.rotation().T @ (np.asarray(point, float) - np.asarray(p.pos, float))
    radial = math.hypot(float(local[0]), float(local[2])) - NUT_R
    axial = abs(float(local[1])) - FIT_LEN / 2
    if radial <= 0 and axial <= 0:
        return max(radial, axial)
    return math.hypot(max(radial, 0.0), max(axial, 0.0))


def gripper_points(tcp, approach, samples: int = 24) -> np.ndarray:
    """Points along one gripper, from its wrist (behind the TCP) to its pad tips."""
    s = np.linspace(-GRIPPER_BACK, PAD_TIP, samples)
    return np.asarray(tcp, float) + s[:, None] * np.asarray(approach, float)


def let_go(p: Pose, grippers) -> bool:
    """No gripper point within CLEAR of the fitting. ``grippers`` is a list of (tcp, approach) pairs (world, z above the
    table top), or None when the arms could not be read: then nothing counts as let go."""
    if grippers is None:
        return False
    return all(surface_distance(p, q) >= CLEAR for tcp, approach in grippers for q in gripper_points(tcp, approach))


def settled(history) -> bool:
    """The last SETTLE_STEPS centre positions stay within SETTLE_M of each other (fewer samples: not yet settled)."""
    if len(history) < SETTLE_STEPS:
        return False
    pts = np.asarray(list(history)[-SETTLE_STEPS:], float)
    return bool(np.linalg.norm(pts - pts[-1], axis=1).max() <= SETTLE_M)


def in_joint(p: Pose, gap: str) -> bool:
    """The fitting's centre is in or just over the gap: set down there, propped on a pipe end or lying across it."""
    x, y, z = (float(v) for v in p.pos)
    return abs(x - PIPE_X) <= IN_DX and abs(y - GAP_Y[gap]) <= IN_DY and z <= AXIS_Z + IN_ABOVE


def seated(p: Pose, name: str, gap: str) -> bool:
    """Resting in the gap's saddle, in line with the pipe, still."""
    x, y, z = (float(v) for v in p.pos)
    along = abs(float(p.axis()[1]))
    return (abs(x - PIPE_X) <= SEAT_DX and abs(y - GAP_Y[gap]) <= SEAT_DY and abs(z - SEAT_Z[name]) <= SEAT_DZ
            and along >= math.cos(math.radians(SEAT_AXIS_DEG)) and p.still())


def brass_toward_copper(p: Pose) -> bool:
    """In joint A the copper section is on the right (-y): the union's brass end must point that way."""
    return float(p.axis()[1]) < 0.0


def reversed_union(p: Pose) -> bool:
    """The union's brass end turned toward the steel pipe (+y), within 60 degrees of it. A union lying across the gap
    is merely not seated."""
    return float(p.axis()[1]) >= REVERSED_COS


def wrong_fittings(fittings: dict[str, Pose]) -> list[str]:
    """Every fitting in a joint it does not belong in, as 'name@gap'."""
    out = []
    for name, p in fittings.items():
        for gap in GAP_Y:
            if not in_joint(p, gap):
                continue
            if name != BELONGS[gap] or (name == "union" and reversed_union(p)):
                out.append(f"{name}@{gap}")
    return out


def knocked(pipework: Pose, start: Pose) -> bool:
    """The pipework moved 1 cm or turned 2 degrees from where it started."""
    moved = np.linalg.norm(np.asarray(pipework.pos, float) - np.asarray(start.pos, float))
    rel = start.rotation().T @ pipework.rotation()
    angle = math.degrees(math.acos(max(-1.0, min(1.0, (float(np.trace(rel)) - 1.0) / 2.0))))
    return bool(moved >= KNOCK_M or angle >= KNOCK_DEG)


def joints_done(fittings: dict[str, Pose], free: dict[str, bool]) -> dict[str, bool]:
    """``free[name]``: the fitting is let go and settled."""
    union = fittings["union"]
    return {"A": seated(union, "union", "A") and brass_toward_copper(union) and free["union"],
            "B": seated(fittings["cu_coupling"], "cu_coupling", "B") and free["cu_coupling"]}


def score(fittings: dict[str, Pose], was_knocked: bool, free: dict[str, bool]) -> float:
    if was_knocked or wrong_fittings(fittings):
        return 0.0
    done = joints_done(fittings, free)
    return round(sum(WEIGHT[g] for g, ok in done.items() if ok), 4)


def success(fittings: dict[str, Pose], was_knocked: bool, free: dict[str, bool]) -> bool:
    return score(fittings, was_knocked, free) == 1.0
