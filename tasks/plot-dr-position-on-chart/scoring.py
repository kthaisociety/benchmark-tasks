"""Where the position marker ends up on the chart, and what that is worth. Pure Python (numpy only), so it can be
tested without the simulator.

Frame: metres from the midpoint between the arm bases, x forward, y toward the left arm, z up; TOP is the table top.

The navigator's rule scored: distance on a Mercator chart is measured on the latitude scale (the side borders), level
with the leg, because one minute of latitude is one nautical mile. A minute of longitude (top and bottom borders) is
shorter by cos(latitude), about two thirds of a mile here, so six minutes taken off it put the boat 2.1 nm short.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

import chart as C

# -- the position marker (a weighted plotting marker; its origin is the centre of its disc) ---------------------------
DISC_R, DISC_H = 0.0125, 0.006
KNOB_R, KNOB_H = 0.006, 0.016
MARKER_HALF_H = DISC_H / 2

FULL_NM = 0.2  # 1.0 within 0.2 nm (7.4 mm on the chart) of the DR position
HALF_NM = 0.5  # 0.5 within 0.5 nm (18.5 mm)
MAX_TILT_DEG = 10.0
LIFT_M = 0.003  # lying on the chart: the disc's base within 3 mm of the paper
SETTLE_STEPS = 20  # let go and left alone: moved at most SETTLE_M over the last 20 control steps (1 s at 20 Hz)
SETTLE_M = 0.002
CLEAR_M = 0.05  # let go: no point of either gripper, wrist mount to pad tips, within 5 cm of the marker
GRIPPER_BACK = 0.134  # the gripper reaches this far behind the TCP along its approach (wrist mount)
PAD_TIP = 0.013  # and this far past it (the pad tips)


def paper_top(top: float) -> float:
    return top + C.PAPER_THICK


def rotation(quat_xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def tilt_deg(quat_xyzw) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, rotation(quat_xyzw)[2, 2]))))


def mark_point(pos, quat_xyzw) -> np.ndarray:
    """The centre of the marker's base: the position it marks on the chart."""
    return np.asarray(pos, float) + rotation(quat_xyzw) @ np.array([0.0, 0.0, -MARKER_HALF_H])


def chart_nm(x: float, y: float, to=C.DR_XY) -> float:
    """Distance on the chart from (x, y) to a point, in nautical miles off the latitude scale at that latitude."""
    lat = C.to_chart(*to)[0]
    return math.dist((x, y), to) / C.lat_minute(lat)


def on_chart(x: float, y: float) -> bool:
    """Inside the charted area (the inner neatline)."""
    return C.INNER_X[0] <= x <= C.INNER_X[1] and C.INNER_Y[0] <= y <= C.INNER_Y[1]


@dataclass(frozen=True)
class Reading:
    """What a navigator reads off the final state: where the marker marks, in nautical miles from the DR position."""

    lying: bool  # flat on the paper, inside the charted area
    off_nm: float


def read(pos, quat_xyzw, top: float) -> Reading:
    m = mark_point(pos, quat_xyzw)
    flat = tilt_deg(quat_xyzw) <= MAX_TILT_DEG and abs(m[2] - paper_top(top)) <= LIFT_M
    return Reading(lying=flat and on_chart(m[0], m[1]), off_nm=chart_nm(m[0], m[1]))


def points(r: Reading) -> float:
    """1.0 within 0.2 nm of the DR position; 0.5 within 0.5 nm; 0 farther (the longitude-scale answer is 2.1 nm
    short), off the charted area, or not lying flat on the chart."""
    if not r.lying:
        return 0.0
    if r.off_nm <= FULL_NM:
        return 1.0
    if r.off_nm <= HALF_NM:
        return 0.5
    return 0.0


def _marker_distance(point, pos, quat_xyzw) -> float:
    """Distance from a point to the marker (disc and knob as one upright cylinder, 0 inside)."""
    local = rotation(quat_xyzw).T @ (np.asarray(point, float) - np.asarray(pos, float))
    lo, hi = -MARKER_HALF_H, MARKER_HALF_H + KNOB_H
    radial = max(math.hypot(local[0], local[1]) - DISC_R, 0.0)
    vertical = max(lo - local[2], local[2] - hi, 0.0)
    return math.hypot(radial, vertical)


def gripper_clear(tcp, approach, pos, quat_xyzw, samples: int = 40) -> bool:
    """No point on the gripper's centre line, from its wrist mount to its pad tips, within CLEAR_M of the marker."""
    tcp, approach = np.asarray(tcp, float), np.asarray(approach, float)
    return all(_marker_distance(tcp + s * approach, pos, quat_xyzw) >= CLEAR_M
               for s in np.linspace(-GRIPPER_BACK, PAD_TIP, samples))


def settled(history) -> bool:
    """The last SETTLE_STEPS + 1 marked points (oldest first) all lie within SETTLE_M of the latest."""
    if len(history) < SETTLE_STEPS + 1:
        return False
    recent = np.asarray(list(history)[-(SETTLE_STEPS + 1):], float)
    return float(np.max(np.linalg.norm(recent - recent[-1], axis=1))) <= SETTLE_M


def final(r: Reading, grippers_clear: bool | None, history) -> float:
    """The trial's score: the mark's points, only once the marker is let go (both grippers clear; None, if the arms
    can't be read, counts as not let go) and has stayed put for the last second."""
    return points(r) if grippers_clear and settled(history) else 0.0
