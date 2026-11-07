"""Scoring for the buried-pipe trainer. NumPy only, so it runs and is tested without Isaac Sim.

A strike is any point of the spade within STRIKE_GAP of the pipe's surface (in the real trainer the pipe is
rubbed with chalk and any scuff on the spade or the pipe is a strike). Exposure is the length of the pipe's
top (a 10 mm strip along its crown), inside the marked stretch, that no soil ball lies over, as seen straight down.
"""

from dataclasses import dataclass, field

import numpy as np

import task

STRIKE_GAP = 0.001  # m from the pipe's surface
SAMPLE = 0.002  # spacing of the sample points on the spade's faces (m)
SEGMENT = 0.005  # the marked stretch is judged in 5 mm segments
TIERS = ((0.080, 1.0), (0.040, 0.5), (0.010, 0.25))  # bare length (m) -> score
HOLD_S = 1.0
STILL_SPEED = 0.02  # m/s, for every ball over the pipe
SUBSTEPS = 4  # spade poses checked between control steps
CROWN_HALF = 0.005  # the pipe's top is the 10 mm wide strip along its crown


def _face_points(center, size, spacing=SAMPLE):
    c, h = np.asarray(center, float), 0.5 * np.asarray(size, float)
    axes = [np.linspace(-h[i], h[i], max(2, int(np.ceil(2 * h[i] / spacing)) + 1)) for i in range(3)]
    points = []
    for i in range(3):
        j, k = [a for a in range(3) if a != i]
        uu, vv = np.meshgrid(axes[j], axes[k], indexing="ij")
        for sign in (-1.0, 1.0):
            p = np.zeros((uu.size, 3))
            p[:, i], p[:, j], p[:, k] = sign * h[i], uu.ravel(), vv.ravel()
            points.append(p + c)
    return np.unique(np.round(np.concatenate(points), 6), axis=0)


SPADE_POINTS = np.concatenate([_face_points(p.center, p.size) for p in task.SPADE_PARTS if p.collides])
SEGMENTS_X = np.arange(task.MARKS_X[0] + SEGMENT / 2, task.MARKS_X[1], SEGMENT)


def quat_xyzw_to_matrix(q):
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def slerp(q0, q1, s):
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    d = float(np.dot(q0, q1))
    if d < 0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + s * (q1 - q0)
    else:
        a = np.arccos(d)
        q = (np.sin((1 - s) * a) * q0 + np.sin(s * a) * q1) / np.sin(a)
    return q / np.linalg.norm(q)


def pipe_gap(points):
    """Distance (m) from each world point to the pipe's surface (inf beyond its ends)."""
    p = np.asarray(points, float)
    radial = np.hypot(p[:, 1] - task.PIPE_AXIS[0], p[:, 2] - task.PIPE_AXIS[1]) - task.PIPE_R
    inside = (p[:, 0] >= task.PIPE_X[0]) & (p[:, 0] <= task.PIPE_X[1])
    return np.where(inside, radial, np.inf)


def spade_gap(pos, quat_xyzw):
    """Closest approach (m) of the spade's faces to the pipe's surface at one pose."""
    world = SPADE_POINTS @ quat_xyzw_to_matrix(quat_xyzw).T + np.asarray(pos, float)
    return float(pipe_gap(world).min())


def swept_gap(prev, now):
    """Closest approach over SUBSTEPS poses interpolated from ``prev`` to ``now`` (each (pos, quat_xyzw))."""
    if prev is None:
        return spade_gap(*now)
    gaps = []
    for s in np.linspace(1.0 / SUBSTEPS, 1.0, SUBSTEPS):
        pos = (1 - s) * np.asarray(prev[0], float) + s * np.asarray(now[0], float)
        gaps.append(spade_gap(pos, slerp(prev[1], now[1], s)))
    return min(gaps)


def bare_length(balls):
    """Metres of the pipe's top strip, within the marks, with no ball over it."""
    b = np.asarray(balls, float).reshape(-1, 3)
    above = b[b[:, 2] > task.CROWN_Z]
    if len(above) == 0:
        return SEGMENT * len(SEGMENTS_X)
    dx = np.abs(above[None, :, 0] - SEGMENTS_X[:, None])
    dy = np.abs(above[None, :, 1] - task.PIPE_AXIS[0])
    covered = ((dx < task.SOIL_R) & (dy < task.SOIL_R + CROWN_HALF)).any(axis=1)
    return SEGMENT * int((~covered).sum())


def over_pipe(balls):
    b = np.asarray(balls, float).reshape(-1, 3)
    return (
        (b[:, 0] > task.BOX_X[0]) & (b[:, 0] < task.BOX_X[1])
        & (b[:, 1] > task.BOX_Y[0]) & (b[:, 1] < task.BOX_Y[1])
    )


def tier(length):
    for need, value in TIERS:
        if length >= need - 1e-9:
            return value
    return 0.0


@dataclass
class Judge:
    """Per-trial state: a strike latches 0; each tier counts once it has held, still, for HOLD_S."""

    control_hz: float = 20.0
    struck: bool = False
    closest: float = np.inf
    best: float = 0.0
    held: dict = field(default_factory=dict)
    last_pose: tuple | None = None

    def step(self, spade_pose, balls, ball_speeds):
        gap = swept_gap(self.last_pose, spade_pose)
        self.last_pose = (np.asarray(spade_pose[0], float), np.asarray(spade_pose[1], float))
        self.closest = min(self.closest, gap)
        if gap < STRIKE_GAP:
            self.struck = True
        length = bare_length(balls)
        speeds = np.asarray(ball_speeds, float).reshape(-1)
        mask = over_pipe(balls)
        still = bool((speeds[mask] < STILL_SPEED).all()) if mask.any() else True
        need = max(1, round(HOLD_S * self.control_hz))
        for _, value in TIERS:
            ok = still and tier(length) >= value
            self.held[value] = self.held.get(value, 0) + 1 if ok else 0
            if self.held[value] >= need:
                self.best = max(self.best, value)
        score = 0.0 if self.struck else self.best
        return {
            "score": score,
            "success": score == 1.0,
            "struck": self.struck,
            "spade_gap_m": gap,
            "closest_gap_m": self.closest,
            "bare_m": length,
            "still": still,
        }
