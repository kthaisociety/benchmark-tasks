"""Checks of scaffold_baseplates.py's ground model and scoring, without the simulator. Needs yam_arena:

    python -m pytest -q test_scaffold_baseplates.py
"""

import math
from pathlib import Path

import numpy as np
import pytest

from yam_arena.task_file import ObjectState, load

M = load(Path(__file__).with_name("scaffold_baseplates.py")).module
B, P, T = M.BOARD.name, {s: p.name for s, p in M.PLATES.items()}, {s: t.name for s, t in M.TUBES.items()}


def st(x, y, z, yaw=0.0, tilt=0.0, v=0.0):
    sz, cz, sx, cx = math.sin(yaw / 2), math.cos(yaw / 2), math.sin(math.radians(tilt) / 2), math.cos(math.radians(tilt) / 2)
    zero = np.zeros(3)
    return ObjectState(pos=np.array([x, y, z]), quat_xyzw=np.array([sx * cz, -sx * sz, cx * sz, cx * cz]),
                       lin_vel=np.array([v, 0, 0]), ang_vel=zero, low=zero, high=zero)


def scene(board="start", plates="start", tubes="start"):
    """Every part at one of: start, or its finished place (board on the lawn, plates on the board / on the grass,
    tubes in the plates / on the grass)."""
    s = {B: st(*M.BOARD_START, M.TOP + M.BOARD_T / 2) if board == "start" else st(M.LX, 0, M.LAWN + M.BOARD_T / 2)}
    pz = {"start": None, "board": M.LAWN + M.BOARD_T, "grass": M.LAWN}[plates]
    for side in M.PLATES:
        xy = M.PLATE_START[side] if plates == "start" else M.MARKS[side]
        s[P[side]] = st(*xy, (pz or M.TOP) + M.PLATE_T / 2)
        if tubes == "start":
            s[T[side]] = st(*M.TUBE_START[side], M.TOP + M.TUBE_H / 2)
        else:
            tz = (pz + M.PLATE_T) if tubes == "plates" else M.LAWN
            s[T[side]] = st(*M.MARKS[side], tz + M.TUBE_H / 2)
    return s


def run(s, steps=20):
    M._reset()
    for _ in range(steps):
        M._ground(s)
    return M.score(s)


@pytest.mark.parametrize(("parts", "expected"), [
    (dict(), 0.0),
    (dict(board="lawn"), 0.2),
    (dict(board="lawn", plates="board"), 0.5),
    (dict(board="lawn", plates="board", tubes="plates"), 1.0),
    (dict(tubes="grass"), 0.0),
    (dict(plates="grass", tubes="plates"), 0.0),
    (dict(board="lawn", tubes="grass"), 0.2),
])
def test_scores(parts, expected):
    assert run(scene(**parts)) == pytest.approx(expected)


def test_things_on_grass_sink_and_stay_sunk():
    s = scene(plates="grass", tubes="plates")
    run(s)
    assert set(M._S["sunk"]) == set(P.values())
    s[P["left"]] = st(*M.MARKS["left"], M.LAWN + M.BOARD_T + M.PLATE_T / 2)  # lifted onto a board later: still sunk
    M._ground(s)
    assert P["left"] in M._S["sunk"]


def test_moving_over_the_grass_does_not_sink():
    s = scene()
    s[T["left"]] = st(*M.MARKS["left"], M.LAWN + M.TUBE_H / 2, v=0.1)  # passing low but moving
    run(s)
    assert not M._S["sunk"]


def test_tube_off_axis_or_leaning_does_not_count():
    s = scene(board="lawn", plates="board", tubes="plates")
    x, y = M.MARKS["left"]
    s[T["left"]] = st(x + 0.012, y, M.LAWN + M.BOARD_T + M.PLATE_T + M.TUBE_H / 2)
    assert run(s) == pytest.approx(0.75)
    s[T["left"]] = st(x, y, M.LAWN + M.BOARD_T + M.PLATE_T + M.TUBE_H / 2, tilt=7)
    assert run(s) == pytest.approx(0.75)


def test_demo_fits_the_run_limit():
    for arm in M.demo().values():
        assert sum(w.duration for w in arm) + 1.0 < 120.0
