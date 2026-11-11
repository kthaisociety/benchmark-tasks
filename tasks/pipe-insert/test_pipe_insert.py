"""Checks of pipe_insert.py's scoring, latch and demo plans, without the simulator. Needs yam_arena:

    python -m pytest -q test_pipe_insert.py
"""

import math
from pathlib import Path

import numpy as np
import pytest

from yam_arena.task_file import ObjectState, load

M = load(Path(__file__).with_name("pipe_insert.py")).module
RUN_LIMIT_S = 120.0


def st(x, y, z, tilt_deg=0.0):
    s = math.sin(math.radians(tilt_deg) / 2)
    z0 = np.zeros(3)
    return ObjectState(pos=np.array([x, y, z]), quat_xyzw=np.array([s, 0.0, 0.0, math.cos(math.radians(tilt_deg) / 2)]),
                       lin_vel=z0, ang_vel=z0, low=z0, high=z0)


def start():
    return {M.INSERT.name: st(*M.STAND_XY, M.TOP + M.STAND_H + M.SHANK_L),
            **{n: st(M.TRAY_X, e.pos[1], M.TOP + M.TRAY_T + M.SOCK_D) for n, e in M.ELBOWS.items()}}


def end_state(plan):
    """Where each part ends if every move in ``plan`` lands exactly; the latch is stepped after each move."""
    M._reset()
    s, stacked = start(), {"copper": 0.0, "pex": 0.0}
    for _, name, pipe in plan:
        x, y = M.PIPES[pipe]
        if name == "insert":
            s[name] = st(x, y, M.RIM)
            stacked[pipe] = M.COLLAR_H
        else:
            s[name] = st(x, y, M.RIM + stacked[pipe])
        M._update(s)
    return s


VARIANTS = {  # the same plans run_checks.sh runs in the simulator
    "demo": (M.DEMO_PLAN, 1.0),
    "idle": ((), 0.0),
    "no_insert": ((("right", "elbow_right", "pex"), ("left", "elbow_left", "copper")), 0.5),
    "insert_in_copper": ((("right", "insert", "copper"), ("right", "elbow_right", "pex"),
                          ("left", "elbow_left", "copper")), 0.0),
}


@pytest.mark.parametrize("name", VARIANTS)
def test_variant_scores(name):
    plan, expected = VARIANTS[name]
    assert M.score(end_state(plan)) == expected


@pytest.mark.parametrize("name", VARIANTS)
def test_variant_fits_the_run_limit(name):
    M.DEMO_PLAN = VARIANTS[name][0]
    try:
        d = M.demo()
        assert max(sum(w.duration for w in d[a]) for a in d) + 1.0 < RUN_LIMIT_S
        assert abs(sum(w.duration for w in d["left"]) - sum(w.duration for w in d["right"])) < 0.1
    finally:
        M.DEMO_PLAN = VARIANTS["demo"][0]


def test_elbow_on_plastic_first_is_latched_even_if_fixed_later():
    s = end_state((("right", "elbow_right", "pex"),))
    assert M._S["pex_without_insert"]
    s["elbow_right"] = st(M.TRAY_X, -0.10, M.TOP + M.TRAY_T + M.SOCK_D)  # elbow taken off again
    s["insert"] = st(*M.PIPES["pex"], M.RIM)
    M._update(s)
    s["elbow_right"] = st(*M.PIPES["pex"], M.RIM + M.COLLAR_H)
    M._update(s)
    assert M.score(s) == 0.0


@pytest.mark.parametrize(("off_mm", "ok"), [(2.5, True), (3.5, False)])
def test_on_axis_within_3_mm(off_mm, ok):
    s = end_state(M.DEMO_PLAN)
    x, y = M.PIPES["copper"]
    s["elbow_left"] = st(x + off_mm / 1000, y, M.RIM)
    assert M.score(s) == (1.0 if ok else 0.5)


def test_elbow_resting_on_the_insert_collar_not_home_on_copper():
    s = end_state(M.DEMO_PLAN)
    s["elbow_left"] = st(*M.PIPES["copper"], M.RIM + 0.01)  # stuck 1 cm short
    assert M.score(s) == 0.5


def test_tilted_elbow_does_not_count():
    s = end_state(M.DEMO_PLAN)
    s["elbow_right"] = st(*M.PIPES["pex"], M.RIM + M.COLLAR_H, tilt_deg=7)
    assert M.score(s) == 0.5
