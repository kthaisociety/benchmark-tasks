"""Checks of wheelchair_cushion.py's scoring and demo plans, without the simulator. Needs yam_arena:

    python -m pytest -q test_wheelchair_cushion.py
"""

import math
from pathlib import Path

import numpy as np
import pytest

from yam_arena.task_file import ObjectState, load

M = load(Path(__file__).with_name("wheelchair_cushion.py")).module
NAME = M.CUSHION.name


def cushion(x, y, bottom, yaw_deg=0.0, tilt_deg=0.0, moving=False):
    """Turned yaw about vertical, then tilted about its own y."""
    sz, cz = math.sin(math.radians(yaw_deg) / 2), math.cos(math.radians(yaw_deg) / 2)
    sp, cp = math.sin(math.radians(tilt_deg) / 2), math.cos(math.radians(tilt_deg) / 2)
    z = np.zeros(3)
    return {NAME: ObjectState(pos=np.array([x, y, bottom + M.BASE[2] / 2]),
                              quat_xyzw=np.array([-sz * sp, cz * sp, cp * sz, cz * cp]),
                              lin_vel=np.array([0.1, 0, 0]) if moving else z, ang_vel=z, low=z, high=z)}


SEATED = (M.SX, M.SY, M.SEAT_TOP)


@pytest.mark.parametrize(("state", "expected"), [
    (cushion(*M.START, M.TOP, 180), 0.0),             # the start, backwards
    (cushion(*SEATED, 0), 1.0),                       # seated, well to the backrest
    (cushion(*SEATED, 15), 1.0),                      # 15 deg off still counts
    (cushion(*SEATED, 180), 0.0),                     # seated backwards
    (cushion(*SEATED, -90), 0.0),                     # seated sideways
    (cushion(*M.START, M.TOP, 0), 0.25),              # turned round, left on the table
    (cushion(*M.START, M.TOP, -90), 0.0),             # one quarter turn only
    (cushion(M.SX, M.SY, M.TOP + M.GUARD_H, 0), 0.5),  # resting on the guards, not down
    (cushion(*SEATED, 0, tilt_deg=8), 0.5),           # tilted in the seat
    (cushion(M.SX + 0.06, M.SY, M.SEAT_TOP, 0), 0.5),  # pushed 6 cm toward the backrest
    (cushion(*SEATED, 0, moving=True), 0.0),          # still moving (held or falling)
])
def test_scores(state, expected):
    assert M.score(state) == expected


@pytest.mark.parametrize("plan", [("turn", "turn", "seat"), (), ("seat",), ("turn", "seat")])
def test_plan_fits_the_run_limit(plan):
    M.DEMO_PLAN = plan
    try:
        assert sum(w.duration for w in M.demo()["right"]) + 1.0 < 120.0
    finally:
        M.DEMO_PLAN = ("turn", "turn", "seat")


def test_two_quarter_turns_make_a_half_turn():
    turns = [w.yaw for w in M.demo()["right"] if w.label == "quarter turn"]
    assert len(turns) == 2 and math.degrees(math.pi + sum(turns)) % 360 == pytest.approx(0)
