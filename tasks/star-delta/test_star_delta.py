"""Checks of star_delta.py's scoring and demo plan, without the simulator. Needs yam_arena (the official stack):

    python -m pytest -q test_star_delta.py
"""

import math
from pathlib import Path

import numpy as np
import pytest

from yam_arena.task_file import ObjectState, load

M = load(Path(__file__).with_name("star_delta.py")).module
RUN_LIMIT_S = 120.0


def link(x, y, bottom, yaw=0.0, tilt_deg=0.0):
    """A link state: centre x, y, plate bottom z, turned ``yaw`` about vertical, then tilted about its own y."""
    sz, cz = math.sin(yaw / 2), math.cos(yaw / 2)
    sp, cp = math.sin(math.radians(tilt_deg) / 2), math.cos(math.radians(tilt_deg) / 2)
    z = np.zeros(3)
    return ObjectState(pos=np.array([x, y, bottom + M.LINK_T / 2]), quat_xyzw=np.array([-sz * sp, cz * sp, cp * sz, cz * cp]),
                       lin_vel=z, ang_vel=z, low=z, high=z)


def end_state(plan):
    """Where each link ends up if every move in ``plan`` lands exactly; a link set on a stud already used stacks 4 mm
    up at that end and tilts by the plate's thickness over the pitch."""
    at = {n: link(*M._mid(*pair), M.FLOOR) for n, pair in M.DELTA.items()}
    used, parked = set(), 0
    for n, target in plan:
        if target == "tray":
            at[n] = link(M.TX, M.TY + M.TRAY_SLOTS[parked], M.TRAY_FLOOR)
            parked += 1
        elif target == "table":
            at[n] = link(*M.TABLE_XY, M.TOP)
        else:
            a, b = target.split("-")
            stack = bool(used & {a, b})
            at[n] = link(*M._mid(a, b), M.FLOOR + (M.LINK_T / 2 if stack else 0.0),
                         yaw=math.pi / 2 if a[1] == b[1] else 0.0,
                         tilt_deg=math.degrees(math.atan2(M.LINK_T, M.PITCH)) if stack else 0.0)
            used |= {a, b}
    return {M.LINKS[n].name: s for n, s in at.items()}


VARIANTS = {  # the same plans run_checks.sh runs in the simulator
    "demo": (M.DEMO_PLAN, 1.0),
    "idle": ((), 0.0),
    "park_all": ((("w1v2", "tray"), ("v1u2", "tray"), ("u1w2", "tray")), 0.3),
    "one_bridge": ((("w1v2", "tray"), ("v1u2", "U2-V2"), ("u1w2", "tray")), 0.6),
    "star_plus_delta": ((("w1v2", "U2-V2"),), 0.0),
    "star_on_line_row": ((("w1v2", "tray"), ("v1u2", "V1-W1"), ("u1w2", "U1-V1")), 0.0),
    "link_on_table": ((("w1v2", "table"), ("v1u2", "U2-V2"), ("u1w2", "W2-U2")), 0.0),
}


@pytest.mark.parametrize("name", VARIANTS)
def test_variant_scores(name):
    plan, expected = VARIANTS[name]
    assert M.score(end_state(plan)) == expected
    assert M.success(end_state(plan)) == (expected == 1.0)


@pytest.mark.parametrize("name", VARIANTS)
def test_variant_fits_the_run_limit(name):
    M.DEMO_PLAN = VARIANTS[name][0]
    try:
        assert sum(w.duration for w in M.demo()["right"]) + 1.0 < RUN_LIMIT_S
    finally:
        M.DEMO_PLAN = VARIANTS["demo"][0]


def test_start_is_delta_and_seated_on_the_columns():
    s = end_state(())
    for n, pair in M.DELTA.items():
        assert M._seated(s[M.LINKS[n].name], pair)


@pytest.mark.parametrize(("off_mm", "seated"), [(3.5, True), (4.5, False)])
def test_seated_within_4_mm(off_mm, seated):
    a, b = M.STAR[0]
    mx, my = M._mid(a, b)
    assert M._seated(link(mx + off_mm / 1000, my, M.FLOOR, yaw=math.pi / 2), (a, b)) == seated


def test_either_end_round_and_a_few_degrees_of_yaw():
    a, b = M.STAR[1]
    mx, my = M._mid(a, b)
    assert M._seated(link(mx, my, M.FLOOR, yaw=-math.pi / 2 + math.radians(4)), (a, b))


def test_resting_on_the_stud_tops_is_not_seated():
    a, b = M.STAR[0]
    assert not M._seated(link(*M._mid(a, b), M.FLOOR + M.STUD_H, yaw=math.pi / 2), (a, b))


def test_too_tilted_is_not_seated():
    a, b = M.STAR[0]
    assert not M._seated(link(*M._mid(a, b), M.FLOOR, yaw=math.pi / 2, tilt_deg=10), (a, b))


def test_loose_on_the_board_scores_zero():
    s = end_state(VARIANTS["one_bridge"][0])
    s[M.LINKS["u1w2"].name] = link(M.BX - 0.06, M.BY, M.FLOOR)  # put down on the board, near the robot, on no stud
    assert M.score(s) == 0.0


def test_held_above_the_tray_scores_zero():
    s = end_state(M.DEMO_PLAN)
    s[M.LINKS["w1v2"].name] = link(M.TX, M.TY, M.TRAY_FLOOR + 0.05)
    assert M.score(s) == 0.0


def test_demo_turns_only_the_star_links():
    yaws = {}
    for w in M.demo()["right"]:
        if w.label == "set down":
            yaws[len(yaws)] = round(math.degrees(w.yaw))
    assert list(yaws.values()) == [90, 0, 0]  # the gripper closes along the link: tray along x, star along y
