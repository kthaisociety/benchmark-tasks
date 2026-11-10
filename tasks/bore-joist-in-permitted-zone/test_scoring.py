"""Checks of the scoring with synthetic guide poses. The scoring tests need only numpy and pytest:

    python -m pytest -q test_scoring.py

The last two tests load task.py and need yam_arena (the official stack); elsewhere they are skipped.
"""

import math
import os
from pathlib import Path

import numpy as np
import pytest

import scoring as S

TOP = 0.7475  # any table height; the scorer is relative to it
FACE = S.joist_face_z(TOP)
RIGHT_ZONE_MID = -(S.END_Y - sum(S.ZONE) / 2)  # y = -0.14


def quat(tilt_deg=0.0, yaw_deg=0.0):
    """A spin about vertical times a tilt about the guide's own x axis (x, y, z, w)."""
    t, p = math.radians(tilt_deg) / 2, math.radians(yaw_deg) / 2
    sx, cx, sz, cz = math.sin(t), math.cos(t), math.sin(p), math.cos(p)
    return np.array([cz * sx, sz * sx, sz * cx, cz * cx])


def guide(bx, by, base_z=FACE, tilt_deg=0.0, yaw_deg=0.0, moving=False):
    """The guide with its bore point (base centre) at bx, by, base_z."""
    q = quat(tilt_deg, yaw_deg)
    pos = np.array([bx, by, base_z]) + S.rotation(q) @ np.array([0.0, 0.0, S.GUIDE_HALF_H])
    lin = np.array([0.05, 0, 0]) if moving else np.zeros(3)
    return S.read(pos, q, lin, np.zeros(3), TOP)


def pts(*a, **k):
    return S.points(guide(*a, **k))


def test_quaternion_helpers_agree():
    q = quat(4.0, 30.0)
    assert S.tilt_deg(q) == pytest.approx(4.0, abs=1e-9)
    assert np.allclose(S.rotation(q) @ S.rotation(q).T, np.eye(3))


def test_bore_point_is_the_base_centre():
    r = guide(S.JOIST_X, RIGHT_ZONE_MID)
    assert r.standing and r.off_centre_line == pytest.approx(0) and r.from_support == pytest.approx(0.26)


@pytest.mark.parametrize("yaw", [0, 17, 45, 90])
def test_right_zone_on_the_centre_line_is_full_score_any_way_round(yaw):
    assert pts(S.JOIST_X, RIGHT_ZONE_MID, yaw_deg=yaw) == 1.0


@pytest.mark.parametrize("from_end", [0.2005, 0.23, 0.29, 0.3195])
def test_anywhere_in_the_right_zone_counts(from_end):
    assert pts(S.JOIST_X, -(S.END_Y - from_end)) == 1.0


@pytest.mark.parametrize(("off_mm", "expected"), [(0, 1.0), (9.5, 1.0), (-9.5, 1.0), (10.5, 0.0), (15, 0.0),
                                                  (-15, 0.0), (30, 0.0)])
def test_distance_off_the_centre_line(off_mm, expected):
    assert pts(S.JOIST_X + off_mm / 1000, RIGHT_ZONE_MID) == expected


@pytest.mark.parametrize("from_end", [0.0, 0.07, 0.1995, 0.3205, 0.36, 0.40])
def test_outside_the_zone_scores_zero(from_end):
    # 0 to 70 mm: at the bearing; 360-400 mm: towards and at mid-span
    assert pts(S.JOIST_X, -(S.END_Y - from_end)) == 0.0


def test_the_chalk_cross_scores_zero():
    assert pts(S.JOIST_X - S.JOIST_DEPTH / 2 + 0.015, 0.0) == 0.0


def test_mid_span_on_the_centre_line_scores_zero():
    assert pts(S.JOIST_X, 0.0) == 0.0


def test_the_left_zone_is_ruled_out_by_the_existing_hole():
    # every centre-line spot in the left zone is within three diameters (66 mm) of the hole at 260 mm
    for from_end in np.linspace(S.ZONE[0], S.ZONE[1], 25):
        r = guide(S.JOIST_X, S.END_Y - from_end)
        assert r.in_zone and not r.spaced and S.points(r) == 0.0


def test_on_the_existing_hole_scores_zero():
    assert pts(*S.EXISTING_HOLE) == 0.0


def test_the_zones_mirror_except_for_the_hole():
    for from_end in (0.21, 0.26, 0.31):
        assert guide(S.JOIST_X, S.END_Y - from_end).in_zone == guide(S.JOIST_X, -(S.END_Y - from_end)).in_zone


def test_on_the_table_scores_zero():
    assert pts(S.JOIST_X, RIGHT_ZONE_MID, base_z=TOP) == 0.0  # e.g. beside the joist, or never moved
    assert pts(0.21, 0.0, base_z=TOP) == 0.0  # the starting spot


def test_held_above_the_face_scores_zero():
    assert pts(S.JOIST_X, RIGHT_ZONE_MID, base_z=FACE + 0.01) == 0.0


def test_on_a_chalk_mark_is_still_seated():
    assert guide(S.JOIST_X, RIGHT_ZONE_MID, base_z=FACE + 0.0004).standing


@pytest.mark.parametrize(("tilt", "expected"), [(3.0, 1.0), (8.0, 0.0), (90.0, 0.0)])
def test_tilted_or_on_its_side(tilt, expected):
    # a 3 degree tilt on a flat face is contact noise; 8 degrees means it rests on something
    r = guide(S.JOIST_X, RIGHT_ZONE_MID, tilt_deg=tilt)
    assert S.points(r) == expected


def test_off_the_end_of_the_joist_scores_zero():
    assert pts(S.JOIST_X, -(S.END_Y + 0.05)) == 0.0


def test_a_moving_guide_scores_but_is_not_yet_a_success():
    r = guide(S.JOIST_X, RIGHT_ZONE_MID, moving=True)
    assert S.points(r) == 1.0 and not r.still


# -- let go and left alone ------------------------------------------------------------------------------------------

GUIDE_POS = np.array([S.JOIST_X, RIGHT_ZONE_MID, FACE + S.GUIDE_HALF_H])
DOWN = np.array([0.0, 0.0, -1.0])  # a straight-down gripper's approach
STILL = [np.array([S.JOIST_X, RIGHT_ZONE_MID, FACE])] * (S.SETTLE_STEPS + 1)


def test_a_gripper_holding_the_guide_is_not_clear():
    tcp = GUIDE_POS + np.array([0, 0, 0.025 - S.GUIDE_HALF_H + S.PAD_TIP])  # pad tips 25 mm above the base
    assert not S.gripper_clear(tcp, DOWN, GUIDE_POS, quat())


def test_a_gripper_just_above_the_guide_is_not_clear():
    tcp = GUIDE_POS + np.array([0, 0, S.GUIDE_HALF_H + S.PAD_TIP + 0.02])  # pad tips 2 cm above its top
    assert not S.gripper_clear(tcp, DOWN, GUIDE_POS, quat())


def test_a_parked_or_raised_gripper_is_clear():
    assert S.gripper_clear((0.12, -0.42, TOP + 0.20), DOWN, GUIDE_POS, quat())
    tcp = GUIDE_POS + np.array([0, 0, S.GUIDE_HALF_H + S.PAD_TIP + 0.06])  # pad tips 6 cm above its top
    assert S.gripper_clear(tcp, DOWN, GUIDE_POS, quat())


def test_settled_needs_a_full_second_within_2_mm():
    assert S.settled(STILL)
    assert not S.settled(STILL[:-1])  # not a full second yet
    nudged = [p + np.array([0.003, 0, 0]) for p in STILL[:5]] + STILL[5:]
    assert not S.settled(nudged)
    crept = [p + np.array([0.0015, 0, 0]) for p in STILL[:5]] + STILL[5:]
    assert S.settled(crept)


@pytest.mark.parametrize(("clear", "history", "expected"), [
    (True, STILL, 1.0),
    (False, STILL, 0.0),  # still held
    (None, STILL, 0.0),  # the arms couldn't be read
    (True, STILL[:5], 0.0),  # just set down
])
def test_full_score_needs_let_go_and_left_alone(clear, history, expected):
    assert S.final(guide(S.JOIST_X, RIGHT_ZONE_MID), clear, history) == expected


def test_let_go_in_a_wrong_spot_is_still_zero():
    assert S.final(guide(S.JOIST_X, 0.0), True, STILL) == 0.0


def test_the_largest_allowed_hole_is_bigger_than_the_one_bored():
    assert S.HOLE_D <= 0.25 * S.JOIST_DEPTH


# -- these need yam_arena -----------------------------------------------------------------------------------------

def _load_task():
    pytest.importorskip("yam_arena")
    from yam_arena.task_file import load
    return load(Path(__file__).with_name("task.py")).module


@pytest.mark.parametrize("variant", ["demo", "offcentre", "edge", "cross", "midspan", "bearing", "on_hole",
                                     "near_hole"])
def test_each_variant_target_scores_as_expected(variant):
    m = _load_task()
    _, x, y, expected = m.VARIANTS[variant]
    assert pts(x, y) == expected


@pytest.mark.parametrize("variant", ["demo", "offcentre", "edge", "cross", "midspan", "bearing", "on_hole",
                                     "near_hole", "held", "nothing"])
def test_the_demo_is_within_reach(variant, monkeypatch):
    pytest.importorskip("yam_arena")
    monkeypatch.setenv("JHZ_VARIANT", variant)
    m = _load_task()
    from yam_arena import station
    from yam_arena.demo import ArmPlan
    for arm, steps in m.demo().items():
        plan = ArmPlan(steps, station.BASES[arm])
        for t in np.arange(0.0, plan.duration, 0.05):
            plan.action7(t)
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)


def test_variant_env_is_not_left_set():
    assert os.environ.get("JHZ_VARIANT") in (None, "demo")
