"""Checks of the chart geometry and the scoring with synthetic marker poses. Most need only numpy and pytest:

    python -m pytest -q test_scoring.py

The tests at the end load task.py and need yam_arena (the official stack); elsewhere they are skipped.
"""

import math
import os
from pathlib import Path

import numpy as np
import pytest

import chart as C
import scoring as S

TOP = 0.7475
PAPER = S.paper_top(TOP)
NM_DR = C.lat_minute(C.DR[0])
STILL = [np.array([*C.DR_XY, PAPER])] * (S.SETTLE_STEPS + 1)


def quat(tilt_deg=0.0, yaw_deg=0.0):
    t, p = math.radians(tilt_deg) / 2, math.radians(yaw_deg) / 2
    sx, cx, sz, cz = math.sin(t), math.cos(t), math.sin(p), math.cos(p)
    return np.array([cz * sx, sz * sx, sz * cx, cz * cx])


def marker(x, y, base_z=PAPER, tilt_deg=0.0, yaw_deg=0.0):
    """The reading of a marker whose base centre is at x, y, base_z."""
    q = quat(tilt_deg, yaw_deg)
    pos = np.array([x, y, base_z]) + S.rotation(q) @ np.array([0.0, 0.0, S.MARKER_HALF_H])
    return S.read(pos, q, TOP)


def pts(*a, **k):
    return S.points(marker(*a, **k))


# -- the chart -----------------------------------------------------------------------------------------------------

def test_scales_are_mercator_at_50n():
    assert C.LON_MIN == pytest.approx(0.02381, abs=1e-5)
    assert C.lat_minute(50.0) == pytest.approx(1852 / 50_000, abs=1e-6)
    assert C.lat_minute(C.LAT_N) > C.lat_minute(C.LAT_S)  # minutes of latitude lengthen northward


def test_projection_round_trips():
    for lat, lon in [(50.01, -2.99), (50.1, -2.75), C.BUOY, C.DR]:
        assert C.to_chart(*C.to_world(lat, lon)) == pytest.approx((lat, lon), abs=1e-9)


def test_a_minute_of_latitude_on_the_chart_is_a_nautical_mile():
    x0, _ = C.to_world(50.05, -2.9)
    x1, _ = C.to_world(50.05 + 1 / 60, -2.9)
    assert (x1 - x0) / C.lat_minute(50.05 + 0.5 / 60) == pytest.approx(1.0, abs=1e-4)


def test_the_dr_position_is_6_nm_from_the_buoy_on_the_chart():
    lat_mid = (C.BUOY[0] + C.DR[0]) / 2
    assert math.dist(C.BUOY_XY, C.DR_XY) / C.lat_minute(lat_mid) == pytest.approx(6.0, abs=0.01)


def test_the_course_line_runs_045_and_passes_through_the_answer():
    (bx, by), (dx, dy), (ex, ey) = C.BUOY_XY, C.DR_XY, C.LINE_END_XY
    assert math.degrees(math.atan2(-(dy - by), dx - bx)) == pytest.approx(45.0, abs=0.05)
    assert math.degrees(math.atan2(-(ey - by), ex - bx)) == pytest.approx(45.0, abs=0.05)
    assert C.course_point(math.dist(C.BUOY_XY, C.DR_XY)) == pytest.approx(C.DR_XY, abs=2e-4)


def test_the_longitude_scale_mistake_lands_2_nm_short():
    assert S.chart_nm(*C.LON_SCALE_XY) == pytest.approx(6 * (1 - math.cos(math.radians(50))), abs=0.05)


def test_everything_is_on_the_chart_and_the_sheet_is_a3_sized():
    for p in (C.BUOY_XY, C.DR_XY, C.LINE_END_XY, C.LON_SCALE_XY):
        assert S.on_chart(*p)
    w, h = sorted(C.PAPER_SIZE[:2])
    assert w == pytest.approx(0.297, abs=0.003) and h == pytest.approx(0.420, abs=0.004)


def test_decals_lie_on_the_sheet():
    hx, hy = C.PAPER_SIZE[0] / 2, C.PAPER_SIZE[1] / 2
    for (cx, cy, cz), (sx, sy, sz), _, _ in C.decals():
        assert abs(cx) <= hx + 1e-9 and abs(cy) <= hy + 1e-9
        assert cz - sz / 2 >= C.PAPER_THICK / 2 - 1e-12  # on top of the paper, not inside it


# -- scoring -------------------------------------------------------------------------------------------------------

@pytest.mark.parametrize(("off_nm", "expected"), [(0.0, 1.0), (0.19, 1.0), (0.21, 0.5), (0.49, 0.5), (0.51, 0.0),
                                                  (2.0, 0.0)])
def test_points_by_distance_in_any_direction(off_nm, expected):
    for bearing in (0, 45, 135, 270):
        b = math.radians(bearing)
        x = C.DR_XY[0] + off_nm * NM_DR * math.cos(b)
        y = C.DR_XY[1] + off_nm * NM_DR * math.sin(b)
        assert pts(x, y) == expected, bearing


@pytest.mark.parametrize(("point", "expected"), [(C.DR_XY, 1.0), (C.LON_SCALE_XY, 0.0), (C.BUOY_XY, 0.0),
                                                 (C.LINE_END_XY, 0.0)])
def test_named_spots(point, expected):
    assert pts(*point) == expected


def test_marker_must_lie_flat_on_the_paper():
    assert pts(*C.DR_XY, tilt_deg=8) == 1.0
    assert pts(*C.DR_XY, tilt_deg=30) == 0.0
    assert pts(*C.DR_XY, base_z=PAPER + 0.02) == 0.0  # held above it
    assert pts(*C.DR_XY, yaw_deg=73) == 1.0  # its turn doesn't matter


def test_off_the_charted_area_scores_nothing():
    assert pts(C.INNER_X[0] - 0.003, C.DR_XY[1]) == 0.0
    assert pts(0.115, 0.0, base_z=TOP) == 0.0  # still on the table where it started


def test_settled_needs_a_full_second_within_2_mm():
    assert S.settled(STILL)
    assert not S.settled(STILL[:-1])
    assert not S.settled([p + np.array([0.003, 0, 0]) for p in STILL[:5]] + STILL[5:])


@pytest.mark.parametrize(("clear", "history", "expected"), [(True, STILL, 1.0), (False, STILL, 0.0),
                                                            (None, STILL, 0.0), (True, STILL[:5], 0.0)])
def test_full_score_needs_let_go_and_left_alone(clear, history, expected):
    assert S.final(marker(*C.DR_XY), clear, history) == expected


def test_gripper_clear():
    pos = np.array([*C.DR_XY, PAPER + S.MARKER_HALF_H])
    down = np.array([0.0, 0.0, -1.0])
    q = quat()
    assert not S.gripper_clear(pos + np.array([0, 0, 0.03]), down, pos, q)  # hand on the knob
    assert S.gripper_clear(pos + np.array([0, 0, 0.25]), down, pos, q)  # risen 25 cm above it
    assert S.gripper_clear(pos + np.array([0.0, -0.2, 0.1]), down, pos, q)  # moved aside


# -- these need yam_arena ----------------------------------------------------------------------------------------

def _load_task():
    pytest.importorskip("yam_arena")
    from yam_arena.task_file import load
    return load(Path(__file__).with_name("task.py")).module


def test_task_loads_and_names_three_objects():
    m = _load_task()
    assert [o.name for o in m.OBJECTS] == ["chart", "marker", "dividers"]
    assert len(m.CHART.decals) == len(C.decals())


@pytest.mark.parametrize("variant", ["demo", "dividers", "close", "half", "lon_scale", "at_buoy", "line_end"])
def test_each_variant_target_scores_as_expected(variant):
    m = _load_task()
    x, y, expected = m.VARIANTS[variant]
    assert pts(x, y) == expected


def test_dividers_open_to_6_nm_inside_their_hinge_range():
    m = _load_task()
    assert 2 * m.LEG_L * math.sin(math.radians(m.DIV_OPEN_DEG) / 2) == pytest.approx(m.DIV_SPAN)
    assert 0 < m.DIV_START_DEG < m.DIV_OPEN_DEG < 120
    # opened, the dividers stay on the paper and leg B's point stays clear of the answer
    tip = (m.DIV_HEAD[0] + m.LEG_L * math.cos(m.DIV_YAW + math.radians(m.DIV_OPEN_DEG)),
           m.DIV_HEAD[1] + m.LEG_L * math.sin(m.DIV_YAW + math.radians(m.DIV_OPEN_DEG)))
    assert C.PAPER_X[0] < tip[0] < C.PAPER_X[1] and C.PAPER_Y[0] < tip[1] < C.PAPER_Y[1]
    assert math.dist(tip, C.DR_XY) > S.DISC_R + 0.02


@pytest.mark.parametrize("variant", ["demo", "dividers", "close", "half", "lon_scale", "at_buoy", "line_end", "held",
                                     "nothing"])
def test_the_demo_is_within_reach(variant, monkeypatch):
    pytest.importorskip("yam_arena")
    monkeypatch.setenv("CDR_VARIANT", variant)
    m = _load_task()
    from yam_arena import station
    from yam_arena.demo import ArmPlan
    for arm, steps in m.demo().items():
        plan = ArmPlan(steps, station.BASES[arm])
        for t in np.arange(0.0, plan.duration, 0.05):
            plan.action7(t)
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)


def test_variant_env_is_not_left_set():
    assert os.environ.get("CDR_VARIANT") in (None, "demo")
