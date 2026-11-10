"""Offline checks of the zone rule and the grading with synthetic poses (no simulator).

    python -m pytest -q test_scoring.py

The last tests replay each scripted variant's planned placements through the scorer; they need yam_arena
importable for the Waypoint type and are skipped otherwise.
"""

import math

import pytest

import scoring as S
from scoring import Pose

SX, SY = S.SOCKET_XY
NX, NY = S.NEW_BOX_XY
HB, HL, HW, G = S.BOX / 2, S.PIPE_L / 2, S.PIPE_W / 2, 0.0075
UP, ALONG = 0.0, math.pi / 2           # yaw of a length running up the wall / along it

BENCH = {"conduit_a": Pose(0.385, 0.26, UP, low_z=S.TOP),
         "conduit_b": Pose(0.385, -0.26, UP, low_z=S.TOP),
         "conduit_c": Pose(0.385, 0.32, UP, low_z=S.TOP)}

ROW_B = Pose(SX, SY - HB - G - HL, ALONG)           # along the wall, level with the socket
COL_B = Pose(NX - HB - G - HL, NY, UP)              # up the wall, in line with the back box
COL_A = Pose(SX + HB + G + HL, SY, UP)              # up the wall, in line with the socket
ROW_A = Pose(NX, NY + HB + G + HL, ALONG)           # along the wall, level with the back box
DIAG = Pose((SX + NX) / 2, (SY + NY) / 2, math.atan2(NY - SY, NX - SX))


def laid(**pieces):
    out = dict(BENCH)
    out.update(pieces)
    return out


def test_both_l_routes_score_full():
    assert S.grade(laid(conduit_a=ROW_B, conduit_b=COL_B)) == 1.0
    assert S.grade(laid(conduit_a=COL_A, conduit_c=ROW_A)) == 1.0


def test_zones_named():
    assert S.zone(ROW_B) == "socket row"
    assert S.zone(COL_B) == "new_box column"
    assert S.zone(COL_A) == "socket column"
    assert S.zone(ROW_A) == "new_box row"
    assert S.zone(DIAG) is None


def test_length_reversed_end_for_end_is_the_same_run():
    flipped = Pose(COL_B.x, COL_B.y, math.pi)
    assert S.grade(laid(conduit_a=ROW_B, conduit_b=flipped)) == 1.0


def test_diagonal_joins_the_boxes_but_scores_zero():
    poses = laid(conduit_b=DIAG)
    assert S.joined(poses, S.JOINT)            # the tempting short cut does make the connection
    assert S.grade(poses) == 0.0


def test_full_route_plus_stray_length_on_panel_scores_zero():
    stray = Pose(0.30, -0.03, UP)
    assert S.zone(stray) is None and S.on_panel(stray)
    assert S.grade(laid(conduit_a=ROW_B, conduit_b=COL_B, conduit_c=stray)) == 0.0


def test_full_route_plus_diagonal_spare_scores_zero():
    assert S.grade(laid(conduit_a=ROW_B, conduit_b=COL_B, conduit_c=DIAG)) == 0.0


def test_one_leg_butted_to_a_box_is_a_quarter():
    assert S.grade(laid(conduit_b=COL_B)) == 0.25
    assert S.grade(laid(conduit_a=COL_A)) == 0.25


def test_leg_in_zone_but_touching_nothing_scores_zero():
    assert S.grade(laid(conduit_b=Pose(COL_B.x - 0.06, COL_B.y, UP))) == 0.0


@pytest.mark.parametrize("short", [0.021, 0.03, 0.04])
def test_open_joint_is_a_half(short):
    # The column pulled back from the back box: gap = 7.5 mm + short.
    assert S.grade(laid(conduit_a=ROW_B, conduit_b=Pose(COL_B.x - short, COL_B.y, UP))) == 0.5


def test_joint_too_open_falls_back_to_started():
    poses = laid(conduit_a=ROW_B, conduit_b=Pose(COL_B.x - 0.045, COL_B.y, UP))   # 52.5 mm short of the box
    assert S.grade(poses) == 0.25                                                 # the row still butts the socket


def test_nothing_scores_zero():
    assert S.grade(dict(BENCH)) == 0.0


def test_lengths_off_the_panel_or_held_are_ignored():
    held = Pose(0.30, 0.0, 0.7, low_z=S.PANEL_TOP + 0.08)       # carried across the panel, out of zone
    assert not S.on_panel(held)
    assert S.grade(laid(conduit_a=ROW_B, conduit_b=COL_B, conduit_c=held)) == 1.0


@pytest.mark.parametrize("bad", [
    Pose(COL_B.x, COL_B.y, UP, tilt_deg=15.0),                       # propped up at one end
    Pose(COL_B.x, COL_B.y, UP, low_z=S.PANEL_TOP + S.BOX_H),         # lying on top of a box
    Pose(COL_B.x, COL_B.y, UP, low_z=S.PANEL_TOP + S.PIPE_H),        # on top of another length
    Pose(COL_B.x, COL_B.y, UP, resting=False),                       # still moving at the end
    Pose(COL_B.x, COL_B.y, math.radians(12)),                        # skewed more than 10 degrees
    Pose(COL_B.x, COL_B.y - 0.04, UP),                               # beside the back box's line, not in it
])
def test_faulty_leg_scores_zero(bad):
    assert S.grade(laid(conduit_a=ROW_B, conduit_b=bad)) == 0.0


def test_small_skew_and_offset_within_tolerance():
    ok = Pose(COL_B.x, COL_B.y - 0.008, math.radians(3))   # 8 mm off the plan, 3 degrees skew
    assert S.zone(ok) == "new_box column"
    assert S.grade(laid(conduit_a=ROW_B, conduit_b=ok)) == 1.0


def test_length_hanging_off_the_panel_edge_counts_as_laid():
    # Centre beyond the right-hand edge but one end on the face, out of every zone: still a fault.
    edge = Pose(0.30, S.PANEL_Y[0] - 0.005, ALONG)
    assert S.on_panel(edge) and S.zone(edge) is None
    assert S.grade(laid(conduit_b=edge)) == 0.0


def test_staircase_through_open_wall_scores_zero():
    # Up from the socket, then along the wall at mid-height (level with neither box): out of zone.
    mid_row = Pose(0.30, 0.03, ALONG)
    assert S.grade(laid(conduit_a=COL_A, conduit_c=mid_row)) == 0.0


def test_gap_geometry():
    a = S.rect(0.0, 0.0, 0.1, 0.02, 0.0)
    assert S.gap(a, S.rect(0.08, 0.0, 0.04, 0.02, 0.0)) == pytest.approx(0.01)
    assert S.gap(a, S.rect(0.0, 0.0, 0.04, 0.04, 0.3)) == 0.0
    assert S.gap(a, S.rect(0.0, 0.05, 0.1, 0.02, 0.0)) == pytest.approx(0.03)


def test_yaw_tilt_from_quaternion():
    yaw, tilt = S.yaw_tilt((0, 0, math.sin(0.4), math.cos(0.4)))
    assert yaw == pytest.approx(0.8) and tilt == pytest.approx(0.0, abs=1e-9)
    roll = S.yaw_tilt((math.sin(math.pi / 4), 0, 0, math.cos(math.pi / 4)))   # quarter roll about its axis
    assert roll[1] == pytest.approx(0.0, abs=1e-9)
    pitch = S.yaw_tilt((0, math.sin(math.radians(10)), 0, math.cos(math.radians(10))))   # 20 deg nose-up
    assert pitch[1] == pytest.approx(20.0)


# ---------------------------------------------------------------- the scripted plans, replayed

def _planned(task, variant, monkeypatch):
    monkeypatch.setenv("CSZ_VARIANT", variant)
    plan = task.demo()
    poses = {n: Pose(*task.START[n], UP, low_z=S.TOP) for n in S.PIPES}
    for waypoints in plan.values():
        held = None
        for w in waypoints:
            if w.label.endswith(": above"):
                held = w.label.split(":")[0]
            if w.label == "release" and held:
                poses[held] = Pose(w.pos[0], w.pos[1], w.yaw)
                held = None
    return poses


def test_scripted_variants_score_as_expected(monkeypatch):
    task = pytest.importorskip("task")
    for variant, expected in task.EXPECTED.items():
        assert S.grade(_planned(task, variant, monkeypatch)) == expected, variant


def test_planned_placements_clear_boxes_and_each_other(monkeypatch):
    task = pytest.importorskip("task")
    boxes = [S.box_rect(b) for b in S.BOXES]
    for variant in task.VARIANTS:
        poses = _planned(task, variant, monkeypatch)
        rects = [S.pipe_rect(p) for p in poses.values()]
        for i, r in enumerate(rects):
            assert all(S.gap(r, b) >= 0.005 for b in boxes), variant
            assert all(S.gap(r, q) >= 0.005 for q in rects[i + 1:]), variant


def test_start_off_the_panel():
    for p in BENCH.values():
        corners = S.pipe_rect(p)
        assert all(not (S.PANEL_Y[0] < y < S.PANEL_Y[1]) for _, y in corners)
