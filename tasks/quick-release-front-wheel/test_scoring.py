"""Offline checks of the scorer and the lever's mechanism, no simulator: python -m pytest -q test_scoring.py

States come from the wheel's own kinematics (parts.Part.state), so the tests use the same lever and axle geometry as
the simulation."""

import itertools
import math

import numpy as np
import pytest

import bike
import layout as L
import mechanism as M
import scoring as S

FX, FY = L.FORK_XY
SEAT = (FX, FY, L.TOP + L.H_SEAT)
CLOSED = -math.pi / 2


def yaw_q(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def wheel(pos=SEAT, cam=0.0, aim=0.0, yaw=0.0, moving=False):
    s = bike.WHEEL.state(joints={"cam": cam, "aim": aim}, pos=pos, quat_xyzw=yaw_q(yaw))
    if moving:
        s = type(s)(**{**s.__dict__, "lin_vel": np.array((0.05, 0.0, 0.0))})
    return {"wheel": s}


def run(*frames, repeat=1):
    t = S.Trial()
    out = [t.step(f) for f in frames for _ in range(repeat)]
    return out[-1], t


def deg(d):
    return math.radians(d)


# -- end states --------------------------------------------------------------------------------------------------------
def test_start_scores_zero():
    v, t = run({"wheel": bike.WHEEL.state()})
    assert (v, t.fault, t.what) == (0.0, "", "not in the dropouts")


def test_seated_and_closed_along_the_blade_is_full():
    v, t = run(wheel(cam=CLOSED))
    assert (v, t.fault) == (1.0, "")
    assert S.lever_aim(t_w := wheel(cam=CLOSED)["wheel"]) == pytest.approx(0.0, abs=1e-6)
    assert t.success(wheel(cam=CLOSED)) and not t.success(wheel(cam=CLOSED, moving=True))
    assert t_w.links["cam"][0][0] == pytest.approx(FX + L.PIVOT_X)


def test_seated_lever_open_or_part_way_is_half():
    for c in (0.0, 30.0, 59.0, 79.0):
        assert run(wheel(cam=-deg(c)))[0] == 0.5, c
    assert run(wheel(cam=-deg(80.0)))[0] == 1.0


def test_lever_pointing_forward_is_three_quarters_rearward_is_full():
    assert run(wheel(cam=CLOSED, aim=deg(29.0)))[0] == 1.0
    assert run(wheel(cam=CLOSED, aim=deg(31.0)))[0] == 0.75
    assert run(wheel(cam=CLOSED, aim=deg(90.0)))[0] == 0.75
    assert run(wheel(cam=CLOSED, aim=deg(-90.0)))[0] == 1.0  # toward the rear


def test_lever_side_is_not_scored():
    flipped = wheel(cam=CLOSED, yaw=math.pi)  # wheel in the other way round: lever on the far side
    assert flipped["wheel"].links["cam"][0][0] == pytest.approx(FX - L.PIVOT_X)
    assert run(flipped)[0] == 1.0
    # turned round, a positive aim points the lever to the rear
    assert run(wheel(cam=CLOSED, aim=deg(60.0), yaw=math.pi))[0] == 1.0
    assert run(wheel(cam=CLOSED, aim=deg(-60.0), yaw=math.pi))[0] == 0.75


def test_lateral_play_still_seats():
    for dx in (-0.014, 0.014):
        assert run(wheel(pos=(FX + dx, FY, SEAT[2]), cam=CLOSED))[0] == 1.0


def test_not_fully_down_is_not_seated():
    up = (FX, FY, SEAT[2] + S.SEAT_Z + 0.001)
    assert run(wheel(pos=up))[0] == 0.2
    assert run(wheel(pos=up, cam=CLOSED))[0] == 0.0
    assert run(wheel(pos=(FX, FY, SEAT[2] + S.SEAT_Z - 0.0005)))[0] == 0.5


def test_off_centre_in_the_slot_is_not_seated():
    assert run(wheel(pos=(FX, FY + 0.008, SEAT[2] + 0.002)))[0] == 0.2


def test_cocked_one_side_seated_is_point_two_closed_is_zero():
    # turned 0.3 rad and 2 cm toward the rear: the near end in its slot, the far end on the far jaw, outside the mouth
    near_in = (FX, FY - 0.020, SEAT[2] + 0.004)
    w = wheel(pos=near_in, yaw=-0.30)["wheel"]
    near, far = S.at_dropout(w, -1), S.at_dropout(w, 1)
    assert abs(near[0]) < L.MOUTH_HALF < abs(far[0])
    assert run(wheel(pos=near_in, yaw=-0.30))[0] == 0.0  # far end outside its mouth
    # far end only just in its mouth
    assert run(wheel(pos=(FX, FY - 0.004, SEAT[2] + 0.004), yaw=-0.30))[0] == 0.2
    assert run(wheel(pos=(FX, FY - 0.004, SEAT[2] + 0.004), yaw=-0.30, cam=CLOSED))[0] == 0.0


def test_resting_on_top_of_the_dropouts_is_zero():
    assert run(wheel(pos=(FX, FY + 0.03, L.TOP + L.MOUTH_TOP + 0.01)))[0] == 0.0
    assert run(wheel(pos=(FX, FY, L.TOP + L.MOUTH_TOP + 0.02)))[0] == 0.0  # held above the fork


def test_closed_in_the_stand_is_zero():
    v, t = run({"wheel": bike.WHEEL.state(joints={"cam": CLOSED})}, repeat=5)
    assert (v, t.fault, t.what) == (0.0, "", "lever closed on an unseated axle")


# -- the latched fault -------------------------------------------------------------------------------------------------
def test_closing_then_fitting_is_a_latched_fault():
    stand_closed = {"wheel": bike.WHEEL.state(joints={"cam": CLOSED})}
    v, t = run(stand_closed, wheel(cam=CLOSED))
    assert (v, t.fault) == (0.0, "the wheel moved while the lever was closed")
    # reopening, then closing it properly afterwards, doesn't clear it
    for f in (wheel(cam=0.0), wheel(cam=CLOSED)):
        assert t.step(f) == 0.0
    assert t.fault


def test_reopening_before_moving_the_wheel_is_fine():
    stand_closed = {"wheel": bike.WHEEL.state(joints={"cam": CLOSED})}
    stand_open = {"wheel": bike.WHEEL.state(joints={"cam": -deg(50.0)})}
    v, t = run(stand_closed, stand_open, wheel(cam=0.0), wheel(cam=CLOSED))
    assert (v, t.fault) == (1.0, "")


def test_small_settling_while_closed_is_fine():
    v, t = run(wheel(cam=CLOSED), wheel(pos=(FX + 0.004, FY, SEAT[2]), cam=CLOSED))
    assert (v, t.fault) == (1.0, "")


def test_pulling_a_closed_wheel_out_is_a_fault():
    _, t = run(wheel(cam=CLOSED))
    t.step(wheel(pos=(FX, FY, SEAT[2] + 0.02), cam=CLOSED))
    assert t.step(wheel(cam=CLOSED)) == 0.0 and t.fault


def test_a_new_trial_clears_the_fault():
    _, t = run({"wheel": bike.WHEEL.state(joints={"cam": CLOSED})}, wheel(cam=CLOSED))
    assert t.fault
    assert run(wheel(cam=CLOSED))[1].fault == ""


# -- every outcome is covered -----------------------------------------------------------------------------------------
def test_every_state_gets_a_listed_score():
    allowed = {0.0, 0.2, 0.5, 0.75, 1.0}
    xs = (FX - 0.02, FX, FX + 0.02, L.STAND_XY[0])
    ys = (FY - 0.03, FY - 0.01, FY, FY + 0.01, L.STAND_XY[1])
    zs = (L.TOP + L.R_OUT, SEAT[2], SEAT[2] + 0.005, L.TOP + L.MOUTH_TOP + 0.02)
    for x, y, z, yaw, cam, aim in itertools.product(xs, ys, zs, (0.0, 0.3, math.pi / 2, math.pi),
                                                     (0.0, -deg(45), CLOSED), (0.0, deg(60), deg(-60))):
        assert run(wheel(pos=(x, y, z), yaw=yaw, cam=cam, aim=aim))[0] in allowed


# -- the lever's mechanism ---------------------------------------------------------------------------------------------
def test_open_detent_springs_back():
    m = M.Lever()
    assert m.targets(-deg(10.0), 0.0) == (pytest.approx(0.0), pytest.approx(0.0))


def test_friction_zone_holds_where_left_and_trails_by_slip():
    m = M.Lever()
    cam, _ = m.targets(-deg(30.0), 0.0)
    assert -math.degrees(cam) == pytest.approx(30.0)
    cam, _ = m.targets(-deg(33.0), 0.0)  # moved less than SLIP: the rest point stays
    assert -math.degrees(cam) == pytest.approx(30.0)
    cam, _ = m.targets(-deg(45.0), 0.0)  # pushed on: it trails by SLIP
    assert -math.degrees(cam) == pytest.approx(45.0 - L.SLIP)


def test_over_centre_snaps_closed_and_clamps_the_skewer():
    m = M.Lever()
    m.targets(-deg(40.0), deg(10.0))
    cam, aim = m.targets(-deg(61.0), deg(12.0))
    assert -math.degrees(cam) == pytest.approx(90.0)
    cam, aim = m.targets(-deg(85.0), deg(14.0))  # closed: the skewer's rest point is frozen where it closed
    cam, aim = m.targets(-deg(90.0), deg(40.0))
    assert math.degrees(aim) == pytest.approx(14.0)
    cam, aim = m.targets(-deg(50.0), deg(40.0))  # pulled back open past the over-centre point: free again
    assert math.degrees(aim) == pytest.approx(40.0 - L.SLIP)
    assert -math.degrees(cam) == pytest.approx(50.0)


def test_skewer_turns_with_friction_while_open():
    m = M.Lever()
    _, aim = m.targets(0.0, deg(4.0))
    assert math.degrees(aim) == pytest.approx(0.0)
    _, aim = m.targets(0.0, deg(20.0))
    assert math.degrees(aim) == pytest.approx(20.0 - L.SLIP)


# -- geometry sanity -----------------------------------------------------------------------------------------------------
def test_geometry_clearances():
    assert L.PLATE_X - L.PLATE_T / 2 - L.LOCKNUT_X == pytest.approx(0.015)  # 1.5 cm play each side
    assert L.AXLE_HALF - 0.015 > L.PLATE_X + L.PLATE_T / 2  # shifted fully, the axle still spans both dropouts
    assert L.H_SEAT - L.R_OUT > L.CROWN["z1"]  # seated, the tyre clears the crown
    assert L.H_SEAT - (L.LEVER["paddle"][1] + 0.001) > L.BASE["h"]  # the closed lever clears the stand's base
    assert L.PIVOT_X + L.LEVER["thick"] < L.BASE["x0"] - 0.01  # and hangs outside it
