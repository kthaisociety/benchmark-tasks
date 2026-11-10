"""Checks of the scoring logic with synthetic poses, without the simulator (needs only NumPy and pytest):

    python -m pytest -q test_scoring.py
"""

import math

import numpy as np
import pytest

import scoring as S


def yaw_quat(yaw, tilt_deg=0.0):
    """A fitting turned ``yaw`` about the vertical, then tilted ``tilt_deg`` about world x (nose up or down)."""
    t = math.radians(tilt_deg)
    qz = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    qx = (math.sin(t / 2), 0.0, 0.0, math.cos(t / 2))
    # qx * qz (tilt applied after the yaw)
    x1, y1, z1, w1 = qx
    x2, y2, z2, w2 = qz
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


BRASS_TO_COPPER = math.pi        # local +y -> world -y: the brass end toward the copper section (joint A)
BRASS_TO_STEEL = 0.0             # local +y -> world +y
IN_TRAY = math.pi / 2            # as they start: axis front to back
TRAY = {"steel_coupling": (0.20, 0.10, 0.04), "union": (0.20, 0.0, 0.042), "cu_coupling": (0.20, -0.10, 0.04)}


def at_gap(name, gap, yaw, dx=0.0, dy=0.0, dz=0.0, tilt=0.0, moving=False):
    return S.Pose(pos=(S.PIPE_X + dx, S.GAP_Y[gap] + dy, S.SEAT_Z[name] + dz), quat_xyzw=yaw_quat(yaw, tilt),
                  lin_vel=(0.05, 0.0, 0.0) if moving else (0.0, 0.0, 0.0))


def in_tray(name):
    return S.Pose(pos=TRAY[name], quat_xyzw=yaw_quat(IN_TRAY))


def scene(**placed):
    return {name: placed.get(name, in_tray(name)) for name in S.FITTINGS}


FREE = dict.fromkeys(S.FITTINGS, True)
FULL = {"union": at_gap("union", "A", BRASS_TO_COPPER), "cu_coupling": at_gap("cu_coupling", "B", math.pi)}


def test_start_scores_zero():
    assert S.score(scene(), False, FREE) == 0.0


def test_both_joints_right_is_full_score():
    assert S.score(scene(**FULL), False, FREE) == 1.0
    assert S.success(scene(**FULL), False, FREE)


def test_coupling_either_way_round():
    s = scene(union=FULL["union"], cu_coupling=at_gap("cu_coupling", "B", 0.0))
    assert S.score(s, False, FREE) == 1.0


def test_union_alone_scores_three_quarters():
    assert S.score(scene(union=FULL["union"]), False, FREE) == 0.75


def test_copper_coupling_alone_scores_a_quarter():
    assert S.score(scene(cu_coupling=FULL["cu_coupling"]), False, FREE) == 0.25


@pytest.mark.parametrize("placed", [
    {"union": at_gap("union", "A", BRASS_TO_STEEL)},                                          # union reversed
    {"union": at_gap("union", "A", BRASS_TO_STEEL), "cu_coupling": FULL["cu_coupling"]},
    {"union": at_gap("union", "A", BRASS_TO_STEEL + 0.9), "cu_coupling": FULL["cu_coupling"]},  # 50 deg off reversed
    {"steel_coupling": at_gap("steel_coupling", "A", 0.0), "cu_coupling": FULL["cu_coupling"]},  # plain steel in A
    {"cu_coupling": at_gap("cu_coupling", "A", 0.0)},                                         # plain copper in A
    {"union": at_gap("union", "B", BRASS_TO_COPPER)},                                         # union between coppers
    {"union": at_gap("union", "B", BRASS_TO_STEEL), "cu_coupling": at_gap("cu_coupling", "A", 0.0)},  # swapped
    {"union": FULL["union"], "steel_coupling": at_gap("steel_coupling", "B", 0.0)},           # steel between coppers
])
def test_any_wrong_fitting_in_a_joint_scores_zero(placed):
    assert S.score(scene(**placed), False, FREE) == 0.0


def test_a_sloppy_wrong_fitting_still_counts_as_wrong():
    # propped on the steel pipe's end, 5 cm off-centre and 2 cm high, tilted 25 degrees
    sloppy = at_gap("steel_coupling", "A", 0.0, dy=0.05, dz=0.02, tilt=25)
    assert S.wrong_fittings(scene(steel_coupling=sloppy, **FULL)) == ["steel_coupling@A"]
    assert S.score(scene(steel_coupling=sloppy, **FULL), False, FREE) == 0.0


def test_a_wrong_fitting_held_high_above_a_gap_is_not_in_it():
    above = at_gap("steel_coupling", "A", 0.0, dz=0.08)
    assert S.score(scene(steel_coupling=above, **FULL), False, FREE) == 1.0


def test_union_lying_across_gap_a_is_not_seated_but_not_wrong():
    across = at_gap("union", "A", math.pi / 2)
    assert S.wrong_fittings(scene(union=across)) == []
    assert S.score(scene(union=across, cu_coupling=FULL["cu_coupling"]), False, FREE) == 0.25


@pytest.mark.parametrize(("kw", "seated"), [
    ({"dy": 0.014}, True), ({"dy": -0.016}, False),
    ({"dx": 0.009}, True), ({"dx": 0.011}, False),
    ({"dz": 0.005}, True), ({"dz": 0.010}, False), ({"dz": -0.007}, False),
    ({"tilt": 12}, True), ({"tilt": 18}, False),
    ({"moving": True}, False),
])
def test_seating_tolerances(kw, seated):
    union = at_gap("union", "A", BRASS_TO_COPPER, **kw)
    assert S.score(scene(union=union), False, FREE) == (0.75 if seated else 0.0)


def test_union_turned_slightly_off_the_line_still_counts():
    assert S.score(scene(union=at_gap("union", "A", BRASS_TO_COPPER + math.radians(12))), False, FREE) == 0.75
    assert S.score(scene(union=at_gap("union", "A", BRASS_TO_COPPER + math.radians(20))), False, FREE) == 0.0


def test_knocked_pipework_scores_zero_and_stays_zero():
    start = S.Pose(pos=(S.PIPE_X, 0.0, 0.001))
    assert not S.knocked(S.Pose(pos=(S.PIPE_X + 0.005, 0.0, 0.001)), start)
    assert S.knocked(S.Pose(pos=(S.PIPE_X + 0.011, 0.0, 0.001)), start)
    assert S.knocked(S.Pose(pos=(S.PIPE_X, 0.0, 0.001), quat_xyzw=yaw_quat(math.radians(2.5))), start)
    assert S.score(scene(**FULL), True, FREE) == 0.0


def test_seat_heights():
    assert S.AXIS_Z == pytest.approx(0.030 + 0.008 + math.sqrt(0.025 ** 2 - 0.016 ** 2))
    assert S.SEAT_Z["union"] - S.SEAT_Z["cu_coupling"] == pytest.approx(0.00254, abs=1e-4)


def test_a_fitting_still_held_or_settling_does_not_count():
    held = {**FREE, "union": False}
    assert S.score(scene(**FULL), False, held) == 0.25
    assert S.score(scene(**FULL), False, dict.fromkeys(S.FITTINGS, False)) == 0.0


def test_a_held_wrong_fitting_in_a_gap_still_scores_zero():
    s = scene(steel_coupling=at_gap("steel_coupling", "A", 0.0), cu_coupling=FULL["cu_coupling"])
    assert S.score(s, False, {**FREE, "steel_coupling": False}) == 0.0


def _gripper_above(p, height):
    """A gripper pointing straight down with its pad tips ``height`` above the fitting's axis."""
    tip = np.asarray(p.pos) + np.array([0.0, 0.0, height])
    return (tip + np.array([0.0, 0.0, S.PAD_TIP]), np.array([0.0, 0.0, -1.0]))


def test_let_go_needs_5_cm_between_any_gripper_and_the_fitting():
    p = FULL["union"]
    assert S.let_go(p, [_gripper_above(p, S.NUT_R + 0.051)])
    assert not S.let_go(p, [_gripper_above(p, S.NUT_R + 0.045)])
    assert not S.let_go(p, [_gripper_above(p, 0.0)])          # fingers around it
    assert not S.let_go(p, None)                              # arms unreadable: never counts as let go
    beside = (np.asarray(p.pos) + np.array([0.0, S.FIT_LEN / 2 + 0.06, 0.0]), np.array([0.0, 0.0, -1.0]))
    assert S.let_go(p, [beside])


def test_settled_needs_one_second_within_2_mm():
    still = [(0.38, 0.15, 0.06)] * S.SETTLE_STEPS
    assert S.settled(still)
    assert not S.settled(still[:-1])                          # not a full second of samples yet
    drift = [(0.38, 0.15 + 0.0025 * i / S.SETTLE_STEPS, 0.06) for i in range(S.SETTLE_STEPS)]
    assert not S.settled(drift)
    assert S.settled([(0.30, 0.0, 0.1)] * 5 + still)          # earlier motion is forgotten
