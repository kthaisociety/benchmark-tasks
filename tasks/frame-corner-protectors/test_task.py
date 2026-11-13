"""Offline checks for the frame-corner-protectors task: the geometry, the scoring rules on hand-built states, and the
scripted demo's reach (inverse kinematics only, no Isaac Sim). Run from this folder with yam_arena importable:

    python -m pytest test_task.py -q
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import task as T  # noqa: E402
from partkit import PartState  # noqa: E402
from yam_arena import ik, station, task_file  # noqa: E402
from yam_arena.constants import FINGER_OPEN, READY  # noqa: E402
from yam_arena.demo import ArmPlan, grip_rotation  # noqa: E402

RUN_LIMIT_S = 2400 / 20
GRIPPER_SPAN = 0.094  # pads 94 mm apart fully open


def _quat_xyzw(yaw=0.0, roll=0.0):
    """Rotation about z by yaw, then about x by roll (xyzw)."""
    cy, sy, cr, sr = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(roll / 2), math.sin(roll / 2)
    # q = qz(yaw) * qx(roll)
    w = cy * cr
    x = cy * sr
    y = sy * sr
    z = sy * cr
    return np.array([x, y, z, w])


def _state(pos, yaw=0.0, roll=0.0, sliders=None, vel=0.0):
    return PartState(pos=np.asarray(pos, float), quat_xyzw=_quat_xyzw(yaw, roll), lin_vel=np.array([vel, 0, 0.0]),
                     ang_vel=np.zeros(3), low=np.zeros(3), high=np.zeros(3), sliders=sliders or {})


def _frame_state(dz=0.0, roll=0.0):
    return _state((T.FRAME_AT[0], T.FRAME_AT[1], T.FB + dz), roll=roll)


def _seated_protector(name, frame=None, squeeze=0.0015, shift=(0.0, 0.0, 0.0), yaw_err=0.0):
    frame = frame or _frame_state()
    corner = T.corner_world(frame, name)
    # seated: the corner at the pocket's back, the frame's front 0.5 mm above the origin (the roof on the frame's back)
    pos = corner + np.array([0.0, 0.0, -T.CLEAR]) + np.asarray(shift)
    return _state(pos, yaw=T.YAW[name] + math.radians(yaw_err), sliders={f"floor_{name}": -squeeze})


def _objects(prots=None, frame=None):
    frame = frame or _frame_state()
    out = {T.FRAME.name: frame}
    for name, spec in T.PROTECTORS.items():
        out[spec.name] = (prots or {}).get(name) or _state((*T.start_xy(name), T.TOP + T.LIP), yaw=T.YAW[name],
                                                           sliders={f"floor_{name}": 0.0})
    return out


@pytest.fixture(autouse=True)
def _fresh_trial():
    T._reset()
    T._S["tcps"] = [np.array(T.PARK["left"]), np.array(T.PARK["right"])]
    yield


# -- the task file loads ------------------------------------------------------------------------------------------------

def test_task_file_loads():
    tf = task_file.load(HERE / "task.py")
    names = set(tf.names)
    assert {"frame", "block_left", "block_right"} <= names
    assert {f"protector_{n}" for n in T.CORNERS} <= names
    assert tf.scene == "station" and tf.score is not None and tf.demo is not None


# -- geometry ----------------------------------------------------------------------------------------------------------

def test_protector_fits_the_open_gripper():
    width = T.POCKET + T.WALL
    assert width < GRIPPER_SPAN - 0.016  # at least 8 mm of room either side when the pads come down


def test_pocket_squeezes_the_frame():
    # roof above the frame's back by CLEAR; the floor stands PROUD above its front: net squeeze 1.5 mm, about 6 N
    squeeze = T.PROUD - T.CLEAR
    assert squeeze == pytest.approx(0.0015)
    assert 4.0 <= T.FLOOR_RATE * squeeze <= 10.0
    assert squeeze < T.FLOOR_TRAVEL


def test_the_bevels_leave_a_height_window_around_the_aim():
    # with the protector's origin at height z (relative to the frame's front), the floor's bevelled edge must be below the
    # frame's front and the roof's bevelled edge above its back
    lo = -(T.ROOF + T.RAMP_RISE - T.THICK)  # roof edge at the frame's back
    hi = T.RAMP_RISE - T.PROUD  # floor edge at the frame's front
    assert lo < T.Z_AIM < hi
    assert hi - lo >= 0.009


def test_corners_overhang_the_blocks_past_the_pocket():
    for name in T.CORNERS:
        cx, cy = T.corner_xy(name)
        sx, sy = T.CORNERS[name]
        # the seated protector's footprint, world
        lo_x, hi_x = sorted([cx - sx * T.POCKET, cx + sx * T.WALL])
        lo_y, hi_y = sorted([cy - sy * T.POCKET, cy + sy * T.WALL])
        bx0, bx1 = T.BLOCK_X - T.BLOCK[0] / 2, T.BLOCK_X + T.BLOCK[0] / 2
        by = sy * T.BLOCK_Y
        by0, by1 = by - T.BLOCK[1] / 2, by + T.BLOCK[1] / 2
        overlap = hi_x > bx0 and lo_x < bx1 and hi_y > by0 and lo_y < by1
        assert not overlap, name
        assert min(abs(lo_x - bx1), abs(hi_x - bx0)) >= 0.02  # 2 cm of clearance from the block's end


def test_blocks_sit_under_the_moulding():
    rail_in, rail_out = T.HALF[1] - T.MOULD_W, T.HALF[1]
    assert T.BLOCK_Y - T.BLOCK[1] / 2 < rail_out and T.BLOCK_Y + T.BLOCK[1] / 2 > rail_in


def test_protectors_start_clear_of_frame_and_blocks():
    for name in T.CORNERS:
        x, y = T.start_xy(name)
        sy = T.CORNERS[name][1]
        nearest = y - sy * T.POCKET  # the protector's edge nearest the frame
        assert abs(nearest) > T.BLOCK_Y + T.BLOCK[1] / 2 + 0.01, name


def test_pinch_stays_on_the_closed_wall_band():
    """The fingers close along world y; both pads must land on solid foam outside the frame's footprint along x."""
    for name in T.CORNERS:
        seat = T.corner_xy(name)
        g = T.grip_world(name, seat)
        sx = T.CORNERS[name][0]
        # the band is the closed wall beyond the frame's end along x: from the corner out to WALL
        band = sorted([seat[0], seat[0] + sx * T.WALL])
        assert band[0] < g[0] < band[1], name
        assert abs(g[0] - seat[0]) >= 0.010, name  # a 2 cm pad centred here stays (just) off the frame's end


# -- scoring -----------------------------------------------------------------------------------------------------------

def test_all_four_seated_and_let_go_scores_full():
    prots = {n: _seated_protector(n) for n in T.CORNERS}
    objs = _objects(prots)
    assert set(T.protected(objs)) == set(T.CORNERS)
    assert T.score(objs) == 1.0 and T.success(objs)


def test_partial_credit_counts_corners():
    for k in range(4):
        names = list(T.CORNERS)[:k]
        objs = _objects({n: _seated_protector(n) for n in names})
        assert T.score(objs) == pytest.approx(0.25 * k)


def test_moving_parts_cap_full_score():
    prots = {n: _seated_protector(n) for n in T.CORNERS}
    prots["far_left"] = PartState(**{**prots["far_left"].__dict__, "lin_vel": np.array([0.05, 0, 0])})
    assert T.score(_objects(prots)) == 0.75


@pytest.mark.parametrize("shift", [(0.006, 0, 0), (0, 0.006, 0), (-0.006, -0.006, 0), (0, 0, 0.005)])
def test_not_pushed_home_is_not_seated(shift):
    name = "far_left"
    frame = _frame_state()
    # shift in the protector's own frame: out along its pocket's opening
    yaw = T.YAW[name]
    c, s = math.cos(yaw), math.sin(yaw)
    world = (c * shift[0] - s * shift[1], s * shift[0] + c * shift[1], shift[2])
    if shift[0] > 0 or shift[1] > 0:  # into the wall: impossible physically, but the rule allows only 2 mm
        world = tuple(-v for v in world[:2]) + (world[2],)
    prot = _seated_protector(name, frame, shift=world)
    assert not T.seated_on(frame, prot, name)


def test_unsqueezed_floor_is_not_seated():
    frame = _frame_state()
    prot = _seated_protector("near_right", frame, squeeze=0.0)
    assert not T.seated_on(frame, prot, "near_right")


def test_turned_protector_is_not_seated():
    frame = _frame_state()
    prot = _seated_protector("near_left", frame, yaw_err=10.0)
    assert not T.seated_on(frame, prot, "near_left")


def test_wrong_protector_orientation_does_not_count_on_another_corner():
    frame = _frame_state()
    # the far-left protector, as turned, put on the near-left corner's place: its pocket faces the wrong way
    prot = _seated_protector("far_left", frame)
    assert not T.seated_on(frame, prot, "near_left")


def test_still_held_does_not_count():
    prots = {n: _seated_protector(n) for n in T.CORNERS}
    T._S["tcps"] = [T.protector_middle(prots["far_right"]) + np.array([0.0, 0.0, 0.02]), np.array(T.PARK["left"])]
    objs = _objects(prots)
    assert "far_right" not in T.protected(objs)
    assert T.score(objs) == 0.75


def test_dropped_frame_scores_zero():
    prots = {n: _seated_protector(n) for n in T.CORNERS}
    T._S["dropped"] = True
    assert T.score(_objects(prots)) == 0.0


def test_frame_off_the_blocks_or_tilted_scores_zero():
    for frame in (_frame_state(dz=-0.02), _frame_state(roll=math.radians(5))):
        prots = {n: _seated_protector(n, frame) for n in T.CORNERS}
        assert T.score(_objects(prots, frame)) == 0.0


def test_untouched_scene_scores_zero():
    assert T.score(_objects()) == 0.0


# -- the demo ------------------------------------------------------------------------------------------------------------

VARIANTS = ["full", "nothing", "no_hold", "keep_holding", "two_only"]


def _plans(variant, monkeypatch):
    monkeypatch.setenv("DEMO_VARIANT", variant)
    plan = T.demo()
    return {arm: ArmPlan(wps, station.BASES[arm], q_start=READY, finger_start=FINGER_OPEN) for arm, wps in plan.items()}


@pytest.mark.parametrize("variant", VARIANTS)
def test_demo_fits_the_run_limit(variant, monkeypatch):
    plans = _plans(variant, monkeypatch)
    assert max(p.duration for p in plans.values()) < RUN_LIMIT_S - 5.0


@pytest.mark.parametrize("variant", VARIANTS)
def test_demo_is_reachable(variant, monkeypatch):
    plans = _plans(variant, monkeypatch)
    for arm, plan in plans.items():
        t = 0.0
        while t <= plan.duration + 0.5:
            plan.action7(t)
            t += 1 / 20
        assert plan.worst_ik_mm < 2.0, (variant, arm, plan.worst_ik_mm)
        assert plan.worst_ik_deg < 2.0, (variant, arm, plan.worst_ik_deg)


@pytest.mark.parametrize("variant", VARIANTS)
def test_arms_keep_apart(variant, monkeypatch):
    plans = _plans(variant, monkeypatch)
    if len(plans) < 2:
        return
    end = max(p.duration for p in plans.values()) + 0.5
    t, worst = 0.0, 9.0
    while t <= end:
        a = plans["left"].target(t)[0]
        b = plans["right"].target(t)[0]
        worst = min(worst, float(np.linalg.norm(np.asarray(a) - np.asarray(b))))
        t += 0.05
    assert worst > 0.15, (variant, worst)


def test_full_demo_holds_the_frame_during_every_push(monkeypatch):
    """Whenever one arm is pushing a protector on, the other arm's fingertips are pressed on the frame's back."""
    plans = _plans("full", monkeypatch)
    z_press = T.FB + T.SLAB_Z[1] + T.TIP - T.PRESS
    for arm, other in (("left", "right"), ("right", "left")):
        t = 0.0
        while t <= plans[arm].duration:
            label = plans[arm].target(t)[3]
            if "push it on" in label or "back of the pocket" in label or "settle" in label:
                pos = plans[other].target(t)[0]
                assert pos[2] == pytest.approx(z_press, abs=0.004), (arm, t, label, pos)
            t += 0.1


def test_grip_rotation_is_straight_down():
    r = grip_rotation(0.0)
    assert np.allclose(r @ np.array([0.0, 0.0, 1.0]), [0.0, 0.0, -1.0], atol=1e-6)
    _ = ik  # imported for the reach tests' warm start


def _box_face_line(spec, axis, top):
    """The two ends (s, z) of a slope box's top (or bottom) face, along ``axis``, from its centre, size and rotation."""
    centre, size, _, q = spec
    w, x, y, z = q
    r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                  [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                  [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    i = 0 if axis == "x" else 1
    half = np.array(size) / 2
    ends = []
    for sgn in (-1, 1):
        local = np.zeros(3)
        local[i] = sgn * half[i]
        local[2] = half[2] if top else -half[2]
        p = np.array(centre) + r @ local
        ends.append((p[i], p[2]))
    return sorted(ends)


@pytest.mark.parametrize("axis", ["x", "y"])
def test_bevels_run_where_intended(axis):
    p, flat, h = T.POCKET, T.POCKET - T.RAMP_RUN, T.RAMP_RISE
    floor = T._slope(axis, -p, -h, -flat, 0.0, (-p, 0.0), 0.004, True, T.FOAM)
    (s0, z0), (s1, z1) = _box_face_line(floor, axis, top=True)
    assert (s0, z0, s1, z1) == pytest.approx((-p, -h, -flat, 0.0), abs=1e-9)
    roof = T._slope(axis, -p, T.ROOF + h, -flat, T.ROOF, (-p, 0.0), 0.004, False, T.FOAM)
    (s0, z0), (s1, z1) = _box_face_line(roof, axis, top=False)
    assert (s0, z0, s1, z1) == pytest.approx((-p, T.ROOF + h, -flat, T.ROOF), abs=1e-9)


def test_floor_rests_proud_in_the_part_frame():
    """partkit places slider boxes in the part's frame: at rest the floor's top is PROUD above the origin, and its
    bottom stays above the walls' bottom even fully pressed."""
    _, floor = T._protector_shapes()
    core = floor[0]
    top = core[0][2] + core[1][2] / 2
    assert top == pytest.approx(T.PROUD)
    bottom = min(c[2] - s[2] / 2 for c, s, *_ in floor[:2])
    assert bottom - T.FLOOR_TRAVEL > -T.LIP
