"""Offline checks for take-one-cup-from-dispenser: the geometry, the scoring rules on hand-built states, and the
scripted demo's reach and clearances (inverse kinematics and the robot's meshes only, no Isaac Sim). Run from this
folder with yam_arena importable:

    PYTHONPATH=<path to yam-isaac-lab-arena> python -m pytest test_task.py -q
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import subprocess
import sys

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import task as T  # noqa: E402
from partkit import PartState, slider_axis, slider_frame  # noqa: E402
from yam_arena import station, task_file  # noqa: E402
from yam_arena.demo import ArmPlan, grip_rotation  # noqa: E402

RUN_LIMIT_STEPS = 1200
GRIPPER_SPAN = 0.094  # pads 94 mm apart fully open
STACK_WEIGHT = (T.N_CUPS - 1) * T.CUP_MASS * 9.81  # what the flaps carry once the bottom cup is gone (N)


def _quat_xyzw(tilt_deg=0.0, about="x"):
    h = math.radians(tilt_deg) / 2
    if about == "x":
        return np.array([math.sin(h), 0.0, 0.0, math.cos(h)])
    return np.array([0.0, math.sin(h), 0.0, math.cos(h)])


def _cup(pos, tilt=0.0, vel=0.0):
    return PartState(pos=np.asarray(pos, float), quat_xyzw=_quat_xyzw(tilt), lin_vel=np.array([vel, 0.0, 0.0]),
                     ang_vel=np.zeros(3), low=np.zeros(3), high=np.zeros(3), sliders={})


def _hanging(k):
    return _cup((T.AXIS_XY[0], T.AXIS_XY[1], T.CUP1_BASE_Z + (k - 1) * T.NEST_PITCH))


def _after(k):
    """Cup k once the bottom cup is gone: everything one pitch lower."""
    return _cup((T.AXIS_XY[0], T.AXIS_XY[1], T.CUP1_BASE_Z + (k - 2) * T.NEST_PITCH))


def _objects(first=None, rest=None):
    out = {T.CUPS[0].name: first if first is not None else _hanging(1)}
    for k in range(2, T.N_CUPS + 1):
        out[T.cup_name(k)] = (rest or {}).get(k, _after(k) if first is not None else _hanging(k))
    return out


ON_MAT = (T.MAT_XY[0], T.MAT_XY[1], T.MAT_TOP)
FAR = [np.array([0.18, 0.34, T.TOP + 0.16]), np.array([0.18, -0.34, T.TOP + 0.16])]


@pytest.fixture(autouse=True)
def _fresh_state():
    T._reset()
    T._S["tcps"] = [t.copy() for t in FAR]
    yield
    T._reset()


# -- loading ---------------------------------------------------------------------------------------------------------


def test_task_file_loads():
    tf = task_file.load(HERE / "task.py")
    names = [o.name for o in tf.objects]
    assert names == ["dispenser", "gasket", "cup1", "cup2", "cup3", "cup4", "mat"]
    assert not set(names) & task_file.RESERVED_NAMES
    assert tf.score is not None and tf.success is not None and tf.demo is not None


def test_object_poses_are_plain_floats():
    for o in T.OBJECTS:
        assert all(type(v) is float for v in o.pos), o.name


def test_task_json_matches_and_fits():
    data = json.loads((HERE / "task.json").read_text())
    for key in ("title", "category", "instruction", "setup", "scoring", "run_limit", "fidelity", "scores"):
        assert data.get(key), key
    assert data["instruction"] == T.INSTRUCTION
    assert data["category"] in ("Impact on jobs", "Impact on AI timelines", "Safety and alignment",
                                "Societal relatability", "Other")
    assert len(data["fidelity"]) <= 500
    assert "1,200 steps" in data["run_limit"]


# -- the cup ----------------------------------------------------------------------------------------------------------


def test_cup_is_a_12oz_hot_cup():
    assert T.CUP_H == pytest.approx(0.110)
    assert 2 * (T.BEAD_RC + T.BEAD_R) == pytest.approx(T.RIM_D)
    assert 2 * T.R0 == pytest.approx(T.BASE_D)


def test_rim_stands_proud_of_the_wall():
    proud = T.BEAD_RC + T.BEAD_R - T.wall_radius(T.BEAD_ZC)
    assert 0.0015 <= proud <= 0.003


def test_cup_fits_the_open_gripper_where_it_is_pinched():
    width = 2 * T.R_GRIP / math.cos(math.pi / T.WALL_SEGS)  # across the facets' corners
    assert width < GRIPPER_SPAN - 0.02
    # the still finger meets the cup's -y facet when the TCP is STILL_PAD - r_grip to its +y
    tcp = T.tcp_for_cup(T.AXIS_XY, T.CUP1_BASE_Z)
    assert tcp[1] - T.STILL_PAD == pytest.approx(T.AXIS_XY[1] - T.R_GRIP, abs=1e-9)


def test_pinch_is_on_the_cup_and_near_the_pad_tips():
    a = T.approach_dir()
    tcp = np.array(T.tcp_for_cup(T.AXIS_XY, T.CUP1_BASE_Z))
    contact = tcp + T.GRIP_FORWARD * a  # where the pads meet the cup's sides
    assert contact[0] == pytest.approx(T.AXIS_XY[0])
    assert contact[2] - T.CUP1_BASE_Z == pytest.approx(T.GRIP_UP)
    assert 0.005 <= T.GRIP_FORWARD <= 0.013 + 0.003  # the pads reach 13 mm past the TCP


def test_nested_cups_clear_each_other():
    gap = T.SLOPE * T.NEST_PITCH - T.WALL_T
    assert 0.0005 <= gap <= 0.002
    # each stacking bump's top carries the next cup's bottom edge one pitch up
    _, lugs, _ = T._cup_shapes()
    for centre, r, _ in lugs:
        assert centre[2] + r == pytest.approx(T.NEST_PITCH)
        assert T.R0 - T.WALL_T <= math.hypot(centre[0], centre[1]) <= T.R0


def test_no_collider_is_turned():
    for o in T.OBJECTS:
        for b in getattr(o, "boxes", ()):
            assert len(b) < 4 or b[3] in (None, (1.0, 0.0, 0.0, 0.0)), o.name


def test_hulls_are_prisms():
    for o in T.OBJECTS:
        shapes = list(getattr(o, "hulls", ()))
        for s in getattr(o, "sliders", ()):
            shapes += list(s.hulls)
        for h in shapes:
            assert len(h[0]) == len(h[1]) >= 3


# -- the dispenser and its gasket ---------------------------------------------------------------------------------------


def test_flaps_hold_a_rim_but_not_the_wall_below_it():
    rim_out = T.BEAD_RC + T.BEAD_R
    assert rim_out - T.APERTURE >= 0.002  # the flaps reach 2 mm or more inside the rim
    below_rim = T.wall_radius(T.BEAD_ZC - T.BEAD_R - 0.003)
    assert below_rim <= T.APERTURE + 0.0003  # ... but don't squeeze the wall under it


def test_flap_travel_lets_the_rim_through():
    out = T.FLAP_TRAVEL * math.cos(T.SLIDE_DOWN)
    assert T.FLAP_RC + out >= T.BEAD_RC + T.BEAD_R + T.FLAP_TIP_R + 0.0005


def test_flaps_stay_clear_of_the_tube_and_lip():
    out = T.FLAP_TRAVEL * math.cos(T.SLIDE_DOWN)
    down = T.FLAP_TRAVEL * math.sin(T.SLIDE_DOWN)
    assert T.FLAP_RC + out + T.FLAP_TIP_R < T.TUBE_IN
    assert T.FLAP_Z - down - T.FLAP_TIP_R > 0.0  # above the lip's top face


def test_flap_springs_hold_the_stack_and_let_a_pull_through():
    # the force a flap pushes back with once the rim is past its tip, along its slide; the rim's slope turns about
    # half of the pull into that push
    out_needed = T.BEAD_RC + T.BEAD_R + T.FLAP_TIP_R - T.FLAP_RC
    spring = T.FLAP_RATE * out_needed / math.cos(T.SLIDE_DOWN)
    total = T.FLAPS * spring
    assert total >= 2.0 * STACK_WEIGHT  # the stack can't push through on its own
    assert total <= 6.0  # a gentle pull: a few newtons


def test_flap_slides_point_inward_and_up():
    for i, s in enumerate(T.GASKET.sliders):
        d = slider_axis(s)
        home = T.flap_home(i)
        assert d[2] == pytest.approx(math.sin(T.SLIDE_DOWN))
        assert np.dot(d[:2], home[:2]) < 0  # toward the axis


def test_slider_frame_turns_z_onto_the_direction():
    for s in T.GASKET.sliders:
        w, x, y, z = slider_frame(s)
        r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                      [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                      [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
        assert r @ np.array([0, 0, 1.0]) == pytest.approx(slider_axis(s), abs=1e-9)


def test_rim_clears_the_bezel_and_lip():
    rim_out = T.BEAD_RC + T.BEAD_R
    assert T.BEZEL_IN - rim_out >= 0.002
    assert T.LIP_IN - rim_out >= 0.001
    assert T.BEZEL_OUT < T.TUBE_IN


def test_bottom_cup_hangs_by_its_rim_on_the_flaps():
    rim_centre = T.CUP1_BASE_Z + T.BEAD_ZC
    tip = T.OUTLET_Z + T.FLAP_Z
    assert math.hypot(T.FLAP_RC - T.BEAD_RC, rim_centre - tip) == pytest.approx(T.FLAP_TIP_R + T.BEAD_R)
    assert rim_centre > tip


def test_bottom_cup_hangs_well_out_and_above_the_counter():
    exposed = T.DISPENSER_BOTTOM - T.CUP1_BASE_Z
    assert exposed >= 0.085
    assert T.CUP1_BASE_Z - T.TOP >= 0.05


def test_pull_brings_the_whole_cup_out_above_the_counter():
    rim_after = T.CUP1_BASE_Z - T.PULL + T.CUP_H
    assert T.DISPENSER_BOTTOM - rim_after >= 0.008
    assert T.CUP1_BASE_Z - T.PULL - T.PULL_OVER - T.TOP >= 0.015


def test_next_cup_lands_after_the_rim_has_passed():
    # the bottom cup's rim is past the flaps (its centre a tip-plus-rim below theirs) before the next rim arrives
    past = 2 * (T.FLAP_TIP_R + T.BEAD_R)
    assert T.NEST_PITCH > past + 0.004


def test_stack_fits_in_the_tube():
    top = T.CUP1_BASE_Z + (T.N_CUPS - 1) * T.NEST_PITCH + T.CUP_H
    assert top < T.OUTLET_Z + T.TUBE_L - 0.05


def test_wall_and_brackets_stay_out_of_the_tube():
    for centre, size, _ in [b[:3] for b in T.DISPENSER.boxes]:
        assert centre[0] - size[0] / 2 >= T.TUBE_IN, centre  # brackets and wall start at the tube's back face or beyond
    assert T.WALL_X > T.AXIS_XY[0] + T.TUBE_OUT


def test_mat_is_clear_of_the_dispenser_and_wall():
    mat_near_y = T.MAT_XY[1] - T.MAT_SIZE[1] / 2
    assert mat_near_y - (T.AXIS_XY[1] + T.TUBE_OUT) >= 0.04
    assert T.MAT_XY[0] + T.MAT_SIZE[0] / 2 < T.WALL_X - 0.005


def test_dispenser_is_clear_of_the_left_arm_ready_pose():
    from yam_arena import ik
    from yam_arena.constants import READY

    tcp, _ = ik.tcp_pose(np.array(READY), station.BASES["left"])
    assert math.hypot(tcp[0] - T.AXIS_XY[0], tcp[1] - T.AXIS_XY[1]) > T.TUBE_OUT + 0.08


# -- scoring -----------------------------------------------------------------------------------------------------------


def test_untouched_scene_scores_zero():
    objs = _objects()
    assert all(T.is_held(o) for o in objs.values())
    assert T.score(objs) == 0.0


def test_cup_on_the_mat_let_go_scores_full():
    objs = _objects(first=_cup(ON_MAT))
    assert T.score(objs) == 1.0 and T.success(objs)


def test_cup_on_the_counter_scores_half():
    objs = _objects(first=_cup((T.COUNTER_SPOT[0], T.COUNTER_SPOT[1], T.TOP)))
    assert T.standing_on(objs["cup1"]) == "counter"
    assert T.score(objs) == 0.5


def test_cup_overhanging_the_mat_edge_scores_half():
    edge = T.MAT_XY[1] + T.MAT_SIZE[1] / 2 - T.R0 / 2  # centre on the mat, part of the bottom off it
    objs = _objects(first=_cup((T.MAT_XY[0], edge, T.MAT_TOP)))
    assert T.standing_on(objs["cup1"]) == "mat" and not T.wholly_on_mat(objs["cup1"])
    assert T.score(objs) == 0.5


def test_cup_held_in_the_air_scores_quarter():
    objs = _objects(first=_cup((0.32, 0.2, T.TOP + 0.03)))
    assert T.is_out(objs["cup1"])
    assert T.score(objs) == 0.25


def test_cup_still_held_by_the_gripper_scores_quarter():
    T._S["tcps"] = [np.array(T.tcp_for_cup(T.MAT_XY, T.MAT_TOP)), FAR[1]]
    objs = _objects(first=_cup(ON_MAT))
    assert not T.released(objs["cup1"])
    assert T.score(objs) == 0.25


def test_backing_away_is_not_yet_let_go_but_parking_is():
    tcp = np.array(T.tcp_for_cup(T.MAT_XY, T.MAT_TOP)) - np.array([0, T.STILL_CLEAR, 0]) - T.RETREAT * T.approach_dir()
    T._S["tcps"] = [tcp, FAR[1]]
    assert not T.released(_cup(ON_MAT))  # 6 cm back along the approach, the fingers are still within 1.5 cm
    T._S["tcps"] = [np.array(T.PARK["left"]), FAR[1]]
    assert T.released(_cup(ON_MAT))


def test_moving_cup_scores_quarter():
    objs = _objects(first=_cup(ON_MAT, vel=0.05))
    assert T.score(objs) == 0.25


def test_leaning_cup_does_not_stand():
    objs = _objects(first=_cup(ON_MAT, tilt=15.0))
    assert T.standing_on(objs["cup1"]) is None
    assert T.score(objs) == 0.25


def test_two_cups_out_scores_zero():
    objs = _objects(first=_cup(ON_MAT), rest={2: _cup((0.30, 0.2, T.TOP))})
    assert T.score(objs) == 0.0


def test_next_cup_dragged_into_the_gasket_caps_at_quarter():
    jammed = _cup((T.AXIS_XY[0], T.AXIS_XY[1], T.CUP1_BASE_Z - 0.012))  # rim between the flaps and the bottom edge
    assert not T.is_out(jammed) and not T.is_held(jammed)
    objs = _objects(first=_cup(ON_MAT), rest={2: jammed})
    assert T.score(objs) == 0.25


def test_a_cup_that_ever_tipped_scores_zero():
    T._S["tipped"] = True
    assert T.score(_objects(first=_cup(ON_MAT))) == 0.0


def test_a_cup_on_the_floor_scores_zero():
    T._S["fell"] = True
    assert T.score(_objects(first=_cup(ON_MAT))) == 0.0


def test_out_means_the_whole_rim_is_below_the_dispenser():
    just_in = _cup((0.36, 0.14, T.DISPENSER_BOTTOM - T.CUP_H + 0.001))
    just_out = _cup((0.36, 0.14, T.DISPENSER_BOTTOM - T.CUP_H - 0.001))
    assert not T.is_out(just_in) and T.is_out(just_out)


def test_cup_distance():
    c = _cup(ON_MAT)
    assert T.cup_distance(c, ON_MAT + np.array([0, 0, 0.05])) == 0.0
    assert T.cup_distance(c, ON_MAT + np.array([0, T.RIM_D / 2 + 0.02, 0.05])) == pytest.approx(0.02)
    assert T.cup_distance(c, ON_MAT + np.array([0, 0, T.CUP_H + 0.03])) == pytest.approx(0.03)


# -- the demo ------------------------------------------------------------------------------------------------------------


VARIANTS = ("full", "nothing", "pull_only", "counter", "keep_holding")


def _plans(variant, monkeypatch):
    monkeypatch.setenv("DEMO_VARIANT", variant)
    return {arm: ArmPlan(wps, station.BASES[arm]) for arm, wps in T.demo().items()}


@pytest.mark.parametrize("variant", ("full", "counter", "nothing"))
def test_demo_fits_the_run_limit(variant, monkeypatch):
    for plan in _plans(variant, monkeypatch).values():
        assert plan.duration < RUN_LIMIT_STEPS / 20 - 15  # with room for settling and the 1 s success hold


@pytest.mark.parametrize("variant", VARIANTS)
def test_demo_is_reachable(variant, monkeypatch):
    for arm, plan in _plans(variant, monkeypatch).items():
        t = 0.0
        while t < min(plan.duration, 30.0) + 0.05:
            plan.action7(t)
            t += 0.05
        assert plan.worst_ik_mm < 2.0, (arm, plan.worst_ik_mm)
        assert plan.worst_ik_deg < 1.0, (arm, plan.worst_ik_deg)


def test_tilt_is_a_float_and_carries_its_pitch():
    g = T.Tilt(0.3, 0.5)
    assert float(g) == 0.3 and g.pitch == 0.5 and isinstance(g, float)


def test_tilted_grip_points_forward_and_down():
    r = grip_rotation(T.GRIP)
    assert r[:, 2] == pytest.approx(T.approach_dir(), abs=1e-9)
    assert r[:, 0] == pytest.approx([0.0, -1.0, 0.0], abs=1e-9)  # the fingers still close along world y


def test_plain_yaw_grip_is_unchanged():
    assert grip_rotation(0.0) == pytest.approx(T._demo._untilted_grip_rotation(0.0))
    assert grip_rotation(0.4) == pytest.approx(T._demo._untilted_grip_rotation(0.4))


def test_demo_pulls_straight_down():
    wps = {w.label: w for w in T._take("mat")}
    pinch, pulled = np.array(wps["pinch"].pos), np.array(wps["pull straight down through the gasket"].pos)
    d = pulled - pinch
    assert d[0] == pytest.approx(0.0) and d[1] == pytest.approx(0.0)
    assert -d[2] == pytest.approx(T.PULL + T.PULL_OVER)


def test_demo_sets_the_cup_wholly_on_the_mat():
    wps = {w.label: w for w in T._take("mat")}
    tcp = np.array(wps["set it down on the mat"].pos)
    a = T.approach_dir()
    axis = tcp + T.GRIP_FORWARD * a - np.array([0.0, T.STILL_PAD - T.R_GRIP, 0.0])
    cup = _cup((axis[0], axis[1], T.MAT_TOP))
    assert T.wholly_on_mat(cup)


def test_mesh_clearances():
    """The robot's meshes along the full demo stay clear of the dispenser, wall, counter and the hanging cup."""
    env = dict(os.environ, PYTHONPATH=os.pathsep.join(sys.path))
    out = subprocess.run([sys.executable, str(HERE / "check_clearance.py"), "full"], capture_output=True, text=True,
                         env=env, timeout=600, check=True).stdout
    lines = [ln for ln in out.splitlines() if ln.strip().startswith(("OK", "LOW", "HIT"))]
    bad = [ln for ln in lines if not ln.strip().startswith("OK") and "pinched" not in ln]
    assert lines and not bad, out
