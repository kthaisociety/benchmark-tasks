"""Offline checks of the scorer, no simulator: python -m pytest -q test_scoring.py"""

import math
from types import SimpleNamespace

import numpy as np

import chair as C
import layout as L
import scoring as S

FAR = (0.30, -0.27, L.TOP + 0.20)


def state(pos, pitch_deg=0.0, roll_deg=0.0, still=True):
    """A pose from the axle-middle position and a tip back (about y, front up positive) then roll."""
    a, b = math.radians(-pitch_deg) / 2, math.radians(roll_deg) / 2
    qy = np.array((0.0, math.sin(a), 0.0, math.cos(a)))
    qx = np.array((math.sin(b), 0.0, 0.0, math.cos(b)))
    x1, y1, z1, w1 = qy
    x2, y2, z2, w2 = qx
    q = np.array((w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                  w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2))
    v = np.zeros(3) if still else np.array((0.05, 0.0, 0.0))
    return SimpleNamespace(pos=np.asarray(pos, float), quat_xyzw=q, lin_vel=v, ang_vel=np.zeros(3))


def scene(chair, dummy_offset=(0.0, 0.0, 0.0), right=FAR):
    R, p = S.frame({"chair": chair})
    d = p + R @ np.array((L.DUMMY["x"], 0.0, C.SEAT["top"] + L.DUMMY["h"] / 2)) + np.asarray(dummy_offset)
    dummy = SimpleNamespace(pos=d, quat_xyzw=chair.quat_xyzw, lin_vel=np.zeros(3), ang_vel=np.zeros(3))
    tcp = lambda x: SimpleNamespace(pos=np.asarray(x, float))  # noqa: E731
    return {"chair": chair, "dummy": dummy, "left_tcp": tcp((0.30, 0.27, L.TOP + 0.20)), "right_tcp": tcp(right)}


def run(objects, steps=1):
    t = S.Trial()
    out = [t.step(objects) for _ in range(steps)]
    return out[-1], t.fault


START = (L.AXLE0[0], 0.0, L.TOP)
# rear wheels on the road, casters on the kerb: the chair pitches up by the kerb height over the wheelbase
CASTERS_UP_X = L.KERB_X0 + 0.03 - C.CASTER["x"]
CASTERS_UP = math.degrees(math.asin(L.KERB_H / C.CASTER["x"]))
ALL_UP = (L.KERB_X0 + C.REAR["r"] + 0.02, 0.0, L.KERB_TOP)


def test_start_is_zero_and_clean():
    assert run(scene(state(START))) == (0.0, "")


def test_casters_up_is_a_quarter():
    assert run(scene(state((CASTERS_UP_X, 0.0, L.TOP), CASTERS_UP))) == (0.25, "")


def test_all_up_but_held_is_point_six():
    near_bar = (ALL_UP[0] + C.BAR_BACK - 0.01, 0.0, L.KERB_TOP + C.BAR_Z)
    assert run(scene(state(ALL_UP), right=near_bar), steps=40) == (0.6, "")


def test_all_up_needs_a_second_at_rest():
    assert run(scene(state(ALL_UP)), steps=19)[0] == 0.6
    assert run(scene(state(ALL_UP)), steps=20) == (1.0, "")
    assert run(scene(state(ALL_UP, still=False)), steps=40)[0] == 0.6


def test_rear_wheels_at_the_edge_are_not_up():
    at_edge = (L.KERB_X0 + 0.005, 0.0, L.KERB_TOP)
    assert run(scene(state(at_edge)), steps=40)[0] == 0.25


def test_lifting_the_chair_clear_is_a_fault():
    assert run(scene(state((START[0], 0.0, L.TOP + 0.02))))[1] == "lifted clear of the ground"
    # set down on the kerb afterwards: still 0
    t = S.Trial()
    t.step(scene(state((START[0], 0.0, L.TOP + 0.02))))
    assert max(t.step(scene(state(ALL_UP))) for _ in range(40)) == 0.0


def test_tipping_and_dummy_faults():
    assert run(scene(state(START, 30.0)))[1] == "tipped back past 25 degrees"
    assert run(scene(state(START, -20.0)))[1] == "tipped forward past 15 degrees"
    assert run(scene(state(START, 0.0, 15.0)))[1] == "tipped sideways past 12 degrees"
    assert run(scene(state(START), dummy_offset=(0.08, 0.0, -0.05)))[1] == "dummy off the seat"


def test_normal_tip_is_allowed():
    # tipped back 12 degrees on the rear wheels: the rear wheels stay on the road, no fault
    assert run(scene(state(START, 12.0))) == (0.0, "")
