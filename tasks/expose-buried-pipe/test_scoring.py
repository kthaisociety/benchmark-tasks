"""Offline checks of the buried-pipe scorer and the scripted demo's reach. No Isaac Sim needed."""

import numpy as np
import pytest

import scoring
import task

UP = (0.0, 0.0, 0.0, 1.0)
FILL = np.asarray(task.FILL)


def spade_at_edge(edge):
    """The spade's own pose (grasp point) that puts the upright blade's edge at ``edge``."""
    return (edge[0], edge[1], edge[2] - task.EDGE_Z)


def judge_run(spade_poses, balls, steps=None):
    judge = scoring.Judge(control_hz=20.0)
    out = None
    for i in range(steps or len(spade_poses)):
        out = judge.step(spade_poses[min(i, len(spade_poses) - 1)], balls, np.zeros(len(balls)))
    return judge, out


def bare_pipe(keep=lambda b: False):
    """The starting fill with every ball over the pipe's top removed unless ``keep`` says otherwise."""
    over = (FILL[:, 2] > task.CROWN_Z) & (np.abs(FILL[:, 1] - task.CHANNEL_Y) < 0.012)
    return np.array([b for b, o in zip(FILL, over) if not o or keep(b)])


def test_starting_layout_is_covered_and_the_spade_is_clear():
    assert len(FILL) == 156
    assert scoring.bare_length(FILL) == 0.0
    assert scoring.spade_gap(task.SPADE_HOME, UP) > 0.05
    gaps = scoring.pipe_gap(FILL)
    assert gaps.min() > task.SOIL_R


def test_uncovered_pipe_scores_full_after_a_still_second():
    balls = bare_pipe()
    assert scoring.bare_length(balls) == pytest.approx(0.100)
    judge, out = judge_run([(task.SPADE_HOME, UP)], balls, steps=19)
    assert out["score"] == 0.0  # not yet held for 1 s
    out = judge.step((task.SPADE_HOME, UP), balls, np.zeros(len(balls)))
    assert out["score"] == 1.0 and out["success"]


@pytest.mark.parametrize("bare_mm, expected", [(100, 1.0), (80, 1.0), (75, 0.5), (40, 0.5), (35, 0.25), (10, 0.25), (5, 0.0), (0, 0.0)])
def test_partial_credit_by_bare_length(bare_mm, expected):
    edge = task.MARKS_X[0] + bare_mm / 1000.0
    balls = bare_pipe(keep=lambda b: b[0] > edge + task.SOIL_R)
    covered_from = edge
    # put one ball on the crown every 12.7 mm beyond the bare stretch so the rest stays covered
    extra = [(x, task.CHANNEL_Y, task.CROWN_Z + task.SOIL_R) for x in np.arange(covered_from + task.SOIL_R, task.MARKS_X[1] + 0.006, 0.010)]
    if extra:
        balls = np.vstack([balls, extra])
    assert scoring.bare_length(balls) == pytest.approx(bare_mm / 1000.0, abs=0.0051)
    _, out = judge_run([(task.SPADE_HOME, UP)], balls, steps=25)
    assert out["score"] == expected


def test_a_ball_beside_the_pipe_does_not_cover_it_but_one_on_top_does():
    beside = np.array([[0.28, task.CHANNEL_Y + 0.0195, task.CROWN_Z + 0.004]])
    assert scoring.bare_length(beside) == pytest.approx(0.100)
    on_top = np.array([[0.28, task.CHANNEL_Y + 0.004, task.CROWN_Z + 0.004]])
    assert scoring.bare_length(on_top) == pytest.approx(0.100 - 0.010)


def test_any_touch_of_the_pipe_zeroes_the_trial_even_after_partial_credit():
    balls = bare_pipe()
    judge, out = judge_run([(task.SPADE_HOME, UP)], balls, steps=10)
    touch = spade_at_edge((0.30, task.CHANNEL_Y, task.CROWN_Z + 0.0005))
    out = judge.step((touch, UP), balls, np.zeros(len(balls)))
    assert out["struck"]
    for _ in range(40):
        out = judge.step((task.SPADE_HOME, UP), balls, np.zeros(len(balls)))
    assert out["score"] == 0.0


def test_skimming_two_millimetres_over_the_pipe_is_not_a_strike():
    near = spade_at_edge((0.30, task.CHANNEL_Y, task.CROWN_Z + 0.002))
    assert scoring.spade_gap(near, UP) == pytest.approx(0.002, abs=2e-4)
    judge, out = judge_run([(near, UP)], FILL, steps=5)
    assert not out["struck"]


def test_a_fast_swipe_through_the_pipe_between_control_steps_is_caught():
    above = spade_at_edge((0.30, task.CHANNEL_Y + 0.0, task.CROWN_Z + 0.030))
    below = spade_at_edge((0.30, task.CHANNEL_Y + 0.0, task.CROWN_Z - 0.030))
    assert scoring.spade_gap(above, UP) > 0.02
    judge = scoring.Judge()
    judge.step((above, UP), FILL, np.zeros(len(FILL)))
    out = judge.step((below, UP), FILL, np.zeros(len(FILL)))
    assert out["struck"]


def test_moving_soil_does_not_latch_credit():
    balls = bare_pipe()
    judge = scoring.Judge()
    speeds = np.full(len(balls), 0.05)
    for _ in range(40):
        out = judge.step((task.SPADE_HOME, UP), balls, speeds)
    assert out["score"] == 0.0 and not out["still"]


def test_demo_stays_within_reach_and_clear_of_the_pipe():
    import run
    from yam_arena import station
    from yam_arena.demo import ArmPlan

    run.install_tilt()
    plan = ArmPlan(task.demo()["right"], station.BASES["right"])
    t, closest = 0.0, np.inf
    while t < plan.duration:
        plan.action7(t)
        pos, rot, _, _ = plan.target(t)
        from yam_arena.demo import grip_rotation

        tilt = rot @ np.linalg.inv(grip_rotation(task.YAW))
        quat = _matrix_to_xyzw(tilt)
        held = np.asarray(pos) - np.array([0.0, 0.0, task.GRIP_ABOVE])  # the spade hangs below the TCP
        closest = min(closest, scoring.spade_gap(held, quat))
        t += 0.05
    assert plan.worst_ik_mm < 1.0 and plan.worst_ik_deg < 0.5
    assert plan.duration * 20 < 3000
    assert 0.004 < closest < 0.007  # the plan's closest approach: 5 mm over the pipe


def _matrix_to_xyzw(r):
    from yam_arena import ik

    return ik.matrix_to_quat_xyzw(r)
