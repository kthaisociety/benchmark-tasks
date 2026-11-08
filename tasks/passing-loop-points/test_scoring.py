"""Exercise the passing-loop scorer on synthetic ball paths, including the ways to fake or skip a throw."""

import pytest

import layout as L
from scoring import Run, Sample

PARKED = ((0.20, 0.25, L.TOP + 0.2), (0.20, -0.25, L.TOP + 0.2))
Z = L.BED + L.BALL_R
N, R = L.LEVER_NORMAL, L.LEVER_REVERSE


def straight(y0, y1, x, n=40):
    return [(x, y0 + (y1 - y0) * i / n, Z) for i in range(n + 1)]


def diagonal(p0, p1, n=30):
    return [(p0[0] + (p1[0] - p0[0]) * i / n, p0[1] + (p1[1] - p0[1]) * i / n, Z) for i in range(n + 1)]


ON_MAIN = straight(0.33, 0.25, L.MAIN_X)
TO_A = straight(0.25, L.JUNCTION_A, L.MAIN_X)
ENTRY = diagonal((L.MAIN_X, L.JUNCTION_A), (L.LOOP_X, L.JUNCTION_A - 0.10))
LOOP = straight(L.JUNCTION_A - 0.10, L.JUNCTION_B, L.LOOP_X)
EXIT = diagonal((L.LOOP_X, L.JUNCTION_B), (L.MAIN_X, L.JUNCTION_B - 0.10))
HOME = straight(L.JUNCTION_B - 0.10, -0.30, L.MAIN_X)
PIT = straight(L.JUNCTION_B, L.PIT_END + 0.02, L.LOOP_X)
TO_BLOCKER = straight(L.JUNCTION_A, L.BLOCKER[1] + 0.045, L.MAIN_X)


def play(path, lever, tcp=PARKED):
    """lever(i, ball) gives the lever angle at each sample."""
    run = Run()
    for i, ball in enumerate(path):
        run.step(Sample(ball, lever(i, ball), tcp))
    return run


def throws(pull_at, restore_at):
    return lambda i, ball: R if pull_at <= i < restore_at else N


FULL_PATH = ON_MAIN + TO_A + ENTRY + LOOP + EXIT + HOME
LOOP_START = len(ON_MAIN + TO_A + ENTRY)


def test_both_throws_in_their_windows_score_full():
    run = play(FULL_PATH, throws(5, LOOP_START + 10))
    assert run.fault is None and run.merged and run.score == 1.0


def test_lever_left_pulled_sends_the_ball_to_the_sand_drag():
    run = play(ON_MAIN + TO_A + ENTRY + LOOP + PIT, throws(5, 10_000))
    assert run.in_pit and not run.merged and run.score == 0.5


def test_doing_nothing_runs_into_the_wagon():
    assert play(ON_MAIN + TO_A + TO_BLOCKER, lambda i, b: N).score == 0.0


@pytest.mark.parametrize("restore_at", [len(ON_MAIN + TO_A) + 2, len(ON_MAIN + TO_A) - 3])
def test_restoring_with_the_ball_on_junction_a_scores_zero(restore_at):
    run = play(FULL_PATH, throws(5, restore_at))
    assert run.fault and "junction A" in run.fault and run.score == 0.0


def test_restoring_with_the_ball_on_junction_b_scores_zero():
    restore = len(ON_MAIN + TO_A + ENTRY + LOOP) + 3
    run = play(FULL_PATH, throws(5, restore))
    assert run.fault and "junction B" in run.fault


def test_a_half_thrown_lever_on_a_junction_scores_zero():
    half = (N + R) / 2
    run = play(FULL_PATH, lambda i, b: half if i >= 5 else N)
    assert run.fault and run.score == 0.0


def test_gripper_over_the_track_scores_zero():
    tcp = (PARKED[0], (L.MAIN_X, 0.30, L.TOP + 0.05))
    run = play(FULL_PATH, throws(5, LOOP_START + 10), tcp=tcp)
    assert run.fault == "gripper over the track" and run.score == 0.0


def test_lifting_the_ball_off_the_track_scores_zero():
    path = FULL_PATH[:60] + [(x, y, z + 0.05) for x, y, z in FULL_PATH[60:]]
    assert play(path, throws(5, LOOP_START + 10)).score == 0.0


def test_ball_carried_round_to_the_exit_without_the_loop_scores_zero():
    # Any route to the exit other than through the loop leaves entered_loop unset.
    path = ON_MAIN + straight(0.25, -0.30, L.MAIN_X)
    assert play(path, lambda i, b: N).score == 0.0


def test_result_is_final_once_merged():
    run = play(FULL_PATH, throws(5, LOOP_START + 10))
    run.step(Sample((L.MAIN_X, -0.30, Z + 0.1), N, PARKED))
    assert run.score == 1.0
