"""Checks of generator_sync.py's generator model and scoring, without the simulator. Needs yam_arena (the official
stack):

    python -m pytest -q test_generator_sync.py
"""

from pathlib import Path

from yam_arena.task_file import load

M = load(Path(__file__).with_name("generator_sync.py")).module
UP = {"close": 0.0, "raise": 0.0, "lower": 0.0}


def run(steps, **pressed):
    """Advance the model ``steps`` control steps with the named buttons held down."""
    sliders = {**UP, **{k: -0.008 for k, v in pressed.items() if v}}
    for _ in range(steps):
        M._advance(sliders)


def tap(name, times=1):
    for _ in range(times):
        run(3, **{name: True})
        run(3)


def run_until_pointer_near(deg, tol=3.0, limit=2000):
    for _ in range(limit):
        if abs(M._S["phase"] - deg) <= tol:
            return
        run(1)
    raise AssertionError(f"pointer never reached {deg}")


def test_start_and_untouched():
    M._reset()
    assert M._S["slip"] == M.SLIP0_HZ and M._S["phase"] == 180.0
    run(400)
    assert M.score(None) == 0.0 and not M.success(None)


def test_held_button_counts_once():
    M._reset()
    run(20, **{"raise": True})
    assert M._S["slip"] == M.SLIP0_HZ + M.SLIP_STEP_HZ


def test_raise_and_lower():
    M._reset()
    tap("raise", 5)
    assert abs(M._S["slip"] - 0.125) < 1e-9
    tap("lower", 2)
    assert abs(M._S["slip"] + 0.125) < 1e-9


def test_pointer_turns_clockwise_when_fast():
    M._reset()
    tap("raise", 5)
    before = M._S["phase"]
    run(10)  # 0.5 s at +0.125 Hz: +22.5 degrees
    assert abs((M._S["phase"] - before) - 22.5) < 1e-6


def test_close_in_sync():
    M._reset()
    tap("raise", 5)
    run_until_pointer_near(0.0)
    run(1, close=True)
    assert M.score(None) == 1.0 and M.success(None)


def test_close_latched():
    """Only the first press counts: a good press can't be spoiled, a bad one can't be fixed."""
    M._reset()
    tap("raise", 5)
    run_until_pointer_near(0.0)
    run(1, close=True)
    run(1)
    run_until_pointer_near(150.0)
    run(1, close=True)
    assert M.score(None) == 1.0
    M._reset()
    run(1, close=True)  # first press while fast
    tap("raise", 5)
    run_until_pointer_near(0.0)
    run(1, close=True)
    assert M.score(None) == 0.0


def test_close_while_fast_scores_zero_even_at_12():
    M._reset()
    run_until_pointer_near(0.0, tol=10.0)
    run(1, close=True)
    assert M.score(None) == 0.0


def test_half_credit_and_out_of_phase():
    M._reset()
    tap("raise", 5)
    run_until_pointer_near(25.0, tol=2.0)
    run(1, close=True)
    assert M.score(None) == 0.5 and not M.success(None)
    M._reset()
    tap("raise", 5)
    run_until_pointer_near(90.0)
    run(1, close=True)
    assert M.score(None) == 0.0


def test_speed_matched_but_never_closed():
    M._reset()
    tap("raise", 4)  # slip 0: the pointer stops
    run(100)
    assert M.score(None) == 0.25


def test_overshoot_too_fast_the_other_way():
    M._reset()
    tap("raise", 6)  # +0.25 Hz, above SLOW_HZ
    run_until_pointer_near(0.0, tol=5.0)
    run(1, close=True)
    assert M.score(None) == 0.0
