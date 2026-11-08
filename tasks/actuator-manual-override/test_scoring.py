"""Offline checks of the clutch rule and the grading (no simulator): python -m pytest -q test_scoring.py"""

import math

import task

DEG = math.radians


def test_dogs():
    assert task._crossed_dog(0.0, DEG(51))          # first dog at 50 deg clockwise
    assert not task._crossed_dog(0.0, DEG(49))
    assert task._crossed_dog(0.0, DEG(-71))         # or 70 deg anticlockwise
    assert not task._crossed_dog(DEG(51), DEG(169))  # next dog at 170
    assert task._crossed_dog(DEG(169), DEG(171))


def test_grades():
    full = 1.0 - task.SHUT_TOL / task.CLOSED
    cases = [
        (False, 1.0, 0.0),    # wheel spun but clutch never engaged
        (True, 0.0, 0.25),    # engaged only
        (True, 0.49, 0.25),
        (True, 0.5, 0.5),
        (True, full - 1e-4, 0.5),  # 5.1 degrees short of the stop
        (True, full, 1.0),
        (True, 1.0, 1.0),
    ]
    for engaged, fraction, expected in cases:
        assert task.grade(engaged, fraction) == expected, (engaged, fraction)


def test_fraction_needs_engagement():
    task.STATE.update(engaged=False, q0=0.0, q=10.0)
    assert task.closed_fraction() == 0.0
    task.STATE.update(engaged=True, q0=DEG(50), q=DEG(50) + task.CLOSED / 2)
    assert abs(task.closed_fraction() - 0.5) < 1e-9
