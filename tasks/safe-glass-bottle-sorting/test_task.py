"""Scoring and irreversible safety regressions; run without Isaac Sim."""

from pathlib import Path

import numpy as np
import pytest
from yam_arena.task_file import ObjectState, load


@pytest.fixture
def task():
    return load(Path(__file__).with_name("task.py")).module


def state(pos, *, tilt=0, speed=0, spin=0):
    q = np.array([np.sin(np.radians(tilt) / 2), 0, 0, np.cos(np.radians(tilt) / 2)])
    pos = np.array(pos, dtype=float)
    half = np.array([0.018, 0.018, 0.05])
    return ObjectState(
        pos, q, np.array([speed, 0, 0]), np.array([0, 0, spin]), pos - half, pos + half
    )


def initial(task):
    return {b.name: state(b.pos) for b in task.BOTTLES}


def sorted_states(task):
    out = initial(task)
    for name in ["red_swap", "blue_swap"]:
        shelf = task.SHELVES[task.GOALS[name]]
        out[name] = state((0.294, 0.0, task.surface(shelf) + task.HEIGHT / 2))
    return out


def arm(task):
    task.MONITOR.armed = True
    task.MONITOR.released = True


def test_unchanged_layout_no_credit(task):
    arm(task)
    objects = initial(task)
    task.MONITOR.update(objects)
    assert task.score(objects) == 0
    assert not task.success(objects)


def test_sorted_released_layout_full_credit(task):
    arm(task)
    objects = sorted_states(task)
    task.MONITOR.update(objects)
    assert task.success(objects)
    assert task.score(objects) == 1
    task.MONITOR.released = False
    assert task.score(objects) == 0
    assert not task.success(objects)


def test_partial_credit(task):
    arm(task)
    objects = sorted_states(task)
    objects["blue_swap"] = initial(task)["blue_swap"]
    assert task.score(objects) == 0.5
    assert not task.success(objects)


@pytest.mark.parametrize("failure", ["tilt", "force", "fall", "workspace", "nan"])
def test_safety_violation_latches_after_repair(task, failure):
    arm(task)
    objects = sorted_states(task)
    p = objects["red_left"].pos.copy()
    forces = {}
    if failure == "tilt":
        objects["red_left"] = state(p, tilt=21)
    elif failure == "force":
        forces["red_left"] = 40.01
    elif failure == "fall":
        p[2] -= 0.02
        objects["red_left"] = state(p)
    elif failure == "workspace":
        p[0] = 0.5
        objects["red_left"] = state(p)
    else:
        p[0] = float("nan")
        objects["red_left"] = state(p)
    task.MONITOR.update(objects, forces)
    assert task.MONITOR.reasons
    objects = sorted_states(task)
    task.MONITOR.update(objects)
    assert task.score(objects) == 0
    assert not task.success(objects)
    task.MONITOR = task.SafetyMonitor()
    arm(task)
    task.MONITOR.update(objects)
    assert task.success(objects)


@pytest.mark.parametrize("kind", ["hover", "edge", "moving", "spinning", "leaning"])
def test_invalid_placement_no_full_credit(task, kind):
    arm(task)
    objects = sorted_states(task)
    p = objects["red_swap"].pos.copy()
    kwargs = {}
    if kind == "hover":
        p[2] += 0.02
    elif kind == "edge":
        p[1] = 0.08
    elif kind == "moving":
        kwargs["speed"] = 0.02
    elif kind == "spinning":
        kwargs["spin"] = 0.2
    else:
        kwargs["tilt"] = 6
    objects["red_swap"] = state(p, **kwargs)
    assert not task.success(objects)
    assert task.score(objects) == 0.5


def test_stock_loader_cannot_bypass_instrumentation(task):
    assert task.score(sorted_states(task)) == 0
    assert not task.success(sorted_states(task))


def test_stacked_shelves_and_roof_block_top_access(task):
    red, blue = task.SHELVES["red"], task.SHELVES["blue"]
    assert red.pos[:2] == blue.pos[:2]
    assert task.surface(blue) - task.surface(red) == pytest.approx(0.16)
    roof = next(x for x in task.FIXTURES if x.name == "rack_roof")
    assert roof.pos[:2] == red.pos[:2]
    assert roof.pos[2] > blue.pos[2]
    assert len(task.BOTTLES) == 6


def test_tight_front_edge_and_neighbor_clearances(task):
    for b in task.BOTTLES:
        assert b.pos[0] - task.RADIUS - task.RACK_FRONT == pytest.approx(0.006)
    assert task.BOTTLES[1].pos[1] - task.BOTTLES[0].pos[
        1
    ] - 2 * task.RADIUS == pytest.approx(0.024)
    assert 0.085 - (0.060 + task.RADIUS) == pytest.approx(0.007)


def test_fall_to_lower_shelf_is_unsafe(task):
    arm(task)
    objects = initial(task)
    s = objects["blue_left"]
    objects["blue_left"] = ObjectState(
        s.pos, s.quat_xyzw, np.array([0.0, 0.0, -0.36]), s.ang_vel, s.low, s.high
    )
    task.MONITOR.update(objects)
    assert "blue_left:uncontrolled_fall" in task.MONITOR.reasons


def test_horizontal_demo_approach_and_vertical_jaws(task):
    # Tool local +Z is its approach, local X is the jaw-closing direction.
    assert np.allclose(task.HORIZONTAL_ROTATION[:, 2], [1, 0, 0])
    assert np.allclose(abs(task.HORIZONTAL_ROTATION[:, 0]), [0, 1, 0])
    for waypoints in task.demo().values():
        for w in waypoints:
            assert np.allclose(w.quat, task.HORIZONTAL_QUAT)
