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
    for name, y in [("red_swap", 0.13), ("blue_swap", -0.13)]:
        shelf = task.SHELVES[task.GOALS[name]]
        out[name] = state((0.16, y, task.surface(shelf) + task.HEIGHT / 2))
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
    p = objects["red_neighbor"].pos.copy()
    forces = {}
    if failure == "tilt":
        objects["red_neighbor"] = state(p, tilt=21)
    elif failure == "force":
        forces["red_neighbor"] = 40.01
    elif failure == "fall":
        p[2] -= 0.02
        objects["red_neighbor"] = state(p)
    elif failure == "workspace":
        p[0] = 0.5
        objects["red_neighbor"] = state(p)
    else:
        p[0] = float("nan")
        objects["red_neighbor"] = state(p)
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
        p[1] = 0.20
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
