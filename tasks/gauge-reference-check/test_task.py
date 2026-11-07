"""Offline tests of the scorer and the demo plan (no simulator): python -m pytest -q test_task.py"""

import importlib.util
import pathlib

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("gauge_task", HERE / "task.py")
T = importlib.util.module_from_spec(spec)
spec.loader.exec_module(T)
from partkit import PartState  # noqa: E402

SIZES = {"part": T.BLOCK, "reference": T.BLOCK, "tag": T.TAG}
START = {"part": T.SUSPECT_XY, "reference": T.HOLDER_XY, "tag": T.TAG_HOME_XY}


def state(name, xy, bottom=0.0, moving=False):
    sx, sy, sz = SIZES[name]
    centre = np.array([xy[0], xy[1], T.TOP + bottom + sz / 2])
    half = np.array([sx, sy, sz]) / 2
    return PartState(pos=centre, quat_xyzw=np.array([0, 0, 0, 1.0]),
                     lin_vel=np.array([0.05 if moving else 0.0, 0, 0]), ang_vel=np.zeros(3),
                     low=centre - half, high=centre + half)


class Trial:
    def __init__(self, trial):
        self.record = T.new_record(trial)
        self.where = {n: (xy, 0.0) for n, xy in START.items()}
        self.last = 0.0

    def put(self, name, xy, bottom=0.0):
        self.where[name] = (xy, bottom)
        return self.step()

    def step(self, pressed=False):
        objects = {n: state(n, xy, b) for n, (xy, b) in self.where.items()}
        self.last = T.judge(self.record, objects, pressed)
        return self.last

    def press(self):
        self.step(pressed=True)
        return self.step(pressed=False)

    def check(self):
        self.put("reference", T.PAD_XY, T.PAD_TOP)
        self.press()
        return self.record["lamp"]


OK, DRIFTED = 1, 2


def test_trials_alternate():
    assert T.gauge_ok(1) and not T.gauge_ok(2) and T.gauge_ok(3)


def test_start_and_do_nothing_score_zero():
    for trial in (OK, DRIFTED):
        t = Trial(trial)
        assert t.step() == 0.0
        assert t.press() == 0.0 and t.record["lamp"] is None        # empty pad: no test


def test_lamp_shows_the_gauge_state_only_through_the_reference():
    t = Trial(OK)
    t.put("part", T.PAD_XY, T.PAD_TOP)
    t.press()
    assert t.record["lamp"] == "fail" and not t.record["checked"]   # retesting the part tells nothing
    assert Trial(OK).check() == "pass"
    assert Trial(DRIFTED).check() == "fail"


def test_full_procedure_in_calibration():
    t = Trial(OK)
    t.check()
    assert t.last == 0.25
    t.put("reference", T.HOLDER_XY)
    assert t.last == 0.5
    assert t.put("part", T.QUARANTINE_XY) == 1.0


def test_full_procedure_drifted():
    t = Trial(DRIFTED)
    t.check()
    t.put("reference", T.HOLDER_XY)
    assert t.put("tag", T.TAG_PLATE_XY, 0.007) == 0.75
    assert t.put("part", T.RETEST_XY) == 1.0


def test_reference_left_on_pad_loses_its_quarter():
    t = Trial(OK)
    t.check()
    assert t.put("part", T.QUARANTINE_XY) == 0.75


@pytest.mark.parametrize("trial,moves", [
    (OK, [("part", T.RETEST_XY, 0.0)]),
    (OK, [("part", T.QUARANTINE_XY, 0.0), ("tag", T.TAG_PLATE_XY, 0.007)]),     # both dispositions
    (DRIFTED, [("part", T.QUARANTINE_XY, 0.0), ("tag", T.TAG_PLATE_XY, 0.007)]),
    (DRIFTED, [("tag", T.TAG_PLATE_XY, 0.007), ("part", T.QUARANTINE_XY, 0.0), ("part", T.RETEST_XY, 0.0)]),
])
def test_wrong_disposition_caps_at_the_check(trial, moves):
    t = Trial(trial)
    t.check()
    t.put("reference", T.HOLDER_XY)
    for name, xy, bottom in moves:
        t.put(name, xy, bottom)
    assert t.last == 0.25


@pytest.mark.parametrize("trial", [OK, DRIFTED])
def test_guessing_without_the_check_scores_zero(trial):
    t = Trial(trial)
    if T.gauge_ok(trial):
        t.put("part", T.QUARANTINE_XY)
    else:
        t.put("tag", T.TAG_PLATE_XY, 0.007)
        t.put("part", T.RETEST_XY)
    assert t.last == 0.0


def test_disposition_before_the_check_is_latched():
    t = Trial(OK)
    t.put("part", T.QUARANTINE_XY)
    t.put("part", T.SUSPECT_XY)          # moved back before the check
    t.check()
    t.put("reference", T.HOLDER_XY)
    assert t.put("part", T.QUARANTINE_XY) == 0.25


def test_reference_put_in_a_bin_caps():
    t = Trial(OK)
    t.check()
    t.put("part", T.QUARANTINE_XY)
    t.put("reference", T.RETEST_XY)
    assert t.put("reference", T.HOLDER_XY) == 0.25


def test_carrying_over_a_bin_does_not_count():
    t = Trial(OK)
    t.put("part", T.RETEST_XY, 0.10)     # lifted over the tray on its way past
    t.check()
    t.put("reference", T.HOLDER_XY)
    assert t.put("part", T.QUARANTINE_XY) == 1.0


def test_moving_objects_do_not_score_their_placement():
    t = Trial(OK)
    t.check()
    objects = {"part": state("part", T.QUARANTINE_XY, moving=True), "reference": state("reference", T.HOLDER_XY),
               "tag": state("tag", T.TAG_HOME_XY)}
    assert T.judge(t.record, objects, False) == 0.5


def test_part_and_reference_on_the_pad_together_is_not_a_check():
    t = Trial(DRIFTED)
    t.put("reference", T.PAD_XY, T.PAD_TOP)
    t.where["part"] = ((T.PAD_XY[0] + 0.005, T.PAD_XY[1]), T.PAD_TOP)
    t.press()
    assert not t.record["checked"] and t.last == 0.0


@pytest.mark.parametrize("ok", [True, False])
def test_demo_plan_is_reachable_and_waits_for_the_check(ok):
    plan = T.demo_plan(ok)
    for arm, waypoints in plan.items():
        for w in waypoints:
            x, y, _ = w.pos
            assert 0.10 <= x <= 0.50, (arm, w)
            if arm == "right":
                assert y < 0.15, w
            else:
                assert y > -0.15, w
    press_end = sum(w.duration for w in plan["right"][:12])
    assert plan["left"][0].duration >= press_end
    total = max(sum(w.duration for w in ws) for ws in plan.values())
    assert total * 20 < 1200                                   # within the run limit
