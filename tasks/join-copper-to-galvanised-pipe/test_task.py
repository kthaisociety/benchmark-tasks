"""Checks of task.py without the simulator: the task file loads, nothing starts overlapping, every scripted plan is within
the arms' reach, and where each plan leaves the fittings scores what it should. Needs yam_arena (the official stack):

    python -m pytest -q test_task.py
"""

import math
import os
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("yam_arena")

from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.task_file import load  # noqa: E402

TASK = load(Path(__file__).with_name("task.py"))
M = TASK.module
S = M.S


def plan(variant):
    old = os.environ.get("DU_VARIANT")
    os.environ["DU_VARIANT"] = variant
    try:
        return M.demo()
    finally:
        if old is None:
            os.environ.pop("DU_VARIANT")
        else:
            os.environ["DU_VARIANT"] = old


def test_loads():
    assert TASK.scene == "station"
    assert [o.name for o in TASK.objects] == ["pipework", "union", "cu_coupling", "steel_coupling"]
    assert TASK.score is not None and TASK.success is not None and TASK.demo is not None


def test_instruction_does_not_state_the_rule():
    words = M.INSTRUCTION.lower()
    assert "let go" in words
    for leak in ("union", "dielectric", "galvan", "corros", "steel", "insulat", "brass"):
        assert leak not in words


def test_slots_leave_room_for_the_fingers():
    ys = sorted(M.SLOT_Y.values())
    for a, b in zip(ys, ys[1:]):
        assert b - a - 2 * S.NUT_R >= 0.03 + 0.015   # two fingers (about 1.5 cm) between neighbours, with margin


def test_gap_play_and_saddle():
    assert S.GAP_LEN - S.FIT_LEN == pytest.approx(0.03)          # 1.5 cm of play at each end
    assert S.RAIL_IN < S.FIT_R                                     # the rails catch the fitting's lower half
    assert S.AXIS_Z - S.FIT_R > S.PLATE_TOP                        # the body clears the plate


def _end_state(variant):
    """Where each fitting ends if the plan does exactly what it says: picked fittings lie seated at their target gap,
    turned by the plan's final yaw; the rest stay in the tray."""
    out = {name: S.Pose(pos=(M.TRAY_X, M.SLOT_Y[name], 0.04),
                        quat_xyzw=(0, 0, math.sin(M.START_YAW / 2), math.cos(M.START_YAW / 2)))
           for name in S.FITTINGS}
    for wps in plan(variant).values():
        for w in wps:
            if "lower into gap" in w.label:
                name = w.label.split(":")[0]
                gap = w.label[-1]
                yaw = M.START_YAW + w.yaw
                out[name] = S.Pose(pos=(S.PIPE_X, S.GAP_Y[gap], S.SEAT_Z[name]),
                                   quat_xyzw=(0, 0, math.sin(yaw / 2), math.cos(yaw / 2)))
    return out


@pytest.mark.parametrize("variant", sorted(M.EXPECTED))
def test_each_plan_scores_as_expected_if_executed_exactly(variant):
    held = {w.label.split(":")[0] for wps in plan(variant).values() for w in wps if "keep holding" in w.label}
    free = {name: name not in held for name in S.FITTINGS}
    assert S.score(_end_state(variant), False, free) == M.EXPECTED[variant]


def test_score_without_a_simulator_never_counts_a_fitting_let_go():
    from yam_arena.task_file import ObjectState

    def state(pos, yaw):
        return ObjectState(pos=np.array(pos), quat_xyzw=np.array([0, 0, math.sin(yaw / 2), math.cos(yaw / 2)]),
                           lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=np.zeros(3), high=np.zeros(3))

    with pytest.warns(RuntimeWarning):
        objects = {"pipework": state((S.PIPE_X, 0.0, M.TOP + 0.001), 0.0),
                   "union": state((S.PIPE_X, S.GAP_Y["A"], M.TOP + S.SEAT_Z["union"]), math.pi),
                   "cu_coupling": state((S.PIPE_X, S.GAP_Y["B"], M.TOP + S.SEAT_Z["cu_coupling"]), math.pi),
                   "steel_coupling": state((M.TRAY_X, 0.10, M.TOP + 0.04), M.START_YAW)}
        M._reset()
        assert M.score(objects) == 0.0


@pytest.mark.parametrize("variant", sorted(M.EXPECTED))
@pytest.mark.parametrize("arm", ["left", "right"])
def test_each_plan_is_within_reach(variant, arm):
    p = ArmPlan(plan(variant)[arm], station.BASES[arm])
    for t in np.arange(0.0, p.duration, 0.1):
        p.action7(t)
    assert p.worst_ik_mm < 1.0 and p.worst_ik_deg < 0.5, (p.worst_ik_mm, p.worst_ik_deg)
