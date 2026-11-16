"""Offline checks for open-shelf-ready-tray (no simulator): geometry, the finger's gap, the tear's force balance,
scoring and the scripted demo's reach. Run:  PYTHONPATH=<yam-isaac-lab-arena> python -m pytest -q test_task.py"""

import math
import os

import numpy as np
import pytest

for _k in ("DEMO_VARIANT", "F_LIP", "F_DIAG", "PINCH_DEPTH", "PEEL_DEG", "PULL_OUT"):
    os.environ.pop(_k, None)

import task as T  # noqa: E402


def test_objects():
    names = [o.name for o in T.OBJECTS]
    assert len(names) == len(set(names)) == 4 + 12
    assert names.index("tray") < names.index("panel") < names.index("flap_left")
    assert len(T.TIES) == len(T.LIP_TIE_Y) + 6 == 15


def test_cans_fit_the_tray():
    inner_y = T.TW / 2 - T.T if hasattr(T, "T") else T.TW / 2 - T.B
    assert max(abs(y) for y in T.CAN_Y) + T.CAN_R < inner_y - 0.0005
    assert T.CAN_X[-1] + T.CAN_R < T.TD - T.B - 0.0005
    assert T.CAN_X[0] - T.CAN_R - T.B == pytest.approx(T.CAN_FRONT_GAP)
    assert T.CAN_PITCH > 2 * T.CAN_R  # no overlap
    assert T.CAN_H < T.TH  # below the rim


def _gap_width(x_tray: float) -> float:
    """Free width (y) between the two middle front cans at depth x (tray frame)."""
    dx = T.CAN_X[0] - x_tray
    half = T.CAN_Y[1] - math.sqrt(max(T.CAN_R ** 2 - dx ** 2, 0.0))
    return 2 * half


def test_back_finger_fits_between_the_middle_cans():
    """The back finger: its pad BACK_CLEAR behind the panel, its body up to about 10 mm thick behind that, 27 mm wide."""
    pad_x = T.B + T.BACK_CLEAR
    assert T.BACK_CLEAR >= 0.004  # the arm lands a few mm off (2 mm seen)
    for x in np.linspace(pad_x, pad_x + 0.010, 6):
        assert _gap_width(x) > 0.030, (x, _gap_width(x))


def _peel_loads(n_diag_left=3, hook_h=None):
    """Rigid panel+flaps swinging out about the lip line, ties as equal stiff springs: (pull to break the outermost
    diagonal tie, worst lip tie load per newton of pull)."""
    h = hook_h if hook_h is not None else (T.TH - T.PINCH_DEPTH) - T.LIP
    pts = [T.diag_point(f) for f in T.DIAG_TIE_F][:n_diag_left]
    rel = [(x - T.B / 2, z - T.LIP) for x, z in pts] * 2  # both sides
    r2 = sum(dx * dx + dz * dz for dx, dz in rel)
    r_max = max(math.hypot(dx, dz) for dx, dz in rel)
    pull_to_tear = T.F_DIAG * r2 / (h * r_max)
    fx = sum(dz for dx, dz in rel) * h / r2  # the diagonal ties' pull back, per newton
    fz = sum(dx for dx, dz in rel) * h / r2
    lip_per_n = math.hypot(1.0 - fx, fz) / len(T.LIP_TIE_Y)
    return pull_to_tear, lip_per_n


def test_peel_tears_with_a_light_pull_and_the_lip_holds_meanwhile():
    pull, lip_per_n = _peel_loads()
    assert 2.0 < pull < 4.5, pull  # within what the arm pulls sideways (about 200 N/m)
    assert lip_per_n * pull < T.F_LIP, (lip_per_n * pull, T.F_LIP)  # the lip doesn't go first


def test_tear_runs_on_once_started():
    """With the outermost diagonal ties gone, the next ones need less pull: the tear runs down the diagonals."""
    p3, _ = _peel_loads(3)
    p2, _ = _peel_loads(2)
    p1, _ = _peel_loads(1)
    assert p1 < p2 < p3


def test_straight_lift_spreads_over_all_ties():
    """Lifting the panel straight up, in its plane: every tie shares it, so it takes far more than the peel."""
    lift_to_break = len(T.LIP_TIE_Y) * T.F_LIP + 6 * T.F_DIAG
    pull, _ = _peel_loads()
    assert lift_to_break > 4.0 * pull
    # the freed panel's lip tears from one end, a tie or two at a time: each within what the arm pulls
    assert 2 * T.F_LIP <= 3.0
    # a straight lift or an accidental push spreads over every tie: far more than the peel
    assert len(T.LIP_TIE_Y) * T.F_LIP + 6 * T.F_DIAG > 4.0 * pull


def test_tie_geometry():
    for name, seg, p_seg, p_tray in T.TIE_LOCAL:
        # at the start each tie's two ends coincide
        sp = T._SEG_POS[seg]
        assert np.allclose(sp + p_seg, T.TRAY_POS + p_tray)
    for f in T.DIAG_TIE_F:
        x, z = T.diag_point(f)
        assert T.B < x < T.DX and T.LIP < z < T.TH


def _R_y(deg):
    t = math.radians(deg)
    return np.array([[math.cos(t), 0, math.sin(t)], [0, 1, 0], [-math.sin(t), 0, math.cos(t)]])


def test_torn_detection():
    poses = {"tray": (T.TRAY_POS, np.eye(3))}
    for n in T.SEGMENTS:
        poses[n] = (T._SEG_POS[n], np.eye(3))
    assert T.torn_ties(poses) == set()
    pp = T._SEG_POS["panel"]
    poses["panel"] = (pp + [-0.01, 0, 0], np.eye(3))
    assert {n for n in T.torn_ties(poses) if n.startswith("lip")} == {n for n, s, *_ in T.TIE_LOCAL if s == "panel"}


def test_points():
    base = {"torn": set(), "off": False, "removed": False, "tray_ok": True, "cans_ok": True, "let_go": True,
            "still": True, "tipped": False}
    assert T.points(base) == 0.0
    assert T.points({**base, "torn": {"diag_l2"}}) == 0.25
    full = {**base, "torn": {n for n, *_ in T.TIES}, "off": True, "removed": True}
    assert T.points(full) == 1.0
    assert T.points({**full, "let_go": False}) == 0.5
    assert T.points({**full, "removed": False}) == 0.5
    assert T.points({**full, "cans_ok": False}) == 0.5
    assert T.points({**full, "tipped": True}) == 0.0


def test_in_or_on_tray():
    assert T.in_or_on_tray(T.TRAY_POS + [0.1, 0, 0.05], T.TRAY_POS, np.eye(3))
    assert not T.in_or_on_tray(T.TRAY_POS + [-0.05, 0, 0.01], T.TRAY_POS, np.eye(3))


@pytest.mark.parametrize("variant", ["full", "nothing", "lift", "start_only", "keep_holding", "drop_in_tray"])
def test_demo_builds_and_arms_end_together(variant, monkeypatch):
    monkeypatch.setenv("DEMO_VARIANT", variant)
    plans = T.demo()
    assert T._length(plans["left"]) == pytest.approx(T._length(plans["right"]))


def test_demo_reaches_every_waypoint():
    from yam_arena import ik, station

    bases = {k: np.array(v) for k, v in station.BASES.items()}
    for side, wps in T.demo().items():
        q = None
        for w in wps:
            R = np.asarray(w.yaw, float)
            q, pe, re = ik.solve(np.asarray(w.pos), ik.matrix_to_quat_xyzw(R), q_init=q, base_pos=bases[side], iters=400)
            assert pe < 0.002 and math.degrees(re) < 1.5, (side, w.label, np.round(w.pos, 3), pe, math.degrees(re))


def test_prim_names_are_valid_usd_identifiers():
    import re

    names = [f"tie_{n}" for n, *_ in T.TIES] + ["crease_l", "crease_r"] + [o.name for o in T.OBJECTS]
    assert all(re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", n) for n in names), names
