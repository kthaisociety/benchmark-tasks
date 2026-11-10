"""Checks of dc_earth_fault.py's DC-system model and scoring, without the simulator. Needs yam_arena (the official
stack):

    python -m pytest -q test_dc_earth_fault.py
"""

from pathlib import Path

from yam_arena.task_file import load

M = load(Path(__file__).with_name("dc_earth_fault.py")).module


def seated(f):
    return (*M.SOCKET_XY[f], M.FLOOR + 0.001)


class Rig:
    """Carrier positions (centre x, y, bottom z), stepped through the model."""

    def __init__(self):
        M._reset()
        self.c = {f: seated(f) for f in M.FEEDERS}

    def run(self, steps=10):
        for _ in range(steps):
            M._advance(self.c)

    def pull(self, f):  # held 5 cm up, over its socket
        x, y, _ = seated(f)
        self.c[f] = (x, y, M.FLOOR + 0.05)
        self.run()

    def set_down(self, f):
        self.c[f] = (M.SOCKET_XY[f][0], M.PARK_Y, M.TOP)
        self.run()

    def insert(self, f, socket=None):
        self.c[f] = seated(socket or f)
        self.run()

    def check(self, *feeders):
        for f in feeders:
            self.pull(f)
            self.insert(f)


def test_start():
    r = Rig()
    r.run(100)
    assert not any(M._S["open"].values()) and M._S["needle"] == M.RED_DEG and M.score(None) == 0.0


def test_rattling_in_its_socket_stays_closed():
    r = Rig()
    x, y, _ = seated("f2")
    for lift in (0.002, 0.006, 0.009, 0.0):
        r.c["f2"] = (x + 0.004, y - 0.004, M.FLOOR + lift)
        r.run(1)
        assert not M._S["open"]["f2"]


def test_needle_follows_f3():
    r = Rig()
    r.pull("f3")
    r.run(30)  # 2 s in all
    assert M._S["needle"] > M.GREEN_DEG - 5
    r.insert("f3")
    r.run(30)
    assert M._S["needle"] < M.RED_DEG + 5


def test_needle_stays_red_for_healthy_feeders():
    r = Rig()
    r.pull("f1")
    r.run(30)
    assert M._S["needle"] < M.RED_DEG + 1


def test_procedure_scores_full():
    r = Rig()
    r.check("f1", "f2")
    r.pull("f3")
    r.set_down("f3")
    assert M._S["max_open"] == 1 and M.score(None) == 1.0


def test_right_end_state_but_two_out_at_once():
    r = Rig()
    r.pull("f1"), r.set_down("f1"), r.pull("f2"), r.set_down("f2"), r.insert("f1"), r.insert("f2"), r.pull("f3")
    assert M.score(None) == 0.5


def test_pull_everything():
    r = Rig()
    for f in M.FEEDERS:
        r.pull(f), r.set_down(f)
    assert M.score(None) == 0.25


def test_found_then_put_back():
    r = Rig()
    r.check("f1", "f2", "f3")
    assert M.score(None) == 0.0


def test_healthy_left_out_without_f3():
    r = Rig()
    r.pull("f1"), r.set_down("f1"), r.pull("f2"), r.set_down("f2")
    assert M.score(None) == 0.0


def test_any_carrier_closes_a_socket():
    r = Rig()  # F3's own carrier out, F1's moved into F3's socket: F3 is live again and F1 is open
    r.pull("f1"), r.pull("f3"), r.set_down("f3"), r.insert("f1", socket="f3")
    assert M._S["open"]["f1"] and not M._S["open"]["f3"] and M.score(None) == 0.0


def test_demo_plan_pulls_only_f3():
    plan = dict((f, a) for a, f in M.DEMO_PLAN)
    assert plan == {"f1": "check", "f2": "check", "f3": "pull"}
