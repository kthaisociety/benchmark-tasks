"""Offline checks for the gym safety arms (no simulator): the notch window, the score of every outcome, the detent,
every demo variant on a one-dimensional model of the arms and the gripper, drive-lag robustness, and the geometry
limits from the onboarding notes (reach, parking zone, finger clearance).

    python3 -m unittest test_task        (from this folder)
"""

import itertools
import math
import unittest

import safety as s

BASES = {"left": (0.0, 0.305), "right": (0.0, -0.305)}
RUN_LIMIT_S = 30.0          # 600 steps at 20 Hz


class Window(unittest.TestCase):
    def test_one_correct_notch(self):
        self.assertEqual(s.CORRECT, (0.075,))
        self.assertEqual(len(s.NOTCHES), 8)
        self.assertTrue(all(abs(b - a - 0.015) < 1e-9 for a, b in zip(s.NOTCHES, s.NOTCHES[1:])))

    def test_window_against_the_chest(self):
        lo, hi = s.WINDOW
        self.assertAlmostEqual(hi, s.CHEST_TOP)            # a bar touching the chest is not stopped
        self.assertAlmostEqual(hi - lo, 0.010)             # and is caught within 1 cm (5 cm at full scale)
        self.assertGreater(s.CORRECT[0], s.BENCH_TOP)
        self.assertLess(s.BAR_Z - s.BAR_W / 2, s.TOP_NOTCH + 0.06)   # the bar rests well above every arm

    def test_start_scores_zero_and_needs_both_directions(self):
        self.assertEqual(s.score_heights(s.START["left"], s.START["right"]), 0.0)
        self.assertGreater(s.START["left"], s.CORRECT[0])  # the left arm has to come down ...
        self.assertLess(s.START["right"], s.CORRECT[0])    # ... and the right one up


class Scoring(unittest.TestCase):
    def test_every_pair_of_notches(self):
        """All 64 end states: 0 if either side is low; otherwise 1.0, 0.5 or 0.25 by how many sides are in the window."""
        seen = set()
        for a, b in itertools.product(s.NOTCHES, repeat=2):
            sc = s.score_heights(a, b)
            ka, kb = s.classify(a), s.classify(b)
            if "low" in (ka, kb):
                self.assertEqual(sc, 0.0, (a, b))
            else:
                self.assertEqual(sc, {2: 1.0, 1: 0.5, 0: 0.25}[(ka, kb).count("ok")], (a, b))
            seen.add(sc)
        self.assertEqual(seen, {0.0, 0.25, 0.5, 1.0})
        self.assertEqual(sum(s.score_heights(a, b) == 1.0 for a, b in itertools.product(s.NOTCHES, repeat=2)), 1)

    def test_moving_the_rack_scores_zero(self):
        self.assertEqual(s.score_heights(s.CORRECT[0], s.CORRECT[0], rack_moved=True), 0.0)
        tr = s.Trial()
        tr.reset()
        tr.observe(s.CORRECT[0], s.CORRECT[0], rack_moved=True)
        tr.observe(s.CORRECT[0], s.CORRECT[0], rack_moved=False)     # latched
        self.assertEqual(tr.score(), 0.0)
        tr.reset()
        self.assertFalse(tr.rack_moved)

    def test_window_edges(self):
        lo, hi = s.WINDOW
        self.assertEqual(s.classify(lo), "ok")
        self.assertEqual(s.classify(lo - 1e-4), "low")
        self.assertEqual(s.classify(hi), "high")


class Detent(unittest.TestCase):
    def test_a_brush_does_not_move_a_locked_arm(self):
        pin = s.Detent()
        pin.reset(s.CORRECT[0])
        for k in range(40):                                  # dragged up to 5 mm off its notch and back
            h = s.CORRECT[0] + 0.005 * math.sin(k / 6)
            self.assertAlmostEqual(pin.target(h), s.CORRECT[0])
        self.assertFalse(pin.free)

    def test_a_pull_unlocks_it_and_it_relocks_at_the_nearest_notch(self):
        for stop in [s.LOW_NOTCH + i * 0.001 for i in range(0, 106, 3)]:
            pin = s.Detent()
            pin.reset(s.TOP_NOTCH)
            h = s.TOP_NOTCH
            while h > stop + 1e-9:                           # pulled down at 5 cm/s
                h = max(stop, h - 0.0025)
                self.assertAlmostEqual(pin.target(h), h, msg=stop) if pin.free else None
            for _ in range(s.STILL_STEPS + 10):              # then let go: still, until the drive seats it
                tgt = pin.target(h)
                h += (tgt - h) * 0.7
            if abs(stop - s.TOP_NOTCH) > s.UNLOCK:
                self.assertFalse(pin.free)
                self.assertAlmostEqual(tgt, s.nearest_notch(stop), msg=stop)

    def test_lock_and_sag_are_light(self):
        self.assertLessEqual(s.RATE * s.UNLOCK, 1.0)         # newtons to unlock; the arm pushes about 2 N
        self.assertLessEqual(s.RATE * 0.0075, 1.2)           # the most a locked arm pulls back toward its notch
        self.assertLess(s.SAG, 0.004)


class Variants(unittest.TestCase):
    def test_expected_scores(self):
        for v, want in s.EXPECTED.items():
            with self.subTest(variant=v):
                tr, _ = s.simulate(s.demo_plan(v), seconds=RUN_LIMIT_S)
                self.assertEqual(tr.score(), want)

    def test_variants_end_where_intended(self):
        for v, (lt, rt) in s.VARIANTS.items():
            with self.subTest(variant=v):
                _, log = s.simulate(s.demo_plan(v), seconds=RUN_LIMIT_S)
                end = {"left": log[-1][1], "right": log[-1][2]}
                for side, want in (("left", lt), ("right", rt)):
                    want = s.START[side] if want is None else s.nearest_notch(want)
                    self.assertAlmostEqual(end[side], want, delta=0.001, msg=side)
                    self.assertIn(round(end[side], 3), s.NOTCHES)      # every arm ends seated in a notch

    def test_correct_finishes_well_inside_the_run_limit(self):
        p = s.demo_plan("correct")
        self.assertLess(max(s.plan_duration(x) for x in p.values()), RUN_LIMIT_S - 10)

    def test_drive_lag(self):
        for lag in (0.02, 0.05, 0.1, 0.2):
            for v in ("correct", "one_low", "uneven", "held_between"):
                with self.subTest(lag=lag, variant=v):
                    tr, _ = s.simulate(s.demo_plan(v), seconds=RUN_LIMIT_S, lag=lag)
                    self.assertEqual(tr.score(), s.EXPECTED[v])

    def test_grip_height_margin(self):
        """Pad tips 4 mm off the planned height still take the handle."""
        for dz in (-0.004, 0.004):
            with self.subTest(dz=dz):
                old = s.GRIP_UP
                try:
                    p = s.demo_plan("correct")
                    for side in p:
                        p[side] = [(d, (x, y, z + dz) if "handle" in lab or "arm" in lab or "notch" in lab or lab == "settle"
                                    or lab == "let go" else (x, y, z), f, lab, yw) for d, (x, y, z), f, lab, yw in p[side]]
                    tr, _ = s.simulate(p, seconds=RUN_LIMIT_S)
                    self.assertEqual(tr.score(), 1.0)
                finally:
                    s.GRIP_UP = old


class Geometry(unittest.TestCase):
    def test_reach(self):
        for side, (bx, by) in BASES.items():
            y = s.ARM_Y if side == "left" else -s.ARM_Y
            d = math.dist((s.HANDLE_X, y), (bx, by))
            self.assertTrue(0.10 <= d <= 0.50, (side, d))

    def test_parking_zone_is_clear(self):
        """Nothing in x < 0.30 with |y| > 0.20: the plates' outer faces stop at |y| 0.199, and everything that reaches
        x < 0.30 (handles, arms, bench, figure) stays inside |y| 0.15."""
        self.assertLessEqual(s.PLATE_Y[-1] + s.PLATE_T / 2, 0.200)
        self.assertLessEqual(s.POST_Y + 0.015, 0.200)
        self.assertLessEqual(s.ARM_Y + s.ARM_W / 2, 0.150)
        self.assertLessEqual(s.BAR_HALF_LEN, 0.200)

    def test_fingers_clear_everything_but_the_handle(self):
        opening = -2 * s.OPEN
        self.assertGreaterEqual((opening - s.HANDLE[1]) / 2, 0.015)              # 1.5 cm free each side
        finger_half_x = 0.010                                                    # pad half-width along x
        self.assertGreater(s.RACK_X - s.BAR_W / 2 - (s.HANDLE_X + finger_half_x), 0.04)   # the bar
        self.assertGreater(s.RACK_X - 0.015 - (s.HANDLE_X + finger_half_x), 0.04)          # the uprights
        inner = s.ARM_Y - opening / 2 - 0.005                                    # inner finger, minus its thickness
        self.assertGreater(inner, 0.065 + 0.005)                                 # the lifter's torso (|y| 0.065)
        self.assertAlmostEqual(-2 * s.GRIP, s.HANDLE[1] - s.SQUEEZE)

    def test_handle_under_the_palm(self):
        self.assertLess(s.GRIP_UP, s.HANDLE[2] / 2)                       # the pads take most of the handle
        self.assertGreaterEqual(s.GRIP_UP + s.PAD_LEN - s.HANDLE[2], 0.005)   # the palm stays 5 mm or more above it
        self.assertGreaterEqual(s.GRIP_UP - 0.004, 0.002)                 # a grip 4 mm low still clears the arm's top

    def test_heights_reachable(self):
        lowest = s.tcp_for(s.LOW_NOTCH) - s.TIP
        self.assertGreater(lowest, 0.030)                         # pad tips well above the table
        self.assertLess(s.START["left"] + s.HANDLE[2] + s.CLEAR + s.TIP, 0.30)


if __name__ == "__main__":
    unittest.main()
