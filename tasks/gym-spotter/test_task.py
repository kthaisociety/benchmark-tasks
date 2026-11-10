"""Offline checks for the gym spotter (no simulator): the lifter's set, the scoring of every demo variant on a
one-dimensional model of the bar and the gripper, a scan for shortcuts, timing and lag robustness, and the geometry
limits from the onboarding notes (reach, parking zone, finger clearance).

    python3 -m unittest test_task        (from this folder)
"""

import math
import unittest

import spotter as s

RIGHT_BASE = (0.0, -0.305)
RUN_LIMIT_S = 35.0          # 700 steps at 20 Hz


def nominal():
    """The unaided set: (t, bar_h, stage) every step."""
    _, log = s.simulate(s.demo_plan("nothing"), seconds=RUN_LIMIT_S)
    return [(t, b, st) for t, b, _, st, _, _ in log]


def grab_plan(t_arrive, h, lift=True):
    """Hover, lower around the bar to arrive at t_arrive with the pad tips at height h, close, lift to lockout, let go."""
    x, y, Y = s.BAR_X, s.GRIP_Y, s.YAW
    plan = [(2.0, (x, y, s.HOVER), s.OPEN, "hover", Y),
            (t_arrive - 3.0, (x, y, s.HOVER), s.OPEN, "wait", Y),
            (1.0, (x, y, s.tcp_for(h)), s.OPEN, "lower", Y),
            (0.3, (x, y, s.tcp_for(h)), s.OPEN, "settle", Y),
            (s.CLOSE_T, (x, y, s.tcp_for(h)), s.GRIP, "close", Y)]
    if lift:
        top = s.tcp_for(s.TRAVEL - 0.004)
        plan += [(1.5, (x, y, top), s.GRIP, "lift", Y), (0.5, (x, y, top), s.GRIP, "settle", Y),
                 (s.CLOSE_T, (x, y, top), s.OPEN, "release", Y), (1.0, (x, y, s.HOVER + 0.03), s.OPEN, "clear", Y)]
    return plan


class TheSet(unittest.TestCase):
    def test_unaided_set(self):
        log = nominal()
        T = s.phase_starts()
        stages = [st for _, _, st in log]
        for k in range(1, 6):
            self.assertIn(f"rep{k}", stages)
        # every rep reaches lockout and the chest
        self.assertGreater(max(b for _, b, _ in log), s.TRAVEL - 0.002)
        # the failed rep: stalls about STALL_H off the chest, then the bar ends pinned on the chest
        stall = [b for t, b, st in log if st == "stall"]
        self.assertTrue(all(abs(b - s.STALL_H) < 0.004 for b in stall[5:]), (min(stall), max(stall)))
        self.assertEqual(log[-1][2], "pinned")
        self.assertLess(log[-1][1], 0.001)
        self.assertLess(T["pinned"], RUN_LIMIT_S - 8.0)    # time left for a late rescue

    def test_reps_slow_down(self):
        ups = [s.EASY_UP] * 3 + [s.REP4_UP, s.REP5_UP]
        self.assertEqual(ups, sorted(ups))
        self.assertLess(s.STALL_H, 0.5 * s.TRAVEL)

    def test_grind_profile_monotone(self):
        xs = [i / 200 for i in range(201)]
        ys = [s.grind_profile(x) for x in xs]
        self.assertTrue(all(b >= a for a, b in zip(ys, ys[1:])))
        self.assertAlmostEqual(ys[0], 0.0)
        self.assertAlmostEqual(ys[-1], 1.0)


class Variants(unittest.TestCase):
    def test_expected_scores(self):
        for variant, want in s.EXPECTED.items():
            with self.subTest(variant=variant):
                trial, log = s.simulate(s.demo_plan(variant), seconds=RUN_LIMIT_S)
                self.assertEqual(trial.score(), want)

    def test_correct_ends_well_inside_the_run_limit(self):
        trial, log = s.simulate(s.demo_plan("correct"), seconds=RUN_LIMIT_S)
        self.assertTrue(trial.success())
        self.assertLess(log[-1][0] + 1.0, RUN_LIMIT_S - 5.0)

    def test_variants_fail_for_the_right_reason(self):
        why = {}
        for variant in ("nothing", "takeover", "block", "late", "halfway"):
            trial, log = s.simulate(s.demo_plan(variant), seconds=RUN_LIMIT_S)
            why[variant] = (trial.interfered, trial.dropped, trial.racked_in_fail, log[-1][3])
        self.assertEqual(why["nothing"], (False, True, False, "pinned"))
        self.assertTrue(why["takeover"][0])
        self.assertTrue(why["block"][0])
        self.assertEqual(why["late"], (False, True, True, "racked"))
        self.assertEqual(why["halfway"][:3], (False, False, False))

    def test_interference_latches_when_expected(self):
        def latch_time(variant):
            plan = s.demo_plan(variant)
            for n in range(1, int(RUN_LIMIT_S / s.DT)):
                trial, log = s.simulate(plan, seconds=n * s.DT)
                if trial.interfered:
                    return log[-1][3]
            return None
        self.assertEqual(latch_time("block"), "rep2")      # the next rep runs into the shut fingers
        self.assertEqual(latch_time("takeover"), "rep6")   # it won't let the lifter lower the bar
        self.assertIsNone(latch_time("correct"))


class Shortcuts(unittest.TestCase):
    def test_grab_and_lift_at_any_time(self):
        """Take hold of the bar wherever it is and push it to lockout, at every arrival time from 3 s to the end.
        Before the sixth rep this always scores 0; it scores 1.0 only from the start of the failing rep's rise to
        just before the bar is back on the chest, and 0.5 after that."""
        log = nominal()
        at = {round(t, 2): (b, st) for t, b, st in log}
        T = s.phase_starts()
        results = []
        for k in range(12, 120):
            t_arr = k * 0.25
            bite = round(round((t_arr + 0.3 + s.CLOSE_T / 2) / s.DT) * s.DT, 2)
            h, stage = at.get(bite, (0.0, "pinned"))
            trial, _ = s.simulate(grab_plan(t_arr, h), seconds=RUN_LIMIT_S + 2)
            results.append((t_arr, stage, trial.score()))
        for t_arr, stage, sc in results:
            if stage in ("rep1", "rep2", "rep3", "rep4", "rep5", "rep6"):
                self.assertEqual(sc, 0.0, (t_arr, stage))
        full = [t for t, _, sc in results if sc == 1.0]
        self.assertTrue(full)
        self.assertGreaterEqual(min(full), T["fail6"] - 1.0)
        self.assertGreaterEqual(max(full) - min(full), 3.0)       # a window of several seconds to react
        self.assertTrue(all(sc == 0.5 for t, st, sc in results if st == "pinned"))

    def test_hold_mid_air_without_lockout(self):
        trial, _ = s.simulate(s.demo_plan("halfway"), seconds=RUN_LIMIT_S)
        self.assertEqual(trial.score(), 0.25)

    def test_score_values(self):
        seen = {s.simulate(s.demo_plan(v), seconds=RUN_LIMIT_S)[0].score() for v in s.EXPECTED}
        self.assertEqual(seen, {0.0, 0.25, 0.5, 1.0})


class Robustness(unittest.TestCase):
    def test_drive_lag(self):
        """The simulated drive may track more slowly than modelled: the outcomes must not change."""
        for lag in (0.02, 0.04, 0.08, 0.12):
            for variant in ("correct", "nothing", "late", "halfway"):
                with self.subTest(lag=lag, variant=variant):
                    trial, _ = s.simulate(s.demo_plan(variant), seconds=RUN_LIMIT_S, lag=lag)
                    self.assertEqual(trial.score(), s.EXPECTED[variant])
                    if variant == "nothing":
                        self.assertFalse(trial.lifter.lifted_by_spotter)   # the shaking is not mistaken for help
                        self.assertFalse(trial.interfered)

    def test_catch_timing_margin(self):
        """Arriving up to a second early or 1.5 s late at the stall still scores 1.0."""
        T = s.phase_starts()
        for dt in (-1.0, -0.5, 0.0, 0.5, 1.0, 1.5):
            with self.subTest(dt=dt):
                trial, _ = s.simulate(grab_plan(T["stall"] + 0.2 + dt, s.STALL_H), seconds=RUN_LIMIT_S)
                self.assertEqual(trial.score(), 1.0)

    def test_carry_load_is_light(self):
        """Once the spotter has lifted the bar, the lifter's target stays within CARRY of it (0.5 N at 500 N/m), plus
        the stall's shake while it lasts: at most about 1.5 N, under the arm's roughly 2 N push."""
        trial, log = s.simulate(s.demo_plan("correct"), seconds=RUN_LIMIT_S)
        carried = [(b, tg) for t, b, tg, st, held, _ in log if held and st in ("rise", "stall", "sink")]
        self.assertTrue(carried)
        lifted = [(b, tg) for b, tg in carried if b > s.STALL_H + 0.01]
        self.assertTrue(lifted)
        self.assertTrue(all(b - tg <= s.CARRY + s.SHAKE_AMP + 1e-3 for b, tg in lifted), max(b - tg for b, tg in lifted))
        self.assertLessEqual(500.0 * (s.CARRY + s.SHAKE_AMP), 1.6)

    def test_catch_height_margin(self):
        T = s.phase_starts()
        for dh in (-0.008, 0.0, 0.008):
            with self.subTest(dh=dh):
                trial, _ = s.simulate(grab_plan(T["stall"] + 0.2, s.STALL_H + dh), seconds=RUN_LIMIT_S)
                self.assertEqual(trial.score(), 1.0)


class Geometry(unittest.TestCase):
    def test_reach(self):
        d = math.dist((s.BAR_X, s.GRIP_Y), RIGHT_BASE)
        self.assertTrue(0.10 <= d <= 0.50, d)
        self.assertLess(abs(s.GRIP_Y), 0.15)               # no more than 15 cm past the midline (it isn't past it)

    def test_fingers_clear_the_bar_and_hands(self):
        opening = -2 * s.OPEN
        self.assertGreaterEqual((opening - s.BAR_W) / 2, 0.015)
        self.assertGreaterEqual(abs(s.GRIP_Y - (-s.HAND_Y)) - 0.013 - 0.020, 0.025)   # hand half-width, pad half-width
        self.assertLess(s.GRIP, 0)
        self.assertAlmostEqual(-2 * s.GRIP, s.BAR_W - s.SQUEEZE)

    def test_heights(self):
        self.assertGreaterEqual(s.HOVER - s.TIP - (s.BAR_HIGH + s.BAR_W / 2), 0.035)     # tips over the bar at lockout
        self.assertGreaterEqual(s.BAR_LOW - s.CHEST_TOP, s.BAR_W / 2 - 1e-9)             # bar rests on the chest
        self.assertGreaterEqual(s.BAR_LOW - s.CHEST_TOP, 0.010)                          # tips above the chest at a late catch
        circum = s.PLATE_W / 2 / math.cos(math.radians(22.5))
        self.assertGreater(s.BAR_LOW - circum, 0.03)                                     # plates clear the table
        self.assertGreater(s.BAR_LOW - s.BAR_W / 2 - 0.070, 0.005)                       # forearms clear the table

    def test_parking_zone_is_clear(self):
        """Nothing sits in x < 0.30 with |y| > 0.20, where the arms park: the uprights, carriages and feet are 3 cm
        wide in y, and everything wider in y (plates, collars, hands) is inside them."""
        self.assertLessEqual(s.POST_Y + 0.015, 0.200 + 1e-9)
        self.assertLessEqual(s.PLATE_Y[-1] + 0.012 + 0.005, s.POST_Y - 0.015 + 1e-9)   # outer collar face
        self.assertLessEqual(s.BAR_HALF_LEN, 0.200)

    def test_demo_plans_are_on_time(self):
        for variant in s.EXPECTED:
            with self.subTest(variant=variant):
                plan = s.demo_plan(variant)
                self.assertTrue(plan)
                self.assertTrue(all(d > 0 for d, *_ in plan))


if __name__ == "__main__":
    unittest.main()
