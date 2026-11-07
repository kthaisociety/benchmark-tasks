"""Synthetic pose sequences for the scoring and timing rules."""

import math
import unittest

from release_logic import (
    BALL_RADIUS,
    CHUTE_EXIT,
    CHUTE_X,
    DT,
    GATE_BASE_GAP,
    GATE_LIFT,
    GATE_Y,
    LIFT_TO_RELEASE_S,
    PITCH,
    POCKET_OFFSET,
    ROLL_AND_DROP_S,
    TRAY_FLOOR,
    TRAY_Y,
    TRIALS,
    GatePose,
    ReleaseScore,
    deck,
    demo_lift_time,
)


def gate(index, lift=0.0, dx=0.0):
    y = GATE_Y[index]
    return GatePose((CHUTE_X + dx, y, deck(y) + GATE_BASE_GAP + lift), 1.0, 1.0)


def track_ball(y, dz=0.0):
    return (CHUTE_X, y, deck(y) + BALL_RADIUS / math.cos(PITCH) + dz)


class ReleaseTests(unittest.TestCase):
    def setUp(self):
        self.trial = TRIALS["A"]
        self.score = ReleaseScore(self.trial)
        self.closed = (gate(0), gate(1))
        self.staging = (gate(0, GATE_LIFT), gate(1))
        self.open = (gate(0, GATE_LIFT), gate(1, GATE_LIFT))
        self.start = GATE_Y[0] + 0.028
        self.stop = GATE_Y[1] + 0.027

    def stage(self):
        self.score.step(track_ball(self.start), self.closed, 0.30)
        for step in range(19):
            y = self.start + (self.stop - self.start) * step / 18
            self.assertEqual(self.score.step(track_ball(y), self.staging, 0.30), 0)
        for _ in range(12):
            result = self.score.step(track_ball(self.stop), self.staging, 0.30)
        self.assertEqual(result, 0.25)
        self.assertTrue(self.score.staged)

    def release(self):
        for step in range(21):
            y = self.stop + (CHUTE_EXIT - self.stop) * step / 20
            self.assertEqual(self.score.step(track_ball(y), self.open, 0.30), 0)
        self.assertTrue(self.score.exited)

    def land(self, pocket, speed=0.10):
        center = CHUTE_X - pocket * POCKET_OFFSET
        ball = (CHUTE_X, TRAY_Y, TRAY_FLOOR + BALL_RADIUS)
        self.score.step(
            (CHUTE_X, CHUTE_EXIT - 0.020, 0.056), self.open, center - speed * DT
        )
        result = self.score.step(ball, self.open, center)
        return result, center

    def test_staged_transfer_settles_while_riding_the_shuttle(self):
        self.stage()
        self.release()
        result, center = self.land(self.trial.empty)
        self.assertEqual(result, 0)
        for step in range(1, 24):
            shift = step * 0.10 * DT
            ball = (CHUTE_X + shift, TRAY_Y, TRAY_FLOOR + BALL_RADIUS)
            result = self.score.step(ball, self.open, center + shift)
        self.assertEqual(result, 1)
        self.assertEqual(
            self.score.step((0.8, TRAY_Y, 0), self.open, center + shift), 0
        )
        self.assertTrue(self.score.failed)

    def test_stage_only_is_partial_credit(self):
        self.stage()
        self.assertEqual(
            self.score.step(track_ball(self.stop), self.staging, 0.33), 0.25
        )

    def test_opening_second_gate_early_latches_zero(self):
        result = self.score.step(track_ball(self.start), self.open, 0.30)
        self.assertEqual(result, 0)
        self.assertEqual(self.score.step(track_ball(self.stop), self.staging, 0.30), 0)
        self.assertTrue(self.score.failed)

    def test_do_nothing_scores_zero(self):
        for _ in range(120):
            self.assertEqual(
                self.score.step(track_ball(self.start), self.closed, 0.30), 0
            )
        self.assertFalse(self.score.failed)

    def test_ball_lifted_out_of_the_chute_fails(self):
        self.stage()
        self.assertEqual(
            self.score.step(track_ball(self.stop, 0.05), self.staging, 0.30), 0
        )
        self.assertTrue(self.score.failed)

    def test_pushing_gate_out_sideways_fails(self):
        self.assertEqual(
            self.score.step(track_ball(self.start), (gate(0, dx=0.03), gate(1)), 0.30),
            0,
        )
        self.assertTrue(self.score.failed)

    def test_a_fast_shortcut_to_the_outlet_fails(self):
        self.stage()
        for step in range(7):
            y = self.stop + (CHUTE_EXIT - self.stop) * step / 6
            self.assertEqual(self.score.step(track_ball(y), self.open, 0.30), 0)
        self.assertTrue(self.score.failed)

    def test_waiting_with_the_ball_at_the_outlet_fails(self):
        self.stage()
        self.release()
        for _ in range(8):
            self.assertEqual(
                self.score.step(track_ball(CHUTE_EXIT), self.open, 0.30), 0
            )
        self.assertTrue(self.score.failed)

    def test_occupied_pocket_fails(self):
        self.stage()
        self.release()
        result, _ = self.land(self.trial.occupied)
        self.assertEqual(result, 0)
        self.assertTrue(self.score.failed)

    def test_stopping_the_shuttle_does_not_score(self):
        self.stage()
        self.release()
        result, _ = self.land(self.trial.empty, speed=0.0)
        self.assertEqual(result, 0)
        self.assertTrue(self.score.failed)

    def test_missing_both_pockets_scores_zero(self):
        self.stage()
        self.release()
        result = self.score.step(
            (CHUTE_X, TRAY_Y, TRAY_FLOOR + BALL_RADIUS), self.open, CHUTE_X
        )
        self.assertEqual(result, 0)
        self.assertTrue(self.score.failed)

    def test_fresh_trial_resets_all_history(self):
        self.score.step(track_ball(self.start), self.open, 0.30)
        self.assertTrue(self.score.failed)
        self.score = ReleaseScore(self.trial)
        self.stage()


class TimingTests(unittest.TestCase):
    def test_plan_leads_the_empty_pocket_in_every_trial(self):
        for name, trial in TRIALS.items():
            with self.subTest(trial=name):
                release = demo_lift_time(trial, "timed")
                predicted_arrival = release + LIFT_TO_RELEASE_S + ROLL_AND_DROP_S
                x = (
                    trial.world_x(trial.joint_target(predicted_arrival))
                    + trial.empty * POCKET_OFFSET
                )
                self.assertAlmostEqual(x, CHUTE_X)

    def test_reactive_release_would_miss_the_empty_pocket(self):
        for name, trial in TRIALS.items():
            with self.subTest(trial=name):
                release = demo_lift_time(trial, "aligned")
                predicted_arrival = release + LIFT_TO_RELEASE_S + ROLL_AND_DROP_S
                x = (
                    trial.world_x(trial.joint_target(predicted_arrival))
                    + trial.empty * POCKET_OFFSET
                )
                self.assertGreater(abs(x - CHUTE_X), 0.04)

    def test_swapping_occupancy_invalidates_the_default_open_loop_release(self):
        trial = TRIALS["B"]
        release = demo_lift_time(trial, "fixed")
        time = release + LIFT_TO_RELEASE_S + ROLL_AND_DROP_S
        x = trial.world_x(trial.joint_target(time)) + trial.occupied * POCKET_OFFSET
        self.assertAlmostEqual(x, CHUTE_X)


if __name__ == "__main__":
    unittest.main()
