#!/usr/bin/env python3
import pathlib
import sys
import types
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.modules.setdefault("chelper", types.ModuleType("chelper"))

from klippy.extras import motion_queuing


class StepGenerationTargetTests(unittest.TestCase):
    def test_catches_up_after_observed_k400_gap(self):
        target = motion_queuing._calc_step_gen_target(
            last_step_gen_time=86541.025,
            est_print_time=86540.975,
            aggr_sg_time=86542.813,
        )
        self.assertAlmostEqual(target, 86542.725)

    def test_covers_latest_k400_gap_with_available_motion(self):
        target = motion_queuing._calc_step_gen_target(
            last_step_gen_time=913.421,
            est_print_time=913.421,
            aggr_sg_time=915.084,
        )
        self.assertAlmostEqual(target, 915.084)

    def test_keeps_normal_quarter_second_batches(self):
        target = motion_queuing._calc_step_gen_target(
            last_step_gen_time=101.500,
            est_print_time=100.000,
            aggr_sg_time=102.000,
        )
        self.assertAlmostEqual(target, 101.750)

    def test_respects_available_motion_horizon(self):
        target = motion_queuing._calc_step_gen_target(
            last_step_gen_time=100.050,
            est_print_time=100.000,
            aggr_sg_time=100.600,
        )
        self.assertAlmostEqual(target, 100.600)

    def test_does_not_move_generation_time_backwards(self):
        target = motion_queuing._calc_step_gen_target(
            last_step_gen_time=101.800,
            est_print_time=100.000,
            aggr_sg_time=102.000,
        )
        self.assertAlmostEqual(target, 101.800)

    def test_margin_covers_observed_gap_and_serial_backlog(self):
        serial_backlog_time = 3801. / 25000.
        remaining = (motion_queuing.BGFLUSH_SG_LOW_TIME
                     - 1.098 - serial_backlog_time)
        self.assertGreater(remaining, 0.240)


class FlushWakeTimeTests(unittest.TestCase):
    def test_uses_absolute_monotonic_time_once(self):
        waketime = motion_queuing._calc_monotonic_waketime(
            eventtime=898.000,
            print_waketime=914.500,
            est_print_time=913.400,
        )
        self.assertAlmostEqual(waketime, 899.100)


if __name__ == "__main__":
    unittest.main()
