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
        self.assertAlmostEqual(target, 86541.975)

    def test_keeps_normal_quarter_second_batches(self):
        target = motion_queuing._calc_step_gen_target(
            last_step_gen_time=100.700,
            est_print_time=100.000,
            aggr_sg_time=102.000,
        )
        self.assertAlmostEqual(target, 100.950)

    def test_respects_available_motion_horizon(self):
        target = motion_queuing._calc_step_gen_target(
            last_step_gen_time=100.050,
            est_print_time=100.000,
            aggr_sg_time=100.600,
        )
        self.assertAlmostEqual(target, 100.600)

    def test_does_not_move_generation_time_backwards(self):
        target = motion_queuing._calc_step_gen_target(
            last_step_gen_time=101.100,
            est_print_time=100.000,
            aggr_sg_time=102.000,
        )
        self.assertAlmostEqual(target, 101.100)


if __name__ == "__main__":
    unittest.main()
