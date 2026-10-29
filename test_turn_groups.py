"""Turn strength selection through the real camera loop and UART packets."""
from types import SimpleNamespace
import unittest

from test_main import load_robot
from test_tracking import GrayImage, run_images


class TurnGroupTests(unittest.TestCase):
    def packets_for(self, xs, *, settings=None, wide=None, recovered=None):
        def observation(robot, sent):
            points = tuple((x, y) if x is not None else None
                           for x, y in zip(xs, (105, 75, 45)))
            return robot.TrackObservation(points, wide, recovered)
        return run_images(lambda now, sent: GrayImage(), packet_limit=2,
                          settings=settings, observation_factory=observation)

    def test_parallel_offset_uses_original_turns_in_both_directions(self):
        for xs, expected in (((40, 40, 40), 3), ((120, 120, 120), 4)):
            with self.subTest(xs=xs):
                packets = self.packets_for(xs)
                self.assertEqual([p[4] for _, p in packets], [0, expected])

    def test_near_far_curve_uses_large_turns_in_both_directions(self):
        for xs, expected in (((60, 40, 20), 7), ((100, 120, 140), 8)):
            with self.subTest(xs=xs):
                packets = self.packets_for(xs)
                self.assertEqual(packets[-1][1], bytes((0x55, 0x55, 5, 6, expected, 1, 0)))

    def test_middle_bend_selects_large_turn_even_with_aligned_endpoints(self):
        packets = self.packets_for((40, 50, 40))
        self.assertEqual([p[4] for _, p in packets], [0, 7])

    def test_shape_thresholds_are_inclusive_for_small_corrections(self):
        for xs, expected in (((50, 43, 36), 3), ((50, 42.5, 35), 7),
                             ((40, 48, 40), 3), ((40, 49, 40), 7)):
            with self.subTest(xs=xs):
                self.assertEqual(self.packets_for(xs)[-1][1][4], expected)

    def test_uncertain_or_contaminated_curve_uses_original_turn(self):
        for xs, options in (
            ((60, None, 20), {}),
            ((None, 40, 20), {}),
            ((60, 40, 20), {"wide": (False, True, False)}),
            ((60, 40, 20), {"recovered": (False, True, False)}),
        ):
            with self.subTest(xs=xs, options=options):
                self.assertEqual(self.packets_for(xs, **options)[-1][1][4], 3)

    def test_reverse_turns_swaps_direction_at_both_strengths(self):
        for xs, expected in (((40, 40, 40), 4), ((120, 120, 120), 3),
                             ((60, 40, 20), 8), ((100, 120, 140), 7)):
            with self.subTest(xs=xs):
                self.assertEqual(self.packets_for(xs, settings={"REVERSE_TURNS": True})[-1][1][4], expected)

    def test_loop_switches_from_small_to_large_then_recovers_forward(self):
        def observation(robot, sent):
            xs = (40, 40, 40) if len(sent) < 2 else (60, 40, 20) if len(sent) < 3 else (80, 55, 30)
            return robot.TrackObservation(tuple(zip(xs, (105, 75, 45))))
        packets = run_images(lambda now, sent: GrayImage(), packet_limit=4,
                             observation_factory=observation)
        self.assertEqual([p[4] for _, p in packets], [0, 3, 7, 1])
        self.assertGreaterEqual(packets[3][0] - packets[2][0], 790)

    def test_large_turn_waits_for_actual_group_duration_and_settling(self):
        for group, moving_until, ready_at in ((7, 1640, 1800), (8, 1635, 1795)):
            with self.subTest(group=group):
                robot, namespace = load_robot()
                namespace["TURN_SETTLE_MS"] = 160
                namespace["time"] = SimpleNamespace(ticks_diff=lambda now, before: now - before)
                controller = robot.GaitController()
                controller.mark_sent(group, 1000)
                self.assertTrue(controller.discard_observation(moving_until - 1))
                self.assertFalse(controller.discard_observation(moving_until))
                self.assertIsNone(controller.next_action(1, ready_at - 1))
                self.assertEqual(controller.next_action(1, ready_at), 1)

    def test_large_turn_durations_must_be_positive(self):
        for group in (7, 8):
            with self.subTest(group=group):
                robot, namespace = load_robot()
                namespace["ACTION_TIME_MS"] = dict(namespace["ACTION_TIME_MS"], **{})
                namespace["ACTION_TIME_MS"][group] = 0
                with self.assertRaises(ValueError):
                    robot.validate_config()


if __name__ == "__main__":
    unittest.main()
