import os
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch


SOURCE = Path(os.environ.get("ROBOT_MAIN", Path(__file__).with_name("main.py")))


def load_robot():
    namespace = {"__name__": "robot_under_test"}
    exec(compile(SOURCE.read_bytes(), str(SOURCE), "exec"), namespace)
    robot = SimpleNamespace(**namespace)
    return robot, namespace


class EndSimulation(Exception):
    pass


def run_robot(near_after_turn, far_after_turn, packet_limit=3,
              initial_line=(40, 30), turn_settle_ms=None, stand_settle_ms=None):
    robot, namespace = load_robot()
    packets = []
    elapsed = [0]
    frames = [0]

    def snapshot():
        frames[0] += 1
        elapsed[0] += 50
        if frames[0] > 200:
            raise EndSimulation()
        return SimpleNamespace(draw_string=lambda *args, **kwargs: None)

    def write(packet):
        packets.append((elapsed[0], bytes(packet)))
        if len(packets) >= packet_limit:
            raise EndSimulation()
        return len(packet)

    sensor = SimpleNamespace(
        GRAYSCALE=1, QQVGA=2, snapshot=snapshot,
        reset=lambda: None,
        set_pixformat=lambda value: None,
        set_framesize=lambda value: None,
        set_hmirror=lambda value: None,
        set_vflip=lambda value: None,
        skip_frames=lambda **kwargs: None,
        set_auto_gain=lambda value: None,
        set_auto_whitebal=lambda value: None,
        set_auto_exposure=lambda value: None,
    )
    namespace["ENABLE_MOTION"] = True
    if turn_settle_ms is not None:
        namespace["TURN_SETTLE_MS"] = turn_settle_ms
    if stand_settle_ms is not None:
        namespace["STAND_SETTLE_MS"] = stand_settle_ms
    namespace["time"] = SimpleNamespace(
        ticks_ms=lambda: elapsed[0],
        ticks_diff=lambda now, before: now - before,
        sleep_ms=lambda duration: None,
    )
    namespace["detect_line"] = lambda image: (
        initial_line if len(packets) < 2 else (near_after_turn, far_after_turn)
    )
    with patch.dict(sys.modules, {
        "sensor": sensor,
        "pyb": SimpleNamespace(UART=lambda *args, **kwargs: SimpleNamespace(write=write)),
    }), patch("builtins.print"):
        try:
            robot.run()
        except EndSimulation:
            pass
    return packets


class TurnRecoveryTests(unittest.TestCase):
    def test_real_loop_advances_when_near_line_is_centered_after_turn(self):
        # Far x=30 still requests LEFT with the old weighted rule (-17.5 px).
        packets = run_robot(80, 30)
        self.assertEqual([packet[4] for _, packet in packets], [0, 3, 1])

    def test_real_loop_advances_after_a_right_turn_too(self):
        packets = run_robot(80, 130, initial_line=(120, 130))
        self.assertEqual([packet[4] for _, packet in packets], [0, 4, 1])

    def test_recovery_does_not_permanently_disable_curve_detection(self):
        packets = run_robot(80, 30, packet_limit=5)
        self.assertEqual([packet[4] for _, packet in packets], [0, 3, 1, 0, 3])

    def test_real_loop_waits_for_both_rois_after_turn(self):
        packets = run_robot(80, None)
        self.assertEqual([packet[4] for _, packet in packets], [0, 3])

    def test_missing_near_roi_cannot_trigger_recovery(self):
        packets = run_robot(None, 30)
        self.assertEqual([packet[4] for _, packet in packets], [0, 3])

    def test_loop_does_not_accumulate_direction_while_turn_is_moving(self):
        packets = run_robot(80, 30, turn_settle_ms=0)
        # A 750 ms turn must be followed by >=100 ms of fresh stable observations.
        self.assertGreaterEqual(packets[2][0] - packets[1][0], 850)

    def test_loop_does_not_accumulate_direction_while_stand_is_moving(self):
        packets = run_robot(80, 30, stand_settle_ms=0)
        # A 1000 ms stand must be followed by >=100 ms of fresh observations.
        self.assertGreaterEqual(packets[1][0] - packets[0][0], 1100)

    def test_loop_overlaps_turn_settling_and_stability_without_early_dispatch(self):
        packets = run_robot(80, 30)
        gap = packets[2][0] - packets[1][0]
        self.assertGreaterEqual(gap, 930)  # Full 750 ms action plus 180 ms settling.
        self.assertLess(gap, 1030)  # No extra 100 ms stable-window wait afterward.

    def test_loop_respects_full_stand_settling_before_first_turn(self):
        packets = run_robot(80, 30)
        self.assertGreaterEqual(packets[1][0] - packets[0][0], 1150)

    def test_real_loop_keeps_turning_when_near_line_is_still_off_center(self):
        packets = run_robot(40, 30)
        self.assertEqual([packet[4] for _, packet in packets], [0, 3, 3])

    def test_center_tolerance_has_symmetric_inclusive_boundaries(self):
        for near, far, expected in ((68, 20, 1), (67, 20, 3),
                                    (92, 140, 1), (93, 140, 4)):
            with self.subTest(near=near, far=far):
                robot, _ = load_robot()
                action, _ = robot.choose_action(near, far, after_turn=True)
                self.assertEqual(action, expected)

    def test_regular_tracking_keeps_the_original_far_curve_anticipation(self):
        robot, _ = load_robot()
        self.assertEqual(robot.choose_action(80, 30)[0], 3)
        self.assertEqual(robot.choose_action(80, 130)[0], 4)
        self.assertEqual(robot.choose_action(80, 80)[0], 1)

    def test_reverse_turns_still_swaps_turns_but_never_forward(self):
        robot, namespace = load_robot()
        namespace["REVERSE_TURNS"] = True
        self.assertEqual(robot.choose_action(40, 30, after_turn=True)[0], 4)
        self.assertEqual(robot.choose_action(120, 130, after_turn=True)[0], 3)
        self.assertEqual(robot.choose_action(80, 30, after_turn=True)[0], 1)

    def test_invalid_recovery_tolerance_is_rejected(self):
        robot, namespace = load_robot()
        for invalid in (0, 80):
            with self.subTest(value=invalid):
                namespace["POST_TURN_CENTER_THRESHOLD"] = invalid
                with self.assertRaises(ValueError):
                    robot.validate_config()

    def test_turn_observations_start_only_after_the_full_action(self):
        robot, namespace = load_robot()
        namespace["time"] = SimpleNamespace(ticks_diff=lambda now, before: now - before)
        for action in (3, 4):
            with self.subTest(action=action):
                controller = robot.GaitController()
                controller.mark_sent(action, 1000)
                self.assertTrue(controller.discard_observation(1749))
                self.assertFalse(controller.discard_observation(1750))
                self.assertIsNone(controller.next_action(1, 1929))
                self.assertEqual(controller.next_action(1, 1930), 1)

    def test_stand_observations_overlap_settle_without_early_dispatch(self):
        robot, namespace = load_robot()
        namespace["time"] = SimpleNamespace(ticks_diff=lambda now, before: now - before)
        controller = robot.GaitController()
        controller.mark_sent(0, 1000)
        self.assertTrue(controller.discard_observation(1999))
        self.assertFalse(controller.discard_observation(2000))
        self.assertIsNone(controller.next_action(1, 2149))
        self.assertEqual(controller.next_action(1, 2150), 1)


if __name__ == "__main__":
    unittest.main()
