"""Desktop fixtures: real thresholded pixels, hardware-only OpenMV substitutes."""
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import patch

from test_main import EndSimulation, load_robot


class GrayImage:
    def __init__(self):
        self.pixels = [bytearray([255] * 160) for _ in range(120)]
        self.drawn = False

    def rect(self, x, y, w, h, value=0):
        for row in range(y, y + h):
            self.pixels[row][x:x + w] = bytes([value] * w)
        return self

    def find_blobs(self, thresholds, *, roi, pixels_threshold, area_threshold, merge):
        # Drawing before the last scan would contaminate the narrow-band rescan.
        if self.drawn:
            raise AssertionError("Detection read pixels after overlay drawing")
        x, y, w, h = roi
        lo, hi = thresholds[0]
        black = {(px, py) for py in range(y, y + h) for px in range(x, x + w)
                 if lo <= self.pixels[py][px] <= hi}
        blobs = []
        while black:
            seed = black.pop()
            pending, component = [seed], [seed]
            while pending:
                px, py = pending.pop()
                for neighbor in ((px - 1, py), (px + 1, py), (px, py - 1), (px, py + 1)):
                    if neighbor in black:
                        black.remove(neighbor)
                        pending.append(neighbor)
                        component.append(neighbor)
            xs, ys = zip(*component)
            bw, bh = max(xs) - min(xs) + 1, max(ys) - min(ys) + 1
            if len(component) < pixels_threshold or bw * bh < area_threshold:
                continue
            blobs.append(SimpleNamespace(
                x=min(xs), y=min(ys), w=bw, h=bh,
                cx=round(sum(xs) / len(xs)), cy=round(sum(ys) / len(ys)),
                rect=(min(xs), min(ys), bw, bh), pixels=len(component)))
        return blobs

    def draw_rectangle(self, *args, **kwargs):
        self.drawn = True

    draw_cross = draw_rectangle
    draw_line = draw_rectangle
    draw_string = draw_rectangle


def centered_line():
    return GrayImage().rect(77, 0, 7, 120)


def run_images(frame_factory, *, frame_ms=20, max_frames=300, packet_limit=None,
               settings=None, observation_factory=None, short_write=False):
    """Exercise the real detector, filters and controller; replace only camera/UART/clock."""
    robot, namespace = load_robot()
    elapsed, frames, packets = [0], [0], []

    def snapshot():
        frames[0] += 1
        if frames[0] > max_frames:
            raise EndSimulation()
        elapsed[0] += frame_ms
        return frame_factory(elapsed[0], packets)

    def write(packet):
        packets.append((elapsed[0], bytes(packet)))
        if packet_limit is not None and len(packets) >= packet_limit:
            raise EndSimulation()
        return len(packet) - 1 if short_write else len(packet)

    sensor = SimpleNamespace(
        GRAYSCALE=1, QQVGA=2, snapshot=snapshot, reset=lambda: None,
        set_pixformat=lambda value: None, set_framesize=lambda value: None,
        set_hmirror=lambda value: None, set_vflip=lambda value: None,
        skip_frames=lambda **kwargs: None, set_auto_gain=lambda value: None,
        set_auto_whitebal=lambda value: None, set_auto_exposure=lambda value: None,
    )
    namespace["ENABLE_MOTION"] = True
    namespace.update(settings or {})
    namespace["time"] = SimpleNamespace(
        ticks_ms=lambda: elapsed[0], ticks_diff=lambda now, before: now - before,
        sleep_ms=lambda duration: None,
    )
    if observation_factory is not None:
        namespace["detect_line"] = lambda image: observation_factory(robot, packets)
    with patch.dict(sys.modules, {
        "sensor": sensor,
        "pyb": SimpleNamespace(UART=lambda *args, **kwargs: SimpleNamespace(write=write)),
    }), patch("builtins.print"):
        try:
            robot.run()
        except EndSimulation:
            pass
    return packets


class CrossingDetectionTests(unittest.TestCase):
    def setUp(self):
        self.robot, self.namespace = load_robot()

    def assert_forward(self, img):
        track = self.robot.detect_line(img)
        self.assertIsNotNone(track.near_x)
        self.assertEqual(self.robot.choose_action(track.near_x, track.far_x)[0], 1)
        self.assertAlmostEqual(track.near_x, 80, delta=1)
        self.assertAlmostEqual(track.far_x, 80, delta=1)
        return track

    def test_crossing_moves_from_far_through_middle_to_near_without_losing_line(self):
        for y in (30, 35, 45, 60, 65, 75, 90, 95, 105, 115):
            with self.subTest(y=y):
                self.assert_forward(centered_line().rect(0, y, 160, 5))

    def test_one_fully_obscured_roi_uses_the_other_two(self):
        for index, y in enumerate((95, 65, 35)):
            with self.subTest(y=y):
                track = self.assert_forward(centered_line().rect(0, y, 160, 20))
                self.assertTrue(track.wide[index])
                self.assertEqual(sum(p is not None for p in track.points), 2)

    def test_two_interfered_rois_recover_longitudinal_line_from_clean_edges(self):
        track = self.assert_forward(centered_line().rect(0, 35, 160, 12).rect(0, 65, 160, 12))
        self.assertTrue(any(track.recovered))

    def test_full_shadow_and_blank_image_have_no_direction(self):
        for image in (GrayImage(), GrayImage().rect(0, 0, 160, 120)):
            track = self.robot.detect_line(image)
            self.assertEqual(self.robot.choose_action(track.near_x, track.far_x), (None, None))

    def test_one_line_roi_and_two_unrecoverable_bands_do_not_authorize_motion(self):
        image = GrayImage().rect(77, 95, 7, 20).rect(0, 35, 160, 20).rect(0, 65, 160, 20)
        track = self.robot.detect_line(image)
        self.assertIsNone(track.near_x)

    def test_wide_horizontal_bands_alone_are_not_longitudinal_line(self):
        image = GrayImage().rect(0, 37, 160, 8).rect(0, 67, 160, 8).rect(0, 97, 160, 8)
        track = self.robot.detect_line(image)
        self.assertIsNone(track.near_x)

    def test_parallel_line_does_not_steal_tracking_at_a_crossing(self):
        self.assert_forward(centered_line().rect(117, 0, 7, 120).rect(0, 95, 160, 20))

    def test_noise_below_pixel_threshold_does_not_become_a_line(self):
        image = GrayImage().rect(80, 100, 2, 4).rect(80, 70, 2, 4).rect(80, 40, 2, 4)
        self.assertIsNone(self.robot.detect_line(image).near_x)

    def test_narrow_band_rescan_keeps_a_two_pixel_wide_track(self):
        image = GrayImage().rect(79, 0, 2, 120).rect(0, 35, 160, 12).rect(0, 65, 160, 12)
        self.assert_forward(image)

    def test_unrelated_centers_do_not_form_a_track(self):
        image = GrayImage().rect(7, 95, 7, 20).rect(147, 65, 7, 20)
        self.assertIsNone(self.robot.detect_line(image).near_x)


class TrackGeometryTests(unittest.TestCase):
    def setUp(self):
        self.robot, self.namespace = load_robot()

    def test_each_missing_roi_is_interpolated_or_extrapolated(self):
        for points in (((80, 105), None, (50, 45)),
                       (None, (65, 75), (50, 45)),
                       ((80, 105), (65, 75), None)):
            with self.subTest(points=points):
                track = self.robot.TrackObservation(points)
                self.assertEqual((track.near_x, track.far_x), (80, 50))

    def test_extrapolation_outside_frame_is_rejected(self):
        track = self.robot.TrackObservation((None, (145, 75), (120, 45)))
        self.assertIsNone(track.near_x)

    def test_overlarge_center_jump_is_rejected(self):
        track = self.robot.TrackObservation(((10, 105), None, (90, 45)))
        self.assertIsNone(track.near_x)


class RoadModeTests(unittest.TestCase):
    def setUp(self):
        self.robot, self.namespace = load_robot()
        self.namespace["time"] = SimpleNamespace(ticks_diff=lambda now, before: now - before)

    def track(self, near=80, middle=80, far=80, **kwargs):
        return self.robot.TrackObservation(((near, 105), (middle, 75), (far, 45)), **kwargs)

    def enter_straight(self, mode):
        self.assertEqual(mode.update(self.track(), 0), "CURVE")
        self.assertEqual(mode.update(self.track(), 50), "CURVE")
        self.assertEqual(mode.update(self.track(), 100), "STRAIGHT")

    def test_straight_requires_three_frames_and_100_ms(self):
        mode = self.robot.RoadModeFilter()
        for now in (0, 20, 40, 80):
            self.assertEqual(mode.update(self.track(), now), "CURVE")
        self.assertEqual(mode.update(self.track(), 100), "STRAIGHT")

    def test_hysteresis_avoids_switching_on_small_drift(self):
        mode = self.robot.RoadModeFilter()
        self.enter_straight(mode)
        self.assertEqual(mode.update(self.track(90, 85, 80), 140), "STRAIGHT")
        self.assertEqual(mode.update(self.track(93, 86, 80), 180), "CURVE")
        self.assertEqual(mode.update(self.track(), 220), "CURVE")
        self.assertEqual(mode.update(self.track(), 270), "CURVE")
        self.assertEqual(mode.update(self.track(), 320), "STRAIGHT")

    def test_middle_point_detects_curve_even_when_endpoints_are_aligned(self):
        mode = self.robot.RoadModeFilter()
        self.enter_straight(mode)
        self.assertEqual(mode.update(self.track(80, 89, 80), 140), "CURVE")

    def test_interference_or_missing_frame_disables_fast_mode(self):
        for track, expected in (
            (self.track(wide=(True, False, False)), "CROSS"),
            (self.track(recovered=(True, False, False)), "CROSS"),
            (self.robot.TrackObservation(((80, 105), None, (80, 45))), "CURVE"),
            (self.robot.TrackObservation(), "LOST"),
        ):
            with self.subTest(expected=expected, points=track.points):
                mode = self.robot.RoadModeFilter()
                self.enter_straight(mode)
                self.assertEqual(mode.update(track, 140), expected)

    def test_gap_over_150_ms_clears_fast_mode_and_stability(self):
        mode = self.robot.RoadModeFilter()
        self.enter_straight(mode)
        self.assertEqual(mode.update(self.track(), 251), "CURVE")

    def test_fast_handoff_only_applies_to_confirmed_forward_on_straight(self):
        for mode, action, ready, expected in (
            ("STRAIGHT", 1, 2340, 1), ("CURVE", 1, 2380, 1),
            ("CROSS", 1, 2380, 1), ("STRAIGHT", 3, 2380, 3),
        ):
            with self.subTest(mode=mode, action=action):
                controller = self.robot.GaitController()
                controller.mark_sent(1, 1000)
                self.assertIsNone(controller.next_action(action, ready - 1, mode))
                self.assertEqual(controller.next_action(action, ready, mode), expected)

    def test_new_curve_revokes_the_shorter_margin_in_the_current_walk(self):
        controller = self.robot.GaitController()
        controller.mark_sent(1, 0)
        self.assertIsNone(controller.next_action(1, 1339, "STRAIGHT"))
        self.assertIsNone(controller.next_action(1, 1340, "CURVE"))
        self.assertEqual(controller.next_action(1, 1380, "CURVE"), 1)


class CameraLoopTests(unittest.TestCase):
    def test_clean_straight_uses_1340_ms_forward_intervals(self):
        packets = run_images(lambda now, sent: centered_line(), packet_limit=4)
        self.assertEqual([p[4] for _, p in packets], [0, 1, 1, 1])
        self.assertEqual([packets[i][0] - packets[i - 1][0] for i in (2, 3)], [1340, 1340])
        self.assertEqual(packets[1][1], bytes((0x55, 0x55, 0x05, 0x06, 1, 1, 0)))

    def test_crossing_keeps_walking_at_normal_margin_without_stand(self):
        def frame(now, packets):
            y = (35, 65, 95)[(now // 200) % 3]
            return centered_line().rect(0, y, 160, 20)
        packets = run_images(frame, packet_limit=4)
        self.assertEqual([p[4] for _, p in packets], [0, 1, 1, 1])
        self.assertEqual([packets[i][0] - packets[i - 1][0] for i in (2, 3)], [1380, 1380])

    def test_real_line_loss_finishes_current_walk_then_stands_only_once(self):
        packets = run_images(lambda now, sent: centered_line() if len(sent) < 2 else GrayImage())
        self.assertEqual([p[4] for _, p in packets], [0, 1, 0])
        self.assertEqual(packets[2][0] - packets[1][0], 1580)

    def test_slow_frames_cannot_launch_motion(self):
        packets = run_images(lambda now, sent: centered_line(), frame_ms=160, max_frames=30)
        self.assertEqual([p[4] for _, p in packets], [0])

    def test_predicted_near_center_cannot_trigger_post_turn_forward_override(self):
        def observation(robot, sent):
            points = ((40, 105), (35, 75), (30, 45)) if len(sent) < 2 else (None, (55, 75), (30, 45))
            return robot.TrackObservation(points)
        packets = run_images(lambda now, sent: centered_line(), packet_limit=3,
                             observation_factory=observation)
        self.assertEqual([p[4] for _, p in packets], [0, 3, 3])
        self.assertGreaterEqual(packets[2][0] - packets[1][0], 910)
        self.assertLess(packets[2][0] - packets[1][0], 930)

    def test_preview_never_transmits_actions(self):
        packets = run_images(lambda now, sent: centered_line(), max_frames=15,
                             settings={"ENABLE_MOTION": False})
        self.assertEqual(packets, [])

    def test_uart_short_write_aborts_instead_of_advancing_controller(self):
        with self.assertRaisesRegex(OSError, "UART short write"):
            run_images(lambda now, sent: centered_line(), short_write=True)


class ConfigAndClockTests(unittest.TestCase):
    def test_invalid_roi_and_speed_settings_are_rejected(self):
        for name, value in (
            ("ROIS", ((0, 95, 160, 20), (0, 90, 160, 20), (0, 35, 160, 20))),
            ("ROIS", ((0, 95, 160, 20), (0, 35, 160, 20))),
            ("RECOVERY_STRIP_HEIGHT", 0), ("RECOVERY_STRIP_HEIGHT", 11),
            ("STRAIGHT_MARGIN_MS", -1), ("STRAIGHT_MARGIN_MS", 81),
            ("STRAIGHT_EXIT_LIMITS", (7, 14, 8)),
            ("STRAIGHT_STABLE_FRAMES", 0), ("STRAIGHT_STABLE_MS", -1),
        ):
            with self.subTest(name=name, value=value):
                robot, namespace = load_robot()
                namespace[name] = value
                with self.assertRaises(ValueError):
                    robot.validate_config()

    def test_wraparound_clock_preserves_full_action_and_stability(self):
        robot, namespace = load_robot()
        namespace["time"] = SimpleNamespace(ticks_diff=lambda a, b: ((a - b + 32768) % 65536) - 32768)
        controller = robot.GaitController()
        controller.mark_sent(1, 65500)
        self.assertIsNone(controller.next_action(3, 1343))
        self.assertEqual(controller.next_action(3, 1344), 3)
        direction = robot.LineFilter()
        self.assertIsNone(direction.update(1, 65520))
        self.assertEqual(direction.update(1, 24), 1)


if __name__ == "__main__":
    unittest.main()
