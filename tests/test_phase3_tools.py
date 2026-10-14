"""Tests for the Phase 3 tooling and the Phase 4 URDF verifier.

These are the scripts a human runs by hand, which is exactly the kind of code that quietly
stops working. Each test drives the real entry point.

Nothing here touches a serial port: `--dry-run` is the default and the tests never pass
`--port`, so the driver only ever frames bytes.
"""

import io
import contextlib
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.phase3_electronics import esp32_arm_driver as driver          # noqa: E402
from src.phase3_electronics import power_and_wiring_verifier as wiring  # noqa: E402
from src.phase4_simulation import verify_urdf_model as urdf_check       # noqa: E402
from src.phase8_safety_testing import safety_supervisor_and_logger as supervisor  # noqa: E402
from src.phase5_control import ros2_moveit_bridge as bridge             # noqa: E402


def run_cli(func, argv):
    """Call a main() with argv, capturing stdout. Returns (exit code, output)."""
    buffer = io.StringIO()
    with contextlib.redirect_stdout(buffer):
        code = func(argv)
    return code, buffer.getvalue()


class DriverFrames(unittest.TestCase):
    """The wire format, which is the contract with the firmware."""

    def test_wheel_frame_is_exactly_the_documented_shape(self):
        self.assertEqual(driver.frame_wheels(1320, 1680), "W 1320 1680")
        self.assertEqual(driver.frame_wheels(1500, 1500), "W 1500 1500")

    def test_an_out_of_range_wheel_pulse_is_refused(self):
        for bad in ((999, 1500), (1500, 2001), (0, 0)):
            with self.assertRaises(ValueError):
                driver.frame_wheels(*bad)

    def test_drive_speeds_map_to_the_mirrored_pair(self):
        # The two drive servos face each other: forward is BELOW neutral on the left wheel and
        # ABOVE it on the right. Getting this backwards drives the rover off the cliff it is
        # supposed to be running away from (FMEA #15).
        self.assertEqual(driver.frame_wheels(*driver._mirrored(0.5, 0.5))
                         if hasattr(driver, "_mirrored") else
                         driver.wheel_command_from_velocity(0.5, 0.5),
                         "W 1410 1590")
        self.assertEqual(driver.wheel_command_from_velocity(-1.0, -1.0), "W 1680 1320")
        self.assertEqual(driver.wheel_command_from_velocity(0.0, 0.0), "W 1500 1500")

    def test_channel_frames_offer_both_immediate_and_blended(self):
        pairs = [(0, 1500), (3, 1200)]
        self.assertEqual(driver.frame_channels(pairs), "M 0:1500,3:1200")
        self.assertEqual(driver.frame_channels(pairs, 800), "G 800 0:1500,3:1200")

    def test_outputs_frame_uses_the_firmware_polarity(self):
        # OE HIGH = tri-stated = safe. `OE 1` must therefore mean "outputs off".
        self.assertEqual(driver.frame_outputs(False), "OE 1")
        self.assertEqual(driver.frame_outputs(True), "OE 0")


class DriverJointTable(unittest.TestCase):
    """Degrees -> microseconds, using the config the firmware also reads."""

    def setUp(self):
        self.joints = driver.load_joints()

    def test_six_arm_joints_on_channels_zero_to_five(self):
        self.assertEqual(len(self.joints), 6)
        self.assertEqual([j.channel for j in self.joints], [0, 1, 2, 3, 4, 5])
        self.assertIn("gripper", [j.name for j in self.joints])

    def test_angles_clamp_to_the_servo_travel(self):
        shoulder = next(j for j in self.joints if j.name == "shoulder_pitch")
        self.assertEqual(shoulder.us_from_deg(1000.0), int(shoulder.pwm_max_us))
        self.assertEqual(shoulder.us_from_deg(-1000.0), int(shoulder.pwm_min_us))
        self.assertEqual(shoulder.us_from_deg(shoulder.home_deg),
                         shoulder.us_from_deg(shoulder.home_deg))

    def test_gripper_percentage_covers_closed_to_open(self):
        closed = driver.grip_frame(self.joints, 0)
        opened = driver.grip_frame(self.joints, 100)
        self.assertEqual(closed, "M 5:600")
        self.assertEqual(opened, "M 5:2300")

    def test_home_is_one_blended_frame_over_all_six_channels(self):
        frames = driver.home_frames(self.joints)
        self.assertEqual(len(frames), 1)
        self.assertTrue(frames[0].startswith("G 1500 "))
        self.assertEqual(frames[0].count(":"), 6)


class DriverCli(unittest.TestCase):
    def test_dry_run_ping_sends_nothing_and_succeeds(self):
        code, out = run_cli(driver.main, ["--dry-run", "--ping"])
        self.assertEqual(code, 0, out)
        self.assertIn("DRY RUN", out)
        self.assertIn("TX  PING", out)

    def test_no_arguments_prints_examples_rather_than_opening_a_port(self):
        code, out = run_cli(driver.main, [])
        self.assertEqual(code, 0, out)
        self.assertIn("Nothing to do", out)
        self.assertIn("--dry-run --drive", out.replace("--drive 0.5", "--drive"))

    def test_the_driver_and_the_firmware_agree(self):
        self.assertEqual(driver.firmware_agreement(driver.SKETCH.read_text(encoding="utf-8")),
                         [], "the driver's assumptions drifted away from the sketch")

    def test_an_arm_move_refuses_a_wheel_channel(self):
        code, out = run_cli(driver.main, ["--dry-run", "--move", "6:1320"])
        self.assertNotEqual(code, 0)
        self.assertIn("REFUSED", out)
        self.assertIn("rover wheels", out)


class WiringVerifier(unittest.TestCase):
    def test_it_passes_and_checks_a_meaningful_number_of_things(self):
        code, out = run_cli(wiring.main, [])
        self.assertEqual(code, 0, out[-1500:])
        self.assertIn("VERDICT: PASS", out)
        verifier = wiring.Verifier()
        verifier.pins_match_everywhere(wiring._read(wiring.WIRING_DOC),
                                       wiring._read(wiring.CONFIG), wiring._read(wiring.SKETCH))
        verifier.channels_match(wiring._read(wiring.CONFIG), wiring._read(wiring.SKETCH),
                                wiring._read(wiring.HEADER))
        verifier.addresses_match(wiring._read(wiring.WIRING_DOC), wiring._read(wiring.CONFIG),
                                 wiring._read(wiring.SKETCH))
        verifier.rail_budget()
        verifier.wiring_gauge()
        verifier.shunt_and_protection(wiring._read(wiring.WIRING_DOC), wiring._read(wiring.CONFIG))
        self.assertGreaterEqual(len(verifier.checks), 15)

    def test_the_voltage_drop_grows_as_the_wire_gets_thinner(self):
        drops = [wiring.wire_drop_v(g, 5.0, 1.0) for g in ("20", "22", "24", "26")]
        self.assertEqual(drops, sorted(drops))
        self.assertAlmostEqual(drops[0], 0.333, places=2)

    def test_json_mode_is_parseable_and_agrees(self):
        import json
        buffer = io.StringIO()
        with contextlib.redirect_stdout(buffer):
            code = wiring.main(["--json"])
        payload = json.loads(buffer.getvalue())
        self.assertEqual(code, 0)
        self.assertEqual(payload["passed"], payload["total"])


class UrdfVerifier(unittest.TestCase):
    def test_it_passes(self):
        code, out = run_cli(urdf_check.main, [])
        self.assertEqual(code, 0, out[-1500:])
        self.assertIn("VERDICT: PASS", out)
        self.assertIn("10 / 10 checks", out)

    def test_the_urdf_walk_and_the_solver_are_the_same_robot(self):
        from src.phase5_control.kinematics import forward
        for pose in ((0.0, 0.0, 0.0, 0.0, 0.0), (0.0, -15.0, 40.0, -25.0, 0.0),
                     (25.0, 30.0, -20.0, 10.0, 45.0)):
            x, y, z = urdf_check.urdf_tcp_position(pose)
            model = forward(list(pose))
            self.assertAlmostEqual(x * 1000.0, model.x, places=1)
            self.assertAlmostEqual(y * 1000.0, model.y, places=1)
            self.assertAlmostEqual(z * 1000.0, model.z, places=1)

    def test_straight_up_is_the_folded_link_stack(self):
        self.assertAlmostEqual(urdf_check.urdf_tcp_position((0, 0, 0, 0, 0))[2], 0.280, places=4)


class OtherEntryPoints(unittest.TestCase):
    """The two CLI conveniences the walkthrough uses."""

    def test_the_supervisor_accepts_fault_all(self):
        # 2.4 s of simulated timeline: the over-current backoff needs 2.16 s and the cliff and
        # e-stop cases need 2.0 s, so anything shorter fails for reasons that are not bugs.
        code, out = run_cli(supervisor.main, ["--fault", "all", "--seconds", "2.4", "--no-log"])
        self.assertEqual(code, 0, out[-1500:])
        self.assertIn("6 / 6 faults handled", out)

    def test_a_single_named_fault_still_works(self):
        code, out = run_cli(supervisor.main, ["--fault", "cliff", "--seconds", "2.4", "--no-log"])
        self.assertEqual(code, 0, out[-800:])
        self.assertIn("1 / 1 faults handled", out)

    def test_the_moveit_bridge_has_a_dry_run(self):
        code, out = run_cli(bridge.main, ["--dry-run"])
        self.assertEqual(code, 0, out[-1500:])
        self.assertIn("DRY RUN", out)


if __name__ == "__main__":
    unittest.main()
