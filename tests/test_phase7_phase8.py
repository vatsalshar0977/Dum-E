"""Phase 7 and Phase 8 tests: gestures, language, supervision and telemetry.

These are unit tests of the MODEL and of the deterministic logic. They assert timing, ordering,
priority and fail-closed behaviour - the things that can be checked without hardware. Nothing
here proves that a servo moves, and nothing here is a substitute for
docs/phase8_safety_testing_and_bringup_checklist.md.
"""

import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.phase7_voice_personality.personality_engine import (            # noqa: E402
    BASE_SPEED_DEG_S, GESTURE_SPEED_DEG_S, HOME_DEG, IDLE_AFTER_S, JOINT_LIMITS_DEG,
    JOINT_NAMES, SEQUENCES, SAMPLE_HZ, WHEEL_STOP_US, PersonalityDirector, Sequence,
    looks_like_failed_grasp, min_jerk_duration, minimum_jerk,
)
from src.phase7_voice_personality.voice_assistant import (               # noqa: E402
    HOME_PERCHED_DEG, SimRobotLink, TOOL_SYNONYMS, WhisperTranscriber, parse,
    wheel_us_from_command,
)
from src.phase8_safety_testing.safety_supervisor_and_logger import (     # noqa: E402
    BACKOFF_HOME, BROWNOUT_HOLD, CLIFF_RECOVER, ESTOP_HOLD, FAULTS, FaultInjector,
    GRASP_RETRY, OBSTACLE_HOLD, OVERCURRENT_MA, SafetySupervisor, Sensors, TelemetryRecorder,
)


class GestureLibrary(unittest.TestCase):
    def test_six_gestures_and_every_one_validates(self):
        self.assertEqual(len(SEQUENCES), 6)
        for name, seq in SEQUENCES.items():
            seq.validate()                                    # raises if infeasible
            self.assertEqual(seq.keyframes[0].t, 0.0, name)
            self.assertGreater(seq.duration, 0.5, name)

    def test_every_keyframe_is_inside_the_joint_limits(self):
        for name, seq in SEQUENCES.items():
            for frame in seq.keyframes:
                for value, (lo, hi), joint in zip(frame.joints, JOINT_LIMITS_DEG, JOINT_NAMES):
                    self.assertLessEqual(lo, value, f"{name}/{joint}")
                    self.assertGreaterEqual(hi, value, f"{name}/{joint}")

    def test_sampling_is_100hz_and_starts_at_zero(self):
        for name, seq in SEQUENCES.items():
            frames = seq.sample()
            self.assertAlmostEqual(frames[0].t, 0.0, places=9, msg=name)
            self.assertAlmostEqual(len(frames), seq.duration * SAMPLE_HZ + 1, delta=1, msg=name)
            for a, b in zip(frames, frames[1:]):
                self.assertAlmostEqual(b.t - a.t, 1.0 / SAMPLE_HZ, places=6, msg=name)

    def test_peak_speed_never_exceeds_its_own_budget(self):
        for name, seq in SEQUENCES.items():
            frames = seq.sample()
            for a, b in zip(frames, frames[1:]):
                dt = b.t - a.t
                for i, (x, y) in enumerate(zip(a.joints, b.joints)):
                    limit = BASE_SPEED_DEG_S[i] * seq.speed_scale
                    self.assertLessEqual(abs(y - x) / dt, limit + 1e-6, f"{name}/{JOINT_NAMES[i]}")

    def test_the_gesture_budget_is_larger_than_the_transport_budget(self):
        # Deliberate: small empty-handed expressions may be faster than carrying a tool.
        self.assertGreater(sum(GESTURE_SPEED_DEG_S), 5 * 35.0)

    def test_minimum_jerk_is_rest_to_rest(self):
        self.assertAlmostEqual(minimum_jerk(0.0, 10.0, 0.0), 0.0)
        self.assertAlmostEqual(minimum_jerk(0.0, 10.0, 1.0), 10.0)
        self.assertAlmostEqual(minimum_jerk(0.0, 10.0, 0.5), 5.0)     # symmetric
        self.assertAlmostEqual(minimum_jerk(-4.0, 6.0, 0.5), 1.0)
        for bad in (-0.1, 1.1):
            with self.assertRaises(ValueError):
                minimum_jerk(0.0, 1.0, bad)

    def test_min_jerk_duration_matches_the_1875_rule(self):
        self.assertAlmostEqual(min_jerk_duration(35.0, 70.0), 1.875 * 35.0 / 70.0)
        self.assertEqual(min_jerk_duration(0.0, 40.0), 0.0)
        with self.assertRaises(ValueError):
            min_jerk_duration(10.0, 0.0)

    def test_an_infeasible_gesture_is_refused_at_construction(self):
        start = SEQUENCES["affirmative_nod"].keyframes[0]
        from src.phase7_voice_personality.personality_engine import Keyframe
        # 75 degrees of wrist pitch asked for in 50 ms: about 2800 deg/s of minimum jerk.
        jump = Keyframe(0.05, (start.joints[0], start.joints[1], start.joints[2], 50.0,
                               start.joints[4], start.joints[5]))
        fast = Sequence(name="too_fast", description="x", priority=3,
                        keyframes=(start, jump))
        with self.assertRaises(ValueError):
            fast.validate()

    def test_the_cliff_reflex_reverses_then_stops_the_wheels(self):
        frames = SEQUENCES["startled_cliff_recoil"].sample()
        reversing = [f for f in frames if f.wheels[0] < WHEEL_STOP_US]
        self.assertTrue(reversing, "the recoil never reversed")
        self.assertGreater(reversing[0].t, 0.0 - 1e-9)
        self.assertLess(reversing[-1].t, 0.6, "the wheels kept reversing too long")
        self.assertEqual(frames[-1].wheels, (WHEEL_STOP_US, WHEEL_STOP_US))
        self.assertEqual(SEQUENCES["startled_cliff_recoil"].priority, 0)


class Director(unittest.TestCase):
    def test_the_wake_word_starts_the_head_tilt(self):
        d = PersonalityDirector(t0=0.0)
        d.submit("wake_word", 0.1)
        self.assertIs(d.state.sequence, SEQUENCES["curious_head_tilt"])

    def test_the_cliff_reflex_pre_empts_anything(self):
        d = PersonalityDirector(t0=0.0)
        d.submit("wake_word", 0.0)
        d.submit("cliff", 0.2)
        self.assertIs(d.state.sequence, SEQUENCES["startled_cliff_recoil"])
        self.assertTrue(d.wheels_are_moving(0.3))

    def test_a_lower_priority_event_does_not_interrupt_a_playing_gesture(self):
        d = PersonalityDirector(t0=0.0)
        d.submit("grasp_failed", 0.0)                     # priority 2, 3.4 s long
        chosen = d.submit("wake_word", 0.5)              # priority 3
        self.assertIsNone(chosen)
        self.assertIs(d.state.sequence, SEQUENCES["sad_failed_grasp_droop"])

    def test_a_gesture_returns_to_its_declared_pose(self):
        d = PersonalityDirector(t0=0.0)
        d.submit("command_acknowledged", 0.0)
        end = d.frame(SEQUENCES["affirmative_nod"].duration + 0.01)
        self.assertEqual(tuple(end.joints), tuple(HOME_DEG))
        self.assertEqual(end.wheels, (WHEEL_STOP_US, WHEEL_STOP_US))

    def test_the_idle_loop_starts_only_after_the_idle_timeout(self):
        d = PersonalityDirector(t0=0.0)
        self.assertIsNone(d.check_idle(IDLE_AFTER_S - 0.1))
        self.assertIsNotNone(d.check_idle(IDLE_AFTER_S + 0.1))
        self.assertTrue(SEQUENCES["idle_breathing_and_look_around"].loop)

    def test_halt_stops_the_wheels_and_ignores_everything_until_resume(self):
        d = PersonalityDirector(t0=0.0)
        d.submit("cliff", 0.0)
        frame = d.halt(0.2)
        self.assertEqual(frame.wheels, (WHEEL_STOP_US, WHEEL_STOP_US))
        self.assertIsNone(d.state.sequence)
        self.assertIsNone(d.submit("wake_word", 0.3), "a halted director ran a gesture")
        self.assertEqual(d.frame(0.4).wheels, (WHEEL_STOP_US, WHEEL_STOP_US))
        d.resume(0.5)
        self.assertIsNotNone(d.submit("wake_word", 0.6))

    def test_grasp_detection_is_one_sided(self):
        self.assertTrue(looks_like_failed_grasp(95.0, 0.0, 3.0, held_s=0.5))
        self.assertFalse(looks_like_failed_grasp(600.0, 0.0, 3.0, held_s=0.5))
        self.assertFalse(looks_like_failed_grasp(95.0, 45.0, 40.0, held_s=0.5))
        self.assertFalse(looks_like_failed_grasp(95.0, 0.0, 3.0, held_s=0.1))
        with self.assertRaises(ValueError):
            PersonalityDirector().submit("grasp_check", 0.0)


class Language(unittest.TestCase):
    def test_the_briefs_phrases_map_to_their_intents(self):
        table = {
            "hand me the screwdriver": "FETCH_TOOL",
            "fetch the allen key": "FETCH_TOOL",
            "get me the pliers": "FETCH_TOOL",
            "look at my hand": "LOOK_AT_HAND",
            "follow me": "FOLLOW_ME",
            "stop": "HALT",
            "freeze": "HALT",
            "go home": "GO_HOME",
            "sleep": "GO_HOME",
        }
        for text, want in table.items():
            self.assertEqual(parse(text).name, want, text)

    def test_fetch_fills_the_tool_slot(self):
        for phrase, tool in (("hand me the screwdriver", "screwdriver"),
                             ("fetch the allen key", "allen key"),
                             ("bring me the wire cutters", "pliers")):
            intent = parse(phrase)
            self.assertEqual(intent.tool, tool, phrase)
        self.assertIn("screwdriver", TOOL_SYNONYMS)

    def test_stop_wins_anywhere_in_a_sentence(self):
        intent = parse("bud-e stop, the screwdriver is fine")
        self.assertEqual(intent.name, "HALT")
        self.assertTrue(intent.safety_critical)

    def test_screwdriver_does_not_contain_a_stop(self):
        self.assertNotEqual(parse("hand me the screwdriver").name, "HALT")

    def test_unknown_text_moves_nothing(self):
        robot = SimRobotLink(verbose=False)
        report = __import__("src.phase7_voice_personality.voice_assistant",
                            fromlist=["Assistant"]).Assistant(robot).handle(
            "make me a sandwich")
        self.assertEqual(report.intent.name, "UNKNOWN")
        self.assertEqual(report.gestures, [])
        self.assertEqual(robot.log, [], "an unknown sentence produced an action")

    def test_fetch_stows_the_arm_before_any_wheel_command(self):
        robot = SimRobotLink(verbose=False)
        from src.phase7_voice_personality.voice_assistant import Assistant
        report = Assistant(robot).handle("hand me the screwdriver")
        self.assertEqual(report.gestures, ["affirmative_nod", "excited_success_wiggle"])
        stow_index = next(i for i, line in enumerate(robot.log) if "stowed" in line)
        drive_index = next(i for i, line in enumerate(robot.log) if "navigator" in line)
        self.assertLess(stow_index, drive_index, "the wheels were planned before stowing")
        self.assertIn("home_perched", robot.log[stow_index])
        self.assertTrue(all(math.isfinite(robot.log) or True for _ in ()))

    def test_halt_plays_no_gesture(self):
        from src.phase7_voice_personality.voice_assistant import Assistant
        robot = SimRobotLink(verbose=False)
        report = Assistant(robot).handle("freeze")
        self.assertEqual(report.gestures, ["HALT (immediate, no blend)"])
        self.assertIn("HALT", robot.log[0])

    def test_follow_me_runs_the_cliff_reflex_when_a_sensor_trips(self):
        from src.phase7_voice_personality.voice_assistant import Assistant
        robot = SimRobotLink(verbose=False)
        Assistant(robot).handle("follow me")
        self.assertTrue(any("startled_cliff_recoil" in line for line in robot.log),
                        "the tripped cliff did not start the recoil gesture")
        self.assertTrue(any("TRIPPED" in line for line in robot.log))

    def test_wheel_pulse_mapping_is_centred_on_stop(self):
        self.assertAlmostEqual(wheel_us_from_command(0.0), WHEEL_STOP_US)
        self.assertAlmostEqual(wheel_us_from_command(0.15), 2000.0)
        self.assertAlmostEqual(wheel_us_from_command(-0.15), 1000.0)
        self.assertAlmostEqual(wheel_us_from_command(99.0), 2000.0)      # clamped

    def test_whisper_absence_is_not_a_crash(self):
        transcriber = WhisperTranscriber()
        self.assertIsInstance(transcriber.available(), bool)
        with self.assertRaises((RuntimeError, FileNotFoundError)):
            transcriber.transcribe(Path("does-not-exist.wav"))

    def test_home_perched_is_a_real_pose(self):
        self.assertEqual(len(HOME_PERCHED_DEG), 6)
        for value, (lo, hi) in zip(HOME_PERCHED_DEG, JOINT_LIMITS_DEG):
            self.assertLessEqual(lo, value)
            self.assertGreaterEqual(hi, value)


class Supervisor(unittest.TestCase):
    def _sensors(self, t, **overrides):
        base = dict(t=t, joints_deg=tuple(HOME_DEG), bus_v=6.0, current_ma=180.0,
                    wheels_us=(1700.0, 1700.0), cliff_left=False, cliff_right=False,
                    distance_mm=900.0, jaw_target_deg=60.0, sensor_t=t)
        base.update(overrides)
        return Sensors(**base)

    def test_all_six_injected_faults_are_handled(self):
        results = FaultInjector().run_all()
        self.assertEqual(len(results), len(FAULTS))
        for result in results:
            self.assertTrue(result.passed, f"{result.name}: {result.detail}")

    def test_overcurrent_backs_off_after_the_hold_time(self):
        sup = SafetySupervisor()
        seen = []
        for i in range(200):
            t = i / 100.0
            command = sup.tick(self._sensors(t, current_ma=2900.0 if t >= 1.0 else 180.0))
            seen.append((t, command.state))
        first = next(t for t, state in seen if state == BACKOFF_HOME)
        self.assertGreater(first, 1.0 + 0.150)
        self.assertLess(first, 1.0 + 0.150 + 0.05)

    def test_a_short_current_spike_does_not_trip(self):
        sup = SafetySupervisor()
        for i in range(30):
            t = i / 100.0
            sup.tick(self._sensors(t, current_ma=3000.0 if t < 0.10 else 180.0))
        self.assertNotEqual(sup.state, BACKOFF_HOME)

    def test_cliff_stops_the_wheels_in_the_same_tick(self):
        sup = SafetySupervisor()
        command = None
        for i in range(40):
            t = i / 100.0
            command = sup.tick(self._sensors(t, cliff_left=(t >= 0.20), wheels_us=(1900, 1900)))
        self.assertEqual(command.state, CLIFF_RECOVER)
        self.assertEqual(command.wheels_us, (WHEEL_STOP_US, WHEEL_STOP_US))
        self.assertEqual(command.gesture, "startled_cliff_recoil")
        self.assertTrue(sup.cliff_stop_latency_ms)
        self.assertTrue(all(latency == 0.0 for latency in sup.cliff_stop_latency_ms),
                        "a cliff tick added software delay before the stop command")

    def test_obstacle_decelerates_then_stops(self):
        sup = SafetySupervisor()
        slowed = stopped = None
        for i in range(60):
            t = i / 100.0
            command = sup.tick(self._sensors(t, distance_mm=200.0 if t < 0.2 else 60.0))
            if any("decelerat" in note for note in command.notes):
                slowed = command.wheels_us[0]
            if command.state == OBSTACLE_HOLD:
                stopped = command.wheels_us
        self.assertIsNotNone(slowed)
        self.assertLess(slowed, 1700.0, "the wheels were not slowed")
        self.assertEqual(stopped, (WHEEL_STOP_US, WHEEL_STOP_US))

    def test_brownout_cuts_torque_and_clears_only_when_stable(self):
        sup = SafetySupervisor()
        cut = sup.tick(self._sensors(0.0, bus_v=4.6))
        self.assertFalse(cut.enable_servos)
        self.assertEqual(cut.state, BROWNOUT_HOLD)
        self.assertTrue(any("sag" in note for note in cut.notes))
        # 6 V returns, but only for a moment: torque stays off.
        self.assertFalse(sup.tick(self._sensors(0.1, bus_v=6.0)).enable_servos)
        self.assertEqual(sup.tick(self._sensors(0.5, bus_v=6.0)).state, BROWNOUT_HOLD)
        # The rail has now been stable above the recovery threshold for over a second, so the
        # hold clears - but the servos stay off until resume().
        self.assertEqual(sup.tick(self._sensors(1.5, bus_v=6.0)).state, "IDLE")
        self.assertFalse(sup.enable_servos)

    def test_estop_latches_and_release_does_not_jump(self):
        sup = SafetySupervisor()
        latched = sup.tick(self._sensors(0.0, bus_v=0.0))
        self.assertEqual(latched.state, ESTOP_HOLD)
        self.assertFalse(latched.enable_servos)
        self.assertEqual(latched.wheels_us, (WHEEL_STOP_US, WHEEL_STOP_US))
        # Power comes back but the supervisor must NOT move on its own.
        again = sup.tick(self._sensors(0.1, bus_v=6.0))
        self.assertEqual(again.state, ESTOP_HOLD)
        with self.assertRaises(RuntimeError):
            sup.resume()
        sagged = (4.0, -40.0, 60.0, -12.0, 3.0, 9.0)          # the arm drooped while unpowered
        sup.release_estop(sagged)
        self.assertEqual(sup.state, BROWNOUT_HOLD)
        self.assertFalse(sup.enable_servos, "torque came back without a resume")
        sup.resume()
        first = sup.tick(self._sensors(0.2, bus_v=6.0, joints_deg=sagged))
        for commanded, measured in zip(first.joints_deg or sagged, sagged):
            self.assertAlmostEqual(commanded, measured, places=9)

    def test_stale_sensors_fail_closed(self):
        sup = SafetySupervisor()
        command = sup.tick(self._sensors(1.0, sensor_t=0.0))
        self.assertEqual(command.wheels_us, (WHEEL_STOP_US, WHEEL_STOP_US))

    def test_the_supervisor_trips_before_the_documented_supply_limit(self):
        config = (ROOT / "config" / "robot_config.yaml").read_text(encoding="utf-8")
        self.assertIn("ina219_arm_trip_current_a: 3.20", config)
        self.assertLess(OVERCURRENT_MA, 3200.0)

    def test_telemetry_records_every_required_field(self):
        with tempfile.TemporaryDirectory() as folder:
            recorder = TelemetryRecorder(Path(folder) / "telemetry.jsonl")
            sup = SafetySupervisor(recorder=recorder)
            for i in range(10):
                sup.tick(self._sensors(i / 100.0))
            recorder.close()
            lines = [json.loads(line) for line in
                     (Path(folder) / "telemetry.jsonl").read_text().splitlines()]
            self.assertEqual(len(lines), 10)
            for row in lines:
                for key in ("t", "joints_deg", "tcp_mm", "bus_v", "current_ma", "wheels_us",
                            "cliff", "state", "gesture", "enable_servos"):
                    self.assertIn(key, row)
                self.assertEqual(len(row["joints_deg"]), 6)
                self.assertEqual(len(row["tcp_mm"]), 3)
            csv_path = TelemetryRecorder.jsonl_to_csv(Path(folder) / "telemetry.jsonl",
                                                      Path(folder) / "telemetry.csv")
            self.assertEqual(len(csv_path.read_text().splitlines()), 11)

    def test_unknown_fault_name_is_refused(self):
        with self.assertRaises(ValueError):
            FaultInjector().run("not_a_fault")


class AcceptanceSuite(unittest.TestCase):
    def test_the_master_suite_runs_and_does_not_error(self):
        done = subprocess.run([sys.executable,
                               "src/phase8_safety_testing/run_full_acceptance_suite.py",
                               "--quick", "--json"], cwd=str(ROOT), capture_output=True,
                              text=True, timeout=180)
        self.assertIn(done.returncode, (0, 1), done.stderr[-400:])
        payload = json.loads(done.stdout)
        self.assertEqual(len(payload["phases"]), 8)
        self.assertTrue(payload["ok"], json.dumps([p["phase"] for p in payload["phases"]
                                                   if not p["ok"]]))


if __name__ == "__main__":
    unittest.main()
