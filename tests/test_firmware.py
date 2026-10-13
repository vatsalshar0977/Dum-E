"""Firmware tests: compile and RUN the Phase 3 firmware's safety logic and sketch.

Three layers, all with the same honesty about what they can and cannot prove:

1. `test_reflex_logic.cpp` — the pure-logic header (`reflex_logic.h`) is compiled and executed
   with g++. Timing rules, priorities and shell behaviour are asserted directly.
2. `tests/firmware_host/main.cpp` — the REAL `.ino` is `#include`d and compiled against stub
   Arduino / ESP32 / Wire APIs, then driven with a simulated clock, simulated GPIO, a
   simulated PCA9685 (register writes decoded back into pulse widths) and a simulated serial
   port. This is the strongest thing available without hardware: the sketch is compiled by a
   real C++ compiler and its control flow actually runs.
3. Source inspection — banned blocking calls, ISR safety and the required pin/channel anchors.

NOT COVERED, EVER, BY THIS FILE: an ESP32. The I2C peripheral, the GPIO interrupts, the task
watchdog, the timers and the servos are all models. The sketch has never been flashed.
"""

import re
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "firmware" / "phase3_full_6dof_esp32"
HOST_STUBS = ROOT / "tests" / "firmware_host"
GXX = shutil.which("g++") or shutil.which("clang++")

SKETCH = FIRMWARE / "phase3_full_6dof_esp32.ino"


def compile_and_run(sources, include_dirs=(), flags=("-std=c++11", "-Wall", "-Wextra")):
    """Compile with g++ and run. Returns (returncode, output). Raises on a build failure."""
    with tempfile.TemporaryDirectory() as folder:
        binary = Path(folder) / "a.out"
        command = [GXX, *flags]
        for directory in include_dirs:
            command.append(f"-I{Path(directory)}")
        command += [str(Path(item)) for item in sources]
        command += ["-o", str(binary)]
        build = subprocess.run(command, capture_output=True, text=True, timeout=300)
        if build.returncode != 0:
            raise AssertionError(
                f"compiler failed:\n{' '.join(command)}\n{build.stdout}\n{build.stderr}")
        if build.stderr.strip():
            raise AssertionError(f"compiler warnings are errors here:\n{build.stderr}")
        run = subprocess.run([str(binary)], capture_output=True, text=True, timeout=300)
        return run.returncode, run.stdout + run.stderr


@unittest.skipIf(GXX is None, "no C++ compiler available")
class ReflexLogic(unittest.TestCase):
    """The safety logic, executed. 73 checks inside the C++ program."""

    def test_logic_header_compiles_and_passes(self):
        code, output = compile_and_run([FIRMWARE / "test_reflex_logic.cpp"],
                                       flags=("-std=c++11", "-Wall", "-Wextra", "-Wpedantic"))
        self.assertEqual(code, 0, output[-2000:])
        match = re.search(r"PASS: (\d+) / (\d+) checks", output)
        self.assertIsNotNone(match, output[-2000:])
        self.assertEqual(match.group(1), match.group(2))
        self.assertGreaterEqual(int(match.group(2)), 70, "the logic tests shrank unexpectedly")

    def test_the_logic_header_has_no_arduino_dependency(self):
        source = (FIRMWARE / "reflex_logic.h").read_text(encoding="utf-8")
        # Comments are allowed to mention Arduino; CODE is not allowed to use it. Stripping
        # comments first is the difference between checking the header and checking its prose.
        code = re.sub(r"/\*.*?\*/", "", source, flags=re.S)
        code = re.sub(r"//[^\n]*", "", code)
        for banned in ("#include <Arduino.h>", "#include <Wire.h>", "Serial.", "digitalWrite",
                       "pinMode", "millis()"):
            self.assertNotIn(banned, code,
                             f"{banned} leaked into the host-testable logic header")


@unittest.skipIf(GXX is None, "no C++ compiler available")
class RealSketchOnHost(unittest.TestCase):
    """The .ino itself, compiled and driven against simulated hardware. 65 checks."""

    def test_sketch_compiles_and_passes_its_host_simulation(self):
        code, output = compile_and_run([HOST_STUBS / "main.cpp"],
                                       include_dirs=[HOST_STUBS])
        self.assertEqual(code, 0, output[-3000:])
        match = re.search(r"PASS: (\d+) / (\d+) checks", output)
        self.assertIsNotNone(match, output[-2000:])
        self.assertEqual(match.group(1), match.group(2))
        self.assertGreaterEqual(int(match.group(2)), 60)

    def test_the_headline_reflex_is_inside_the_budget_in_the_simulation(self):
        code, output = compile_and_run([HOST_STUBS / "main.cpp"], include_dirs=[HOST_STUBS])
        self.assertEqual(code, 0, output[-2000:])
        self.assertIn("both channels were written inside the 5 ms budget", output)
        match = re.search(r"measured edge -> both channels written: (\d+) us", output)
        self.assertIsNotNone(match, output[-1500:])
        self.assertLess(int(match.group(1)), 5000,
                        "the simulated edge-to-write latency blew the 5 ms budget")


@unittest.skipIf(GXX is None, "no C++ compiler available")
class BothCoreGenerations(unittest.TestCase):
    """arduino-esp32 2.x and 3.x configure the task watchdog differently.

    The sketch handles both behind one #if, and BOTH branches must compile: getting this wrong
    is the classic "it built on my machine" failure. The test rewrites the stub's version
    header to claim a 2.x core and rebuilds the same harness.
    """

    def test_the_legacy_watchdog_branch_still_compiles_and_passes(self):
        with tempfile.TemporaryDirectory() as folder:
            work = Path(folder)
            for name in ("Arduino.h", "Wire.h", "esp_err.h", "esp_timer.h", "esp_task_wdt.h",
                         "esp_arduino_version.h", "main.cpp"):
                shutil.copy(HOST_STUBS / name, work / name)
            version_header = work / "esp_arduino_version.h"
            version_header.write_text(
                version_header.read_text(encoding="utf-8").replace(
                    "#define ESP_ARDUINO_VERSION_MAJOR 3", "#define ESP_ARDUINO_VERSION_MAJOR 2"),
                encoding="utf-8")
            shutil.copy(SKETCH, work / SKETCH.name)
            shutil.copy(FIRMWARE / "reflex_logic.h", work / "reflex_logic.h")
            main = work / "main.cpp"
            main.write_text(
                main.read_text(encoding="utf-8").replace(
                    '#include "../../firmware/phase3_full_6dof_esp32/phase3_full_6dof_esp32.ino"',
                    '#include "phase3_full_6dof_esp32.ino"'),
                encoding="utf-8")
            code, output = compile_and_run([main], include_dirs=[work])
            self.assertEqual(code, 0, output[-2000:])
            self.assertIn("PASS:", output)
            self.assertIn("task watchdog timeout is 5 s", output)


class SketchSource(unittest.TestCase):
    """What the source must and must not contain. Cheap, and catches whole classes of bug."""

    def setUp(self):
        self.text = SKETCH.read_text(encoding="utf-8")

    def test_the_briefs_pins_and_channels_are_all_there(self):
        required = {
            "cliff left on GPIO 25": "PIN_CLIFF_LEFT = 25",
            "cliff right on GPIO 26": "PIN_CLIFF_RIGHT = 26",
            "HC-SR04 trigger on GPIO 18": "PIN_SONAR_TRIG = 18",
            "HC-SR04 echo on GPIO 19": "PIN_SONAR_ECHO = 19",
            "I2C SDA on GPIO 21": "PIN_I2C_SDA = 21",
            "I2C SCL on GPIO 22": "PIN_I2C_SCL = 22",
            "PCA9685 OE on GPIO 4": "PIN_PCA9685_OE = 4",
            "left wheel on channel 6": "WHEEL_LEFT_CH = 6",
            "right wheel on channel 7": "WHEEL_RIGHT_CH = 7",
            "PCA9685 at 0x40": "PCA9685_ADDR = 0x40",
            "INA219 at 0x41": "INA219_ADDR = 0x41",
        }
        for what, needle in required.items():
            self.assertIn(needle, self.text, f"missing {what}")
        self.assertNotIn("pulseIn(", self.text)

    def test_the_reflex_numbers_are_the_briefs_numbers(self):
        header = (FIRMWARE / "reflex_logic.h").read_text(encoding="utf-8")
        self.assertIn("reverse_left_us = 1680", header)
        self.assertIn("reverse_right_us = 1320", header)
        self.assertIn("recoil_ms = 350", header)
        self.assertIn("command_timeout_ms = 500", header)
        self.assertIn("reflex_budget_us = 5000", header)
        # ... and the sketch must not quietly redefine any of them.
        for needle in ("recoil_ms =", "command_timeout_ms =", "reverse_left_us ="):
            self.assertNotIn(needle, self.text, f"the sketch shadows {needle} from the header")

    def test_fmea_11_watchdog_is_configured(self):
        self.assertIn("esp_task_wdt_init", self.text)
        self.assertIn("esp_task_wdt_add", self.text)
        self.assertIn("esp_task_wdt_reset()", self.text)
        self.assertIn("WATCHDOG_TIMEOUT_MS = 5000", self.text)
        # Both core generations must be handled, or the sketch breaks on half the toolchains.
        self.assertIn("esp_task_wdt_config_t", self.text)
        self.assertIn("ESP_ARDUINO_VERSION_MAJOR", self.text)

    def test_fmea_12_bus_recovery_is_implemented_and_used_at_boot(self):
        self.assertIn("i2c_bus_recovery()", self.text)
        self.assertIn("nine SCL clocks", self.text.lower().replace("nine scl", "nine SCL"))
        # The recovery must run BEFORE Wire.begin, and the class must be able to cut the OE.
        setup_body = self.text.split("void setup()", 1)[1].split("void loop()", 1)[0]
        self.assertLess(setup_body.index("i2c_bus_recovery()"), setup_body.index("i2cBegin()"),
                        "the bus is recovered after Wire.begin(), which defeats the purpose")
        self.assertIn("kDisableOutputs", self.text)
        self.assertIn("setServoEnable(false)", self.text)
        self.assertIn("Wire.setTimeOut", self.text)

    def test_no_blocking_or_unbounded_calls_in_the_loop(self):
        banned = {
            "pulseIn(": "blocks for up to a second on a missing echo",
            "readStringUntil": "blocks for the whole serial timeout",
            "Serial.readBytes": "blocks until the byte count or the timeout",
            "delay(": "a millisecond-scale stall in the control loop",
        }
        for needle, why in banned.items():
            self.assertNotIn(needle, self.text, f"{needle} in the firmware: {why}")
        # delayMicroseconds is allowed, but only for short, bounded hardware handshakes.
        for match in re.finditer(r"delayMicroseconds\((\d+)\)", self.text):
            self.assertLessEqual(int(match.group(1)), 500,
                                 "a long delayMicroseconds snuck in: "
                                 f"{match.group(0)}")

    def test_the_loop_services_the_cliff_before_the_serial_port(self):
        loop_body = self.text.split("void loop()", 1)[1]
        self.assertLess(loop_body.index("serviceCliffReflex()"),
                        loop_body.index("Serial.available()"),
                        "a burst of serial traffic could delay the reflex")

    def test_interrupt_handlers_never_touch_i2c_serial_or_floats(self):
        handlers = re.findall(r"static void IRAM_ATTR (\w+)\(\) \{(.*?)\n\}", self.text, re.S)
        self.assertGreaterEqual(len(handlers), 3, "expected the three ISRs")
        for name, body in handlers:
            for forbidden in ("Wire.", "Serial.", "digitalWrite", "delayMicroseconds",
                              "float ", "printf"):
                self.assertNotIn(forbidden, body,
                                 f"{name}() does something unsound in an ISR: {forbidden}")
            self.assertIn("esp_timer_get_time()", body,
                          f"{name}() must stamp its edge inside the ISR")

    def test_every_host_wheel_command_goes_through_the_guarded_path(self):
        # The forward gate must be reachable from BOTH the `W` command and the `M`/`G` path.
        self.assertIn("applyWheelPair(wc.left_us, wc.right_us, \"W\")", self.text)
        self.assertIn('applyWheelPair(wanted[WHEEL_LEFT_CH], wanted[WHEEL_RIGHT_CH], "M/G")',
                      self.text)
        # ... and the reflex must NOT be gated by it, or a safety response could be refused.
        reflex = self.text.split("static void serviceCliffReflex()", 1)[1].split(
            "// ====", 1)[0]
        self.assertIn("writeWheels(out.left_us, out.right_us)", reflex)
        self.assertNotIn("applyWheelPair", reflex)

    def test_channel_list_commands_seed_from_the_shadow(self):
        """`M 6:1320` names ONE wheel; applyWheelPair() writes BOTH as one transaction.

        An uninitialised `wanted[]` would push whatever is on the stack into the other wheel -
        potentially full speed, in either direction. The harness cannot catch this reliably
        (stack garbage may happen to be the right value), so it is checked at the source level.
        """
        body = self.text.split("static void handleChannelList(", 1)[1]
        self.assertIn("wanted[ch] = g_pulse_us[ch];", body,
                      "handleChannelList must seed wanted[] from the channel shadow")

    def test_the_shadow_is_initialised_before_any_write(self):
        setup_body = self.text.split("void setup()", 1)[1].split("void loop()", 1)[0]
        first_write = setup_body.index("pcaWriteChannels(0, g_pulse_us")
        loop_that_fills = setup_body.index("g_pulse_us[ch] = 1500")
        self.assertLess(loop_that_fills, first_write,
                        "channels were written before the shadow array was filled")

    def test_the_wheels_are_declared_mirrored(self):
        # The brief's 1680/1320 reverse pair only makes sense if the servos face each other.
        header = (FIRMWARE / "reflex_logic.h").read_text(encoding="utf-8")
        self.assertIn("left_forward_is_above_neutral = false", header)
        self.assertIn("right_forward_is_above_neutral = true", header)
        self.assertIn("MIRRORED WHEELS", header, "the calibration assumption must be spelled out")


class DocsAgree(unittest.TestCase):
    """The FMEA table and the wiring doc must not contradict the firmware any more."""

    def test_fmea_rows_11_and_12_are_no_longer_not_implemented(self):
        checklist = (ROOT / "docs" / "phase8_safety_testing_and_bringup_checklist.md").read_text(
            encoding="utf-8")
        row11 = next(line for line in checklist.splitlines() if line.startswith("| 11 |"))
        row12 = next(line for line in checklist.splitlines() if line.startswith("| 12 |"))
        for row, what in ((row11, "watchdog"), (row12, "I2C recovery")):
            self.assertIn("phase3_full_6dof_esp32.ino", row,
                          f"FMEA row for the {what} does not name the firmware file")
            lowered = row.lower()
            self.assertNotIn("not implemented", lowered.replace(
                "not implemented in python", ""),
                f"FMEA row for the {what} still says not implemented")
        self.assertIn("esp_task_wdt_init", row11)
        self.assertIn("i2c_bus_recovery", row12)

    def test_the_wiring_doc_lists_the_new_pins(self):
        wiring = (ROOT / "docs" / "phase3_electronics_and_wiring.md").read_text(encoding="utf-8")
        for pin in ("25", "26", "18", "19"):
            self.assertIn(pin, wiring, f"GPIO {pin} is not in the wiring table")
        self.assertIn("W <left_us> <right_us>", wiring)
        self.assertIn("phase3_full_6dof_esp32", wiring)

    def test_the_config_agrees_with_the_firmware_channels(self):
        config = (ROOT / "config" / "robot_config.yaml").read_text(encoding="utf-8")
        for needle in ("cliff_left:", "cliff_right:", "sonar_trig:", "sonar_echo:",
                       "wheel_left_ch:", "wheel_right_ch:"):
            self.assertIn(needle, config, f"{needle} missing from robot_config.yaml")
        for value in ("cliff_left: 25", "cliff_right: 26", "wheel_left_ch: 6",
                      "wheel_right_ch: 7"):
            self.assertIn(value, config, f"{value} disagrees with the firmware")
        self.assertIn("cliff_recoil_ms: 350", config)
        self.assertIn("wheel_command_timeout_ms: 500", config)


if __name__ == "__main__":
    unittest.main()
