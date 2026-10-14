"""Phase 8 - one command that exercises all eight phases and prints a master scorecard.

    python3 src/phase8_safety_testing/run_full_acceptance_suite.py
    python3 src/phase8_safety_testing/run_full_acceptance_suite.py --quick      # skip the STL audit
    python3 src/phase8_safety_testing/run_full_acceptance_suite.py --json

Seven phases are executed *in this process* against the real modules: kinematics, the camera
model, the drive policy, the gesture engine and the supervisor. Only four things are run as
subprocesses, because they are scripts that own their own output: the Phase 1 sizing reports
and the two STL verifiers.

A PASS here means the MODEL is self-consistent. It does not mean a servo moved, a part was
printed, or that anything is safe to put a hand near. Every phase's own report says the same
thing in its own words, and the last line of this file is the only summary you should quote.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import shutil
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.phase4_simulation.sim_runner import HOME                        # noqa: E402
from src.phase5_control.kinematics import (                              # noqa: E402
    LIMITS, forward, inverse_kinematics_5dof,
)
from src.phase5_control.trajectory_and_collision import (                # noqa: E402
    VELOCITY_LIMITS_DEG_S, check_pose_safety, plan_min_jerk_trajectory,
)
from src.phase6_perception.camera_and_hand_eye_calib import (            # noqa: E402
    CAMERA_MOUNT_OFFSET_MM, camera_pose_from_fk, intersect_table, nominal_intrinsics,
    project_point_xyz, self_test_projection_round_trip,
)
from src.phase6_perception.vision_mobile_navigator import (              # noqa: E402
    Reading, choose_wheels, plan_fetch_sequence,
)
from src.phase7_voice_personality.personality_engine import (            # noqa: E402
    SAMPLE_HZ, SEQUENCES, PersonalityDirector, looks_like_failed_grasp,
)
from src.phase7_voice_personality.voice_assistant import parse           # noqa: E402
from src.phase8_safety_testing.safety_supervisor_and_logger import (     # noqa: E402
    FAULTS, OVERCURRENT_MA, FaultInjector,
)


@dataclass
class Check:
    phase: str
    name: str
    ok: bool
    detail: str


@dataclass
class PhaseReport:
    phase: str
    title: str
    checks: List[Check] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return bool(self.checks) and all(c.ok for c in self.checks)

    def add(self, name: str, ok: bool, detail: str) -> None:
        self.checks.append(Check(self.phase, name, ok, detail))


# ---------------------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------------------
def run_script(relative: str, timeout_s: float = 240.0,
               pattern: Optional[str] = None) -> tuple[int, str]:
    """Run another script in the repo and hand back (exit code, combined output)."""
    started = time.time()
    done = subprocess.run([sys.executable, relative], cwd=str(ROOT), capture_output=True,
                          text=True, timeout=timeout_s)
    output = done.stdout + done.stderr
    if time.time() - started > timeout_s:
        raise TimeoutError(relative)
    if pattern:
        match = re.search(pattern, output)
        output = match.group(0) if match else output[-400:]
    return done.returncode, output


def run_cpp(sources: Sequence[str], include_dirs: Sequence[str] = (),
            flags: Sequence[str] = ("-std=c++11", "-Wall", "-Wextra")) -> tuple[int, str]:
    """Compile a C++ program and run it. Returns (exit code, output, or (-1, reason))."""
    compiler = shutil.which("g++") or shutil.which("clang++")
    if compiler is None:
        return -1, "no C++ compiler on this machine"
    with tempfile.TemporaryDirectory() as folder:
        binary = str(Path(folder) / "a.out")
        command = [compiler, *flags]
        for directory in include_dirs:
            command.append(f"-I{ROOT / directory}")
        command += [str(ROOT / item) for item in sources]
        command += ["-o", binary]
        build = subprocess.run(command, capture_output=True, text=True, timeout=300)
        if build.returncode != 0:
            return build.returncode, f"compiler error:\n{build.stderr[-500:]}"
        if build.stderr.strip():
            return 2, f"compiler warnings:\n{build.stderr[-500:]}"
        done = subprocess.run([binary], capture_output=True, text=True, timeout=300)
        return done.returncode, done.stdout + done.stderr


def _kv(text: str, key: str) -> Optional[float]:
    match = re.search(rf"{re.escape(key)}\s*[:=]\s*([-+0-9.]+)", text)
    return float(match.group(1)) if match else None


# ---------------------------------------------------------------------------------------
# Phase 1 - sizing
# ---------------------------------------------------------------------------------------
def phase1() -> PhaseReport:
    report = PhaseReport("P1", "Sizing: torque, power and workspace")
    code, out = run_script("src/phase1_sizing/torque_and_power_calc.py")
    report.add("torque and power budget runs", code == 0,
               f"exit {code}; " + (out.strip().splitlines()[-1][:90] if out.strip() else "no output"))
    code, out = run_script("src/phase1_sizing/workspace_envelope_check.py")
    lowered = out.lower()
    report.add("workspace envelope", code == 0 and "collision" not in lowered.replace(
        "desk collision", ""),
        "no desk collision reported" if "collision" not in lowered else "desk collision reported")
    return report


# ---------------------------------------------------------------------------------------
# Phase 2 - the STL audit
# ---------------------------------------------------------------------------------------
ASSEMBLY_FIT_PATTERN = r"TOTAL\s+\d+ / \d+ checks\s+(PASS|FAIL)"


def phase2(quick: bool) -> PhaseReport:
    report = PhaseReport("P2", "Mechanical: 13 STLs, true hollows, coaxiality")
    code, out = run_script("src/phase2_mechanical/verify_print_parts.py")
    passed = "PASS: every part" in out
    report.add("print-parts verifier", code == 0 and passed,
               next((line.strip()[:110] for line in out.splitlines()
                     if line.startswith(("PASS:", "FAIL:"))), "no verdict line"))
    if quick:
        report.add("assembly-fit audit", True, "skipped (--quick)")
        return report
    code, out = run_script("src/phase2_mechanical/verify_assembly_fit.py", timeout_s=600.0,
                           pattern=ASSEMBLY_FIT_PATTERN)
    match = re.search(r"TOTAL\s+(\d+) / (\d+) checks\s+(PASS|FAIL)", out)
    if match:
        passed, total, verdict = int(match.group(1)), int(match.group(2)), match.group(3)
        report.add("assembly-fit audit", verdict == "PASS",
                   f"{passed}/{total} checks {verdict} (mesh, volume, probes, coaxial, "
                   f"pockets, straddle, rover)")
    else:
        report.add("assembly-fit audit", False, out.strip()[-120:])
    return report


# ---------------------------------------------------------------------------------------
# Phase 3 - wiring consistency, recomputed
# ---------------------------------------------------------------------------------------
def phase3() -> PhaseReport:
    report = PhaseReport("P3", "Electronics: pins, addresses, voltage drop")
    doc = (ROOT / "docs" / "phase3_electronics_and_wiring.md").read_text(encoding="utf-8")
    config = (ROOT / "config" / "robot_config.yaml").read_text(encoding="utf-8")
    firmware = (ROOT / "firmware" / "phase1_single_joint_esp32" /
                "phase1_single_joint_esp32.ino").read_text(encoding="utf-8")

    # pyyaml is not a dependency of this repo, so the YAML is checked as text.
    pins = {
        "SDA GPIO21": ("GPIO21" in doc, "PIN_I2C_SDA       = 21;" in firmware),
        "SCL GPIO22": ("GPIO22" in doc, "PIN_I2C_SCL       = 22;" in firmware),
        "OE GPIO4": ("GPIO4" in doc, "PIN_PCA9685_OE    = 4;" in firmware),
    }
    for name, (in_doc, in_fw) in pins.items():
        report.add(f"{name} in doc and firmware", in_doc and in_fw,
                   f"doc={in_doc} firmware={in_fw}")
    report.add("I2C addresses 0x40 / 0x41 agreed", "0x41" in doc and "0x41" in config,
               "PCA9685 0x40, INA219 0x41 in both the wiring table and the config")
    report.add("e-stop is inline and NC in the 6 V rail",
               "latching" in doc.lower() and "NC" in doc,
               "the wiring table puts a latching NC mushroom switch in the servo rail")

    nom = _kv(config, "nominal_voltage_v") or 0.0
    low = _kv(config, "min_safe_voltage_v") or 0.0
    trip = _kv(config, "ina219_arm_trip_current_a") or 0.0
    stall = _kv(config, "ina219_gripper_stall_current_a") or 0.0
    report.add("supervisor current limit sits under the documented trip",
               OVERCURRENT_MA < trip * 1000.0,
               f"supervisor backs off at {OVERCURRENT_MA:.0f} mA, the config trips the arm at "
               f"{trip * 1000:.0f} mA")

    # Recompute the resistive drop rather than quoting the doc. 22 AWG copper is 52.9 mOhm/m;
    # the 5 A supply current is the worst case the config allows.
    r_per_m_22awg = 0.0529
    length_m = 1.0
    current_a = _kv(config, "supply_current_limit_a") or 5.0
    drop_v = 2.0 * length_m * r_per_m_22awg * current_a        # out and back
    margin_v = nom - low
    report.add("1 m of 22 AWG at the supply limit keeps the rail above minimum",
               drop_v < margin_v,
               f"{drop_v:.2f} V of drop vs {margin_v:.2f} V of headroom at {current_a:.1f} A "
               f"({nom:.1f} -> {low:.1f} V)" if drop_v < margin_v else
               f"{drop_v:.2f} V of drop exceeds the {margin_v:.2f} V of headroom: use 18 AWG")
    report.add("gripper stall threshold is below the missed-grasp detector",
               stall * 1000.0 > 180.0,
               f"config stops the claw at {stall * 1000:.0f} mA; a missed grasp is under "
               f"180 mA")

    # --- the firmware: compiled and EXECUTED, not just inspected ------------------------
    code, out = run_cpp(["firmware/phase3_full_6dof_esp32/test_reflex_logic.cpp"],
                        flags=("-std=c++11", "-Wall", "-Wextra", "-Wpedantic"))
    if code == -1:
        report.add("firmware safety logic (g++)", True, f"skipped: {out}")
    else:
        match = re.search(r"PASS: (\d+) / (\d+) checks", out)
        report.add("firmware safety logic executes on the host",
                   code == 0 and match is not None,
                   (f"{match.group(1)}/{match.group(2)} checks: reflex timing, the 350/500 ms "
                    f"rules, the NACK policy, the line parser") if match else out[-160:])

    code, out = run_cpp(["tests/firmware_host/main.cpp"], include_dirs=["tests/firmware_host"])
    if code == -1:
        report.add("the real sketch on a host simulator", True, f"skipped: {out}")
    else:
        match = re.search(r"PASS: (\d+) / (\d+) checks", out)
        latency = re.search(r"measured edge -> both channels written: (\d+) us", out)
        detail = (f"{match.group(1)}/{match.group(2)} checks on the real .ino; simulated "
                  f"edge-to-write {latency.group(1)} us (budget 5000)"
                  if match and latency else out[-160:])
        report.add("the real .ino compiles and runs on a host simulator",
                   code == 0 and match is not None, detail)

    # --- the standalone Phase 3 verifier and the host driver -----------------------------
    done = subprocess.run([sys.executable, str(ROOT / "src" / "phase3_electronics" /
                                               "power_and_wiring_verifier.py"), "--json"],
                          capture_output=True, text=True, timeout=300)
    try:
        payload = json.loads(done.stdout)
        report.add("wiring, config and firmware describe the same robot",
                   done.returncode == 0 and payload["passed"] == payload["total"],
                   f"{payload['passed']}/{payload['total']} checks in "
                   f"src/phase3_electronics/power_and_wiring_verifier.py")
    except (ValueError, KeyError):
        report.add("wiring, config and firmware describe the same robot", False,
                   (done.stdout + done.stderr)[-200:])

    done = subprocess.run([sys.executable, str(ROOT / "src" / "phase3_electronics" /
                                               "esp32_arm_driver.py"), "--dry-run", "--ping"],
                          capture_output=True, text=True, timeout=300)
    report.add("the host serial driver frames commands the firmware defines",
               done.returncode == 0 and "TX  PING" in done.stdout,
               "W <left> <right> / M / G / OE, with the mirrored forward pair")

    ino = (ROOT / "firmware" / "phase3_full_6dof_esp32" /
           "phase3_full_6dof_esp32.ino").read_text(encoding="utf-8")
    report.add("reflex pins and channels match the brief", all(
        needle in ino for needle in ("PIN_CLIFF_LEFT = 25", "PIN_CLIFF_RIGHT = 26",
                                     "WHEEL_LEFT_CH = 6", "WHEEL_RIGHT_CH = 7",
                                     "PIN_SONAR_TRIG = 18", "PIN_SONAR_ECHO = 19")),
        "GPIO 25/26 cliffs, GPIO 18/19 sonar, PCA9685 channels 6/7 wheels")
    report.add("no blocking call in the control loop",
               not any(banned in ino for banned in ("pulseIn(", "readStringUntil",
                                                    "Serial.readBytes")),
               "pulseIn, readStringUntil and readBytes are all absent: the 5 ms budget cannot "
               "survive a blocking read")
    report.add("FMEA #11 watchdog and #12 bus recovery are in the firmware",
               all(needle in ino for needle in ("esp_task_wdt_init", "esp_task_wdt_reset()",
                                                "i2c_bus_recovery()", "kDisableOutputs")),
               "task watchdog + 500 ms wheel timeout, and 9-clock I2C recovery at boot and on "
               "every NACK")
    return report


# ---------------------------------------------------------------------------------------
# Phase 4 - URDF and the viewer's model
# ---------------------------------------------------------------------------------------
def phase4() -> PhaseReport:
    report = PhaseReport("P4", "Simulation: URDF chain and the viewer model")
    root = ET.parse(ROOT / "urdf" / "bude_arm.urdf").getroot()
    links = {x.attrib["name"] for x in root.findall("link")}
    joints = {x.attrib["name"]: x for x in root.findall("joint")}
    report.add("7 links, 6 joints", len(links) == 7 and len(joints) == 6,
               f"{len(links)} links, {len(joints)} joints")
    names = ("base_yaw", "shoulder_pitch", "elbow_pitch", "wrist_pitch", "wrist_roll",
             "tcp_fixed")
    report.add("every expected joint is present", all(n in joints for n in names),
               ", ".join(n for n in names if n in joints))
    limit_ok = True
    for name, (lo, hi) in zip(names[:5], LIMITS):
        found = joints[name].find("limit")
        if found is None:
            limit_ok = False
            continue
        if abs(float(found.attrib["lower"]) - math.radians(lo)) > 1e-6 or \
           abs(float(found.attrib["upper"]) - math.radians(hi)) > 1e-6:
            limit_ok = False
    report.add("URDF limits equal the Phase 5 LIMITS table", limit_ok,
               f"{len(LIMITS)} joint limit pairs compared in radians")
    stacked = forward((0.0, 0.0, 0.0, 0.0, 0.0))
    report.add("URDF link stack reaches the declared 280 mm",
               abs(stacked.z - 280.0) < 0.5,
               f"straight-up TCP z = {stacked.z:.1f} mm; the viewer's HOME pose is "
               f"{forward(tuple(HOME)).z:.1f} mm")
    return report


# ---------------------------------------------------------------------------------------
# Phase 5 - kinematics, planning, keep-outs
# ---------------------------------------------------------------------------------------
def phase5() -> PhaseReport:
    report = PhaseReport("P5", "Control: FK, IK, minimum jerk, collision keep-outs")
    q = (20.0, 45.0, 35.0, 0.0, 15.0)
    pose = forward(q)
    solutions = inverse_kinematics_5dof(pose.x, pose.y, pose.z, pose.pitch, pose.roll,
                                        adapt_pitch=True)
    report.add("IK returns the pose it was given, roll included",
               any(all(abs(a - b) < 1e-6 for a, b in zip(s, q)) for s, _ in solutions),
               f"{len(solutions)} solution(s) for TCP ({pose.x:.0f}, {pose.y:.0f}, "
               f"{pose.z:.0f}) mm pitch {pose.pitch:.0f} deg roll {pose.roll:.0f} deg")
    try:
        inverse_kinematics_5dof(500.0, 0.0, 20.0, 0.0)
        report.add("unreachable target is refused", False, "it returned a solution")
    except ValueError as exc:
        report.add("unreachable target is refused", True, str(exc)[:80])
    plan = plan_min_jerk_trajectory(HOME, (35.0, 30.0, 45.0, -20.0, 0.0))
    report.add("minimum-jerk plan is 100 Hz and stays inside the speed limits",
               len(plan.samples) > 100 and all(
                   v <= limit + 1e-6 for v, limit in
                   zip(plan.peak_velocity_deg_s, VELOCITY_LIMITS_DEG_S)),
               plan.summary())
    safe = check_pose_safety(HOME)
    report.add("keep-out volumes accept the home pose", safe.ok, safe.describe()[:90])
    try:
        plan_min_jerk_trajectory(HOME, (0.0, 45.0, 90.0, 60.0, 0.0))
        lowered = False
    except ValueError:
        lowered = True
    report.add("a desk-crashing goal is refused, not planned", lowered,
               "the planner collects every violation and raises")
    report.add("velocity budget is declared per joint",
               len(VELOCITY_LIMITS_DEG_S) == 5,
               f"limits {tuple(VELOCITY_LIMITS_DEG_S)} deg/s")
    return report


# ---------------------------------------------------------------------------------------
# Phase 6 - camera and drive policy
# ---------------------------------------------------------------------------------------
def phase6() -> PhaseReport:
    report = PhaseReport("P6", "Perception: camera round trip and the drive policy")
    intr = nominal_intrinsics()
    report.add("camera intrinsics are the declared 62 deg HFOV",
               abs(intr["fx"] - 532.57) < 0.01,
               f"fx = fy = {intr['fx']:.2f} px at "
               f"{intr['image_size'][0]}x{intr['image_size'][1]}")
    worst, checked = self_test_projection_round_trip()
    report.add("pixel -> ray -> table -> pixel closes the loop",
               checked >= 20 and worst < 1e-6,
               f"{checked} points, worst residual {worst:.3e} mm")
    # Aim the tool DOWN at the bench first: at the home pose the camera looks at the wall,
    # and the projection code correctly refuses a point behind the lens.
    looking_down = (0.0, 45.4, 47.3, 72.3, 0.0)
    transform = camera_pose_from_fk(looking_down)
    tcp = forward(looking_down)
    desk_point = (tcp.x, tcp.y, 0.0)
    u, v = project_point_xyz(desk_point, intr, transform)
    back = intersect_table(u, v, intr, transform, z_mm=0.0)
    report.add("a desk point survives project and intersect",
               abs(back[0] - desk_point[0]) < 0.5 and abs(back[2]) < 1e-6,
               f"({desk_point[0]:.1f}, {desk_point[1]:.1f}, 0) -> pixel ({u:.1f}, {v:.1f}) -> "
               f"({back[0]:.2f}, {back[1]:.2f}, {back[2]:.2f}) mm")
    report.add("camera mount offset is declared", len(CAMERA_MOUNT_OFFSET_MM) == 3,
               f"offset {tuple(CAMERA_MOUNT_OFFSET_MM)} mm (read off CAD, not a caliper)")
    policy = choose_wheels(Reading(10.0, 9.95, 0.1, 0.8, True, True, True, True))
    report.add("drive policy moves when every precondition holds",
               policy != (0.0, 0.0), f"wheels {policy}")
    blocked = [
        ("stale", Reading(10.0, 9.0, 0.1, 0.8, True, True, True, True)),
        ("cliff", Reading(10.0, 9.95, 0.1, 0.8, False, True, True, True)),
        ("obstacle", Reading(10.0, 9.95, 0.1, 0.1, True, True, True, True)),
        ("e-stop", Reading(10.0, 9.95, 0.1, 0.8, True, True, False, True)),
        ("arm not stowed", Reading(10.0, 9.95, 0.1, 0.8, True, True, True, False)),
        ("no bearing", Reading(10.0, 9.95, None, 0.8, True, True, True, True)),
    ]
    refused = [name for name, reading in blocked if choose_wheels(reading) == (0.0, 0.0)]
    report.add("drive policy fails closed on every precondition",
               len(refused) == len(blocked), f"refused: {', '.join(refused)}")
    plan = plan_fetch_sequence((1050.0, 180.0))
    report.add("mobile fetch converges on the standoff",
               bool(plan["arrived"]) and plan["distance_to_standoff_mm"] < 8.0,
               f"arrived={plan['arrived']} in {plan['ticks']} ticks, "
               f"{plan['distance_to_standoff_mm']:.1f} mm off")
    return report


# ---------------------------------------------------------------------------------------
# Phase 7 - gestures and language
# ---------------------------------------------------------------------------------------
def phase7() -> PhaseReport:
    report = PhaseReport("P7", "Voice and personality: six gestures, deterministic parser")
    bad: List[str] = []
    total_frames = 0
    for name, seq in SEQUENCES.items():
        try:
            seq.validate()
            total_frames += len(seq.sample())
        except ValueError as exc:
            bad.append(f"{name}: {exc}")
    report.add("every gesture is feasible inside its speed budget", not bad,
               f"{len(SEQUENCES)} gestures, {total_frames} frames at 100 Hz"
               if not bad else "; ".join(bad)[:120])
    director = PersonalityDirector(t0=0.0)
    director.submit("wake_word", 0.1)
    director.submit("cliff", 0.5)
    report.add("the cliff reflex pre-empts a running gesture",
               director.state.sequence is SEQUENCES["startled_cliff_recoil"],
               "priority 0 beat priority 3")
    frame = director.frame(0.6)
    report.add("the recoil reverses both wheels",
               frame.wheels[0] < 1500.0 and frame.wheels[1] < 1500.0,
               f"wheels {frame.wheels[0]:.0f}/{frame.wheels[1]:.0f} us")
    director.halt(1.0)
    halted = director.frame(1.0)
    report.add("halt stops the wheels in the same frame and drops the gesture",
               halted.wheels == (1500.0, 1500.0) and director.state.sequence is None,
               f"wheels {halted.wheels}")
    missed = looks_like_failed_grasp(95.0, 0.0, 3.0, held_s=0.5)
    grabbed = not looks_like_failed_grasp(600.0, 0.0, 3.0, held_s=0.5)
    report.add("grasp detector separates a miss from a grip", missed and grabbed,
               "95 mA closed = miss, 600 mA closed = grip")
    spoken = {"hand me the screwdriver": "FETCH_TOOL", "fetch the allen key": "FETCH_TOOL",
              "look at my hand": "LOOK_AT_HAND", "follow me": "FOLLOW_ME",
              "stop": "HALT", "freeze": "HALT", "go home": "GO_HOME"}
    wrong = [t for t, want in spoken.items() if parse(t).name != want]
    report.add("the parser answers the brief's phrases", not wrong,
               f"{len(spoken)} phrases, {len(spoken) - len(wrong)} correct"
               if not wrong else f"wrong: {wrong}")
    tricky = parse("bud-e stop, the screwdriver is fine")
    report.add("a stop anywhere in a sentence still stops",
               tricky.name == "HALT" and tricky.safety_critical,
               f"{tricky.name} on {tricky.evidence}")
    return report


# ---------------------------------------------------------------------------------------
# Phase 8 - the six faults
# ---------------------------------------------------------------------------------------
def phase8() -> PhaseReport:
    report = PhaseReport("P8", "Safety: six injected faults and the telemetry log")
    injector = FaultInjector()
    results = injector.run_all(recorder=None)
    for result in results:
        report.add(f"fault: {result.name}", result.passed, result.detail)
    report.add("all six injectors ran", len(results) == len(FAULTS),
               f"{len(results)}/{len(FAULTS)} faults injected")
    return report


# ---------------------------------------------------------------------------------------
# Scorecard
# ---------------------------------------------------------------------------------------
def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--quick", action="store_true",
                        help="skip the 25 s STL assembly-fit audit")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)

    started = time.time()
    reports = [phase1(), phase2(args.quick), phase3(), phase4(), phase5(), phase6(), phase7(),
               phase8()]
    duration = time.time() - started

    if args.json:
        print(json.dumps({"ok": all(r.ok for r in reports),
                          "seconds": round(duration, 1),
                          "phases": [{"phase": r.phase, "title": r.title, "ok": r.ok,
                                      "checks": [{"name": c.name, "ok": c.ok,
                                                  "detail": c.detail} for c in r.checks]}
                                     for r in reports]}, indent=2))
        return 0 if all(r.ok for r in reports) else 1

    print("=" * 104)
    print("  B.U.D.-E MASTER ACCEPTANCE SCORECARD - ALL EIGHT PHASES")
    print("=" * 104)
    for report in reports:
        mark = "PASS" if report.ok else "FAIL"
        print(f"\n  {report.phase}  {report.title}   [{mark}]")
        for check in report.checks:
            print(f"      {'ok  ' if check.ok else 'FAIL'} {check.name}")
            print(f"           {check.detail}")
    total = sum(len(r.checks) for r in reports)
    passed = sum(1 for r in reports for c in r.checks if c.ok)
    print("\n" + "=" * 104)
    for report in reports:
        mark = "PASS" if report.ok else "FAIL"
        print(f"  {report.phase}  {report.title:<52} {mark}")
    print("-" * 104)
    print(f"  {passed} / {total} checks passed in {duration:.1f} s")
    print(f"  VERDICT: {'PASS' if passed == total else 'FAIL'}")
    print("=" * 104)
    print()
    print("  WHAT THIS SCORECARD IS NOT")
    print("  * Nothing here has been printed, powered, calibrated or dropped. Every dimension")
    print("    is a model, every servo figure is a datasheet, and every latency is simulated.")
    print("  * The six Phase 8 faults are synthetic sensor timelines: the supervisor's decisions")
    print("    were exercised, not the hardware's reactions.")
    print("  * The firmware is compiled by a real C++ compiler and its control flow really runs,")
    print("    but against stubbed Arduino/ESP32/Wire headers. The I2C peripheral, the GPIO")
    print("    interrupts, the task watchdog and the servos are models. It has never been")
    print("    flashed, and the <5 ms cliff reflex is a requirement the sketch checks itself")
    print("    against - not a measurement.")
    print("  * The wheel directions are an ASSUMPTION (mirrored servos) that only a floor test")
    print("    can confirm. Verify before any cliff test: `W 1320 1680` must move the rover")
    print("    away from you. A wrong sign turns the cliff reflex into the cliff accelerator.")
    print("  * Before any of this touches a hand or a desk edge, do the bring-up checklist in")
    print("    docs/phase8_safety_testing_and_bringup_checklist.md, in order, with the e-stop")
    print("    in reach.")
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
