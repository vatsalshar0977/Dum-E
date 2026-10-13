"""Run every phase entry point, in order, and write one transcript.

This is a DIFFERENT thing from `run_full_acceptance_suite.py`, and the difference matters:

  * `run_full_acceptance_suite.py` asserts requirements. It is the thing that fails a build.
  * this script runs the programs a human would type, in the order they would type them, and
    saves exactly what they printed. It is a walkthrough, not a test: a phase "PASSES" here
    when its script ran and its own summary said so, which is a weaker claim.

Both are honest about the same limitation: nothing here has been printed, powered or calibrated.

Usage:
    python3 src/phase8_safety_testing/run_phase_walkthrough.py
    python3 src/phase8_safety_testing/run_phase_walkthrough.py --out docs/my_transcript.txt
    python3 src/phase8_safety_testing/run_phase_walkthrough.py --only 3 8   # selected phases
"""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUT = ROOT / "docs" / "phase_walkthrough_transcript.txt"
TIMEOUT_S = 900


@dataclass
class Step:
    number: int
    title: str
    blurb: str
    commands: List[List[str]] = field(default_factory=list)
    notes: List[str] = field(default_factory=list)


def fw_tests_if_possible() -> List[List[str]]:
    """`pytest -v` when pytest exists, otherwise the stdlib runner on the same tests."""
    if shutil.which("pytest") is not None or _has_module("pytest"):
        return [[sys.executable, "-m", "pytest", "-v"]]
    return [[sys.executable, "-m", "unittest", "-v",
             "tests.test_foundations", "tests.test_phase5_phase6",
             "tests.test_phase7_phase8", "tests.test_firmware"]]


def _has_module(name: str) -> bool:
    try:
        __import__(name)
        return True
    except ImportError:
        return False


STEPS: Sequence[Step] = (
    Step(1, "Phase 1 — Torque, payload and power budget",
         "Sizes the servos against the link masses and the workspace the arm must reach.",
         [
             [sys.executable, "src/phase1_sizing/torque_and_power_calc.py"],
             [sys.executable, "src/phase1_sizing/single_joint_ping.py", "--dry-run"],
         ]),
    Step(2, "Phase 2 — 13 STL printability and the 206-point fit audit",
         "Every printed part: watertight mesh, true hollow cavities, coaxial bores, straddle "
         "clearances, and the rover bolt circle.",
         [
             [sys.executable, "src/phase2_mechanical/verify_print_parts.py"],
             [sys.executable, "src/phase2_mechanical/verify_assembly_fit.py"],
         ]),
    Step(3, "Phase 3 — Electronics pinout, voltage drop and the ESP32 driver",
         "Wiring table vs config vs firmware, rail arithmetic, then the host-side serial driver.",
         [
             [sys.executable, "src/phase3_electronics/power_and_wiring_verifier.py"],
             [sys.executable, "src/phase3_electronics/esp32_arm_driver.py", "--dry-run",
              "--ping", "--home", "--drive", "0.5", "0.5", "--stop"],
         ]),
    Step(4, "Phase 4 — URDF model and the pick-and-handover simulation",
         "The kinematic model checked against the solver, then a full task run offline.",
         [
             [sys.executable, "src/phase4_simulation/verify_urdf_model.py"],
             [sys.executable, "src/phase4_simulation/sim_runner.py", "--backend", "kinematic"],
         ]),
    Step(5, "Phase 5 — FK, 5-DOF IK, minimum-jerk trajectories and collision guard",
         "The control core: reachability, the quintic profile, keep-out volumes, and the "
         "unfinished ROS 2 bridge.",
         [
             [sys.executable, "src/phase5_control/kinematics.py"],
             [sys.executable, "src/phase5_control/trajectory_and_collision.py"],
             [sys.executable, "src/phase5_control/ros2_moveit_bridge.py", "--dry-run"],
         ]),
    Step(6, "Phase 6 — Camera calibration model and mobile visual navigation",
         "Pinhole intrinsics, hand-eye offset, a synthetic round trip, and the drive policy.",
         [
             [sys.executable, "src/phase6_perception/camera_and_hand_eye_calib.py"],
             [sys.executable, "src/phase6_perception/vision_mobile_navigator.py"],
         ]),
    Step(7, "Phase 7 — Voice parser and the expressive personality gestures",
         "Six gestures as minimum-jerk keyframes, the deterministic intent parser, and the "
         "failure detector.",
         [
             [sys.executable, "src/phase7_voice_personality/personality_engine.py"],
             [sys.executable, "src/phase7_voice_personality/voice_assistant.py", "--demo"],
         ]),
    Step(8, "Phase 8 — Safety supervisor, master acceptance suite and unit tests",
         "Six injected faults, then every phase re-checked as an assertion, then the tests.",
         [
             [sys.executable, "src/phase8_safety_testing/safety_supervisor_and_logger.py",
              "--fault", "all", "--seconds", "6", "--no-log"],
             [sys.executable, "src/phase8_safety_testing/run_full_acceptance_suite.py"],
         ] + fw_tests_if_possible()),
)

ROW_RE = re.compile(
    r"(\d+)\s*/\s*(\d+)|(\d+)\s+of\s+(\d+)|PASS:\s*(\d+)|(\d+)\s+passed|VERDICT:\s*(\w+)")


def summarise(output: str) -> str:
    """Pull the headline number out of a script's own output, without inventing one."""
    verdicts = re.findall(r"VERDICT:\s*([A-Za-z ]+)", output)
    if verdicts:
        return f"VERDICT: {verdicts[-1].strip()}"
    collected = re.findall(r"(\d+) collected", output)
    passed = re.findall(r"(\d+) passed", output)
    if passed and collected:
        return f"{passed[-1]} passed / {collected[-1]} collected"
    if passed:
        return f"{passed[-1]} passed"
    for pattern in (r"(\d+)\s*/\s*(\d+) checks", r"TOTAL\s+(\d+)\s*/\s*(\d+)",
                    r"(\d+)\s*/\s*(\d+) checks passed", r"(\d+)\s+of\s+(\d+)"):
        matches = re.findall(pattern, output)
        if matches:
            got, total = matches[-1][:2]
            return f"{got} / {total}"
    if "PASS" in output:
        return "PASS"
    return "ran"


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Run every phase entry point in order.")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT,
                        help=f"transcript path (default {DEFAULT_OUT})")
    parser.add_argument("--only", type=int, nargs="+", metavar="PHASE",
                        help="run only these phase numbers")
    parser.add_argument("--quiet", action="store_true", help="do not echo output as it runs")
    args = parser.parse_args(argv)

    steps = [s for s in STEPS if not args.only or s.number in args.only]
    started = time.strftime("%Y-%m-%d %H:%M:%S")
    lines: List[str] = []
    results: List[Dict[str, object]] = []
    failures = 0

    def emit(text: str = "") -> None:
        lines.append(text)
        if not args.quiet:
            print(text)

    emit("=" * 96)
    emit("  B.U.D.-E — PHASE-BY-PHASE WALKTHROUGH")
    emit(f"  Generated {started} by src/phase8_safety_testing/run_phase_walkthrough.py")
    emit("=" * 96)
    emit("  Every command below is the command you would type. The output is verbatim.")
    emit("  NOTHING IN THIS TRANSCRIPT WAS MEASURED ON HARDWARE: no part has been printed,")
    emit("  no servo powered, no camera calibrated, no floor driven. Dimensions are models,")
    emit("  servo figures are datasheets, and every latency is simulated.")
    emit("=" * 96)

    for step in steps:
        emit()
        emit("#" * 96)
        emit(f"#  STEP {step.number} — {step.title}")
        emit("#" * 96)
        emit(f"#  {step.blurb}")
        emit()
        for command in step.commands:
            # Show `python3 ...` rather than the interpreter's absolute path: this transcript is
            # meant to be copy-pasteable, including onto a Windows box where it becomes `python`.
            printable = " ".join([sys.executable.replace(sys.executable, "python3")]
                                 + list(command[1:]))
            emit("$ " + printable)
            emit("-" * 96)
            try:
                done = subprocess.run(command, cwd=ROOT, capture_output=True, text=True,
                                      timeout=TIMEOUT_S)
                output = done.stdout + done.stderr
                code = done.returncode
            except subprocess.TimeoutExpired:
                output = f"(timed out after {TIMEOUT_S} s)"
                code = -1
            emit(output.rstrip("\n"))
            emit("-" * 96)
            headline = summarise(output)
            status = "OK" if code == 0 else f"EXIT {code}"
            emit(f"  -> exit {code} ({status})   summary: {headline}")
            results.append({"phase": step.number, "command": printable, "exit": code,
                            "summary": headline})
            if code != 0:
                failures += 1
            emit()

    emit("=" * 96)
    emit("  WALKTHROUGH SUMMARY")
    emit("=" * 96)
    emit(f"  {'PHASE':<7}{'EXIT':<7}RESULT")
    for item in results:
        emit(f"  {str(item['phase']):<7}{str(item['exit']):<7}{item['summary']}"
             f"\n         $ {item['command']}")
    emit()
    emit(f"  {len(results) - failures} of {len(results)} commands exited 0.")
    emit()
    emit("  WHAT THIS DOES NOT MEAN")
    emit("  * 'PASS' here means a script's own self-check agreed with itself. It is not an")
    emit("    independent verification, and it is not a measurement of anything physical.")
    emit("  * No STL has been sliced or printed; the fit audit is arithmetic on vertices.")
    emit("  * The firmware has never been flashed; its checks run against stubbed headers.")
    emit("  * Before any of this touches a hand or a desk edge, do the bring-up checklist in")
    emit("    docs/phase8_safety_testing_and_bringup_checklist.md, in order, e-stop in reach.")
    emit("=" * 96)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"\n  transcript written to {args.out.relative_to(ROOT)} "
          f"({args.out.stat().st_size / 1024:.0f} KB)")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
