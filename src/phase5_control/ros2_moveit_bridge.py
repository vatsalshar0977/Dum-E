#!/usr/bin/env python3
"""Cartesian pick-and-handover bridge, in two modes.

MODE 1 - pure Python (`--mode python`, the default)
    Plans and VALIDATES every stage with the schematic kinematics and the minimum-jerk
    planner, then prints the stage table. It does not move anything. All "current" numbers
    come from an explicit declared model, not from a sensor.

MODE 2 - ROS 2 (`--mode ros2`)
    Publishes the same stages as trajectory_msgs/JointTrajectory to
    /bude_arm_controller/joint_trajectory and listens on /joint_states. This mode needs
    rclpy, a running controller_manager and a hardware interface, none of which exist in
    this repository. It has NOT been executed here. Verify on your own machine.

HONESTY NOTES - please read before wiring anything
  * There is no hardware driver in this repository. The ESP32 serial protocol is not
    implemented. Nothing here has driven a motor.
  * The claw is one scalar open/close fraction in this model. The URDF has no finger joint.
  * "GRASPED" in the table means "the declared current model crossed the declared
    threshold", not "the arm picked up your screwdriver".
  * The INA219 sits on the whole servo rail, so it cannot tell you WHICH joint stalled, and
    an I2C poll is not a collision safety function. Keep people out of the workspace and
    use a physical e-stop.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import math
from pathlib import Path
import sys
from typing import List, Optional, Sequence, Tuple

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from src.phase5_control.kinematics import (  # noqa: E402
    inverse_kinematics_5dof,
    forward,
)
from src.phase5_control.trajectory_and_collision import (  # noqa: E402
    MinJerkPlan,
    check_pose_safety,
    plan_min_jerk_trajectory,
)

# Named poses, identical to ros2_ws/src/bude_moveit_config/config/bude_arm.srdf
NAMED_POSES_DEG = {
    "home": (0.0, -15.0, 40.0, -25.0, 0.0),
    "curious_tilt": (15.0, 0.0, 35.0, -15.0, 20.0),
    "handover": (0.0, 11.8, 38.6, 19.5, 0.0),
    "sad_droop": (0.0, 25.8, 84.8, 39.4, 0.0),
}

DESK_HEIGHT_MM = 0.0
PRE_GRASP_CLEARANCE_MM = 55.0
LIFT_CLEARANCE_MM = 70.0
APPROACH_PITCH_DEG = -50.0
GRASP_PITCH_DEG = -60.0
HANDOVER_PITCH_DEG = 20.0


@dataclass
class GraspMonitor:
    """Rail-current tripper with an explicitly declared threshold.

    The 450 mA figure is a PLACEHOLDER from the design brief. It has to be measured on your
    hardware: close the claw on each object you care about, log the current, and pick a
    threshold with margin. It also assumes the arm is holding still while the claw closes -
    if another joint is moving, its current is in the same measurement and the number means
    nothing.
    """

    threshold_ma: float = 450.0
    full_scale_ma: float = 3200.0
    samples: List[float] = field(default_factory=list)
    state: str = "idle"

    def observe(self, current_ma: Optional[float]) -> str:
        if current_ma is None or not math.isfinite(current_ma):
            self.state = "UNKNOWN - hold and stop closing"
            return self.state
        self.samples.append(current_ma)
        if current_ma >= self.full_scale_ma:
            self.state = "OVERCURRENT - stop all motion"
        elif current_ma >= self.threshold_ma:
            self.state = "GRASPED (declared model)"
        else:
            self.state = "closing"
        return self.state


@dataclass
class StageResult:
    index: int
    action: str
    target_mm: Tuple[float, float, float]
    solved_mm: Tuple[float, float, float]
    duration_s: float
    min_tcp_z_mm: float
    current_ma: Optional[float]
    result: str
    note: str = ""


class ArmBridge:
    """Plans and validates a Cartesian pick-and-handover against the schematic model."""

    def __init__(self, q_start: Sequence[float] = NAMED_POSES_DEG["home"]) -> None:
        report = check_pose_safety(q_start)
        if not report.ok:
            raise ValueError(f"start pose rejected: {report.describe()}")
        self.q = tuple(q_start)
        self.claw = 1.0                      # 1.0 fully open, 0.0 fully closed
        self.grasp = GraspMonitor()
        self.results: List[StageResult] = []

    # -- kinematics ------------------------------------------------------------------
    def solve(self, target_mm: Sequence[float], pitch: float, roll: float = 0.0):
        """Solve IK, preferring the exact pitch and falling back to the nearest reachable one.

        The fallback is DELIBERATE and is logged: the achieved pitch is recorded in the
        stage table so nobody has to guess which angle the arm actually took.
        """
        x, y, z = target_mm
        solutions = inverse_kinematics_5dof(x, y, z, pitch, roll, adapt_pitch=True)
        for q, achieved in solutions:
            if check_pose_safety(q).ok:
                return q, achieved
        raise ValueError(f"every IK branch for {target_mm} at pitch {pitch} fails keep-outs")

    # -- motion ----------------------------------------------------------------------
    def move_to(self, index: int, action: str, target_mm, pitch, roll=0.0,
                current_ma: Optional[float] = None, note: str = "") -> StageResult:
        q_goal, achieved_pitch = self.solve(target_mm, pitch, roll)
        plan: MinJerkPlan = plan_min_jerk_trajectory(self.q, q_goal)
        tcp = forward(q_goal)
        self.q = q_goal
        stage_note = note
        if abs(achieved_pitch - pitch) > 1e-6:
            stage_note = (stage_note + f" pitch adapted {pitch:.0f} -> {achieved_pitch:.1f} deg"
                          ).strip()
        result = StageResult(index, action, tuple(target_mm), (tcp.x, tcp.y, tcp.z),
                             plan.duration_s, plan.min_tcp_z_mm, current_ma, "PASS", stage_note)
        self.results.append(result)
        return result

    def pick_and_handover(self, target_mm, *, model_current_ma: float = 980.0) -> List[StageResult]:
        x, y, z = target_mm

        self.move_to(1, "Approach pre-grasp", (x, y, z + PRE_GRASP_CLEARANCE_MM),
                     APPROACH_PITCH_DEG, current_ma=model_current_ma)
        self.move_to(2, "Descend to grasp", (x, y, z), GRASP_PITCH_DEG,
                     current_ma=model_current_ma)

        # Claw close. The object is "detected" only if the declared current model crosses
        # the declared threshold; nothing here measures anything.
        seen = 0.0
        state = "closing"
        while self.claw > 0.0 and seen < 1.4:
            seen = round(seen + 0.15, 2)
            state = self.grasp.observe(model_current_ma if seen >= 0.9 else 320.0)
            if state.startswith("GRASPED") or state.startswith("OVERCURRENT"):
                break
        self.claw = 0.0 if state.startswith("GRASPED") else 0.35
        self.results.append(StageResult(
            3, "Close claw", (x, y, z), (x, y, z), 0.9, z,
            model_current_ma, state,
            "threshold is a DECLARED placeholder; measure it per object",
        ))
        if state.startswith("OVERCURRENT"):
            self.results.append(StageResult(
                3, "Abort", (x, y, z), (x, y, z), 0.0, z, model_current_ma,
                "OVERCURRENT - stop all motion", "physical e-stop first, then diagnose",
            ))
            return self.results

        self.move_to(4, "Lift clear", (x, y, z + LIFT_CLEARANCE_MM), -35.0,
                     current_ma=model_current_ma)
        handover = forward(NAMED_POSES_DEG["handover"])
        hx, hy, hz = handover.x, handover.y, handover.z
        self.move_to(5, "Present to handover", (hx, hy, hz), HANDOVER_PITCH_DEG,
                     current_ma=model_current_ma)
        self.claw = 1.0
        self.results.append(StageResult(
            6, "Open claw + nod", (hx, hy, hz), (hx, hy, hz), 0.65, hz, 520.0,
            "RELEASED", "expressive nod is a joint-space gesture; keep it slow near people",
        ))
        return self.results


def print_table(results: Sequence[StageResult]) -> None:
    print("-" * 112)
    print(f"{'#':<3} {'Stage':<24} {'Target (mm)':<22} {'Solved TCP (mm)':<22} "
          f"{'Time':>6} {'Min Z':>8} {'I/mA':>7} {'Result'}")
    print("-" * 112)
    for r in results:
        print(f"{r.index:<3} {r.action:<24} "
              f"{f'({r.target_mm[0]:.0f}, {r.target_mm[1]:.0f}, {r.target_mm[2]:.0f})':<22} "
              f"{f'({r.solved_mm[0]:.1f}, {r.solved_mm[1]:.1f}, {r.solved_mm[2]:.1f})':<22} "
              f"{r.duration_s:>5.2f}s {r.min_tcp_z_mm:>7.1f} "
              f"{('--' if r.current_ma is None else f'{r.current_ma:.0f}'):>7} {r.result}")
        if r.note:
            print(f"    note: {r.note}")
    print("-" * 112)


def run_ros2(bridge: ArmBridge, target_mm) -> int:
    """Publish the same stages over ROS 2. NOT EXECUTED IN THIS REPOSITORY."""
    try:
        import rclpy  # noqa: F401
        from rclpy.node import Node  # noqa: F401
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # noqa: F401
    except ImportError as exc:
        print("ROS 2 mode needs rclpy and trajectory_msgs, which are not installed here.")
        print("Install ROS 2 and source its setup, e.g.:")
        print("  source /opt/ros/jazzy/setup.bash")
        print(f"import error was: {exc}")
        return 2

    print("ROS 2 mode selected. Before publishing to a real controller:")
    print("  1. The node must refuse commands older than a few control periods.")
    print("  2. The node must require an explicit re-arm after an e-stop.")
    print("  3. The node must compare MEASURED joint positions with commanded ones. With")
    print("     unmodified hobby servos there is nothing to compare; add the optional")
    print("     potentiometer tap or use servos with real encoders.")
    print("  4. Publish to /bude_arm_controller/joint_trajectory only with the arm clear.")
    print()
    print("This path is a template that has never been run. Test it against mock hardware")
    print("(`ros2_control` mock_components) before a real arm, and expect to rewrite it.")
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mode", choices=["python", "ros2"], default="python")
    parser.add_argument("--dry-run", action="store_true",
                        help="alias for --mode python: plan offline with the analytic solver and "
                             "print the stage table without touching ROS 2 or any hardware")
    parser.add_argument("--target", type=float, nargs=3, metavar=("X", "Y", "Z"),
                        default=[150.0, 0.0, 12.0],
                        help="object location on the desk in mm (default 150 0 12)")
    parser.add_argument("--model-current-ma", type=float, default=980.0,
                        help="declared rail current for the stage table; NOT measured")
    args = parser.parse_args(argv)
    if args.dry_run and args.mode != "python":
        parser.error("--dry-run and --mode ros2 are mutually exclusive")

    print("=" * 112)
    print("  PHASE 5 - CARTESIAN PICK AND HANDOVER BRIDGE (schematic model, offline)")
    print("  No hardware driver exists in this repository. No motor has been commanded.")
    if args.dry_run:
        print("  DRY RUN: the analytic 5-DOF solver plans this, not MoveIt 2 and not a servo.")
    print("=" * 112)

    try:
        bridge = ArmBridge()
    except ValueError as exc:
        print(f"start pose rejected: {exc}")
        return 1

    # Stage 0: minimal single-joint step, the smallest thing that can be wrong.
    plan = plan_min_jerk_trajectory(bridge.q, (20.0,) + bridge.q[1:])
    print(f"[STAGE 0] single-joint J1 -> +20.0 deg : {plan.summary()}")
    print()

    try:
        results = bridge.pick_and_handover(tuple(args.target),
                                           model_current_ma=args.model_current_ma)
    except ValueError as exc:
        print(f"planning refused this task: {exc}")
        return 1

    print_table(results)
    print("'Solved TCP' is the FORWARD KINEMATICS of the planned joints, not a measurement.")
    print("'Min Z' is the lowest tool height the planner checked, from the analytic model.")
    print()

    if args.mode == "ros2":
        return run_ros2(bridge, tuple(args.target))

    print("Next physical step is NOT this script. It is: one servo, unloaded, at a low")
    print("speed, on a bench, with a rated power cutoff within reach. See docs/verification_status.md.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
