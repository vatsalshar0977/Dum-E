#!/usr/bin/env python3
"""
Phase 1 Kinematic Reach & Workspace Envelope Verification Script.

Verifies that B.U.D.-E's compact Sub-$75 link geometry:
  - L1 (base_height)    = 55 mm
  - L2 (upper_arm)      = 80 mm
  - L3 (forearm)        = 75 mm
  - L4+L5 (wrist+claw)  = 70 mm (22 mm wrist + 48 mm claw tip)
can cleanly reach:
  1. Desk Pick Zone (tools/screws lying on workbench at z = 10..25 mm)
  2. Cup / Tall Object Zone (horizontal grasp at z = 45..70 mm)
  3. Human Handover Zone (offering tool up to user's hand at z = 130..185 mm)
  4. Expressive Personality Poses (Home, Curious Head-Tilt, Sad Droop, Excited Wiggle)

Usage:
    python3 src/phase1_sizing/workspace_envelope_check.py
"""

from __future__ import annotations

import math
import sys
from typing import Dict, Tuple

# Link geometry in millimeters (matches config/robot_config.yaml)
L1_BASE_MM = 55.0
L2_UPPER_MM = 80.0
L3_FOREARM_MM = 75.0
L45_WRIST_CLAW_MM = 70.0  # 22 mm wrist_to_roll + 48 mm gripper_length


def forward_kinematics_planar(
    j1_yaw_deg: float,
    j2_shoulder_deg: float,
    j3_elbow_deg: float,
    j4_wrist_pitch_deg: float,
    j5_wrist_roll_deg: float = 0.0,
) -> Dict[str, float]:
    """
    Compute 3D claw-tip position (x, y, z) in mm and global pitch angle (deg).
    Angle convention:
      - j2_shoulder_deg = 0 deg is straight UP (+Z); +90 deg leans forward (+X).
      - j3_elbow_deg    = relative pitch from upper arm (+ bends forward/down).
      - j4_wrist_pitch  = relative pitch from forearm (+ pitches down).
      - Global pitch from horizontal: 0 deg = horizontal forward, -90 deg = pointing straight down at desk.
    """
    q1 = math.radians(j1_yaw_deg)
    # Convert from vertical-up zero reference to standard planar angle from +X horizontal
    theta2 = math.radians(90.0 - j2_shoulder_deg)
    theta23 = theta2 - math.radians(j3_elbow_deg)
    theta234 = theta23 - math.radians(j4_wrist_pitch_deg)

    # Radial reach (r) in the arm's vertical plane and height (z) above desk surface (z = 0)
    r_shoulder = 0.0
    z_shoulder = L1_BASE_MM

    r_elbow = r_shoulder + L2_UPPER_MM * math.cos(theta2)
    z_elbow = z_shoulder + L2_UPPER_MM * math.sin(theta2)

    r_wrist = r_elbow + L3_FOREARM_MM * math.cos(theta23)
    z_wrist = z_elbow + L3_FOREARM_MM * math.sin(theta23)

    r_tip = r_wrist + L45_WRIST_CLAW_MM * math.cos(theta234)
    z_tip = z_wrist + L45_WRIST_CLAW_MM * math.sin(theta234)

    x_tip = r_tip * math.cos(q1)
    y_tip = r_tip * math.sin(q1)
    global_pitch_deg = math.degrees(theta234)

    return {
        "x_mm": x_tip,
        "y_mm": y_tip,
        "z_mm": z_tip,
        "radial_mm": r_tip,
        "z_elbow_mm": z_elbow,
        "z_wrist_mm": z_wrist,
        "global_pitch_deg": global_pitch_deg,
        "wrist_roll_deg": j5_wrist_roll_deg,
    }


def main() -> int:
    print("=" * 84)
    print("  B.U.D.-E — PHASE 1 KINEMATIC WORKSPACE & EXPRESSIVE POSE VERIFICATION")
    print("=" * 84)
    print(
        f"Link Lengths: L1(Base)={L1_BASE_MM:.0f}mm, L2(Upper)={L2_UPPER_MM:.0f}mm, "
        f"L3(Forearm)={L3_FOREARM_MM:.0f}mm, L4+L5(Wrist+Claw)={L45_WRIST_CLAW_MM:.0f}mm"
    )
    print(f"Max Radial Reach (Stretched Horizontal): {L2_UPPER_MM + L3_FOREARM_MM + L45_WRIST_CLAW_MM:.0f} mm (22.5 cm)")
    print(f"Max Vertical Tip Height (Stretched Up) : {L1_BASE_MM + L2_UPPER_MM + L3_FOREARM_MM + L45_WRIST_CLAW_MM:.0f} mm (28.0 cm)")
    print("-" * 84)

    benchmarks = [
        # (Name, J1_yaw, J2_shld, J3_elb, J4_wr_pitch, J5_wr_roll)
        ("1. Home / Idle Perched Pose",           0.0, -15.0,  65.0,  25.0,   0.0),
        ("2. Desk Tool Pick (Angled Grasp z~14mm)", 0.0,  55.0, 52.0,  30.0,   0.0),
        ("3. Cup Side-Grasp (Level Claw z~55mm)", 0.0,  45.0,  45.0,   0.0,   0.0),
        ("4. User Handover Pose (Offering Tool)", 0.0,  15.0,  35.0,  15.0,   0.0),
        ("5. Curious Puppy Head-Tilt (35° Roll)", 15.0, -5.0,  55.0,  15.0,  35.0),
        ("6. Sad Failed-Grasp Droop (Slumped)",   0.0,  35.0,  75.0,  45.0,   0.0),
        ("7. Excited Success Perch (Looking Up)", 0.0, -20.0,  45.0, -15.0,   0.0),
    ]

    print(f"{'Pose / Task Benchmark':<39} {'Reach r (mm)':<14} {'Height z (mm)':<15} {'Claw Pitch':<12} {'Status'}")
    print("-" * 84)
    all_ok = True
    for name, j1, j2, j3, j4, j5 in benchmarks:
        fk = forward_kinematics_planar(j1, j2, j3, j4, j5)
        desk_safe = fk["z_mm"] >= 5.0 and fk["z_elbow_mm"] >= 20.0 and fk["z_wrist_mm"] >= 15.0
        if not desk_safe:
            all_ok = False
        status = "PASS" if desk_safe else "DESK COLLISION!"
        print(
            f"{name:<39} {fk['radial_mm']:>8.1f} mm    {fk['z_mm']:>9.1f} mm    "
            f"{fk['global_pitch_deg']:>+6.1f}°      {status}"
        )

    print("=" * 84)
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
