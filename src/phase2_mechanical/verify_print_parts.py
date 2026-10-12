#!/usr/bin/env python3
"""
Phase 2 Mechanical & STL Integrity Verification Script.

Verifies:
  1. All 11 binary STL files in `cad/stl/` exist and have exact byte integrity (84 + 50 * N_tris).
  2. All vertex coordinates are finite and fit within a 180 x 180 x 180 mm build volume.
  3. Outputs the exact servo wire extension & slack-loop table for J1..J6 + USB camera.

Usage:
    python3 src/phase2_mechanical/verify_print_parts.py
"""

from __future__ import annotations

import math
from pathlib import Path
import struct
import sys
from typing import Tuple


def inspect_binary_stl(path: Path) -> Tuple[int, Tuple[float, float, float]]:
    """Verify binary STL byte layout and return (num_triangles, (dx, dy, dz))."""
    raw = path.read_bytes()
    if len(raw) < 84:
        raise ValueError(f"{path.name}: File too small ({len(raw)} bytes) to be a valid binary STL.")
    num_tris = struct.unpack("<I", raw[80:84])[0]
    expected_len = 84 + 50 * num_tris
    if len(raw) != expected_len:
        raise ValueError(f"{path.name}: Byte length mismatch (got {len(raw)}, expected {expected_len}).")

    xs, ys, zs = [], [], []
    offset = 84
    for _ in range(num_tris):
        vals = struct.unpack("<12fH", raw[offset:offset + 50])
        offset += 50
        for v in vals[:12]:
            if not math.isfinite(v):
                raise ValueError(f"{path.name}: Non-finite coordinate/normal detected.")
        xs.extend([vals[3], vals[6], vals[9]])
        ys.extend([vals[4], vals[7], vals[10]])
        zs.extend([vals[5], vals[8], vals[11]])

    dx = max(xs) - min(xs)
    dy = max(ys) - min(ys)
    dz = max(zs) - min(zs)
    return num_tris, (dx, dy, dz)


def main() -> int:
    stl_dir = Path(__file__).resolve().parents[2] / "cad" / "stl"
    expected_files = [
        "00_tolerance_test_coupon.stl",
        "01_desk_c_clamp_frame.stl",
        "02_desk_c_clamp_screw_and_pad.stl",
        "03_base_lower_slewing_ring.stl",
        "04_base_upper_turret_j2.stl",
        "05_upper_arm_link_80mm.stl",
        "06_forearm_link_75mm.stl",
        "07_wrist_pitch_roll_bracket.stl",
        "08_gripper_body_and_cam_mount.stl",
        "09_gripper_geared_fingers_pair.stl",
        "10_cable_clips_and_electronics_tray.stl",
    ]

    print("=" * 88)
    print("  B.U.D.-E — PHASE 2 STL BYTE LAYOUT & BUILD-PLATE SIZE ONLY")
    print("  WARNING: Does NOT test holes, watertightness, mating fit or strength; see docs/verification_status.md")
    print("=" * 88)
    print(f"{'Part Filename':<40} {'Size (X×Y×Z mm)':<24} {'Tris':<8} {'180mm Bed Check'}")
    print("-" * 88)

    all_passed = True
    for fname in expected_files:
        fpath = stl_dir / fname
        if not fpath.exists():
            print(f"[MISSING] {fname}")
            all_passed = False
            continue
        tris, (dx, dy, dz) = inspect_binary_stl(fpath)
        fits_mini = dx <= 180.0 and dy <= 180.0 and dz <= 180.0
        if not fits_mini:
            all_passed = False
        status = "PASS (Fits 180x180)" if fits_mini else "FAIL (Too large)"
        print(f"{fname:<40} {dx:5.1f}×{dy:5.1f}×{dz:4.1f} mm    {tris:<8d} {status}")

    print("\n" + "=" * 88)
    print("  UNVERIFIED CABLE LENGTH ESTIMATES — MEASURE YOUR BUILD FIRST")
    print("=" * 88)
    cable_rows = [
        ("J1 Base Yaw (MG996R)",      "PCA9685 Ch 0", "250 mm (Stock lead)", "35 mm loop inside base turret"),
        ("J2 Shoulder Pitch (MG996R)","PCA9685 Ch 1", "250 mm (Stock lead)", "45 mm loop across J1->J2 waist"),
        ("J3 Elbow Pitch (MG996R)",   "PCA9685 Ch 2", "250 mm + 150 mm ext", "40 mm loop over J2 shoulder bridge"),
        ("J4 Wrist Pitch (MG90S)",    "PCA9685 Ch 3", "250 mm + 300 mm ext", "35 mm loop over J3 elbow pivot"),
        ("J5 Wrist Roll (MG90S)",     "PCA9685 Ch 4", "250 mm + 300 mm ext", "45 mm helical loop for +-90° roll"),
        ("J6 Claw Gripper (MG90S)",   "PCA9685 Ch 5", "250 mm + 350 mm ext", "45 mm helical loop coupled w/ J5"),
        ("Eye-in-Hand USB Camera",    "Laptop USB",   "1500 mm soft USB 2.0", "50 mm service loop at wrist + base clip"),
    ]
    print(f"{'Joint / Sensor':<29} {'Destination':<14} {'Total Wire Length':<21} {'Required Joint Slack Loop'}")
    print("-" * 88)
    for j_name, dest, wire_len, slack in cable_rows:
        print(f"{j_name:<29} {dest:<14} {wire_len:<21} {slack}")
    print("=" * 88)

    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())
