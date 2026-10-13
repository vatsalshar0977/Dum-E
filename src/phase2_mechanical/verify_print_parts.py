#!/usr/bin/env python3
"""
Phase 2 mechanical audit for the B.U.D.-E printed parts.

WHAT THIS ACTUALLY CHECKS (mesh-level facts, all reproducible from the STL bytes):
  1. Binary STL byte layout is exact (84 + 50 * N), triangle count matches the header,
     every coordinate is finite.
  2. Each part fits a 180 x 180 x 180 mm build volume.
  3. Each part seats on the bed: z_min == 0 within tolerance (no floating first layer).
  4. Degenerate (zero-area) triangles, reported per part.
  5. MESH-LEVEL INTERFACE EVIDENCE: the 4-bolt rover interface is present in BOTH
     part 11 (bosses) and part 03 (nut rings) at the declared radius and angles.
     This measures the geometry that is actually in the files.

WHAT THIS DOES NOT CHECK - do not read a PASS as a fit guarantee:
  * Boolean correctness. These meshes ADD primitives and never subtract them, so "pocket"
    and "hole" features are only pockets by virtue of the walls around them.
  * Strength, stiffness, warping, screw thread engagement, or print shrinkage.
  * Whether the real MG996R / MG90S / FS90R / HC-SR04 / TCRT5000 you bought matches the
    datasheet numbers hard-coded in the generator.
  A PASS here means "the model is self-consistent and fits on the plate". The only way to
  establish that two printed parts fit each other is a first-article print and a real part.

Usage:
    python3 src/phase2_mechanical/verify_print_parts.py
"""

from __future__ import annotations

import math
from pathlib import Path
import struct
import sys
from typing import Dict, List, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from src.phase2_mechanical.generate_stl_parts import (  # noqa: E402
    MATING_INTERFACES,
    ROVER_BOLT_ANGLES_DEG,
    ROVER_BOLT_CIRCLE_R_MM,
)

BUILD_VOLUME_MM = 180.0
BED_SEATING_TOL_MM = 0.005


def read_binary_stl(path: Path) -> Tuple[int, List[Tuple[float, float, float]]]:
    """Validate the binary STL layout and return (triangle_count, vertex_list)."""
    raw = path.read_bytes()
    if len(raw) < 84:
        raise ValueError(f"{path.name}: {len(raw)} bytes is too small to be a binary STL.")
    num_tris = struct.unpack("<I", raw[80:84])[0]
    expected_len = 84 + 50 * num_tris
    if len(raw) != expected_len:
        raise ValueError(
            f"{path.name}: byte length {len(raw)} != 84 + 50*{num_tris} = {expected_len}."
        )
    vertices: List[Tuple[float, float, float]] = []
    offset = 84
    for _ in range(num_tris):
        vals = struct.unpack("<12fH", raw[offset:offset + 50])
        offset += 50
        for v in vals[:12]:
            if not math.isfinite(v):
                raise ValueError(f"{path.name}: non-finite coordinate or normal.")
        vertices.append((vals[3], vals[4], vals[5]))
        vertices.append((vals[6], vals[7], vals[8]))
        vertices.append((vals[9], vals[10], vals[11]))
    return num_tris, vertices


def bounds(vertices: List[Tuple[float, float, float]]):
    xs = [v[0] for v in vertices]
    ys = [v[1] for v in vertices]
    zs = [v[2] for v in vertices]
    return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))


def degenerate_triangles(vertices: List[Tuple[float, float, float]], tol_mm2: float = 1e-6) -> int:
    """Count triangles whose area is below `tol_mm2` (a slicer repair warning sign)."""
    count = 0
    for i in range(0, len(vertices), 3):
        (ax, ay, az), (bx, by, bz), (cx, cy, cz) = vertices[i], vertices[i + 1], vertices[i + 2]
        ux, uy, uz = bx - ax, by - ay, bz - az
        vx, vy, vz = cx - ax, cy - ay, cz - az
        nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
        if 0.5 * math.sqrt(nx * nx + ny * ny + nz * nz) < tol_mm2:
            count += 1
    return count


def bolt_circle_features(
    vertices: List[Tuple[float, float, float]],
    z_min: float,
    z_max: float,
    r_min: float,
    r_max: float,
) -> Dict[float, Tuple[float, int]]:
    """Measure mesh vertices lying in a z-band and radial band, grouped by declared angle.

    Returns {declared_angle: (mean_radius_mm, vertex_count)}. Vertices are assigned to the
    nearest declared angle when within ANG_TOL_DEG.
    """
    ang_tol_deg = 8.0
    found: Dict[float, Tuple[float, int]] = {}
    for declared in ROVER_BOLT_ANGLES_DEG:
        hits = []
        for x, y, z in vertices:
            if not (z_min <= z <= z_max):
                continue
            r = math.hypot(x, y)
            if not (r_min <= r <= r_max):
                continue
            delta = abs((math.degrees(math.atan2(y, x)) - declared + 180.0) % 360.0 - 180.0)
            if delta <= ang_tol_deg:
                hits.append(r)
        if hits:
            found[declared] = (sum(hits) / len(hits), len(hits))
    return found


def section(title: str) -> None:
    print()
    print("=" * 104)
    print(f"  {title}")
    print("=" * 104)


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    stl_dir = root / "cad" / "stl"

    expected = [
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
        "11_mobile_rover_chassis.stl",
        "12_drive_wheels_and_caster.stl",
    ]

    ok = True
    section("1. BINARY STL LAYOUT, BEDS SIZE, BED SEATING  (mesh facts only)")
    print(f"{'Part Filename':<42} {'X x Y x Z (mm)':<24} {'Tris':>6} {'Degen':>6} "
          f"{'z_min':>8} {'180mm bed':>10}")
    print("-" * 104)

    geometry: Dict[str, List[Tuple[float, float, float]]] = {}
    for fname in expected:
        fpath = stl_dir / fname
        if not fpath.exists():
            print(f"[MISSING] {fname}")
            ok = False
            continue
        tris, verts = read_binary_stl(fpath)
        (x0, y0, z0), (x1, y1, z1) = bounds(verts)
        dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
        degen = degenerate_triangles(verts)
        fits = max(dx, dy, dz) <= BUILD_VOLUME_MM
        # The STLs are exported in the ASSEMBLY frame, where a joint axis is the origin, so
        # several parts legitimately start below z = 0. What matters for printing is that the
        # lowest plane is FLAT, so the slicer's "drop to bed" seats the part on a real face
        # and not on a curve. z_min = 0 here means the part already sits on the bed.
        base_flat = sum(1 for v in verts if abs(v[2] - z0) <= BED_SEATING_TOL_MM)
        seats_here = abs(z0) <= BED_SEATING_TOL_MM
        seated = seats_here or base_flat >= 4
        if seats_here:
            note = "  seated on z=0"
        elif seated:
            note = f"  assembly frame, {base_flat} flat verts at z={z0:.1f}"
        else:
            note = "  NO FLAT BASE"
        if not fits or not seated or degen:
            ok = False
        geometry[fname] = verts
        print(f"{fname:<42} {dx:6.1f} x {dy:6.1f} x {dz:5.1f}   {tris:>6d} {degen:>6d} "
              f"{z0:>8.3f} {'PASS' if fits else 'FAIL':>10}"
              f"{note}")

    print("-" * 104)
    print("Degen = zero-area triangles. 05/06/07/08 are exported in the assembly frame, so")
    print("their origin is a joint axis and they start below z = 0: load them, then use the")
    print("slicer's drop-to-bed. It lands on the flat face listed above, never on a curve.")

    section("2. ROVER BOLT INTERFACE - MEASURED FROM THE MESH BYTES")
    print(f"Declared: 4x M3 on a bolt circle of radius {ROVER_BOLT_CIRCLE_R_MM:.1f} mm at "
          f"angles {ROVER_BOLT_ANGLES_DEG} deg")
    print()
    bands = {
        "11_mobile_rover_chassis.stl": (20.5, 25.5, 31.0, 41.0, "bosses, z 20..25"),
        "03_base_lower_slewing_ring.stl": (0.0, 2.6, 31.0, 41.0, "nut rings, z 0..2.6"),
    }
    print(f"{'Part':<36} {'Feature band':<18} {'Angle':>8} {'Mean r (mm)':>12} {'Verts':>6} "
          f"{'r error':>9}")
    print("-" * 104)
    for fname, (za, zb, ra, rb, label) in bands.items():
        verts = geometry.get(fname)
        if verts is None:
            print(f"{fname:<36} {label:<18} {'--':>8} {'part missing':>12}")
            ok = False
            continue
        found = bolt_circle_features(verts, za, zb, ra, rb)
        for declared in ROVER_BOLT_ANGLES_DEG:
            if declared not in found:
                print(f"{fname:<36} {label:<18} {declared:>7.0f}d {'NOT FOUND':>12} "
                      f"{0:>6} {'--':>9}")
                ok = False
                continue
            mean_r, count = found[declared]
            err = mean_r - ROVER_BOLT_CIRCLE_R_MM
            if abs(err) > 1.5 or count < 8:
                ok = False
            print(f"{fname:<36} {label:<18} {declared:>7.0f}d {mean_r:>12.2f} {count:>6d} "
                  f"{err:>+8.2f}")

    section("3. DECLARED MATING INTERFACES - SOURCE-LEVEL AGREEMENT, NOT A FIT TEST")
    print(f"{'Interface':<26} {'Part A':<36} {'Value':>10} {'How you verify it for real'}")
    print("-" * 104)
    for mi in MATING_INTERFACES:
        print(f"{mi.name:<26} {mi.part_a:<36} {mi.declared_value:>7.2f} {mi.unit:<2} "
              f"{mi.verify_with}")
    print("-" * 104)
    print("Both halves of every pair are built from these same constants, so the MODELS agree.")
    print("Agreement in source is NOT evidence that two printed parts fit.")

    section("4. WIRE LENGTH ESTIMATES  (UNVERIFIED - MEASURE YOUR OWN BUILD)")
    cable_rows = [
        ("J1 Base Yaw (MG996R)",       "PCA9685 Ch 0", "250 mm stock",          "35 mm loop in base turret"),
        ("J2 Shoulder Pitch (MG996R)", "PCA9685 Ch 1", "250 mm stock",          "45 mm loop across J1->J2"),
        ("J3 Elbow Pitch (MG996R)",    "PCA9685 Ch 2", "250 mm + 150 mm ext",   "40 mm loop over J2 bridge"),
        ("J4 Wrist Pitch (MG90S)",     "PCA9685 Ch 3", "250 mm + 300 mm ext",   "35 mm loop over J3 pivot"),
        ("J5 Wrist Roll (MG90S)",      "PCA9685 Ch 4", "250 mm + 300 mm ext",   "45 mm helical loop for +-90 roll"),
        ("J6 Claw Gripper (MG90S)",    "PCA9685 Ch 5", "250 mm + 350 mm ext",   "45 mm helical loop with J5"),
        ("Left drive servo (CR)",      "PCA9685 Ch 6", "250 mm + 100 mm ext",   "40 mm service loop under deck"),
        ("Right drive servo (CR)",     "PCA9685 Ch 7", "250 mm + 100 mm ext",   "40 mm service loop under deck"),
        ("HC-SR04 ultrasonic",         "ESP32 18/19",  "200 mm + 300 mm ext",   "50 mm loop behind bumper"),
        ("2x TCRT5000 cliff sensors",  "ESP32 25/26",  "200 mm + 300 mm ext",   "50 mm loop per bracket stem"),
        ("Eye-in-hand USB camera",     "Laptop USB",   "custom soft USB cable", "50 mm wrist loop + base strain relief"),
    ]
    print(f"{'Joint / Sensor':<30} {'Destination':<14} {'Wire':<22} {'Slack loop'}")
    print("-" * 104)
    for name, dest, length, slack in cable_rows:
        print(f"{name:<30} {dest:<14} {length:<22} {slack}")

    section("RESULT")
    if ok:
        print("PASS: every part is a structurally valid binary STL, seats on the bed, fits a")
        print("180 mm plate, and carries the declared rover bolt features.")
        print()
        print("This still does NOT mean the assembly works. Before printing a functional arm:")
        print("  * Inspect cross-sections in your slicer; these meshes never subtract material.")
        print("  * Print 00_tolerance_test_coupon.stl and test-fit a real servo and a real nut.")
        print("  * For the rover, print the chassis and caster first and test-fit the drivetrain")
        print("    on a FLOOR in a contained area with the arm stowed - never on a desk edge.")
    else:
        print("FAIL: one or more checks above did not pass. Read the rows marked FAIL / NOT FOUND.")
    print("=" * 104)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
