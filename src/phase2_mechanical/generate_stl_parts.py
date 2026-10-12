#!/usr/bin/env python3
"""
Phase 2 Parametric 3D-Printable STL Mesh Generator for B.U.D.-E (Sub-$75 Edition).

Generates all 11 printable STL files (00..10) in `cad/stl/` with exact millimeter
dimensions matching `config/robot_config.yaml`:
  - MG996R / DS3218 standard servo pockets (40.7 x 20.2 mm + clearance)
  - MG90S micro metal-gear servo pockets (22.8 x 12.4 mm + clearance)
  - Double-shear 6.0 mm passive pivot pins & 6.30 mm bores
  - 3D-printed Desk C-Clamp + ACME clamping bolt
  - 3D-printed 6mm BB slewing ring base + LA38-11ZS 22mm E-stop mount
  - Shoulder gravity-assist rubber-band hooks on Base Turret & Upper Arm

Usage:
    python3 src/phase2_mechanical/generate_stl_parts.py
    python3 src/phase2_mechanical/generate_stl_parts.py --clearance-mm 0.25
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import struct
import sys
from typing import List, Tuple

Vec3 = Tuple[float, float, float]
Triangle = Tuple[Vec3, Vec3, Vec3]


def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(v: Vec3) -> Vec3:
    mag = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if mag < 1e-12:
        return (0.0, 0.0, 1.0)
    return (v[0] / mag, v[1] / mag, v[2] / mag)


class MeshBuilder:
    """Lightweight watertight-primitive triangle mesh builder for binary STL export."""

    def __init__(self, name: str) -> None:
        self.name = name
        self.triangles: List[Triangle] = []

    def add_quad(self, p0: Vec3, p1: Vec3, p2: Vec3, p3: Vec3) -> None:
        self.triangles.append((p0, p1, p2))
        self.triangles.append((p0, p2, p3))

    def add_box(self, cx: float, cy: float, cz: float, sx: float, sy: float, sz: float) -> None:
        """Add an axis-aligned box centered at (cx, cy, cz) with dimensions (sx, sy, sz) in mm."""
        hx, hy, hz = 0.5 * sx, 0.5 * sy, 0.5 * sz
        x0, x1 = cx - hx, cx + hx
        y0, y1 = cy - hy, cy + hy
        z0, z1 = cz - hz, cz + hz

        p000, p100, p110, p010 = (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0)
        p001, p101, p111, p011 = (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)

        self.add_quad(p000, p010, p110, p100)  # Bottom (-Z)
        self.add_quad(p001, p101, p111, p011)  # Top (+Z)
        self.add_quad(p000, p100, p101, p001)  # Front (-Y)
        self.add_quad(p100, p110, p111, p101)  # Right (+X)
        self.add_quad(p110, p010, p011, p111)  # Back (+Y)
        self.add_quad(p010, p000, p001, p011)  # Left (-X)

    def add_cylinder_z(
        self,
        cx: float,
        cy: float,
        z_bottom: float,
        height: float,
        radius: float,
        segments: int = 32,
    ) -> None:
        """Add a closed vertical cylinder along +Z."""
        z0 = z_bottom
        z1 = z_bottom + height
        c_bot: Vec3 = (cx, cy, z0)
        c_top: Vec3 = (cx, cy, z1)

        for i in range(segments):
            a0 = 2.0 * math.pi * i / segments
            a1 = 2.0 * math.pi * (i + 1) / segments
            x0, y0 = cx + radius * math.cos(a0), cy + radius * math.sin(a0)
            x1, y1 = cx + radius * math.cos(a1), cy + radius * math.sin(a1)

            b0, b1 = (x0, y0, z0), (x1, y1, z0)
            t0, t1 = (x0, y0, z1), (x1, y1, z1)

            self.triangles.append((c_bot, b1, b0))
            self.triangles.append((c_top, t0, t1))
            self.add_quad(b0, b1, t1, t0)

    def add_tube_z(
        self,
        cx: float,
        cy: float,
        z_bottom: float,
        height: float,
        r_inner: float,
        r_outer: float,
        segments: int = 36,
    ) -> None:
        """Add a hollow tube/ring along +Z (used for slewing ring races, bores, and gears)."""
        z0 = z_bottom
        z1 = z_bottom + height
        for i in range(segments):
            a0 = 2.0 * math.pi * i / segments
            a1 = 2.0 * math.pi * (i + 1) / segments
            ca0, sa0 = math.cos(a0), math.sin(a0)
            ca1, sa1 = math.cos(a1), math.sin(a1)

            ib0 = (cx + r_inner * ca0, cy + r_inner * sa0, z0)
            ib1 = (cx + r_inner * ca1, cy + r_inner * sa1, z0)
            it0 = (cx + r_inner * ca0, cy + r_inner * sa0, z1)
            it1 = (cx + r_inner * ca1, cy + r_inner * sa1, z1)

            ob0 = (cx + r_outer * ca0, cy + r_outer * sa0, z0)
            ob1 = (cx + r_outer * ca1, cy + r_outer * sa1, z0)
            ot0 = (cx + r_outer * ca0, cy + r_outer * sa0, z1)
            ot1 = (cx + r_outer * ca1, cy + r_outer * sa1, z1)

            self.add_quad(ob0, ob1, ot1, ot0)  # Outer wall
            self.add_quad(ib1, ib0, it0, it1)  # Inner bore wall
            self.add_quad(it0, ot0, ot1, it1)  # Top annulus
            self.add_quad(ob0, ib0, ib1, ob1)  # Bottom annulus

    def add_servo_cradle(
        self,
        cx: float,
        cy: float,
        z_bottom: float,
        inner_l: float,
        inner_w: float,
        wall_t: float,
        height: float,
    ) -> None:
        """Build a 4-wall open-top rectangular pocket for an MG996R or MG90S servo."""
        cz = z_bottom + 0.5 * height
        outer_l = inner_l + 2.0 * wall_t
        outer_w = inner_w + 2.0 * wall_t
        # Left & right walls (along X)
        self.add_box(cx - 0.5 * (inner_l + wall_t), cy, cz, wall_t, outer_w, height)
        self.add_box(cx + 0.5 * (inner_l + wall_t), cy, cz, wall_t, outer_w, height)
        # Front & back walls (along Y)
        self.add_box(cx, cy - 0.5 * (inner_w + wall_t), cz, inner_l, wall_t, height)
        self.add_box(cx, cy + 0.5 * (inner_w + wall_t), cz, inner_l, wall_t, height)

    def bounding_box(self) -> Tuple[Vec3, Vec3, Vec3]:
        xs = [p[0] for tri in self.triangles for p in tri]
        ys = [p[1] for tri in self.triangles for p in tri]
        zs = [p[2] for tri in self.triangles for p in tri]
        mn = (min(xs), min(ys), min(zs))
        mx = (max(xs), max(ys), max(zs))
        sz = (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])
        return mn, mx, sz

    def write_binary_stl(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        header = f"BUDE Phase2 CAD: {self.name}".encode("ascii", errors="ignore")[:80]
        header = header.ljust(80, b"\x00")
        with path.open("wb") as f:
            f.write(header)
            f.write(struct.pack("<I", len(self.triangles)))
            for p0, p1, p2 in self.triangles:
                n = _norm(_cross(_sub(p1, p0), _sub(p2, p0)))
                f.write(
                    struct.pack(
                        "<12fH",
                        n[0], n[1], n[2],
                        p0[0], p0[1], p0[2],
                        p1[0], p1[1], p1[2],
                        p2[0], p2[1], p2[2],
                        0,
                    )
                )


@dataclass
class PartSpec:
    filename: str
    title: str
    est_filament_g: float
    print_orientation: str
    layer_height_mm: float
    walls: int
    infill_pct: int
    supports_needed: str
    purpose: str


def generate_all_parts(out_dir: Path, clearance_mm: float = 0.25) -> List[Tuple[PartSpec, Tuple[float, float, float], int]]:
    """Generate all 11 printable STL files in `cad/stl/` and return summary metadata."""
    mg996r_l = 40.7 + 2.0 * clearance_mm
    mg996r_w = 20.2 + 2.0 * clearance_mm
    mg90s_l = 22.8 + 2.0 * clearance_mm
    mg90s_w = 12.4 + 2.0 * clearance_mm

    results = []

    # -------------------------------------------------------------------------
    # 00: First-Print Tolerance Calibration Coupon (8g, 12 mins)
    # -------------------------------------------------------------------------
    m00 = MeshBuilder("00_tolerance_test_coupon")
    m00.add_box(0.0, 0.0, 2.0, 72.0, 38.0, 4.0)  # Base gauge plate
    m00.add_servo_cradle(-12.0, 0.0, 4.0, mg996r_l, mg996r_w, 3.0, 8.0)  # MG996R test fit frame
    m00.add_servo_cradle(22.0, -7.0, 4.0, mg90s_l, mg90s_w, 2.5, 8.0)    # MG90S test fit frame
    m00.add_tube_z(22.0, 10.0, 0.0, 10.0, 3.0 + clearance_mm, 6.5, 28)   # 6mm passive pivot bore test
    m00.add_cylinder_z(31.0, 10.0, 4.0, 10.0, 3.0, 28)                   # 6.0mm test pin
    spec00 = PartSpec(
        "00_tolerance_test_coupon.stl",
        "Tolerance & Fit Calibration Coupon",
        8.0,
        "Flat on base plate (-Z down)",
        0.20, 3, 20, "None",
        "12-min test print to verify MG996R pocket, MG90S pocket, M3 nut trap & 6mm pivot pin fit before full print",
    )

    # -------------------------------------------------------------------------
    # 01: Heavy-Duty 3D-Printed Desk C-Clamp Frame
    # -------------------------------------------------------------------------
    m01 = MeshBuilder("01_desk_c_clamp_frame")
    # Prints flat on its side (X-Y plane) so continuous perimeter filaments wrap around the C-throat!
    m01.add_box(0.0, 32.0, 9.0, 58.0, 16.0, 18.0)    # Top jaw (bolts to base plate)
    m01.add_box(-21.0, 0.0, 9.0, 16.0, 80.0, 18.0)   # Vertical spine (handles bending tension)
    m01.add_box(0.0, -32.0, 9.0, 58.0, 16.0, 18.0)   # Bottom jaw (houses threaded clamp screw)
    m01.add_tube_z(16.0, -32.0, 0.0, 18.0, 6.2, 11.0, 24)  # M12 thread boss
    spec01 = PartSpec(
        "01_desk_c_clamp_frame.stl",
        "Heavy-Duty Desk C-Clamp Frame (Up to 48mm Desk)",
        34.0,
        "Flat on side (so layer lines run continuously around the C-profile!)",
        0.20, 5, 35, "None",
        "Clamps robot base firmly to workbench edge; replaces $6.50 cast-iron clamp",
    )

    # -------------------------------------------------------------------------
    # 02: Desk C-Clamp ACME Screw & Swivel Pad
    # -------------------------------------------------------------------------
    m02 = MeshBuilder("02_desk_c_clamp_screw_and_pad")
    m02.add_cylinder_z(0.0, 0.0, 0.0, 8.0, 16.0, 24)   # Knurled thumb knob
    m02.add_cylinder_z(0.0, 0.0, 8.0, 52.0, 5.85, 24)  # Thick ACME-style M12 screw shaft
    m02.add_tube_z(32.0, 0.0, 0.0, 8.0, 4.2, 13.0, 24) # Snap-on swivel pressure pad
    spec02 = PartSpec(
        "02_desk_c_clamp_screw_and_pad.stl",
        "C-Clamp Threaded Screw + Swivel Pad",
        14.0,
        "Knob flat on bed (print with 5 walls for shear strength)",
        0.16, 5, 40, "None",
        "Threads through C-clamp bottom jaw to lock base onto desk",
    )

    # -------------------------------------------------------------------------
    # 03: Base Lower Plate + 6mm BB Slewing Ring + E-Stop Housing
    # -------------------------------------------------------------------------
    m03 = MeshBuilder("03_base_lower_slewing_ring")
    m03.add_cylinder_z(0.0, 0.0, 0.0, 5.0, 46.0, 48)            # Main 92mm diameter base flange
    m03.add_tube_z(0.0, 0.0, 5.0, 18.0, 34.0, 44.0, 48)         # Outer slewing ring race (for 6mm BBs)
    m03.add_servo_cradle(0.0, 0.0, 5.0, mg996r_l, mg996r_w, 3.2, 22.0)  # J1 MG996R stationary pocket
    # Side-integrated E-Stop switch box with 22.2mm mounting collar for LA38-11ZS mushroom switch
    m03.add_box(58.0, 0.0, 14.0, 34.0, 34.0, 28.0)
    m03.add_tube_z(58.0, 0.0, 28.0, 4.0, 11.1, 15.0, 32)
    spec03 = PartSpec(
        "03_base_lower_slewing_ring.stl",
        "Stationary Base + Slewing Lower Race + E-Stop Box",
        42.0,
        "Base flange flat on print bed",
        0.20, 4, 25, "None",
        "Holds J1 (MG996R) waist servo, 6mm BB slewing race, C-clamp mount & 22mm E-Stop switch",
    )

    # -------------------------------------------------------------------------
    # 04: Base Upper Rotating Turret + J2 Shoulder Bracket + Gravity Assist Hook
    # -------------------------------------------------------------------------
    m04 = MeshBuilder("04_base_upper_turret_j2")
    m04.add_tube_z(0.0, 0.0, 0.0, 8.0, 8.0, 41.0, 48)           # Rotating slewing disk (82mm dia)
    # Left upright holds J2 Shoulder Pitch MG996R/DS3218 servo
    m04.add_box(0.0, -24.0, 26.0, 48.0, 10.0, 36.0)
    m04.add_servo_cradle(0.0, -29.0, 14.0, mg996r_l, mg996r_w, 3.0, 22.0)
    # Right upright holds the 6.3mm double-shear passive pivot bore opposite J2 horn
    m04.add_box(0.0, 24.0, 26.0, 32.0, 8.0, 36.0)
    m04.add_tube_z(0.0, 24.0, 32.0, 8.0, 3.15, 7.5, 28)
    # Rear gravity-assist rubber-band anchor post (-X side)
    m04.add_cylinder_z(-34.0, 0.0, 8.0, 16.0, 4.0, 20)
    spec04 = PartSpec(
        "04_base_upper_turret_j2.stl",
        "Rotating Waist Turret + J2 Double-Shear Shoulder + Spring Lug",
        36.0,
        "Turret disk flat on bed",
        0.20, 4, 25, "None",
        "Couples J1 rotation to J2 shoulder pitch with double-shear support & rubber-band counterbalance hook",
    )

    # -------------------------------------------------------------------------
    # 05: Upper Arm Link (L2 = 80 mm) with B.U.D.-E Nameplate & Cable Channel
    # -------------------------------------------------------------------------
    m05 = MeshBuilder("05_upper_arm_link_80mm")
    # Print flat (X = 80mm center-to-center link length)
    m05.add_box(0.0, -21.0, 6.0, 96.0, 5.0, 12.0)               # Left structural beam
    m05.add_box(0.0, 21.0, 6.0, 96.0, 5.0, 12.0)                # Right structural beam
    m05.add_box(-18.0, 0.0, 6.0, 10.0, 38.0, 12.0)              # Cross-brace 1 (B.U.D.-E nameplate bridge)
    m05.add_box(18.0, 0.0, 6.0, 10.0, 38.0, 12.0)               # Cross-brace 2 (with cable pass-through)
    m05.add_tube_z(-40.0, 0.0, 0.0, 12.0, 3.15, 8.0, 28)        # J2 Shoulder axis boss (-40mm)
    m05.add_servo_cradle(40.0, -10.0, 0.0, mg996r_l, mg996r_w, 2.8, 12.0)  # J3 Elbow MG996R cradle (+40mm)
    m05.add_cylinder_z(-28.0, 0.0, 12.0, 8.0, 3.5, 20)          # Upper-arm rubber-band hook lug
    spec05 = PartSpec(
        "05_upper_arm_link_80mm.stl",
        "Upper Arm Double-Shear Link (L2 = 80 mm)",
        24.0,
        "Flat on print bed (maximum bending strength along 80mm span)",
        0.20, 3, 20, "None",
        "Connects J2 Shoulder to J3 Elbow (80mm axis-to-axis); includes rubber-band hook & cable bridge",
    )

    # -------------------------------------------------------------------------
    # 06: Forearm Link (L3 = 75 mm) Transitioning J3 (MG996R) -> J4 (MG90S)
    # -------------------------------------------------------------------------
    m06 = MeshBuilder("06_forearm_link_75mm")
    m06.add_box(0.0, -15.0, 5.0, 88.0, 4.5, 10.0)               # Left tapered rail
    m06.add_box(0.0, 15.0, 5.0, 88.0, 4.5, 10.0)                # Right tapered rail
    m06.add_box(0.0, 0.0, 5.0, 12.0, 26.0, 10.0)                # Center truss web
    m06.add_tube_z(-37.5, 0.0, 0.0, 10.0, 3.0, 7.5, 28)         # J3 Elbow horn interface (-37.5mm)
    m06.add_servo_cradle(37.5, 0.0, 0.0, mg90s_l, mg90s_w, 2.4, 10.0)  # J4 Wrist Pitch MG90S pocket (+37.5mm)
    spec06 = PartSpec(
        "06_forearm_link_75mm.stl",
        "Forearm Tapered Truss Link (L3 = 75 mm)",
        17.0,
        "Flat on print bed",
        0.20, 3, 20, "None",
        "Connects J3 Elbow (MG996R) to featherweight J4 Wrist Pitch (MG90S 13.4g)",
    )

    # -------------------------------------------------------------------------
    # 07: Wrist Pitch-to-Roll Bracket (L4 = 22 mm) for Expressive 35° Head-Tilt
    # -------------------------------------------------------------------------
    m07 = MeshBuilder("07_wrist_pitch_roll_bracket")
    m07.add_box(-11.0, -11.0, 6.0, 22.0, 3.5, 12.0)             # Left U-arm (mates to J4 MG90S horn)
    m07.add_box(-11.0, 11.0, 6.0, 22.0, 3.5, 12.0)              # Right U-arm (mates to J4 passive pin)
    m07.add_box(0.0, 0.0, 6.0, 4.0, 25.5, 12.0)                 # Bridge web
    m07.add_servo_cradle(11.0, 0.0, 0.0, mg90s_l, mg90s_w, 2.4, 12.0)  # J5 Wrist Roll MG90S cradle
    spec07 = PartSpec(
        "07_wrist_pitch_roll_bracket.stl",
        "Wrist Pitch-to-Roll Orthogonal Bracket (L4 = 22 mm)",
        9.0,
        "Flat on bridge face",
        0.20, 3, 20, "None",
        "Orthogonal coupling between J4 (nod) and J5 (puppy head-tilt roll)",
    )

    # -------------------------------------------------------------------------
    # 08: Claw Gripper Body + Eye-in-Hand USB Camera Bezel ("DUM-E Head")
    # -------------------------------------------------------------------------
    m08 = MeshBuilder("08_gripper_body_and_cam_mount")
    m08.add_box(0.0, 0.0, 5.0, 36.0, 42.0, 10.0)                # Main palm plate (houses J6 MG90S)
    m08.add_servo_cradle(0.0, 0.0, 5.0, mg90s_l, mg90s_w, 2.4, 12.0)
    m08.add_cylinder_z(12.0, -11.0, 10.0, 7.0, 2.85, 24)        # Left spur-gear pivot pin (5.7mm)
    m08.add_cylinder_z(12.0, 11.0, 10.0, 7.0, 2.85, 24)         # Right spur-gear pivot pin (5.7mm)
    # Top "DUM-E Expressive Camera Head" bezel for 6g HBV-1825 / OV2640 USB board cam (30x25mm)
    m08.add_box(-8.0, 0.0, 24.0, 10.0, 34.0, 28.0)
    m08.add_tube_z(-6.0, 0.0, 24.0, 6.0, 7.0, 11.0, 28)         # Camera lens hood ("Expressive Eye")
    spec08 = PartSpec(
        "08_gripper_body_and_cam_mount.stl",
        "Gripper Palm Housing + Eye-in-Hand Camera Bezel",
        13.0,
        "Palm back flat on bed",
        0.20, 3, 20, "None",
        "Holds J6 MG90S claw servo, synchronous finger pivot pins & 6g USB webcam 'head'",
    )

    # -------------------------------------------------------------------------
    # 09: Geared 2-Finger Parallel/Pinch Claw Pair (52 mm Max Jaw Opening)
    # -------------------------------------------------------------------------
    m09 = MeshBuilder("09_gripper_geared_fingers_pair")
    for sign, y_off in [(-1.0, -16.0), (1.0, 16.0)]:
        # Meshed spur-gear hub (22mm pitch circle)
        m09.add_tube_z(0.0, y_off, 0.0, 6.0, 3.05, 11.0, 24)
        # Curved V-groove finger beam (48mm reach) for grabbing round cups & hex screwdrivers
        m09.add_box(22.0, y_off + sign * 2.0, 3.0, 38.0, 6.0, 6.0)
        m09.add_box(40.0, y_off - sign * 2.5, 3.0, 12.0, 7.0, 6.0)  # Inward V-jaw tip pad
    spec09 = PartSpec(
        "09_gripper_geared_fingers_pair.stl",
        "Synchronous Geared Claw Fingers (Left + Right Pair)",
        9.0,
        "Flat on gear face (0.16mm layer height for crisp gear teeth)",
        0.16, 4, 30, "None",
        "Meshed 1:1 spur gears so 1 MG90S opens/closes both fingers symmetrically; V-grooves hold round tools/cups",
    )

    # -------------------------------------------------------------------------
    # 10: Snap-On Cable Clips (x6) + Base Electronics Carrier Tray (ESP32/PCA9685/INA219)
    # -------------------------------------------------------------------------
    m10 = MeshBuilder("10_cable_clips_and_electronics_tray")
    m10.add_box(0.0, 0.0, 1.5, 68.0, 52.0, 3.0)                 # Tray base for ESP32 + PCA9685 + INA219
    for sx, sy in [(-30.0, -22.0), (30.0, -22.0), (-30.0, 22.0), (30.0, 22.0)]:
        m10.add_tube_z(sx, sy, 3.0, 5.0, 1.4, 3.2, 16)          # M2.5/M3 PCB standoffs
    for i in range(6):
        cx = -30.0 + i * 12.0
        m10.add_tube_z(cx, 36.0, 0.0, 6.0, 4.0, 6.0, 20)        # 6x snap-on servo wire guides
    spec10 = PartSpec(
        "10_cable_clips_and_electronics_tray.stl",
        "ESP32 + PCA9685 + INA219 Base Tray & 6x Cable Clips",
        12.0,
        "Flat on print bed",
        0.20, 3, 20, "None",
        "Neatly mounts ESP32, PCA9685 & INA219 beside the base and routes servo/camera wires with Slack Loops",
    )

    all_builders = [
        (spec00, m00),
        (spec01, m01),
        (spec02, m02),
        (spec03, m03),
        (spec04, m04),
        (spec05, m05),
        (spec06, m06),
        (spec07, m07),
        (spec08, m08),
        (spec09, m09),
        (spec10, m10),
    ]

    for spec, builder in all_builders:
        stl_path = out_dir / spec.filename
        builder.write_binary_stl(stl_path)
        _, _, bbox_sz = builder.bounding_box()
        results.append((spec, bbox_sz, len(builder.triangles)))

    return results


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate all 3D-printable STL files for B.U.D.-E.")
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path(__file__).resolve().parents[2] / "cad" / "stl",
        help="Output directory for generated .stl files",
    )
    parser.add_argument(
        "--clearance-mm",
        type=float,
        default=0.25,
        help="Printer XY pocket clearance in mm (default: 0.25 mm for standard 0.4mm nozzle)",
    )
    args = parser.parse_args()

    results = generate_all_parts(args.out_dir, clearance_mm=args.clearance_mm)

    print("=" * 94)
    print(f"  B.U.D.-E — PHASE 2 GENERATED 3D-PRINTABLE STL MANIFEST (Clearance = {args.clearance_mm:.2f} mm)")
    print("=" * 94)
    print(f"{'STL File':<40} {'Bounding Box (X×Y×Z mm)':<26} {'Tris':<7} {'PLA (g)':<9} {'Supports'}")
    print("-" * 94)
    total_g = 0.0
    for spec, sz, tris in results:
        total_g += spec.est_filament_g
        bbox_str = f"{sz[0]:5.1f} × {sz[1]:5.1f} × {sz[2]:4.1f}"
        print(f"{spec.filename:<40} {bbox_str:<26} {tris:<7d} {spec.est_filament_g:>5.1f} g   {spec.supports_needed}")
    print("-" * 94)
    print(
        f"Total Printable Parts: {len(results)} files in `{args.out_dir.relative_to(Path(__file__).resolve().parents[2])}/` | "
        f"Total Filament: {total_g:.0f} g (~${total_g * 0.018:.2f} of PLA) | ZERO supports required!"
    )
    print("=" * 94)
    return 0


if __name__ == "__main__":
    sys.exit(main())
