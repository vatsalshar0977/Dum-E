#!/usr/bin/env python3
"""
WARNING: EXPERIMENTAL MESH DEMO ONLY. Does not perform Boolean subtraction or
verify mating; generated STLs are NOT ready for functional robot assembly.
See docs/verification_status.md before printing anything.

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

    def add_wall_cradle(
        self,
        cx: float,
        cy: float,
        cz: float,
        ex: float,
        ey: float,
        ez: float,
        wall_t: float,
        omit: Tuple[str, ...] = (),
    ) -> None:
        """Add up to six walls around an interior void of size (ex, ey, ez) centred at (cx, cy, cz).

        `omit` names faces to leave open, e.g. ('+Z',) for an open-top pocket or
        ('+Y', '+Z') for a pocket you slide a part into from +Y with a deck closing the top.
        Wall extents run the *full* interior size on the two axes that are not omitted,
        so an open box still has a clean rim.

        NOTE: like every primitive in this file this ADDS material. It never subtracts a
        cavity from an intersecting box, so several "pockets" here are pockets only by
        virtue of their surrounding walls. Inspect cross-sections in a slicer.
        """
        hx, hy, hz = 0.5 * ex, 0.5 * ey, 0.5 * ez
        walls = {
            "+X": ((cx + hx + 0.5 * wall_t, cy, cz), (wall_t, ey + 2 * wall_t, ez)),
            "-X": ((cx - hx - 0.5 * wall_t, cy, cz), (wall_t, ey + 2 * wall_t, ez)),
            "+Y": ((cx, cy + hy + 0.5 * wall_t, cz), (ex, wall_t, ez)),
            "-Y": ((cx, cy - hy - 0.5 * wall_t, cz), (ex, wall_t, ez)),
            "+Z": ((cx, cy, cz + hz + 0.5 * wall_t), (ex, ey, wall_t)),
            "-Z": ((cx, cy, cz - hz - 0.5 * wall_t), (ex, ey, wall_t)),
        }
        for face, (centre, size) in walls.items():
            if face in omit:
                continue
            self.add_box(centre[0], centre[1], centre[2], size[0], size[1], size[2])

    def add_hex_tube_z(
        self,
        cx: float,
        cy: float,
        z_bottom: float,
        height: float,
        across_flats_inner: float,
        wall_t: float,
    ) -> None:
        """Add a closed hexagonal ring along +Z, used to MARK an M3 hex-nut trap position.

        A hex nut is 5.5 mm across flats (a cheap-clone M3 nut may measure 5.4-5.8 mm, so
        measure yours). This ring draws the trap outline; the trap itself must be a Boolean
        cut in real CAD, which this generator cannot perform.
        """
        r_i = across_flats_inner / math.sqrt(3.0)
        r_o = r_i + wall_t
        z0, z1 = z_bottom, z_bottom + height
        for i in range(6):
            a0 = math.pi / 3.0 * i + math.pi / 6.0
            a1 = math.pi / 3.0 * (i + 1) + math.pi / 6.0
            ib0 = (cx + r_i * math.cos(a0), cy + r_i * math.sin(a0), z0)
            ib1 = (cx + r_i * math.cos(a1), cy + r_i * math.sin(a1), z0)
            it0 = (cx + r_i * math.cos(a0), cy + r_i * math.sin(a0), z1)
            it1 = (cx + r_i * math.cos(a1), cy + r_i * math.sin(a1), z1)
            ob0 = (cx + r_o * math.cos(a0), cy + r_o * math.sin(a0), z0)
            ob1 = (cx + r_o * math.cos(a1), cy + r_o * math.sin(a1), z0)
            ot0 = (cx + r_o * math.cos(a0), cy + r_o * math.sin(a0), z1)
            ot1 = (cx + r_o * math.cos(a1), cy + r_o * math.sin(a1), z1)
            self.add_quad(ob0, ob1, ot1, ot0)
            self.add_quad(ib1, ib0, it0, it1)
            self.add_quad(it0, ot0, ot1, it1)
            self.add_quad(ob0, ib0, ib1, ob1)

    def mesh_volume_mm3(self) -> float:
        """Signed volume of the triangle soup by the divergence theorem.

        Every primitive is closed and consistently wound, so the sum is close to the
        intended solid volume. Where added primitives OVERLAP each other the overlap is
        counted more than once, so treat this as an UPPER BOUND on printed material.
        Your slicer's filament estimate is the authoritative number.
        """
        total = 0.0
        for a, b, c in self.triangles:
            total += (
                a[0] * (b[1] * c[2] - b[2] * c[1])
                - a[1] * (b[0] * c[2] - b[2] * c[0])
                + a[2] * (b[0] * c[1] - b[1] * c[0])
            )
        return abs(total) / 6.0

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


# =============================================================================
# DECLARED MATING INTERFACES
# =============================================================================
# Every number below is a *source-level declaration*: the generator uses it to build
# both halves of a mating pair, and verify_print_parts.py checks the two halves agree.
# Agreement in source is NOT evidence that two printed parts fit. Fit requires a
# first-article print plus the real component, a caliper, and a test fit with power off.
#
# CORRECTION TO AN EARLIER CLAIM: the rover was previously described as bolting to the
# base at (-52.0, +-8.0) mm "using the exact same 2x M3 holes" as the C-clamp. That is
# impossible against the geometry in this file: part 03's flange is a 92 mm disc
# (r = 46 mm), so (-52, +-8) lies outside the printed material (r = 52.6 mm). It also
# collides with the E-stop housing, which spans x = +41..+75 mm. The rover interface is
# therefore declared as a 4-bolt circle at r = 36 mm on diagonals, which sits inside the
# flange and clear of both the J1 servo cradle and the E-stop box.
BASE_FLANGE_DIAMETER_MM = 92.0
BASE_FLANGE_RADIUS_MM = BASE_FLANGE_DIAMETER_MM / 2.0
ROVER_BOLT_CIRCLE_R_MM = 36.0
ROVER_BOLT_ANGLES_DEG = (45.0, 135.0, 225.0, 315.0)
M3_CLEARANCE_HOLE_D_MM = 3.4          # free fit for M3
M3_NUT_ACROSS_FLATS_MM = 5.5          # nominal; MEASURE your bag of nuts
E_STOP_BOX_MIN_MM = (41.0, -17.0, 0.0)
E_STOP_BOX_MAX_MM = (75.0, 17.0, 32.0)

# Rover drivetrain declarations. Every one of these must be replaced with a caliper
# reading from the servo and wheel you actually bought before you trust the fit.
WHEEL_DIAMETER_MM = 48.0
WHEEL_THICKNESS_MM = 5.0
GROUND_GAP_MM = 8.0                   # chassis underside above the floor, assembled
AXLE_HEIGHT_MM = WHEEL_DIAMETER_MM / 2.0 - GROUND_GAP_MM   # in part-11 coordinates
DT_SERVO_L_MM, DT_SERVO_W_MM, DT_SERVO_H_MM = 22.8, 12.2, 22.5
DT_SERVO_SHAFT_OFFSET_MM = 5.8        # shaft centre from body centre along the body length
CASTER_BALL_D_MM = 12.0
CASTER_CUP_BORE_MM = 6.4


@dataclass(frozen=True)
class MatingInterface:
    """One declared joint between two printed parts, or between a part and a component."""

    name: str
    part_a: str
    part_b: str
    parameter: str
    declared_value: float
    unit: str
    verify_with: str


MATING_INTERFACES: List[MatingInterface] = [
    MatingInterface("Rover base bolt circle", "11_mobile_rover_chassis.stl",
                    "03_base_lower_slewing_ring.stl", "bolt circle radius",
                    ROVER_BOLT_CIRCLE_R_MM, "mm", "calipers across opposite bolts, assembled"),
    MatingInterface("Rover base bolt pattern", "11_mobile_rover_chassis.stl",
                    "03_base_lower_slewing_ring.stl", "bolt angles (4x)",
                    float(len(ROVER_BOLT_ANGLES_DEG)), "count", "visual + 90 deg spacing check"),
    MatingInterface("Rover bolt clearance hole", "11_mobile_rover_chassis.stl",
                    "M3 SHCS", "through hole diameter",
                    M3_CLEARANCE_HOLE_D_MM, "mm", "M3 screw must drop through, no reaming"),
    MatingInterface("Rover nut trap", "03_base_lower_slewing_ring.stl",
                    "M3 hex nut", "across flats",
                    M3_NUT_ACROSS_FLATS_MM, "mm", "press your own nut in; measure first"),
    MatingInterface("Drive servo pocket", "11_mobile_rover_chassis.stl",
                    "continuous-rotation micro servo", "pocket length/width/height",
                    DT_SERVO_L_MM, "mm", "slide the real servo in with power off"),
    MatingInterface("Wheel to servo spline", "12_drive_wheels_and_caster.stl",
                    "servo horn", "hub bore", 4.9, "mm",
                    "MEASURE your horn spline; 20T micro horns vary by brand"),
    MatingInterface("Caster ball", "12_drive_wheels_and_caster.stl",
                    f"steel or glass ball {CASTER_BALL_D_MM:g} mm", "cup bore",
                    CASTER_CUP_BORE_MM, "mm", "ball must spin freely and not fall through"),
    MatingInterface("Axle height", "12_drive_wheels_and_caster.stl",
                    "11_mobile_rover_chassis.stl", "axle above chassis underside",
                    AXLE_HEIGHT_MM, "mm", "ruler from desk to axle centre, assembled"),
]


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
    # Rover interface: four M3 hex-nut traps in the UNDERSIDE of the 5 mm base flange.
    # At r = 36 mm these land in solid flange material, clear of the J1 cradle (out to
    # x = +-23.8 mm) and clear of the E-stop box (x = +41..+75 mm). A trap is 2.6 mm
    # deep, leaving ~2.4 mm of flange above it so the screw cannot break into the BB race
    # that starts at z = 5 mm. These hex rings MARK the trap positions; cut the actual
    # hex cavity in real CAD.
    for ang in ROVER_BOLT_ANGLES_DEG:
        _a = math.radians(ang)
        m03.add_hex_tube_z(
            ROVER_BOLT_CIRCLE_R_MM * math.cos(_a),
            ROVER_BOLT_CIRCLE_R_MM * math.sin(_a),
            0.0, 2.6, M3_NUT_ACROSS_FLATS_MM, 1.4,
        )
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

    # -------------------------------------------------------------------------
    # 11: Mobile Rover Chassis (2WD + caster + ultrasonic bumper + cliff sensor slots)
    # -------------------------------------------------------------------------
    # GEOMETRY NOTE ON WHEEL SIZE: the brief asked for 64 mm wheels inside a 25 mm tall
    # chassis. That does not close. A standard micro servo is 12.2 mm across its body
    # width and its output shaft is centred on that width, so mounting it shaft-outboard
    # with the axle at wheel radius height puts the servo body centre at z = 24 mm and its
    # top face at z = 30.1 mm before you add a single wall. A 25 mm chassis is consistent
    # with roughly 48 mm wheels and an 8 mm ground gap (axle at 16 mm), which is what is
    # built here. 64 mm wheels need a ~41 mm chassis, a lower ground gap, or N20 gearmotors.
    m11 = MeshBuilder("11_mobile_rover_chassis")
    m11.add_box(0.0, 0.0, 22.5, 146.0, 104.0, 5.0)                     # Main deck plate
    for side in (-1.0, 1.0):
        m11.add_box(0.0, side * 55.0, 12.5, 44.0, 6.0, 25.0)           # Left/right side plates
        # Drive-servo pocket: servo lies on its side, shaft exits outboard through the
        # side plate. Open on the outboard face (slip-in) and open on +Z (deck closes it).
        m11.add_wall_cradle(
            -DT_SERVO_SHAFT_OFFSET_MM, side * 40.0, AXLE_HEIGHT_MM,
            DT_SERVO_L_MM + 0.25, DT_SERVO_H_MM + 0.25, DT_SERVO_W_MM + 0.25,
            2.5, omit=("+Y" if side > 0 else "-Y", "+Z"),
        )
        m11.add_box(70.0, side * 33.0, 11.0, 6.0, 26.0, 22.0)          # Ultrasonic bumper bar
        # Downward TCRT5000 desk-edge brackets: a stem drops from the deck underside to a
        # slot that holds the module ~5 mm above the chassis bottom plane. Open top and
        # bottom so the module slides through and its emitters look straight down.
        m11.add_box(53.0, side * 25.0, 14.0, 4.0, 18.5, 12.0)          # Bracket stem
        m11.add_wall_cradle(53.0, side * 25.0, 6.5, 32.5, 14.5, 3.0, 2.0, omit=("-Z", "+Z"))
    m11.add_wall_cradle(50.0, 0.0, 11.0, 45.5, 20.5, 15.5, 2.5, omit=("+X",))  # HC-SR04 cradle
    for ang in ROVER_BOLT_ANGLES_DEG:                                  # Base mounting bosses
        _a = math.radians(ang)
        m11.add_tube_z(
            ROVER_BOLT_CIRCLE_R_MM * math.cos(_a),
            ROVER_BOLT_CIRCLE_R_MM * math.sin(_a),
            20.0, 5.0, 0.5 * M3_CLEARANCE_HOLE_D_MM, 3.2, 24,
        )
    for sy in (-20.0, 20.0):                                           # Rear caster bosses
        m11.add_tube_z(-64.0, sy, 0.0, 4.0, 0.5 * M3_CLEARANCE_HOLE_D_MM, 3.0, 20)
    spec11 = PartSpec(
        "11_mobile_rover_chassis.stl",
        "Modular 2WD Rover Chassis (deck + drive pockets + ultrasonic bumper + cliff slots)",
        38.0,
        "Deck plate down onto the bed, side plates vertical",
        0.20, 4, 25, "None",
        "Bolts under the J1 base flange on a 4x M3 circle at r=36mm; carries 2 continuous-rotation drive servos, front HC-SR04 and 2 downward TCRT5000 brackets",
    )

    # -------------------------------------------------------------------------
    # 12: Drive Wheels (x2) + Rear Caster Cup
    # -------------------------------------------------------------------------
    m12 = MeshBuilder("12_drive_wheels_and_caster")
    for wy in (-30.0, 30.0):
        cx, cy = 0.0, wy
        m12.add_tube_z(cx, cy, 0.0, WHEEL_THICKNESS_MM, 17.0, 22.0, 40)       # Rim band
        m12.add_tube_z(cx, cy, 0.0, 1.4, 17.0, 24.0, 40)                      # Tire flange (hub side)
        m12.add_tube_z(cx, cy, WHEEL_THICKNESS_MM - 1.4, 1.4, 17.0, 24.0, 40) # Tire flange (outer)
        m12.add_tube_z(cx, cy, 0.0, WHEEL_THICKNESS_MM, 2.5, 9.0, 32)         # Hub boss
        for k in range(6):                                                    # 6 spokes
            a = math.pi / 3.0 * k
            r_mid = 13.0
            m12.add_box(cx + r_mid * math.cos(a), cy + r_mid * math.sin(a),
                        WHEEL_THICKNESS_MM / 2.0, 9.0, 3.0, WHEEL_THICKNESS_MM)
        m12.add_tube_z(cx, cy, 0.0, WHEEL_THICKNESS_MM, 2.45, 3.4, 24)        # Horn bore marker
    # Rear caster: downward-opening cup for a 12 mm ball. Printed mouth-up, so in this
    # file the cup cap is at z = 0 and the mounting plate sits above it; the print is
    # flipped over for assembly.
    m12.add_tube_z(0.0, 0.0, 0.0, 10.0, CASTER_CUP_BORE_MM, 10.0, 36)         # Ball cup
    m12.add_box(0.0, 0.0, 12.0, 26.0, 26.0, 4.0)                              # Mounting plate
    m12.add_tube_z(-9.0, 0.0, 10.0, 4.0, 0.5 * M3_CLEARANCE_HOLE_D_MM, 3.0, 18)
    m12.add_tube_z(9.0, 0.0, 10.0, 4.0, 0.5 * M3_CLEARANCE_HOLE_D_MM, 3.0, 18)
    spec12 = PartSpec(
        "12_drive_wheels_and_caster.stl",
        "2x 48 mm Spoked Drive Wheels + 12 mm Ball Caster Cup",
        17.0,
        "Wheels FLAT (axis vertical) and caster cup MOUTH UP; no supports",
        0.16, 4, 25, "None",
        "Rubber band or silicone strip sits in the tyre groove for traction; ball caster takes a 12 mm steel/glass ball",
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
        (spec11, m11),
        (spec12, m12),
    ]

    for spec, builder in all_builders:
        stl_path = out_dir / spec.filename
        builder.write_binary_stl(stl_path)
        _, _, bbox_sz = builder.bounding_box()
        results.append((spec, bbox_sz, len(builder.triangles), builder.mesh_volume_mm3()))

    return results


PLA_DENSITY_G_PER_CM3 = 1.24


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
    root = Path(__file__).resolve().parents[2]

    print("=" * 116)
    print(f"  B.U.D.-E - PHASE 2 GENERATED MESH MANIFEST (Clearance = {args.clearance_mm:.2f} mm)")
    print("  These meshes ADD primitives only: no Boolean cuts, no mating check, no strength check.")
    print("  Read docs/verification_status.md before printing a functional assembly.")
    print("=" * 116)
    print(f"{'STL File':<42} {'Bounding Box (X x Y x Z mm)':<26} {'Tris':<7} "
          f"{'cm3 solid':>9} {'g solid':>8} {'Declared g':>11}")
    print("-" * 116)
    total_g = 0.0
    for spec, sz, tris, vol_mm3 in results:
        total_g += spec.est_filament_g
        bbox_str = f"{sz[0]:5.1f} x {sz[1]:5.1f} x {sz[2]:4.1f}"
        print(f"{spec.filename:<42} {bbox_str:<26} {tris:<7d} "
              f"{vol_mm3 / 1000.0:>9.1f} {vol_mm3 / 1000.0 * PLA_DENSITY_G_PER_CM3:>7.1f} g "
              f"{spec.est_filament_g:>9.1f} g")
    print("-" * 116)
    print(
        f"Parts: {len(results)} files in `{args.out_dir.relative_to(root)}/`   "
        f"|  Declared filament total: {total_g:.0f} g  (~${total_g * 0.018:.2f} at $18/kg)"
    )
    print("`g solid` is the mesh volume x PLA density and IGNORES infill and every overlap")
    print("between added primitives, so it is an UPPER BOUND. Your slicer's estimate rules.")
    print("=" * 116)

    print()
    print("=" * 116)
    print("  DECLARED MATING INTERFACES (source-level agreement only - NOT a fit test)")
    print("=" * 116)
    print(f"{'Interface':<26} {'Part A':<34} {'Parameter':<30} {'Value':>10} {'Verify with'}")
    print("-" * 116)
    for mi in MATING_INTERFACES:
        print(f"{mi.name:<26} {mi.part_a:<34} {mi.parameter:<30} "
              f"{mi.declared_value:>7.2f} {mi.unit:<2} {mi.verify_with}")
    print("=" * 116)
    print("Both halves of each pair are built from these same constants, so the MODELS agree.")
    print("Whether two printed parts fit is decided by a first-article print and a real part.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
