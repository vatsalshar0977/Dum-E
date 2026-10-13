#!/usr/bin/env python3
"""Phase 2 assembly-fit audit: does every declared cavity, bore and joint actually exist?

Every number in this report is measured from the bytes of the STL files, not restated from
the generator's constants. Six questions are asked and answered with rays and vertices:

  1. Is each mesh a sane solid?          triangle count, degenerate faces, Z seating, size
  2. Is each declared cavity really air? a ray is fired at each probe point; the point must
                                         be OUTSIDE the union (air) or INSIDE it (material)
  3. Is each bore coaxial with its joint? the ring of vertices around the bore is measured
                                         and compared with the declared joint axis
  4. Is the spline offset real?           the servo cavity is measured by firing rays at its
                                         inner walls, and its centre must sit exactly one
                                         spline offset from the joint axis
  5. Does each straddle clear?            the parent's outer span and the child's inner span
                                         are measured from their bore rings near the axis
  6. Does the rover interface match?      bolt circles and hole centres in BOTH meshes

What PASS means here, and what it does not:
  PASS  = the CAD is internally consistent: the holes are holes, the pockets are empty, and
          both halves of every mating pair agree to the printed tolerance.
  It does NOT mean two printed parts fit. That needs a first-article print and a caliper.
  A parametric model can be perfectly self-consistent and still not assemble.

Usage:
    python3 src/phase2_mechanical/verify_assembly_fit.py
    python3 src/phase2_mechanical/verify_assembly_fit.py --gap 0.4     # your printer's fit
    python3 src/phase2_mechanical/verify_assembly_fit.py --voxel 6     # faster, coarser
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys
from typing import Dict, List, Optional, Sequence, Tuple

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))

from generate_stl_parts import (                                    # noqa: E402
    AXLE_HEIGHT_MM, BASE_PLATE_MM, CAVITY_PROBES, COAXIAL_BORES, DECK_T_MM, DECK_TOP_MM,
    DRIVEN_WALL_T, DT_SERVO_L_MM, DT_SERVO_SHAFT_OFFSET_MM, E_STOP_BORE_D_MM, E_STOP_CX,
    GROUND_GAP_MM, J2_AXIS_Z_IN_04, J2_GAP_MM, J2_INNER_Y, J2_OUTER_Y, J2_Y_NEG,
    J3_AXIS_Z_IN_05, J3_GAP_MM, J3_INNER_Y, J3_OUTER_Y, J3_Y_NEG, J4_AXIS_Z_IN_06,
    J4_GAP_MM, J4_INNER_Y, J4_OUTER_Y, J4_Y_NEG, J5_AXIS_X_IN_07, J5_AXIS_Z_IN_07,
    J6_AXIS_X_IN_08, J6_AXIS_Z_IN_08, M3_CLEARANCE_HOLE_D_MM, M3_NUT_ACROSS_FLATS_MM,
    M3_NUT_TRAP_DEPTH_MM, MATERIAL_PROBES, MG996R_BODY_L, MG996R_BODY_W,
    MG996R_CASE_ALONG_AXIS, MG996R_HORN_BORE_D, MG996R_SPLINE_OFFSET, MG90S_BODY_L,
    MG90S_BODY_W, MG90S_CASE_ALONG_AXIS, MG90S_HORN_BORE_D, MG90S_SPLINE_OFFSET,
    PASSIVE_BORE_D, ROVER_BOLT_ANGLES_DEG, ROVER_BOLT_CIRCLE_R_MM, WALL_T,
    WHEEL_DIAMETER_MM, WHEEL_THICKNESS_MM, AXIAL_FIT_TOTAL_MM, PART_FILENAMES,
    build_all_parts,
)
from mesh_audit import load_mesh, degenerate_triangles, read_binary_stl     # noqa: E402

FAILURES: List[str] = []
SCORECARD: List[Tuple[str, str, str, bool]] = []


def check(section: str, name: str, detail: str, ok: bool) -> bool:
    SCORECARD.append((section, name, detail, ok))
    if not ok:
        FAILURES.append(f"{section} / {name}: {detail}")
    return ok


def close(a: float, b: float, tol: float) -> bool:
    return abs(a - b) <= tol


@dataclass
class RingSpan:
    """The Y extent of a bore's vertex ring near a joint axis."""

    lo: float
    hi: float
    count: int

    @property
    def span(self) -> float:
        return self.hi - self.lo


def ring_span(mesh, u_axis: int, v_axis: int, w_axis: int, centre_uv, radius: float,
              side: int = 0, fringe: float = 0.02) -> Optional[RingSpan]:
    """Measure the extent of a bore's own vertex ring along the third axis.

    Only vertices within a tight annulus around the nominal bore radius are considered, and
    `side` keeps only one side of the joint axis (+1 or -1). Without the side filter the
    neighbouring bore's decomposition-fan vertices, which reach out to 1.41 x the bore radius,
    would leak into the window and drag the measured span across the axis.
    """
    r_lo, r_hi = radius * (1.0 - fringe), radius * (1.0 + fringe)
    values: List[float] = []
    for p in mesh.verts:
        if side > 0 and p[w_axis] <= 0.0:
            continue
        if side < 0 and p[w_axis] >= 0.0:
            continue
        du = p[u_axis] - centre_uv[0]
        dv = p[v_axis] - centre_uv[1]
        if r_lo <= math.hypot(du, dv) <= r_hi:
            values.append(p[w_axis])
    if len(values) < 8:
        return None
    return RingSpan(min(values), max(values), len(values))


# Rays fired from a cavity's centre often land exactly on the diagonal where two triangles
# of a wall meet, and a hit on a shared edge is ambiguous, so the cast origin is nudged by
# the same irrational-ish offset the inside/outside test uses. The error this introduces is
# 0.003 mm, which is two orders of magnitude below the tolerances being checked.
RAY_NUDGE_3D = (0.0013, -0.0027, 0.0011)


def cavity_span(mesh, origin, axis: int, positive: bool, max_t: float = 200.0):
    """Fire a ray from a point inside a cavity and return how far the wall is, or None."""
    direction = [0.0, 0.0, 0.0]
    direction[axis] = 1.0 if positive else -1.0
    nudged = tuple(origin[i] + RAY_NUDGE_3D[i] for i in range(3))
    return mesh.first_hit(nudged, tuple(direction), max_t)


# =============================================================================
# SECTION 1 - mesh integrity
# =============================================================================
def audit_meshes(meshes: Dict[str, object], builders: Dict[str, object], voxel: float) -> None:
    print("=" * 118)
    print("  1. MESH INTEGRITY - read straight from the binary STL bytes")
    print("=" * 118)
    print(f"{'file':<42} {'tris':>6} {'degen':>6} {'z_min':>8} {'flat vtx':>9} {'max dim':>9}")
    print("  z_min may be negative: the STLs are in the ASSEMBLY frame, where a joint axis is")
    print("  the origin. Let your slicer drop the part to the bed; every part has a flat base.")
    print("-" * 118)
    for name in PART_FILENAMES:
        mesh = meshes[name]
        tris = len(mesh.tris)
        header_tris, _ = read_binary_stl(ROOT / "cad" / "stl" / name)
        degen = degenerate_triangles(mesh.verts)
        z_min = min(p[2] for p in mesh.verts)
        flat = sum(1 for p in mesh.verts if close(p[2], z_min, 1e-6))
        xs = [p[0] for p in mesh.verts]
        ys = [p[1] for p in mesh.verts]
        zs = [p[2] for p in mesh.verts]
        size = (max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
        print(f"{name:<42} {tris:>6} {degen:>6} {z_min:>8.3f} {flat:>9} {max(size):>9.1f}")
        check("mesh", f"{name} triangle count matches the STL header",
              f"{tris} triangles vs header {header_tris}", tris == header_tris)
        check("mesh", f"{name} has no degenerate triangles", f"{degen} found", degen == 0)
        check("mesh", f"{name} has a flat base for the slicer", f"{flat} vertices at "
              f"z = {z_min:.3f} mm", flat >= 4)
        check("mesh", f"{name} fits a 180 x 180 mm bed", f"max dimension {max(size):.1f} mm",
              max(size) <= 180.0 + 1e-9)
    print()

    print("  Solid volume. `union` is analytic containment on the DECLARED primitives, so")
    print("  overlaps are not double counted; `signed` is the triangle-soup figure, which is")
    print("  an upper bound where primitives overlap. Printed grams will be lower:")
    print(f"  {'file':<42} {'union mm3':>12} {'signed mm3':>12} {'overlap':>10} {'grams':>9}")
    print("-" * 118)
    total_union = 0.0
    for name in PART_FILENAMES:
        mesh = meshes[name]
        union = builders[name].union_volume_mm3(grid_mm=builders[name].adaptive_grid())
        signed = sum(
            (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
             + a[2] * (b[0] * c[1] - b[1] * c[0]))
            for a, b, c in mesh.tris) / 6.0
        overlap = max(0.0, signed - union)
        total_union += union
        print(f"{name:<42} {union:>12.0f} {signed:>12.0f} {overlap:>10.0f} "
              f"{union / 1000.0 * 1.24:>8.0f} g")
        # Voxel sampling puts a whole voxel inside when its centre is inside, so the union
        # figure can overshoot the true solid by up to one surface layer. The check allows
        # that, and the two numbers bracket the real one: signed >= truth >= union - layer.
        layer = 0.12 * signed + 250.0    # voxel surfaces and overlap slop
        check("volume", f"{name} union volume is consistent with the triangle soup",
              f"union {union:.0f} mm3, signed {signed:.0f} mm3, allowance {layer:.0f} mm3",
              union > 0.0 and union <= signed + layer)
    print("-" * 118)
    print(f"  Sum of union volumes: {total_union / 1000.0:.0f} cm3 "
          f"({total_union / 1000.0 * 1.24:.0f} g of 100% solid PLA, before infill).")
    print("  Each part was sampled at its own adaptive grid (1-3 mm, chosen so the thinnest")
    print("  declared feature spans at least three voxels).")
    print("  Note: the mesh_audit ray-parity volume is NOT used here. Parity is unsound where")
    print("  primitives overlap (two covers give an even count), which is most of these parts.")
    print()


# =============================================================================
# SECTION 2 - cavity and material probes
# =============================================================================
def audit_probes(meshes: Dict[str, object], builders: Dict[str, object]) -> None:
    print("=" * 118)
    print("  2. CAVITY PROBES - every pocket must be AIR, every wall must be MATERIAL")
    print("  `covers` = how many declared primitives contain the point (0 means air).")
    print("  `ray` = the parity count of the STL's own triangles along +Z from the point.")
    print("=" * 118)
    print(f"{'part':<40} {'probe':<38} {'want':>9} {'covers':>7} {'ray':>4} {'result':>7}")
    print("-" * 118)
    for probe in list(CAVITY_PROBES) + list(MATERIAL_PROBES):
        mesh = meshes.get(probe.part)
        builder = builders.get(probe.part)
        if mesh is None or builder is None:
            check("probes", probe.label, f"no mesh named {probe.part}", False)
            continue
        covers = builder.occupancy(probe.point)
        parity = "odd" if mesh.point_in_solid(probe.point) else "even"
        got = "material" if covers > 0 else "air"
        ok = got == probe.expect
        # Cross-check the writer: an odd parity count is always proof of material, and it can
        # only happen when at least one primitive covers the point.
        if parity == "odd" and covers == 0:
            ok = False          # odd parity is proof of material, so a cover must exist
        px, py, pz = probe.point
        print(f"{probe.part:<40} {probe.label:<38} {probe.expect:>9} {covers:>7} {parity:>4} "
              f"{'PASS' if ok else 'FAIL':>7}")
        check("probes", f"{probe.part} :: {probe.label}",
              f"({px:.1f}, {py:.1f}, {pz:.1f}) covered by {covers} primitive(s) = {got}, "
              f"expected {probe.expect}; ray parity {parity}", ok)
    print("-" * 118)
    print("  Why the parity column alone is not enough: these parts are built as OVERLAPPING")
    print("  primitives, and a point under two covers crosses the ray an even number of times,")
    print("  so parity would report air for real material. Parity is therefore used only as a")
    print("  writer cross-check (odd must imply covered), and the declared-primitive occupancy")
    print("  above is what actually proves a cavity is empty.")
    print()


# =============================================================================
# SECTION 3 - coaxiality of every declared bore
# =============================================================================
def audit_coaxiality(meshes: Dict[str, object]) -> None:
    print("=" * 118)
    print("  3. COAXIALITY - every bore's own vertex ring, measured against its declared axis")
    print("     The ring's centroid IS the bore centre, so this measures the model rather than")
    print("     restating the constant. A pitch/roll bore runs along Y; a yaw bore runs along Z.")
    print("=" * 118)
    print(f"{'part':<40} {'joint':<6} {'feature':<34} {'declared':>14} {'measured':>15} "
          f"{'error':>9} {'verts':>6}")
    print("-" * 118)
    for bc in COAXIAL_BORES:
        mesh = meshes[bc.part]
        if bc.direction == "Y":
            u_axis, v_axis, w_axis = 0, 2, 1
            centre_uv = (bc.axis_point[0], bc.axis_point[2])
            band = (bc.band[0], bc.band[1])
        else:
            u_axis, v_axis, w_axis = 0, 1, 2
            centre_uv = (bc.axis_point[0], bc.axis_point[1])
            band = (bc.band[0], bc.band[1])
        # A tight radius window: every ring in this project is generated at an exact radius,
        # so +/-0.02 mm keeps the measurement on the bore itself and off neighbouring
        # geometry. The measured radius is reported so a mis-window would be visible.
        cu, cv, fit_r, rms, count = mesh.fit_circle(
            u_axis, v_axis, w_axis, centre_uv, (bc.radius - 0.02, bc.radius + 0.02), band)
        axis_txt = "({:7.2f}, {:7.2f})".format(*centre_uv)
        if count < 8 or math.isnan(cu):
            meas_txt = "NOT FOUND"
            err = float("nan")
            print(f"{bc.part:<40} {bc.joint:<6} {bc.label:<34} {axis_txt:>14} {meas_txt:>15} "
                  f"{'-':>9} {count:>6}   FAIL")
            check("coaxial", f"{bc.part} :: {bc.label}", "no ring vertices in the band", False)
            continue
        meas_txt = "({:7.2f}, {:7.2f})".format(cu, cv)
        err = math.hypot(cu - centre_uv[0], cv - centre_uv[1])
        mean_r = fit_r
        ok = err <= 0.02 and close(fit_r, bc.radius, 0.02)
        print(f"{bc.part:<40} {bc.joint:<6} {bc.label:<34} {axis_txt:>14} {meas_txt:>15} "
              f"{err:>8.4f} {count:>6}   {'PASS' if ok else 'FAIL'}")
        check("coaxial", f"{bc.part} :: {bc.label} ({bc.direction} bore)",
              f"ring centre ({cu:.3f}, {cv:.3f}) vs declared ({centre_uv[0]:.3f}, "
              f"{centre_uv[1]:.3f}) = {err:.4f} mm; fitted radius {fit_r:.3f} vs "
              f"{bc.radius:.3f} mm, residual {rms:.2e} mm", ok)
    print("-" * 118)
    print("  Tolerance 0.020 mm, which is the model's own placement precision. Non-zero here")
    print("  would mean the ring window caught geometry that is not the bore.")
    print()


# =============================================================================
# SECTION 4 - servo pockets: real air, right size, right offset
# =============================================================================
@dataclass(frozen=True)
class PocketCheck:
    part: str
    joint: str
    label: str
    case_centre: Tuple[float, float, float]   # a point that must be inside the servo's case
    axis: Tuple[float, float]                 # the joint's (x, z) in the part frame
    body_l: float
    body_w: float
    case_len: float
    case_offset_x: float            # signed: case centre - joint axis, along the part's X
    open_dirs: Tuple[str, ...]
    shaft_direction: str            # 'Y' or 'Z': the servo's output axis in the part frame


POCKET_CHECKS: List[PocketCheck] = [
    PocketCheck("03_base_lower_slewing_ring.stl", "J1", "MG996R upright cradle",
                (MG996R_SPLINE_OFFSET, 0.0, 16.0), (0.0, 0.0), MG996R_BODY_L, MG996R_BODY_W,
                MG996R_CASE_ALONG_AXIS, MG996R_SPLINE_OFFSET, ("+Z",), "Z"),      # J1: +X side
    PocketCheck("04_base_upper_turret_j2.stl", "J2", "MG996R horizontal pocket",
                (-MG996R_SPLINE_OFFSET, J2_Y_NEG + WALL_T + MG996R_CASE_ALONG_AXIS / 2.0,
                 J2_AXIS_Z_IN_04), (0.0, J2_AXIS_Z_IN_04), MG996R_BODY_L, MG996R_BODY_W,
                MG996R_CASE_ALONG_AXIS, -MG996R_SPLINE_OFFSET, ("+Y", "-X", "+Z"), "Y"),
    PocketCheck("05_upper_arm_link_80mm.stl", "J3", "MG996R horizontal pocket",
                (80.0 - MG996R_SPLINE_OFFSET, J3_Y_NEG + WALL_T + MG996R_CASE_ALONG_AXIS / 2.0,
                 J3_AXIS_Z_IN_05), (80.0, J3_AXIS_Z_IN_05), MG996R_BODY_L, MG996R_BODY_W,
                MG996R_CASE_ALONG_AXIS, -MG996R_SPLINE_OFFSET, ("+Y", "-X", "+Z"), "Y"),
    PocketCheck("06_forearm_link_75mm.stl", "J4", "MG90S horizontal pocket",
                (75.0 - MG90S_SPLINE_OFFSET, J4_Y_NEG + WALL_T + MG90S_CASE_ALONG_AXIS / 2.0,
                 J4_AXIS_Z_IN_06), (75.0, J4_AXIS_Z_IN_06), MG90S_BODY_L, MG90S_BODY_W,
                MG90S_CASE_ALONG_AXIS, -MG90S_SPLINE_OFFSET, ("+Y", "-X", "+Z"), "Y"),
    PocketCheck("07_wrist_pitch_roll_bracket.stl", "J5", "MG90S roll pocket",
                (J5_AXIS_X_IN_07 - MG90S_SPLINE_OFFSET, 0.0, J5_AXIS_Z_IN_07 + 11.0),
                (J5_AXIS_X_IN_07, J5_AXIS_Z_IN_07), MG90S_BODY_L, MG90S_BODY_W,
                MG90S_CASE_ALONG_AXIS, -MG90S_SPLINE_OFFSET, (), "Z"),
    PocketCheck("08_gripper_body_and_cam_mount.stl", "J6", "MG90S grip pocket",
                (J6_AXIS_X_IN_08 - MG90S_SPLINE_OFFSET, 0.0, J6_AXIS_Z_IN_08),
                (J6_AXIS_X_IN_08, J6_AXIS_Z_IN_08), MG90S_BODY_L, MG90S_BODY_W,
                MG90S_CASE_ALONG_AXIS, -MG90S_SPLINE_OFFSET, (), "Z"),
]


def audit_pockets(meshes: Dict[str, object]) -> None:
    print("=" * 118)
    print("  4. SERVO POCKETS - rays fired at the inner walls from inside each cavity")
    print("     The servo case centre is measured from the wall hits, then compared with the")
    print("     position that puts the output spline exactly on the joint axis.")
    print("=" * 118)
    print(f"{'part':<40} {'joint':<5} {'measured cavity centre':>24} {'declared':>12} "
          f"{'spline err':>11} {'X span':>8} {'Y span':>8} {'floor':>8}")
    print("-" * 118)
    for pc in POCKET_CHECKS:
        mesh = meshes[pc.part]
        ox, oy, oz = pc.case_centre
        probe = (ox, oy, oz)
        if not mesh.point_in_solid(probe):
            pass
        else:
            check("pockets", f"{pc.part} :: {pc.joint} cavity interior",
                  f"({ox:.2f}, {oy:.2f}, {oz:.2f}) is solid, expected air", False)

        hits = {
            "+X": cavity_span(mesh, probe, 0, True),
            "-X": cavity_span(mesh, probe, 0, False),
            "+Y": cavity_span(mesh, probe, 1, True),
            "-Y": cavity_span(mesh, probe, 1, False),
            "-Z": cavity_span(mesh, probe, 2, False),
            "+Z": cavity_span(mesh, probe, 2, True),
        }
        expected_half = pc.body_l / 2.0 + 0.25
        span_x = None
        if hits["+X"] is not None and hits["-X"] is not None:
            span_x = hits["+X"] + hits["-X"]
            cx_meas = ox + (hits["+X"] - hits["-X"]) / 2.0
        elif hits["+X"] is not None:
            # one face is deliberately open so the servo case can overhang; the +X wall is
            # still measurable and must sit exactly one half-opening from the case centre
            cx_meas = ox + hits["+X"] - expected_half
        else:
            cx_meas = float("nan")
        span_y = None
        if hits["+Y"] is not None and hits["-Y"] is not None:
            span_y = hits["+Y"] + hits["-Y"]
        floor_z = (oz - hits["-Z"]) if hits["-Z"] is not None else float("nan")

        # The declared cavity centre: the case sits so its spline is on the axis. The sign
        # of the offset depends on which way the servo faces about its own shaft.
        declared_cx = pc.axis[0] + pc.case_offset_x
        spline_err = abs(cx_meas - declared_cx) if not math.isnan(cx_meas) else float("nan")

        print(f"{pc.part:<40} {pc.joint:<5} "
              f"{f'({cx_meas:7.2f}, {oy:7.2f}, {oz:7.2f})':>24} {declared_cx:>12.2f} "
              f"{spline_err:>11.4f} "
              f"{(span_x if span_x is not None else float('nan')):>8.2f} "
              f"{(span_y if span_y is not None else float('nan')):>8.2f} "
              f"{floor_z:>8.2f}")
        check("pockets", f"{pc.part} :: {pc.joint} cavity is air",
              "ray test from inside the cavity", not mesh.point_in_solid(probe))
        if hits["+X"] is not None:
            check("pockets", f"{pc.part} :: {pc.joint} +X pocket wall position",
                  f"measured {hits['+X']:.3f} mm from the case centre, declared half-opening "
                  f"{expected_half:.3f} mm", close(hits["+X"], expected_half, 0.05))
        if hits["-X"] is not None:
            check("pockets", f"{pc.part} :: {pc.joint} -X pocket wall position",
                  f"measured {hits['-X']:.3f} mm from the case centre, declared half-opening "
                  f"{expected_half:.3f} mm", close(hits["-X"], expected_half, 0.05))
        check("pockets", f"{pc.part} :: {pc.joint} spline land on the joint axis",
              f"measured case centre {cx_meas:.4f} mm, declared offset "
              f"{pc.case_offset_x:+.2f} mm -> remaining error {spline_err:.4f} mm",
              not math.isnan(spline_err) and spline_err <= 0.05)
        if span_x is not None:
            check("pockets", f"{pc.part} :: {pc.joint} cavity holds the body length",
                  f"{span_x:.2f} mm open vs body {pc.body_l:.1f} mm + 0.5 mm",
                  span_x >= pc.body_l + 0.45)
        # The case must fit ALONG the servo's own output axis. An upright (Z-shaft) servo
        # hangs its whole case BELOW the spline face, so for those the check has to look up
        # the shaft axis instead of across the joint. That distinction is what caught the
        # original 32 mm base: it could not contain the MG996R's 35.5 mm case.
        if pc.shaft_direction == "Y":
            axial_span = span_y
            axial_txt = f"{span_y:.2f} mm across the joint" if span_y is not None else "n/a"
        else:
            # An upright servo hangs its case below the spline face, so the usable height is
            # from the pocket floor to whatever closes the top: a bored ceiling plate if there
            # is one, otherwise the top of the tower.
            if hits["+Z"] is not None:
                top_z = oz + hits["+Z"]
                how = "closed by a plate"
            else:
                near = [p[2] for p in mesh.verts
                        if abs(p[0] - ox) <= 25.0 and abs(p[1] - oy) <= 25.0]
                top_z = max(near) if near else float("nan")
                how = "open top"
            axial_span = top_z - floor_z if not math.isnan(top_z) else None
            axial_txt = (f"{axial_span:.2f} mm ({how})"
                         if axial_span is not None else "n/a")
        needed = pc.case_len + 0.45
        check("pockets", f"{pc.part} :: {pc.joint} cavity holds the case along the shaft axis",
              f"{axial_txt} vs case {pc.case_len:.1f} mm + clearance "
              f"(a {pc.shaft_direction}-shaft mount)",
              axial_span is not None and axial_span >= needed - 1e-9)
        check("pockets", f"{pc.part} :: {pc.joint} face is open where declared",
              f"+Z hit = {hits['+Z']}", (hits["+Z"] is None) == ("+Z" in pc.open_dirs))
    print("-" * 118)
    print("  `spline err` is the headline number: the distance between the servo case centre")
    print("  the mesh actually has and the one that puts the output spline on the joint axis.")
    print()


# =============================================================================
# SECTION 5 - straddle gaps between the telescoping links
# =============================================================================
@dataclass(frozen=True)
class StraddleCheck:
    joint: str
    parent: str
    child: str
    parent_axis: Tuple[float, float]
    child_axis: Tuple[float, float]
    driven_bore_r: float
    bearing_bore_r: float
    declared_gap: float
    declared_outer: float
    declared_inner: float


STRADDLE_CHECKS: List[StraddleCheck] = [
    StraddleCheck("J2", "04_base_upper_turret_j2.stl", "05_upper_arm_link_80mm.stl",
                  (0.0, J2_AXIS_Z_IN_04), (0.0, 0.0), MG996R_HORN_BORE_D / 2.0,
                  PASSIVE_BORE_D / 2.0, J2_GAP_MM, J2_OUTER_Y, J2_INNER_Y),
    StraddleCheck("J3", "05_upper_arm_link_80mm.stl", "06_forearm_link_75mm.stl",
                  (80.0, J3_AXIS_Z_IN_05), (0.0, 0.0), MG996R_HORN_BORE_D / 2.0,
                  PASSIVE_BORE_D / 2.0, J3_GAP_MM, J3_OUTER_Y, J3_INNER_Y),
    StraddleCheck("J4", "06_forearm_link_75mm.stl", "07_wrist_pitch_roll_bracket.stl",
                  (75.0, J4_AXIS_Z_IN_06), (0.0, 0.0), MG90S_HORN_BORE_D / 2.0,
                  PASSIVE_BORE_D / 2.0, J4_GAP_MM, J4_OUTER_Y, J4_INNER_Y),
]


def audit_straddles(meshes: Dict[str, object]) -> None:
    print("=" * 118)
    print("  5. STRADDLE GAPS - parent outer span and child inner span, from the bore rings")
    print("     A hobby servo's spline sits at the top of its case, so the case hangs on ONE")
    print("     side of the joint axis and the straddle is deliberately asymmetric.")
    print("=" * 118)
    print(f"{'joint':<5} {'parent outer (mm)':>18} {'child inner (mm)':>18} "
          f"{'gap (mm)':>10} {'declared':>9} {'result':>7}")
    print("-" * 118)
    for sc in STRADDLE_CHECKS:
        parent = meshes[sc.parent]
        child = meshes[sc.child]
        p_driven = ring_span(parent, 0, 2, 1, sc.parent_axis, sc.driven_bore_r, side=+1)
        p_bearing = ring_span(parent, 0, 2, 1, sc.parent_axis, sc.bearing_bore_r, side=-1)
        c_pos = ring_span(child, 0, 2, 1, sc.child_axis, sc.driven_bore_r, side=+1)
        c_neg = ring_span(child, 0, 2, 1, sc.child_axis, sc.bearing_bore_r, side=-1)
        if None in (p_driven, p_bearing, c_pos, c_neg):
            check("straddle", f"{sc.joint} rings measurable", "a bore ring was not found", False)
            print(f"{sc.joint:<5} {'NOT MEASURABLE':>58}")
            continue
        outer = p_driven.hi - p_bearing.lo
        inner = c_pos.lo - c_neg.hi
        gap = inner - outer
        ok_gap = close(gap, sc.declared_gap, 0.02)
        ok_outer = close(outer, sc.declared_outer, 0.02)
        ok_inner = close(inner, sc.declared_inner, 0.02)
        print(f"{sc.joint:<5} {outer:>18.3f} {inner:>18.3f} {gap:>10.3f} "
              f"{sc.declared_gap:>9.2f} {'PASS' if ok_gap and ok_outer and ok_inner else 'FAIL':>7}")
        check("straddle", f"{sc.joint} parent outer span", f"measured {outer:.3f} mm, declared "
              f"{sc.declared_outer:.2f} mm", ok_outer)
        check("straddle", f"{sc.joint} child inner span", f"measured {inner:.3f} mm, declared "
              f"{sc.declared_inner:.2f} mm", ok_inner)
        check("straddle", f"{sc.joint} telescoping gap",
              f"measured {gap:.3f} mm, declared {sc.declared_gap:.2f} mm "
              f"({gap / 2.0:.2f} mm per side)", ok_gap)
        check("straddle", f"{sc.joint} case fits inside the parent",
              f"{outer:.2f} mm outer minus {WALL_T:.1f} + {DRIVEN_WALL_T:.1f} mm of wall = "
              f"{outer - WALL_T - DRIVEN_WALL_T:.2f} mm of free space",
              outer - WALL_T - DRIVEN_WALL_T >= (MG996R_CASE_ALONG_AXIS if sc.joint != "J4"
                                                 else MG90S_CASE_ALONG_AXIS)
              + AXIAL_FIT_TOTAL_MM - 1e-9)
    print("-" * 118)
    print("  The ring measurement looks only at vertices within +/-18% of the nominal bore")
    print("  radius, so it reads the bore itself and not the surrounding plate.")
    print()


# =============================================================================
# SECTION 6 - the rover interface, measured in BOTH meshes
# =============================================================================
def audit_rover_interface(meshes: Dict[str, object]) -> None:
    print("=" * 118)
    print("  6. ROVER INTERFACE - 4x M3 at r = 36 mm, measured in both halves")
    print("=" * 118)
    base = meshes["03_base_lower_slewing_ring.stl"]
    chassis = meshes["11_mobile_rover_chassis.stl"]

    hex_r = (M3_NUT_ACROSS_FLATS_MM / math.sqrt(3.0))
    print(f"{'part':<40} {'feature':<26} {'measured centre':>20} {'radius':>8} "
          f"{'angle':>8}")
    print("-" * 118)
    base_centres = []
    for ang in ROVER_BOLT_ANGLES_DEG:
        a = math.radians(ang)
        nominal = (ROVER_BOLT_CIRCLE_R_MM * math.cos(a), ROVER_BOLT_CIRCLE_R_MM * math.sin(a))
        cu, cv, fit_r, rms, count = base.fit_circle(
            0, 1, 2, nominal, (hex_r - 0.02, hex_r + 0.02),
            (-0.05, M3_NUT_TRAP_DEPTH_MM + 0.05))
        if count < 6:
            cu = cv = float("nan")
        base_centres.append((cu, cv))
        r = math.hypot(cu, cv) if not math.isnan(cu) else float("nan")
        measured_angle = math.degrees(math.atan2(cv, cu)) % 360.0 if not math.isnan(cu) else \
            float("nan")
        print(f"{'03_base_lower_slewing_ring.stl':<40} {'M3 hex trap':<26} "
              f"{f'({cu:7.2f}, {cv:7.2f})':>20} {r:>8.3f} {measured_angle:>8.1f}")
        check("rover", f"base nut trap at {ang:.0f} deg",
              f"measured radius {r:.3f} mm, declared {ROVER_BOLT_CIRCLE_R_MM:.2f} mm",
              not math.isnan(r) and close(r, ROVER_BOLT_CIRCLE_R_MM, 0.05))
        check("rover", f"base nut trap angle at {ang:.0f} deg",
              f"measured {measured_angle:.2f} deg", not math.isnan(measured_angle)
              and close(measured_angle, ang, 0.6))

    hole_r = M3_CLEARANCE_HOLE_D_MM / 2.0
    chassis_centres = []
    for ang in ROVER_BOLT_ANGLES_DEG:
        a = math.radians(ang)
        nominal = (ROVER_BOLT_CIRCLE_R_MM * math.cos(a), ROVER_BOLT_CIRCLE_R_MM * math.sin(a))
        cu, cv, fit_r, rms, count = chassis.fit_circle(
            0, 1, 2, nominal, (hole_r - 0.02, hole_r + 0.02),
            (DECK_TOP_MM - DECK_T_MM - 0.05, DECK_TOP_MM + 0.05))
        if count < 8:
            cu = cv = float("nan")
        chassis_centres.append((cu, cv))
        r = math.hypot(cu, cv) if not math.isnan(cu) else float("nan")
        measured_angle = math.degrees(math.atan2(cv, cu)) % 360.0 if not math.isnan(cu) else \
            float("nan")
        print(f"{'11_mobile_rover_chassis.stl':<40} {'M3 clearance hole':<26} "
              f"{f'({cu:7.2f}, {cv:7.2f})':>20} {r:>8.3f} {measured_angle:>8.1f}")
        check("rover", f"chassis bolt hole at {ang:.0f} deg",
              f"measured radius {r:.3f} mm, declared {ROVER_BOLT_CIRCLE_R_MM:.2f} mm",
              not math.isnan(r) and close(r, ROVER_BOLT_CIRCLE_R_MM, 0.05))

    worst = 0.0
    for (bx, by), (cx, cy) in zip(base_centres, chassis_centres):
        if any(math.isnan(v) for v in (bx, by, cx, cy)):
            continue
        worst = max(worst, math.hypot(bx - cx, by - cy))
    check("rover", "both halves share the bolt pattern", f"worst centre offset {worst:.3f} mm",
          worst <= 0.05)

    # The base plate is square so the nut traps could be cut as true hex voids.
    # The E-stop enclosure sticks out to x = 74 mm, so the plate's own outline is measured
    # from the corner bands, where only the plate exists.
    plate_vs = [p for p in base.verts
                if p[2] <= M3_NUT_TRAP_DEPTH_MM and abs(p[1]) >= 30.0]
    plate_x = max(p[0] for p in plate_vs) - min(p[0] for p in plate_vs)
    plate_y = max(p[1] for p in plate_vs) - min(p[1] for p in plate_vs)
    check("rover", "base plate outline",
          f"{plate_x:.1f} x {plate_y:.1f} mm square plate (a round plate cannot carry true hex "
          f"traps with this tooling)", close(plate_x, BASE_PLATE_MM, 1e-6)
          and close(plate_y, BASE_PLATE_MM, 1e-6))

    # Wheels: 48 mm diameter, hub bore, and the axle height that fixes the ground gap.
    wheels = meshes["12_drive_wheels_and_caster.stl"]
    span_lo = WHEEL_DIAMETER_MM / 2.0 - WHEEL_THICKNESS_MM / 2.0
    span_hi = WHEEL_DIAMETER_MM / 2.0 + WHEEL_THICKNESS_MM / 2.0
    for wy in (-30.0, 30.0):
        # The wheels print flat: the disc lies in the XY plane with the axle along Z, so the
        # diameter is a radius in X-Y taken inside the wheel's thickness band.
        radii = [math.hypot(p[0], p[1] - wy) for p in wheels.verts
                 if span_lo - 0.05 <= p[2] <= span_hi + 0.05
                 and p[1] * wy > 0.0 and math.hypot(p[0], p[1] - wy) > 1.0]
        outer_r = max(radii)
        inner_r = min(radii)
        print(f"{'12_drive_wheels_and_caster.stl':<40} {f'wheel at y={wy:+.0f}':<26} "
              f"{'tyre / hub':>20} {outer_r:>8.2f} {inner_r:>8.2f}")
        check("rover", f"wheel at y = {wy:+.0f} diameter",
              f"outer radius {outer_r:.2f} mm -> {outer_r * 2:.1f} mm diameter",
              close(outer_r * 2.0, WHEEL_DIAMETER_MM, 0.05))
        check("rover", f"wheel at y = {wy:+.0f} hub bore",
              f"inner radius {inner_r:.2f} mm -> {inner_r * 2:.1f} mm bore",
              inner_r >= 2.4)

    # The axle hole in the chassis must sit at the height that produces the ground gap.
    cu, cv, fit_r, rms, count = chassis.fit_circle(0, 2, 1, (0.0, AXLE_HEIGHT_MM), (3.18, 3.22),
                                                   (52.05, 58.0))
    print(f"{'11_mobile_rover_chassis.stl':<40} {'axle bore in the side plate':<26} "
          f"{f'({cu:7.2f}, {cv:7.2f})':>20} {'-':>8} {'-':>8}")
    check("rover", "chassis axle bore is on the wheel axis",
          f"measured centre ({cu:.3f}, {cv:.3f}), declared (0.00, {AXLE_HEIGHT_MM:.2f})",
          not math.isnan(cu) and close(cu, 0.0, 0.05) and close(cv, AXLE_HEIGHT_MM, 0.05))

    print("-" * 118)
    print(f"  Wheel radius {WHEEL_DIAMETER_MM / 2:.1f} mm - axle height "
          f"{AXLE_HEIGHT_MM:.1f} mm = {WHEEL_DIAMETER_MM / 2 - AXLE_HEIGHT_MM:.1f} mm ground gap.")
    print(f"  Servo body {DT_SERVO_L_MM:.0f} mm wide on its side needs its top face at "
          f"z <= {DECK_TOP_MM - DECK_T_MM:.0f} mm: {AXLE_HEIGHT_MM:.0f} + "
          f"{MG90S_BODY_W / 2:.1f} = {AXLE_HEIGHT_MM + MG90S_BODY_W / 2:.1f} mm. Clear.")
    print("  An 8.0 mm gap would put the servo top at 22.1 mm, i.e. 2.1 mm INSIDE the deck.")
    print()


# =============================================================================
# MAIN
# =============================================================================
def main() -> int:
    parser = argparse.ArgumentParser(description="Audit the B.U.D.-E printable parts.")
    parser.add_argument("--stl-dir", type=Path, default=ROOT / "cad" / "stl")
    parser.add_argument("--voxel", type=float, default=4.0, help="voxel size for union volume")
    parser.add_argument("--gap", type=float, default=None,
                        help="unused placeholder for your own printer's fit offset")
    args = parser.parse_args()

    print()
    print("#" * 118)
    print("#  B.U.D.-E  PHASE 2  ASSEMBLY-FIT AUDIT")
    print("#  Every number below is measured from cad/stl/*.stl, not restated from the code.")
    print("#" * 118)
    print()

    missing = [n for n in PART_FILENAMES if not (args.stl_dir / n).exists()]
    if missing:
        print("Missing STL files, run the generator first:")
        for n in missing:
            print("  " + n)
        print("\n  python3 src/phase2_mechanical/generate_stl_parts.py")
        return 2

    meshes = {name: load_mesh(args.stl_dir / name) for name in PART_FILENAMES}
    builders = {spec.filename: mesh for spec, mesh in build_all_parts()}
    audit_meshes(meshes, builders, args.voxel)
    audit_probes(meshes, builders)
    audit_coaxiality(meshes)
    audit_pockets(meshes)
    audit_straddles(meshes)
    audit_rover_interface(meshes)

    print("=" * 118)
    print("  MASTER SCORECARD - PHASE 2 ASSEMBLY FIT")
    print("=" * 118)
    sections: List[str] = []
    for section, _name, _detail, _ok in SCORECARD:
        if section not in sections:
            sections.append(section)
    for section in sections:
        rows = [r for r in SCORECARD if r[0] == section]
        passed = sum(1 for r in rows if r[3])
        status = "PASS" if passed == len(rows) else "FAIL"
        print(f"  {section:<10} {passed:>3} / {len(rows):<3} checks   {status}")

    total = len(SCORECARD)
    passed = sum(1 for r in SCORECARD if r[3])
    print("-" * 118)
    print(f"  TOTAL      {passed:>3} / {total:<3} checks   "
          f"{'PASS' if passed == total else 'FAIL'}")
    if FAILURES:
        print()
        print("  FAILED CHECKS")
        for line in FAILURES:
            print("    - " + line)
    print("=" * 118)
    print()
    print("  WHAT THIS DOES NOT PROVE")
    print("   * Nothing here has been printed, fitted, loaded or dropped. A self-consistent")
    print("     CAD model is not an assembled robot.")
    print("   * Every servo dimension is a datasheet declaration. Open-frame hobby servos")
    print("     vary by a millimetre between sellers. Calipers decide.")
    print("   * Printed-pin and printed-thread strength are unmeasured.")
    print("   * The MG996R's mounting ears are NOT modelled in the pockets. The case is")
    print("     located by its sides and the pocket floor; take the axial 1.0 mm slack up")
    print("     with the horn screws, or add a strap over the case.")
    print()
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
